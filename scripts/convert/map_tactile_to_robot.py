# -*- coding: utf-8 -*-
import os
import cv2
import csv
import h5py
import numpy as np
from tqdm import tqdm
import argparse

class PolicyDataFormatter:
    def __init__(
        self,
        image_height: int = 32,
        image_width: int = 32,
        context_length: int = 10,
        horizon_length: int = 10,
        max_force_capacity: float = 1200.0
    ):
        self.image_height = image_height
        self.image_width = image_width
        self.context_length = context_length
        self.horizon_length = horizon_length
        self.max_force_capacity = max_force_capacity
        self.path_file = []

    def create_tactile_image(self, static_tactile_frame: np.ndarray, baseline: np.ndarray = None, keep_baseline: bool = False) -> np.ndarray:
        """
        Converts tactile sensor array into a normalized 2D image matrix.
        Shape input: (NUM_FINGERS, 7, 4) -> Output: (H, W, 1) or (H, W, 3)
        """
        frame = static_tactile_frame.astype(np.float32)
        if baseline is not None and keep_baseline:
            # keep baseline means h5 was saved as raw data, subtract baseline
            frame = frame - baseline.astype(np.float32)

        normalized = np.clip(frame / self.max_force_capacity, 0.0, 1.0)

        # Combine 2 fingers
        num_fingers = normalized.shape[0]
        f0 = normalized[0]
        f1 = normalized[1] if num_fingers > 1 else np.zeros_like(f0)
        combined_grid = np.hstack([f0, f1])

        # Convert to 1-channel uint8 (0-255) for image representation
        uint8_grid = (combined_grid * 255.0).astype(np.uint8)

        # Upsample using INTER_CUBIC to smooth low-res 7x8 grid into target image size (32x32)
        reshaped_image = cv2.resize(
            uint8_grid,
            dsize=(self.image_width, self.image_height),
            interpolation=cv2.INTER_CUBIC
        )

        # Add channel (3rd) dimension -> (32, 32, 1)
        return np.expand_dims(reshaped_image, axis=-1)

    def load_file_data(self, robot_h5_path: str, tactile_h5_path: str):
        """
        Loads robot states and maps to stacked, high frequency tactile data.
        """
        with h5py.File(robot_h5_path, "r") as r_file:
            if "robot_state" in r_file:
                robot_states = r_file["robot_state"][:]
            else:
                # Droid structure
                robot_states = r_file["observation/robot_state"][:] if "obs_dict/robot_state" in r_file else r_file["state"][:]

            if "timestamp/robot_state" in r_file:
                robot_ts = r_file["timestamp/robot_state"][:]
            else:
                robot_ts = r_file["timestamp"][:]

        with h5py.File(tactile_h5_path, "r") as t_file:
            static_tactile = t_file["static_tactile"][:]
            tactile_ts = t_file["timestamp"][:]
            baseline = t_file["baseline"][:] if "baseline" in t_file else None
            keep_baseline = t_file.attrs["keep_baseline"]

        aligned_tactile_images = []

        # Align & convert tactile data
        num_robot_steps = len(robot_ts)
        for step_idx in range(num_robot_steps):
            t_end = robot_ts[step_idx]
            if step_idx == 0:
                dt = robot_ts[1] - robot_ts[0] if num_robot_steps > 1 else 66  # control rate is 15hz
                t_start = t_end - dt
            else:
                t_start = robot_ts[step_idx - 1]

            # Find all tactile readings inside interval [t_start, t_end]
            valid_indices = np.where((tactile_ts >= t_start) & (tactile_ts <= t_end))[0]

            if len(valid_indices) == 0:
                nearest_idx = np.argmin(np.abs(tactile_ts - t_end))
                valid_indices = np.array([nearest_idx])

            window_images = [
                self.create_tactile_image(static_tactile[idx], baseline, keep_baseline)
                for idx in valid_indices
            ]

            # Stack frames and take average per step, any other options of how to handle this?
            step_image = np.mean(np.stack(window_images, axis=0), axis=0).astype(np.uint8)
            aligned_tactile_images.append(step_image)

        return np.array(aligned_tactile_images), np.array(robot_states), robot_ts

    def process_and_save(
        self,
        robot_h5_path: str,
        tactile_h5_path: str,
        save_dir: str,
        experiment_number: int = 0
    ):
        os.makedirs(save_dir, exist_ok=True)
        self.path_file = []
        index_to_save = 0

        tactile_images, robot_data, time_steps = self.load_file_data(robot_h5_path, tactile_h5_path)

        # Save images
        tactile_image_names = []
        for t in range(len(tactile_images)):
            image_name = f"tactile_image_{experiment_number}_step_{t}.npy"
            tactile_image_names.append(image_name)
            np.save(os.path.join(save_dir, image_name), tactile_images[t])

        sequence_length = self.context_length + self.horizon_length
        for t in range(len(tactile_images) - sequence_length):
            robot_sequence = robot_data[t : t + sequence_length]
            tactile_sequence = tactile_images[t : t + sequence_length] # Stacked image sequence: (Seq_Len, 32, 32, 1)
            tactile_image_name_sequence = tactile_image_names[t : t + sequence_length]
            time_step_sequence = time_steps[t : t + sequence_length]

            # Save sequence files
            robot_file = f"robot_data_sequence_{index_to_save}.npy"
            tactile_seq_file = f"tactile_data_sequence_{index_to_save}.npy"
            tactile_img_names_file = f"tactile_image_name_sequence_{index_to_save}.npy"
            exp_num_file = f"experiment_number_{index_to_save}.npy"
            time_step_file = f"time_step_data_{index_to_save}.npy"

            np.save(os.path.join(save_dir, robot_file), robot_sequence)
            np.save(os.path.join(save_dir, tactile_seq_file), tactile_sequence)
            np.save(os.path.join(save_dir, tactile_img_names_file), tactile_image_name_sequence)
            np.save(os.path.join(save_dir, exp_num_file), experiment_number)
            np.save(os.path.join(save_dir, time_step_file), time_step_sequence)

            # Record entry for index mapping
            ref = [
                robot_file,
                tactile_seq_file,
                tactile_img_names_file,
                exp_num_file,
                time_step_file
            ]
            self.path_file.append(ref)
            index_to_save += 1

        self.save_map(save_dir)

    def save_map(self, path: str):
        csv_path = os.path.join(path, "map.csv")
        with open(csv_path, "w", newline="") as csvfile:
            writer = csv.writer(csvfile, quoting=csv.QUOTE_ALL)
            writer.writerow([
                "robot_data_sequence",
                "tactile_data_sequence",
                "tactile_image_name_sequence",
                "experiment_number",
                "time_steps"
            ])
            for row in self.path_file:
                writer.writerow(row)
        print(f"Saved dataset map with {len(self.path_file)} sequence entries to '{csv_path}'.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert HDF5 or CSV tactile data into policy training frames and index map.")

    parser.add_argument("--input_robot", type=str, required=True, help="Path to input file")
    parser.add_argument("--input_tactile", type=str, required=True, help="Path to input file for tactile")
    parser.add_argument("--out-dir", type=str, default="policy_dataset", help="Output directory for frames and dataset_map.csv")
    parser.add_argument("--max-capacity", type=float, default=1200.0, help="Fixed max physical sensor bound (preserves absolute scale across frames)")

    args = parser.parse_args()
    formatter = PolicyDataFormatter(
        image_height=32,
        image_width=32,
        context_length=10,
        horizon_length=10,
        max_force_capacity=1200.0
    )

    formatter.process_and_save(
        robot_h5_path=args.input_robot,
        tactile_h5_path=args.input_tactile,
        save_dir="policy_dataset_trial_0",
        experiment_number=0
    )