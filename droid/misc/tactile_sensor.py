import time
import numpy as np
from droid.misc.time import time_ms
import threading
import queue

from robotiq_tactile_sensor.sensor import TSF85TactileSensor


class TactileSensorInterface:

    def __init__(self, port=None, auto_connect=True, keep_baseline=False):
        self.t_sensor = TSF85TactileSensor()
        self.port = port
        self.is_connected = False
        self.latest_frame = None
        self.keep_baseline = keep_baseline
        self.frame_count = 0
        self.frame_queue = queue.Queue(maxsize=10000)
        self.is_running = False
        self.frequency = 1000

        if auto_connect:
            self.connect()

    def connect(self) -> bool:
        """Find, connect, and calibrate tactile sensors."""
        target_port = self.port or self.t_sensor.find_sensor()

        if not target_port:
            print("[TactileSensorInterface Warning] Sensor not found.")
            self.is_connected = False
            return False

        if not self.t_sensor.connect(target_port):
            print(f"[TactileSensorInterface Warning] Failed to connect to port: {target_port}")
            self.is_connected = False
            return False

        if not self.t_sensor.start_autosend(period_ms=1):
            print("[TactileSensorInterface Warning] Failed to start data autosend stream.")
            self.t_sensor.cleanup()
            self.is_connected = False
            return False

        self.t_sensor.detect_connected_fingers()
        print("[TactileSensorInterface] Calibrating initial baseline (please leave untouched)...")
        self.calibrate_baseline(num_samples=500)
        
        self.is_connected = True
        self.thread = threading.Thread(target=self._poll_sensor_loop, daemon=True)
        self.thread.start()
        self.is_running = True

        print("[TactileSensorInterface] Initialization complete.")
        return True

    def calibrate_baseline(self, num_samples: int = 200):
        """Re-zero taxel baseline offsets."""
        if self.t_sensor:
            self.t_sensor.reset_baseline(num_samples=num_samples)

    def start_recording(self, requested_filename):
        if self.t_sensor:
            self.t_sensor.start_recording(requested_filename, keep_baseline=self.keep_baseline)

    def stop_recording(self):
        if self.t_sensor:
            self.t_sensor.stop_recording()
            self.frame_count = self.t_sensor.recorder.recorded_count

    def _poll_sensor_loop(self):
        """poll sensor to clear serial buffer"""
        while self.is_running:
            if self.is_connected and self.t_sensor:
                for data in self.t_sensor.poll_data():
                    obs_dict = self.get_obs_dict(data)

                    try:
                        self.frame_queue.put_nowait(obs_dict)
                    except queue.Full:
                        pass

                    self.latest_frame = data
                    self.latest_obs = obs_dict

            time.sleep(0.0002)

    def get_obs_dict(self, frame):
        """
        Build observation dict.
        """
        tactile_obs = {
            "connected_fingers": [finger for finger in self.t_sensor.connected_fingers],
            "fingers": {},
        }

        for finger_id in self.t_sensor.connected_fingers:
            finger = frame.fingers[finger_id]
            baseline = self.t_sensor.baseline[finger_id]

            # Element-wise baseline correction for 28 taxels (7x4 grid)
            static_corrected = np.array(
                [s - b for s, b in zip(finger.static_tactile, baseline)], dtype=np.int16
            ).reshape((7, 4))

            raw_static = np.array(finger.static_tactile, dtype=np.uint16).reshape((7, 4))

            tactile_obs["fingers"][str(finger_id)] = {
                "static_tactile": static_corrected,
                "raw_static_tactile": raw_static,
                "dynamic_tactile": int(finger.dynamic_tactile),
                "accelerometer": np.array(finger.accelerometer, dtype=np.int16),
                "gyroscope": np.array(finger.gyroscope, dtype=np.int16),
                "sensor_timestamp": int(finger.timestamp),
                "observation_timestamp": time_ms(),
            }

        return tactile_obs

    def read_tactile_sensor_frame(self):
        """
        Get latest tactile sensor observation.

        Returns:
            tactile_obs (dict): Formatted per-finger sensor readings.
            timestamp_dict (dict): Dictionary of start and end read timestamps.
        """
        timestamp_dict = {"tactile_read_start": time_ms()}

        if not self.is_connected or not self.t_sensor:
            timestamp_dict["tactile_read_end"] = time_ms()
            return None, timestamp_dict

        if self.latest_frame is None:
            return None, timestamp_dict

        timestamp_dict["tactile_read_end"] = time_ms()
        tactile_obs = self.latest_obs

        return tactile_obs, timestamp_dict

    def read_tactile_sensor(self):
        """
        Get all tactile sensor observations since last read.

        Returns:
            obs_list (list): List of formatted per-finger sensor readings.
            timestamp_dict (dict): Dictionary of start and end read timestamps.
        """
        timestamp_dict = {"tactile_read_start": time_ms()}

        if not self.is_connected or not self.t_sensor:
            timestamp_dict["tactile_read_end"] = time_ms()
            return None, timestamp_dict

        obs_list = []

        while not self.frame_queue.empty():
            try:
                frame_obs = self.frame_queue.get_nowait()
                obs_list.append(frame_obs)
            except queue.Empty:
                break

        if len(obs_list) == 0:
            return None, timestamp_dict

        timestamp_dict["tactile_read_start"] = obs_list[0]['fingers']['0']["observation_timestamp"]
        timestamp_dict["tactile_read_end"] = obs_list[-1]['fingers']['0']["observation_timestamp"]

        return obs_list, timestamp_dict

    def close(self):
        self.is_running = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)

        if self.t_sensor:
            self.t_sensor.cleanup()
            self.is_connected = False

