import queue
import threading
import time
from droid.misc.time import time_ms
import numpy as np

from robotiq_tactile_sensor.sensor import TSF85TactileSensor


class TactileSensorInterface:

    def __init__(self, port=None, auto_connect=True, keep_baseline=False):
        self.t_sensor = TSF85TactileSensor()
        self.port = port
        self.is_connected = False
        self.latest_frame = None
        self.latest_obs = None
        self.keep_baseline = keep_baseline
        self.frame_count = 0
        self.frame_queue = queue.Queue(maxsize=10000)
        self.is_running = False
        self.frequency = 1000
        self.thread = None

        if auto_connect:
            self.connect()

    def connect(self) -> bool:
        """Find, connect, and calibrate tactile sensors."""
        try:
            target_port = self.port or self.t_sensor.find_sensor()

            if not target_port:
                print("[TactileSensorInterface Warning] Sensor not found.")
                self.is_connected = False
                return False

            if not self.t_sensor.connect(target_port):
                print(
                    f"[TactileSensorInterface Warning] Failed to connect to port: {target_port}"
                )
                self.is_connected = False
                return False

            if not self.t_sensor.start_autosend(period_ms=1):
                print(
                    "[TactileSensorInterface Warning] Failed to start data autosend stream."
                )
                self.t_sensor.cleanup()
                self.is_connected = False
                return False

            self.t_sensor.detect_connected_fingers()
            print(
                "[TactileSensorInterface] Calibrating initial baseline (please leave untouched)..."
            )
            self.calibrate_baseline(num_samples=500)
            self.is_running = True
            self.thread = threading.Thread(target=self._poll_sensor_loop, daemon=True)
            self.thread.start()

            self.is_connected = True

            print("[TactileSensorInterface] Initialization complete.")
            return True

        except Exception as e:
            print(f"[TactileSensorInterface Error] Connection failed: {e}")
            self.is_connected = False
            self.is_running = False
            return False

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
        """Poll sensor for data."""
        loop_count = 0
        last_stat_time = time.perf_counter()
        while self.is_running:
            if self.is_connected and self.t_sensor:
                try:
                    # loop_count += 1
                    # now = time.perf_counter()
                    # elapsed = now - last_stat_time

                    # if elapsed >= 1.0:
                    #     fps = loop_count / elapsed
                    #     bytes_in_buffer = (
                    #         self.t_sensor.serial_port.in_waiting
                    #         if self.t_sensor.serial_port
                    #         else 0
                    #     )
                    #     print(
                    #         f"[Tactile Thread Diagnostic] Loop Rate: {fps:.1f} Hz | OS Buffer: {bytes_in_buffer} bytes"
                    #     )

                    #     loop_count = 0
                    #     last_stat_time = now
                    for data in self.t_sensor.poll_data():
                        try:
                            self.frame_queue.put_nowait(data)
                        except queue.Full:
                            pass

                        self.latest_frame = data

                    # if hasattr(self, "latest_frame") and self.latest_frame:
                    #     self.latest_obs = self.get_obs_dict(self.latest_frame)

                except Exception as e:
                    time.sleep(0.01)
            else:
                time.sleep(0.01)

    def get_obs_dict(self, frame):
        """Build observation dict from frame data."""
        try:
            tactile_obs = {
                "connected_fingers": [
                    finger for finger in self.t_sensor.connected_fingers
                ],
                "fingers": {},
            }

            for finger_id in self.t_sensor.connected_fingers:
                if finger_id >= len(frame.fingers):
                    continue

                finger = frame.fingers[finger_id]
                baseline = self.t_sensor.baseline[finger_id]

                # Element-wise baseline correction for 28 taxels (7x4 grid)
                static_corrected = np.array(
                    [s - b for s, b in zip(finger.static_tactile, baseline)],
                    dtype=np.int16,
                ).reshape((7, 4))

                tactile_obs["fingers"][str(finger_id)] = {
                    "static_tactile": static_corrected,
                    "dynamic_tactile": int(finger.dynamic_tactile),
                    "accelerometer": np.array(
                        finger.accelerometer, dtype=np.int16
                    ),
                    "gyroscope": np.array(finger.gyroscope, dtype=np.int16),
                    "observation_timestamp": time_ms(),
                }

            return tactile_obs

        except Exception as e:
            print(
                f"[TactileSensorInterface Error] Error parsing frame observation: {e}"
            )
            return None

    def read_tactile_sensor_frame(self):
        """Get latest tactile sensor observation."""
        timestamp_dict = {"tactile_read_start": time_ms()}

        if (
            not self.is_connected
            or not self.t_sensor
        ):
            timestamp_dict["tactile_read_end"] = time_ms()
            return None, timestamp_dict

        timestamp_dict["tactile_read_end"] = time_ms()
        if hasattr(self, "latest_frame") and self.latest_frame:
            self.latest_obs = self.get_obs_dict(self.latest_frame)

        return self.latest_obs, timestamp_dict

    def read_tactile_sensor(self):
        """Get tactile sensor observation and timestamp range since last read."""
        timestamp_dict = {"tactile_read_start": time_ms()}

        if (
            not self.is_connected
            or not self.t_sensor
            or self.frame_queue.empty()
        ):
            timestamp_dict["tactile_read_end"] = time_ms()
            return None, timestamp_dict

        first_obs_dict = None
        first_frame = True

        while not self.frame_queue.empty():
            try:
                frame_raw = self.frame_queue.get_nowait()
                if first_frame:
                    first_obs_dict = self.get_obs_dict(frame_raw)
                    first_frame = False
            except queue.Empty:
                break

        try:
            if first_obs_dict and "fingers" in first_obs_dict:
                first_key = list(first_obs_dict["fingers"].keys())[0]
                timestamp_dict["tactile_read_start"] = first_obs_dict[
                    "fingers"
                ][first_key]["observation_timestamp"]
            self.latest_obs = self.get_obs_dict(frame_raw)

            if self.latest_obs and "fingers" in self.latest_obs:
                last_key = list(self.latest_obs["fingers"].keys())[0]
                timestamp_dict["tactile_read_end"] = self.latest_obs["fingers"][
                    last_key
                ]["observation_timestamp"]
            else:
                timestamp_dict["tactile_read_end"] = time_ms()

        except (IndexError, KeyError, TypeError):
            timestamp_dict["tactile_read_start"] = time_ms()
            timestamp_dict["tactile_read_end"] = time_ms()

        return self.latest_obs, timestamp_dict

    def close(self):
        self.is_running = False

        if self.thread is not None:
            try:
                self.thread.join(timeout=1.0)
            except Exception as e:
                print(
                    f"[TactileSensorInterface Error] Thread join error: {e}"
                )

        if self.t_sensor:
            try:
                self.t_sensor.cleanup()
            except Exception as e:
                print(
                    f"[TactileSensorInterface Error] Sensor cleanup error: {e}"
                )
            finally:
                self.is_connected = False
