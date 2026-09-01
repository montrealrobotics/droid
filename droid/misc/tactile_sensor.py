import time
import numpy as np
from droid.misc.time import time_ms

from core.sensor import TSF85TactileSensor


class TactileSensorInterface:

    def __init__(self, port=None, auto_connect=True, keep_baseline=False):
        self.t_sensor = TSF85TactileSensor()
        self.port = port
        self.is_connected = False
        self.latest_frame = None
        self.keep_baseline = False
        self.frame_count = 0

        if auto_connect:
            self.connect()

    def connect(self) -> bool:
        """Find, connect, and calibrate tactile sensors."""
        target_port = self.port or self.t_sensor.find_sensor()

        if not target_port:
            print("[TactileWrapper Warning] Sensor not found.")
            self.is_connected = False
            return False

        if not self.t_sensor.connect(target_port):
            print(f"[TactileWrapper Warning] Failed to connect to port: {target_port}")
            self.is_connected = False
            return False

        if not self.t_sensor.start_autosend(period_ms=1):
            print("[TactileWrapper Warning] Failed to start data autosend stream.")
            self.t_sensor.cleanup()
            self.is_connected = False
            return False

        self.t_sensor.detect_connected_fingers()
        print("[TactileWrapper] Calibrating initial baseline (please leave untouched)...")
        self.calibrate_baseline(num_samples=500)
        
        self.is_connected = True
        print("[TactileWrapper] Initialization complete.")
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

    def read_tactile_sensor(self):
        """
        Poll latest stream frame and format into NumPy observation dict.
        
        Returns:
            tactile_obs (dict): Formatted per-finger sensor readings.
            timestamp (int): Local millisecond timestamp of read execution.
        """
        read_time = time_ms()

        if not self.is_connected or not self.t_sensor:
            return None, read_time

        for data in self.t_sensor.poll_data():
            self.latest_frame = data

        if self.latest_frame is None:
            return None, read_time

        tactile_obs = {
            "connected_fingers": list(self.t_sensor.connected_fingers),
            "fingers": {},
        }

        for finger_id in self.t_sensor.connected_fingers:
            finger = self.latest_frame.fingers[finger_id]
            baseline = self.t_sensor.baseline[finger_id]

            # Element-wise baseline correction for 28 taxels (7x4 grid)
            static_corrected = np.array(
                [s - b for s, b in zip(finger.static_tactile, baseline)], dtype=np.int16
            ).reshape((7, 4))

            raw_static = np.array(finger.static_tactile, dtype=np.uint16).reshape((7, 4))

            tactile_obs["fingers"][finger_id] = {
                "static_tactile": static_corrected,
                "raw_static_tactile": raw_static,
                "dynamic_tactile": int(finger.dynamic_tactile),
                "accelerometer": np.array(finger.accelerometer, dtype=np.int16),
                "gyroscope": np.array(finger.gyroscope, dtype=np.int16),
                "sensor_timestamp": int(finger.timestamp),
            }

        return tactile_obs, read_time

    def close(self):
        if self.t_sensor:
            self.t_sensor.cleanup()
            self.is_connected = False

