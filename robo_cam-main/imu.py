"""ESP32-WROOM-32 + MPU-6050 tilt sensor over I2C (SDA GPIO 21, SCL GPIO 22).

ESP32 I2C communication matching the RiceTrack hardware diagram:
  - MPU-6050 SDA -> ESP32 GPIO 21 (I2C SDA)
  - MPU-6050 SCL -> ESP32 GPIO 22 (I2C SCL)
  - MPU-6050 VCC -> ESP32 3.3V
  - MPU-6050 GND -> ESP32 GND

Emits:
  - reading(roll, pitch)    every new sample (20 Hz, degrees)
  - state_changed(bool)     True  = MPU-6050 answering on ESP32 I2C
                            False = MPU-6050 disconnected or I2C unavailable
"""

from PySide6.QtCore import QObject, Signal
from esp32_hardware import PINS, ESP32MPU6050


class IMUReader(QObject):
    reading = Signal(float, float)   # roll, pitch (degrees)
    state_changed = Signal(bool)

    def __init__(self, bus_num: int = getattr(PINS, "i2c_bus", 0), parent=None):
        super().__init__(parent)
        self._mpu = ESP32MPU6050(bus_num=bus_num, parent=self)
        self._mpu.imu_reading.connect(self.reading)
        self._mpu.imu_state.connect(self.state_changed)

    def start(self):
        self._mpu.start()

    def stop(self):
        self._mpu.stop()
