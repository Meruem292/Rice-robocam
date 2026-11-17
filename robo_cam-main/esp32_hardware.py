"""
esp32_hardware.py
-----------------
Unified hardware interface for RiceTrack on the **ESP32-WROOM-32 DevKit V1**,
communicating with the host PC over Serial USB (115200 baud, newline-delimited
JSON). The host PC runs the Python/PySide6 UI and handles the USB webcam via
OpenCV. The ESP32 handles motor PWM, IMU reading, and GPS reading.

Complete wiring schematic:

1. Microcontroller:
   - ESP32-WROOM-32 DevKit V1
   - Power: Buck Converter 24V→5V for ESP32 power via USB or VIN
           → onboard AMS1117 3.3V regulator for ESP32 logic
   - Host connection: USB Serial @ 115200 baud (newline-delimited JSON)

2. USB Webcam (Host PC):
   - Standard USB webcam connected to the host PC (not to the ESP32)
   - Driven via OpenCV VideoCapture on the host side

3. NEO-6M GPS Module (ESP32 UART2):
   - TX  → ESP32 GPIO17 (ESP32 TX2 → GPS RX)
   - RX  → ESP32 GPIO16 (GPS TX → ESP32 RX2)
   - VCC → 3.3V Rail
   - GND → GND
   - ESP32 reads NMEA @ 9600 baud on UART2, forwards parsed data over USB serial

4. IMU Gyroscope MPU-6050 (ESP32 I2C):
   - SDA → ESP32 GPIO21 (I2C SDA)
   - SCL → ESP32 GPIO22 (I2C SCL)
   - VCC → 3.3V Rail
   - GND → GND
   - I2C address 0x68, ESP32 reads at 20Hz, forwards over USB serial

5. Dual BTS7960 Motor Drivers + Dual MY1016Z2 DC Motors (Differential Drivetrain):
   - Traction Power: 24V 12Ah Ebike Battery → 40A Fuse → E-Stop → BTS7960 B+/B-
   - Logic power: ESP32 3.3V GPIO outputs → BTS7960 enable/PWM inputs
   - Left BTS7960:
       * R_EN → GPIO25, L_EN → GPIO26
       * RPWM → GPIO32 (LEDC Channel 0), LPWM → GPIO33 (LEDC Channel 1)
   - Right BTS7960:
       * R_EN → GPIO27, L_EN → GPIO14
       * RPWM → GPIO18 (LEDC Channel 2), LPWM → GPIO19 (LEDC Channel 3)
   - PWM: 1 kHz frequency, 10-bit resolution (ESP32 LEDC peripheral)
   - High-Current Motor Outputs: M+ / M- (14 AWG) → Left & Right MY1016Z2 24V DC Motors

Serial Protocol (115200 baud, newline-delimited JSON):
   Host→ESP32:
       {"c":"m","l":0.65,"r":0.65}   — motor command (signed duty per side)
       {"c":"s"}                       — emergency stop
       {"c":"p"}                       — ping / heartbeat
    ESP32→Host:
        {"t":"i","r":1.2,"p":-0.5,"ok":1}           — IMU data (20Hz)
        {"t":"g","s":"fix","la":15.67,"lo":120.89,"n":8}  — GPS data (1Hz)
        {"t":"k",...}                               — pong + link/driver status
        {"t":"s","l":{...},"r":{...},"host":1,"ha":125,"up":42}
                                                    — driver/link status (1Hz)
"""

import json
import math
import os
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable
import urllib.request

import cv2
import numpy as np
import serial
import serial.tools.list_ports
from PySide6.QtCore import QObject, QTimer, Signal

try:
    from picamera2 import Picamera2
    HAS_PICAMERA2 = True
    _PICAM_IMPORT_ERR = None
except Exception as _err:
    Picamera2 = None
    HAS_PICAMERA2 = False
    _PICAM_IMPORT_ERR = str(_err)


# ==============================================================================
# PIN & HARDWARE CONFIGURATION (ESP32-WROOM-32 DevKit V1)
# ==============================================================================

@dataclass(frozen=True)
class ESP32PinConfig:
    """All ESP32 GPIO pin assignments and serial bridge configuration."""

    # I2C for MPU-6050 IMU (SDA = GPIO21, SCL = GPIO22)
    i2c_bus: int = 0
    i2c_sda_gpio: int = 21
    i2c_scl_gpio: int = 22
    mpu6050_addr: int = 0x68

    # UART2 for NEO-6M GPS (TX = GPIO17 ESP→GPS, RX = GPIO16 GPS→ESP)
    uart2_tx_gpio: int = 17
    uart2_rx_gpio: int = 16
    gps_baud: int = 9600

    # Left BTS7960 Dual Motor Driver → Left MY1016Z2 DC Motor
    left_r_en_gpio: int = 25
    left_l_en_gpio: int = 26
    left_rpwm_gpio: int = 32   # LEDC Channel 0
    left_lpwm_gpio: int = 33   # LEDC Channel 1

    # Right BTS7960 Dual Motor Driver → Right MY1016Z2 DC Motor
    right_r_en_gpio: int = 27
    right_l_en_gpio: int = 14
    right_rpwm_gpio: int = 18  # LEDC Channel 2
    right_lpwm_gpio: int = 19  # LEDC Channel 3

    # PWM configuration (ESP32 LEDC peripheral: 1 kHz, 10-bit resolution)
    pwm_freq_hz: int = 1000

    # Default duty cycles for MY1016Z2 24V DC Motors
    drive_duty: float = 0.75   # 75% duty cycle for solid starting torque (~0.7 m/s)
    turn_duty: float = 0.75    # 75% duty cycle for confident pivot turns

    # Serial USB bridge configuration (ESP32 ↔ Host PC)
    bridge_baud: int = 115200
    bridge_ports: tuple = (
        "COM3", "COM4", "COM5",
        "/dev/serial0", "/dev/ttyAMA0", "/dev/ttyS0",
        "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0", "/dev/ttyACM1"
    )


PINS = ESP32PinConfig()


# ==============================================================================
# 1. ESP32 SERIAL BRIDGE (USB SERIAL ↔ ESP32-WROOM-32)
# ==============================================================================

class ESP32SerialBridge(QObject):
    """
    Background-thread serial bridge between the host PC and the ESP32-WROOM-32
    over USB (115200 baud, newline-delimited JSON).

    Reads incoming JSON messages from the ESP32, dispatches IMU and GPS data to
    registered callbacks, and provides methods for sending motor commands.
    Auto-reconnects when the serial port is lost. Falls back to simulation mode
    when no ESP32 is detected (for Windows development).
    """
    connection_state = Signal(bool)   # True = ESP32 serial port connected

    RECONNECT_INTERVAL_S = 2.0
    HEARTBEAT_INTERVAL_S = 3.0
    HEARTBEAT_TIMEOUT_S = 8.0
    SIM_IMU_HZ = 20
    SIM_GPS_HZ = 1

    def __init__(self, parent=None):
        super().__init__(parent)
        self._serial: serial.Serial | None = None
        self._connected = False
        self._simulating = False
        self._running = False
        self._lock = threading.Lock()

        # Callbacks: list of callables for each data type
        self._imu_callbacks: list[Callable] = []
        self._gps_callbacks: list[Callable] = []

        self._read_thread: threading.Thread | None = None
        self._last_pong_time = 0.0
        self._last_heartbeat_time = 0.0

        # Link + per-driver monitoring (reported by ESP32 firmware)
        self._status_lock = threading.Lock()
        self._last_status_time = 0.0
        self._esp32_uptime_s: int | None = None
        self._host_ack = False
        self._host_ack_age_ms: int | None = None
        self._left_motor = self._empty_motor_status()
        self._right_motor = self._empty_motor_status()

        # Simulation state
        self._sim_timer_imu: QTimer | None = None
        self._sim_timer_gps: QTimer | None = None
        self._sim_roll = 0.0
        self._sim_pitch = 0.0
        self._sim_yaw = 0.0
        self._current_roll = 0.0
        self._current_pitch = 0.0
        self._current_yaw = 0.0

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def simulating(self) -> bool:
        return self._simulating

    @staticmethod
    def _empty_motor_status() -> dict:
        return {
            "reported": False,
            "connected": False,
            "enabled": False,
            "duty": 0.0,
            "direction": "stop",
        }

    @classmethod
    def _normalize_motor_status(cls, raw: Any) -> dict:
        """Normalize a firmware driver status block into the web schema."""
        if not isinstance(raw, dict):
            return cls._empty_motor_status()
        duty = float(raw.get("d", 0.0) or 0.0)
        direction = str(raw.get("dir", "stop") or "stop").lower()
        if direction not in ("forward", "reverse", "stop"):
            direction = "stop"
        return {
            "reported": True,
            "connected": bool(raw.get("c", 0)),
            "enabled": bool(raw.get("e", 0)),
            "duty": round(duty, 3),
            "direction": direction,
        }

    def link_snapshot(self) -> dict:
        """Return ESP32 link health and left/right motor-driver status."""
        now = time.monotonic()
        last_rx = max(self._last_status_time, self._last_pong_time)
        telemetry_age = round(now - last_rx, 1) if last_rx > 0 else None
        telemetry_ok = (
            self._connected
            and last_rx > 0
            and (now - last_rx) <= self.HEARTBEAT_TIMEOUT_S
        )
        with self._status_lock:
            left = dict(self._left_motor)
            right = dict(self._right_motor)
            host_ack = self._host_ack and telemetry_ok
            host_age = self._host_ack_age_ms
            uptime = self._esp32_uptime_s
        return {
            "esp32_connected": self._connected,
            "simulating": self._simulating,
            "telemetry_ok": telemetry_ok,
            "telemetry_age_s": telemetry_age,
            "host_ack": host_ack,
            "host_age_ms": host_age,
            "esp32_uptime_s": uptime,
            "left": left,
            "right": right,
        }

    def register_imu_callback(self, cb: Callable):
        """Register a callback: cb(roll_deg, pitch_deg, ok_bool)."""
        if cb not in self._imu_callbacks:
            self._imu_callbacks.append(cb)

    def unregister_imu_callback(self, cb: Callable):
        try:
            self._imu_callbacks.remove(cb)
        except ValueError:
            pass

    def register_gps_callback(self, cb: Callable):
        """Register a callback: cb(state_str, lat, lon, sats)."""
        if cb not in self._gps_callbacks:
            self._gps_callbacks.append(cb)

    def unregister_gps_callback(self, cb: Callable):
        try:
            self._gps_callbacks.remove(cb)
        except ValueError:
            pass

    def start(self):
        """Start the serial bridge. Attempts connection, falls back to simulation."""
        if self._running:
            return
        self._running = True
        self._read_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._read_thread.start()

    def stop(self):
        """Stop the serial bridge and close the port."""
        self._running = False
        self._stop_simulation()
        if self._read_thread is not None and self._read_thread.is_alive():
            self._read_thread.join(timeout=2.0)
        self._read_thread = None
        self._close_serial()

    def send_motor(self, left_duty: float, right_duty: float):
        """Send a motor command to the ESP32: {"c":"m","l":<left>,"r":<right>}."""
        msg = {"c": "m", "l": round(left_duty, 3), "r": round(right_duty, 3)}
        sent = self._send_json(msg)
        if not sent and not self._connected:
            self._apply_simulated_motor(left_duty, right_duty)

    def send_stop(self):
        """Send emergency stop to the ESP32: {"c":"s"}."""
        sent = self._send_json({"c": "s"})
        if not sent and not self._connected:
            self._apply_simulated_motor(0.0, 0.0)

    @staticmethod
    def _sim_side(duty: float) -> dict:
        duty = max(-1.0, min(1.0, float(duty)))
        return {
            "reported": True,
            "connected": True,
            "enabled": abs(duty) > 0.001,
            "duty": round(duty, 3),
            "direction": "forward" if duty > 0.001 else "reverse" if duty < -0.001 else "stop",
        }

    def _apply_simulated_motor(self, left_duty: float, right_duty: float):
        """Mirror motor commands into status when no ESP32 is attached."""
        with self._status_lock:
            self._left_motor = self._sim_side(left_duty)
            self._right_motor = self._sim_side(right_duty)
            self._host_ack = True
            self._host_ack_age_ms = 0
            self._last_status_time = time.monotonic()

    def send_ping(self):
        """Send heartbeat ping to the ESP32: {"c":"p"}."""
        self._send_json({"c": "p"})

    def send_reset_yaw(self):
        """Send command to reset ESP32 integrated yaw to 0.0: {"c":"ry"}."""
        self._send_json({"c": "ry"})
        self._current_yaw = 0.0
        self._sim_yaw = 0.0

    def _send_json(self, obj: dict) -> bool:
        """Serialize and send a JSON message over serial (with newline delimiter)."""
        with self._lock:
            ser = self._serial
            if ser is not None and ser.is_open:
                try:
                    line = json.dumps(obj, separators=(",", ":")) + "\n"
                    ser.write(line.encode("ascii"))
                    return True
                except (serial.SerialException, OSError):
                    pass
        return False

    # ----- Connection management -----

    def _find_esp32_port(self) -> str | None:
        """Auto-detect the ESP32 serial port."""
        # First, try known port names
        for port_name in PINS.bridge_ports:
            try:
                s = serial.Serial(port_name, PINS.bridge_baud, timeout=0.5)
                s.close()
                return port_name
            except (serial.SerialException, OSError):
                continue
        # Second, try scanning all available ports for CP210x / CH340 / FTDI chips
        try:
            ports = serial.tools.list_ports.comports()
            for p in ports:
                desc = (p.description or "").lower()
                vid_pid = f"{p.vid or 0:04x}:{p.pid or 0:04x}"
                if any(kw in desc for kw in ("cp210", "ch340", "ch910", "ftdi", "silicon labs", "usb-serial")):
                    return p.device
                # Common ESP32 USB-UART bridge VID:PIDs
                if vid_pid in ("10c4:ea60", "1a86:7523", "1a86:55d4", "0403:6001"):
                    return p.device
        except Exception:
            pass
        return None

    def _try_connect(self) -> bool:
        """Attempt to open the serial connection to the ESP32."""
        port = self._find_esp32_port()
        if port is None:
            return False
        try:
            self._serial = serial.Serial(port, PINS.bridge_baud, timeout=0.1)
            self._serial.reset_input_buffer()
            time.sleep(0.1)
            self._last_pong_time = time.monotonic()
            self._last_heartbeat_time = time.monotonic()
            self._set_connected(True)
            self._stop_simulation()
            return True
        except (serial.SerialException, OSError):
            self._serial = None
            return False

    def _close_serial(self):
        with self._lock:
            if self._serial is not None:
                try:
                    self._serial.close()
                except Exception:
                    pass
                self._serial = None
        if self._connected:
            self._set_connected(False)

    def _set_connected(self, state: bool):
        if state != self._connected:
            self._connected = state
            self.connection_state.emit(state)

    # ----- Reader thread -----

    def _reader_loop(self):
        """Main loop: connect → read → reconnect. Runs in background thread."""
        while self._running:
            # Try to connect
            if self._serial is None or not self._serial.is_open:
                if not self._try_connect():
                    # No ESP32 found — enter simulation mode
                    self._start_simulation()
                    # Wait before retrying
                    for _ in range(int(self.RECONNECT_INTERVAL_S * 10)):
                        if not self._running:
                            return
                        time.sleep(0.1)
                    continue

            ser = self._serial
            if ser is None or not ser.is_open:
                continue

            # Connected — read lines
            try:
                raw = ser.readline()
                if raw:
                    self._process_line(raw)
            except (serial.SerialException, OSError, AttributeError):
                self._close_serial()
                continue

            # Heartbeat
            now = time.monotonic()
            if now - self._last_heartbeat_time >= self.HEARTBEAT_INTERVAL_S:
                self.send_ping()
                self._last_heartbeat_time = now

            # Check for timeout (no pong received)
            if now - self._last_pong_time > self.HEARTBEAT_TIMEOUT_S:
                self._close_serial()

    def _apply_motor_status(self, msg: dict):
        """Update left/right driver + host-link state from an ESP32 status packet."""
        has_left = isinstance(msg.get("l"), dict)
        has_right = isinstance(msg.get("r"), dict)
        if not has_left and not has_right:
            return
        with self._status_lock:
            if has_left:
                self._left_motor = self._normalize_motor_status(msg.get("l"))
            if has_right:
                self._right_motor = self._normalize_motor_status(msg.get("r"))
            if "host" in msg:
                self._host_ack = bool(msg.get("host", 0))
            if "ha" in msg:
                try:
                    self._host_ack_age_ms = int(msg.get("ha"))
                except (TypeError, ValueError):
                    self._host_ack_age_ms = None
            if "up" in msg:
                try:
                    self._esp32_uptime_s = int(msg.get("up"))
                except (TypeError, ValueError):
                    self._esp32_uptime_s = None
            self._last_status_time = time.monotonic()

    def _process_line(self, raw: bytes):
        """Parse a single newline-delimited JSON message from ESP32."""
        line = raw.decode("ascii", errors="ignore").strip()
        if not line:
            return
        try:
            msg = json.loads(line)
        except json.JSONDecodeError:
            return

        msg_type = msg.get("t")
        if msg_type:
            self._last_pong_time = time.monotonic()

        if msg_type == "i":
            # IMU data: {"t":"i","r":<roll>,"p":<pitch>,"y":<yaw>,"ok":<0|1>}
            roll = float(msg.get("r", 0.0))
            pitch = float(msg.get("p", 0.0))
            yaw = float(msg.get("y", 0.0))
            ok = bool(msg.get("ok", 0))
            self._current_roll = roll
            self._current_pitch = pitch
            self._current_yaw = yaw
            for cb in self._imu_callbacks:
                try:
                    try:
                        cb(roll, pitch, yaw, ok)
                    except TypeError:
                        cb(roll, pitch, ok)
                except Exception:
                    pass

        elif msg_type == "g":
            # GPS data: {"t":"g","s":"fix","la":<lat>,"lo":<lon>,"n":<sats>}
            state = str(msg.get("s", "none"))
            lat = float(msg.get("la", 0.0))
            lon = float(msg.get("lo", 0.0))
            sats = int(msg.get("n", 0))
            for cb in self._gps_callbacks:
                try:
                    cb(state, lat, lon, sats)
                except Exception:
                    pass

        elif msg_type == "k":
            # Pong response (also carries link/driver status in new firmware)
            self._last_pong_time = time.monotonic()
            self._apply_motor_status(msg)

        elif msg_type == "s":
            # Periodic link/driver status: {"t":"s","l":{...},"r":{...},...}
            self._apply_motor_status(msg)

    # ----- Simulation fallback -----

    def _start_simulation(self):
        """Enter simulation mode for Windows development (no ESP32 connected)."""
        if self._simulating:
            return
        self._simulating = True
        self._set_connected(False)
        with self._status_lock:
            self._left_motor = self._sim_side(0.0)
            self._right_motor = self._sim_side(0.0)
            self._host_ack = True
            self._host_ack_age_ms = 0
            self._last_status_time = time.monotonic()

        # Use QTimers for simulation ticks (must be created/started in main thread context)
        # Since we may be called from reader thread, schedule via thread-safe approach
        self._sim_timer_imu = QTimer()
        self._sim_timer_imu.setInterval(int(1000 / self.SIM_IMU_HZ))
        self._sim_timer_imu.timeout.connect(self._sim_tick_imu)
        self._sim_timer_imu.start()

        self._sim_timer_gps = QTimer()
        self._sim_timer_gps.setInterval(int(1000 / self.SIM_GPS_HZ))
        self._sim_timer_gps.timeout.connect(self._sim_tick_gps)
        self._sim_timer_gps.start()

    def _stop_simulation(self):
        """Exit simulation mode."""
        if not self._simulating:
            return
        self._simulating = False
        if self._sim_timer_imu is not None:
            self._sim_timer_imu.stop()
            self._sim_timer_imu = None
        if self._sim_timer_gps is not None:
            self._sim_timer_gps.stop()
            self._sim_timer_gps = None

    def _sim_tick_imu(self):
        """Generate simulated IMU data (gentle oscillation + simulated yaw turning)."""
        t = time.monotonic()
        self._sim_roll = 2.0 * math.sin(t * 0.5)
        self._sim_pitch = 1.5 * math.cos(t * 0.3)
        dt = 1.0 / self.SIM_IMU_HZ
        with self._status_lock:
            ld = float(self._left_motor.get("duty", 0.0) or 0.0)
            rd = float(self._right_motor.get("duty", 0.0) or 0.0)
        # Differential steer yaw rate: turning right (ld > 0, rd < 0) -> +yaw
        turn_rate = (ld - rd) * 45.0  # ~45 deg/sec at 0.5 turn duty
        self._sim_yaw += turn_rate * dt
        self._current_roll = self._sim_roll
        self._current_pitch = self._sim_pitch
        self._current_yaw = self._sim_yaw
        for cb in self._imu_callbacks:
            try:
                try:
                    cb(self._sim_roll, self._sim_pitch, self._sim_yaw, True)
                except TypeError:
                    cb(self._sim_roll, self._sim_pitch, True)
            except Exception:
                pass

    def _sim_tick_gps(self):
        """Generate simulated GPS data (static position)."""
        for cb in self._gps_callbacks:
            try:
                cb("fix", 14.5995, 120.9842, 6)
            except Exception:
                pass


# Module-level singleton
_bridge_instance: ESP32SerialBridge | None = None
_bridge_lock = threading.Lock()


def get_bridge() -> ESP32SerialBridge:
    """Return the module-level ESP32SerialBridge singleton."""
    global _bridge_instance
    with _bridge_lock:
        if _bridge_instance is None:
            _bridge_instance = ESP32SerialBridge()
        return _bridge_instance


# ==============================================================================
# 2. MPU-6050 I2C IMU READER (VIA ESP32 SERIAL BRIDGE)
# ==============================================================================

class ESP32MPU6050(QObject):
    """
    Receives MPU-6050 IMU data from the ESP32 Serial Bridge.
    The ESP32 reads the MPU-6050 directly over I2C (SDA GPIO21, SCL GPIO22)
    and forwards roll/pitch data over USB serial as JSON.

    The `bus_num` parameter is accepted but ignored (for API compatibility
    with the imu.py module which used direct I2C bus access).
    """
    imu_reading = Signal(float, float)   # roll, pitch (degrees)
    imu_state = Signal(bool)             # True = MPU-6050 responding via ESP32

    def __init__(self, bus_num: int = 1, parent=None):
        super().__init__(parent)
        self._bus_num = bus_num  # Ignored — kept for API compatibility
        self._bridge = get_bridge()
        self._ok = False
        self._started = False

    def start(self):
        """Start receiving IMU data from the ESP32 bridge."""
        if self._started:
            return
        self._started = True
        self._bridge.start()
        self._bridge.register_imu_callback(self._on_imu_data)

    def stop(self):
        """Stop receiving IMU data."""
        self._started = False
        self._bridge.unregister_imu_callback(self._on_imu_data)
        self._set_ok(False)

    # Compatibility methods for legacy callers
    def _try_init(self) -> bool:
        self.start()
        return True

    def _close_bus(self):
        self.stop()

    def _poll(self):
        pass

    @property
    def yaw(self) -> float:
        """Current integrated yaw angle from MPU-6050 (degrees)."""
        return self._bridge._current_yaw

    def reset_yaw(self):
        """Reset MPU-6050 yaw angle to 0.0."""
        self._bridge.send_reset_yaw()

    def _on_imu_data(self, roll: float, pitch: float, *args):
        """Callback invoked by the ESP32 serial bridge with IMU data."""
        ok = bool(args[-1]) if args else True
        self._set_ok(ok)
        if ok:
            self.imu_reading.emit(roll, pitch)

    def _set_ok(self, ok: bool):
        if ok != self._ok:
            self._ok = ok
            self.imu_state.emit(ok)


# ==============================================================================
# 3. NEO-6M UART GPS READER (VIA ESP32 SERIAL BRIDGE)
# ==============================================================================

def parse_nmea_degree(raw_val: str, hemisphere: str) -> float | None:
    """Convert NMEA ddmm.mmmm / dddmm.mmmm into signed decimal degrees."""
    if not raw_val or not hemisphere or "." not in raw_val:
        return None
    try:
        dot_idx = raw_val.index(".")
        deg_Digits = dot_idx - 2
        if deg_Digits <= 0:
            return None
        degrees = float(raw_val[:deg_Digits])
        minutes = float(raw_val[deg_Digits:])
        decimal = degrees + minutes / 60.0
        if hemisphere in ("S", "W"):
            decimal = -decimal
        elif hemisphere not in ("N", "E"):
            return None
        return decimal
    except ValueError:
        return None


def verify_nmea_checksum(sentence: str) -> bool:
    """Validate an NMEA 0183 sentence checksum (*XX)."""
    if not sentence.startswith("$") or "*" not in sentence:
        return False
    body, _, checksum_hex = sentence[1:].partition("*")
    if len(checksum_hex) < 2:
        return False
    try:
        expected = int(checksum_hex[:2], 16)
    except ValueError:
        return False
    calc = 0
    for ch in body:
        calc ^= ord(ch)
    return calc == expected


class ESP32NEO6MGPS(QObject):
    """
    Receives NEO-6M GPS data from the ESP32 Serial Bridge.
    The ESP32 reads the NEO-6M directly over UART2 (TX GPIO17, RX GPIO16)
    at 9600 baud and forwards parsed GPS data over USB serial as JSON.

    The `feed(bytes)` method is kept for testing — it parses raw NMEA data
    locally, bypassing the ESP32 bridge.
    """
    gps_changed = Signal(str, float, float, int)  # state ("none"|"searching"|"fix"), lat, lon, sats

    REPORT_MS = 1000

    def __init__(self, port: str | None = None, baud: int = 9600, parent=None):
        super().__init__(parent)
        self._bridge = get_bridge()
        self._started = False

        # State
        self._has_fix = False
        self._lat = 0.0
        self._lon = 0.0
        self._sats = 0
        self._state = "none"
        self._last_update = 0.0

        # For local NMEA parsing via feed()
        self._buf = bytearray()

        # Periodic status report timer
        self._report_timer = QTimer(self)
        self._report_timer.timeout.connect(self._emit_status)

    def start(self):
        """Start receiving GPS data from the ESP32 bridge."""
        if self._started:
            return
        self._started = True
        self._bridge.start()
        self._bridge.register_gps_callback(self._on_gps_data)
        self._report_timer.start(self.REPORT_MS)

    def stop(self):
        """Stop receiving GPS data."""
        self._started = False
        self._bridge.unregister_gps_callback(self._on_gps_data)
        self._report_timer.stop()
        self._state = "none"
        self._has_fix = False
        self._sats = 0
        self.gps_changed.emit("none", 0.0, 0.0, 0)

    # Compatibility methods and properties for legacy callers
    def _try_open(self) -> bool:
        self.start()
        return True

    def _drop(self):
        self.stop()

    def _poll(self):
        pass

    @property
    def _serial(self):
        return getattr(self._bridge, "_serial", None)

    def feed(self, data: bytes):
        """
        Feed raw NMEA bytes for local parsing (public for testing).
        Allows unit tests to inject NMEA sentences without an ESP32.
        """
        self._buf += data
        while True:
            line, sep, rest = self._buf.partition(b"\n")
            if not sep:
                break
            self._buf = rest
            self._parse_nmea(line.decode("ascii", errors="ignore").strip())
        if len(self._buf) > 4096:
            self._buf.clear()

    def _on_gps_data(self, state: str, lat: float, lon: float, sats: int):
        """Callback invoked by the ESP32 serial bridge with parsed GPS data."""
        self._last_update = time.monotonic()
        self._sats = sats
        if state == "fix":
            self._has_fix = True
            self._lat = lat
            self._lon = lon
            self._state = "fix"
        elif state == "searching":
            self._has_fix = False
            self._state = "searching"
        else:
            self._has_fix = False
            self._state = "none"
        self.gps_changed.emit(self._state, self._lat if self._has_fix else 0.0,
                              self._lon if self._has_fix else 0.0, self._sats)

    def _parse_nmea(self, sentence: str):
        """Parse a single NMEA sentence (used by feed() for local testing)."""
        if not verify_nmea_checksum(sentence):
            return
        body, _, _ = sentence[1:].partition("*")
        fields = body.split(",")
        if not fields:
            return

        talker_type = fields[0]
        self._last_update = time.monotonic()

        # $GPGGA / $GNGGA: Global Positioning System Fix Data
        if talker_type.endswith("GGA") and len(fields) >= 8:
            fix_qual = fields[6]
            try:
                self._sats = int(fields[7]) if fields[7] else self._sats
            except ValueError:
                pass

            if fix_qual and fix_qual != "0":
                lat = parse_nmea_degree(fields[2], fields[3])
                lon = parse_nmea_degree(fields[4], fields[5])
                if lat is not None and lon is not None:
                    self._lat = lat
                    self._lon = lon
                    self._has_fix = True
            else:
                self._has_fix = False

        # $GPRMC / $GNRMC: Recommended Minimum Specific GNSS Data
        elif talker_type.endswith("RMC") and len(fields) >= 7:
            status = fields[2]  # 'A' = Active/Valid, 'V' = Void
            if status == "A":
                lat = parse_nmea_degree(fields[3], fields[4])
                lon = parse_nmea_degree(fields[5], fields[6])
                if lat is not None and lon is not None:
                    self._lat = lat
                    self._lon = lon
                    self._has_fix = True
            elif status == "V":
                self._has_fix = False

        # $GPGSV / $GNGSV: Satellites in View
        elif talker_type.endswith("GSV") and len(fields) >= 4:
            if not self._has_fix:
                try:
                    sats_in_view = int(fields[3])
                    if sats_in_view > 0:
                        self._sats = max(self._sats, sats_in_view)
                except ValueError:
                    pass

    def _emit_status(self):
        """Periodic status emission (1 Hz)."""
        now = time.monotonic()
        SILENCE_TIMEOUT_S = 3.5
        if self._last_update == 0.0 or (now - self._last_update) > SILENCE_TIMEOUT_S:
            if self._state != "none":
                self._state = "none"
                self._has_fix = False
                self.gps_changed.emit("none", 0.0, 0.0, 0)
            return

        if self._has_fix:
            self._state = "fix"
            self.gps_changed.emit("fix", self._lat, self._lon, self._sats)
        else:
            self._state = "searching"
            self.gps_changed.emit("searching", 0.0, 0.0, self._sats)


# ==============================================================================
# 4. DUAL BTS7960 MOTOR DRIVER CONTROLLER (VIA ESP32 SERIAL BRIDGE)
# ==============================================================================

class ESP32BTS7960Motors(QObject):
    """
    Differential drive controller for the two BTS7960 Dual Motor Drivers
    and two MY1016Z2 DC Motors, commanded through the ESP32 Serial Bridge.

    The ESP32 handles the actual GPIO PWM output:
      - Left BTS7960:  R_EN=GPIO25, L_EN=GPIO26, RPWM=GPIO32, LPWM=GPIO33
      - Right BTS7960: R_EN=GPIO27, L_EN=GPIO14, RPWM=GPIO18, LPWM=GPIO19
      - LEDC 1 kHz, 10-bit resolution

    This class sends signed duty cycle commands over serial JSON and emits
    the motor_state signal for UI updates.
    """
    motor_state = Signal(str, float, bool)  # command, speed_mps, hardware_ready

    MAX_SPEED_MPS = 0.9  # Nominal MY1016Z2 rover ground speed at 100% duty

    def __init__(self, parent=None):
        super().__init__(parent)
        self._bridge = get_bridge()
        self._current_cmd = "stop"
        self._current_speed = 0.0

    @property
    def gpio_ready(self) -> bool:
        """True when the ESP32 serial bridge is connected (motors can be commanded)."""
        return self._bridge.connected or self._bridge.simulating

    def link_status(self) -> dict:
        """Serial link health plus left/right BTS7960 driver status for the web UI."""
        return self._bridge.link_snapshot()

    def execute(self, command: str):
        """Execute a rover movement command ('forward', 'backward', 'left', 'right', 'stop')."""
        cmd = (command or "stop").lower().strip()
        if cmd == self._current_cmd and cmd != "stop":
            return  # Already executing this motion; do not disturb motors or spam UART

        d_drive = PINS.drive_duty
        d_turn = PINS.turn_duty

        if cmd == "forward":
            self._bridge.send_motor(d_drive, d_drive)
            speed = round(d_drive * self.MAX_SPEED_MPS, 1)
        elif cmd == "backward":
            self._bridge.send_motor(-d_drive, -d_drive)
            speed = round(d_drive * self.MAX_SPEED_MPS, 1)
        elif cmd == "left":
            self._bridge.send_motor(-d_turn, d_turn)
            speed = round(d_turn * self.MAX_SPEED_MPS, 1)
        elif cmd == "right":
            self._bridge.send_motor(d_turn, -d_turn)
            speed = round(d_turn * self.MAX_SPEED_MPS, 1)
        else:
            cmd = "stop"
            self._bridge.send_stop()
            speed = 0.0

        self._current_cmd = cmd
        self._current_speed = speed
        self.motor_state.emit(cmd, speed, self.gpio_ready)

        # Dispatch command to ESP32 AP over Wi-Fi
        try:
            threading.Thread(
                target=lambda: urllib.request.urlopen(
                    f"http://192.168.4.1/cmd?act={cmd}&spd={d_drive}",
                    timeout=0.25,
                ),
                daemon=True,
            ).start()
        except Exception:
            pass

    def stop(self):
        """Emergency stop — halt both motors and send stop command to ESP32."""
        self._bridge.send_stop()
        self._current_cmd = "stop"
        self._current_speed = 0.0
        self.motor_state.emit("stop", 0.0, self.gpio_ready)

        # Dispatch stop command to ESP32 AP over Wi-Fi
        try:
            threading.Thread(
                target=lambda: urllib.request.urlopen(
                    "http://192.168.4.1/cmd?act=stop",
                    timeout=0.25,
                ),
                daemon=True,
            ).start()
        except Exception:
            pass


# ==============================================================================
# 5. LIVE CAMERA PROVIDER (USB WEBCAM ON HOST PC VIA OPENCV)
# ==============================================================================

class ESP32Camera(QObject):
    """
    Live camera provider for RiceTrack supporting:
      1. Raspberry Pi Camera Module v2 (CSI ribbon via Picamera2 / libcamera)
      2. Standard USB Webcams via OpenCV VideoCapture (V4L2 / DirectShow)

    Automatically selects Picamera2 on Raspberry Pi when a CSI camera is attached,
    and falls back to OpenCV VideoCapture for USB webcams or desktop simulation.
    Captures frames in a background thread at ~25 FPS for smooth monitoring.
    """
    frame_ready = Signal(object)      # BGR np.ndarray live frame
    camera_state = Signal(bool, str)  # (connected, description)

    def __init__(self, camera_index: int = 0, parent=None):
        super().__init__(parent)
        self._camera_index = camera_index
        self._cv_cap: Any = None
        self._picam2: Any = None
        self._is_picam2 = False
        self._connected = False
        self._paused = False
        self._running = False
        self._reopen_requested = False
        self._last_frame_bgr: np.ndarray | None = None
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def camera_index(self) -> int:
        return self._camera_index

    @property
    def last_frame(self) -> np.ndarray | None:
        with self._lock:
            return None if self._last_frame_bgr is None else self._last_frame_bgr.copy()

    def start(self):
        """Start the background camera capture thread."""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def stop(self):
        """Stop the camera capture thread and release the device."""
        self._running = False
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.5)
        self._thread = None
        self._release()

    def set_paused(self, paused: bool):
        """Pause live preview updates while inspecting a captured/uploaded still image."""
        self._paused = paused

    def set_camera_index(self, index: int):
        """Switch to a different camera device index (e.g. 0, 1, 2)."""
        if index != self._camera_index:
            self._camera_index = index
        self._reopen_requested = True

    def reconnect(self):
        """Force re-probing the active camera."""
        self._reopen_requested = True

    def _try_connect(self) -> bool:
        """Attempt to open either Raspberry Pi Camera Module v2 via Picamera2 or a USB webcam via OpenCV."""
        if self._connected and not self._reopen_requested:
            return True
        self._reopen_requested = False
        self._release(emit_offline=False)

        idx = self._camera_index

        # 1. On Raspberry Pi with CSI port, try Picamera2 first (Raspberry Pi Camera Module v2)
        if idx == 0:
            if HAS_PICAMERA2:
                picam = None
                try:
                    picam = Picamera2()
                    cfg = picam.create_video_configuration(main={"size": (1280, 720), "format": "BGR888"})
                    picam.configure(cfg)
                    picam.start()
                    time.sleep(0.1)  # Warm up sensor auto-exposure
                    frame = picam.capture_array()
                    if frame is not None and frame.size > 0:
                        self._picam2 = picam
                        self._is_picam2 = True
                        with self._lock:
                            self._last_frame_bgr = frame
                        h, w = frame.shape[:2]
                        self._set_state(True, f"Pi Camera v2 CSI ({w}×{h})")
                        self.frame_ready.emit(frame)
                        return True
                except Exception as e:
                    print(f"[RiceTrack Camera] Picamera2 connection failed: {e}")
                    if picam is not None:
                        try:
                            picam.stop()
                            picam.close()
                        except Exception:
                            pass
                    self._picam2 = None
                    self._is_picam2 = False
            elif _PICAM_IMPORT_ERR:
                # Picamera2 couldn't be imported in current Python environment
                pass

        # 2. Fall back to standard OpenCV VideoCapture (for USB Webcams or PC development)
        backends = [cv2.CAP_ANY]
        if os.name == "nt" and hasattr(cv2, "CAP_DSHOW"):
            backends = [cv2.CAP_DSHOW, cv2.CAP_ANY]
        elif os.name == "posix" and hasattr(cv2, "CAP_V4L2"):
            backends = [cv2.CAP_V4L2, cv2.CAP_ANY]

        for backend in backends:
            if not self._running:
                return False
            try:
                cap = cv2.VideoCapture(idx, backend)
                if cap is not None and cap.isOpened():
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
                    ok, frame = cap.read()
                    if ok and frame is not None and frame.size > 0:
                        self._cv_cap = cap
                        with self._lock:
                            self._last_frame_bgr = frame
                        h, w = frame.shape[:2]
                        self._set_state(True, f"Camera {idx} ({w}×{h})")
                        self.frame_ready.emit(frame)
                        return True
                if cap is not None:
                    cap.release()
            except Exception:
                pass

        self._set_state(False, f"No Camera (Cam {idx})")
        return False

    def _release(self, emit_offline: bool = True):
        """Release the camera device (Picamera2 or OpenCV)."""
        if self._is_picam2 and self._picam2 is not None:
            try:
                self._picam2.stop()
                self._picam2.close()
            except Exception:
                pass
            self._picam2 = None
            self._is_picam2 = False

        if self._cv_cap is not None:
            try:
                self._cv_cap.release()
            except Exception:
                pass
            self._cv_cap = None
        if emit_offline:
            self._set_state(False, f"No Camera (Cam {self._camera_index})")

    def _set_state(self, connected: bool, label: str):
        self._connected = connected
        self.camera_state.emit(connected, label)

    def _capture_loop(self):
        """Background thread: connect → grab frames → reconnect on failure."""
        while self._running:
            if self._reopen_requested or not self._connected:
                connected = self._try_connect()
                if not connected:
                    for _ in range(25):
                        if not self._running or self._reopen_requested:
                            break
                        time.sleep(0.1)
                    continue

            if self._paused:
                time.sleep(0.05)
                continue

            frame = self._grab_frame()
            if frame is None:
                self._release(emit_offline=True)
                time.sleep(0.5)
                continue

            with self._lock:
                self._last_frame_bgr = frame
            self.frame_ready.emit(frame)
            time.sleep(0.04)  # ~25 FPS live monitoring

    def _grab_frame(self) -> np.ndarray | None:
        """Grab a single frame from Picamera2 or OpenCV VideoCapture."""
        if self._is_picam2 and self._picam2 is not None:
            try:
                frame = self._picam2.capture_array()
                if frame is not None and frame.size > 0:
                    return frame
            except Exception:
                return None
        elif self._cv_cap is not None:
            try:
                ok, frame = self._cv_cap.read()
                if ok and frame is not None and frame.size > 0:
                    return frame
            except Exception:
                return None
        return None

    def capture_still(self) -> np.ndarray | None:
        """Return the latest BGR frame from the live camera stream."""
        with self._lock:
            if self._last_frame_bgr is not None:
                return self._last_frame_bgr.copy()
        frame = self._grab_frame()
        if frame is not None:
            with self._lock:
                self._last_frame_bgr = frame
            return frame.copy()
        return None


LiveCameraProvider = ESP32Camera


# ==============================================================================
# 6. UNIFIED ESP32-WROOM-32 HARDWARE CONTROLLER
# ==============================================================================

class ESP32HardwareController(QObject):
    """
    Complete ESP32-WROOM-32 hardware hub combining:
      - MPU-6050 IMU via ESP32 I2C (SDA GPIO21, SCL GPIO22)
      - NEO-6M GPS via ESP32 UART2 (TX GPIO17, RX GPIO16)
      - Dual BTS7960 Motor Drivers via ESP32 LEDC PWM (GPIOs 25/26/32/33 & 27/14/18/19)
      - USB Webcam on Host PC via OpenCV
      - ESP32 Serial Bridge (USB 115200 baud, newline-delimited JSON)
      - Autonomous Train Drive Engine (Every 5s drive cycle + Auto-Drive Reminder)
    """
    imu_reading = Signal(float, float)             # roll, pitch (deg)
    imu_state = Signal(bool)                       # True = MPU-6050 active via ESP32
    gps_changed = Signal(str, float, float, int)   # state, lat, lon, sats
    motor_state = Signal(str, float, bool)         # cmd, speed_mps, bridge_ready
    camera_frame = Signal(object)                  # BGR ndarray from USB webcam
    camera_state = Signal(bool, str)               # (connected, label)
    auto_drive_state = Signal(bool, str, int, int) # (active, reminder_text, cycle, seconds_left)
    auto_capture_requested = Signal()              # emitted on 5s station stop in stop_capture mode
    auto_cycle_completed = Signal(int, str, str, int)  # (cycle_num, mode, direction, interval_s)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._bridge = get_bridge()
        self.imu = ESP32MPU6050(parent=self)
        self.gps = ESP32NEO6MGPS(parent=self)
        self.motors = ESP32BTS7960Motors(parent=self)
        self.camera = ESP32Camera(parent=self)

        # Autonomous train drive state
        self._auto_active = False
        self._auto_interval_s = 5
        self._auto_countdown_s = 5
        self._auto_cycle = 0
        self._auto_mode = "train_pulse"  # "train_pulse" | "stop_capture"
        self._auto_direction = "forward"
        self._auto_pausing = False

        self._auto_timer = QTimer(self)
        self._auto_timer.setInterval(1000)
        self._auto_timer.timeout.connect(self._on_auto_tick)

        # Wire sub-component signals to unified hub signals
        self.imu.imu_reading.connect(self.imu_reading)
        self.imu.imu_state.connect(self.imu_state)
        self.gps.gps_changed.connect(self.gps_changed)
        self.motors.motor_state.connect(self.motor_state)
        self.camera.frame_ready.connect(self.camera_frame)
        self.camera.camera_state.connect(self.camera_state)

    @property
    def auto_drive_active(self) -> bool:
        return self._auto_active

    @property
    def auto_drive_direction(self) -> str:
        return self._auto_direction

    def start(self):
        """Start all hardware subsystems (ESP32 bridge, IMU, GPS, camera)."""
        self._bridge.start()
        self.imu.start()
        self.gps.start()
        self.camera.start()

    def stop(self):
        """Stop all hardware subsystems and halt motors."""
        self.stop_auto_train_drive()
        self.motors.stop()
        self.camera.stop()
        self.imu.stop()
        self.gps.stop()
        self._bridge.stop()

    def start_auto_train_drive(
        self,
        interval_s: int = 5,
        mode: str = "train_pulse",
        direction: str = "forward",
    ):
        """Start autonomous Train Drive that runs/pulses every `interval_s` seconds (default 5s)."""
        self._auto_interval_s = max(1, int(interval_s))
        self._auto_mode = mode if mode in ("train_pulse", "stop_capture") else "train_pulse"
        self._auto_direction = (
            direction if direction in ("forward", "backward", "left", "right") else "forward"
        )
        self._auto_active = True
        self._auto_pausing = False
        self._auto_cycle = 1
        self._auto_countdown_s = self._auto_interval_s

        self.motors.execute(self._auto_direction)
        self._emit_auto_reminder(pulsed=True)
        self.auto_cycle_completed.emit(
            self._auto_cycle, self._auto_mode, self._auto_direction, self._auto_interval_s
        )
        self._auto_timer.start()

    def set_auto_drive_direction(self, direction: str):
        """Steer or change the active direction during Auto Train Drive."""
        cmd = (direction or "forward").lower().strip()
        if cmd not in ("forward", "backward", "left", "right"):
            return
        self._auto_direction = cmd
        if self._auto_active:
            self._auto_pausing = False
            self._auto_countdown_s = self._auto_interval_s
            self.motors.execute(self._auto_direction)
            self._emit_auto_reminder(pulsed=True)
            self.auto_cycle_completed.emit(
                self._auto_cycle, self._auto_mode, self._auto_direction, self._auto_interval_s
            )
            self._auto_timer.start()

    def stop_auto_train_drive(self, reason: str = ""):
        """Stop autonomous Train Drive and halt the motors."""
        was_active = self._auto_active
        self._auto_timer.stop()
        self._auto_active = False
        self._auto_pausing = False
        if was_active:
            self.motors.execute("stop")
            msg = reason or "Auto Train Drive disengaged — Manual control active"
            self.auto_drive_state.emit(False, msg, self._auto_cycle, 0)

    def set_auto_drive_interval(self, interval_s: int):
        """Change the auto-drive cycle interval (seconds)."""
        self._auto_interval_s = max(1, int(interval_s))
        if self._auto_active:
            self._auto_countdown_s = min(self._auto_countdown_s, self._auto_interval_s)
            self._emit_auto_reminder(pulsed=False)

    def set_auto_drive_mode(self, mode: str):
        """Switch auto-drive mode ('train_pulse' or 'stop_capture')."""
        if mode in ("train_pulse", "stop_capture"):
            self._auto_mode = mode
            if self._auto_active:
                self._auto_pausing = False
                self.motors.execute(self._auto_direction)
                self._emit_auto_reminder(pulsed=False)

    def _on_auto_tick(self):
        """1-second tick handler for autonomous Train Drive engine."""
        if not self._auto_active:
            return

        if self._auto_pausing:
            # Finished brief station stop after interval run → start next drive cycle
            self._auto_pausing = False
            self._auto_cycle += 1
            self._auto_countdown_s = self._auto_interval_s
            self.motors.execute(self._auto_direction)
            self._emit_auto_reminder(pulsed=True)
            return

        self._auto_countdown_s -= 1
        if self._auto_countdown_s <= 0:
            if self._auto_mode == "stop_capture":
                # Station stop, trigger auto-capture & path save, then resume on next tick
                self._auto_pausing = True
                self.motors.execute("stop")
                msg = f"Station Stop · Cycle #{self._auto_cycle} — Sampling & saving GPS path…"
                self.auto_drive_state.emit(True, msg, self._auto_cycle, 0)
                self.auto_capture_requested.emit()
                self.auto_cycle_completed.emit(
                    self._auto_cycle, self._auto_mode, self._auto_direction, self._auto_interval_s
                )
            else:
                # Re-command Train Drive, emit Auto-Drive reminder pulse
                self._auto_cycle += 1
                self._auto_countdown_s = self._auto_interval_s
                self.motors.execute(self._auto_direction)
                self._emit_auto_reminder(pulsed=True)
                self.auto_cycle_completed.emit(
                    self._auto_cycle, self._auto_mode, self._auto_direction, self._auto_interval_s
                )
        else:
            self._emit_auto_reminder(pulsed=False)

    def _emit_auto_reminder(self, pulsed: bool):
        """Emit auto_drive_state with a human-readable reminder string."""
        mode_lbl = "Stop-&-Sample" if self._auto_mode == "stop_capture" else "Train Run"
        prefix = "⚡ AUTO PULSE" if pulsed else "● AUTO ACTIVE"
        msg = (
            f"{prefix} ({mode_lbl}) · {self._auto_direction.upper()} "
            f"· Cycle #{self._auto_cycle} · Next in {self._auto_countdown_s}s"
        )
        self.auto_drive_state.emit(True, msg, self._auto_cycle, self._auto_countdown_s)

    def execute_motor_command(self, cmd: str):
        """Execute a motor command, respecting auto-drive state."""
        norm_cmd = (cmd or "stop").lower().strip()
        if self._auto_active:
            if norm_cmd in ("forward", "backward", "left", "right"):
                self.set_auto_drive_direction(norm_cmd)
                return
            self.stop_auto_train_drive("Stopped by operator — Auto Train Drive OFF")
        self.motors.execute(norm_cmd)

    def set_camera_index(self, index: int):
        """Switch to a different camera device index."""
        self.camera.set_camera_index(index)

    def reconnect_camera(self):
        """Force re-probing the active camera."""
        self.camera.reconnect()

    def capture_csi_frame(self) -> np.ndarray | None:
        """Capture a still frame (method name kept for API compatibility)."""
        return self.camera.capture_still()
