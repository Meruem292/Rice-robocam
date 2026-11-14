# ==============================================================================
# RiceTrack Rover — ESP32 MicroPython Firmware (main.py)
# ==============================================================================
# Target MCU : ESP32-WROOM-32 DevKit V1
# Operating  : MicroPython v1.20+ (240 MHz)
# Interfaces :
#   - USB Serial REPL (UART0, 115200 baud) — Host PC bridge (newline JSON)
#   - I2C0 (GPIO21 SDA, GPIO22 SCL @ 400kHz) — MPU-6050 6-DOF IMU
#   - UART2 (GPIO17 TX, GPIO16 RX @ 9600 baud) — NEO-6M GPS Module
#   - Dual BTS7960 43A Motor Drivers — Differential Traction Drive
#       * Left:  R_EN=Pin(25), L_EN=Pin(26), RPWM=Pin(32), LPWM=Pin(33)
#       * Right: R_EN=Pin(27), L_EN=Pin(14), RPWM=Pin(18), LPWM=Pin(19)
#   - Status LED: Pin(2) — Heartbeat toggle (500 ms)
# ==============================================================================

import math
import struct
import sys
import time

try:
    import ujson
except ImportError:
    import json as ujson

try:
    import uselect as select
except ImportError:
    try:
        import select
    except ImportError:
        select = None

# Hardware import with CPython testing mock fallback
try:
    import machine
    from machine import I2C, PWM, UART, Pin
except ImportError:
    # Mocks for testing/linting in host CPython environments
    class MockPin:
        OUT = 1
        IN = 0
        def __init__(self, pin, mode=None, value=0):
            self.pin = pin
            self._val = value
        def value(self, v=None):
            if v is not None:
                self._val = v
            return self._val
    Pin = MockPin

    class MockPWM:
        def __init__(self, pin, freq=1000, duty=0):
            self.pin = pin
            self._f = freq
            self._d = duty
        def duty(self, d=None):
            if d is not None:
                self._d = d
            return self._d
        def freq(self, f=None):
            if f is not None:
                self._f = f
            return self._f
    PWM = MockPWM

    class MockI2C:
        def __init__(self, id=0, sda=None, scl=None, freq=400000):
            pass
        def writeto_mem(self, addr, memaddr, data):
            pass
        def readfrom_mem(self, addr, memaddr, nbytes):
            return b'\x00' * nbytes
    I2C = MockI2C

    class MockUART:
        def __init__(self, id=2, baudrate=9600, tx=17, rx=16):
            pass
        def any(self):
            return 0
        def read(self, n=None):
            return b''
        def write(self, buf):
            return len(buf)
    UART = MockUART

    class MockMachine:
        Pin = MockPin
        PWM = MockPWM
        I2C = MockI2C
        UART = MockUART
        @staticmethod
        def freq(f=None):
            return 240000000
    machine = MockMachine()

# Provide time.ticks_* fallbacks if running under CPython for validation
if not hasattr(time, 'ticks_ms'):
    time.ticks_ms = lambda: int(time.time() * 1000)
if not hasattr(time, 'ticks_us'):
    time.ticks_us = lambda: int(time.time() * 1000000)
if not hasattr(time, 'ticks_diff'):
    time.ticks_diff = lambda a, b: a - b
if not hasattr(time, 'sleep_ms'):
    time.sleep_ms = lambda ms: time.sleep(ms / 1000.0)


# ==============================================================================
# PIN SETUP & PERIPHERAL INITIALIZATION
# ==============================================================================

# Built-in heartbeat LED
led = Pin(2, Pin.OUT)

# I2C bus for MPU-6050 6-DOF IMU
i2c = I2C(0, sda=Pin(21), scl=Pin(22), freq=400000)

# UART2 for NEO-6M GPS receiver
gps_uart = UART(2, baudrate=9600, tx=17, rx=16)

# Left BTS7960 Motor Driver
l_r_en = Pin(25, Pin.OUT, value=1)
l_l_en = Pin(26, Pin.OUT, value=1)
l_rpwm = PWM(Pin(32), freq=1000, duty=0)
l_lpwm = PWM(Pin(33), freq=1000, duty=0)

# Right BTS7960 Motor Driver
r_r_en = Pin(27, Pin.OUT, value=1)
r_l_en = Pin(14, Pin.OUT, value=1)
r_rpwm = PWM(Pin(18), freq=1000, duty=0)
r_lpwm = PWM(Pin(19), freq=1000, duty=0)


# ==============================================================================
# GLOBAL RUNTIME STATE
# ==============================================================================

imu_ok = False
imu_roll = 0.0
imu_pitch = 0.0
imu_last_us = 0

gps_buf = ""
gps_lat = 0.0
gps_lon = 0.0
gps_sats = 0
gps_state = "none"

# Motor driver + host link monitoring state
boot_ms = time.ticks_ms()
last_host_msg_ms = None
drivers_ready = False
left_driver = {"enabled": False, "duty": 0.0, "direction": "stop"}
right_driver = {"enabled": False, "duty": 0.0, "direction": "stop"}


# ==============================================================================
# 1. IMU MPU-6050 FUNCTIONS
# ==============================================================================

def mpu6050_init():
    """Write 0x00 to register 0x6B (PWR_MGMT_1) at addr 0x68 to wake sensor."""
    global imu_ok
    try:
        i2c.writeto_mem(0x68, 0x6B, b'\x00')
        time.sleep_ms(50)
        imu_ok = True
        return True
    except Exception:
        imu_ok = False
        return False


def mpu6050_read():
    """Read 14 bytes from register 0x3B, unpack as big-endian signed shorts,
    compute roll/pitch using complementary filter:
      ACCEL_SCALE=16384.0, GYRO_SCALE=131.0, alpha=0.2.
    Return (roll, pitch, ok_bool)."""
    global imu_ok, imu_roll, imu_pitch, imu_last_us

    if not imu_ok:
        if not mpu6050_init():
            return (round(imu_roll, 2), round(imu_pitch, 2), False)

    try:
        raw = i2c.readfrom_mem(0x68, 0x3B, 14)
    except Exception:
        imu_ok = False
        return (round(imu_roll, 2), round(imu_pitch, 2), False)

    if not raw or len(raw) < 14:
        return (round(imu_roll, 2), round(imu_pitch, 2), False)

    # 14 bytes: ax, ay, az, temp, gx, gy, gz
    ax, ay, az, _, gx, gy, gz = struct.unpack('>hhhhhhh', raw)

    # Scale to physical units
    ax_g = ax / 16384.0
    ay_g = ay / 16384.0
    az_g = az / 16384.0
    gx_ds = gx / 131.0
    gy_ds = gy / 131.0

    # Accelerometer roll and pitch (degrees)
    accel_roll = math.atan2(ay_g, az_g) * 57.2957795
    denom = math.sqrt(ay_g * ay_g + az_g * az_g)
    accel_pitch = math.atan2(-ax_g, denom) * 57.2957795 if denom != 0 else 0.0

    now_us = time.ticks_us()
    if imu_last_us == 0:
        imu_roll = accel_roll
        imu_pitch = accel_pitch
        imu_last_us = now_us
    else:
        dt = time.ticks_diff(now_us, imu_last_us) / 1000000.0
        imu_last_us = now_us
        if dt <= 0 or dt > 0.5:
            dt = 0.05  # Default 20 Hz step if timing glitch occurs

        alpha = 0.2
        # Complementary filter fusion
        imu_roll = (1.0 - alpha) * (imu_roll + gx_ds * dt) + alpha * accel_roll
        imu_pitch = (1.0 - alpha) * (imu_pitch + gy_ds * dt) + alpha * accel_pitch

    return (round(imu_roll, 2), round(imu_pitch, 2), True)


# ==============================================================================
# 2. NEO-6M GPS & NMEA FUNCTIONS
# ==============================================================================

def parse_nmea_degree(raw_val, hemisphere):
    """Convert NMEA ddmm.mmmm to decimal degrees, negate for S/W."""
    if not raw_val:
        return 0.0
    try:
        raw_str = str(raw_val).strip()
        dot_idx = raw_str.find('.')
        if dot_idx < 2:
            return 0.0
        deg = float(raw_str[:dot_idx - 2])
        minutes = float(raw_str[dot_idx - 2:])
        dec = deg + (minutes / 60.0)
        if hemisphere in ('S', 's', 'W', 'w'):
            dec = -dec
        return round(dec, 6)
    except Exception:
        return 0.0


def verify_nmea_checksum(sentence):
    """XOR checksum validation for NMEA sentence between '$' and '*'."""
    if not sentence or '*' not in sentence:
        return False
    try:
        if sentence.startswith('$'):
            sentence = sentence[1:]
        body, chk = sentence.rsplit('*', 1)
        calc = 0
        for char in body:
            calc ^= ord(char)
        expected = int(chk[:2], 16)
        return calc == expected
    except Exception:
        return False


def read_gps():
    """Read available bytes from gps_uart, buffer lines, parse $GPGGA/$GPRMC.
    Return (state_str, lat, lon, sats)."""
    global gps_buf, gps_lat, gps_lon, gps_sats, gps_state
    try:
        avail = gps_uart.any()
        if avail:
            chunk = gps_uart.read(avail)
            if chunk:
                gps_buf += chunk.decode('ascii', 'ignore')
    except Exception:
        pass

    while '\n' in gps_buf:
        line, gps_buf = gps_buf.split('\n', 1)
        line = line.strip('\r \t')
        if not line.startswith('$'):
            continue
        if not verify_nmea_checksum(line):
            continue

        parts = line.split('*')[0].split(',')
        talker = parts[0]

        if talker in ('$GPGGA', '$GNGGA') and len(parts) >= 8:
            try:
                quality = int(parts[6]) if parts[6] else 0
                sats = int(parts[7]) if parts[7] else 0
                gps_sats = sats
                if quality > 0:
                    gps_state = "fix"
                    if parts[2] and parts[3]:
                        gps_lat = parse_nmea_degree(parts[2], parts[3])
                    if parts[4] and parts[5]:
                        gps_lon = parse_nmea_degree(parts[4], parts[5])
                else:
                    gps_state = "none"
            except Exception:
                pass

        elif talker in ('$GPRMC', '$GNRMC') and len(parts) >= 7:
            try:
                status = parts[2] if len(parts) > 2 else 'V'
                if status == 'A':
                    gps_state = "fix"
                    if parts[3] and parts[4]:
                        gps_lat = parse_nmea_degree(parts[3], parts[4])
                    if parts[5] and parts[6]:
                        gps_lon = parse_nmea_degree(parts[5], parts[6])
                else:
                    gps_state = "none"
            except Exception:
                pass

    if len(gps_buf) > 1024:
        gps_buf = gps_buf[-512:]

    return (gps_state, gps_lat, gps_lon, gps_sats)


# ==============================================================================
# 3. MOTOR DRIVER FUNCTIONS (BTS7960)
# ==============================================================================

def _update_driver_state(state, duty, ready=True):
    """Record the commanded state of one BTS7960 driver for telemetry."""
    duty = max(-1.0, min(1.0, float(duty)))
    state["enabled"] = ready and abs(duty) > 0.001
    state["duty"] = round(duty, 3)
    state["direction"] = "forward" if duty > 0.001 else "reverse" if duty < -0.001 else "stop"


def set_motors(left_duty, right_duty):
    """Accept signed floats -1.0..1.0.
    Positive = RPWM active (forward), negative = LPWM active (reverse).
    Scale to 10-bit PWM duty (0-1023). Enable pins HIGH."""
    global drivers_ready
    left_duty = max(-1.0, min(1.0, float(left_duty)))
    right_duty = max(-1.0, min(1.0, float(right_duty)))

    # Ensure H-bridge driver enable inputs are active
    l_r_en.value(1)
    l_l_en.value(1)
    r_r_en.value(1)
    r_l_en.value(1)
    drivers_ready = True

    # Left motor PWM
    l_val = int(abs(left_duty) * 1023)
    if left_duty >= 0:
        l_rpwm.duty(l_val)
        l_lpwm.duty(0)
    else:
        l_rpwm.duty(0)
        l_lpwm.duty(l_val)

    # Right motor PWM
    r_val = int(abs(right_duty) * 1023)
    if right_duty >= 0:
        r_rpwm.duty(r_val)
        r_lpwm.duty(0)
    else:
        r_rpwm.duty(0)
        r_lpwm.duty(r_val)

    _update_driver_state(left_driver, left_duty)
    _update_driver_state(right_driver, right_duty)


def stop_motors():
    """Set all PWM duty to 0 and disable motor driver enables."""
    global drivers_ready
    l_rpwm.duty(0)
    l_lpwm.duty(0)
    r_rpwm.duty(0)
    r_lpwm.duty(0)
    l_r_en.value(0)
    l_l_en.value(0)
    r_r_en.value(0)
    r_l_en.value(0)
    drivers_ready = True
    _update_driver_state(left_driver, 0.0, ready=False)
    _update_driver_state(right_driver, 0.0, ready=False)


def motor_status_message(msg_type="s"):
    """Build a link/driver status packet for the host web app.

    `host` tells the ESP32's view of the host link: 1 when a host command was
    received within the last 5 s. `l`/`r` report each BTS7960 driver state.
    """
    now = time.ticks_ms()
    host_age_ms = -1
    host_ok = 0
    if last_host_msg_ms is not None:
        host_age_ms = time.ticks_diff(now, last_host_msg_ms)
        host_ok = 1 if host_age_ms <= 5000 else 0

    return {
        "t": msg_type,
        "l": {
            "c": 1 if drivers_ready else 0,
            "e": 1 if left_driver["enabled"] else 0,
            "d": left_driver["duty"],
            "dir": left_driver["direction"],
        },
        "r": {
            "c": 1 if drivers_ready else 0,
            "e": 1 if right_driver["enabled"] else 0,
            "d": right_driver["duty"],
            "dir": right_driver["direction"],
        },
        "host": host_ok,
        "ha": host_age_ms,
        "up": time.ticks_diff(now, boot_ms) // 1000,
    }


# ==============================================================================
# 4. SERIAL COMMAND DISPATCH & PROTOCOL
# ==============================================================================

# Non-blocking stdin poller
_serial_poll = None
try:
    if select and hasattr(select, 'poll'):
        _serial_poll = select.poll()
        _serial_poll.register(sys.stdin, select.POLLIN)
except Exception:
    _serial_poll = None


def read_serial_line():
    """Non-blocking read of one line from sys.stdin."""
    global _serial_poll
    try:
        if _serial_poll is not None:
            if _serial_poll.poll(0):
                return sys.stdin.readline()
        elif select and hasattr(select, 'select'):
            r, _, _ = select.select([sys.stdin], [], [], 0)
            if r:
                return sys.stdin.readline()
    except Exception:
        pass
    return None


def process_command(line):
    """Parse JSON command from host, dispatch to motors/ping.
    Returns True if valid motor or stop command processed (resets watchdog)."""
    global last_host_msg_ms
    if not line:
        return False
    try:
        if isinstance(line, bytes):
            line = line.decode('ascii', 'ignore')
        line = line.strip()
        if not line:
            return False

        cmd = ujson.loads(line)
        c = cmd.get("c")

        if c in ("m", "s", "p"):
            last_host_msg_ms = time.ticks_ms()

        if c == "m":
            # Motor speed command: {"c":"m","l":0.65,"r":0.65}
            l = float(cmd.get("l", 0.0))
            r = float(cmd.get("r", 0.0))
            set_motors(l, r)
            return True
        elif c == "s":
            # Emergency stop command: {"c":"s"}
            stop_motors()
            return True
        elif c == "p":
            # Heartbeat ping command: {"c":"p"} -> pong + link/driver status
            print(ujson.dumps(motor_status_message("k")))
            return False
    except Exception:
        pass
    return False


# ==============================================================================
# 5. OBJECT-ORIENTED WRAPPERS (FOR MODULAR INTEGRATION)
# ==============================================================================

class IMUReader:
    """Object wrapper for MPU-6050 complementary-filter IMU."""
    def __init__(self, i2c_bus=None):
        mpu6050_init()
    def read(self):
        return mpu6050_read()

class GPSReader:
    """Object wrapper for NEO-6M UART NMEA GPS receiver."""
    def __init__(self, uart_bus=None):
        pass
    def poll(self):
        return read_gps()
    @property
    def fix(self):
        return gps_state
    @property
    def lat(self):
        return gps_lat
    @property
    def lon(self):
        return gps_lon
    @property
    def sats(self):
        return gps_sats

class MotorDriver:
    """Object wrapper for dual BTS7960 motor driver H-bridges."""
    def drive(self, left_duty, right_duty):
        set_motors(left_duty, right_duty)
    def stop(self):
        stop_motors()


# ==============================================================================
# 6. MAIN FIRMWARE EVENT LOOP
# ==============================================================================

def main():
    """Main loop: read serial commands (non-blocking via sys.stdin),
    run IMU at 20Hz, GPS at 1Hz, LED heartbeat every 500ms,
    motor watchdog (auto-stop if no command for 2s).
    Use time.ticks_ms() for timing."""

    # Initialize subsystems
    mpu6050_init()
    stop_motors()

    now = time.ticks_ms()
    last_imu_ms = now
    last_gps_ms = now
    last_led_ms = now
    last_cmd_ms = now
    last_status_ms = now
    led_state = False

    while True:
        now = time.ticks_ms()

        # --- 1. Serial command ingest (non-blocking) ---
        line = read_serial_line()
        if line:
            if process_command(line):
                last_cmd_ms = now

        # --- 2. Motor watchdog (auto-stop after 2s without command) ---
        if time.ticks_diff(now, last_cmd_ms) > 2000:
            stop_motors()
            last_cmd_ms = now  # Reset so stop_motors() is not continuously called

        # --- 3. IMU telemetry @ 20 Hz (50 ms interval) ---
        if time.ticks_diff(now, last_imu_ms) >= 50:
            last_imu_ms = now
            roll, pitch, ok = mpu6050_read()
            msg = {
                "t": "i",
                "r": roll,
                "p": pitch,
                "ok": 1 if ok else 0
            }
            print(ujson.dumps(msg))

        # --- 4. GPS read & telemetry @ 1 Hz (1000 ms interval) ---
        state, lat, lon, sats = read_gps()
        if time.ticks_diff(now, last_gps_ms) >= 1000:
            last_gps_ms = now
            msg = {
                "t": "g",
                "s": state,
                "la": lat,
                "lo": lon,
                "n": sats
            }
            print(ujson.dumps(msg))

        # --- 5. Link / motor driver status telemetry @ 1 Hz (1000 ms) ---
        if time.ticks_diff(now, last_status_ms) >= 1000:
            last_status_ms = now
            print(ujson.dumps(motor_status_message("s")))

        # --- 6. Heartbeat LED toggle every 500 ms ---
        if time.ticks_diff(now, last_led_ms) >= 500:
            last_led_ms = now
            led_state = not led_state
            led.value(1 if led_state else 0)

        # Yield 1 ms to prevent CPU starvation
        time.sleep_ms(1)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        stop_motors()
        led.value(0)
