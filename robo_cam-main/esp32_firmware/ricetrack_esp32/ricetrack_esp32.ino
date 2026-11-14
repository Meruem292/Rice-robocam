/* ==============================================================================
 * RiceTrack Rover — ESP32 Arduino C++ Firmware (ricetrack_esp32.ino)
 * ==============================================================================
 * Target Board : DOIT ESP32 DEVKIT V1 (or any standard ESP32-WROOM-32 board)
 * IDE          : Arduino IDE 1.8.x or 2.x
 * Core         : esp32 by Espressif Systems (v2.x or v3.x compatible)
 *
 * Hardware Wiring:
 * 1. Dual BTS7960 Motor Drivers (MY1016Z2 24V DC Motors):
 *    - Left Motor:
 *        * R_EN -> GPIO 25, L_EN -> GPIO 26
 *        * RPWM -> GPIO 32, LPWM -> GPIO 33
 *    - Right Motor:
 *        * R_EN -> GPIO 27, L_EN -> GPIO 14
 *        * RPWM -> GPIO 18, LPWM -> GPIO 19
 *
 * 2. MPU-6050 6-Axis IMU (I2C):
 *    - SDA -> GPIO 21
 *    - SCL -> GPIO 22
 *    - VCC -> 3.3V, GND -> GND
 *
 * 3. NEO-6M GPS Module (UART2):
 *    - GPS TX -> ESP32 RX2 (GPIO 16)
 *    - GPS RX -> ESP32 TX2 (GPIO 17)
 *    - VCC -> 3.3V, GND -> GND
 *
 * 4. USB Serial to Host PC:
 *    - Baud: 115200 (newline-delimited JSON)
 *
 * 5. Manual Motor Test (Serial Monitor, 115200 baud):
 *    - Send one character + newline: W/S/A/D, 1/2, X, +/-, ?
 *    - These commands are ignored by the web app and do not use JSON.
 * ============================================================================== */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <ctype.h>

// ------------------------------------------------------------------------------
// PIN DEFINITIONS
// ------------------------------------------------------------------------------
// Left BTS7960
const int PIN_LEFT_R_EN = 25;
const int PIN_LEFT_L_EN = 26;
const int PIN_LEFT_RPWM = 32;
const int PIN_LEFT_LPWM = 33;

// Right BTS7960
const int PIN_RIGHT_R_EN = 27;
const int PIN_RIGHT_L_EN = 14;
const int PIN_RIGHT_RPWM = 18;
const int PIN_RIGHT_LPWM = 19;

// MPU-6050 (I2C0)
const int PIN_I2C_SDA = 21;
const int PIN_I2C_SCL = 22;
const uint8_t MPU_ADDR = 0x68;

// NEO-6M GPS (Serial2)
const int PIN_GPS_RX = 16; // ESP32 RX2 <- GPS TX
const int PIN_GPS_TX = 17; // ESP32 TX2 -> GPS RX
const uint32_t GPS_BAUD = 9600;

// Status LED (Onboard blue LED on DevKit V1 is GPIO 2)
const int PIN_LED = 2;

// PWM Configuration
const int PWM_FREQ = 1000;       // 1 kHz
const int PWM_RESOLUTION = 10;   // 10-bit (0 - 1023)
const int PWM_MAX = 1023;

// PWM Channels (for ESP32 core v2.x)
const int CH_LEFT_RPWM  = 0;
const int CH_LEFT_LPWM  = 1;
const int CH_RIGHT_RPWM = 2;
const int CH_RIGHT_LPWM = 3;

// Hardware Serial for GPS
HardwareSerial GPS_Serial(2);

// ------------------------------------------------------------------------------
// STATE VARIABLES
// ------------------------------------------------------------------------------
bool mpu_ok = false;
float roll_deg = 0.0f;
float pitch_deg = 0.0f;
float yaw_deg = 0.0f;
float gz_bias = 0.0f;

// Complementary filter weights
float comp_roll = 0.0f;
float comp_pitch = 0.0f;
uint32_t last_imu_time_us = 0;

// GPS State
String gps_status = "none";
double gps_lat = 0.0;
double gps_lon = 0.0;
int gps_sats = 0;
String nmea_buffer = "";

// Timers
uint32_t last_imu_tx_ms = 0;     // 20 Hz IMU telemetry (every 50 ms)
uint32_t last_gps_tx_ms = 0;     // 1 Hz GPS telemetry (every 1000 ms)
uint32_t last_cmd_time_ms = 0;   // Watchdog timer (auto-stop motors if communication lost)
uint32_t last_led_blink_ms = 0;  // Heartbeat LED
uint32_t last_status_tx_ms = 0;  // 1 Hz link + motor driver status telemetry
bool led_state = false;

const uint32_t MOTOR_TIMEOUT_MS = 2000; // Stop motors if no command within 2 seconds

// Link + per-driver monitoring state (reported to the web app)
uint32_t last_host_msg_ms = 0;
bool host_seen = false;
bool drivers_ready = false;
bool left_driver_enabled = false;
bool right_driver_enabled = false;
float left_driver_duty = 0.0f;
float right_driver_duty = 0.0f;
const char* left_driver_dir = "stop";
const char* right_driver_dir = "stop";

// Serial input buffer
String input_line = "";

// Manual motor test mode (single-character serial commands)
bool test_mode_active = false;
char test_motion = 'X';
int test_speed = 120; // 0 - 255, same default as the standalone test sketch

// ------------------------------------------------------------------------------
// MOTOR CONTROL
// ------------------------------------------------------------------------------
void setPWM(int pin, int channel, int duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    // ESP32 Arduino Core 3.x
    ledcWrite(pin, duty);
#else
    // ESP32 Arduino Core 2.x
    ledcWrite(channel, duty);
#endif
}

void motorsInit() {
    pinMode(PIN_LEFT_R_EN, OUTPUT);
    pinMode(PIN_LEFT_L_EN, OUTPUT);
    pinMode(PIN_RIGHT_R_EN, OUTPUT);
    pinMode(PIN_RIGHT_L_EN, OUTPUT);

    // Enable drivers
    digitalWrite(PIN_LEFT_R_EN, HIGH);
    digitalWrite(PIN_LEFT_L_EN, HIGH);
    digitalWrite(PIN_RIGHT_R_EN, HIGH);
    digitalWrite(PIN_RIGHT_L_EN, HIGH);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcAttach(PIN_LEFT_RPWM, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(PIN_LEFT_LPWM, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(PIN_RIGHT_RPWM, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(PIN_RIGHT_LPWM, PWM_FREQ, PWM_RESOLUTION);
#else
    ledcSetup(CH_LEFT_RPWM, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(CH_LEFT_LPWM, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(CH_RIGHT_RPWM, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(CH_RIGHT_LPWM, PWM_FREQ, PWM_RESOLUTION);

    ledcAttachPin(PIN_LEFT_RPWM, CH_LEFT_RPWM);
    ledcAttachPin(PIN_LEFT_LPWM, CH_LEFT_LPWM);
    ledcAttachPin(PIN_RIGHT_RPWM, CH_RIGHT_RPWM);
    ledcAttachPin(PIN_RIGHT_LPWM, CH_RIGHT_LPWM);
#endif

    // Initial stop
    setPWM(PIN_LEFT_RPWM, CH_LEFT_RPWM, 0);
    setPWM(PIN_LEFT_LPWM, CH_LEFT_LPWM, 0);
    setPWM(PIN_RIGHT_RPWM, CH_RIGHT_RPWM, 0);
    setPWM(PIN_RIGHT_LPWM, CH_RIGHT_LPWM, 0);

    drivers_ready = true;
    left_driver_enabled = false;
    right_driver_enabled = false;
    left_driver_duty = 0.0f;
    right_driver_duty = 0.0f;
    left_driver_dir = "stop";
    right_driver_dir = "stop";
}

void motorsStop() {
    setPWM(PIN_LEFT_RPWM, CH_LEFT_RPWM, 0);
    setPWM(PIN_LEFT_LPWM, CH_LEFT_LPWM, 0);
    setPWM(PIN_RIGHT_RPWM, CH_RIGHT_RPWM, 0);
    setPWM(PIN_RIGHT_LPWM, CH_RIGHT_LPWM, 0);

    left_driver_enabled = false;
    right_driver_enabled = false;
    left_driver_duty = 0.0f;
    right_driver_duty = 0.0f;
    left_driver_dir = "stop";
    right_driver_dir = "stop";
}

void setMotors(float left_duty, float right_duty) {
    // Clamp values between -1.0 and 1.0
    if (left_duty > 1.0f) left_duty = 1.0f;
    if (left_duty < -1.0f) left_duty = -1.0f;
    if (right_duty > 1.0f) right_duty = 1.0f;
    if (right_duty < -1.0f) right_duty = -1.0f;

    // Left Motor (RPWM = forward, LPWM = reverse)
    if (left_duty > 0.02f) {
        int duty = (int)(left_duty * PWM_MAX);
        setPWM(PIN_LEFT_RPWM, CH_LEFT_RPWM, duty);
        setPWM(PIN_LEFT_LPWM, CH_LEFT_LPWM, 0);
    } else if (left_duty < -0.02f) {
        int duty = (int)(-left_duty * PWM_MAX);
        setPWM(PIN_LEFT_RPWM, CH_LEFT_RPWM, 0);
        setPWM(PIN_LEFT_LPWM, CH_LEFT_LPWM, duty);
    } else {
        setPWM(PIN_LEFT_RPWM, CH_LEFT_RPWM, 0);
        setPWM(PIN_LEFT_LPWM, CH_LEFT_LPWM, 0);
    }

    // Right Motor (RPWM = forward, LPWM = reverse)
    if (right_duty > 0.02f) {
        int duty = (int)(right_duty * PWM_MAX);
        setPWM(PIN_RIGHT_RPWM, CH_RIGHT_RPWM, duty);
        setPWM(PIN_RIGHT_LPWM, CH_RIGHT_LPWM, 0);
    } else if (right_duty < -0.02f) {
        int duty = (int)(-right_duty * PWM_MAX);
        setPWM(PIN_RIGHT_RPWM, CH_RIGHT_RPWM, 0);
        setPWM(PIN_RIGHT_LPWM, CH_RIGHT_LPWM, duty);
    } else {
        setPWM(PIN_RIGHT_RPWM, CH_RIGHT_RPWM, 0);
        setPWM(PIN_RIGHT_LPWM, CH_RIGHT_LPWM, 0);
    }

    left_driver_duty = left_duty;
    right_driver_duty = right_duty;
    left_driver_enabled = left_duty > 0.02f || left_duty < -0.02f;
    right_driver_enabled = right_duty > 0.02f || right_duty < -0.02f;
    left_driver_dir = left_driver_enabled ? (left_duty > 0 ? "forward" : "reverse") : "stop";
    right_driver_dir = right_driver_enabled ? (right_duty > 0 ? "forward" : "reverse") : "stop";

    last_cmd_time_ms = millis();
}

// ------------------------------------------------------------------------------
// MANUAL MOTOR TEST MODE (single-character serial commands)
// ------------------------------------------------------------------------------
void printMotorTestMenu() {
    Serial.println("Motor test commands (send one character + newline):");
    Serial.println("  W = both forward    S = both reverse");
    Serial.println("  A = pivot left      D = pivot right");
    Serial.println("  1 = left forward    2 = right forward");
    Serial.println("  X = stop            + / - = speed up/down");
    Serial.println("  ? = this menu");
    Serial.print("  speed: ");
    Serial.println(test_speed);
}

void applyTestMotion() {
    if (!test_mode_active || test_motion == 'X') {
        motorsStop();
        return;
    }

    float duty = test_speed / 255.0f;

    switch (test_motion) {
        case 'W': setMotors(duty, duty); break;
        case 'S': setMotors(-duty, -duty); break;
        case 'A': setMotors(-duty, duty); break;
        case 'D': setMotors(duty, -duty); break;
        case '1': setMotors(duty, 0.0f); break;
        case '2': setMotors(0.0f, duty); break;
        default: motorsStop(); break;
    }
}

bool handleMotorTestCommand(const String &cmd) {
    if (cmd.length() != 1) return false;

    char c = (char)toupper((unsigned char)cmd[0]);

    switch (c) {
        case 'W':
        case 'S':
        case 'A':
        case 'D':
        case '1':
        case '2':
            test_motion = c;
            test_mode_active = true;
            applyTestMotion();
            Serial.print("Motor test: ");
            Serial.print(c);
            Serial.print(" @ ");
            Serial.println(test_speed);
            return true;

        case 'X':
            test_motion = 'X';
            test_mode_active = false;
            motorsStop();
            last_cmd_time_ms = millis();
            Serial.println("Motor test: STOP");
            return true;

        case '+':
        case '=':
            test_speed += 15;
            if (test_speed > 255) test_speed = 255;
            if (test_mode_active) applyTestMotion();
            Serial.print("Motor test speed: ");
            Serial.println(test_speed);
            return true;

        case '-':
        case '_':
            test_speed -= 15;
            if (test_speed < 0) test_speed = 0;
            if (test_mode_active) applyTestMotion();
            Serial.print("Motor test speed: ");
            Serial.println(test_speed);
            return true;

        case '?':
            printMotorTestMenu();
            return true;

        default:
            return false;
    }
}

// ------------------------------------------------------------------------------
// LINK + MOTOR DRIVER STATUS TELEMETRY (ESP32 -> Web App)
// ------------------------------------------------------------------------------
void sendMotorStatus(const char* type) {
    uint32_t now = millis();
    int32_t host_age = -1;
    int host_ok = 0;
    if (host_seen) {
        host_age = (int32_t)(now - last_host_msg_ms);
        host_ok = (host_age <= 5000) ? 1 : 0;
    }

    Serial.print("{\"t\":\"");
    Serial.print(type);
    Serial.print("\",\"l\":{\"c\":");
    Serial.print(drivers_ready ? 1 : 0);
    Serial.print(",\"e\":");
    Serial.print(left_driver_enabled ? 1 : 0);
    Serial.print(",\"d\":");
    Serial.print(left_driver_duty, 3);
    Serial.print(",\"dir\":\"");
    Serial.print(left_driver_dir);
    Serial.print("\"},\"r\":{\"c\":");
    Serial.print(drivers_ready ? 1 : 0);
    Serial.print(",\"e\":");
    Serial.print(right_driver_enabled ? 1 : 0);
    Serial.print(",\"d\":");
    Serial.print(right_driver_duty, 3);
    Serial.print(",\"dir\":\"");
    Serial.print(right_driver_dir);
    Serial.print("\"},\"host\":");
    Serial.print(host_ok);
    Serial.print(",\"ha\":");
    Serial.print(host_age);
    Serial.print(",\"up\":");
    Serial.print(now / 1000);
    Serial.println("}");
}

// ------------------------------------------------------------------------------
// MPU-6050 IMU ROUTINES (Native Wire.h — No external libraries needed)
// ------------------------------------------------------------------------------
bool mpuInit() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000); // 400 kHz Fast-Mode I2C
    delay(20);

    // Wake up MPU-6050 (write 0x00 to PWR_MGMT_1 register 0x6B)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0x00);
    if (Wire.endTransmission() != 0) {
        return false;
    }

    // Set Accelerometer sensitivity: +/- 4g (register 0x1C = 0x08)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1C);
    Wire.write(0x08);
    Wire.endTransmission();

    // Set Gyroscope sensitivity: +/- 500 deg/s (register 0x1B = 0x08)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1B);
    Wire.write(0x08);
    Wire.endTransmission();

    delay(20);
    last_imu_time_us = micros();

    // Calibrate Z-axis gyro bias (average 50 readings while stationary)
    long gz_sum = 0;
    int samples = 0;
    for (int i = 0; i < 50; i++) {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x47); // GYRO_ZOUT_H
        if (Wire.endTransmission(false) == 0) {
            if (Wire.requestFrom((uint16_t)MPU_ADDR, (uint8_t)2, (bool)true) == 2) {
                int16_t gz_raw = (Wire.read() << 8) | Wire.read();
                gz_sum += gz_raw;
                samples++;
            }
        }
        delay(3);
    }
    if (samples > 0) {
        gz_bias = (float)gz_sum / (float)samples;
    }
    yaw_deg = 0.0f;
    return true;
}

void mpuUpdate() {
    if (!mpu_ok) return;

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); // Starting at ACCEL_XOUT_H
    if (Wire.endTransmission(false) != 0) {
        mpu_ok = false;
        return;
    }

    // Read 14 bytes: Accel (6), Temp (2), Gyro (6)
    int n = Wire.requestFrom((uint16_t)MPU_ADDR, (uint8_t)14, (bool)true);
    if (n < 14) return;

    int16_t ax = (Wire.read() << 8) | Wire.read();
    int16_t ay = (Wire.read() << 8) | Wire.read();
    int16_t az = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); // skip temp
    int16_t gx = (Wire.read() << 8) | Wire.read();
    int16_t gy = (Wire.read() << 8) | Wire.read();
    int16_t gz = (Wire.read() << 8) | Wire.read();

    // Scale Accelerometer (+/- 4g -> 8192 LSB/g)
    float ax_g = (float)ax / 8192.0f;
    float ay_g = (float)ay / 8192.0f;
    float az_g = (float)az / 8192.0f;

    // Scale Gyroscope (+/- 500 deg/s -> 65.5 LSB/(deg/s))
    float gx_dps = (float)gx / 65.5f;
    float gy_dps = (float)gy / 65.5f;
    float gz_dps = ((float)gz - gz_bias) / 65.5f;
    if (fabs(gz_dps) < 0.30f) gz_dps = 0.0f; // Deadband filter to prevent stationary drift

    // Accel angles
    float accel_roll = atan2(ay_g, az_g) * 180.0f / M_PI;
    float accel_pitch = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180.0f / M_PI;

    uint32_t now_us = micros();
    float dt = (float)(now_us - last_imu_time_us) / 1000000.0f;
    if (dt <= 0.0f || dt > 0.5f) dt = 0.05f;
    last_imu_time_us = now_us;

    // Complementary Filter: 96% Gyro Integration + 4% Accelerometer Absolute
    comp_roll = 0.96f * (comp_roll + gx_dps * dt) + 0.04f * accel_roll;
    comp_pitch = 0.96f * (comp_pitch + gy_dps * dt) + 0.04f * accel_pitch;
    yaw_deg += gz_dps * dt;

    roll_deg = comp_roll;
    pitch_deg = comp_pitch;
}

// ------------------------------------------------------------------------------
// NEO-6M GPS NMEA PARSER (Native — No external library needed)
// ------------------------------------------------------------------------------
double parseNmeaCoord(String raw, String dir) {
    if (raw.length() < 4) return 0.0;
    int dot_idx = raw.indexOf('.');
    if (dot_idx < 2) return 0.0;

    int deg_len = (dot_idx == 4) ? 2 : 3; // ddmm.mmmm vs dddmm.mmmm
    double degrees = raw.substring(0, deg_len).toDouble();
    double minutes = raw.substring(deg_len).toDouble();
    double dec = degrees + (minutes / 60.0);
    if (dir == "S" || dir == "W") dec = -dec;
    return dec;
}

void parseNmeaLine(String line) {
    // Check GPRMC or GNRMC (Recommended Minimum Navigation Information)
    if (line.startsWith("$GPRMC") || line.startsWith("$GNRMC")) {
        // Example: $GPRMC,045319.00,A,1540.3021,N,12053.5681,E,0.02,23.4,121126,,,A*78
        int idx = 0;
        String tokens[13];
        int token_count = 0;
        while (idx < line.length() && token_count < 13) {
            int next_comma = line.indexOf(',', idx);
            if (next_comma == -1) next_comma = line.indexOf('*', idx);
            if (next_comma == -1) next_comma = line.length();
            tokens[token_count++] = line.substring(idx, next_comma);
            idx = next_comma + 1;
        }

        if (token_count >= 7) {
            String status = tokens[2]; // 'A' = valid, 'V' = receiver warning
            if (status == "A") {
                gps_status = "fix";
                gps_lat = parseNmeaCoord(tokens[3], tokens[4]);
                gps_lon = parseNmeaCoord(tokens[5], tokens[6]);
            } else {
                gps_status = "none";
            }
        }
    }
    // Check GPGGA or GNGGA (Global Positioning System Fix Data)
    else if (line.startsWith("$GPGGA") || line.startsWith("$GNGGA")) {
        // Example: $GPGGA,045319.00,1540.3021,N,12053.5681,E,1,08,1.2,45.0,M,...
        int idx = 0;
        String tokens[15];
        int token_count = 0;
        while (idx < line.length() && token_count < 15) {
            int next_comma = line.indexOf(',', idx);
            if (next_comma == -1) next_comma = line.indexOf('*', idx);
            if (next_comma == -1) next_comma = line.length();
            tokens[token_count++] = line.substring(idx, next_comma);
            idx = next_comma + 1;
        }

        if (token_count >= 8) {
            int quality = tokens[6].toInt();
            gps_sats = tokens[7].toInt();
            if (quality > 0) {
                gps_status = "fix";
                if (gps_lat == 0.0) gps_lat = parseNmeaCoord(tokens[2], tokens[3]);
                if (gps_lon == 0.0) gps_lon = parseNmeaCoord(tokens[4], tokens[5]);
            }
        }
    }
}

void gpsPoll() {
    while (GPS_Serial.available()) {
        char c = (char)GPS_Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            if (nmea_buffer.length() > 6) {
                parseNmeaLine(nmea_buffer);
            }
            nmea_buffer = "";
        } else {
            if (nmea_buffer.length() < 120) {
                nmea_buffer += c;
            }
        }
    }
}

// ------------------------------------------------------------------------------
// HOST SERIAL JSON COMMAND PARSER
// ------------------------------------------------------------------------------
void handleCommand(String json) {
    json.trim();
    if (json.length() == 0) return;

    // Single-character manual motor test command?
    if (handleMotorTestCommand(json)) return;

    if (json.length() < 5) return;

    // Any valid host message proves the host->ESP32 link is alive
    if (json.indexOf("\"c\"") != -1) {
        host_seen = true;
        last_host_msg_ms = millis();
    }

    // Check command type "c"
    // Motor command: {"c":"m","l":0.65,"r":0.65}
    int c_idx = json.indexOf("\"c\":\"m\"");
    if (c_idx == -1) c_idx = json.indexOf("\"c\": \"m\"");
    if (c_idx != -1) {
        float l = 0.0f;
        float r = 0.0f;

        // Parse "l"
        int l_idx = json.indexOf("\"l\":");
        if (l_idx != -1) {
            int l_end = json.indexOf(',', l_idx);
            if (l_end == -1) l_end = json.indexOf('}', l_idx);
            if (l_end != -1) {
                l = json.substring(l_idx + 4, l_end).toFloat();
            }
        }

        // Parse "r"
        int r_idx = json.indexOf("\"r\":");
        if (r_idx != -1) {
            int r_end = json.indexOf(',', r_idx);
            if (r_end == -1) r_end = json.indexOf('}', r_idx);
            if (r_end != -1) {
                r = json.substring(r_idx + 4, r_end).toFloat();
            }
        }

        setMotors(l, r);
        test_mode_active = false;
        return;
    }

    // Stop command: {"c":"s"}
    if (json.indexOf("\"c\":\"s\"") != -1 || json.indexOf("\"c\": \"s\"") != -1) {
        test_mode_active = false;
        motorsStop();
        last_cmd_time_ms = millis();
        return;
    }

    // Ping command: {"c":"p"} -> pong + link/driver status
    if (json.indexOf("\"c\":\"p\"") != -1 || json.indexOf("\"c\": \"p\"") != -1) {
        sendMotorStatus("k");
        return;
    }

    // Reset Yaw command: {"c":"ry"}
    if (json.indexOf("\"c\":\"ry\"") != -1 || json.indexOf("\"c\": \"ry\"") != -1) {
        yaw_deg = 0.0f;
        return;
    }
}

// ------------------------------------------------------------------------------
// ARDUINO SETUP & MAIN LOOP
// ------------------------------------------------------------------------------
void setup() {
    // 1. USB Serial to Host PC
    Serial.begin(115200);
    delay(200);

    // 2. Status LED
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);

    // 3. Dual BTS7960 Motors
    motorsInit();

    // 4. MPU-6050 I2C
    mpu_ok = mpuInit();

    // 5. NEO-6M GPS UART2
    GPS_Serial.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);

    last_cmd_time_ms = millis();
    last_imu_tx_ms = millis();
    last_gps_tx_ms = millis();
    last_led_blink_ms = millis();
    last_status_tx_ms = millis();

    // Initial announce
    Serial.println("{\"t\":\"k\",\"status\":\"ready\"}");
    sendMotorStatus("k");
    digitalWrite(PIN_LED, LOW);
}

void loop() {
    uint32_t now = millis();

    // 1. Read incoming commands from Host PC (USB Serial)
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (input_line.length() > 0) {
                handleCommand(input_line);
                input_line = "";
            }
        } else {
            if (input_line.length() < 128) {
                input_line += c;
            }
        }
    }

    // 2. Read GPS NMEA Stream
    gpsPoll();

    // 3. Update IMU readings
    mpuUpdate();

    // 4. Send IMU Telemetry @ 20 Hz (every 50 ms)
    // Format: {"t":"i","r":1.2,"p":-0.5,"y":0.0,"ok":1}
    if (now - last_imu_tx_ms >= 50) {
        last_imu_tx_ms = now;
        Serial.print("{\"t\":\"i\",\"r\":");
        Serial.print(roll_deg, 2);
        Serial.print(",\"p\":");
        Serial.print(pitch_deg, 2);
        Serial.print(",\"y\":");
        Serial.print(yaw_deg, 2);
        Serial.print(",\"ok\":");
        Serial.print(mpu_ok ? 1 : 0);
        Serial.println("}");
    }

    // 5. Send GPS Telemetry @ 1 Hz (every 1000 ms)
    // Format: {"t":"g","s":"fix","la":15.67170,"lo":120.89280,"n":8}
    if (now - last_gps_tx_ms >= 1000) {
        last_gps_tx_ms = now;
        Serial.print("{\"t\":\"g\",\"s\":\"");
        Serial.print(gps_status);
        Serial.print("\",\"la\":");
        Serial.print(gps_lat, 6);
        Serial.print(",\"lo\":");
        Serial.print(gps_lon, 6);
        Serial.print(",\"n\":");
        Serial.print(gps_sats);
        Serial.println("}");
    }

    // 6. Link + motor driver status @ 1 Hz (every 1000 ms)
    // Format: {"t":"s","l":{...},"r":{...},"host":1,"ha":125,"up":42}
    if (now - last_status_tx_ms >= 1000) {
        last_status_tx_ms = now;
        sendMotorStatus("s");
    }

    // 7. Safety Watchdog: Stop motors if no command received for 2.0 seconds.
    //    Manual test mode is exempt so motors keep running until 'X' is sent.
    if (!test_mode_active && now - last_cmd_time_ms > MOTOR_TIMEOUT_MS) {
        motorsStop();
    }

    // 8. Heartbeat LED blink (toggles every 500 ms)
    if (now - last_led_blink_ms >= 500) {
        last_led_blink_ms = now;
        led_state = !led_state;
        digitalWrite(PIN_LED, led_state ? HIGH : LOW);
    }
}

