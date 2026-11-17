/* ==============================================================================
 * RiceTrack Rover — Integrated ESP32 AP Controller + Camera & MPU Guidance
 * File: ricetrack_esp32_ap_web.ino
 * ==============================================================================
 * Drivetrain & Sensor Architecture:
 * 1. Dual BTS7960 Motor Drivers:
 *      Left:  RPWM -> GPIO 32, LPWM -> GPIO 33, R_EN -> GPIO 25, L_EN -> GPIO 26
 *      Right: RPWM -> GPIO 18, LPWM -> GPIO 19, R_EN -> GPIO 27, L_EN -> GPIO 14
 * 2. MPU-6050 6-Axis IMU (I2C):
 *      SDA -> GPIO 21, SCL -> GPIO 22 (400 kHz Fast-Mode I2C)
 *      Calculates Yaw heading with bias calibration & gyro integration.
 *      Drives Closed-Loop Precision Angle Turns (e.g. exact 90° & 180° turns).
 * 3. NEO-6M GPS Module (UART2):
 *      GPS TX -> ESP32 RX2 (GPIO 16), GPS RX -> ESP32 TX2 (GPIO 17) @ 9600 baud.
 *      Dedicated to geotagging photo locations (not for steering).
 * 4. Wi-Fi SoftAP:
 *      SSID: RiceTrack-Rover | Password: ricetrack123 | IP: 192.168.4.1
 *      Hosts responsive rover web UI, closed-loop turn controls, and embeds
 *      Raspberry Pi camera stream & crop health analysis.
 * ============================================================================== */

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <math.h>

// ------------------------------------------------------------------------------
// PIN DEFINITIONS
// ------------------------------------------------------------------------------
// Left BTS7960 Driver
const int PIN_LEFT_RPWM  = 32;
const int PIN_LEFT_LPWM  = 33;
const int PIN_LEFT_R_EN  = 25;
const int PIN_LEFT_L_EN  = 26;

// Right BTS7960 Driver
const int PIN_RIGHT_RPWM = 18;
const int PIN_RIGHT_LPWM = 19;
const int PIN_RIGHT_R_EN = 27;
const int PIN_RIGHT_L_EN = 14;

// Onboard Blue Status LED
const int PIN_LED        = 2;

// MPU-6050 IMU (I2C)
const int PIN_I2C_SDA    = 21;
const int PIN_I2C_SCL    = 22;
const uint8_t MPU_ADDR   = 0x68;

// NEO-6M GPS (UART2)
const int PIN_GPS_RX     = 16;  // ESP32 RX2 <- GPS TX
const int PIN_GPS_TX     = 17;  // ESP32 TX2 -> GPS RX
const uint32_t GPS_BAUD  = 9600;

// PWM Parameters
const int PWM_FREQ       = 1000;  // 1 kHz
const int PWM_RESOLUTION = 10;    // 10-bit (0 - 1023)
const int PWM_MAX        = 1023;

WebServer server(80);
HardwareSerial GPS_Serial(2);

// ------------------------------------------------------------------------------
// STATE VARIABLES
// ------------------------------------------------------------------------------
float current_speed = 0.75f;
String current_motion = "STOP";
float left_duty = 0.0f;
float right_duty = 0.0f;

// IMU / MPU-6050 State
bool mpu_ok = false;
float roll_deg = 0.0f;
float pitch_deg = 0.0f;
float yaw_deg = 0.0f;
float gz_bias = 0.0f;
float comp_roll = 0.0f;
float comp_pitch = 0.0f;
uint32_t last_imu_time_us = 0;

// GPS State (Geotagging)
String gps_status = "none";
double gps_lat = 0.0;
double gps_lon = 0.0;
int gps_sats = 0;
String nmea_buffer = "";

// Closed-Loop Turn Controller (MPU-6050 Guidance)
bool is_turning = false;
String turn_direction = "none";  // "left" or "right"
float turn_target_degrees = 90.0f;
float turn_start_yaw = 0.0f;
uint32_t turn_start_ms = 0;
const uint32_t TURN_TIMEOUT_MS = 7000;

// Serial & Telemetry Timers
uint32_t last_imu_tx_ms = 0;
uint32_t last_gps_tx_ms = 0;
String serial_rx_buf = "";

// Raspberry Pi Auto-Discovery State
String pi_ip_address = "";
int pi_web_port = 8000;
uint32_t pi_last_heartbeat_ms = 0;
bool pi_online = false;

// Forward declarations
void setMotors(float l, float r);
void stopMotors();
void applyCommand(const String& cmd);
void startTurn(const String& dir, float deg);
void updateTurn();
bool mpuInit();
void mpuUpdate();
void gpsPoll();
void parseNmeaLine(String line);
double parseNmeaCoord(String raw, String dir);
void handlePiHeartbeat();

// ------------------------------------------------------------------------------
// PWM MOTOR CONTROL
// ------------------------------------------------------------------------------
void initPWM() {
    pinMode(PIN_LEFT_R_EN, OUTPUT);
    pinMode(PIN_LEFT_L_EN, OUTPUT);
    pinMode(PIN_RIGHT_R_EN, OUTPUT);
    pinMode(PIN_RIGHT_L_EN, OUTPUT);

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
    ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(2, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(3, PWM_FREQ, PWM_RESOLUTION);

    ledcAttachPin(PIN_LEFT_RPWM, 0);
    ledcAttachPin(PIN_LEFT_LPWM, 1);
    ledcAttachPin(PIN_RIGHT_RPWM, 2);
    ledcAttachPin(PIN_RIGHT_LPWM, 3);
#endif
}

void writePWM(int pin, int channel, int duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWrite(pin, duty);
#else
    ledcWrite(channel, duty);
#endif
}

void setMotors(float l, float r) {
    l = constrain(l, -1.0f, 1.0f);
    r = constrain(r, -1.0f, 1.0f);

    left_duty = l;
    right_duty = r;

    // Left Motor
    if (l > 0.02f) {
        int d = (int)(l * PWM_MAX);
        writePWM(PIN_LEFT_RPWM, 0, d);
        writePWM(PIN_LEFT_LPWM, 1, 0);
    } else if (l < -0.02f) {
        int d = (int)(-l * PWM_MAX);
        writePWM(PIN_LEFT_RPWM, 0, 0);
        writePWM(PIN_LEFT_LPWM, 1, d);
    } else {
        writePWM(PIN_LEFT_RPWM, 0, 0);
        writePWM(PIN_LEFT_LPWM, 1, 0);
    }

    // Right Motor
    if (r > 0.02f) {
        int d = (int)(r * PWM_MAX);
        writePWM(PIN_RIGHT_RPWM, 2, d);
        writePWM(PIN_RIGHT_LPWM, 3, 0);
    } else if (r < -0.02f) {
        int d = (int)(-r * PWM_MAX);
        writePWM(PIN_RIGHT_RPWM, 2, 0);
        writePWM(PIN_RIGHT_LPWM, 3, d);
    } else {
        writePWM(PIN_RIGHT_RPWM, 2, 0);
        writePWM(PIN_RIGHT_LPWM, 3, 0);
    }

    digitalWrite(PIN_LED, (l != 0.0f || r != 0.0f) ? HIGH : LOW);
}

void stopMotors() {
    setMotors(0.0f, 0.0f);
    current_motion = "STOP";
    is_turning = false;
}

void applyCommand(const String& cmd) {
    current_motion = cmd;
    float spd = current_speed;

    if (cmd == "forward") {
        setMotors(spd, spd);
    } else if (cmd == "backward") {
        setMotors(-spd, -spd);
    } else if (cmd == "left") {
        setMotors(-spd, spd);  // Left reverse, Right forward -> pivot left
    } else if (cmd == "right") {
        setMotors(spd, -spd);  // Left forward, Right reverse -> pivot right
    } else if (cmd == "left_fwd") {
        setMotors(0.0f, spd);
    } else if (cmd == "left_rev") {
        setMotors(0.0f, -spd);
    } else if (cmd == "right_fwd") {
        setMotors(spd, 0.0f);
    } else if (cmd == "right_rev") {
        setMotors(-spd, 0.0f);
    } else {
        stopMotors();
    }
}

// ------------------------------------------------------------------------------
// CLOSED-LOOP MPU-6050 TARGET ANGLE ROTATION
// ------------------------------------------------------------------------------
void startTurn(const String& dir, float deg) {
    turn_direction = dir;
    turn_target_degrees = (deg > 3.0f) ? deg : 90.0f;
    turn_start_yaw = yaw_deg;
    turn_start_ms = millis();
    is_turning = true;
    current_motion = (dir == "left") ? "TURN_LEFT_MPU" : "TURN_RIGHT_MPU";
}

void updateTurn() {
    if (!is_turning) return;

    // Failsafe timeout
    if (millis() - turn_start_ms > TURN_TIMEOUT_MS) {
        stopMotors();
        is_turning = false;
        return;
    }

    if (!mpu_ok) {
        // Fallback: If MPU is offline, turn using calibrated rate (~45 deg/sec at turn duty)
        float elapsed_sec = (float)(millis() - turn_start_ms) / 1000.0f;
        float est_turned = elapsed_sec * 45.0f;
        if (est_turned >= turn_target_degrees) {
            stopMotors();
            is_turning = false;
        } else {
            float spd = current_speed;
            if (turn_direction == "left") setMotors(-spd, spd);
            else setMotors(spd, -spd);
        }
        return;
    }

    float delta = fabs(yaw_deg - turn_start_yaw);
    float remaining = turn_target_degrees - delta;

    if (remaining <= 2.5f) {
        // Target angle reached! Stop immediately with dynamic braking
        stopMotors();
        is_turning = false;
    } else {
        // Two-stage turn speed: Slow down (creep) near the target for precision
        float spd = current_speed;
        if (remaining < 22.0f) {
            spd = constrain(current_speed * 0.55f, 0.38f, 0.58f);
        }
        if (turn_direction == "left") {
            setMotors(-spd, spd);
        } else {
            setMotors(spd, -spd);
        }
    }
}

// ------------------------------------------------------------------------------
// MPU-6050 IMU ROUTINES (Native Wire.h)
// ------------------------------------------------------------------------------
bool mpuInit() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
    delay(25);

    // Wake up MPU-6050
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0x00);
    if (Wire.endTransmission() != 0) {
        return false;
    }

    // Accel +/- 4g
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1C);
    Wire.write(0x08);
    Wire.endTransmission();

    // Gyro +/- 500 deg/s
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1B);
    Wire.write(0x08);
    Wire.endTransmission();

    delay(20);
    last_imu_time_us = micros();

    // Calibrate Z-gyro bias over 60 stationary samples
    long gz_sum = 0;
    int samples = 0;
    for (int i = 0; i < 60; i++) {
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
    Wire.write(0x3B);
    if (Wire.endTransmission(false) != 0) {
        mpu_ok = false;
        return;
    }

    int n = Wire.requestFrom((uint16_t)MPU_ADDR, (uint8_t)14, (bool)true);
    if (n < 14) return;

    int16_t ax = (Wire.read() << 8) | Wire.read();
    int16_t ay = (Wire.read() << 8) | Wire.read();
    int16_t az = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); // skip temp
    int16_t gx = (Wire.read() << 8) | Wire.read();
    int16_t gy = (Wire.read() << 8) | Wire.read();
    int16_t gz = (Wire.read() << 8) | Wire.read();

    float ax_g = (float)ax / 8192.0f;
    float ay_g = (float)ay / 8192.0f;
    float az_g = (float)az / 8192.0f;

    float gx_dps = (float)gx / 65.5f;
    float gy_dps = (float)gy / 65.5f;
    float gz_dps = ((float)gz - gz_bias) / 65.5f;
    if (fabs(gz_dps) < 0.25f) gz_dps = 0.0f; // Deadband filter

    float accel_roll = atan2(ay_g, az_g) * 180.0f / M_PI;
    float accel_pitch = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180.0f / M_PI;

    uint32_t now_us = micros();
    float dt = (float)(now_us - last_imu_time_us) / 1000000.0f;
    if (dt <= 0.0f || dt > 0.5f) dt = 0.05f;
    last_imu_time_us = now_us;

    comp_roll = 0.96f * (comp_roll + gx_dps * dt) + 0.04f * accel_roll;
    comp_pitch = 0.96f * (comp_pitch + gy_dps * dt) + 0.04f * accel_pitch;
    yaw_deg += gz_dps * dt;

    roll_deg = comp_roll;
    pitch_deg = comp_pitch;
}

// ------------------------------------------------------------------------------
// NEO-6M GPS NMEA PARSER (Geotagging Only)
// ------------------------------------------------------------------------------
double parseNmeaCoord(String raw, String dir) {
    if (raw.length() < 4) return 0.0;
    int dot_idx = raw.indexOf('.');
    if (dot_idx < 2) return 0.0;

    int deg_len = (dot_idx == 4) ? 2 : 3;
    double degrees = raw.substring(0, deg_len).toDouble();
    double minutes = raw.substring(deg_len).toDouble();
    double dec = degrees + (minutes / 60.0);
    if (dir == "S" || dir == "W") dec = -dec;
    return dec;
}

void parseNmeaLine(String line) {
    if (line.startsWith("$GPRMC") || line.startsWith("$GNRMC")) {
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
            String status = tokens[2];
            if (status == "A") {
                gps_status = "fix";
                gps_lat = parseNmeaCoord(tokens[3], tokens[4]);
                gps_lon = parseNmeaCoord(tokens[5], tokens[6]);
            } else {
                gps_status = "none";
            }
        }
    }
    else if (line.startsWith("$GPGGA") || line.startsWith("$GNGGA")) {
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
// HTML WEB PAGE (Stored in Flash Memory)
// ------------------------------------------------------------------------------
const char PAGE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
  <title>RiceTrack Rover Dashboard</title>
  <style>
    * { box-sizing: border-box; -webkit-tap-highlight-color: transparent; }
    body {
      margin: 0; padding: 12px; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
      background: #0d1117; color: #c9d1d9; text-align: center; user-select: none;
    }
    .header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 8px; }
    h2 { margin: 0; color: #3fb950; font-size: 18px; font-weight: 800; }
    .badge { font-size: 11px; padding: 3px 8px; border-radius: 12px; font-weight: 600; }
    .badge.green { background: #238636; color: #fff; }
    .badge.orange { background: #9e6a03; color: #fff; }
    .badge.red { background: #da3633; color: #fff; }

    /* CARD STYLES */
    .card {
      background: #161b22; border: 1px solid #30363d; border-radius: 10px;
      padding: 8px 10px; margin-bottom: 10px; text-align: left;
    }
    .card-title { font-size: 13px; font-weight: 700; color: #8b949e; margin-bottom: 6px; display: flex; justify-content: space-between; }

    /* CAMERA BOX */
    .cam-box {
      width: 100%; height: 230px; background: #000; border-radius: 6px;
      display: flex; align-items: center; justify-content: center; overflow: hidden; position: relative;
    }
    .cam-box img { width: 100%; height: 100%; object-fit: contain; }
    .cam-overlay-btn { margin-top: 8px; display: flex; gap: 8px; }

    /* SENSOR & TELEMETRY ROW */
    .telemetry-grid {
      display: grid; grid-template-columns: 1fr 1fr; gap: 8px; font-size: 12px;
    }
    .tele-item { background: #0d1117; padding: 6px 8px; border-radius: 6px; border: 1px solid #21262d; }
    .tele-val { font-weight: bold; color: #58a6ff; font-size: 13px; }

    /* BUTTONS */
    .btn-group { display: flex; justify-content: center; gap: 6px; margin-bottom: 10px; }
    .spd-btn {
      background: #21262d; color: #c9d1d9; border: 1px solid #30363d; border-radius: 6px;
      padding: 6px 10px; font-weight: bold; font-size: 13px; cursor: pointer; flex: 1;
    }
    .spd-btn.active { background: #238636; border-color: #3fb950; color: #fff; }

    .dpad-wrap { display: flex; justify-content: center; margin: 6px 0 10px; }
    .dpad {
      display: grid; grid-template-columns: 85px 85px 85px; grid-template-rows: 70px 70px 70px; gap: 8px;
    }
    .btn {
      background: #21262d; color: #f0f6fc; border: 1px solid #30363d; border-radius: 12px;
      font-size: 24px; font-weight: bold; display: flex; align-items: center; justify-content: center;
      cursor: pointer; transition: all 0.08s;
    }
    .btn:active { background: #388bfd; transform: scale(0.95); }
    .btn-stop { background: #da3633; border-color: #f85149; font-size: 18px; }
    .btn-stop:active { background: #b62324; }

    .btn-turn {
      background: #1f2937; border: 1px solid #3b82f6; color: #60a5fa; border-radius: 8px;
      padding: 8px 12px; font-weight: 700; font-size: 12px; cursor: pointer; flex: 1;
    }
    .btn-turn:active { background: #2563eb; color: #fff; }

    .btn-action {
      background: #238636; color: #fff; border: 1px solid #3fb950; border-radius: 6px;
      padding: 10px 14px; font-weight: 700; font-size: 13px; cursor: pointer; flex: 1;
    }
    .btn-action:active { background: #2ea043; }

    .btn-secondary {
      background: #21262d; color: #58a6ff; border: 1px solid #30363d; border-radius: 6px;
      padding: 8px 12px; font-size: 12px; font-weight: 600; text-decoration: none; display: inline-block;
    }

    #evalBox {
      display: none; background: #0f2d1e; border: 1px solid #2ea043; border-radius: 8px;
      padding: 10px; margin-bottom: 10px; text-align: left; font-size: 13px; color: #7ee787;
    }

    .mission-box {
      background: #0f172a; border: 1px solid #1e293b; border-radius: 8px; padding: 10px; margin-top: 8px;
    }
  </style>
</head>
<body>
  <div class="header">
    <h2>🌾 RiceTrack Rover</h2>
    <span id="badgeLink" class="badge green">ESP32 + MPU Guidance</span>
  </div>

  <!-- SENSOR TELEMETRY: MPU-6050 & NEO-6M GPS -->
  <div class="card">
    <div class="card-title">
      <span>🧭 Navigation Sensors</span>
      <button onclick="resetYaw()" style="background:#30363d; color:#c9d1d9; border:none; border-radius:4px; padding:2px 8px; font-size:11px; cursor:pointer;">⟲ Zero Heading</button>
    </div>
    <div class="telemetry-grid">
      <div class="tele-item">
        <div style="color:#8b949e;">MPU-6050 Heading</div>
        <div id="lblYaw" class="tele-val">0.0°</div>
      </div>
      <div class="tele-item">
        <div style="color:#8b949e;">Motion State</div>
        <div id="lblMotion" class="tele-val">STOP</div>
      </div>
      <div class="tele-item" style="grid-column: span 2;">
        <div style="color:#8b949e;">📍 GPS Geotag (For Captures)</div>
        <div id="lblGps" class="tele-val" style="font-size:12px; color:#7ee787;">Waiting for fix…</div>
      </div>
    </div>
  </div>

  <!-- LIVE CAMERA STREAM FROM PI -->
  <div class="card">
    <div class="card-title">
      <span>📷 Live Camera Feed</span>
      <span id="camStatus" style="color:#e3b341; font-weight:bold;">Connecting…</span>
    </div>

    <div style="display:flex; gap:6px; margin-bottom:8px; align-items:center; background:#21262d; padding:6px 8px; border-radius:6px; flex-wrap:wrap;">
      <span style="font-size:12px; color:#8b949e;">Pi Host:</span>
      <input id="inpPiHost" type="text" value="192.168.4.2:8000" style="background:#0d1117; color:#58a6ff; border:1px solid #30363d; border-radius:4px; padding:3px 6px; width:135px; font-size:12px; font-weight:bold;">
      <button onclick="applyPiHost()" style="background:#238636; color:#fff; border:none; border-radius:4px; padding:4px 10px; font-size:12px; font-weight:bold; cursor:pointer;">Connect</button>
      <button onclick="autoScanPi()" style="background:#1f6feb; color:#fff; border:none; border-radius:4px; padding:4px 8px; font-size:11px; font-weight:600; cursor:pointer;">🔍 Auto-Find</button>
    </div>

    <div id="piOfflineNotice" style="display:none; background:#2d1b00; border:1px solid #9e6a03; border-radius:6px; padding:6px 10px; margin-bottom:8px; font-size:11px; color:#f0883e; text-align:left;">
      ⚠️ <b>Raspberry Pi not yet detected on AP.</b><br>
      • On Pi, verify Wi-Fi is connected to <b>RiceTrack-Rover</b> (run <code>./connect_to_esp32_ap.sh</code>)<br>
      • Ensure <code>web_app.py</code> is running on port 8000.
    </div>

    <div class="cam-box">
      <img id="camStream" 
           src="http://192.168.4.2:8000/api/video_feed"
           onload="onCamConnected()"
           onerror="onCamError()">
    </div>
    <div class="cam-overlay-btn">
      <button class="btn-action" onclick="captureEvaluate()">📸 Capture &amp; Geotag Sample</button>
      <a id="btnFullDash" href="http://192.168.4.2:8000" target="_blank" class="btn-secondary">📊 Full Dashboard</a>
    </div>
  </div>

  <!-- AI EVALUATION RESULT -->
  <div id="evalBox">
    <div style="font-weight:bold; font-size:14px; margin-bottom:4px;" id="evalTitle">🌱 Crop Health: Evaluating…</div>
    <div id="evalDetail" style="font-size:12px; color:#c9d1d9;">Connecting to Raspberry Pi vision model…</div>
  </div>

  <!-- SPEED SELECTOR -->
  <div class="btn-group">
    <button class="spd-btn" onclick="setSpeed(0.40)">40%</button>
    <button class="spd-btn" onclick="setSpeed(0.60)">60%</button>
    <button class="spd-btn active" onclick="setSpeed(0.75)">75%</button>
    <button class="spd-btn" onclick="setSpeed(0.85)">85%</button>
    <button class="spd-btn" onclick="setSpeed(1.00)">100%</button>
  </div>

  <!-- D-PAD CONTROLS -->
  <div class="dpad-wrap">
    <div class="dpad">
      <div></div>
      <button class="btn" onmousedown="pressBtn('forward')" onmouseup="releaseBtn()" ontouchstart="pressBtn('forward'); event.preventDefault();" ontouchend="releaseBtn(); event.preventDefault();">▲</button>
      <div></div>

      <button class="btn" onmousedown="pressBtn('left')" onmouseup="releaseBtn()" ontouchstart="pressBtn('left'); event.preventDefault();" ontouchend="releaseBtn(); event.preventDefault();">◄</button>
      <button class="btn btn-stop" onclick="execCmd('stop')">STOP</button>
      <button class="btn" onmousedown="pressBtn('right')" onmouseup="releaseBtn()" ontouchstart="pressBtn('right'); event.preventDefault();" ontouchend="releaseBtn(); event.preventDefault();">►</button>

      <div></div>
      <button class="btn" onmousedown="pressBtn('backward')" onmouseup="releaseBtn()" ontouchstart="pressBtn('backward'); event.preventDefault();" ontouchend="releaseBtn(); event.preventDefault();">▼</button>
      <div></div>
    </div>
  </div>

  <!-- MPU CLOSED-LOOP TARGET ANGLE TURNS -->
  <div style="display:flex; gap:8px; margin-bottom:12px;">
    <button class="btn-turn" onclick="turnAngle('left', 90)">⟲ 90° Left (MPU)</button>
    <button class="btn-turn" onclick="turnAngle('right', 90)">⟳ 90° Right (MPU)</button>
    <button class="btn-turn" onclick="turnAngle('right', 180)">↩ 180° Return</button>
  </div>

  <!-- INSTRUCTED ROVING ROUTINE LAUNCHER -->
  <div class="card mission-box">
    <div class="card-title" style="color:#38bdf8;">
      <span>🚀 Instructed Mission (Auto Roving)</span>
      <span id="missionStatusChip" style="background:#334155; padding:2px 8px; border-radius:10px; font-size:11px;">READY</span>
    </div>
    <div style="font-size:12px; color:#94a3b8; margin-bottom:8px;">
      Executes straight legs with MPU heading turns and automatic GPS geotagging.
    </div>
    <div style="display:flex; gap:6px;">
      <select id="selMission" style="background:#1e293b; color:#f8fafc; border:1px solid #334155; border-radius:6px; padding:6px; font-size:12px; flex:2;">
        <option value="preset_4step_inspection">Preset 1: 4-Step Field Mission (5m R → 10m L → 4m L → Return)</option>
        <option value="preset_u_turn_sweep">Preset 2: U-Turn Canopy Sweep (8m R → 4m R → 8m → Return)</option>
        <option value="preset_straight_sampling">Preset 3: Linear Row Sampling (5m → 5m → 5m → Return)</option>
      </select>
      <button class="btn-action" style="flex:1; padding:6px 10px;" onclick="startInstructedMission()">Start</button>
      <button class="btn-stop" style="border-radius:6px; font-size:12px; padding:6px 12px; font-weight:bold; cursor:pointer;" onclick="abortInstructedMission()">Abort</button>
    </div>
    <div id="missionDetail" style="font-size:11px; color:#38bdf8; margin-top:6px; display:none;"></div>
  </div>

  <div style="margin-top:8px; font-size:13px; color:#8b949e;">
    <label style="cursor:pointer;">
      <input type="checkbox" id="chkContinuous" onchange="toggleContinuous(this.checked)" checked>
      <b>Continuous Drive</b> (Click to run, Click STOP to halt)
    </label>
  </div>

  <script>
    let isContinuous = true;
    let curSpeed = 0.75;
    let piHost = localStorage.getItem("pi_host") || "192.168.4.2:8000";
    let camConnected = false;
    let autoFindDone = false;
    let missionActive = false;

    const candidateHosts = [
      "192.168.4.2:8000",
      "192.168.4.3:8000",
      "192.168.4.50:8000",
      "192.168.4.4:8000",
      "192.168.4.5:8000"
    ];

    window.applyPiHost = (newHost) => {
      if (newHost) {
        piHost = newHost;
      } else {
        const inp = document.getElementById('inpPiHost');
        if (inp && inp.value.trim()) {
          piHost = inp.value.trim();
        }
      }
      localStorage.setItem("pi_host", piHost);
      const inp = document.getElementById('inpPiHost');
      if (inp) inp.value = piHost;
      const lnk = document.getElementById('btnFullDash');
      if (lnk) lnk.href = "http://" + piHost;
      document.getElementById('camStatus').textContent = "Connecting…";
      document.getElementById('camStatus').style.color = "#e3b341";
      document.getElementById('camStream').src = "http://" + piHost + "/api/video_feed?t=" + Date.now();
    };

    window.onCamConnected = () => {
      camConnected = true;
      document.getElementById('camStatus').textContent = "ONLINE (" + piHost + ")";
      document.getElementById('camStatus').style.color = "#2ea043";
      document.getElementById('piOfflineNotice').style.display = "none";
    };

    window.onCamError = () => {
      camConnected = false;
      document.getElementById('camStatus').textContent = "Standby / Checking…";
      document.getElementById('camStatus').style.color = "#e3b341";
      if (!autoFindDone) {
        autoFindDone = true;
        setTimeout(window.autoScanPi, 1200);
      }
    };

    window.autoScanPi = async () => {
      document.getElementById('camStatus').textContent = "Auto-Finding Pi…";
      document.getElementById('camStatus').style.color = "#58a6ff";
      for (const host of candidateHosts) {
        try {
          const ctrl = new AbortController();
          const tid = setTimeout(() => ctrl.abort(), 900);
          const res = await fetch("http://" + host + "/api/telemetry", { signal: ctrl.signal, mode: "cors" });
          clearTimeout(tid);
          if (res.ok) {
            window.applyPiHost(host);
            return;
          }
        } catch (e) {}
      }
      document.getElementById('camStatus').textContent = "Offline (Tap Auto-Find)";
      document.getElementById('camStatus').style.color = "#da3633";
      document.getElementById('piOfflineNotice').style.display = "block";
    };

    window.toggleContinuous = (val) => {
      isContinuous = val;
    };

    window.setSpeed = (spd) => {
      curSpeed = spd;
      document.querySelectorAll('.spd-btn').forEach(b => {
        b.classList.toggle('active', b.textContent === Math.round(spd * 100) + '%');
      });
      fetch('/cmd?spd=' + spd).catch(()=>{});
    };

    window.execCmd = (act) => {
      document.getElementById('lblMotion').textContent = act.toUpperCase();
      fetch('/cmd?act=' + act + '&spd=' + curSpeed).catch(()=>{});
    };

    window.turnAngle = (dir, deg) => {
      document.getElementById('lblMotion').textContent = "TURN " + deg + "° " + dir.toUpperCase();
      fetch('/cmd?act=turn&dir=' + dir + '&deg=' + deg + '&spd=' + curSpeed).catch(()=>{});
    };

    window.resetYaw = () => {
      fetch('/cmd?act=reset_yaw').catch(()=>{});
      document.getElementById('lblYaw').textContent = "0.0°";
    };

    window.pressBtn = (act) => {
      window.execCmd(act);
    };

    window.releaseBtn = () => {
      if (!isContinuous) {
        window.execCmd('stop');
      }
    };

    // AI CROP HEALTH EVALUATION + GEOTAG FROM PI
    window.captureEvaluate = async () => {
      const box = document.getElementById('evalBox');
      const title = document.getElementById('evalTitle');
      const detail = document.getElementById('evalDetail');
      box.style.display = "block";
      title.textContent = "📸 Capturing & Geotagging Crop…";
      detail.textContent = "Running IRRI Leaf Color Chart (LCC) & ExG model on Pi…";

      try {
        const res = await fetch('http://' + piHost + '/api/capture_evaluate', { method: 'POST' });
        const data = await res.json();
        if (data.ok || data.status || data.panel) {
          const panel = data.panel || data.status || "Evaluated";
          const action = data.action || data.summary || "Canopy analyzed successfully.";
          const cov = data.coverage_pct != null ? Math.round(data.coverage_pct) : 0;
          title.textContent = "🌾 IRRI LCC Panel: " + panel;
          detail.textContent = action + " (Canopy: " + cov + "%)";
        } else {
          title.textContent = "⚠️ Evaluation Note";
          detail.textContent = data.error || "No canopy detected in frame.";
        }
      } catch (err) {
        title.textContent = "⚠️ Connection Note";
        detail.textContent = "Could not reach Pi at http://" + piHost + ". Check Pi address.";
      }
    };

    // INSTRUCTED ROVING CONTROLS
    window.startInstructedMission = async () => {
      const preset = document.getElementById('selMission').value;
      const statusChip = document.getElementById('missionStatusChip');
      const detail = document.getElementById('missionDetail');
      statusChip.textContent = "STARTING…";
      statusChip.style.background = "#0284c7";
      detail.style.display = "block";
      detail.textContent = "Initiating mission: " + preset;

      try {
        const res = await fetch('http://' + piHost + '/api/instructed_roving/start', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ preset_id: preset, steps: [] })
        });
        const d = await res.json();
        if (d.ok) {
          missionActive = true;
          statusChip.textContent = "ACTIVE";
          statusChip.style.background = "#238636";
        } else {
          statusChip.textContent = "ERROR";
          statusChip.style.background = "#da3633";
          detail.textContent = d.error || "Failed to start mission.";
        }
      } catch (e) {
        statusChip.textContent = "ERROR";
        detail.textContent = "Could not reach Raspberry Pi on http://" + piHost;
      }
    };

    window.abortInstructedMission = async () => {
      try {
        await fetch('http://' + piHost + '/api/instructed_roving/stop', { method: 'POST' });
        document.getElementById('missionStatusChip').textContent = "ABORTED";
        document.getElementById('missionStatusChip').style.background = "#da3633";
        missionActive = false;
        window.execCmd('stop');
      } catch (e) {}
    };

    const pollInstructedStatus = async () => {
      if (!missionActive) return;
      try {
        const res = await fetch('http://' + piHost + '/api/instructed_roving/status');
        const st = await res.json();
        const statusChip = document.getElementById('missionStatusChip');
        const detail = document.getElementById('missionDetail');
        if (st.active) {
          statusChip.textContent = (st.phase || "ACTIVE").toUpperCase();
          statusChip.style.background = "#238636";
          detail.style.display = "block";
          detail.textContent = st.phase_detail || "Mission in progress...";
        } else if (st.phase === "completed") {
          statusChip.textContent = "COMPLETED";
          statusChip.style.background = "#238636";
          detail.style.display = "block";
          detail.textContent = st.phase_detail || "Mission Completed!";
          missionActive = false;
        } else if (st.phase === "aborted") {
          statusChip.textContent = "ABORTED";
          statusChip.style.background = "#da3633";
          missionActive = false;
        }
      } catch (e) {}
    };
    setInterval(pollInstructedStatus, 800);

    // TELEMETRY REFRESH POLLER (Heading, GPS & Pi Handshake)
    const pollSensors = async () => {
      try {
        const res = await fetch('/telemetry');
        const data = await res.json();
        if (data.yaw !== undefined) {
          document.getElementById('lblYaw').textContent = (data.yaw >= 0 ? "+" : "") + data.yaw.toFixed(1) + "°";
        }
        if (data.motion) {
          document.getElementById('lblMotion').textContent = data.motion;
        }
        if (data.gps_status === "fix") {
          document.getElementById('lblGps').textContent = "📍 " + data.gps_lat.toFixed(5) + ", " + data.gps_lon.toFixed(5) + " (" + data.gps_sats + " sats)";
          document.getElementById('lblGps').style.color = "#7ee787";
        } else {
          document.getElementById('lblGps').textContent = "🧭 MPU Guidance Active (GPS Geotag Standby)";
          document.getElementById('lblGps').style.color = "#38bdf8";
        }

        // Auto-synchronize Pi Host if detected by ESP32 heartbeat
        if (data.pi_online && data.pi_ip && data.pi_ip.length >= 7) {
          const autoHost = data.pi_ip + ":" + (data.pi_port || 8000);
          document.getElementById('piOfflineNotice').style.display = "none";
          if (autoHost !== piHost || !camConnected) {
            window.applyPiHost(autoHost);
          }
        } else if (!camConnected && data.pi_online === 0) {
          document.getElementById('piOfflineNotice').style.display = "block";
        }
      } catch (e) {}
    };
    setInterval(pollSensors, 250);

    // Initialize from localStorage if saved
    window.addEventListener("DOMContentLoaded", () => {
      const saved = localStorage.getItem("pi_host");
      if (saved) {
        window.applyPiHost(saved);
      }
    });
      }
    });

    // Keyboard support
    window.addEventListener("keydown", e => {
      if (e.repeat) return;
      if (e.code === "KeyW") window.execCmd('forward');
      if (e.code === "KeyS") window.execCmd('backward');
      if (e.code === "KeyA") window.execCmd('left');
      if (e.code === "KeyD") window.execCmd('right');
      if (e.code === "Space") window.execCmd('stop');
    });

    window.addEventListener("keyup", e => {
      if (!isContinuous && ["KeyW", "KeyS", "KeyA", "KeyD"].includes(e.code)) {
        window.execCmd('stop');
      }
    });
  </script>
</body>
</html>
)rawliteral";

// ------------------------------------------------------------------------------
// HTTP REQUEST HANDLERS
// ------------------------------------------------------------------------------
void handleRoot() {
    server.send_P(200, "text/html", PAGE_HTML);
}

void handleCmd() {
    if (server.hasArg("spd")) {
        current_speed = constrain(server.arg("spd").toFloat(), 0.1f, 1.0f);
    }

    if (server.hasArg("act")) {
        String act = server.arg("act");

        if (act == "turn_left" || (act == "turn" && server.arg("dir") == "left")) {
            float deg = server.hasArg("deg") ? server.arg("deg").toFloat() : 90.0f;
            startTurn("left", deg);
        } else if (act == "turn_right" || (act == "turn" && server.arg("dir") == "right")) {
            float deg = server.hasArg("deg") ? server.arg("deg").toFloat() : 90.0f;
            startTurn("right", deg);
        } else if (act == "reset_yaw") {
            yaw_deg = 0.0f;
            turn_start_yaw = 0.0f;
            is_turning = false;
        } else {
            is_turning = false;
            applyCommand(act);
        }
    }

    server.send(200, "application/json", "{\"status\":\"ok\"}");
}

void handlePiHeartbeat() {
    if (server.hasArg("ip") && server.arg("ip").length() >= 7) {
        pi_ip_address = server.arg("ip");
    } else {
        pi_ip_address = server.client().remoteIP().toString();
    }
    if (server.hasArg("port")) {
        pi_web_port = server.arg("port").toInt();
    }
    pi_last_heartbeat_ms = millis();
    pi_online = true;
    server.send(200, "application/json", "{\"status\":\"ok\",\"pi_ip\":\"" + pi_ip_address + "\"}");
}

void handleTelemetry() {
    // If request originated from Raspberry Pi, register its IP immediately
    if (server.hasArg("src") && server.arg("src") == "pi") {
        if (server.hasArg("ip") && server.arg("ip").length() >= 7) {
            pi_ip_address = server.arg("ip");
        } else {
            pi_ip_address = server.client().remoteIP().toString();
        }
        if (server.hasArg("port")) {
            pi_web_port = server.arg("port").toInt();
        }
        pi_last_heartbeat_ms = millis();
        pi_online = true;
    } else {
        pi_online = (pi_ip_address.length() >= 7 && (millis() - pi_last_heartbeat_ms < 5000));
    }

    String json = "{";
    json += "\"yaw\":" + String(yaw_deg, 2) + ",";
    json += "\"pitch\":" + String(pitch_deg, 2) + ",";
    json += "\"roll\":" + String(roll_deg, 2) + ",";
    json += "\"mpu_ok\":" + String(mpu_ok ? 1 : 0) + ",";
    json += "\"gps_status\":\"" + gps_status + "\",";
    json += "\"gps_lat\":" + String(gps_lat, 6) + ",";
    json += "\"gps_lon\":" + String(gps_lon, 6) + ",";
    json += "\"gps_sats\":" + String(gps_sats) + ",";
    json += "\"motion\":\"" + current_motion + "\",";
    json += "\"speed\":" + String(current_speed, 2) + ",";
    json += "\"is_turning\":" + String(is_turning ? 1 : 0) + ",";
    json += "\"target_deg\":" + String(turn_target_degrees, 1) + ",";
    json += "\"pi_online\":" + String(pi_online ? 1 : 0) + ",";
    json += "\"pi_ip\":\"" + pi_ip_address + "\",";
    json += "\"pi_port\":" + String(pi_web_port);
    json += "}";
    server.send(200, "application/json", json);
}

// ------------------------------------------------------------------------------
// SETUP & MAIN LOOP
// ------------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(200);

    Serial.println("\n========================================================");
    Serial.println("RiceTrack — ESP32 AP Controller + MPU & GPS Guidance");
    Serial.println("========================================================");

    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);

    // 1. Initialize Motors
    initPWM();
    stopMotors();

    // 2. Initialize MPU-6050
    mpu_ok = mpuInit();
    Serial.print("MPU-6050 IMU: ");
    Serial.println(mpu_ok ? "READY (I2C SDA=21, SCL=22)" : "NOT FOUND (Will retry in loop)");

    // 3. Initialize NEO-6M GPS on UART2
    GPS_Serial.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
    Serial.println("NEO-6M GPS  : READY (UART2 RX=16, TX=17 @ 9600)");

    // 4. Start Wi-Fi Access Point
    WiFi.mode(WIFI_AP);
    WiFi.softAP("RiceTrack-Rover", "ricetrack123");
    IPAddress myIP = WiFi.softAPIP();

    Serial.println("--------------------------------------------------------");
    Serial.println("1. Connect Phone/Tablet to Wi-Fi:");
    Serial.println("   SSID     : RiceTrack-Rover");
    Serial.println("   Password : ricetrack123");
    Serial.print  ("2. Open in browser : http://");
    Serial.println(myIP);
    Serial.println("========================================================\n");

    server.on("/", handleRoot);
    server.on("/cmd", handleCmd);
    server.on("/telemetry", handleTelemetry);
    server.on("/pi_heartbeat", handlePiHeartbeat);
    server.begin();

    digitalWrite(PIN_LED, LOW);
}

void loop() {
    server.handleClient();

    // 1. Update MPU-6050 Readings
    if (!mpu_ok) {
        static uint32_t last_retry = 0;
        if (millis() - last_retry >= 2000) {
            last_retry = millis();
            mpu_ok = mpuInit();
        }
    } else {
        mpuUpdate();
    }

    // 2. Poll GPS NMEA Stream
    gpsPoll();

    // 3. Closed-Loop Turn Controller (MPU Target Angle Guidance)
    if (is_turning) {
        updateTurn();
    }

    // 4. Broadcast IMU Telemetry over Serial @ 20 Hz
    uint32_t now = millis();
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

    // 5. Broadcast GPS Telemetry over Serial @ 1 Hz
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

    // 6. Check incoming Serial Commands from Raspberry Pi
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            if (serial_rx_buf.length() > 0) {
                // Parse simple serial JSON
                if (serial_rx_buf.indexOf("\"c\":\"m\"") != -1) {
                    int l_idx = serial_rx_buf.indexOf("\"l\":");
                    int r_idx = serial_rx_buf.indexOf("\"r\":");
                    if (l_idx != -1 && r_idx != -1) {
                        float l = serial_rx_buf.substring(l_idx + 4).toFloat();
                        float r = serial_rx_buf.substring(r_idx + 4).toFloat();
                        is_turning = false;
                        setMotors(l, r);
                    }
                } else if (serial_rx_buf.indexOf("\"c\":\"s\"") != -1) {
                    stopMotors();
                } else if (serial_rx_buf.indexOf("\"c\":\"tl\"") != -1) {
                    int d_idx = serial_rx_buf.indexOf("\"deg\":");
                    float deg = (d_idx != -1) ? serial_rx_buf.substring(d_idx + 6).toFloat() : 90.0f;
                    startTurn("left", deg);
                } else if (serial_rx_buf.indexOf("\"c\":\"tr\"") != -1) {
                    int d_idx = serial_rx_buf.indexOf("\"deg\":");
                    float deg = (d_idx != -1) ? serial_rx_buf.substring(d_idx + 6).toFloat() : 90.0f;
                    startTurn("right", deg);
                } else if (serial_rx_buf.indexOf("\"c\":\"ry\"") != -1) {
                    yaw_deg = 0.0f;
                }
            }
            serial_rx_buf = "";
        } else {
            if (serial_rx_buf.length() < 120) {
                serial_rx_buf += c;
            }
        }
    }
}
