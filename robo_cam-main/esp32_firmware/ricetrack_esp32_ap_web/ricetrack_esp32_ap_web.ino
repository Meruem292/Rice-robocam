/* ==============================================================================
 * RiceTrack Rover — Integrated ESP32 AP Controller + Raspberry Pi Camera Display
 * File: ricetrack_esp32_ap_web.ino
 * ==============================================================================
 * Architecture:
 * 1. ESP32 hosts its own high-speed Wi-Fi Access Point:
 *      SSID     : RiceTrack-Rover
 *      Password : ricetrack123
 *      IP       : 192.168.4.1
 * 2. Motor commands execute directly on the ESP32 (0ms latency, zero serial bugs!).
 * 3. Raspberry Pi connects to this Wi-Fi AP at 192.168.4.2.
 * 4. The ESP32 dashboard embeds the Raspberry Pi's live camera stream:
 *      http://192.168.4.2:8000/api/video_feed
 *    and provides one-click AI Crop Health Evaluation!
 *
 * Pinout:
 *   Left BTS7960:  RPWM -> GPIO 32, LPWM -> GPIO 33, R_EN -> GPIO 25, L_EN -> GPIO 26
 *   Right BTS7960: RPWM -> GPIO 18, LPWM -> GPIO 19, R_EN -> GPIO 27, L_EN -> GPIO 14
 *   Status LED:    GPIO 2
 * ============================================================================== */

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <math.h>

// ------------------------------------------------------------------------------
// PIN DEFINITIONS
// ------------------------------------------------------------------------------
const int PIN_LEFT_RPWM  = 32;
const int PIN_LEFT_LPWM  = 33;
const int PIN_LEFT_R_EN  = 25;
const int PIN_LEFT_L_EN  = 26;

const int PIN_RIGHT_RPWM = 18;
const int PIN_RIGHT_LPWM = 19;
const int PIN_RIGHT_R_EN = 27;
const int PIN_RIGHT_L_EN = 14;

const int PIN_LED        = 2;

// MPU-6050 (I2C)
const int PIN_I2C_SDA    = 21;
const int PIN_I2C_SCL    = 22;
const uint8_t MPU_ADDR   = 0x68;

// PWM Parameters
const int PWM_FREQ       = 1000;  // 1 kHz
const int PWM_RESOLUTION = 10;    // 10-bit (0 - 1023)
const int PWM_MAX        = 1023;

WebServer server(80);

// Motor State variables
float current_speed = 0.75f;  // 75% default speed
String current_motion = "STOP";
float left_duty = 0.0f;
float right_duty = 0.0f;

// MPU-6050 State variables
bool mpu_ok = false;
float roll_deg = 0.0f;
float pitch_deg = 0.0f;
float yaw_deg = 0.0f;
float gz_bias = 0.0f;
float comp_roll = 0.0f;
float comp_pitch = 0.0f;
uint32_t last_imu_time_us = 0;
uint32_t last_imu_tx_ms = 0;

// Closed-Loop Turn Control State Machine
bool is_turning_closed_loop = false;
float turn_target_degrees = 90.0f;
float turn_accumulated_degrees = 0.0f;
String turn_active_dir = "left";
uint32_t turn_start_time_ms = 0;
uint32_t turn_timeout_time_ms = 0;
int turn_state = 0;  // 0: idle, 1: turning, 2: reverse brake, 3: settle, 4: micro-nudge
uint32_t turn_brake_time_ms = 0;

// Straight-Line Gyro Compass PID Heading Lock
bool heading_lock_enabled = true;
bool heading_lock_active = false;
float heading_lock_target = 0.0f;
float heading_err_integral = 0.0f;
float heading_last_err = 0.0f;
float gyro_z_sign = 1.0f;            // +1.0 (chip-up: Left = +Yaw), -1.0 (if MPU mounted inverted)
const float HEADING_KP = 0.035f;     // Immediate centering stiffness (3.5% PWM per deg)
const float HEADING_KI = 0.015f;     // Eliminates steady-state motor mismatch / wheel drag
const float HEADING_KD = 0.008f;     // Gyro rate damping against sudden bumps/terrain pull
const float MAX_STEER_TRIM = 0.35f;  // Up to 35% PWM differential authority

// ------------------------------------------------------------------------------
// PWM CONTROL
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
    heading_lock_active = false;
    heading_err_integral = 0.0f;
    heading_last_err = 0.0f;
    setMotors(0.0f, 0.0f);
    current_motion = "STOP";
}

void applyCommand(const String& cmd) {
    current_motion = cmd;
    float spd = current_speed;

    if (cmd == "forward") {
        heading_lock_target = yaw_deg;
        heading_err_integral = 0.0f;
        heading_last_err = 0.0f;
        heading_lock_active = heading_lock_enabled && mpu_ok;
        setMotors(spd, spd);
    } else if (cmd == "backward") {
        heading_lock_target = yaw_deg;
        heading_err_integral = 0.0f;
        heading_last_err = 0.0f;
        heading_lock_active = heading_lock_enabled && mpu_ok;
        setMotors(-spd, -spd);
    } else {
        heading_lock_active = false;
        heading_err_integral = 0.0f;
        if (cmd == "left") {
            setMotors(-spd, spd);  // Left reverse, Right forward -> Pivot Left (CCW)
        } else if (cmd == "right") {
            setMotors(spd, -spd);  // Left forward, Right reverse -> Pivot Right (CW)
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
}

// ------------------------------------------------------------------------------
// MPU-6050 CLOSED-LOOP DEGREE TURNING (Precision State Machine + Active Brake)
// ------------------------------------------------------------------------------
void startClosedLoopTurn(const String& dir, float degrees) {
    if (degrees <= 1.0f) return;
    is_turning_closed_loop = true;
    turn_active_dir = dir;
    turn_target_degrees = degrees;
    turn_accumulated_degrees = 0.0f;
    turn_start_time_ms = millis();
    turn_timeout_time_ms = (uint32_t)(degrees * 95 + 3500); // 3.5s baseline + 95ms/deg
    turn_state = 1; // 1: Active turning phase
    heading_lock_active = false;
    heading_err_integral = 0.0f;

    // Controlled cruise speed (0.42) to cut angular kinetic momentum by >50%
    float initial_spd = 0.42f;
    if (dir == "left") {
        setMotors(-initial_spd, initial_spd);
    } else {
        setMotors(initial_spd, -initial_spd);
    }
    current_motion = "TURN_" + dir + "_" + String((int)degrees);
}

void updateClosedLoopTurn(float gz_dps, float dt) {
    if (!is_turning_closed_loop) return;

    uint32_t now = millis();
    uint32_t elapsed = now - turn_start_time_ms;

    // Safety timeout check
    if (elapsed >= turn_timeout_time_ms) {
        stopMotors();
        is_turning_closed_loop = false;
        turn_state = 0;
        heading_lock_target = yaw_deg;
        Serial.print("{\"t\":\"turn_done\",\"accum\":");
        Serial.print(turn_accumulated_degrees, 1);
        Serial.println(",\"timeout\":true}");
        return;
    }

    if (turn_state == 1) { // ACTIVE TURNING WITH DYNAMIC DECELERATION
        // Integrate angular turn velocity
        turn_accumulated_degrees += fabs(gz_dps) * dt;
        float remaining = turn_target_degrees - turn_accumulated_degrees;

        // Dynamic lead angle cutoff for active reverse brake
        float lead_deg = (turn_target_degrees < 45.0f) ? 2.0f : 3.2f;

        if (remaining <= lead_deg) {
            // PHASE 2: ACTIVE DYNAMIC REVERSE BRAKE PULSE (Reverse Plug)
            // Fire short reverse pulse (45ms) to immediately cancel motor gearbox inertia
            turn_state = 2;
            turn_brake_time_ms = now;
            float brake_pwr = 0.32f;
            if (turn_active_dir == "left") {
                setMotors(brake_pwr, -brake_pwr); // Reverse torque against Left turn
            } else {
                setMotors(-brake_pwr, brake_pwr); // Reverse torque against Right turn
            }
            return;
        }

        // DYNAMIC DECELERATION PROFILE:
        // Cruise (> 35° remaining): 0.42
        // Ramp down (12° to 35° remaining): smooth linear deceleration to 0.27
        // Final approach (<= 12° remaining): crawl at 0.27 to eliminate momentum
        float spd = 0.42f;
        if (remaining <= 12.0f) {
            spd = 0.27f;
        } else if (remaining <= 35.0f) {
            float ratio = (remaining - 12.0f) / 23.0f;
            spd = 0.27f + ratio * (0.42f - 0.27f);
        }

        if (turn_active_dir == "left") {
            setMotors(-spd, spd);
        } else {
            setMotors(spd, -spd);
        }
    }
    else if (turn_state == 2) { // ACTIVE REVERSE BRAKE PHASE (45ms pulse)
        if (now - turn_brake_time_ms >= 45) {
            setMotors(0.0f, 0.0f); // Cut motor power completely
            turn_state = 3;        // Transition to settle phase
            turn_brake_time_ms = now;
        }
    }
    else if (turn_state == 3) { // SETTLE PHASE (120ms to allow physical chassis to rest)
        if (now - turn_brake_time_ms >= 120) {
            float final_err = turn_target_degrees - turn_accumulated_degrees;
            // If under-turned by > 2.5 degrees, give a tiny 50ms crawl nudge
            if (final_err > 2.5f) {
                turn_state = 4;
                turn_brake_time_ms = now;
                float nudge_spd = 0.27f;
                if (turn_active_dir == "left") {
                    setMotors(-nudge_spd, nudge_spd);
                } else {
                    setMotors(nudge_spd, -nudge_spd);
                }
                return;
            }

            // Turn successfully finished
            stopMotors();
            is_turning_closed_loop = false;
            turn_state = 0;
            heading_lock_target = yaw_deg; // Store new reference heading for subsequent straight driving
            Serial.print("{\"t\":\"turn_done\",\"accum\":");
            Serial.print(turn_accumulated_degrees, 1);
            Serial.print(",\"target\":");
            Serial.print(turn_target_degrees, 1);
            Serial.println("}");
        }
    }
    else if (turn_state == 4) { // MICRO-NUDGE PHASE (50ms crawl)
        turn_accumulated_degrees += fabs(gz_dps) * dt;
        if (now - turn_brake_time_ms >= 50) {
            stopMotors();
            is_turning_closed_loop = false;
            turn_state = 0;
            heading_lock_target = yaw_deg;
            Serial.print("{\"t\":\"turn_done\",\"accum\":");
            Serial.print(turn_accumulated_degrees, 1);
            Serial.print(",\"target\":");
            Serial.print(turn_target_degrees, 1);
            Serial.println("}");
        }
    }
}

// ------------------------------------------------------------------------------
// MPU-6050 IMU ROUTINES (Native Wire.h — 400kHz Fast I2C)
// ------------------------------------------------------------------------------
bool mpuInit() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
    delay(20);

    // Wake up MPU-6050
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0x00);
    if (Wire.endTransmission() != 0) {
        return false;
    }

    // Enable 42 Hz Digital Low-Pass Filter (DLPF) to reject motor/gearbox vibration noise
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1A);
    Wire.write(0x03);
    Wire.endTransmission();

    // Accelerometer +/- 4g
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1C);
    Wire.write(0x08);
    Wire.endTransmission();

    // Gyroscope +/- 500 deg/s
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1B);
    Wire.write(0x08);
    Wire.endTransmission();

    delay(25);
    last_imu_time_us = micros();

    // Calibrate stationary gyro bias (200 samples for rock-solid zero-drift compass)
    long gz_sum = 0;
    int samples = 0;
    for (int i = 0; i < 200; i++) {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x47);
        if (Wire.endTransmission(false) == 0) {
            if (Wire.requestFrom((uint16_t)MPU_ADDR, (uint8_t)2, (bool)true) == 2) {
                int16_t gz_raw = (Wire.read() << 8) | Wire.read();
                gz_sum += gz_raw;
                samples++;
            }
        }
        delay(2);
    }
    if (samples > 0) {
        gz_bias = (float)gz_sum / (float)samples;
    }
    yaw_deg = 0.0f;
    heading_lock_target = 0.0f;
    heading_err_integral = 0.0f;
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
    float gz_dps = (((float)gz - gz_bias) / 65.5f) * gyro_z_sign;
    if (fabs(gz_dps) < 0.25f) gz_dps = 0.0f; // Deadband filter against stationary drift

    float accel_roll = atan2(ay_g, az_g) * 180.0f / M_PI;
    float accel_pitch = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180.0f / M_PI;

    uint32_t now_us = micros();
    float dt = (float)(now_us - last_imu_time_us) / 1000000.0f;
    if (dt <= 0.0f || dt > 0.5f) dt = 0.02f;
    last_imu_time_us = now_us;

    comp_roll = 0.96f * (comp_roll + gx_dps * dt) + 0.04f * accel_roll;
    comp_pitch = 0.96f * (comp_pitch + gy_dps * dt) + 0.04f * accel_pitch;
    yaw_deg += gz_dps * dt;

    roll_deg = comp_roll;
    pitch_deg = comp_pitch;

    if (is_turning_closed_loop) {
        updateClosedLoopTurn(gz_dps, dt);
    }
    // Straight-Line Gyro Compass PID Heading Lock (Maintains Dead Center)
    else if (heading_lock_active && mpu_ok) {
        float err = yaw_deg - heading_lock_target;
        // Normalize error to [-180, +180]
        while (err > 180.0f) err -= 360.0f;
        while (err < -180.0f) err += 360.0f;
        heading_last_err = err;

        // Tight deadband: ignore sub-0.25° mechanical vibration
        float active_err = (fabs(err) < 0.25f) ? 0.0f : err;

        // Integral accumulator with anti-windup clamp (cancels permanent motor/wheel drag)
        heading_err_integral += active_err * dt;
        heading_err_integral = constrain(heading_err_integral, -12.0f, 12.0f);

        // Full PID Compass output:
        // P: immediate pull back to dead center
        // I: compensates uneven motor RPM / tire drag so steady-state error = 0.0°
        // D: uses live gz_dps to damp sudden bumps & mud pull before heading drifts
        float trim = (active_err * HEADING_KP)
                   + (heading_err_integral * HEADING_KI)
                   + (gz_dps * HEADING_KD);
        trim = constrain(trim, -MAX_STEER_TRIM, MAX_STEER_TRIM);

        float base_spd = current_speed;
        if (current_motion == "forward") {
            // Veering LEFT -> yaw increases (err > 0, trim > 0)
            // Steer RIGHT back to dead center: Left faster (+trim), Right slower (-trim)
            float l_spd = base_spd + trim;
            float r_spd = base_spd - trim;
            // Preserve full differential steering authority even at 100% throttle
            if (l_spd > 1.0f) { r_spd -= (l_spd - 1.0f); l_spd = 1.0f; }
            if (r_spd > 1.0f) { l_spd -= (r_spd - 1.0f); r_spd = 1.0f; }
            l_spd = constrain(l_spd, 0.15f, 1.0f);
            r_spd = constrain(r_spd, 0.15f, 1.0f);
            setMotors(l_spd, r_spd);
        } else if (current_motion == "backward") {
            // In reverse: CW correcting torque is still Left more positive (+trim), Right more negative (-trim)
            float l_spd = -base_spd + trim;
            float r_spd = -base_spd - trim;
            if (l_spd < -1.0f) { r_spd -= (l_spd + 1.0f); l_spd = -1.0f; }
            if (r_spd < -1.0f) { l_spd -= (r_spd + 1.0f); r_spd = -1.0f; }
            l_spd = constrain(l_spd, -1.0f, -0.15f);
            r_spd = constrain(r_spd, -1.0f, -0.15f);
            setMotors(l_spd, r_spd);
        }
    }
}

// ------------------------------------------------------------------------------
// SERIAL UART COMMAND & TELEMETRY PROCESSOR
// ------------------------------------------------------------------------------
void processSerial() {
    while (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) continue;

        if (line.indexOf("\"c\":\"ry\"") != -1 || line.indexOf("\"c\": \"ry\"") != -1) {
            yaw_deg = 0.0f;
            heading_lock_target = 0.0f;
            heading_err_integral = 0.0f;
            Serial.println("{\"t\":\"ack\",\"cmd\":\"ry\"}");
        } else if (line.indexOf("\"c\":\"s\"") != -1 || line.indexOf("\"c\": \"s\"") != -1) {
            stopMotors();
            is_turning_closed_loop = false;
        } else if (line.indexOf("\"c\":\"m\"") != -1 || line.indexOf("\"c\": \"m\"") != -1) {
            int l_idx = line.indexOf("\"l\":");
            int r_idx = line.indexOf("\"r\":");
            if (l_idx != -1 && r_idx != -1) {
                float l = line.substring(l_idx + 4).toFloat();
                float r = line.substring(r_idx + 4).toFloat();
                // If both motors are commanded equally (forward/backward drive), engage gyro compass lock!
                if (fabs(l - r) < 0.05f && fabs(l) > 0.15f) {
                    current_speed = fabs(l);
                    current_motion = (l > 0) ? "forward" : "backward";
                    heading_lock_target = yaw_deg;
                    heading_err_integral = 0.0f;
                    heading_lock_active = heading_lock_enabled && mpu_ok;
                    setMotors(l, r);
                } else {
                    heading_lock_active = false;
                    heading_err_integral = 0.0f;
                    setMotors(l, r);
                }
            }
        } else if (line.indexOf("\"c\":\"f\"") != -1 || line.indexOf("\"c\": \"f\"") != -1) {
            applyCommand("forward");
        } else if (line.indexOf("\"c\":\"b\"") != -1 || line.indexOf("\"c\": \"b\"") != -1) {
            applyCommand("backward");
        } else if (line.indexOf("\"c\":\"t\"") != -1 || line.indexOf("\"c\": \"t\"") != -1) {
            String dir = "left";
            if (line.indexOf("\"right\"") != -1) dir = "right";
            float deg = 90.0f;
            int d_idx = line.indexOf("\"deg\":");
            if (d_idx != -1) deg = line.substring(d_idx + 6).toFloat();
            startClosedLoopTurn(dir, deg);
        } else if (line.indexOf("\"c\":\"p\"") != -1 || line.indexOf("\"c\": \"p\"") != -1) {
            Serial.println("{\"t\":\"k\"}");
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

    /* CAMERA CARD */
    .cam-card {
      background: #161b22; border: 1px solid #30363d; border-radius: 10px;
      padding: 8px; margin-bottom: 12px; text-align: left;
    }
    .cam-header { display: flex; justify-content: space-between; font-size: 12px; color: #8b949e; margin-bottom: 6px; }
    .cam-box {
      width: 100%; height: 240px; background: #000; border-radius: 6px;
      display: flex; align-items: center; justify-content: center; overflow: hidden; position: relative;
    }
    .cam-box img { width: 100%; height: 100%; object-fit: contain; }
    .cam-overlay-btn {
      margin-top: 8px; display: flex; gap: 8px;
    }

    /* STATUS CARD */
    .status-card {
      background: #161b22; border: 1px solid #30363d; border-radius: 8px;
      padding: 8px; margin-bottom: 10px; font-size: 13px; display: flex; justify-content: space-around;
    }
    .status-val { font-weight: bold; color: #58a6ff; }

    /* EVAL RESULT CARD */
    #evalBox {
      display: none; background: #0f2d1e; border: 1px solid #2ea043; border-radius: 8px;
      padding: 10px; margin-bottom: 12px; text-align: left; font-size: 13px; color: #7ee787;
    }

    /* BUTTONS & DPAD */
    .btn-group { display: flex; justify-content: center; gap: 6px; margin-bottom: 10px; }
    .spd-btn {
      background: #21262d; color: #c9d1d9; border: 1px solid #30363d; border-radius: 6px;
      padding: 6px 12px; font-weight: bold; font-size: 13px; cursor: pointer; flex: 1;
    }
    .spd-btn.active { background: #238636; border-color: #3fb950; color: #fff; }

    .dpad-wrap { display: flex; justify-content: center; margin: 8px 0 12px; }
    .dpad {
      display: grid; grid-template-columns: 85px 85px 85px; grid-template-rows: 75px 75px 75px; gap: 8px;
    }
    .btn {
      background: #21262d; color: #f0f6fc; border: 1px solid #30363d; border-radius: 12px;
      font-size: 24px; font-weight: bold; display: flex; align-items: center; justify-content: center;
      cursor: pointer; transition: all 0.08s;
    }
    .btn:active { background: #388bfd; transform: scale(0.95); }
    .btn-stop { background: #da3633; border-color: #f85149; font-size: 18px; }
    .btn-stop:active { background: #b62324; }

    .btn-action {
      background: #238636; color: #fff; border: 1px solid #3fb950; border-radius: 6px;
      padding: 10px 14px; font-weight: 700; font-size: 14px; cursor: pointer; flex: 1;
    }
    .btn-action:active { background: #2ea043; }

    .btn-secondary {
      background: #21262d; color: #58a6ff; border: 1px solid #30363d; border-radius: 6px;
      padding: 8px 12px; font-size: 12px; font-weight: 600; text-decoration: none; display: inline-block;
    }

    .mode-wrap { margin-top: 8px; font-size: 13px; color: #8b949e; }
  </style>
</head>
<body>
  <div class="header">
    <h2>🌾 RiceTrack Rover</h2>
    <span id="badgeLink" class="badge green">ESP32 Direct Control</span>
  </div>

  <!-- RASPBERRY PI LIVE CAMERA -->
  <div class="cam-card">
    <div class="cam-header">
      <span>📷 Live Camera</span>
      <span id="camStatus" style="color:#e3b341; font-weight:bold;">Connecting…</span>
    </div>

    <!-- PI ADDRESS & AUTO-FIND BAR -->
    <div style="display:flex; gap:6px; margin-bottom:6px; align-items:center; background:#21262d; padding:6px 8px; border-radius:6px; flex-wrap:wrap;">
      <span style="font-size:12px; color:#8b949e;">Pi Host:</span>
      <input id="inpPiHost" type="text" value="192.168.4.50:8000" style="background:#0d1117; color:#58a6ff; border:1px solid #30363d; border-radius:4px; padding:3px 6px; width:135px; font-size:12px; font-weight:bold;">
      <button onclick="applyPiHost()" style="background:#238636; color:#fff; border:none; border-radius:4px; padding:4px 10px; font-size:12px; font-weight:bold; cursor:pointer;">Connect</button>
      <button onclick="autoScanPi()" style="background:#1f6feb; color:#fff; border:none; border-radius:4px; padding:4px 8px; font-size:11px; font-weight:600; cursor:pointer;">🔍 Auto-Find</button>
    </div>
    <div id="stationAlert" style="font-size:11px; color:#8b949e; margin-bottom:8px; text-align:left;">
      Wi-Fi Devices: <span id="lblStations" style="color:#58a6ff; font-weight:bold;">Checking…</span>
    </div>

    <div class="cam-box">
      <img id="camStream" 
           src="http://192.168.4.50:8000/api/video_feed"
           onload="onCamConnected()"
           onerror="onCamError()">
    </div>
    <div class="cam-overlay-btn">
      <button class="btn-action" onclick="captureEvaluate()">📸 Capture &amp; Evaluate Crop</button>
      <a id="btnFullDash" href="http://192.168.4.50:8000" target="_blank" class="btn-secondary">📊 Full Dashboard</a>
    </div>
  </div>

  <!-- AI EVALUATION RESULT -->
  <div id="evalBox">
    <div style="font-weight:bold; font-size:14px; margin-bottom:4px;" id="evalTitle">🌱 Crop Health: Evaluating…</div>
    <div id="evalDetail" style="font-size:12px; color:#c9d1d9;">Connecting to Raspberry Pi vision model…</div>
  </div>

  <!-- MOTOR & HEADING LOCK STATUS -->
  <div class="status-card">
    <div>Motion: <span id="lblMotion" class="status-val">STOP</span></div>
    <div>Speed: <span id="lblSpd" class="status-val">75%</span></div>
    <div>Gyro Lock: <span id="lblLock" class="status-val" style="color:#8b949e;">STANDBY</span></div>
  </div>

  <!-- MPU-6050 COMPASS & CLOSED-LOOP 90° TURN CARD -->
  <div class="status-card" style="flex-direction:column; gap:6px;">
    <div style="display:flex; justify-content:space-around; width:100%;">
      <div>🧭 Yaw: <span id="lblYaw" class="status-val">0.0°</span></div>
      <div>Pitch: <span id="lblPitch" class="status-val">0.0°</span></div>
      <div>Roll: <span id="lblRoll" class="status-val">0.0°</span></div>
    </div>
    <div style="display:flex; justify-content:space-around; width:100%; font-size:11px; color:#8b949e;">
      <div>L-PWM: <span id="lblLPwm" style="color:#58a6ff; font-weight:bold;">0%</span></div>
      <div>R-PWM: <span id="lblRPwm" style="color:#58a6ff; font-weight:bold;">0%</span></div>
      <div>Err: <span id="lblErr" style="color:#3fb950; font-weight:bold;">0.0°</span></div>
    </div>
    <div style="display:flex; gap:6px; width:100%; justify-content:center; margin-top:2px;">
      <button class="spd-btn" style="flex:1;" onclick="turnDeg('left', 90)">↶ Turn 90° L</button>
      <button class="spd-btn" style="flex:1;" onclick="turnDeg('right', 90)">↷ Turn 90° R</button>
      <button class="spd-btn" style="flex:0.55; color:#8b949e;" onclick="resetYaw()">↺ Zero</button>
      <button class="spd-btn" style="flex:0.55; color:#e3b341;" onclick="flipGyro()" title="Flip MPU-6050 Z-axis if mounted inverted">🔄 Flip</button>
    </div>
  </div>

  <!-- SPEED SELECTOR -->
  <div class="btn-group">
    <button class="spd-btn" onclick="setSpeed(0.40)">40%</button>
    <button class="spd-btn" onclick="setSpeed(0.60)">60%</button>
    <button class="spd-btn active" onclick="setSpeed(0.75)">75%</button>
    <button class="spd-btn" onclick="setSpeed(0.85)">85%</button>
    <button class="spd-btn" onclick="setSpeed(1.00)">100%</button>
  </div>

  <!-- DRIVE PAD -->
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

  <div class="mode-wrap">
    <label style="cursor:pointer;">
      <input type="checkbox" id="chkContinuous" onchange="toggleContinuous(this.checked)" checked>
      <b>Continuous Mode</b> (Click to run, Click STOP to halt)
    </label>
  </div>

  <script>
    let isContinuous = true;
    let curSpeed = 0.75;
    let piHost = localStorage.getItem("pi_host") || "192.168.4.50:8000";
    if (!piHost || piHost.startsWith("192.168.4.1")) {
      piHost = "192.168.4.50:8000";
      localStorage.setItem("pi_host", piHost);
    }

    const candidateHosts = [
      "192.168.4.50:8000",
      "192.168.4.2:8000",
      "192.168.4.3:8000",
      "192.168.4.4:8000",
      "192.168.4.5:8000",
      "192.168.4.50:8001",
      "192.168.4.2:8001",
      "192.168.4.3:8001"
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
      if (piHost.startsWith("192.168.4.1")) {
        alert("Notice: 192.168.4.1 is the ESP32! The Raspberry Pi is at 192.168.4.50 (or 192.168.4.2 / 192.168.4.3).");
        piHost = "192.168.4.50:8000";
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
      document.getElementById('camStatus').textContent = "ONLINE (" + piHost + ")";
      document.getElementById('camStatus').style.color = "#2ea043";
    };

    let autoFindDone = false;
    window.onCamError = () => {
      document.getElementById('camStatus').textContent = "Standby / Checking…";
      document.getElementById('camStatus').style.color = "#e3b341";
      if (!autoFindDone) {
        autoFindDone = true;
        setTimeout(window.autoScanPi, 1000);
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
    };

    window.toggleContinuous = (val) => {
      isContinuous = val;
    };

    window.setSpeed = (spd) => {
      curSpeed = spd;
      document.querySelectorAll('.spd-btn').forEach(b => {
        b.classList.toggle('active', b.textContent === Math.round(spd * 100) + '%');
      });
      document.getElementById('lblSpd').textContent = Math.round(spd * 100) + '%';
      fetch('/cmd?spd=' + spd).catch(()=>{});
    };

    window.execCmd = (act) => {
      document.getElementById('lblMotion').textContent = act.toUpperCase();
      fetch('/cmd?act=' + act + '&spd=' + curSpeed).catch(()=>{});
    };

    window.pressBtn = (act) => {
      window.execCmd(act);
    };

    window.releaseBtn = () => {
      if (!isContinuous) {
        window.execCmd('stop');
      }
    };

    // AI CROP HEALTH EVALUATION FROM PI
    window.captureEvaluate = async () => {
      const box = document.getElementById('evalBox');
      const title = document.getElementById('evalTitle');
      const detail = document.getElementById('evalDetail');
      box.style.display = "block";
      title.textContent = "📸 Capturing & Analyzing Crop…";
      detail.textContent = "Running IRRI Leaf Color Chart (LCC) & ExG model…";

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

    // Closed-loop degree turn trigger
    window.turnDeg = (dir, deg) => {
      document.getElementById('lblMotion').textContent = ("TURN " + dir + " " + deg + "°").toUpperCase();
      fetch('/cmd?act=turn_deg&dir=' + dir + '&deg=' + deg).catch(()=>{});
    };

    window.resetYaw = () => {
      fetch('/cmd?act=reset_yaw').catch(()=>{});
      document.getElementById('lblYaw').textContent = "0.0°";
    };

    window.flipGyro = () => {
      fetch('/cmd?act=flip_gyro').catch(()=>{});
    };

    // Live MPU Telemetry Poller (Heading, Pitch, Roll, Live PWM & Compass Error)
    setInterval(async () => {
      try {
        const res = await fetch('/telemetry');
        const data = await res.json();
        if (data) {
          document.getElementById('lblYaw').textContent = (data.yaw != null ? data.yaw.toFixed(1) : "0.0") + "°";
          document.getElementById('lblPitch').textContent = (data.pitch != null ? data.pitch.toFixed(1) : "0.0") + "°";
          document.getElementById('lblRoll').textContent = (data.roll != null ? data.roll.toFixed(1) : "0.0") + "°";
          if (data.l_pwm != null) document.getElementById('lblLPwm').textContent = Math.round(data.l_pwm * 100) + "%";
          if (data.r_pwm != null) document.getElementById('lblRPwm').textContent = Math.round(data.r_pwm * 100) + "%";
          if (data.err != null) {
            const errEl = document.getElementById('lblErr');
            errEl.textContent = (data.err >= 0 ? "+" : "") + data.err.toFixed(1) + "°";
            errEl.style.color = Math.abs(data.err) <= 1.0 ? "#3fb950" : "#e3b341";
          }
          if (data.turning) {
            document.getElementById('lblMotion').textContent = "TURNING (" + (data.accum ? data.accum.toFixed(0) : "0") + "°)";
          }
          const lockEl = document.getElementById('lblLock');
          if (lockEl) {
            if (data.locked) {
              lockEl.textContent = "LOCKED (" + (data.target_yaw != null ? data.target_yaw.toFixed(1) : "0.0") + "°)";
              lockEl.style.color = "#3fb950";
            } else {
              lockEl.textContent = "STANDBY";
              lockEl.style.color = "#8b949e";
            }
          }
          if (data.stations != null) {
            const stTxt = document.getElementById('lblStations');
            if (stTxt) {
              if (data.stations >= 2) {
                stTxt.textContent = data.stations + " (Phone + Pi Connected ✅)";
                stTxt.style.color = "#2ea043";
              } else {
                stTxt.textContent = data.stations + " (Phone only · Pi NOT connected ⚠️)";
                stTxt.style.color = "#e3b341";
              }
            }
          }
        }
      } catch(e) {}
    }, 280);

    // Initialize from localStorage if saved
    window.addEventListener("DOMContentLoaded", () => {
      const saved = localStorage.getItem("pi_host");
      if (saved && !saved.startsWith("192.168.4.1")) {
        window.applyPiHost(saved);
      }
    });

    // Keyboard support
    window.addEventListener("keydown", e => {
      if (e.repeat) return;
      if (e.code === "KeyW") window.execCmd('forward');
      if (e.code === "KeyS") window.execCmd('backward');
      if (e.code === "KeyA") window.execCmd('left');
      if (e.code === "KeyD") window.execCmd('right');
      if (e.code === "KeyQ") window.turnDeg('left', 90);
      if (e.code === "KeyE") window.turnDeg('right', 90);
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

void handleTelemetry() {
    String json = "{";
    json += "\"yaw\":" + String(yaw_deg, 2) + ",";
    json += "\"pitch\":" + String(pitch_deg, 2) + ",";
    json += "\"roll\":" + String(roll_deg, 2) + ",";
    json += "\"mpu_ok\":" + String(mpu_ok ? "true" : "false") + ",";
    json += "\"turning\":" + String(is_turning_closed_loop ? "true" : "false") + ",";
    json += "\"accum\":" + String(turn_accumulated_degrees, 1) + ",";
    json += "\"locked\":" + String(heading_lock_active ? "true" : "false") + ",";
    json += "\"target_yaw\":" + String(heading_lock_target, 1) + ",";
    json += "\"err\":" + String(heading_last_err, 2) + ",";
    json += "\"l_pwm\":" + String(left_duty, 2) + ",";
    json += "\"r_pwm\":" + String(right_duty, 2) + ",";
    json += "\"gyro_sign\":" + String(gyro_z_sign, 0) + ",";
    json += "\"motion\":\"" + current_motion + "\",";
    json += "\"speed\":" + String(current_speed, 2) + ",";
    json += "\"stations\":" + String(WiFi.softAPgetStationNum());
    json += "}";
    server.send(200, "application/json", json);
}

void handleCmd() {
    if (server.hasArg("spd")) {
        current_speed = constrain(server.arg("spd").toFloat(), 0.1f, 1.0f);
    }

    if (server.hasArg("act")) {
        String act = server.arg("act");
        if (act == "turn_deg") {
            String dir = server.hasArg("dir") ? server.arg("dir") : "left";
            float deg = server.hasArg("deg") ? server.arg("deg").toFloat() : 90.0f;
            startClosedLoopTurn(dir, deg);
        } else if (act == "reset_yaw") {
            yaw_deg = 0.0f;
            heading_lock_target = 0.0f;
            heading_err_integral = 0.0f;
            heading_last_err = 0.0f;
        } else if (act == "flip_gyro") {
            gyro_z_sign = -gyro_z_sign;
            heading_err_integral = 0.0f;
        } else if (act == "toggle_lock") {
            heading_lock_enabled = !heading_lock_enabled;
            if (!heading_lock_enabled) heading_lock_active = false;
        } else {
            is_turning_closed_loop = false;
            applyCommand(act);
        }
    }

    server.send(200, "application/json", "{\"status\":\"ok\"}");
}

// ------------------------------------------------------------------------------
// SETUP & MAIN LOOP
// ------------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(300);

    Serial.println("\n========================================================");
    Serial.println("RiceTrack — ESP32 AP Controller + Camera Display + MPU");
    Serial.println("========================================================");

    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);

    initPWM();
    stopMotors();

    // Initialize MPU-6050
    mpu_ok = mpuInit();
    if (mpu_ok) {
        Serial.println("✅ MPU-6050 Gyro/Accel initialized (I2C calibrated)");
    } else {
        Serial.println("⚠️ MPU-6050 not detected on I2C (SDA=21, SCL=22)");
    }

    // Broadcast Wi-Fi AP
    WiFi.mode(WIFI_AP);
    WiFi.softAP("RiceTrack-Rover", "ricetrack123");
    IPAddress myIP = WiFi.softAPIP();

    Serial.println("1. Connect Phone/Tablet to Wi-Fi:");
    Serial.println("   SSID     : RiceTrack-Rover");
    Serial.println("   Password : ricetrack123");
    Serial.print  ("2. Open in browser : http://");
    Serial.println(myIP);
    Serial.println("========================================================\n");

    server.on("/", handleRoot);
    server.on("/cmd", handleCmd);
    server.on("/telemetry", handleTelemetry);
    server.begin();

    digitalWrite(PIN_LED, LOW);
}

void loop() {
    server.handleClient();
    mpuUpdate();
    processSerial();

    // Broadcast IMU telemetry over Serial @ 20Hz (every 50ms)
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
}
