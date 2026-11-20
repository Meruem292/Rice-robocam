/* ==============================================================================
 * RiceTrack — Standalone ESP32 + MPU-6050 + NEO-6M GPS Diagnostic Test Sketch
 * ==============================================================================
 * Zero external libraries required! (Uses built-in Wire.h + Serial2)
 *
 * WIRING:
 *   MPU-6050 (I2C):
 *     VCC -> 3.3V (or 5V on GY-521)
 *     GND -> GND
 *     SDA -> GPIO 21
 *     SCL -> GPIO 22
 *
 *   NEO-6M GPS (UART2 @ 9600 baud):
 *     VCC -> 3.3V / 5V
 *     GND -> GND
 *     TX  -> GPIO 16 (ESP32 RX2)
 *     RX  -> GPIO 17 (ESP32 TX2)
 *
 * Open Serial Monitor at 115200 baud to see live MPU & GPS diagnostics!
 * ============================================================================== */

#include <Wire.h>
#include <math.h>

// Pins
const int PIN_I2C_SDA = 21;
const int PIN_I2C_SCL = 22;
const int PIN_GPS_RX  = 16;  // Connects to GPS TX
const int PIN_GPS_TX  = 17;  // Connects to GPS RX
const int PIN_LED     = 2;

// MPU-6050 state
uint8_t mpu_addr = 0x68;
bool mpu_ok = false;
float gx_bias = 0.0f, gy_bias = 0.0f, gz_bias = 0.0f;
float roll_zero = 0.0f, pitch_zero = 0.0f;
float roll_deg = 0.0f, pitch_deg = 0.0f, yaw_deg = 0.0f;
uint32_t last_imu_us = 0;

// GPS state
String nmea_line = "";
uint32_t gps_bytes_rx = 0;
uint32_t gps_sentences = 0;
bool gps_fix = false;
int gps_sats = 0;
double gps_lat = 0.0;
double gps_lon = 0.0;
String last_gga = "";

uint32_t last_print_ms = 0;

// ------------------------------------------------------------------------------
// NMEA PARSER HELPER (Zero external libraries)
// ------------------------------------------------------------------------------
double nmeaToDecimal(const String& raw, const String& hemi) {
  if (raw.length() < 4 || raw.indexOf('.') == -1) return 0.0;
  int dot = raw.indexOf('.');
  int degDigits = dot - 2;
  if (degDigits <= 0) return 0.0;
  double deg = raw.substring(0, degDigits).toDouble();
  double mins = raw.substring(degDigits).toDouble();
  double dec = deg + (mins / 60.0);
  if (hemi == "S" || hemi == "W") dec = -dec;
  return dec;
}

String getField(const String& line, int idx) {
  int start = 0;
  int count = 0;
  for (int i = 0; i <= (int)line.length(); i++) {
    if (i == (int)line.length() || line[i] == ',' || line[i] == '*') {
      if (count == idx) return line.substring(start, i);
      count++;
      start = i + 1;
    }
  }
  return "";
}

void parseNMEA(const String& line) {
  gps_sentences++;
  if (line.startsWith("$GPGGA") || line.startsWith("$GNGGA")) {
    last_gga = line;
    String latStr = getField(line, 2);
    String latH   = getField(line, 3);
    String lonStr = getField(line, 4);
    String lonH   = getField(line, 5);
    int fixQ      = getField(line, 6).toInt();
    gps_sats      = getField(line, 7).toInt();
    gps_fix       = (fixQ > 0);
    if (gps_fix && latStr.length() > 0 && lonStr.length() > 0) {
      gps_lat = nmeaToDecimal(latStr, latH);
      gps_lon = nmeaToDecimal(lonStr, lonH);
    }
  }
}

// ------------------------------------------------------------------------------
// MPU-6050 I2C SCAN + INIT + AUTO-CALIBRATION
// ------------------------------------------------------------------------------
bool initMPU() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
  delay(30);

  Serial.println("Scanning I2C bus (SDA=21, SCL=22)...");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("  -> Found I2C device at 0x");
      Serial.println(addr, HEX);
      if (addr == 0x68 || addr == 0x69) mpu_addr = addr;
      found++;
    }
  }
  if (found == 0) {
    Serial.println("  ❌ No I2C devices found! Check MPU-6050 VCC, GND, SDA(21), SCL(22).");
    return false;
  }

  // Wake up MPU-6050
  Wire.beginTransmission(mpu_addr);
  Wire.write(0x6B);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) return false;

  // 42 Hz DLPF filter
  Wire.beginTransmission(mpu_addr); Wire.write(0x1A); Wire.write(0x03); Wire.endTransmission();
  // Accel +/- 4g
  Wire.beginTransmission(mpu_addr); Wire.write(0x1C); Wire.write(0x08); Wire.endTransmission();
  // Gyro +/- 500 deg/s
  Wire.beginTransmission(mpu_addr); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission();

  delay(30);
  Serial.println("Calibrating MPU-6050 base level & gyro bias (keep rover still)...");

  long gx_s = 0, gy_s = 0, gz_s = 0;
  double r_s = 0.0, p_s = 0.0;
  int n = 0;
  for (int i = 0; i < 200; i++) {
    Wire.beginTransmission(mpu_addr);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom((uint16_t)mpu_addr, (uint8_t)14, (bool)true) == 14) {
      int16_t ax = (Wire.read() << 8) | Wire.read();
      int16_t ay = (Wire.read() << 8) | Wire.read();
      int16_t az = (Wire.read() << 8) | Wire.read();
      Wire.read(); Wire.read();
      int16_t gx = (Wire.read() << 8) | Wire.read();
      int16_t gy = (Wire.read() << 8) | Wire.read();
      int16_t gz = (Wire.read() << 8) | Wire.read();
      float ax_g = ax / 8192.0f, ay_g = ay / 8192.0f, az_g = az / 8192.0f;
      r_s += atan2(ay_g, az_g) * 180.0 / M_PI;
      p_s += atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180.0 / M_PI;
      gx_s += gx; gy_s += gy; gz_s += gz;
      n++;
    }
    delay(2);
  }
  if (n > 0) {
    gx_bias = (float)gx_s / n;
    gy_bias = (float)gy_s / n;
    gz_bias = (float)gz_s / n;
    roll_zero = (float)(r_s / n);
    pitch_zero = (float)(p_s / n);
  }
  last_imu_us = micros();
  Serial.println("  ✅ MPU-6050 Ready & Auto-Calibrated to (0.0°, 0.0°, 0.0°)!");
  return true;
}

void updateMPU() {
  if (!mpu_ok) return;
  Wire.beginTransmission(mpu_addr);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return;
  if (Wire.requestFrom((uint16_t)mpu_addr, (uint8_t)14, (bool)true) < 14) return;

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();
  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();

  float ax_g = ax / 8192.0f, ay_g = ay / 8192.0f, az_g = az / 8192.0f;
  float gx_dps = (gx - gx_bias) / 65.5f;
  float gy_dps = (gy - gy_bias) / 65.5f;
  float gz_dps = (gz - gz_bias) / 65.5f;
  if (fabs(gz_dps) < 0.25f) gz_dps = 0.0f;

  float accel_r = (atan2(ay_g, az_g) * 180.0f / M_PI) - roll_zero;
  float accel_p = (atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180.0f / M_PI) - pitch_zero;
  while (accel_r > 180.0f) accel_r -= 360.0f;
  while (accel_r < -180.0f) accel_r += 360.0f;
  while (accel_p > 180.0f) accel_p -= 360.0f;
  while (accel_p < -180.0f) accel_p += 360.0f;

  uint32_t now_us = micros();
  float dt = (now_us - last_imu_us) / 1000000.0f;
  if (dt <= 0.0f || dt > 0.5f) dt = 0.01f;
  last_imu_us = now_us;

  roll_deg  = 0.96f * (roll_deg  + gx_dps * dt) + 0.04f * accel_r;
  pitch_deg = 0.96f * (pitch_deg + gy_dps * dt) + 0.04f * accel_p;
  yaw_deg  += gz_dps * dt;
}

// ------------------------------------------------------------------------------
// SETUP & LOOP
// ------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(400);
  pinMode(PIN_LED, OUTPUT);

  Serial.println("\n========================================================");
  Serial.println("   RiceTrack ESP32 + MPU-6050 + NEO-6M GPS Test");
  Serial.println("========================================================");

  // 1. Init MPU-6050
  mpu_ok = initMPU();

  // 2. Init NEO-6M GPS on UART2 (RX=16, TX=17, 9600 baud)
  Serial2.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  Serial.println("GPS UART2 started at 9600 baud (RX=GPIO16 <- GPS TX, TX=GPIO17 -> GPS RX)");
  Serial.println("Type 'z' in Serial Monitor to re-zero Compass Yaw.");
  Serial.println("========================================================\n");
}

void loop() {
  // Read MPU-6050 continuously
  updateMPU();

  // Read NEO-6M GPS UART2 bytes
  while (Serial2.available()) {
    char c = (char)Serial2.read();
    gps_bytes_rx++;
    if (c == '\n') {
      nmea_line.trim();
      if (nmea_line.startsWith("$")) parseNMEA(nmea_line);
      nmea_line = "";
    } else if (c != '\r') {
      if (nmea_line.length() < 120) nmea_line += c;
    }
  }

  // Serial command: type 'z' to zero yaw
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'z' || cmd == 'Z') {
      yaw_deg = 0.0f;
      Serial.println(">>> Compass Yaw zeroed to 0.0°");
    }
  }

  // Print clean diagnostic summary every 500ms
  uint32_t now = millis();
  if (now - last_print_ms >= 500) {
    last_print_ms = now;
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));

    Serial.print("[MPU] ");
    if (mpu_ok) {
      Serial.print("Yaw(Rot): "); Serial.print(yaw_deg, 1); Serial.print("°");
      Serial.print(" | Roll: ");  Serial.print(roll_deg, 1); Serial.print("°");
      Serial.print(" | Pitch: "); Serial.print(pitch_deg, 1); Serial.print("°");
    } else {
      Serial.print("NOT DETECTED");
    }

    Serial.print("   ||   [GPS] ");
    if (gps_bytes_rx == 0) {
      Serial.print("NO DATA (Check TX->GPIO16 / power)");
    } else if (!gps_fix) {
      Serial.print("CONNECTED (Searching sky... Sats: ");
      Serial.print(gps_sats);
      Serial.print(", NMEA pkts: ");
      Serial.print(gps_sentences);
      Serial.print(")");
    } else {
      Serial.print("FIX OK! Lat: "); Serial.print(gps_lat, 6);
      Serial.print(", Lon: ");       Serial.print(gps_lon, 6);
      Serial.print(" (Sats: ");      Serial.print(gps_sats);
      Serial.print(")");
    }
    Serial.println();
  }
}
