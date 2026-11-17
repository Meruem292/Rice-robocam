/* ==============================================================================
 * RiceTrack Rover — Standalone ESP32 Wi-Fi AP Motor Diagnostic Sketch
 * File: esp32_ap_motor_test.ino
 * ==============================================================================
 * Purpose:
 *   100% standalone hardware diagnostic tool.
 *   - NO Raspberry Pi required.
 *   - NO Python server required.
 *   - NO USB Serial connection required (optional).
 * 
 * How to Use:
 *   1. Upload this sketch to your ESP32 in Arduino IDE.
 *   2. On your phone/laptop, connect to Wi-Fi:
 *        SSID     : RiceTrack-Test
 *        Password : ricetrack123
 *   3. Open your browser and go to:
 *        http://192.168.4.1
 *   4. Test each motor individually and together at 40%, 60%, 80%, 100% power!
 *      Supports both "Click to Run (Continuous)" and "Hold to Run" modes.
 *
 * Pinout (Same as Rover Project):
 *   Left BTS7960:
 *     RPWM -> GPIO 32
 *     LPWM -> GPIO 33
 *     R_EN -> GPIO 25 (or jumped to VCC)
 *     L_EN -> GPIO 26 (or jumped to VCC)
 *   Right BTS7960:
 *     RPWM -> GPIO 18
 *     LPWM -> GPIO 19
 *     R_EN -> GPIO 27 (or jumped to VCC)
 *     L_EN -> GPIO 14 (or jumped to VCC)
 *   Status LED: GPIO 2
 * ============================================================================== */

#include <WiFi.h>
#include <WebServer.h>

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

// PWM Parameters
const int PWM_FREQ       = 1000;  // 1 kHz
const int PWM_RESOLUTION = 10;    // 10-bit (0 - 1023)
const int PWM_MAX        = 1023;

// Web Server on port 80
WebServer server(80);

// State variables
float current_speed = 0.70f;  // 70% default speed
String current_motion = "STOP";
float left_duty = 0.0f;
float right_duty = 0.0f;

// ------------------------------------------------------------------------------
// PWM WRAPPER (Compatible with ESP32 Core 2.x and 3.x)
// ------------------------------------------------------------------------------
void initPWM() {
    pinMode(PIN_LEFT_R_EN, OUTPUT);
    pinMode(PIN_LEFT_L_EN, OUTPUT);
    pinMode(PIN_RIGHT_R_EN, OUTPUT);
    pinMode(PIN_RIGHT_L_EN, OUTPUT);

    // Keep enable pins HIGH
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
}

void applyCommand(const String& cmd) {
    current_motion = cmd;
    float spd = current_speed;

    if (cmd == "forward") {
        setMotors(spd, spd);
    } else if (cmd == "backward") {
        setMotors(-spd, -spd);
    } else if (cmd == "left") {
        setMotors(spd, -spd);
    } else if (cmd == "right") {
        setMotors(-spd, spd);
    } else if (cmd == "left_fwd") {
        setMotors(spd, 0.0f);
    } else if (cmd == "left_rev") {
        setMotors(-spd, 0.0f);
    } else if (cmd == "right_fwd") {
        setMotors(0.0f, spd);
    } else if (cmd == "right_rev") {
        setMotors(0.0f, -spd);
    } else {
        stopMotors();
    }

    Serial.print("Action: ");
    Serial.print(cmd);
    Serial.print(" | L: ");
    Serial.print((int)(left_duty * 100));
    Serial.print("% | R: ");
    Serial.print((int)(right_duty * 100));
    Serial.println("%");
}

// ------------------------------------------------------------------------------
// HTML WEB PAGE
// ------------------------------------------------------------------------------
const char PAGE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
  <title>ESP32 Motor Diagnostic</title>
  <style>
    * { box-sizing: border-box; -webkit-tap-highlight-color: transparent; }
    body {
      margin: 0; padding: 16px; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
      background: #0d1117; color: #c9d1d9; text-align: center; user-select: none;
    }
    h2 { margin: 4px 0 8px; color: #58a6ff; font-size: 20px; }
    .status-card {
      background: #161b22; border: 1px solid #30363d; border-radius: 8px;
      padding: 10px; margin-bottom: 14px; font-size: 14px;
    }
    .status-val { font-weight: bold; color: #2ea043; font-size: 16px; }
    .btn-group { display: flex; justify-content: center; gap: 8px; margin-bottom: 12px; }
    .spd-btn {
      background: #21262d; color: #c9d1d9; border: 1px solid #30363d; border-radius: 6px;
      padding: 8px 14px; font-weight: bold; font-size: 14px; cursor: pointer;
    }
    .spd-btn.active { background: #1f6feb; border-color: #58a6ff; color: #fff; }
    
    /* D-PAD GRID */
    .dpad-wrap { display: flex; justify-content: center; margin: 10px 0 16px; }
    .dpad {
      display: grid; grid-template-columns: 80px 80px 80px; grid-template-rows: 75px 75px 75px; gap: 8px;
    }
    .btn {
      background: #21262d; color: #f0f6fc; border: 1px solid #30363d; border-radius: 10px;
      font-size: 22px; font-weight: bold; display: flex; align-items: center; justify-content: center;
      cursor: pointer; transition: all 0.1s;
    }
    .btn:active { background: #388bfd; transform: scale(0.96); }
    .btn-stop { background: #da3633; border-color: #f85149; font-size: 16px; }
    .btn-stop:active { background: #b62324; }

    /* INDIVIDUAL MOTOR TESTS */
    .section-title { font-size: 12px; color: #8b949e; text-transform: uppercase; margin: 12px 0 6px; }
    .motor-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; margin-bottom: 16px; }
    .btn-sub {
      background: #161b22; color: #c9d1d9; border: 1px solid #30363d; border-radius: 6px;
      padding: 10px; font-size: 13px; font-weight: 600; cursor: pointer;
    }
    .btn-sub:active { background: #238636; }
    
    /* MODE SWITCH */
    .mode-wrap { margin-top: 10px; font-size: 13px; color: #8b949e; }
    .switch-label { cursor: pointer; }
  </style>
</head>
<body>
  <h2>RiceTrack Motor Test (ESP32 AP)</h2>

  <div class="status-card">
    Motion: <span id="lblMotion" class="status-val">STOP</span> &nbsp;|&nbsp; 
    Power: <span id="lblSpd" class="status-val">70%</span>
  </div>

  <div class="section-title">Speed / Power Level</div>
  <div class="btn-group">
    <button class="spd-btn" onclick="setSpeed(0.40)">40%</button>
    <button class="spd-btn" onclick="setSpeed(0.60)">60%</button>
    <button class="spd-btn active" onclick="setSpeed(0.70)">70%</button>
    <button class="spd-btn" onclick="setSpeed(0.85)">85%</button>
    <button class="spd-btn" onclick="setSpeed(1.00)">100%</button>
  </div>

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

  <div class="section-title">Individual Motor Isolation</div>
  <div class="motor-grid">
    <button class="btn-sub" onclick="execCmd('left_fwd')">Left Only FWD</button>
    <button class="btn-sub" onclick="execCmd('right_fwd')">Right Only FWD</button>
    <button class="btn-sub" onclick="execCmd('left_rev')">Left Only REV</button>
    <button class="btn-sub" onclick="execCmd('right_rev')">Right Only REV</button>
  </div>

  <div class="mode-wrap">
    <label class="switch-label">
      <input type="checkbox" id="chkContinuous" onchange="toggleContinuous(this.checked)">
      <b>Continuous Mode</b> (Click to run continuously, Click STOP to halt)
    </label>
  </div>

  <script>
    let isContinuous = false;
    let curSpeed = 0.70;

    function toggleContinuous(val) {
      isContinuous = val;
    }

    function setSpeed(spd) {
      curSpeed = spd;
      document.querySelectorAll('.spd-btn').forEach(b => {
        b.classList.toggle('active', b.textContent === Math.round(spd * 100) + '%');
      });
      document.getElementById('lblSpd').textContent = Math.round(spd * 100) + '%';
      fetch('/cmd?spd=' + spd).catch(()=>{});
    }

    function execCmd(act) {
      document.getElementById('lblMotion').textContent = act.toUpperCase();
      fetch('/cmd?act=' + act + '&spd=' + curSpeed).catch(()=>{});
    }

    function pressBtn(act) {
      execCmd(act);
    }

    function releaseBtn() {
      if (!isContinuous) {
        execCmd('stop');
      }
    }
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
        applyCommand(act);
    }

    server.send(200, "application/json", "{\"status\":\"ok\"}");
}

// ------------------------------------------------------------------------------
// SETUP & LOOP
// ------------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(300);

    Serial.println("\n========================================================");
    Serial.println("RiceTrack — Standalone ESP32 Motor Diagnostic AP");
    Serial.println("========================================================");

    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);

    initPWM();
    stopMotors();

    // Start SoftAP
    WiFi.mode(WIFI_AP);
    WiFi.softAP("RiceTrack-Test", "ricetrack123");
    IPAddress myIP = WiFi.softAPIP();

    Serial.println("1. Connect to Wi-Fi :");
    Serial.println("   SSID     : RiceTrack-Test");
    Serial.println("   Password : ricetrack123");
    Serial.print  ("2. Open in browser  : http://");
    Serial.println(myIP);
    Serial.println("3. Or send single characters via Serial Monitor:");
    Serial.println("   W / S / A / D / X (Stop) / 1 (Left) / 2 (Right) / + / -");
    Serial.println("========================================================\n");

    server.on("/", handleRoot);
    server.on("/cmd", handleCmd);
    server.begin();

    digitalWrite(PIN_LED, LOW);
}

void loop() {
    server.handleClient();

    // Also support Serial Monitor test characters
    if (Serial.available()) {
        char c = (char)toupper((unsigned char)Serial.read());
        switch (c) {
            case 'W': applyCommand("forward"); break;
            case 'S': applyCommand("backward"); break;
            case 'A': applyCommand("left"); break;
            case 'D': applyCommand("right"); break;
            case '1': applyCommand("left_fwd"); break;
            case '2': applyCommand("right_fwd"); break;
            case '3': applyCommand("left_rev"); break;
            case '4': applyCommand("right_rev"); break;
            case 'X': stopMotors(); break;
            case '+': 
                current_speed = min(1.0f, current_speed + 0.10f);
                Serial.print("Speed: "); Serial.println((int)(current_speed * 100));
                break;
            case '-':
                current_speed = max(0.20f, current_speed - 0.10f);
                Serial.print("Speed: "); Serial.println((int)(current_speed * 100));
                break;
            default: break;
        }
    }
}
