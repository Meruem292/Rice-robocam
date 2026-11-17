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

WebServer server(80);

// State variables
float current_speed = 0.75f;  // 75% default speed
String current_motion = "STOP";
float left_duty = 0.0f;
float right_duty = 0.0f;

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
      <span>📷 Live Camera (<span id="txtPiIp">192.168.4.2</span>)</span>
      <span id="camStatus" style="color:#e3b341; font-weight:bold;">Connecting…</span>
    </div>
    <div class="cam-box">
      <img id="camStream" 
           src="http://192.168.4.2:8000/api/video_feed"
           onload="onCamConnected()"
           onerror="onCamError()">
    </div>
    <div class="cam-overlay-btn">
      <button class="btn-action" onclick="captureEvaluate()">📸 Capture &amp; Evaluate Crop</button>
      <a id="btnFullDash" href="http://192.168.4.2:8000" target="_blank" class="btn-secondary">📊 Full Dashboard</a>
    </div>
  </div>

  <!-- AI EVALUATION RESULT -->
  <div id="evalBox">
    <div style="font-weight:bold; font-size:14px; margin-bottom:4px;" id="evalTitle">🌱 Crop Health: Evaluating…</div>
    <div id="evalDetail" style="font-size:12px; color:#c9d1d9;">Connecting to Raspberry Pi vision model…</div>
  </div>

  <!-- MOTOR STATUS -->
  <div class="status-card">
    <div>Motion: <span id="lblMotion" class="status-val">STOP</span></div>
    <div>Speed: <span id="lblSpd" class="status-val">75%</span></div>
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
    let piIp = "192.168.4.2";

    window.onCamConnected = () => {
      document.getElementById('camStatus').textContent = "ONLINE (Pi 4B)";
      document.getElementById('camStatus').style.color = "#2ea043";
    };

    window.onCamError = () => {
      document.getElementById('camStatus').textContent = "Pi Stream Standby";
      document.getElementById('camStatus').style.color = "#e3b341";
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
        const res = await fetch('http://' + piIp + ':8000/api/capture_evaluate', { method: 'POST' });
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
        detail.textContent = "Pi at http://" + piIp + ":8000. Check Pi is connected to ESP32 AP.";
      }
    };

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
        applyCommand(act);
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
    Serial.println("RiceTrack — ESP32 AP Controller + Camera Display");
    Serial.println("========================================================");

    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);

    initPWM();
    stopMotors();

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
    server.begin();

    digitalWrite(PIN_LED, LOW);
}

void loop() {
    server.handleClient();
}
