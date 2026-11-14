# RiceTrackCV — Raspberry Pi 4B + ESP32 Agricultural Rover

Autonomous rice field health monitoring and rover control system powered by a **Raspberry Pi 4B** (High-Level Brain, Computer Vision & Web Server) and an **ESP32-WROOM-32** (Real-Time Motor PWM & Sensor Controller), equipped with a **USB Camera**, **NEO-6M GPS Module**, **MPU-6050 6-Axis IMU**, **Dual BTS7960 Motor Drivers**, **Dual MY1016Z2 24V DC Motors**, and a **24V 12Ah Ebike Battery** with a **24V → 5V Buck Converter**.

---

## 1. System Architecture (Raspberry Pi 4B + ESP32)

```mermaid
flowchart TB
    BAT["24V 12Ah E-Bike Battery (B+, B-)"]
    FUSE["40A DC Inline Blade Fuse"]
    ESTOP["30A DC Master E-Stop Switch"]
    PROT["Protection Cluster (2200µF 35V Cap + SMBJ28A TVS)"]
    BUCK["24V -> 5V 5A Buck Converter"]
    
    RPI["Raspberry Pi 4B (Brain)<br/>• FastAPI Web Server<br/>• OpenCV ExG & IRRI LCC Analysis<br/>• SQLite Database (ricetrack.db)<br/>• In-Field Wi-Fi Hotspot"]
    CAM["USB Camera (OpenCV 1280x720)"]
    
    ESP["ESP32-WROOM-32 DevKit V1<br/>• 1 kHz Hardware PWM (LEDC)<br/>• 20 Hz IMU Complementary Filter<br/>• GPS NMEA UART Parser<br/>• Safety Heartbeat Watchdog"]
    
    GPS["NEO-6M GPS Module (UART2)"]
    IMU["MPU-6050 IMU Gyroscope (I2C)"]
    BTS_L["Left BTS7960 Motor Driver"]
    BTS_R["Right BTS7960 Motor Driver"]
    MOT_L["Left MY1016Z2 24V DC Motor"]
    MOT_R["Right MY1016Z2 24V DC Motor"]

    BAT -->|"B+ (12 AWG)"| FUSE
    FUSE --> ESTOP
    ESTOP --> PROT
    PROT -->|"24V High Current (12 AWG)"| BTS_L
    PROT -->|"24V High Current (12 AWG)"| BTS_R
    PROT -->|"24V Input"| BUCK

    BUCK -->|"Regulated 5V 5A (USB-C / GPIO)"| RPI
    RPI <-->|"USB Data Cable (/dev/ttyUSB0)"| ESP
    CAM <-->|"USB Port"| RPI

    ESP -->|"GPIO 16/17 (UART2)"| GPS
    ESP <-->|"GPIO 21/22 (I2C)"| IMU

    ESP -->|"3.3V Logic & GND"| GPS
    ESP -->|"3.3V Logic & GND"| IMU
    ESP -->|"3.3V Logic & GND"| BTS_L
    ESP -->|"3.3V Logic & GND"| BTS_R

    ESP -->|"GPIO 25 (R_En), 26 (L_En), 32 (RPWM), 33 (LPWM)"| BTS_L
    ESP -->|"GPIO 27 (R_En), 14 (L_En), 18 (RPWM), 19 (LPWM)"| BTS_R

    BTS_L -->|"M+ / M- (14 AWG)"| MOT_L
    BTS_R -->|"M+ / M- (14 AWG)"| MOT_R
```

---

## 2. Complete Pinout & Wiring Specification

### A. Power & Ground Distribution (Star Grounding)
* **Main Traction Rail:** Battery (+) $\to$ 40A Fuse $\to$ 30A E-Stop Switch $\to$ Parallel branch to BTS7960 `B+` (Left & Right) and Buck Converter `In+`.
* **Buck Converter (24V $\to$ 5V 5A):** Supplies 5.1V clean DC power to Raspberry Pi 4B (via USB-C or GPIO Pins 2/4 & 6).
* **ESP32 Power:** Powered directly from Raspberry Pi 4B via USB Cable (or from 5V buck converter into `VIN`).
* **Common Ground (Star Ground):** The Battery negative (`B-`), Buck Converter `In-` & `Out-`, and BTS7960 `B-` must all terminate at a heavy-gauge central star-ground terminal block to prevent motor return currents from flowing through the Raspberry Pi logic.

### B. Module Pin-to-Pin Connections

| Module / Component | Module Pin | Connected To | Logic / Function | Wire Gauge |
| :--- | :--- | :--- | :--- | :--- |
| **Raspberry Pi 4B** | USB Port 1 | **ESP32 USB Port** | Serial Bridge @ 115200 baud (`/dev/ttyUSB0`) | Standard USB |
| **Raspberry Pi 4B** | USB Port 2 | **USB Camera** | OpenCV VideoCapture (`/dev/video0`) | Standard USB |
| **Raspberry Pi 4B** | USB-C Power | **Buck Converter 5V Out** | 5.1V 3A–5A Regulated Power | 18 AWG |
| **NEO-6M GPS Module** | `TX` | ESP32 `GPIO 16` (RX2) | NMEA 9600 baud GPS Telemetry | 24–26 AWG |
| **NEO-6M GPS Module** | `RX` | ESP32 `GPIO 17` (TX2) | GPS Configuration | 24–26 AWG |
| **NEO-6M GPS Module** | `VCC` / `GND` | ESP32 `3.3V` / `GND` | 3.3V Sensor Power Rail | 24–26 AWG |
| **MPU-6050 6-Axis IMU** | `SDA` | ESP32 `GPIO 21` | I2C Data (Addr `0x68`) | 24–26 AWG |
| **MPU-6050 6-Axis IMU** | `SCL` | ESP32 `GPIO 22` | I2C Clock (Addr `0x68`) | 24–26 AWG |
| **MPU-6050 6-Axis IMU** | `VCC` / `GND` | ESP32 `3.3V` / `GND` | 3.3V Sensor Power Rail | 24–26 AWG |
| **Left BTS7960 Driver** | `R_EN` / `L_EN` | ESP32 `GPIO 25` / `GPIO 26` | Digital Enables (Active HIGH) | 22–24 AWG |
| **Left BTS7960 Driver** | `RPWM` / `LPWM` | ESP32 `GPIO 32` / `GPIO 33` | LEDC Hardware PWM Ch 0/1 @ 1 kHz | 22–24 AWG |
| **Left BTS7960 Driver** | `VCC` / `GND` | ESP32 `3.3V` / `GND` | Logic Power & Ground | 22–24 AWG |
| **Left BTS7960 Driver** | `B+` / `B-` | 24V Battery Bus (post-Estop) | High Current Traction Supply | 12–14 AWG |
| **Left BTS7960 Driver** | `M+` / `M-` | Left MY1016Z2 DC Motor | Left Drive Output | 14 AWG |
| **Right BTS7960 Driver** | `R_EN` / `L_EN` | ESP32 `GPIO 27` / `GPIO 14` | Digital Enables (Active HIGH) | 22–24 AWG |
| **Right BTS7960 Driver** | `RPWM` / `LPWM` | ESP32 `GPIO 18` / `GPIO 19` | LEDC Hardware PWM Ch 2/3 @ 1 kHz | 22–24 AWG |
| **Right BTS7960 Driver** | `VCC` / `GND` | ESP32 `3.3V` / `GND` | Logic Power & Ground | 22–24 AWG |
| **Right BTS7960 Driver** | `B+` / `B-` | 24V Battery Bus (post-Estop) | High Current Traction Supply | 12–14 AWG |
| **Right BTS7960 Driver** | `M+` / `M-` | Right MY1016Z2 DC Motor | Right Drive Output | 14 AWG |

---

## 3. Flash ESP32 Firmware

1. Open [`esp32_firmware/ricetrack_esp32/ricetrack_esp32.ino`](file:///esp32_firmware/ricetrack_esp32/ricetrack_esp32.ino) in the Arduino IDE.
2. Select Board: **DOIT ESP32 DEVKIT V1**.
3. Select Port: **COMx** (Windows) or `/dev/ttyUSBx` (Linux).
4. Click **Upload**. (No third-party libraries needed; uses native `Wire.h` and `HardwareSerial`).

---

## 4. Raspberry Pi 4B Setup & Quickstart

### Step 1: Clone & Run Automated Setup
On your Raspberry Pi running Raspberry Pi OS (64-bit recommended):
```bash
git clone <your-repo-url>
cd robo_cam-main/robo_cam-main

# Run automated dependency installer and permission configuration
chmod +x setup_rpi.sh
./setup_rpi.sh
```

### Step 2 (Optional): Enable In-Field Wi-Fi Hotspot
If operating in remote rice fields without an existing router, configure the Pi to broadcast its own Wi-Fi AP:
```bash
chmod +x create_wifi_ap.sh
./create_wifi_ap.sh
```
* **Hotspot Name (SSID):** `RiceTrack-Rover`
* **Password:** `ricetrack123`
* **Rover IP:** `192.168.4.1`

### Step 3: Run the Web Application
```bash
chmod +x run_web.sh
./run_web.sh
```
Open **`http://<pi-ip>:8000`** (or `http://192.168.4.1:8000` if connected to hotspot) in any web browser on your phone, tablet, or laptop.

### Step 4: Auto-Start on Rover Boot (systemd service)
To have RiceTrack start automatically whenever the rover is switched on:
```bash
sudo cp ricetrack.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now ricetrack.service
```

---

## 5. Crop Health Checking Pipeline

* **Algorithm:** Combines Excess Green Index ($\text{ExG} = 2G - R - B$) with calibrated HSV hue and CIELAB color space distance ($\Delta E^*$).
* **IRRI Standard:** Evaluates leaves against the official International Rice Research Institute (IRRI) 4-panel Leaf Color Chart (Panels 2, 3, 4, 5).
* **Botanical Verification:** Prevents false positives from green plastics, weeds, and synthetic objects via Canny edge density and Laplacian texture variance.
* **Storage & Export:** All snapshots, coordinates, timestamps, and health scores are saved to `ricetrack.db` and can be exported as PDF field reports directly from the web interface.

---

## 6. Semi-Autonomous Instructed Roving

The system features an **Instructed Roving Engine** that coordinates sequential waypoint movements, exact turns, and automated crop health sampling:

### A. Turn Calculation via MPU-6050
* The ESP32 continuously calibrates and integrates the $Z$-axis gyroscope ($g_z$) with a deadband filter into `yaw_deg` ($20\text{ Hz}$).
* When turning Left or Right, the skid-steer drivetrain pivots until $|\Delta \text{yaw}| \ge 90^\circ$ (with a safety timeout).

### B. Standard 4-Step Mission Routine
1. **Move 1:** Drive 5 meters forward (timed at calibrated seconds/meter), turn Right 90° using MPU-6050, steady camera for 1 second, capture & evaluate crop health.
2. **Move 2:** Drive 10 meters forward, turn Left 90° using MPU-6050, steady camera, capture & evaluate crop health.
3. **Move 3:** Drive 4 meters forward, turn Left 90° using MPU-6050, steady camera, capture & evaluate crop health.
4. **Move 4 (Final Return):** Inverted Step Retrace — Rover turns 180°, then retraces completed legs in reverse order back to the initial starting point $(0, 0)$ and restores starting orientation.

### C. Selectable Interface & Speed Calibration
* **Select-Type Mission Builder:** Choose from pre-configured presets or customize distance (1m–30m), turn direction, and sampling per step using dropdown `<select>` inputs in the Web UI.
* **Speed Factor:** Default is **3.0 seconds/meter** (configurable in Standards tab to match dry or flooded field soil).
* **Emergency Abort:** One-click abort button immediately kills motor PWM and halts the mission.
