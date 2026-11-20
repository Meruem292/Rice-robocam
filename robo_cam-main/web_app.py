"""
web_app.py
----------
Option A Web Application for RiceTrack (ESP32-WROOM-32 + USB Camera).
Exposes the complete 5-tab RiceTrack experience (Rover Control, Field Map,
Records, Validation, Standards) over HTTP / MJPEG / REST using FastAPI.
"""

from contextlib import asynccontextmanager
import base64
import csv
from datetime import datetime
import io
import os
from pathlib import Path
import threading
import time
from typing import Any

import cv2
from fastapi import FastAPI, File, HTTPException, Query, UploadFile
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse, JSONResponse, Response, StreamingResponse
from pydantic import BaseModel
import numpy as np
import uvicorn

from common import STANDARDS
import database
import pdf_manuals
from esp32_hardware import (
    PINS,
    ESP32BTS7960Motors,
    ESP32Camera,
    ESP32MPU6050,
    ESP32NEO6MGPS,
)
from vision import NoVegetationDetected, detect_scene, draw_scene_alert, encode_thumbnail, evaluate, overlay_mask

BASE_DIR = Path(__file__).resolve().parent
TEMPLATE_PATH = BASE_DIR / "templates" / "index.html"


INSTRUCTED_PRESETS = [
    {
        "id": "preset_4step_inspection",
        "name": "Field Mission: 5m R → 10m L → 4m L → Return",
        "description": "Move 1: 5m then turn Right & capture; Move 2: 10m then turn Left & capture; Move 3: 4m then turn Left & capture; Move 4: Inverted retrace back to initial position.",
        "steps": [
            {"step": 1, "type": "leg", "distance_m": 5.0, "turn": "right", "capture": True},
            {"step": 2, "type": "leg", "distance_m": 10.0, "turn": "left", "capture": True},
            {"step": 3, "type": "leg", "distance_m": 4.0, "turn": "left", "capture": True},
            {"step": 4, "type": "return", "distance_m": 0.0, "turn": "none", "capture": False},
        ],
    },
    {
        "id": "preset_u_turn_sweep",
        "name": "U-Turn Canopy Inspection (8m R → 4m R → 8m → Return)",
        "description": "Sweep across parallel paddy rows with right 90° turns and health sampling.",
        "steps": [
            {"step": 1, "type": "leg", "distance_m": 8.0, "turn": "right", "capture": True},
            {"step": 2, "type": "leg", "distance_m": 4.0, "turn": "right", "capture": True},
            {"step": 3, "type": "leg", "distance_m": 8.0, "turn": "none", "capture": True},
            {"step": 4, "type": "return", "distance_m": 0.0, "turn": "none", "capture": False},
        ],
    },
    {
        "id": "preset_straight_sampling",
        "name": "Linear Row Sampling (5m → 5m → 5m → Return)",
        "description": "Straight-line row inspection with health samples every 5 meters, then returns.",
        "steps": [
            {"step": 1, "type": "leg", "distance_m": 5.0, "turn": "none", "capture": True},
            {"step": 2, "type": "leg", "distance_m": 5.0, "turn": "none", "capture": True},
            {"step": 3, "type": "leg", "distance_m": 5.0, "turn": "none", "capture": True},
            {"step": 4, "type": "return", "distance_m": 0.0, "turn": "none", "capture": False},
        ],
    },
]


def _frame_to_data_url(img_bgr: np.ndarray, quality: int = 88) -> str:
    ok, buf = cv2.imencode(".jpg", img_bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        return ""
    b64 = base64.b64encode(buf.tobytes()).decode("ascii")
    return f"data:image/jpeg;base64,{b64}"


def _format_date_short(iso_str: str) -> str:
    try:
        return datetime.fromisoformat(iso_str).strftime("%b %d, %H:%M")
    except (TypeError, ValueError):
        return iso_str or "--"


def _format_date_full(iso_str: str) -> str:
    try:
        return datetime.fromisoformat(iso_str).strftime("%B %d, %Y · %H:%M")
    except (TypeError, ValueError):
        return iso_str or "--"


class WebHardwareHub:
    """
    Thread-safe hardware & autonomous Train-Drive coordinator for the Web App.
    Operates without requiring a Qt event loop so it runs headlessly on ESP32 or PC.
    """

    def __init__(self):
        self.imu = ESP32MPU6050()
        self.gps = ESP32NEO6MGPS()
        self.motors = ESP32BTS7960Motors()
        self.camera = ESP32Camera(camera_index=0)

        self.roll: float = 0.0
        self.pitch: float = 0.0
        self.yaw: float = 0.0
        self.imu_ok: bool = False
        self.speed: float = 0.0
        self.motor_cmd: str = "stop"

        self.gps_state: str = "none"
        self.lat: float = 15.67170
        self.lon: float = 120.89280
        self.sats: int = 0
        self._sim_lat: float = 15.67170
        self._sim_lon: float = 120.89280

        # Connect hardware signals to WebHardwareHub state
        self.imu.imu_reading.connect(self._on_imu_reading)
        self.imu.imu_state.connect(self._on_imu_state)
        self.gps.gps_changed.connect(self._on_gps_changed)

        self.auto_active: bool = False
        self.auto_interval_s: int = int(STANDARDS.get("auto_drive_s", 5))
        self.auto_countdown_s: int = self.auto_interval_s
        self.auto_cycle: int = 0
        self.auto_mode: str = "train_pulse"
        self.auto_direction: str = "forward"
        self.auto_pausing: bool = False
        self.auto_reminder: str = (
            f"Train Mode Standby · Runs every {self.auto_interval_s}s when Auto Drive is ON"
        )
        self.latest_path_text: str = "Standby (auto_drive_path.json)"

        # Semi-Autonomous Instructed Roving state
        self.instructed_active: bool = False
        self.instructed_thread: threading.Thread | None = None
        self.instructed_state: dict = self._initial_instructed_state()

        self.live_hue: float | None = None
        self.live_panel: str = "--"
        self.live_in_band: bool = True
        self.live_summary: str = "--"
        self._last_live_eval_t: float = 0.0
        self._last_live_result: Any = None
        self._last_scene_t: float = 0.0
        self._scene_alert: Any = None

        self._running = False
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()

    def _initial_instructed_state(self) -> dict:
        return {
            "active": False,
            "phase": "idle",  # "idle", "driving", "turning", "capturing", "retracing", "completed", "aborted"
            "phase_detail": "Standby — Ready to start mission",
            "current_step": 0,
            "total_steps": 0,
            "current_action": "None",
            "distance_m": 0.0,
            "elapsed_s": 0.0,
            "target_s": 0.0,
            "current_yaw": 0.0,
            "target_yaw": 0.0,
            "start_gps": None,
            "last_capture_id": None,
        }

    def _on_imu_reading(self, roll: float, pitch: float, *args):
        self.roll = float(roll)
        self.pitch = float(pitch)
        self.yaw = round(self.imu.yaw, 1)
        tilt_limit = float(STANDARDS.get("tilt", 15))
        if (self.auto_active or self.instructed_active) and max(abs(self.roll), abs(self.pitch)) > tilt_limit:
            self._stop_auto_unlocked(
                f"Safety cutoff: Tilt {max(abs(self.roll), abs(self.pitch)):.0f}° > {tilt_limit:.0f}° — Stopped"
            )
            self.stop_instructed_roving(f"Safety cutoff: Tilt {max(abs(self.roll), abs(self.pitch)):.0f}° > {tilt_limit:.0f}°")

    def _on_imu_state(self, ok: bool):
        self.imu_ok = bool(ok)
        if not ok:
            self.roll = 0.0
            self.pitch = 0.0

    def _on_gps_changed(self, state: str, lat: float, lon: float, sats: int):
        self.gps_state = state
        if state == "fix":
            self.lat = float(lat)
            self.lon = float(lon)
        self.sats = int(sats or 0)

    def start(self):
        if self._running:
            return
        self._running = True
        self.imu.start()
        self.gps.start()
        self.camera.start()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        self.stop_auto_drive("Server shutting down")
        self.motors.stop()
        self.camera.stop()
        self.imu.stop()
        self.gps.stop()

    def execute_drive(self, cmd: str):
        norm = (cmd or "stop").lower().strip()
        with self._lock:
            if self.auto_active:
                if norm in ("forward", "backward", "left", "right"):
                    self.auto_direction = norm
                    self.auto_pausing = False
                    self.auto_countdown_s = self.auto_interval_s
                    self.motors.execute(self.auto_direction)
                    self.motor_cmd = self.auto_direction
                    self.speed = self.motors._current_speed
                    self._update_reminder(pulsed=True)
                    self._record_auto_cycle()
                    return
                else:
                    self._stop_auto_unlocked("Stopped by operator — Auto Train Drive OFF")

            self.motors.execute(norm)
            self.motor_cmd = self.motors._current_cmd
            self.speed = self.motors._current_speed

    def start_auto_drive(self, interval_s: int = 5, mode: str = "train_pulse", direction: str = "forward"):
        with self._lock:
            self.auto_interval_s = max(1, int(interval_s))
            self.auto_mode = mode if mode in ("train_pulse", "stop_capture") else "train_pulse"
            self.auto_direction = direction if direction in ("forward", "backward", "left", "right") else "forward"
            self.auto_active = True
            self.auto_pausing = False
            self.auto_cycle = 1
            self.auto_countdown_s = self.auto_interval_s
            self.motors.execute(self.auto_direction)
            self.motor_cmd = self.auto_direction
            self.speed = self.motors._current_speed
            self._update_reminder(pulsed=True)
            self._record_auto_cycle()

    def stop_auto_drive(self, reason: str = ""):
        with self._lock:
            self._stop_auto_unlocked(reason)

    def _stop_auto_unlocked(self, reason: str = ""):
        was_active = self.auto_active
        self.auto_active = False
        self.auto_pausing = False
        self.motors.execute("stop")
        self.motor_cmd = "stop"
        self.speed = 0.0
        if was_active:
            self.auto_reminder = reason or "Auto Train Drive disengaged — Manual control active"

    def _update_reminder(self, pulsed: bool):
        mode_lbl = "Stop-&-Sample" if self.auto_mode == "stop_capture" else "Train Run"
        prefix = "⚡ AUTO PULSE" if pulsed else "● AUTO ACTIVE"
        self.auto_reminder = (
            f"{prefix} ({mode_lbl}) · {self.auto_direction.upper()} "
            f"· Cycle #{self.auto_cycle} · Next in {self.auto_countdown_s}s"
        )

    def _record_auto_cycle(self, auto_record_id: str | None = None):
        step_deg = 0.00003
        if self.gps_state != "fix":
            if self.auto_direction == "forward":
                self._sim_lon += step_deg
            elif self.auto_direction == "backward":
                self._sim_lon -= step_deg
            elif self.auto_direction == "left":
                self._sim_lat += step_deg
            elif self.auto_direction == "right":
                self._sim_lat -= step_deg
            lat, lon = self._sim_lat, self._sim_lon
        else:
            lat, lon = self.lat, self.lon

        saved = database.log_auto_drive_cycle(
            {
                "cycle_num": self.auto_cycle,
                "mode": self.auto_mode,
                "direction": self.auto_direction,
                "interval_s": self.auto_interval_s,
                "lat": lat,
                "lon": lon,
                "gps": f"{lat:.5f}, {lon:.5f}",
                "sats": self.sats,
                "roll": self.roll,
                "pitch": self.pitch,
                "speed_mps": self.speed,
                "median_hue": self.live_hue,
                "status": "Auto Sampled" if auto_record_id else "Auto Drive",
                "record_id": auto_record_id,
            }
        )
        self.latest_path_text = (
            f"Cycle #{self.auto_cycle} ({self.auto_direction.upper()}) → JSON & GPS ({saved['gps']})"
        )

    def _auto_sample_frame_if_available(self) -> str | None:
        frame = self.camera.capture_still()
        if frame is None:
            return None
        try:
            res = evaluate(frame)
            rec_id = database.insert_record(
                {
                    "avg_hue": res.median_hue,
                    "leaf_report": res.tone,
                    "image": encode_thumbnail(frame),
                    "raw_bgr": frame,
                    "lab_l": res.lab_l,
                    "lab_a": res.lab_a,
                    "lab_b": res.lab_b,
                    "hue_method": "median",
                    "segmentation_v_min": 12,
                    "gps": f"{self._sim_lat:.5f}, {self._sim_lon:.5f}",
                }
            )
            return rec_id
        except Exception:
            return None

    def start_instructed_roving(self, steps: list[dict]) -> dict:
        with self._lock:
            if self.instructed_active:
                return {"ok": False, "error": "Instructed roving is already active"}
            if self.auto_active:
                self._stop_auto_unlocked("Stopping Auto Train for Instructed Roving")

            self.instructed_active = True
            self.instructed_state = self._initial_instructed_state()
            self.instructed_state["active"] = True
            self.instructed_state["total_steps"] = len(steps)
            self.instructed_state["start_gps"] = f"{self.lat:.5f}, {self.lon:.5f}"
            self.instructed_thread = threading.Thread(
                target=self._instructed_worker, args=(steps,), daemon=True
            )
            self.instructed_thread.start()
            return {"ok": True, "total_steps": len(steps)}

    def stop_instructed_roving(self, reason: str = "Operator Aborted"):
        with self._lock:
            if not self.instructed_active:
                return {"ok": True, "active": False}
            self.instructed_active = False
            self.instructed_state["active"] = False
            self.instructed_state["phase"] = "aborted"
            self.instructed_state["phase_detail"] = f"Mission Aborted: {reason}"
            self.motors.execute("stop")
            self.motor_cmd = "stop"
            self.speed = 0.0
            return {"ok": True, "active": False, "reason": reason}

    def _send_esp32_cmd(self, act: str, spd: float | None = None, turn_dir: str | None = None, deg: float | None = None):
        """Dispatch commands to ESP32 over Serial bridge and/or Wi-Fi AP."""
        # 1. Serial USB bridge
        if act == "stop":
            self.motors.execute("stop")
        elif act in ("forward", "backward", "left", "right"):
            self.motors.execute(act)
        elif act == "turn_deg" and turn_dir and deg:
            self.motors._bridge._send_json({"c": "t", "dir": turn_dir, "deg": deg})

        # 2. Wi-Fi HTTP bridge failover (if ESP32 AP is active at 192.168.4.1)
        def _bg_post():
            try:
                import urllib.request
                q = f"act={act}"
                if spd is not None:
                    q += f"&spd={spd}"
                if turn_dir:
                    q += f"&dir={turn_dir}"
                if deg is not None:
                    q += f"&deg={deg}"
                urllib.request.urlopen(f"http://192.168.4.1/cmd?{q}", timeout=0.8)
            except Exception:
                pass
        threading.Thread(target=_bg_post, daemon=True).start()

    def _execute_turn(self, turn_dir: str, target_degrees: float = 90.0, timeout_s: float = 8.0) -> bool:
        """Rotate rover in turn_dir until MPU-6050 yaw changes by target_degrees with closed-loop precision."""
        turn_dir = turn_dir.lower()
        if turn_dir not in ("left", "right"):
            return True

        # Instruct ESP32 hardware closed-loop turn controller
        self._send_esp32_cmd("turn_deg", turn_dir=turn_dir, deg=target_degrees)
        self.motor_cmd = f"turn_{turn_dir}"
        self.speed = 0.42

        start_yaw = self.imu.yaw
        t0 = time.monotonic()
        actual_timeout = timeout_s if target_degrees <= 90.0 else timeout_s * 1.8

        while time.monotonic() - t0 < actual_timeout:
            if not self.instructed_active or not self._running:
                self._send_esp32_cmd("stop")
                return False

            cur_yaw = self.imu.yaw
            delta = abs(cur_yaw - start_yaw)
            if delta > 180.0:
                delta = abs(delta - 360.0)

            self.instructed_state["current_yaw"] = round(cur_yaw, 1)
            self.instructed_state["target_yaw"] = round(target_degrees, 1)
            self.instructed_state["phase_detail"] = (
                f"Turning {turn_dir.capitalize()}: {delta:.1f}° / {target_degrees:.0f}° (MPU-6050 Precision)"
            )

            # Lead angle detection — allow ESP32 active reverse brake to execute
            if delta >= (target_degrees - 1.5):
                break
            time.sleep(0.04)

        # Allow ESP32 dynamic brake pulse & settle sequence to finalize
        time.sleep(0.25)
        self.motor_cmd = "stop"
        self.speed = 0.0
        time.sleep(0.20)
        return True

    def _instructed_worker(self, steps: list[dict]):
        sec_per_m = float(STANDARDS.get("sec_per_meter", 3.0))
        completed_legs = []

        try:
            for idx, step in enumerate(steps):
                if not self.instructed_active:
                    break

                step_num = idx + 1
                step_type = str(step.get("type", "leg")).lower()
                self.instructed_state["current_step"] = step_num
                self.instructed_state["current_action"] = step_type

                if step_type == "leg":
                    dist = float(step.get("distance_m", 5.0))
                    turn_dir = str(step.get("turn", "none")).lower()
                    do_capture = bool(step.get("capture", True))

                    # Phase 1: Drive forward
                    self.instructed_state["phase"] = "driving"
                    target_s = dist * sec_per_m
                    self.instructed_state["target_s"] = target_s
                    self.instructed_state["distance_m"] = dist

                    self._send_esp32_cmd("forward", spd=0.75)
                    self.motor_cmd = "forward"
                    self.speed = self.motors._current_speed or 0.75

                    t0 = time.monotonic()
                    while time.monotonic() - t0 < target_s:
                        if not self.instructed_active:
                            break
                        elapsed = time.monotonic() - t0
                        self.instructed_state["elapsed_s"] = round(elapsed, 1)
                        self.instructed_state["phase_detail"] = (
                            f"Move {step_num}: Driving {dist:.1f}m ({elapsed:.0f}s / {target_s:.0f}s) · Gyro Heading Lock"
                        )
                        time.sleep(0.1)

                    self._send_esp32_cmd("stop")
                    self.motor_cmd = "stop"
                    self.speed = 0.0
                    time.sleep(0.3)

                    if not self.instructed_active:
                        break

                    # Phase 2: Turn via MPU-6050
                    if turn_dir in ("left", "right"):
                        self.instructed_state["phase"] = "turning"
                        ok = self._execute_turn(turn_dir, 90.0)
                        if not ok or not self.instructed_active:
                            break

                    # Phase 3: Capture & Evaluate
                    if do_capture:
                        self.instructed_state["phase"] = "capturing"
                        self.instructed_state["phase_detail"] = f"Move {step_num}: Steadying camera & analyzing crop health..."
                        time.sleep(1.0)
                        if not self.instructed_active:
                            break

                        rec_id = self._auto_sample_frame_if_available()
                        self._record_auto_cycle(auto_record_id=rec_id)
                        self.instructed_state["last_capture_id"] = rec_id
                        self.instructed_state["phase_detail"] = f"Move {step_num}: Analyzed (Record #{rec_id or '--'})"
                        time.sleep(0.5)

                    completed_legs.append({"dist": dist, "turn": turn_dir})

                elif step_type == "return":
                    # Inverted Step Retrace back to initial position
                    self.instructed_state["phase"] = "retracing"
                    self.instructed_state["phase_detail"] = "Move 4: Returning to start (Inverted Step Retrace)..."

                    if completed_legs:
                        last_leg = completed_legs[-1]
                        # First turn to align with reverse of last leg
                        if last_leg["turn"] == "left":
                            self._execute_turn("left", 90.0)
                        elif last_leg["turn"] == "right":
                            self._execute_turn("right", 90.0)
                        else:
                            self._execute_turn("right", 180.0)

                        # Retrace each leg in reverse order
                        for r_idx in range(len(completed_legs) - 1, -1, -1):
                            if not self.instructed_active:
                                break
                            r_leg = completed_legs[r_idx]
                            r_dist = r_leg["dist"]
                            r_dur = r_dist * sec_per_m
                            self.instructed_state["phase_detail"] = f"Retracing Leg {r_idx + 1}: {r_dist:.1f}m back to waypoint"

                            self._send_esp32_cmd("forward", spd=0.75)
                            self.motor_cmd = "forward"
                            self.speed = self.motors._current_speed or 0.75

                            t_r = time.monotonic()
                            while time.monotonic() - t_r < r_dur:
                                if not self.instructed_active:
                                    break
                                time.sleep(0.1)

                            self._send_esp32_cmd("stop")
                            self.motor_cmd = "stop"
                            self.speed = 0.0
                            time.sleep(0.3)

                            # Turn opposite of the previous leg's turn
                            if r_idx > 0:
                                prev_turn = completed_legs[r_idx - 1]["turn"]
                                if prev_turn == "left":
                                    self._execute_turn("right", 90.0)
                                elif prev_turn == "right":
                                    self._execute_turn("left", 90.0)

                        # Finally turn 180° to restore starting orientation
                        if self.instructed_active:
                            self.instructed_state["phase_detail"] = "Restoring initial orientation (180° turn)..."
                            self._execute_turn("right", 180.0)

            if self.instructed_active:
                self.instructed_state["phase"] = "completed"
                self.instructed_state["phase_detail"] = "Mission Completed! Rover returned to initial position."
        finally:
            self.motors.execute("stop")
            self.motor_cmd = "stop"
            self.speed = 0.0
            self.instructed_active = False
            self.instructed_state["active"] = False

    def get_instructed_status(self) -> dict:
        st = dict(self.instructed_state)
        st["active"] = self.instructed_active
        st["sec_per_meter"] = float(STANDARDS.get("sec_per_meter", 3.0))
        return st

    def _loop(self):
        last_tick = time.monotonic()
        last_imu_retry = 0.0
        last_gps_scan = 0.0
        last_esp_wifi_poll = 0.0
        while self._running:
            now = time.monotonic()

            # 1. Poll MPU-6050 IMU Gyroscope + Accelerometer over I2C1 (SDA GPIO 2, SCL GPIO 3) at 20 Hz
            if self.imu._ok or (now - last_imu_retry) >= 2.0:
                if not self.imu._ok:
                    last_imu_retry = now
                try:
                    self.imu._poll()
                except Exception:
                    pass

            # 1b. If USB serial is disconnected, sync MPU telemetry from ESP32 AP over Wi-Fi
            if not self.imu._bridge.connected and (now - last_esp_wifi_poll) >= 0.15:
                last_esp_wifi_poll = now
                try:
                    import urllib.request, json
                    req = urllib.request.Request("http://192.168.4.1/telemetry")
                    with urllib.request.urlopen(req, timeout=0.25) as resp:
                        if resp.status == 200:
                            tdata = json.loads(resp.read().decode())
                            if tdata.get("mpu_ok"):
                                self.imu._bridge._current_yaw = float(tdata.get("yaw", 0.0))
                                self.imu._bridge._current_pitch = float(tdata.get("pitch", 0.0))
                                self.imu._bridge._current_roll = float(tdata.get("roll", 0.0))
                                self.imu._ok = True
                                self.imu_ok = True
                                self.yaw = round(self.imu._bridge._current_yaw, 1)
                                self.pitch = round(self.imu._bridge._current_pitch, 1)
                                self.roll = round(self.imu._bridge._current_roll, 1)
                except Exception:
                    pass

            # 2. Poll NEO-6M GPS over UART0 (TX GPIO 14, RX GPIO 15)
            if self.gps._serial is None and (now - last_gps_scan) >= 2.0:
                last_gps_scan = now
                try:
                    self.gps._try_open()
                except Exception:
                    pass
            try:
                self.gps._poll()
            except Exception:
                pass

            # 3. Evaluate live camera frame ~4x/sec if connected
            frame = self.camera.last_frame
            if frame is not None and (now - self._last_live_eval_t) >= 0.25:
                self._last_live_eval_t = now
                try:
                    res = evaluate(frame)
                    self._last_live_result = res
                    hue_lo, hue_hi = STANDARDS["hue"]
                    in_band = (hue_lo <= res.median_hue <= hue_hi) or res.is_lcc_chart
                    self.live_hue = res.median_hue
                    self.live_panel = "4-Panel LCC" if res.is_lcc_chart else res.panel
                    self.live_in_band = in_band
                    self.live_summary = (
                        f"Coverage {res.coverage_pct:.0f}% · Lab {res.lab_l:.0f}/{res.lab_a:.0f}/{res.lab_b:.0f}"
                    )
                except NoVegetationDetected:
                    self._last_live_result = None
                    self.live_hue = None
                    self.live_panel = "--"
                    self.live_summary = "No leaf canopy in frame"
                except Exception:
                    self._last_live_result = None

            # 3b. Scene alert (human / object / plant) ~2x/sec
            if frame is not None and (now - self._last_scene_t) >= 0.5:
                self._last_scene_t = now
                try:
                    plant_mask = self._last_live_result.mask if self._last_live_result is not None else None
                    self._scene_alert = detect_scene(frame, plant_mask)
                except Exception:
                    self._scene_alert = None
            elif frame is None:
                self._scene_alert = None

            # 1-second tick for GPS status & 5s Autonomous Train Drive
            if (now - last_tick) >= 1.0:
                last_tick = now
                try:
                    self.gps._emit_status()
                except Exception:
                    pass
                with self._lock:
                    if self.auto_active:
                        if self.auto_pausing:
                            self.auto_pausing = False
                            self.auto_cycle += 1
                            self.auto_countdown_s = self.auto_interval_s
                            self.motors.execute(self.auto_direction)
                            self.motor_cmd = self.auto_direction
                            self.speed = self.motors._current_speed
                            self._update_reminder(pulsed=True)
                        else:
                            self.auto_countdown_s -= 1
                            if self.auto_countdown_s <= 0:
                                if self.auto_mode == "stop_capture":
                                    self.auto_pausing = True
                                    self.motors.execute("stop")
                                    self.motor_cmd = "stop"
                                    self.speed = 0.0
                                    self.auto_reminder = (
                                        f"Station Stop · Cycle #{self.auto_cycle} — Sampling & saving GPS path…"
                                    )
                                    rec_id = self._auto_sample_frame_if_available()
                                    self._record_auto_cycle(auto_record_id=rec_id)
                                else:
                                    self.auto_cycle += 1
                                    self.auto_countdown_s = self.auto_interval_s
                                    self.motors.execute(self.auto_direction)
                                    self.motor_cmd = self.auto_direction
                                    self.speed = self.motors._current_speed
                                    self._update_reminder(pulsed=True)
                                    self._record_auto_cycle()
                            else:
                                self._update_reminder(pulsed=False)

            time.sleep(0.05)


hub = WebHardwareHub()


@asynccontextmanager
async def lifespan(_app: FastAPI):
    hub.start()
    yield
    hub.stop()


app = FastAPI(title="RiceTrack Web Application", version="1.0.0", lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


class DriveRequest(BaseModel):
    command: str


class AutoDriveRequest(BaseModel):
    enable: bool
    mode: str = "train_pulse"
    interval_s: int = 5


class AutoModeRequest(BaseModel):
    mode: str


class CameraSelectRequest(BaseModel):
    index: int


class SampleEvalRequest(BaseModel):
    path: str


class ValidationInput(BaseModel):
    record_id: str
    manual_height: float
    manual_hue: float
    notes: str = ""


class StandardsInput(BaseModel):
    hue_lo: int
    hue_hi: int
    height_lo: int
    height_hi: int
    pole: float = 1.2
    gps_n: int
    tilt: int
    auto_drive_s: int
    sec_per_meter: float = 3.0


class InstructedStepInput(BaseModel):
    step: int
    type: str = "leg"  # "leg" or "return"
    distance_m: float = 5.0
    turn: str = "none"  # "none", "right", "left"
    capture: bool = True


class InstructedMissionRequest(BaseModel):
    preset_id: str = "preset_4step_inspection"
    steps: list[InstructedStepInput] = []


def _evaluate_and_persist_frame(img_bgr: np.ndarray) -> dict:
    orig_url = _frame_to_data_url(img_bgr)
    try:
        result = evaluate(img_bgr)
    except NoVegetationDetected as exc:
        scene = detect_scene(img_bgr)
        return {
            "error": str(exc),
            "original_image": _frame_to_data_url(draw_scene_alert(img_bgr, scene)),
            "scene_alert": _scene_alert_dict(scene),
        }
    except Exception as exc:
        return {"error": f"Evaluation failed: {exc}", "original_image": orig_url}

    seg_bgr = overlay_mask(img_bgr, result.mask, result=result)
    scene = detect_scene(img_bgr, result.mask)
    seg_bgr = draw_scene_alert(seg_bgr, scene)
    seg_url = _frame_to_data_url(seg_bgr)

    if result.is_lcc_chart and result.detected_panels:
        p_str = " · ".join(
            f"P{p.panel_num}(H:{p.median_hue:.0f}, L*:{p.lab_l:.0f})"
            for p in result.detected_panels
        )
        summary = f"4 panels: {p_str}"
    else:
        summary = (
            f"{result.panel} · Hue {result.median_hue:.0f} · "
            f"Coverage {result.coverage_pct:.0f}% · "
            f"Lab {result.lab_l:.0f}/{result.lab_a:.0f}/{result.lab_b:.0f}"
        )

    gps_str = f"{hub.lat:.5f}, {hub.lon:.5f}" if hub.gps_state == "fix" else f"{hub._sim_lat:.5f}, {hub._sim_lon:.5f}"
    rec_dict = {
        "avg_hue": result.median_hue,
        "leaf_report": result.tone,
        "image": encode_thumbnail(img_bgr),
        "raw_bgr": img_bgr,
        "seg_bgr": seg_bgr,
        "lab_l": result.lab_l,
        "lab_a": result.lab_a,
        "lab_b": result.lab_b,
        "hue_method": "median",
        "segmentation_v_min": 12,
        "gps": gps_str,
    }
    record_id = database.insert_record(rec_dict)
    saved_file = rec_dict.get("saved_file_path") or f"farm_captures/capture_{record_id}.jpg"

    return {
        "record_id": record_id,
        "status": result.status,
        "panel": result.panel,
        "tone": result.tone,
        "median_hue": result.median_hue,
        "coverage_pct": result.coverage_pct,
        "lab_l": result.lab_l,
        "lab_a": result.lab_a,
        "lab_b": result.lab_b,
        "is_lcc_chart": result.is_lcc_chart,
        "summary": summary,
        "original_image": orig_url,
        "segmented_image": seg_url,
        "scene_alert": _scene_alert_dict(scene),
        "saved_file": saved_file,
    }


@app.get("/", response_class=HTMLResponse)
def index():
    return TEMPLATE_PATH.read_text(encoding="utf-8")


@app.get("/api/telemetry")
def get_telemetry():
    return {
        "roll": hub.roll,
        "pitch": hub.pitch,
        "yaw": hub.yaw,
        "imu_ok": hub.imu_ok,
        "speed": hub.speed,
        "motor_cmd": hub.motor_cmd,
        "gps_state": hub.gps_state,
        "lat": hub.lat,
        "lon": hub.lon,
        "sats": hub.sats,
        "camera_connected": hub.camera.is_connected,
        "camera_label": f"Camera {hub.camera.camera_index}",
        "motor_link": hub.motors.link_status(),
        "live_hue": hub.live_hue,
        "live_panel": hub.live_panel,
        "live_in_band": hub.live_in_band,
        "live_summary": hub.live_summary,
        "auto_active": hub.auto_active,
        "auto_interval_s": hub.auto_interval_s,
        "auto_countdown_s": hub.auto_countdown_s,
        "auto_cycle": hub.auto_cycle,
        "auto_mode": hub.auto_mode,
        "auto_direction": hub.auto_direction,
        "auto_reminder": hub.auto_reminder,
        "latest_path_text": hub.latest_path_text,
        "scene_alert": _scene_alert_dict(hub._scene_alert),
        "instructed": hub.get_instructed_status(),
    }


def _scene_alert_dict(scene) -> dict | None:
    """Serialize a SceneAlert (human / object / plant) for the web UI."""
    if scene is None:
        return None
    return {
        "level": scene.level,
        "message": scene.message,
        "has_human": scene.has_human,
        "has_objects": scene.has_objects,
        "has_plants": scene.has_plants,
        "humans": len(scene.humans),
        "objects": [o.label for o in scene.objects],
        "plant_coverage_pct": round(scene.plant_coverage_pct, 1),
        "detector_ready": scene.detector_ready,
    }


@app.get("/api/video_feed")
def video_feed(overlay: int = Query(1)):
    def gen():
        while True:
            frame = hub.camera.last_frame
            if frame is None:
                time.sleep(0.1)
                continue
            out = frame
            if overlay and hub._last_live_result is not None:
                try:
                    out = overlay_mask(frame, hub._last_live_result.mask, result=hub._last_live_result)
                except Exception:
                    out = frame
            if overlay and hub._scene_alert is not None:
                try:
                    out = draw_scene_alert(out, hub._scene_alert)
                except Exception:
                    pass
            ok, buf = cv2.imencode(".jpg", out, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n" + buf.tobytes() + b"\r\n"
                )
            time.sleep(0.05)

    return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=frame")


@app.post("/api/drive")
def drive_rover(req: DriveRequest):
    hub.execute_drive(req.command)
    return {"ok": True, "command": hub.motor_cmd, "speed": hub.speed}


@app.post("/api/auto_drive")
def toggle_auto_drive(req: AutoDriveRequest):
    if req.enable:
        hub.start_auto_drive(interval_s=req.interval_s, mode=req.mode, direction="forward")
    else:
        hub.stop_auto_drive("Auto Train Drive stopped")
    return {"ok": True, "auto_active": hub.auto_active}


@app.post("/api/auto_drive/mode")
def change_auto_mode(req: AutoModeRequest):
    if req.mode in ("train_pulse", "stop_capture"):
        hub.auto_mode = req.mode
    return {"ok": True, "mode": hub.auto_mode}


@app.get("/api/instructed_roving/presets")
def get_instructed_presets():
    return {
        "presets": INSTRUCTED_PRESETS,
        "sec_per_meter": STANDARDS.get("sec_per_meter", 3.0),
    }


@app.post("/api/instructed_roving/start")
def start_instructed_mission(req: InstructedMissionRequest):
    steps = [s.model_dump() for s in req.steps]
    if not steps:
        for p in INSTRUCTED_PRESETS:
            if p["id"] == req.preset_id:
                steps = p["steps"]
                break
    res = hub.start_instructed_roving(steps)
    return res


@app.post("/api/instructed_roving/stop")
def stop_instructed_mission():
    return hub.stop_instructed_roving("Operator Abort via UI")


@app.get("/api/instructed_roving/status")
def get_instructed_status():
    return hub.get_instructed_status()


@app.post("/api/camera/select")
def select_camera(req: CameraSelectRequest):
    hub.camera.set_camera_index(req.index)
    return {"ok": True, "camera_index": req.index}


@app.get("/api/samples")
def list_samples():
    samples = []
    for folder, prefix in (("farm_captures", "Field Capture"), ("images", "Reference / LCC")):
        d = BASE_DIR / folder
        if d.is_dir():
            for f in sorted(d.iterdir()):
                if f.suffix.lower() in (".jpg", ".jpeg", ".png", ".bmp"):
                    if "_seg" in f.name:
                        continue  # Keep sample dropdown clean by omitting annotated mask overlays
                    samples.append(
                        {
                            "path": f"{folder}/{f.name}",
                            "label": f"{prefix}: {f.name}",
                        }
                    )
    return {"samples": samples}


@app.post("/api/evaluate_sample")
def evaluate_sample(req: SampleEvalRequest):
    target = (BASE_DIR / req.path).resolve()
    if not str(target).startswith(str(BASE_DIR)) or not target.is_file():
        raise HTTPException(status_code=404, detail="Sample file not found")
    img = cv2.imread(str(target))
    if img is None:
        return {"error": f"Unable to decode image {req.path}"}
    return _evaluate_and_persist_frame(img)


@app.post("/api/evaluate_upload")
async def evaluate_upload(file: UploadFile = File(...)):
    raw = await file.read()
    arr = np.frombuffer(raw, dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if img is None:
        return {"error": "Could not decode uploaded image file."}
    return _evaluate_and_persist_frame(img)


@app.post("/api/capture_live")
@app.post("/api/capture_evaluate")
def capture_live():
    frame = hub.camera.capture_still()
    if frame is None:
        return {"ok": False, "error": "No live camera frame available. Upload or select a sample photo."}
    res = _evaluate_and_persist_frame(frame)
    if isinstance(res, dict):
        res["ok"] = "error" not in res
    return res


@app.get("/api/auto_path")
def get_auto_path():
    return {"path": database.fetch_auto_drive_path(limit=50)}


@app.get("/api/records")
def get_records(
    month: str = "All",
    plot: str = "All",
    variant: str = "All",
    status: str = "All",
):
    rows = database.fetch_records(month=month, plot=plot, variant=variant, status=status)
    formatted = []
    for r in rows:
        formatted.append(
            {
                "id": r["id"],
                "captured_at": r["captured_at"],
                "captured_short": _format_date_short(r["captured_at"]),
                "captured_full": _format_date_full(r["captured_at"]),
                "plot": r["plot"],
                "variant": r["variant"],
                "gps": r["gps"],
                "avg_height": float(r["avg_height"]),
                "avg_hue": float(r["avg_hue"]),
                "height_report": r["height_report"],
                "leaf_report": r["leaf_report"],
                "status": r["status"],
                "lab_l": r.get("lab_l"),
                "lab_a": r.get("lab_a"),
                "lab_b": r.get("lab_b"),
            }
        )
    return {
        "records": formatted,
        "filters": {
            "months": database.distinct_months(),
            "plots": database.distinct("plot"),
            "variants": database.distinct("variant"),
            "statuses": database.distinct("status"),
        },
    }


@app.get("/api/records/{record_id}/image")
def get_record_image(record_id: str):
    with database._connect() as conn:
        row = conn.execute("SELECT image FROM records WHERE id = ?", (record_id,)).fetchone()
    if not row or not row["image"]:
        raise HTTPException(status_code=404, detail="Image not found")
    return Response(content=bytes(row["image"]), media_type="image/jpeg")


@app.get("/api/records/export.csv")
def export_records_csv():
    rows = database.fetch_records()
    buf = io.StringIO()
    writer = csv.writer(buf)
    writer.writerow(
        [
            "id",
            "captured_at",
            "plot",
            "variant",
            "gps",
            "avg_height_cm",
            "median_hue",
            "lab_l",
            "lab_a",
            "lab_b",
            "leaf_report",
            "height_report",
            "status",
        ]
    )
    for r in rows:
        writer.writerow(
            [
                r["id"],
                r["captured_at"],
                r["plot"],
                r["variant"],
                r["gps"],
                r["avg_height"],
                r["avg_hue"],
                r.get("lab_l"),
                r.get("lab_a"),
                r.get("lab_b"),
                r["leaf_report"],
                r["height_report"],
                r["status"],
            ]
        )
    return Response(
        content=buf.getvalue(),
        media_type="text/csv",
        headers={"Content-Disposition": 'attachment; filename="ricetrack_records.csv"'},
    )


@app.get("/api/validation")
def get_validation():
    return {
        "metrics": database.validation_metrics(),
        "rows": database.fetch_validation_pairs(),
    }


@app.post("/api/validation")
def add_validation(req: ValidationInput):
    try:
        val_id = database.insert_validation_reading(
            record_id=req.record_id,
            manual_height=req.manual_height,
            manual_hue=req.manual_hue,
            notes=req.notes,
        )
        return {"ok": True, "validation_id": val_id}
    except Exception as exc:
        return JSONResponse(status_code=400, content={"error": str(exc)})


@app.get("/api/validation/export.csv")
def export_validation_csv():
    rows = database.fetch_validation_pairs()
    buf = io.StringIO()
    writer = csv.writer(buf)
    writer.writerow(
        [
            "validation_id",
            "capture",
            "captured_at",
            "plot",
            "cv_height_cm",
            "manual_height_cm",
            "height_error_cm",
            "cv_median_hue",
            "manual_hue",
            "hue_error",
            "cv_lab_l",
            "cv_lab_a",
            "cv_lab_b",
            "color_score",
            "height_score",
            "health_index",
            "status",
            "notes",
        ]
    )
    for row in rows:
        writer.writerow(
            [
                row["validation_id"],
                row["record_id"],
                row["captured_at"],
                row["plot"],
                row["cv_height"],
                row["manual_height"],
                round(row["height_error"], 2),
                row["cv_hue"],
                row["manual_hue"],
                round(row["hue_error"], 2),
                row["cv_lab_l"],
                row["cv_lab_a"],
                row["cv_lab_b"],
                round(row["color_score"], 2),
                round(row["height_score"], 2),
                round(row["health_index"], 2),
                row["status"],
                row["notes"],
            ]
        )
    return Response(
        content=buf.getvalue(),
        media_type="text/csv",
        headers={"Content-Disposition": 'attachment; filename="validation.csv"'},
    )


@app.get("/api/standards")
def get_standards():
    return {
        "variety": STANDARDS["variety"],
        "hue": list(STANDARDS["hue"]),
        "height": list(STANDARDS["height"]),
        "pole": STANDARDS["pole"],
        "gps_n": STANDARDS["gps_n"],
        "tilt": STANDARDS["tilt"],
        "auto_drive_s": STANDARDS.get("auto_drive_s", 5),
        "sec_per_meter": STANDARDS.get("sec_per_meter", 3.0),
    }


@app.post("/api/standards")
def update_standards(req: StandardsInput):
    lo, hi = min(req.hue_lo, req.hue_hi), max(req.hue_lo, req.hue_hi)
    h_lo, h_hi = min(req.height_lo, req.height_hi), max(req.height_lo, req.height_hi)
    STANDARDS["hue"] = (lo, hi)
    STANDARDS["height"] = (h_lo, h_hi)
    STANDARDS["pole"] = round(max(0.3, min(3.0, float(req.pole))), 2)
    STANDARDS["gps_n"] = max(1, min(30, req.gps_n))
    STANDARDS["tilt"] = max(1, min(45, req.tilt))
    STANDARDS["auto_drive_s"] = max(1, min(60, req.auto_drive_s))
    STANDARDS["sec_per_meter"] = max(0.5, min(30.0, float(req.sec_per_meter)))
    hub.auto_interval_s = STANDARDS["auto_drive_s"]
    try:
        database.recompute_all_statuses()
    except Exception:
        pass
    return get_standards()


@app.get("/api/manuals/download.pdf")
def download_pdf_manual():
    pdf_path = pdf_manuals.generate_pdf_file()
    return Response(
        content=pdf_path.read_bytes(),
        media_type="application/pdf",
        headers={
            "Content-Disposition": f'attachment; filename="{pdf_path.name}"'
        },
    )



if __name__ == "__main__":
    import socket
    import webbrowser

    host = os.environ.get("RICETRACK_HOST", "0.0.0.0")
    port = int(os.environ.get("RICETRACK_PORT", "8000"))

    # Find the first available port starting at 8000 so Errno 10048 never occurs
    for candidate_port in range(port, port + 20):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                sock.bind((host, candidate_port))
                port = candidate_port
                break
            except OSError:
                continue

    url = f"http://localhost:{port}"
    print(f"RiceTrack Web App running at {url} (LAN: http://{host}:{port})")
    
    def _safe_open_browser():
        try:
            if os.name == "nt" or bool(os.environ.get("DISPLAY")):
                webbrowser.open(url)
        except Exception:
            pass
            
    threading.Timer(1.0, _safe_open_browser).start()
    uvicorn.run(app, host=host, port=port, log_level="info")


