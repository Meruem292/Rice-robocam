import os
import time
from typing import Any
import cv2

from PySide6.QtCore import QEvent, QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QColor, QIcon, QKeySequence, QPainter, QPainterPath, QPen, QShortcut
from PySide6.QtWidgets import (
    QApplication, QComboBox, QFileDialog, QFrame, QGridLayout, QHBoxLayout, QLabel, QMessageBox,
    QProgressBar, QVBoxLayout, QWidget
)
from common import (
    BG, BLUE, GOOD, GREEN, LINE, MUTED, RED_BG, RED_LINE, RED_TEXT, TEXT, STANDARDS, WARN_TEXT,
    Chip, button, card, kv_row, lbl, make_pixmap, set_prop
)
from vision import (
    NoVegetationDetected, cv_to_qpixmap, detect_scene, draw_scene_alert, encode_thumbnail,
    evaluate, overlay_mask,
)
import database
from esp32_hardware import ESP32HardwareController


class AttitudeTile(QFrame):
    def __init__(self, name):
        super().__init__()
        self.setObjectName("tile")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 10, 14, 10)
        layout.setSpacing(2)
        self.name = lbl(name.upper(), 10, MUTED, "700")
        self.value = lbl("--", 18, TEXT, "800")
        layout.addWidget(self.name)
        layout.addWidget(self.value)
        self.set_value("--", False)

    def set_value(self, text, danger):
        bg = RED_BG if danger else "#0e1411"
        border = RED_LINE if danger else "#1f2b25"
        self.setStyleSheet(
            f"QFrame#tile {{ background: {bg}; border: 1px solid {border}; border-radius: 12px; }}"
        )
        self.name.setStyleSheet(
            f"font-size: 10px; font-weight: 700; color: {'#ff948d' if danger else MUTED};"
        )
        self.value.setStyleSheet(
            f"font-size: 18px; font-weight: 800; color: {RED_TEXT if danger else TEXT};"
        )
        self.value.setText(text)


class StatusBanner(QFrame):
    """Always-visible status strip: red = problem, green/blue/orange = auto-drive reminder or status."""
    STYLES = {
        "red":    (RED_BG,    RED_LINE,  RED_TEXT,  "info"),
        "green":  ("#0f2617", "#1e522e", GOOD,      "check"),
        "blue":   ("#0d233a", "#1f4772", "#79c0ff", "wheel"),
        "orange": ("#2e210b", "#5c4214", WARN_TEXT, "info"),
        "gray":   ("#18201c", "#26332c", MUTED,     "info"),
    }

    def __init__(self):
        super().__init__()
        self.setObjectName("statusbanner")
        self._kind = None
        lay = QHBoxLayout(self)
        lay.setContentsMargins(16, 10, 16, 10)
        lay.setSpacing(10)
        self.icon = QLabel()
        self.icon.setFixedSize(16, 16)
        self.text = lbl("", 13, TEXT, "600")
        lay.addWidget(self.icon)
        lay.addWidget(self.text, 1)

    def set_state(self, kind, text):
        if kind != self._kind:
            bg, line, fg, icon = self.STYLES[kind]
            self.setStyleSheet(
                f"QFrame#statusbanner {{ background: {bg}; border: 1px solid {line}; border-radius: 12px; }}"
            )
            self.icon.setPixmap(make_pixmap(icon, fg, 16))
            self.text.setStyleSheet(f"font-size: 13px; font-weight: 600; color: {fg};")
            self._kind = kind
        self.text.setText(text)


class CameraView(QWidget):
    PLACEHOLDER = "No camera connected"
    SUBTITLE = "Connect a camera or click to load an image"

    clicked = Signal()

    def __init__(self):
        super().__init__()
        self._frame: Any = None
        self._hud_badge = ""
        self.setMinimumSize(360, 260)
        self.setCursor(Qt.CursorShape.PointingHandCursor)

    def set_frame(self, pixmap, hud_badge: str = ""):
        self._frame = pixmap
        self._hud_badge = hud_badge
        self.update()

    def mousePressEvent(self, event):
        self.clicked.emit()
        super().mousePressEvent(event)

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()

        clip = QPainterPath()
        clip.addRoundedRect(QRectF(0, 0, w, h), 10, 10)
        p.setClipPath(clip)

        if self._frame is not None and not self._frame.isNull():
            scaled = self._frame.scaled(
                w, h, Qt.AspectRatioMode.KeepAspectRatioByExpanding,
                Qt.TransformationMode.SmoothTransformation,
            )
            p.drawPixmap((w - scaled.width()) // 2, (h - scaled.height()) // 2, scaled)

            p.setPen(QPen(QColor("#232e28"), 1.0))
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawRoundedRect(QRectF(0.5, 0.5, w - 1.0, h - 1.0), 10, 10)

            if self._hud_badge:
                bw = min(w - 24, 320)
                bh = 26
                bx = (w - bw) / 2
                by = h - bh - 10
                p.setPen(QPen(QColor("#2c3d34"), 1))
                p.setBrush(QColor(11, 15, 13, 210))
                p.drawRoundedRect(QRectF(bx, by, bw, bh), 6, 6)
                f = p.font()
                f.setPixelSize(11)
                f.setBold(True)
                p.setFont(f)
                p.setPen(QColor(TEXT))
                p.drawText(QRectF(bx, by, bw, bh), Qt.AlignmentFlag.AlignCenter, self._hud_badge)
            return

        p.fillRect(QRectF(0, 0, w, h), QColor("#0d1310"))
        p.setPen(QPen(QColor("#223029"), 1.0))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawRoundedRect(QRectF(0.5, 0.5, w - 1.0, h - 1.0), 10, 10)

        cam_pm = make_pixmap("camera", MUTED, 24)
        p.drawPixmap(int(w / 2 - 12), int(h / 2 - 36), cam_pm)

        font = p.font()
        font.setPixelSize(13)
        font.setBold(True)
        p.setFont(font)
        p.setPen(QColor(TEXT))
        p.drawText(
            QRectF(24, h / 2 - 4, w - 48, 20),
            Qt.AlignmentFlag.AlignCenter,
            self.PLACEHOLDER,
        )

        font.setPixelSize(12)
        font.setBold(False)
        p.setFont(font)
        p.setPen(QColor(MUTED))
        p.drawText(
            QRectF(32, h / 2 + 18, w - 64, 32),
            Qt.AlignmentFlag.AlignCenter | Qt.TextFlag.TextWordWrap,
            self.SUBTITLE,
        )


class ManualView(QWidget):
    command = Signal(str)
    record_saved = Signal(str)
    camera_index_changed = Signal(int)
    auto_drive_toggled = Signal(bool, str, int)  # (enable, mode, interval_s)
    auto_drive_mode_changed = Signal(str)
    auto_path_updated = Signal()

    LIVE_EVAL_INTERVAL_S = 0.25  # Evaluate live stream ~4x/sec when Live Analysis is ON
    SCENE_EVAL_INTERVAL_S = 0.5  # Human/Object/Plant scene alert ~2x/sec

    KEY_TO_CMD = {
        Qt.Key.Key_W: "forward",
        Qt.Key.Key_Up: "forward",
        Qt.Key.Key_S: "backward",
        Qt.Key.Key_Down: "backward",
        Qt.Key.Key_A: "left",
        Qt.Key.Key_Left: "left",
        Qt.Key.Key_D: "right",
        Qt.Key.Key_Right: "right",
        Qt.Key.Key_Space: "stop",
    }

    def __init__(self):
        super().__init__()
        self.tilt_limit = STANDARDS["tilt"]
        self.auto_interval_s = STANDARDS.get("auto_drive_s", 5)
        self._auto_active = False
        self._auto_direction = "forward"
        self._auto_reminder_text = ""
        self._auto_cycle = 0
        self._auto_countdown_s = self.auto_interval_s

        self._roll, self._pitch, self._speed = 0.0, 0.0, 0.0
        self._imu_ok = False
        self._cam_connected = False
        self._cam_label = "Camera 0"
        self._live_frame_bgr: Any = None
        self._live_overlay_enabled = True
        self._last_live_eval_t = 0.0
        self._live_eval_result: Any = None
        self._last_scene_t = 0.0
        self._scene_alert: Any = None
        self._pad_buttons: dict[str, Any] = {}
        self._active_drive_keys: set[int] = set()

        self._gps: tuple[float, float] | None = None
        self._gps_sats = 0
        self._gps_samples = 0
        self._sim_lat = 15.67170
        self._sim_lon = 120.89280
        self._last_auto_record_id: str | None = None
        self._saved_note = ""
        self.image_bgr: Any = None
        self.segmented_bgr: Any = None
        self.showing_segmented = False
        self.last_result: Any = None
        self.record_saved_for_image = False

        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(12)

        self.alert = StatusBanner()
        root.addWidget(self.alert)

        # Scene alert: Human (red) / Object (orange) / Plant (green)
        self.scene_banner = StatusBanner()
        self.scene_banner.set_state("gray", "Scene Alert · waiting for camera or photo")
        root.addWidget(self.scene_banner)

        body = QHBoxLayout()
        body.setSpacing(14)
        body.addWidget(self._build_left(), 5)
        body.addWidget(self._build_right(), 4)
        root.addLayout(body, 1)

        QShortcut(QKeySequence("C"), self).activated.connect(self._on_capture_button)
        QShortcut(QKeySequence("T"), self).activated.connect(self._toggle_auto_shortcut)

        app = QApplication.instance()
        if app is not None:
            app.installEventFilter(self)
            app.aboutToQuit.connect(lambda: app.removeEventFilter(self))

        self.set_attitude(0.0, 0.0, 0.0)
        self.set_gps("none", 0.0, 0.0, 0)

    def eventFilter(self, watched, event):
        try:
            if self.isVisible() and event.type() in (QEvent.Type.KeyPress, QEvent.Type.KeyRelease):
                key = event.key()
                cmd = self.KEY_TO_CMD.get(key)
                if cmd is not None and not event.isAutoRepeat():
                    if event.type() == QEvent.Type.KeyPress:
                        if cmd == "stop":
                            self._active_drive_keys.clear()
                            if self._auto_active:
                                self.stop_auto_drive("Stopped by Spacebar — Auto Train Drive OFF")
                            self._highlight_pad(None)
                            self.command.emit("stop")
                        else:
                            self._active_drive_keys.add(key)
                            if self._auto_active:
                                self._auto_direction = cmd
                            self._highlight_pad(cmd)
                            self.command.emit(cmd)
                        return True
                    elif event.type() == QEvent.Type.KeyRelease:
                        self._active_drive_keys.discard(key)
                        if not self._active_drive_keys and not self._auto_active:
                            self._highlight_pad(None)
                            self.command.emit("stop")
                        return True
        except RuntimeError:
            pass
        return False

    def _highlight_pad(self, active_cmd: str | None):
        for cmd, btn in self._pad_buttons.items():
            if cmd == active_cmd:
                btn.setStyleSheet(f"background: {GREEN}; color: #ffffff; border: 1px solid #3fb950; border-radius: 12px; font-size: 16px; font-weight: 700;")
            else:
                btn.setStyleSheet("")
                set_prop(btn, "btn", "pad")

    def _build_left(self):
        frame, lay = card(spacing=12)
        top = QHBoxLayout()
        top.setSpacing(8)

        self.cam_selector = QComboBox()
        self.cam_selector.addItem("Camera 0", 0)
        self.cam_selector.addItem("Camera 1", 1)
        self.cam_selector.addItem("Camera 2", 2)
        self.cam_selector.setFixedWidth(130)
        self.cam_selector.currentIndexChanged.connect(self._on_camera_selected)
        top.addWidget(self.cam_selector)

        self.live_hud_btn = button("LCC Overlay", "seg", checkable=True)
        self.live_hud_btn.setChecked(True)
        self.live_hud_btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.live_hud_btn.clicked.connect(self._toggle_live_overlay)
        top.addWidget(self.live_hud_btn)

        top.addStretch()
        self.source_chip = Chip("Standby", "gray")
        self.status_chip = self.source_chip
        top.addWidget(self.source_chip)
        lay.addLayout(top)

        self.camera = CameraView()
        self.camera.setToolTip("Click to capture or toggle segmented view")
        self.camera.clicked.connect(self._on_camera_clicked)
        lay.addWidget(self.camera, 1)

        tiles = QHBoxLayout()
        tiles.setSpacing(10)
        self.roll_tile = AttitudeTile("Roll")
        self.pitch_tile = AttitudeTile("Pitch")
        self.speed_tile = AttitudeTile("Speed")
        self.hue_tile = AttitudeTile("Hue")
        for tile in (self.roll_tile, self.pitch_tile, self.speed_tile, self.hue_tile):
            tiles.addWidget(tile, 1)
        lay.addLayout(tiles)
        return frame

    def _build_right(self):
        holder = QWidget()
        col = QVBoxLayout(holder)
        col.setContentsMargins(0, 0, 0, 0)
        col.setSpacing(10)

        controls, cl = card(margins=(16, 12, 16, 12), spacing=8)
        drive_head = QHBoxLayout()
        drive_head.addWidget(lbl("Rover & Train Drive", 15, TEXT, "800"))
        drive_head.addStretch()
        self.drive_mode_chip = Chip("MANUAL", "gray", "wheel")
        drive_head.addWidget(self.drive_mode_chip)
        cl.addLayout(drive_head)

        # Autonomous Train Drive (Every 5s) control bar + reminder strip
        auto_box = QFrame()
        auto_box.setProperty("inset", "true")
        auto_lay = QVBoxLayout(auto_box)
        auto_lay.setContentsMargins(10, 8, 10, 8)
        auto_lay.setSpacing(6)

        auto_row = QHBoxLayout()
        auto_row.setSpacing(8)
        self.auto_drive_btn = button(
            f" Auto Train ({self.auto_interval_s}s)",
            "dark",
            "play",
            checkable=True,
        )
        self.auto_drive_btn.setFixedHeight(34)
        self.auto_drive_btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.auto_drive_btn.setToolTip(
            "Toggle Autonomous Train Drive (runs every 5 seconds with live Auto-Drive reminder · Shortcut: T)"
        )
        self.auto_drive_btn.clicked.connect(self._on_auto_drive_clicked)

        self.auto_mode_combo = QComboBox()
        self.auto_mode_combo.addItem(f"Every {self.auto_interval_s}s Run", "train_pulse")
        self.auto_mode_combo.addItem(f"{self.auto_interval_s}s + Sample", "stop_capture")
        self.auto_mode_combo.setFixedHeight(34)
        self.auto_mode_combo.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.auto_mode_combo.currentIndexChanged.connect(self._on_auto_mode_selected)

        auto_row.addWidget(self.auto_drive_btn, 1)
        auto_row.addWidget(self.auto_mode_combo, 1)
        auto_lay.addLayout(auto_row)

        self.auto_reminder_lbl = lbl(
            f"Train Mode Standby · Runs every {self.auto_interval_s}s when Auto Drive is ON",
            11,
            MUTED,
            "600",
            wrap=False,
        )
        self.auto_reminder_lbl.setMinimumHeight(18)
        auto_lay.addWidget(self.auto_reminder_lbl)

        self.auto_cycle_bar = QProgressBar()
        self.auto_cycle_bar.setTextVisible(False)
        self.auto_cycle_bar.setRange(0, self.auto_interval_s)
        self.auto_cycle_bar.setValue(0)
        auto_lay.addWidget(self.auto_cycle_bar)
        cl.addWidget(auto_box)
        self._style_auto_drive_btn(False)

        pad = QGridLayout()
        pad.setSpacing(6)
        for glyph, cmd, tip, r, c in (
            ("↑", "forward", "Forward (W / Up)", 0, 1),
            ("←", "left", "Left (A / Left)", 1, 0),
            ("▢", "stop", "Stop (Space)", 1, 1),
            ("→", "right", "Right (D / Right)", 1, 2),
            ("↓", "backward", "Backward (S / Down)", 2, 1),
        ):
            b = button(glyph, "pad")
            b.setFixedSize(42, 42)
            b.setToolTip(tip)
            b.setFocusPolicy(Qt.FocusPolicy.NoFocus)
            self._pad_buttons[cmd] = b
            if cmd == "stop":
                b.clicked.connect(lambda _=False: self._on_manual_pad_stop())
            else:
                b.pressed.connect(lambda k=cmd: self._on_manual_pad_pressed(k))
                b.released.connect(lambda: self._on_manual_pad_released())
            pad.addWidget(b, r, c)
        pad_row = QHBoxLayout()
        pad_row.addStretch()
        pad_row.addLayout(pad)
        pad_row.addStretch()
        cl.addLayout(pad_row)

        kb_hint = lbl(
            "WASD / Arrows / Pad steer in Manual & Auto (5s) · T Auto · Space Stop",
            11,
            MUTED,
            align=Qt.AlignmentFlag.AlignHCenter,
        )
        cl.addWidget(kb_hint)

        actions = QHBoxLayout()
        actions.setSpacing(10)
        self.upload_btn = button(" Upload Photo", "dark", "download")
        self.upload_btn.setFixedHeight(36)
        self.upload_btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.upload_btn.clicked.connect(self._on_upload_button)
        self.capture_btn = button("Capture", "primary")
        self.capture_btn.setFixedHeight(36)
        self.capture_btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.capture_btn.clicked.connect(self._on_capture_button)
        actions.addWidget(self.upload_btn, 1)
        actions.addWidget(self.capture_btn, 1)
        cl.addLayout(actions)

        self.capture_note = lbl("", 11, MUTED, align=Qt.AlignmentFlag.AlignHCenter)
        self.capture_note.setVisible(False)
        cl.addWidget(self.capture_note)
        col.addWidget(controls)

        status, sl2 = card(margins=(16, 12, 16, 12), spacing=7)
        head = QHBoxLayout()
        head.addWidget(lbl("Telemetry", 15, TEXT, "800"))
        head.addStretch()
        self.capture_id = Chip("", "gray")
        self.capture_id.setVisible(False)
        head.addWidget(self.capture_id)
        sl2.addLayout(head)

        self.image_value = lbl("Disconnected", 13, MUTED, "600")
        sl2.addLayout(kv_row("Camera", self.image_value, MUTED))
        self.live_lcc_value = lbl("--", 13, MUTED, "600")
        sl2.addLayout(kv_row("Leaf Color", self.live_lcc_value, MUTED))
        self.gps_value = lbl("No fix", 13, MUTED, "600")
        sl2.addLayout(kv_row("GPS", self.gps_value, MUTED))
        self.gps_bar = QProgressBar()
        self.gps_bar.setTextVisible(False)
        self.gps_bar.setRange(0, STANDARDS["gps_n"])
        self.gps_bar.setValue(0)
        sl2.addWidget(self.gps_bar)
        self.auto_path_value = lbl("Standby (auto_drive_path.json)", 12, MUTED, "600")
        sl2.addLayout(kv_row("Auto Path Log", self.auto_path_value, MUTED))
        self.analysis_value = lbl("--", 12, MUTED, align=Qt.AlignmentFlag.AlignRight, wrap=True)
        sl2.addLayout(kv_row("Summary", self.analysis_value, MUTED))
        col.addWidget(status)
        col.addStretch()
        return holder

    def _style_auto_drive_btn(self, active: bool):
        if active:
            self.auto_drive_btn.setStyleSheet(
                f"QPushButton {{ background: {GREEN}; color: #ffffff; border: 1px solid #3fb950; "
                f"border-radius: 9px; padding: 0px 12px; font-size: 12px; font-weight: 700; }}"
                f"QPushButton:hover {{ background: #36b54d; border: 1px solid #56d364; }}"
            )
        else:
            self.auto_drive_btn.setStyleSheet(
                f"QPushButton {{ background: #171f1b; color: {TEXT}; border: 1px solid {LINE}; "
                f"border-radius: 9px; padding: 0px 12px; font-size: 12px; font-weight: 600; }}"
                f"QPushButton:hover {{ background: #1f2924; border: 1px solid #34463c; }}"
            )

    def _on_manual_pad_pressed(self, cmd: str):
        if self._auto_active:
            self._auto_direction = cmd
        self._highlight_pad(cmd)
        self.command.emit(cmd)

    def _on_manual_pad_released(self):
        if not self._auto_active:
            self._highlight_pad(None)
            self.command.emit("stop")

    def _on_manual_pad_stop(self):
        if self._auto_active:
            self.stop_auto_drive("Stopped by operator — Auto Train Drive OFF")
        self._highlight_pad(None)
        self.command.emit("stop")

    def _toggle_auto_shortcut(self):
        self.auto_drive_btn.setChecked(not self.auto_drive_btn.isChecked())
        self._on_auto_drive_clicked()

    def _on_auto_drive_clicked(self):
        enable = self.auto_drive_btn.isChecked()
        if enable:
            tilt = max(abs(self._roll), abs(self._pitch))
            if tilt > self.tilt_limit:
                self.auto_drive_btn.setChecked(False)
                self._refresh_banner()
                return
            mode = self.auto_mode_combo.currentData() or "train_pulse"
            self.auto_drive_toggled.emit(True, str(mode), self.auto_interval_s)
        else:
            self.stop_auto_drive("Auto Train Drive stopped")

    def _on_auto_mode_selected(self, _idx: int):
        mode = self.auto_mode_combo.currentData()
        if mode:
            self.auto_drive_mode_changed.emit(str(mode))

    def stop_auto_drive(self, reason: str = ""):
        if self.auto_drive_btn.isChecked():
            self.auto_drive_btn.setChecked(False)
        self.auto_drive_toggled.emit(False, reason, self.auto_interval_s)

    def set_auto_drive_interval(self, interval_s: int):
        self.auto_interval_s = max(1, int(interval_s))
        self.auto_cycle_bar.setRange(0, self.auto_interval_s)
        self.auto_mode_combo.setItemText(0, f"Every {self.auto_interval_s}s Run")
        self.auto_mode_combo.setItemText(1, f"{self.auto_interval_s}s + Sample")
        if not self._auto_active:
            self.auto_drive_btn.setText(f" Auto Train ({self.auto_interval_s}s)")
            self.auto_reminder_lbl.setText(
                f"Train Mode Standby · Runs every {self.auto_interval_s}s when Auto Drive is ON"
            )

    def set_auto_drive_state(self, active: bool, reminder_text: str, cycle: int, countdown_s: int):
        self._auto_active = active
        self._auto_reminder_text = reminder_text
        self._auto_cycle = cycle
        self._auto_countdown_s = countdown_s

        if self.auto_drive_btn.isChecked() != active:
            self.auto_drive_btn.setChecked(active)

        if active:
            for d in ("forward", "backward", "left", "right"):
                if f" {d.upper()} " in reminder_text:
                    self._auto_direction = d
                    break
            short_dir = {"forward": "FWD", "backward": "REV", "left": "LEFT", "right": "RIGHT"}.get(
                self._auto_direction, self._auto_direction.upper()
            )
            self.drive_mode_chip.set_text(f"AUTO · {short_dir} ({self.auto_interval_s}s)", "green")
            self.auto_drive_btn.setText(f" Stop Auto ({countdown_s}s)")
            self.auto_drive_btn.setIcon(QIcon(make_pixmap("pause", "#ffffff", 16)))
            self._style_auto_drive_btn(True)
            self.auto_reminder_lbl.setText(reminder_text)
            self.auto_reminder_lbl.setStyleSheet(f"font-size: 11px; font-weight: 600; color: {GOOD};")
            self.auto_cycle_bar.setRange(0, self.auto_interval_s)
            elapsed = max(0, self.auto_interval_s - countdown_s)
            self.auto_cycle_bar.setValue(self.auto_interval_s if countdown_s == 0 else elapsed)
            self._highlight_pad(self._auto_direction if countdown_s > 0 else "stop")
        else:
            self.drive_mode_chip.set_text("MANUAL", "gray")
            self.auto_drive_btn.setText(f" Auto Train ({self.auto_interval_s}s)")
            self.auto_drive_btn.setIcon(QIcon(make_pixmap("play", TEXT, 16)))
            self._style_auto_drive_btn(False)
            self.auto_reminder_lbl.setText(
                reminder_text or f"Train Mode Standby · Runs every {self.auto_interval_s}s when Auto Drive is ON"
            )
            self.auto_reminder_lbl.setStyleSheet(f"font-size: 11px; font-weight: 600; color: {MUTED};")
            self.auto_cycle_bar.setValue(0)
            self._highlight_pad(None)

        self._refresh_banner()

    def _on_camera_selected(self, _idx: int):
        cam_idx = self.cam_selector.currentData()
        if cam_idx is not None:
            self.camera_index_changed.emit(int(cam_idx))

    def _toggle_live_overlay(self):
        self._live_overlay_enabled = self.live_hud_btn.isChecked()

    def set_camera_available(self, connected: bool, label: str = "Camera 0"):
        self._cam_connected = connected
        self._cam_label = label
        if self.image_bgr is None:
            if connected:
                self.source_chip.set_text("Live", "green")
                self._set_value(self.image_value, label, GOOD)
            else:
                self._live_frame_bgr = None
                self.camera.set_frame(None)
                self.source_chip.set_text("Standby", "gray")
                self._set_value(self.image_value, "Disconnected", MUTED)
                self.hue_tile.set_value("--", False)
        self._update_capture_enabled()

    def set_live_camera_frame(self, frame_bgr):
        """Receive live BGR frame from the active camera and update live monitoring."""
        self._live_frame_bgr = frame_bgr
        if self.image_bgr is not None or frame_bgr is None:
            return

        now = time.monotonic()
        if (now - self._last_live_eval_t) >= self.LIVE_EVAL_INTERVAL_S:
            self._last_live_eval_t = now
            try:
                res = evaluate(frame_bgr)
                self._live_eval_result = res
                hue_lo, hue_hi = STANDARDS["hue"]
                in_band = (hue_lo <= res.median_hue <= hue_hi) or res.is_lcc_chart
                self.hue_tile.set_value(f"{res.median_hue:.0f}", not in_band)
                self._set_value(
                    self.live_lcc_value,
                    f"{'4-Panel LCC' if res.is_lcc_chart else res.panel} · Hue {res.median_hue:.0f}",
                    GOOD if in_band else WARN_TEXT,
                )
                self.analysis_value.setText(
                    f"Coverage {res.coverage_pct:.0f}% · Lab {res.lab_l:.0f}/{res.lab_a:.0f}/{res.lab_b:.0f}"
                )
            except NoVegetationDetected:
                self._live_eval_result = None
                self.hue_tile.set_value("--", False)
                self._set_value(self.live_lcc_value, "--", MUTED)
                self.analysis_value.setText("No leaf canopy in frame")
            except Exception:
                self._live_eval_result = None

        # Scene alert (human / object / plant) at ~2 Hz to keep the UI responsive
        if (now - self._last_scene_t) >= self.SCENE_EVAL_INTERVAL_S:
            self._last_scene_t = now
            plant_mask = self._live_eval_result.mask if self._live_eval_result is not None else None
            self._update_scene_alert(frame_bgr, plant_mask)

        hud_badge = ""
        if self._auto_active:
            hud_badge = (
                f"AUTO DRIVE (TRAIN MODE) · Cycle #{self._auto_cycle} · "
                f"Next {self.auto_interval_s}s run in {self._auto_countdown_s}s"
            )

        if self._live_overlay_enabled and self._live_eval_result is not None:
            try:
                vis = overlay_mask(frame_bgr, self._live_eval_result.mask, result=self._live_eval_result)
                vis = draw_scene_alert(vis, self._scene_alert)
                self.camera.set_frame(cv_to_qpixmap(vis), hud_badge)
                return
            except Exception:
                pass

        if self._live_overlay_enabled and self._scene_alert is not None:
            try:
                self.camera.set_frame(cv_to_qpixmap(draw_scene_alert(frame_bgr, self._scene_alert)), hud_badge)
                return
            except Exception:
                pass

        self.camera.set_frame(cv_to_qpixmap(frame_bgr), hud_badge)

    def _update_scene_alert(self, img_bgr, plant_mask=None):
        """Run human/object/plant detection and update the Scene Alert banner."""
        try:
            scene = detect_scene(img_bgr, plant_mask)
        except Exception:
            return None
        self._scene_alert = scene
        self.scene_banner.set_state(scene.level, f"Scene Alert · {scene.message}")
        return scene

    def capture_from_csi(self, frame_bgr):
        """Freeze a frame captured from the live camera stream and evaluate/save it."""
        if frame_bgr is None:
            self.analysis_value.setText("No camera frame available.")
            return
        self.image_bgr = frame_bgr.copy()
        self.segmented_bgr = None
        self.showing_segmented = False
        self.last_result = None
        self.record_saved_for_image = False

        self.camera.set_frame(cv_to_qpixmap(self.image_bgr))
        self._set_value(self.image_value, "Captured frame", GOOD)
        self.source_chip.set_text("Captured", "green")
        self._set_image_mode(True)
        self._evaluate_and_save()

    def _on_camera_clicked(self):
        if self.image_bgr is None:
            if not self._cam_connected:
                self._upload_image()
            else:
                self._on_capture_button()
        else:
            self._toggle_view()

    def _on_upload_button(self):
        if self.image_bgr is None:
            self._upload_image()
        else:
            self._cancel_image()

    def _on_capture_button(self):
        if self.image_bgr is not None:
            if not self.record_saved_for_image:
                self._evaluate_and_save()
            else:
                self._cancel_image()
            return
        if self._cam_connected and self._live_frame_bgr is not None:
            self.capture_from_csi(self._live_frame_bgr)
            return
        self._capture()

    def _upload_image(self):
        default_dir = "farm_captures" if os.path.isdir("farm_captures") else ""
        path, _ = QFileDialog.getOpenFileName(
            self, "Select Image", default_dir,
            "Images (*.png *.jpg *.jpeg *.bmp *.tif *.tiff)",
        )
        if not path:
            return

        img = cv2.imread(path)
        if img is None:
            QMessageBox.warning(self, "Unable to Load Image", f"Could not read:\n{path}")
            return

        self.image_bgr = img
        self.segmented_bgr = None
        self.showing_segmented = False
        self.last_result = None
        self.record_saved_for_image = False

        self.camera.set_frame(cv_to_qpixmap(img))
        self._set_value(self.image_value, os.path.basename(path), GOOD)
        self.capture_id.setVisible(False)
        self.source_chip.set_text("Photo", "blue")
        self._saved_note = ""
        self.analysis_value.setText("Press Evaluate to analyze and save.")
        self._set_image_mode(True)
        self._update_capture_enabled()

    def _cancel_image(self):
        self.image_bgr = None
        self.segmented_bgr = None
        self.showing_segmented = False
        self.last_result = None
        self.record_saved_for_image = False
        self._saved_note = ""
        self.capture_id.setVisible(False)
        self.analysis_value.setText("--")
        self.capture_note.setText("")
        self.capture_note.setVisible(False)
        self._set_image_mode(False)

        if self._cam_connected and self._live_frame_bgr is not None:
            self.camera.set_frame(cv_to_qpixmap(self._live_frame_bgr))
            self.source_chip.set_text("Live", "green")
            self._set_value(self.image_value, self._cam_label, GOOD)
        else:
            self.camera.set_frame(None)
            self.camera.setToolTip("Click to upload a photo")
            self.source_chip.set_text("Standby", "gray")
            self._set_value(self.image_value, "Disconnected", MUTED)
            self.hue_tile.set_value("--", False)
        self._update_capture_enabled()

    def _set_image_mode(self, active):
        if active:
            self.upload_btn.setText(" Resume Live" if self._cam_connected else " Clear Photo")
            self.upload_btn.setIcon(QIcon(make_pixmap("close", TEXT, 16)))
            self.capture_btn.setText("Evaluate")
            self.camera.setToolTip("Click to toggle between original and segmented view")
        else:
            self.upload_btn.setText(" Upload Photo")
            self.upload_btn.setIcon(QIcon(make_pixmap("download", TEXT, 16)))
            self.capture_btn.setText("Capture")

    def _toggle_view(self):
        if self.segmented_bgr is None or self.image_bgr is None:
            return
        self.showing_segmented = not self.showing_segmented
        frame = self.segmented_bgr if self.showing_segmented else self.image_bgr
        badge = "Segmented overlay · Click for original" if self.showing_segmented else "Original image · Click for overlay"
        self.camera.set_frame(cv_to_qpixmap(frame), badge)

    def _evaluate_and_save(self):
        if self.image_bgr is None:
            self.analysis_value.setText("Capture or upload a photo first.")
            return

        gps_text = self._gps_text()
        self.analysis_value.setText("Evaluating…")

        try:
            result = evaluate(self.image_bgr)
        except NoVegetationDetected as exc:
            self.last_result = None
            self.segmented_bgr = None
            self.showing_segmented = False
            scene = self._update_scene_alert(self.image_bgr)
            if scene is not None:
                self.camera.set_frame(cv_to_qpixmap(draw_scene_alert(self.image_bgr, scene)))
            self.source_chip.set_text("No vegetation", "orange")
            self.analysis_value.setText(str(exc))
            return
        except Exception as exc:
            self.last_result = None
            self.segmented_bgr = None
            self.showing_segmented = False
            self.source_chip.set_text("Error", "red")
            self.analysis_value.setText(f"Evaluation failed: {exc}")
            return

        self.last_result = result
        self.segmented_bgr = overlay_mask(self.image_bgr, result.mask, result=result)
        scene = self._update_scene_alert(self.image_bgr, result.mask)
        self.segmented_bgr = draw_scene_alert(self.segmented_bgr, scene)
        self.showing_segmented = True
        self.camera.set_frame(
            cv_to_qpixmap(self.segmented_bgr),
            "Segmented overlay · Click for original",
        )

        healthy = result.status == "Healthy"
        self.hue_tile.set_value(f"{result.median_hue:.0f}", not healthy)
        self._set_value(
            self.live_lcc_value,
            f"{'4-Panel LCC' if result.is_lcc_chart else result.panel} · Hue {result.median_hue:.0f}",
            GOOD if healthy else RED_TEXT,
        )
        self.source_chip.set_text(
            "Calibrated" if result.is_lcc_chart else result.status,
            "green" if healthy else "red",
        )
        if result.is_lcc_chart and result.detected_panels:
            p_str = " · ".join(
                f"P{p.panel_num}(H:{p.median_hue:.0f}, L*:{p.lab_l:.0f})"
                for p in result.detected_panels
            )
            self.analysis_value.setText(f"4 panels: {p_str}")
        else:
            self.analysis_value.setText(
                f"{result.panel} · Hue {result.median_hue:.0f} · "
                f"Coverage {result.coverage_pct:.0f}% · "
                f"Lab {result.lab_l:.0f}/{result.lab_a:.0f}/{result.lab_b:.0f}"
            )

        record = {
            "avg_hue": result.median_hue,
            "leaf_report": result.tone,
            "image": encode_thumbnail(self.image_bgr),
            "raw_bgr": self.image_bgr,
            "seg_bgr": self.segmented_bgr,
            "lab_l": result.lab_l,
            "lab_a": result.lab_a,
            "lab_b": result.lab_b,
            "hue_method": "median",
            "segmentation_v_min": 12,
        }
        if gps_text:
            record["gps"] = gps_text

        try:
            record_id = database.insert_record(record)
        except Exception as exc:
            self.source_chip.set_text("DB error", "red")
            self.analysis_value.setText(f"Save failed: {exc}")
            return

        self.record_saved_for_image = True
        saved_file = record.get("saved_file_path") or f"farm_captures/capture_{record_id}.jpg"
        self.capture_btn.setText("Back to Live" if self._cam_connected else "Saved ✓")
        self.capture_btn.setEnabled(bool(self._cam_connected))
        self.capture_id.set_text(record_id, "green")
        self.capture_id.setVisible(True)
        self._set_value(self.image_value, f"Saved ({record_id})", GOOD)
        self.capture_note.setText(f"Saved: {saved_file}")
        self.capture_note.setVisible(True)
        self.record_saved.emit(record_id)

    def auto_sample_live_frame(self):
        """Non-blocking autonomous sample capture during 5s Train Station Stop if live canopy is present."""
        if self.image_bgr is not None or self._live_frame_bgr is None:
            return
        tilt = max(abs(self._roll), abs(self._pitch))
        if tilt > self.tilt_limit:
            return
        try:
            result = evaluate(self._live_frame_bgr)
        except Exception:
            return
        record = {
            "avg_hue": result.median_hue,
            "leaf_report": result.tone,
            "image": encode_thumbnail(self._live_frame_bgr),
            "raw_bgr": self._live_frame_bgr,
            "lab_l": result.lab_l,
            "lab_a": result.lab_a,
            "lab_b": result.lab_b,
            "hue_method": "median",
            "segmentation_v_min": 12,
        }
        gps_text = self._gps_text()
        if gps_text:
            record["gps"] = gps_text
        try:
            record_id = database.insert_record(record)
            self._last_auto_record_id = record_id
            self.capture_id.set_text(f"Auto {record_id}", "green")
            self.capture_id.setVisible(True)
            self.record_saved.emit(record_id)
        except Exception:
            pass

    def on_auto_cycle_completed(self, cycle_num: int, mode: str, direction: str, interval_s: int):
        """Save every 5-second Auto Train Drive cycle to BOTH auto_drive_path.json and SQLite GPS Path."""
        if self._gps is not None:
            lat, lon = self._gps
        else:
            # Step progressive PhilRice corridor coordinates by ~3.0m per 5s cycle when indoor/no GPS fix
            step_deg = 0.00003
            if direction == "forward":
                self._sim_lon += step_deg
            elif direction == "backward":
                self._sim_lon -= step_deg
            elif direction == "left":
                self._sim_lat += step_deg
            elif direction == "right":
                self._sim_lat -= step_deg
            lat, lon = self._sim_lat, self._sim_lon

        live_hue = self._live_eval_result.median_hue if self._live_eval_result is not None else None
        saved = database.log_auto_drive_cycle(
            {
                "cycle_num": cycle_num,
                "mode": mode,
                "direction": direction,
                "interval_s": interval_s,
                "lat": lat,
                "lon": lon,
                "gps": f"{lat:.5f}, {lon:.5f}",
                "sats": self._gps_sats,
                "roll": self._roll,
                "pitch": self._pitch,
                "speed_mps": self._speed,
                "median_hue": live_hue,
                "status": "Auto Sampled" if self._last_auto_record_id else "Auto Drive",
                "record_id": self._last_auto_record_id,
            }
        )
        self._last_auto_record_id = None
        self._set_value(
            self.auto_path_value,
            f"Cycle #{cycle_num} ({direction.upper()}) → JSON & GPS ({saved['gps']})",
            GOOD,
        )
        self.auto_path_updated.emit()

    def _capture(self):
        if self.capture_btn.isEnabled():
            self.command.emit("capture")

    def set_tilt_limit(self, limit):
        self.tilt_limit = limit
        self.set_attitude(self._roll, self._pitch, self._speed)

    def _update_capture_enabled(self):
        tilted = max(abs(self._roll), abs(self._pitch)) > self.tilt_limit
        if self.image_bgr is not None:
            self.capture_btn.setEnabled(not self.record_saved_for_image or self._cam_connected)
        else:
            self.capture_btn.setEnabled(not tilted)

    @staticmethod
    def _set_value(label, text, color):
        label.setText(text)
        label.setStyleSheet(f"font-size: 13px; font-weight: 600; color: {color};")

    def set_gps(self, state, lat, lon, sats):
        target_n = STANDARDS["gps_n"]
        self.gps_bar.setRange(0, target_n)
        self._gps_sats = int(sats or 0)
        if state == "fix":
            self._gps = (lat, lon)
            self._gps_samples = min(target_n, self._gps_samples + 1)
            self.gps_bar.setValue(self._gps_samples)
            self._set_value(self.gps_value, f"{lat:.5f}, {lon:.5f} ({sats} sats)", GOOD)
        elif state == "searching":
            self._gps = None
            self._gps_samples = 0
            self.gps_bar.setValue(0)
            self._set_value(self.gps_value, f"Searching ({sats} sats)", WARN_TEXT)
        else:
            self._gps = None
            self._gps_samples = 0
            self.gps_bar.setValue(0)
            self._set_value(self.gps_value, "No fix", MUTED)

    def _gps_text(self):
        return f"{self._gps[0]:.5f}, {self._gps[1]:.5f}" if self._gps else ""

    def set_motor_state(self, cmd: str, speed_mps: float, _gpio_ready: bool):
        self._speed = speed_mps
        self.speed_tile.set_value(f"{speed_mps:.1f} m/s", False)

    def set_imu_available(self, ok):
        self._imu_ok = ok
        if not ok:
            self._roll = self._pitch = 0.0
        self.set_attitude(self._roll, self._pitch)

    def _refresh_banner(self):
        tilt = max(abs(self._roll), abs(self._pitch))
        tilted = tilt > self.tilt_limit
        if tilted:
            if self._auto_active:
                self.stop_auto_drive(
                    f"Safety cutoff: Tilt {tilt:.0f}° > {self.tilt_limit}° — Auto Train Drive stopped"
                )
            self.alert.set_state(
                "red", f"Tilt {tilt:.0f}° exceeds {self.tilt_limit}° limit. Level the rover before driving/capturing."
            )
            self.alert.setVisible(True)
        elif self._auto_active:
            banner_kind = "blue" if self._auto_countdown_s == self.auto_interval_s else "green"
            self.alert.set_state(
                banner_kind,
                f"REMINDER: {self._auto_reminder_text} · Press Space or Stop to return to Manual",
            )
            self.alert.setVisible(True)
        else:
            self.alert.setVisible(False)

    def set_attitude(self, roll, pitch, speed=None):
        if speed is None:
            speed = self._speed
        self._roll, self._pitch, self._speed = roll, pitch, speed
        limit = self.tilt_limit
        self.roll_tile.set_value(f"{roll:.0f}°", abs(roll) > limit)
        self.pitch_tile.set_value(f"{pitch:.0f}°", abs(pitch) > limit)
        self.speed_tile.set_value(f"{speed:.1f} m/s", False)
        self._refresh_banner()
        self._update_capture_enabled()


class ControlPage(QWidget):
    command = Signal(str)
    record_saved = Signal(str)
    auto_path_updated = Signal()

    def __init__(self):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.manual = ManualView()
        layout.addWidget(self.manual)

        self.manual.command.connect(self._handle_command)
        self.manual.record_saved.connect(self.record_saved)
        self.manual.camera_index_changed.connect(self._on_camera_index_changed)
        self.manual.auto_drive_toggled.connect(self._on_auto_drive_toggled)
        self.manual.auto_drive_mode_changed.connect(self._on_auto_drive_mode_changed)
        self.manual.auto_path_updated.connect(self.auto_path_updated)

        # Unified Hardware Controller (Live Camera, MPU-6050 IMU, NEO-6M GPS, 2x BTS7960 Motors + 5s Auto Train Drive)
        self.hw = ESP32HardwareController(parent=self)
        self.hw.imu_reading.connect(self.manual.set_attitude)
        self.hw.imu_state.connect(self.manual.set_imu_available)
        self.hw.gps_changed.connect(self.manual.set_gps)
        self.hw.motor_state.connect(self.manual.set_motor_state)
        self.hw.camera_frame.connect(self.manual.set_live_camera_frame)
        self.hw.camera_state.connect(self.manual.set_camera_available)
        self.hw.auto_drive_state.connect(self.manual.set_auto_drive_state)
        self.hw.auto_capture_requested.connect(self.manual.auto_sample_live_frame)
        self.hw.auto_cycle_completed.connect(self.manual.on_auto_cycle_completed)

        app = QApplication.instance()
        if app is not None:
            app.aboutToQuit.connect(self.hw.stop)
        self.hw.start()

    def _on_camera_index_changed(self, index: int):
        self.hw.set_camera_index(index)

    def _on_auto_drive_toggled(self, enable: bool, mode_or_reason: str, interval_s: int):
        if enable:
            self.hw.start_auto_train_drive(interval_s=interval_s, mode=mode_or_reason, direction="forward")
        else:
            self.hw.stop_auto_train_drive(reason=mode_or_reason)

    def _on_auto_drive_mode_changed(self, mode: str):
        self.hw.set_auto_drive_mode(mode)

    def _handle_command(self, cmd: str):
        if cmd == "capture":
            frame = self.hw.capture_csi_frame()
            if frame is not None:
                self.manual.capture_from_csi(frame)
            else:
                self.manual._upload_image()
        else:
            self.hw.execute_motor_command(cmd)
        self.command.emit(cmd)

    def set_attitude(self, roll, pitch, speed=None):
        self.manual.set_attitude(roll, pitch, speed)

    def set_tilt_limit(self, limit):
        self.manual.set_tilt_limit(limit)

    def set_auto_drive_interval(self, interval_s: int):
        self.manual.set_auto_drive_interval(interval_s)
        self.hw.set_auto_drive_interval(interval_s)
