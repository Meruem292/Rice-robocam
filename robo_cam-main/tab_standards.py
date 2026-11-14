from PySide6.QtCore import QRectF, Qt, QTimer, Signal
from PySide6.QtGui import QColor, QIntValidator, QPainter, QPainterPath
from PySide6.QtWidgets import (
    QComboBox, QFrame, QGridLayout, QHBoxLayout, QLineEdit, QPushButton, QVBoxLayout, QWidget
)
import database
from common import GOOD, GREEN, LINE, MUTED, TEXT, STANDARDS, Chip, button, card, kv_row, lbl


class ThresholdSpinBox(QFrame):
    """
    Modern numeric threshold input with '-' / '+' stepper buttons, direct keyboard
    entry, Up/Down arrow key stepping, and mouse wheel support.
    Avoids the Qt 6 Windows 11 QSpinBox subcontrol text-clipping issue.
    """
    valueChanged = Signal(int)

    def __init__(self, low: int, high: int, value: int, suffix: str = ""):
        super().__init__()
        self._low = int(low)
        self._high = int(high)
        self._value = max(self._low, min(self._high, int(value)))
        self._suffix = suffix

        self.setObjectName("threshbox")
        self.setStyleSheet(
            f"QFrame#threshbox {{ background: #0e1411; border: 1px solid #2a3931; border-radius: 10px; }}"
            f"QFrame#threshbox:hover {{ border: 1px solid {GREEN}; }}"
        )
        self.setFixedHeight(36)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(4, 2, 4, 2)
        layout.setSpacing(2)

        self.minus_btn = QPushButton("−")
        self.minus_btn.setFixedSize(26, 28)
        self.minus_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self.minus_btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.minus_btn.setStyleSheet(
            f"QPushButton {{ background: #16201b; color: {TEXT}; border: none; border-radius: 6px; "
            f"font-size: 15px; font-weight: 800; padding: 0px; }}"
            f"QPushButton:hover {{ background: {GREEN}; color: #ffffff; }}"
        )
        self.minus_btn.clicked.connect(lambda: self.stepBy(-1))

        self.edit = QLineEdit()
        self.edit.setFixedWidth(48)
        self.edit.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.edit.setValidator(QIntValidator(self._low, self._high, self))
        self.edit.setStyleSheet(
            f"QLineEdit {{ background: transparent; border: none; color: {TEXT}; "
            f"font-size: 14px; font-weight: 700; padding: 0px; }}"
        )
        self.edit.setText(str(self._value))
        self.edit.textEdited.connect(self._on_text_edited)
        self.edit.editingFinished.connect(self._on_editing_finished)

        self.plus_btn = QPushButton("+")
        self.plus_btn.setFixedSize(26, 28)
        self.plus_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self.plus_btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.plus_btn.setStyleSheet(
            f"QPushButton {{ background: #16201b; color: {TEXT}; border: none; border-radius: 6px; "
            f"font-size: 15px; font-weight: 800; padding: 0px; }}"
            f"QPushButton:hover {{ background: {GREEN}; color: #ffffff; }}"
        )
        self.plus_btn.clicked.connect(lambda: self.stepBy(1))

        layout.addWidget(self.minus_btn)
        layout.addWidget(self.edit)
        if suffix:
            sfx = lbl(suffix, 12, MUTED, "700")
            sfx.setStyleSheet(f"color: {MUTED}; font-size: 12px; font-weight: 700; background: transparent; border: none; padding-right: 2px;")
            layout.addWidget(sfx)
        layout.addWidget(self.plus_btn)

    def value(self) -> int:
        return self._value

    def setValue(self, val: int):
        clamped = max(self._low, min(self._high, int(val)))
        changed = clamped != self._value
        self._value = clamped
        if self.edit.text() != str(clamped):
            self.edit.setText(str(clamped))
        if changed:
            self.valueChanged.emit(self._value)

    def stepBy(self, delta: int):
        self.setValue(self._value + delta)

    def _on_text_edited(self, text: str):
        try:
            v = int(text)
        except ValueError:
            return
        if self._low <= v <= self._high and v != self._value:
            self._value = v
            self.valueChanged.emit(self._value)

    def _on_editing_finished(self):
        try:
            v = int(self.edit.text())
        except ValueError:
            v = self._value
        self.setValue(v)

    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        if delta > 0:
            self.stepBy(1)
        elif delta < 0:
            self.stepBy(-1)
        event.accept()


def make_spin(low, high, value, suffix="", width=116):
    return ThresholdSpinBox(low, high, value, suffix=suffix)


def range_row(label_text, spin_lo, spin_hi, unit=""):
    row = QHBoxLayout()
    row.setSpacing(8)
    row.addWidget(lbl(label_text, 13, MUTED, "600"))
    row.addStretch()
    row.addWidget(spin_lo)
    row.addWidget(lbl("to", 13, MUTED, "600"))
    row.addWidget(spin_hi)
    if unit:
        row.addWidget(lbl(unit, 13, MUTED, "700"))
    return row


class HueBar(QWidget):
    MIN_H, MAX_H = 30, 85

    def __init__(self, lo, hi):
        super().__init__()
        self.lo, self.hi = lo, hi
        self.setFixedHeight(46)

    def set_range(self, lo, hi):
        self.lo, self.hi = min(lo, hi), max(lo, hi)
        self.update()

    def _x(self, value):
        pad = 16.0
        usable = max(1.0, self.width() - 2 * pad)
        span = max(1, self.MAX_H - self.MIN_H)
        return pad + max(0.0, min(1.0, (value - self.MIN_H) / span)) * usable

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        pad = 16.0
        bar = QRectF(pad, 4, max(10.0, self.width() - 2 * pad), 14)
        clip = QPainterPath()
        clip.addRoundedRect(bar, 7, 7)
        p.setClipPath(clip)
        p.setPen(Qt.PenStyle.NoPen)

        y_end = max(self.MIN_H, min(self.lo - 6, self.MAX_H))
        lg_end = max(y_end, min(self.lo, self.MAX_H))
        hg_end = max(lg_end, min(self.hi, self.MAX_H))
        segments = [
            (self.MIN_H, y_end, "#e3b341"),
            (y_end, lg_end, "#85c846"),
            (lg_end, hg_end, "#2ea043"),
            (hg_end, self.MAX_H, "#165c26"),
        ]
        for a, b, color in segments:
            if b > a:
                p.setBrush(QColor(color))
                p.drawRect(QRectF(self._x(a), 4, self._x(b) - self._x(a) + 0.5, 14))
        p.setClipping(False)

        # Draw threshold handle markers
        for value in (self.lo, self.hi):
            x = self._x(value)
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(QColor("#ffffff"))
            p.drawRoundedRect(QRectF(x - 2, 2, 4, 18), 2, 2)

        font = p.font()
        font.setPixelSize(11)
        font.setBold(True)
        p.setFont(font)
        p.setPen(QColor(TEXT))
        for value in (self.lo, self.hi):
            x = self._x(value)
            p.drawText(QRectF(x - 22, 23, 44, 18), Qt.AlignmentFlag.AlignCenter, str(value))


class StandardsPage(QWidget):
    saved = Signal()

    def __init__(self):
        super().__init__()
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(12)

        top = QHBoxLayout()
        self.variety = QComboBox()
        self.variety.addItem(f"Variety: {STANDARDS['variety']}")
        top.addWidget(self.variety)
        top.addStretch()
        self.pdf_btn = button(" Generate PDF Manual & Calibration", "primary")
        self.pdf_btn.clicked.connect(self._export_pdf_manual)
        top.addWidget(self.pdf_btn)
        root.addLayout(top)

        grid = QGridLayout()
        grid.setSpacing(14)

        hue_card, hl = card(spacing=14)
        hl.addWidget(lbl("Leaf Color (HSV Hue)", 15, TEXT, "800"))
        hl.addWidget(lbl(
            "Median HSV Hue is used for leaf classification; CIELAB L*a*b* is logged alongside each sample.",
            12, MUTED, wrap=True,
        ))
        self.hue_lo = make_spin(25, 85, STANDARDS["hue"][0])
        self.hue_hi = make_spin(25, 85, STANDARDS["hue"][1])
        self.hue_bar = HueBar(*STANDARDS["hue"])
        hl.addWidget(self.hue_bar)
        hl.addLayout(range_row("Healthy Hue Band", self.hue_lo, self.hue_hi))
        hl.addStretch()
        self.hue_lo.valueChanged.connect(self._on_threshold_edited)
        self.hue_hi.valueChanged.connect(self._on_threshold_edited)
        grid.addWidget(hue_card, 0, 0)

        height_card, gl = card(spacing=14)
        gl.addWidget(lbl("Plant Height", 15, TEXT, "800"))
        self.h_lo = make_spin(10, 250, STANDARDS["height"][0])
        self.h_hi = make_spin(10, 250, STANDARDS["height"][1])
        gl.addLayout(range_row("Healthy Height Range", self.h_lo, self.h_hi, "cm"))
        gl.addLayout(kv_row("Camera Mounting Height", lbl(f"{STANDARDS['pole']} m", 13, GOOD, "700")))
        gl.addStretch()
        self.h_lo.valueChanged.connect(self._on_threshold_edited)
        self.h_hi.valueChanged.connect(self._on_threshold_edited)
        grid.addWidget(height_card, 0, 1)

        cap_card, cl = card(spacing=14)
        cl.addWidget(lbl("Capture & Auto-Drive Limits", 15, TEXT, "800"))
        self.gps_n = make_spin(1, 30, STANDARDS["gps_n"])
        self.tilt = make_spin(1, 45, STANDARDS["tilt"], "°")
        self.auto_drive_s = make_spin(1, 60, STANDARDS.get("auto_drive_s", 5), "s")
        cl.addLayout(kv_row("GPS readings per capture", self.gps_n))
        cl.addLayout(kv_row("Max tilt angle", self.tilt))
        cl.addLayout(kv_row("Train Auto-Drive cycle", self.auto_drive_s))
        cl.addStretch()
        self.gps_n.valueChanged.connect(self._on_threshold_edited)
        self.tilt.valueChanged.connect(self._on_threshold_edited)
        self.auto_drive_s.valueChanged.connect(self._on_threshold_edited)
        grid.addWidget(cap_card, 1, 0)

        log_card, ll = card(spacing=14)
        ll.addWidget(lbl("Calibration Info", 15, TEXT, "800"))
        ll.addLayout(kv_row("Variety", lbl(STANDARDS["variety"], 13, TEXT, "700")))
        ll.addLayout(kv_row("Reference", lbl("IRRI 4-Panel LCC", 13, GOOD, "700")))
        ll.addLayout(kv_row("Validated by", lbl("Sir Jasper Talada", 13, TEXT, "600")))

        btn_row = QHBoxLayout()
        btn_row.setSpacing(10)
        self.reset_btn = button("Reset Defaults", "dark")
        self.reset_btn.clicked.connect(self._reset_defaults)
        self.save_btn = button("Save Standards", "primary")
        self.save_btn.clicked.connect(self._save)
        btn_row.addWidget(self.reset_btn, 1)
        btn_row.addWidget(self.save_btn, 2)
        ll.addLayout(btn_row)
        ll.addStretch()
        grid.addWidget(log_card, 1, 1)

        for c in (0, 1):
            grid.setColumnStretch(c, 1)
        root.addLayout(grid)
        root.addStretch()

    def _on_threshold_edited(self, *_args):
        hue_a, hue_b = self.hue_lo.value(), self.hue_hi.value()
        lo, hi = (min(hue_a, hue_b), max(hue_a, hue_b))
        h_a, h_b = self.h_lo.value(), self.h_hi.value()
        h_lo, h_hi = (min(h_a, h_b), max(h_a, h_b))
        self.hue_bar.set_range(lo, hi)
        # Apply live so Live Camera Monitoring, Records & Validation immediately reflect threshold changes
        STANDARDS["hue"] = (lo, hi)
        STANDARDS["height"] = (h_lo, h_hi)
        STANDARDS["gps_n"] = self.gps_n.value()
        STANDARDS["tilt"] = self.tilt.value()
        STANDARDS["auto_drive_s"] = self.auto_drive_s.value()
        try:
            database.recompute_all_statuses()
        except Exception:
            pass
        self.saved.emit()

    def _reset_defaults(self):
        self.hue_lo.setValue(48)
        self.hue_hi.setValue(75)
        self.h_lo.setValue(98)
        self.h_hi.setValue(101)
        self.gps_n.setValue(5)
        self.tilt.setValue(15)
        self.auto_drive_s.setValue(5)
        self._save()

    def _save(self):
        self._on_threshold_edited()
        self.save_btn.setText("Saved ✓")
        QTimer.singleShot(1500, lambda: self.save_btn.setText("Save Standards"))

    def _export_pdf_manual(self):
        import os
        import pdf_manuals

        out_path = pdf_manuals.generate_pdf_file()
        self.pdf_btn.setText("PDF Saved ✓")
        QTimer.singleShot(
            1800,
            lambda: self.pdf_btn.setText(" Generate PDF Manual & Calibration"),
        )
        try:
            if os.name == "nt":
                os.startfile(str(out_path))  # type: ignore[attr-defined]
        except Exception:
            pass

