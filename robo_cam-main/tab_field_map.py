from typing import TypedDict
from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QColor, QPainter, QPen
from PySide6.QtWidgets import QHBoxLayout, QToolTip, QVBoxLayout, QWidget
import database
from common import BLUE, CARD, GOOD, GREEN, LINE, MUTED, RED_PIN, TEXT, Chip, button, lbl


class PinSpec(TypedDict):
    id: str
    x: float
    y: float
    bad: bool
    r: float
    acc: float
    when: str
    detail: str


class FieldMap(QWidget):
    open_record = Signal(str)

    PINS: list[PinSpec] = [
        {"id": "A-01-03", "x": 0.145, "y": 0.79, "bad": False, "r": 6.0, "acc": 1.6, "when": "10:12 AM", "detail": "Plot A-01 · 99.4 cm · hue 50 · healthy green"},
        {"id": "A-03-01", "x": 0.310, "y": 0.79, "bad": False, "r": 6.0, "acc": 1.9, "when": "10:31 AM", "detail": "Plot A-02 · 99.9 cm · hue 49 · healthy green"},
        {"id": "A-03-02", "x": 0.485, "y": 0.79, "bad": False, "r": 6.0, "acc": 1.4, "when": "10:44 AM", "detail": "Plot A-02 · 98.7 cm · hue 55 · healthy green"},
        {"id": "A-02-02", "x": 0.665, "y": 0.79, "bad": True,  "r": 6.0, "acc": 2.1, "when": "10:59 AM", "detail": "Plot A-03 · 100.1 cm · hue 43 · light green"},
        {"id": "A-02-06", "x": 0.840, "y": 0.79, "bad": True,  "r": 9.0, "acc": 1.8, "when": "11:53 AM", "detail": "Plot A-04 · 96.2 cm · hue 42 · maturing panicles"},
    ]

    def __init__(self):
        super().__init__()
        self.selected = "A-02-06"
        self._auto_path: list[dict] = []
        self.setMouseTracking(True)
        self.setMinimumHeight(260)
        self.refresh_auto_path()

    def refresh_auto_path(self) -> int:
        try:
            self._auto_path = database.fetch_auto_drive_path(limit=30)
        except Exception:
            self._auto_path = []
        self.update()
        return len(self._auto_path)

    def counts(self):
        bad = sum(1 for pin in self.PINS if pin["bad"])
        return len(self.PINS) - bad, bad

    def select(self, pin_id):
        self.selected = pin_id
        self.update()

    def _pos(self, pin: PinSpec) -> QPointF:
        return QPointF(pin["x"] * self.width(), pin["y"] * self.height())

    def _pin_at(self, point) -> PinSpec | None:
        for pin in reversed(self.PINS):
            c = self._pos(pin)
            if (c.x() - point.x()) ** 2 + (c.y() - point.y()) ** 2 <= (pin["r"] + 5) ** 2:
                return pin
        return None

    def _selected_pin(self) -> PinSpec | None:
        return next((pin for pin in self.PINS if pin["id"] == self.selected), None)

    def _popup_rect(self, pin: PinSpec) -> QRectF:
        pw, ph = 228, 64
        c = self._pos(pin)
        x = min(max(c.x() - pw * 0.61, 8), self.width() - pw - 8)
        y = c.y() - pin["r"] - 12 - ph
        if y < 8:
            y = c.y() + pin["r"] + 10
        return QRectF(x, y, pw, ph)

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()

        # Modern Dark GIS Field Canvas matching actual PhilRice Experimental Plots
        p.setPen(QPen(QColor(LINE), 1.2))
        p.setBrush(QColor("#0e1511"))
        p.drawRoundedRect(QRectF(1, 1, w - 2, h - 2), 16, 16)

        # Side-by-side experimental rice plots separated by irrigation furrows
        plots = [
            ("PLOT A-01 · NSIC Rc 222", QRectF(0.025 * w, 0.06 * h, 0.220 * w, 0.58 * h), False),
            ("PLOT A-02 · NSIC Rc 222", QRectF(0.265 * w, 0.06 * h, 0.220 * w, 0.58 * h), False),
            ("PLOT A-03 · NSIC Rc 222", QRectF(0.515 * w, 0.06 * h, 0.220 * w, 0.58 * h), True),
            ("PLOT A-04 · NSIC Rc 222", QRectF(0.755 * w, 0.06 * h, 0.220 * w, 0.58 * h), True),
        ]

        font = p.font()
        font.setPixelSize(11)
        font.setBold(True)
        p.setFont(font)

        for name, rect, needs_insp in plots:
            border_col = "#6e521b" if needs_insp else "#245233"
            fill_col = "#1c2414" if needs_insp else "#13261a"
            p.setPen(QPen(QColor(border_col), 1.3))
            p.setBrush(QColor(fill_col))
            p.drawRoundedRect(rect, 8, 8)

            # Subtle vertical crop row lines inside each plot
            p.setPen(QPen(QColor("#243b2a" if not needs_insp else "#33381d"), 1.0))
            for r_idx in range(1, 6):
                rx = rect.left() + rect.width() * (r_idx / 6.0)
                p.drawLine(QPointF(rx, rect.top() + 30), QPointF(rx, rect.bottom() - 8))

            # White plot marker placard inside the plot (matching actual farm signboards)
            sign_w, sign_h = 34, 16
            sx = rect.center().x() - sign_w / 2
            sy = rect.top() + rect.height() * 0.52
            p.setPen(QPen(QColor("#8fa196"), 1.0))
            p.setBrush(QColor("#e8f0ec"))
            p.drawRoundedRect(QRectF(sx, sy, sign_w, sign_h), 3, 3)
            sf = p.font()
            sf.setPixelSize(8)
            sf.setBold(True)
            p.setFont(sf)
            p.setPen(QColor("#111814"))
            p.drawText(QRectF(sx, sy, sign_w, sign_h), Qt.AlignmentFlag.AlignCenter, "Rc222")

            p.setFont(font)
            p.setPen(QColor("#e3b341" if needs_insp else GOOD))
            p.drawText(
                rect.adjusted(10, 8, -6, 0),
                Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft,
                name,
            )

        # Plastic Rat-Barrier Fence separating experimental plots from the capture dike
        barrier_y = 0.675 * h
        p.setPen(QPen(QColor("#9fb1a6"), 2.2))
        p.drawLine(QPointF(0.025 * w, barrier_y), QPointF(0.975 * w, barrier_y))
        # Wooden stakes along the plastic barrier
        for s_idx in range(15):
            bx = w * (0.03 + s_idx * (0.94 / 14.0))
            p.drawLine(QPointF(bx, barrier_y - 6), QPointF(bx, barrier_y + 5))

        bf = p.font()
        bf.setPixelSize(10)
        bf.setBold(True)
        p.setFont(bf)
        p.setPen(QColor(MUTED))
        p.drawText(QPointF(0.03 * w, barrier_y - 8), "PERIMETER FENCE")

        # Foreground Rover Capture Dike / Grassy Bund at bottom
        dike_rect = QRectF(0.025 * w, 0.71 * h, 0.95 * w, 0.22 * h)
        p.setPen(QPen(QColor("#26382d"), 1.2))
        p.setBrush(QColor("#152019"))
        p.drawRoundedRect(dike_rect, 10, 10)
        p.setPen(QColor(BLUE))
        p.drawText(
            dike_rect.adjusted(12, 6, 0, 0),
            Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft,
            "CAPTURE CORRIDOR",
        )

        # Dashed rover travel line along the dike
        p.setPen(QPen(QColor("#385444"), 1.6, Qt.PenStyle.DashLine))
        p.drawLine(QPointF(w * 0.05, h * 0.79), QPointF(w * 0.95, h * 0.79))

        # Live Autonomous Train Drive GPS Path (saved every 5s cycle in auto_drive_path.json & SQLite)
        if self._auto_path:
            n_pts = len(self._auto_path)
            pts: list[QPointF] = []
            for idx, wp in enumerate(self._auto_path):
                frac = (idx / max(1, n_pts - 1)) if n_pts > 1 else 0.5
                px = w * (0.08 + 0.84 * frac)
                py = h * 0.865
                pts.append(QPointF(px, py))
            if len(pts) >= 2:
                p.setPen(QPen(QColor(BLUE), 2.2))
                for i in range(len(pts) - 1):
                    p.drawLine(pts[i], pts[i + 1])
            for idx, pt in enumerate(pts):
                is_latest = idx == len(pts) - 1
                p.setPen(QPen(QColor("#0e1511"), 1.5))
                p.setBrush(QColor("#79c0ff" if is_latest else BLUE))
                r_wp = 5.5 if is_latest else 3.8
                p.drawEllipse(pt, r_wp, r_wp)
            last_wp = self._auto_path[-1]
            p.setPen(QColor("#79c0ff"))
            p.drawText(
                dike_rect.adjusted(12, 6, -14, 0),
                Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignRight,
                f"AUTO TRAIN GPS PATH · {n_pts} cycles · Latest: {last_wp.get('gps', '')} ({last_wp.get('direction', '').upper()})",
            )

        for pin in self.PINS:
            c = self._pos(pin)
            color = QColor(RED_PIN if pin["bad"] else GOOD)
            # Camera FOV cone pointing from dike across the barrier into the rice plot
            fov_pen = QPen(QColor(color.red(), color.green(), color.blue(), 110), 1.3, Qt.PenStyle.DotLine)
            p.setPen(fov_pen)
            p.drawLine(c, QPointF(c.x() - 22, 0.52 * h))
            p.drawLine(c, QPointF(c.x() + 22, 0.52 * h))

            # Soft outer glow ring
            glow = QColor(color)
            glow.setAlpha(55)
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(glow)
            p.drawEllipse(c, pin["r"] + 5, pin["r"] + 5)

            p.setPen(QPen(QColor("#0e1511"), 2))
            p.setBrush(color)
            p.drawEllipse(c, pin["r"], pin["r"])

        sel = self._selected_pin()
        if sel:
            rect = self._popup_rect(sel)
            p.setPen(QPen(QColor("#2d4236"), 1.2))
            p.setBrush(QColor(CARD))
            p.drawRoundedRect(rect, 11, 11)

            body = rect.adjusted(12, 8, -10, -6)
            f1 = p.font()
            f1.setPixelSize(11)
            f1.setBold(True)
            p.setFont(f1)
            p.setPen(QColor(TEXT))
            p.drawText(body, Qt.AlignmentFlag.AlignTop, f"{sel['id']} · {sel['when']}")

            f2 = p.font()
            f2.setPixelSize(10)
            f2.setBold(False)
            p.setFont(f2)
            p.setPen(QColor(MUTED))
            p.drawText(body.adjusted(0, 17, 0, 0), Qt.AlignmentFlag.AlignTop, sel["detail"])

            f2.setBold(True)
            p.setFont(f2)
            p.setPen(QColor(RED_PIN if sel["bad"] else GOOD))
            verdict = "Needs inspection" if sel["bad"] else "Healthy"
            p.drawText(
                body.adjusted(0, 33, 0, 0),
                Qt.AlignmentFlag.AlignTop,
                f"{verdict} · open record →",
            )

    def mouseMoveEvent(self, event):
        pos = event.position()
        pin = self._pin_at(pos)
        sel = self._selected_pin()
        over_popup = bool(sel and self._popup_rect(sel).contains(pos))
        if pin:
            QToolTip.showText(
                event.globalPosition().toPoint(),
                f"{pin['id']}  ± {pin['acc']:.1f} m NEO-6M GPS accuracy",
                self,
            )
        else:
            QToolTip.hideText()
        self.setCursor(
            Qt.CursorShape.PointingHandCursor
            if (pin or over_popup)
            else Qt.CursorShape.ArrowCursor
        )

    def mousePressEvent(self, event):
        pos = event.position()
        sel = self._selected_pin()
        if sel and self._popup_rect(sel).contains(pos):
            self.open_record.emit(sel["id"])
            return
        pin = self._pin_at(pos)
        if pin:
            self.select(pin["id"])


class FieldMapPage(QWidget):
    open_record = Signal(str)

    def __init__(self):
        super().__init__()
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(12)

        top = QHBoxLayout()
        top.setSpacing(8)
        top.addWidget(button("Session · PhilRice Plots  ▾", "filter"))
        top.addWidget(button("All Statuses  ▾", "filter"))
        top.addStretch()
        self.path_chip = Chip("Auto Path: 0 cycles (auto_drive_path.json)", "blue", "map")
        top.addWidget(self.path_chip)
        root.addLayout(top)

        self.map = FieldMap()
        self.map.open_record.connect(self.open_record)
        root.addWidget(self.map, 1)
        self.refresh_path()

        healthy, needs = self.map.counts()
        legend = QHBoxLayout()
        legend.setSpacing(18)
        legend.addWidget(lbl(f"<span style='color:{GOOD}'>●</span> Healthy ({healthy})", 12, MUTED, "600"))
        legend.addWidget(lbl(f"<span style='color:{RED_PIN}'>●</span> Needs inspection ({needs})", 12, MUTED, "600"))
        legend.addWidget(lbl(f"<span style='color:{BLUE}'>●</span> Auto Train GPS Path (every 5s → JSON + SQLite)", 12, MUTED, "600"))
        legend.addWidget(lbl("± GPS accuracy (m)", 12, MUTED))
        legend.addStretch()
        root.addLayout(legend)

    def refresh_path(self):
        n = self.map.refresh_auto_path()
        self.path_chip.set_text(f"Auto Path: {n} cycles (auto_drive_path.json)", "blue")

    def select(self, pin_id):
        self.map.select(pin_id)
