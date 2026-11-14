from PySide6.QtCore import QPointF, QRectF, QSize, Qt, Signal
from PySide6.QtGui import QColor, QIcon, QPainter, QPainterPath, QPen, QPixmap, QPolygonF
from PySide6.QtWidgets import (
    QFrame, QHBoxLayout, QLabel, QPushButton, QSizePolicy, QVBoxLayout, QWidget
)

# Modern Dark Botanical / Agritech Palette
BG = "#0b0f0d"
CARD = "#131916"
CARD_ALT = "#18201c"
LINE = "#232e28"
TEXT = "#f3f6f4"
MUTED = "#8fa196"
GREEN = "#2ea043"
GREEN_DARK = "#196c2e"
GOOD = "#3fb950"

RED_BG, RED_TEXT, RED_LINE, RED_PIN = "#2d1215", "#ff7b72", "#5c2126", "#f85149"
WARN_BG, WARN_TEXT, WARN_LINE = "#2e210b", "#e3b341", "#5c4214"
BLUE_BG, BLUE_TEXT, BLUE = "#0d233a", "#79c0ff", "#388bfd"

CHIP_COLORS = {
    "gray":   ("#18201c", MUTED, "#26332c"),
    "green":  ("#0f2918", GOOD, "#1f5730"),
    "orange": (WARN_BG, WARN_TEXT, WARN_LINE),
    "red":    (RED_BG, RED_TEXT, RED_LINE),
    "blue":   (BLUE_BG, BLUE_TEXT, "#1f4772"),
}

# Calibrated to IRRI 4-panel LCC standards for standard inbreds (NSIC Rc 222 / Tubigan 18)
STANDARDS = {
    "variety": "NSIC Rc 222 (Tubigan 18)",
    "hue": (48, 75),  # Healthy/Optimal greenness band (LCC Panels 4 & 5)
    "height": (98, 101),
    "pole": 1.2,
    "gps_n": 5,
    "tilt": 15,
    "auto_drive_s": 5,  # Autonomous Train Drive interval (seconds)
    "sec_per_meter": 3.0,  # Speed calibration: seconds per meter in instructed roving
}

APP_QSS = f"""
* {{
    font-family: "Inter", "Segoe UI", "SF Pro Display", "Helvetica Neue", Arial, sans-serif;
    outline: none;
}}
QWidget#root {{ background: {BG}; }}
QStackedWidget {{ background: transparent; }}
QLabel {{ background: transparent; color: {TEXT}; font-size: 13px; }}

QFrame[card="true"] {{
    background: {CARD};
    border: 1px solid {LINE};
    border-radius: 16px;
}}
QFrame[inset="true"] {{
    background: #0e1411;
    border: 1px solid #1c2621;
    border-radius: 12px;
}}

QPushButton[nav="true"] {{
    text-align: left;
    padding: 10px 14px;
    color: #c9d6ce;
    background: transparent;
    border: 1px solid transparent;
    border-radius: 12px;
    font-size: 13px;
    font-weight: 600;
}}
QPushButton[nav="true"]:hover {{
    background: rgba(46, 160, 67, 0.12);
    color: #ffffff;
    border: 1px solid rgba(46, 160, 67, 0.25);
}}
QPushButton[nav="true"]:checked {{
    background: {GREEN};
    color: #ffffff;
    border: 1px solid #3fb950;
}}

QPushButton[btn="dark"] {{
    background: #171f1b;
    color: {TEXT};
    border: 1px solid {LINE};
    border-radius: 10px;
    padding: 6px 14px;
    min-height: 22px;
    font-size: 13px;
    font-weight: 600;
}}
QPushButton[btn="dark"]:hover {{
    background: #1f2924;
    border: 1px solid #34463c;
}}
QPushButton[btn="dark"]:pressed {{
    background: #121815;
}}

QPushButton[btn="primary"] {{
    background: {GREEN};
    color: #ffffff;
    border: 1px solid #3fb950;
    border-radius: 10px;
    padding: 6px 14px;
    min-height: 22px;
    font-size: 13px;
    font-weight: 700;
}}
QPushButton[btn="primary"]:hover {{
    background: #36b54d;
    border: 1px solid #56d364;
}}
QPushButton[btn="primary"]:pressed {{
    background: {GREEN_DARK};
}}
QPushButton[btn="primary"]:disabled {{
    background: #16241a;
    color: #58695e;
    border: 1px solid #1f3325;
}}

QPushButton[btn="pad"] {{
    background: #161e1a;
    color: {TEXT};
    border: 1px solid #28362e;
    border-radius: 14px;
    font-size: 16px;
    font-weight: 700;
}}
QPushButton[btn="pad"]:hover {{
    background: #1f2b25;
    border: 1px solid {GREEN};
    color: #ffffff;
}}
QPushButton[btn="pad"]:pressed {{
    background: {GREEN};
    border: 1px solid #56d364;
    color: #ffffff;
}}

QPushButton[btn="seg"] {{
    background: transparent;
    color: {MUTED};
    border: none;
    border-radius: 12px;
    padding: 7px 20px;
    font-size: 13px;
    font-weight: 600;
}}
QPushButton[btn="seg"]:hover {{
    color: {TEXT};
}}
QPushButton[btn="seg"]:checked {{
    background: {GREEN};
    color: #ffffff;
}}

QPushButton[btn="filter"] {{
    background: {CARD};
    color: {TEXT};
    border: 1px solid {LINE};
    border-radius: 12px;
    padding: 7px 14px;
    font-size: 12px;
    font-weight: 600;
}}
QPushButton[btn="filter"]:hover {{
    background: {CARD_ALT};
    border: 1px solid #324239;
}}
QPushButton[btn="filter-red"] {{
    background: {RED_BG};
    color: {RED_TEXT};
    border: 1px solid {RED_LINE};
    border-radius: 12px;
    padding: 7px 14px;
    font-size: 12px;
    font-weight: 600;
}}

QFrame[row="true"] {{
    background: {CARD};
    border: none;
    border-bottom: 1px solid #1e2923;
}}
QFrame[row="true"]:hover {{
    background: #19231e;
}}
QFrame[row="true"][selected="true"] {{
    background: #1a2720;
    border-radius: 10px;
    border: 1px solid #2d4537;
}}

QProgressBar {{
    background: #18211d;
    border: 1px solid #222e28;
    border-radius: 4px;
    min-height: 6px;
    max-height: 6px;
}}
QProgressBar::chunk {{
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 {GREEN}, stop:1 {BLUE});
    border-radius: 3px;
}}

QSpinBox, QDoubleSpinBox, QLineEdit {{
    background: #0e1411;
    border: 1px solid #28362e;
    border-radius: 8px;
    color: {TEXT};
    padding: 4px 8px;
    font-size: 13px;
    font-weight: 600;
    selection-background-color: {GREEN};
    selection-color: #ffffff;
}}
QSpinBox:focus, QDoubleSpinBox:focus, QLineEdit:focus {{
    border: 1px solid {GREEN};
}}
QSpinBox::up-button, QSpinBox::down-button,
QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {{
    width: 0px;
    height: 0px;
    border: none;
    background: transparent;
}}

QComboBox {{
    background: {CARD};
    border: 1px solid {LINE};
    border-radius: 11px;
    padding: 7px 14px;
    color: {TEXT};
    font-size: 12px;
    font-weight: 600;
}}
QComboBox:hover {{
    border: 1px solid #34463c;
    background: {CARD_ALT};
}}
QComboBox::drop-down {{
    border: none;
    width: 20px;
}}
QComboBox QAbstractItemView {{
    background: {CARD};
    color: {TEXT};
    border: 1px solid {LINE};
    selection-background-color: {GREEN};
    selection-color: #ffffff;
    padding: 4px;
}}

QScrollArea, QScrollArea > QWidget, QScrollArea > QWidget > QWidget {{
    background: {CARD};
    border: none;
}}
QScrollBar:vertical {{
    background: transparent;
    width: 8px;
    margin: 0px;
}}
QScrollBar::handle:vertical {{
    background: #28362e;
    border-radius: 4px;
    min-height: 24px;
}}
QScrollBar::handle:vertical:hover {{
    background: #384d41;
}}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
    height: 0px;
}}

QDialog {{
    background: {CARD};
    color: {TEXT};
}}
QToolTip {{
    background: #18221d;
    color: {TEXT};
    border: 1px solid {GREEN};
    border-radius: 6px;
    padding: 5px 8px;
    font-size: 12px;
}}
"""


def set_prop(widget, name, value):
    widget.setProperty(name, value)
    st = widget.style()
    if st is not None:
        st.unpolish(widget)
        st.polish(widget)


def lbl(text="", size=13, color=TEXT, weight=None, align=None, wrap=False):
    label = QLabel(text)
    css = f"font-size: {size}px; color: {color};"
    if weight:
        css += f" font-weight: {weight};"
    label.setStyleSheet(css)
    if align is not None:
        label.setAlignment(align)
    label.setWordWrap(wrap)
    return label


def card(margins=(18, 16, 18, 16), spacing=12):
    frame = QFrame()
    frame.setProperty("card", "true")
    layout = QVBoxLayout(frame)
    layout.setContentsMargins(*margins)
    layout.setSpacing(spacing)
    return frame, layout


def kv_row(name, value_widget, name_color=MUTED):
    row = QHBoxLayout()
    row.setSpacing(8)
    row.addWidget(lbl(name, 13, name_color))
    row.addStretch()
    row.addWidget(value_widget)
    return row


def button(text, kind="dark", icon=None, icon_color=TEXT, checkable=False):
    btn = QPushButton(text)
    btn.setProperty("btn", kind)
    btn.setCursor(Qt.CursorShape.PointingHandCursor)
    if icon:
        btn.setIcon(QIcon(make_pixmap(icon, icon_color, 16)))
        btn.setIconSize(QSize(16, 16))
    btn.setCheckable(checkable)
    return btn


def make_pixmap(kind, color, size=18):
    dpr = 2
    pm = QPixmap(size * dpr, size * dpr)
    pm.setDevicePixelRatio(dpr)
    pm.fill(Qt.GlobalColor.transparent)
    p = QPainter(pm)
    p.setRenderHint(QPainter.RenderHint.Antialiasing)
    p.scale(size / 18.0, size / 18.0)
    c = QColor(color)
    pen = QPen(c, 1.6)
    pen.setCapStyle(Qt.PenCapStyle.RoundCap)
    pen.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
    p.setPen(pen)
    p.setBrush(Qt.BrushStyle.NoBrush)

    def poly(*pts, closed=False):
        path = QPainterPath(QPointF(*pts[0]))
        for pt in pts[1:]:
            path.lineTo(QPointF(*pt))
        if closed:
            path.closeSubpath()
        p.drawPath(path)

    def line(x1, y1, x2, y2):
        p.drawLine(QPointF(x1, y1), QPointF(x2, y2))

    def filled():
        p.setBrush(c)
        p.setPen(Qt.PenStyle.NoPen)

    if kind == "leaf":
        path = QPainterPath(QPointF(3, 15))
        path.cubicTo(1.5, 5.5, 9, 2, 16, 2.5)
        path.cubicTo(16, 10, 10.5, 15.5, 3, 15)
        p.drawPath(path)
        line(3, 15, 10, 8)
    elif kind == "wheel":
        p.drawEllipse(QRectF(2, 2, 14, 14))
        p.drawEllipse(QRectF(7.5, 7.5, 3, 3))
        line(2.5, 9, 7.5, 9)
        line(10.5, 9, 15.5, 9)
        line(9, 10.5, 9, 15.5)
    elif kind == "map":
        poly((2, 4.5), (6.5, 3), (11.5, 5), (16, 3.5), (16, 13.5), (11.5, 15), (6.5, 13), (2, 14.5), closed=True)
        line(6.5, 3, 6.5, 13)
        line(11.5, 5, 11.5, 15)
    elif kind == "list":
        for y in (5, 9, 13):
            p.drawEllipse(QRectF(2.8, y - 0.7, 1.4, 1.4))
            line(7, y, 15.5, y)
    elif kind == "chart":
        poly((2.5, 2.5), (2.5, 15.5), (15.5, 15.5))
        poly((5.5, 12), (9, 8), (11.5, 10), (15, 5))
    elif kind == "sliders":
        for x, y in ((4, 6), (9, 12), (14, 8)):
            line(x, 3, x, 15)
            p.setBrush(c)
            p.drawEllipse(QPointF(x, y), 2, 2)
            p.setBrush(Qt.BrushStyle.NoBrush)
    elif kind == "battery":
        p.drawRoundedRect(QRectF(1.5, 5, 12, 8), 1.8, 1.8)
        filled()
        p.drawRect(QRectF(14, 7.5, 1.8, 3))
        p.drawRoundedRect(QRectF(3.3, 6.8, 5, 4.4), 0.8, 0.8)
    elif kind == "signal":
        filled()
        for x, top in ((2.5, 11), (6.3, 8), (10.1, 5), (13.9, 2)):
            p.drawRoundedRect(QRectF(x, top, 2.4, 15 - top), 0.8, 0.8)
    elif kind == "wifi":
        for r in (3, 6.5, 10):
            p.drawArc(QRectF(9 - r, 14 - r, 2 * r, 2 * r), 45 * 16, 90 * 16)
        filled()
        p.drawEllipse(QPointF(9, 14), 1, 1)
    elif kind == "target":
        p.drawEllipse(QRectF(3.5, 3.5, 11, 11))
        line(9, 0.8, 9, 3.5)
        line(9, 14.5, 9, 17.2)
        line(0.8, 9, 3.5, 9)
        line(14.5, 9, 17.2, 9)
        filled()
        p.drawEllipse(QPointF(9, 9), 1.5, 1.5)
    elif kind == "download":
        line(9, 2.5, 9, 11.5)
        poly((5, 8), (9, 12), (13, 8))
        line(3, 15.5, 15, 15.5)
    elif kind == "file":
        poly((4, 2), (10.5, 2), (14.5, 6), (14.5, 16), (4, 16), closed=True)
        poly((10.5, 2), (10.5, 6), (14.5, 6))
    elif kind == "info":
        p.drawEllipse(QRectF(2, 2, 14, 14))
        line(9, 8, 9, 12.5)
        filled()
        p.drawEllipse(QPointF(9, 5.6), 0.9, 0.9)
    elif kind == "hand":
        p.drawRoundedRect(QRectF(4.5, 8, 9, 7.5), 2, 2)
        for x in (6.75, 9, 11.25):
            line(x, 3.5, x, 8)
    elif kind == "pause":
        filled()
        p.drawRoundedRect(QRectF(4.5, 3.5, 3, 11), 1, 1)
        p.drawRoundedRect(QRectF(10.5, 3.5, 3, 11), 1, 1)
    elif kind == "play":
        filled()
        p.drawPolygon(QPolygonF([QPointF(5, 3.5), QPointF(14, 9), QPointF(5, 14.5)]))
    elif kind == "check":
        poly((4, 9.5), (7.5, 13), (14, 5.5))
    elif kind == "close":
        line(4.5, 4.5, 13.5, 13.5)
        line(13.5, 4.5, 4.5, 13.5)
    elif kind == "camera":
        p.drawRoundedRect(QRectF(2, 5, 14, 10), 2, 2)
        poly((6, 5), (7.2, 3), (10.8, 3), (12, 5))
        p.drawEllipse(QRectF(6.5, 7.5, 5, 5))
    elif kind == "cpu":
        p.drawRoundedRect(QRectF(4.5, 4.5, 9, 9), 1.8, 1.8)
        p.drawRect(QRectF(7, 7, 4, 4))
        for pos in (6.5, 9, 11.5):
            line(pos, 2, pos, 4.5)
            line(pos, 13.5, pos, 16)
            line(2, pos, 4.5, pos)
            line(13.5, pos, 16, pos)

    p.end()
    return pm


class Chip(QFrame):
    def __init__(self, text, kind="gray", icon=None):
        super().__init__()
        bg, fg, border = CHIP_COLORS[kind]
        self.setObjectName("chip")
        self.setStyleSheet(
            f"QFrame#chip {{ background: {bg}; border: 1px solid {border}; border-radius: 13px; }}"
        )
        self.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(11, 5, 11, 5)
        layout.setSpacing(6)
        self._icon_name = icon
        self._icon_label = None
        if icon:
            self._icon_label = QLabel()
            self._icon_label.setPixmap(make_pixmap(icon, fg, 14))
            self._icon_label.setFixedSize(14, 14)
            layout.addWidget(self._icon_label)
        self._text = lbl(text, 12, fg, "600")
        layout.addWidget(self._text)

    def set_kind(self, kind):
        bg, fg, border = CHIP_COLORS[kind]
        self.setStyleSheet(
            f"QFrame#chip {{ background: {bg}; border: 1px solid {border}; border-radius: 13px; }}"
        )
        self._text.setStyleSheet("font-size: 12px; font-weight: 600; color: " + fg + ";")
        if self._icon_label and self._icon_name:
            self._icon_label.setPixmap(make_pixmap(self._icon_name, fg, 14))

    def set_text(self, text, kind=None):
        if kind is not None:
            self.set_kind(kind)
        self._text.setText(text)


def pill(text, kind):
    bg, fg, border = CHIP_COLORS[kind]
    label = QLabel(text)
    label.setStyleSheet(
        f"background: {bg}; color: {fg}; border: 1px solid {border}; border-radius: 11px; "
        f"padding: 3px 11px; font-size: 12px; font-weight: 600;"
    )
    holder = QWidget()
    lay = QHBoxLayout(holder)
    lay.setContentsMargins(0, 0, 0, 0)
    lay.addWidget(label)
    lay.addStretch()
    return holder


def banner(kind, icon=None):
    bg, line_c, fg = {
        "red": (RED_BG, RED_LINE, RED_TEXT),
        "orange": (WARN_BG, WARN_LINE, WARN_TEXT),
    }[kind]
    frame = QFrame()
    frame.setObjectName("banner")
    frame.setStyleSheet(
        f"QFrame#banner {{ background: {bg}; border: 1px solid {line_c}; border-radius: 12px; }}"
    )
    lay = QHBoxLayout(frame)
    lay.setContentsMargins(14, 10, 14, 10)
    lay.setSpacing(8)
    if icon:
        icon_label = QLabel()
        icon_label.setPixmap(make_pixmap(icon, fg, 16))
        lay.addWidget(icon_label)
    text = lbl("", 13, fg, "600")
    lay.addWidget(text, 1)
    return frame, text


class _Row(QFrame):
    clicked = Signal()

    def mousePressEvent(self, event):
        self.clicked.emit()
        super().mousePressEvent(event)


class DataTable(QWidget):
    row_selected = Signal(int)

    def __init__(self, columns, weights, selectable=False, row_height=46):
        super().__init__()
        self.weights = weights
        self.selectable = selectable
        self.row_height = row_height
        self._rows = []
        self._selected = -1

        self._layout = QVBoxLayout(self)
        self._layout.setContentsMargins(0, 0, 0, 0)
        self._layout.setSpacing(0)

        header = _Row()
        header.setProperty("row", "true")
        hl = QHBoxLayout(header)
        hl.setContentsMargins(14, 0, 14, 0)
        hl.setSpacing(8)
        header.setFixedHeight(38)
        for name, weight in zip(columns, weights):
            label = lbl(name.upper(), 11, MUTED, "700")
            label.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
            hl.addWidget(label, int(weight))
        self._layout.addWidget(header)

    @staticmethod
    def _cell(spec):
        if isinstance(spec, QWidget):
            spec.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
            return spec
        if isinstance(spec, tuple):
            text, color, bold = (list(spec) + [None, None])[:3]
        else:
            text, color, bold = spec, None, False
        label = lbl(str(text), 13, color or TEXT, "600" if bold else None)
        label.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
        return label

    def set_rows(self, rows):
        for row in self._rows:
            self._layout.removeWidget(row)
            row.deleteLater()
        self._rows = []
        self._selected = -1

        for index, cells in enumerate(rows):
            row = _Row()
            row.setProperty("row", "true")
            row.setFixedHeight(self.row_height)
            rl = QHBoxLayout(row)
            rl.setContentsMargins(14, 0, 14, 0)
            rl.setSpacing(8)
            for spec, weight in zip(cells, self.weights):
                rl.addWidget(self._cell(spec), int(weight))
            if self.selectable:
                row.setCursor(Qt.CursorShape.PointingHandCursor)
                row.clicked.connect(lambda i=index: self.select(i))
            self._layout.addWidget(row)
            self._rows.append(row)

    def select(self, index):
        if not self.selectable or not (0 <= index < len(self._rows)):
            return
        if 0 <= self._selected < len(self._rows):
            set_prop(self._rows[self._selected], "selected", "false")
        self._selected = index
        set_prop(self._rows[index], "selected", "true")
        self.row_selected.emit(index)
