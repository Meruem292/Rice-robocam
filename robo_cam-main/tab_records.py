from datetime import datetime

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QColor, QPixmap
from PySide6.QtWidgets import (
    QComboBox, QFrame, QHBoxLayout, QLabel, QScrollArea, QSizePolicy, QVBoxLayout, QWidget
)

import database
from common import CARD, GOOD, MUTED, RED_TEXT, TEXT, button, card, kv_row, lbl, make_pixmap, pill, set_prop

COLUMN_WEIGHTS: list[int] = [10, 15, 10, 12, 10, 13]
COLUMN_TITLES = ["ID", "CAPTURED", "PLOT", "AVG HEIGHT", "MEDIAN HUE", "STATUS"]


def _format_date(iso_str):
    try:
        return datetime.fromisoformat(iso_str).strftime("%b %d, %H:%M")
    except (TypeError, ValueError):
        return iso_str or "--"


def _format_date_full(iso_str):
    try:
        return datetime.fromisoformat(iso_str).strftime("%B %d, %Y · %H:%M")
    except (TypeError, ValueError):
        return iso_str or "--"


def _status_pill(status):
    if status == "Healthy":
        return ("Healthy", "green")
    if status == "Test - Healthy":
        return ("Test · Healthy", "blue")
    if status.startswith("Test -"):
        return ("Test", "orange")
    return ("Inspect", "red")


def _swatch_color(avg_hue):
    return QColor.fromHsv(int(avg_hue * 2) % 360, 150, 210).name()


def _decode_image(blob):
    if not blob:
        return None
    pixmap = QPixmap()
    if pixmap.loadFromData(blob) and not pixmap.isNull():
        return pixmap
    return None


class ClickableFrame(QFrame):
    clicked = Signal()

    def mousePressEvent(self, event):
        self.clicked.emit()
        super().mousePressEvent(event)


class RecordRow(QFrame):
    """One record: a clickable summary row plus a collapsible detail panel."""

    def __init__(self, record):
        super().__init__()
        self.record = record
        self.expanded = False

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(4)

        self.summary = ClickableFrame()
        set_prop(self.summary, "row", "true")
        self.summary.setCursor(Qt.CursorShape.PointingHandCursor)
        self.summary.clicked.connect(self.toggle)
        sl = QHBoxLayout(self.summary)
        sl.setContentsMargins(14, 12, 14, 12)
        sl.setSpacing(8)

        pill_text, pill_kind = _status_pill(record["status"])
        cells = [
            lbl(record["id"], 13, TEXT, "700"),
            lbl(_format_date(record["captured_at"]), 13, MUTED, "600"),
            lbl(record["plot"], 13, TEXT, "600"),
            lbl(f"{record['avg_height']:.1f} cm", 13, TEXT),
            lbl(f"{record['avg_hue']:.0f}", 13, TEXT, "700"),
            pill(pill_text, pill_kind),
        ]
        for cell, weight in zip(cells, COLUMN_WEIGHTS):
            cell.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
            sl.addWidget(cell, weight)
        outer.addWidget(self.summary)

        self.detail = self._build_detail(record)
        self.detail.setVisible(False)
        outer.addWidget(self.detail)

    def toggle(self):
        self.expanded = not self.expanded
        self.detail.setVisible(self.expanded)
        set_prop(self.summary, "selected", "true" if self.expanded else "false")

    def _build_detail(self, record):
        frame = QFrame()
        set_prop(frame, "inset", "true")
        layout = QHBoxLayout(frame)
        layout.setContentsMargins(18, 14, 18, 14)
        layout.setSpacing(20)

        image_label = QLabel()
        image_label.setFixedSize(124, 92)
        image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        image_label.setStyleSheet("background:#070a08; border: 1px solid #233029; border-radius:10px;")
        image_label.setScaledContents(True)
        pixmap = _decode_image(record.get("image"))
        if pixmap is not None:
            image_label.setPixmap(pixmap)
        else:
            image_label.setText("No image")
            image_label.setStyleSheet(
                f"background:#070a08; border: 1px solid #233029; border-radius:10px; color:{MUTED}; font-size:11px;"
            )
        layout.addWidget(image_label, 0, Qt.AlignmentFlag.AlignTop)

        col1 = QVBoxLayout()
        col1.setSpacing(6)
        col1.addLayout(kv_row("Captured", lbl(_format_date_full(record["captured_at"]), 13, TEXT, "700"), MUTED))
        col1.addLayout(kv_row("Plot", lbl(record["plot"], 13, TEXT, "700"), MUTED))
        col1.addLayout(kv_row("Variant", lbl(record["variant"], 13, TEXT, "700"), MUTED))
        col1.addLayout(kv_row("GPS", lbl(record["gps"] or "No fix", 13, GOOD if record["gps"] else MUTED, "700"), MUTED))
        col1.addStretch()
        layout.addLayout(col1, 1)

        col2 = QVBoxLayout()
        col2.setSpacing(6)

        hue_row = QHBoxLayout()
        hue_row.setSpacing(6)
        swatch = QFrame()
        swatch.setFixedSize(14, 14)
        swatch.setStyleSheet(f"background:{_swatch_color(record['avg_hue'])}; border-radius:4px;")
        hue_row.addWidget(swatch)
        hue_row.addWidget(lbl(f"{record['avg_hue']:.1f} · {record['leaf_report']}", 13, TEXT, "700"))
        hue_row.addStretch()
        col2.addLayout(kv_row("Median hue", self._wrap(hue_row), MUTED))

        lab_l = record.get("lab_l")
        lab_a = record.get("lab_a")
        lab_b = record.get("lab_b")
        lab_text = (
            f"{lab_l:.1f} / {lab_a:.1f} / {lab_b:.1f}"
            if lab_l is not None and lab_a is not None and lab_b is not None
            else "--"
        )
        col2.addLayout(kv_row("CIELAB (L*/a*/b*)", lbl(lab_text, 13, TEXT, "700"), MUTED))

        col2.addLayout(kv_row(
            "Avg height",
            lbl(f"{record['avg_height']:.1f} cm · {record['height_report']}", 13, TEXT, "700"), MUTED,
        ))
        status_color = RED_TEXT if "Inspection" in record["status"] else GOOD
        col2.addLayout(kv_row("Status", lbl(record["status"], 13, status_color, "700"), MUTED))
        col2.addStretch()
        layout.addLayout(col2, 1)

        return frame

    @staticmethod
    def _wrap(layout):
        holder = QWidget()
        holder.setLayout(layout)
        return holder


class RecordsPage(QWidget):
    def __init__(self):
        super().__init__()
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(12)

        top = QHBoxLayout()
        top.setSpacing(8)
        self.month_filter = self._make_filter_combo()
        self.plot_filter = self._make_filter_combo()
        self.variant_filter = self._make_filter_combo()
        self.status_filter = self._make_filter_combo()
        for combo in (self.month_filter, self.plot_filter, self.variant_filter, self.status_filter):
            combo.currentIndexChanged.connect(self.refresh)
            top.addWidget(combo)
        top.addStretch()
        top.addWidget(button(" Field Report", "dark", "file"))
        root.addLayout(top)

        panel, pl = card(margins=(16, 12, 16, 14), spacing=10)

        header = QFrame()
        set_prop(header, "row", "true")
        header.setFixedHeight(36)
        hl = QHBoxLayout(header)
        hl.setContentsMargins(14, 0, 14, 0)
        hl.setSpacing(8)
        for title, weight in zip(COLUMN_TITLES, COLUMN_WEIGHTS):
            label = lbl(title, 11, MUTED, "700")
            label.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
            hl.addWidget(label, weight)
        pl.addWidget(header)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll_body = QWidget()
        scroll_body.setObjectName("scrollbody")
        scroll_body.setStyleSheet(f"QWidget#scrollbody {{ background: {CARD}; }}")
        self.list_container = QVBoxLayout(scroll_body)
        self.list_container.setContentsMargins(0, 0, 0, 0)
        self.list_container.setSpacing(2)
        self.list_container.addStretch()
        scroll.setWidget(scroll_body)
        pl.addWidget(scroll, 1)

        self.empty_label = lbl("No records yet. Capture or evaluate a photo from Rover Control to add one.", 13, MUTED)
        self.empty_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        pl.addWidget(self.empty_label)

        note = QHBoxLayout()
        note.setSpacing(6)
        note_icon = QLabel()
        note_icon.setPixmap(make_pixmap("info", MUTED, 14))
        note.addWidget(note_icon)
        note.addWidget(lbl("Preliminary assessment for decision support. Confirm with an agricultural specialist.",
                           12, MUTED), 1)
        pl.addLayout(note)

        root.addWidget(panel, 1)

        self._rows = []
        self.refresh()

    def _make_filter_combo(self):
        combo = QComboBox()
        combo.setMinimumWidth(130)
        return combo

    def _populate_combo(self, combo, values, labels=None):
        previous = combo.currentData() if combo.count() else "All"
        combo.blockSignals(True)
        combo.clear()
        combo.addItem("All", "All")
        for i, value in enumerate(values):
            text = labels[i] if labels else str(value)
            combo.addItem(text, value)
        index = combo.findData(previous)
        combo.setCurrentIndex(index if index >= 0 else 0)
        combo.blockSignals(False)

    def refresh(self):
        months = database.distinct_months()
        month_labels = [
            datetime.strptime(m, "%Y-%m").strftime("%B %Y")
            for m in months if m
        ]
        self._populate_combo(self.month_filter, months, month_labels)
        self._populate_combo(self.plot_filter, database.distinct("plot"))
        self._populate_combo(self.variant_filter, database.distinct("variant"))
        self._populate_combo(self.status_filter, database.distinct("status"))

        try:
            records = database.fetch_records(
                month=self.month_filter.currentData(),
                plot=self.plot_filter.currentData(),
                variant=self.variant_filter.currentData(),
                status=self.status_filter.currentData(),
            )
        except Exception as exc:
            self._render_rows([])
            self.empty_label.setText(f"Unable to load records: {exc}")
            self.empty_label.setVisible(True)
            return

        self.empty_label.setText(
            "No records yet. Capture or evaluate a photo from Rover Control to add one."
        )
        self._render_rows(records)

    def select(self, record_id):
        self.refresh()
        for row in self._rows:
            if row.record["id"] == record_id and not row.expanded:
                row.toggle()
                return

    def _render_rows(self, records):
        for row in self._rows:
            self.list_container.removeWidget(row)
            row.deleteLater()
        self._rows = []
        for index, record in enumerate(records):
            row = RecordRow(record)
            self.list_container.insertWidget(index, row)
            self._rows.append(row)
        self.empty_label.setVisible(len(records) == 0)
