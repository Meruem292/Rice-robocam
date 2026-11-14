import sys
from PySide6.QtCore import QSize, Qt, Signal
from PySide6.QtGui import QIcon
from PySide6.QtWidgets import (
    QApplication, QButtonGroup, QFrame, QHBoxLayout, QLabel,
    QMainWindow, QPushButton, QStackedWidget, QVBoxLayout, QWidget
)
from common import (
    APP_QSS, CARD, GOOD, GREEN, LINE, MUTED, STANDARDS, TEXT,
    Chip, lbl, make_pixmap
)
from tab_control import ControlPage
from tab_field_map import FieldMapPage
from tab_records import RecordsPage
from tab_validation import ValidationPage
from tab_standards import StandardsPage


class Sidebar(QFrame):
    navigate = Signal(int)
    ITEMS = [
        ("Rover Control", "camera"),
        ("Field Map", "map"),
        ("Records", "list"),
        ("Validation", "chart"),
        ("Standards", "sliders"),
    ]

    def __init__(self, session="Session 07", user="PhilRice Muñoz"):
        super().__init__()
        self.setObjectName("sidebar")
        self.setFixedWidth(210)
        self.setStyleSheet(
            f"QFrame#sidebar {{ background: {CARD}; border: 1px solid {LINE}; border-radius: 16px; }}"
        )

        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 18, 14, 14)
        layout.setSpacing(4)

        brand_row = QHBoxLayout()
        brand_row.setContentsMargins(4, 0, 4, 0)
        brand_row.setSpacing(10)

        badge = QFrame()
        badge.setFixedSize(32, 32)
        badge.setStyleSheet(
            f"background: {GREEN}; border-radius: 8px;"
        )
        bl = QHBoxLayout(badge)
        bl.setContentsMargins(0, 0, 0, 0)
        leaf = QLabel()
        leaf.setPixmap(make_pixmap("leaf", "#ffffff", 18))
        leaf.setAlignment(Qt.AlignmentFlag.AlignCenter)
        bl.addWidget(leaf)
        brand_row.addWidget(badge)

        brand_row.addWidget(lbl("RiceTrack", 16, TEXT, "800"))
        brand_row.addStretch()
        layout.addLayout(brand_row)

        layout.addSpacing(14)

        self.group = QButtonGroup(self)
        self.group.setExclusive(True)
        for index, (name, icon) in enumerate(self.ITEMS):
            btn = QPushButton(f"  {name}")
            btn.setProperty("nav", "true")
            btn.setCheckable(True)
            btn.setCursor(Qt.CursorShape.PointingHandCursor)
            btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
            icon_obj = QIcon()
            icon_obj.addPixmap(make_pixmap(icon, MUTED, 18), QIcon.Mode.Normal, QIcon.State.Off)
            icon_obj.addPixmap(make_pixmap(icon, "#ffffff", 18), QIcon.Mode.Normal, QIcon.State.On)
            btn.setIcon(icon_obj)
            btn.setIconSize(QSize(18, 18))
            self.group.addButton(btn, index)
            layout.addWidget(btn)
        first_btn = self.group.button(0)
        if first_btn is not None:
            first_btn.setChecked(True)
        self.group.idClicked.connect(self.navigate)

        layout.addStretch()

        footer = QFrame()
        footer.setObjectName("session")
        footer.setStyleSheet(
            "QFrame#session { background: #0e1411; border: 1px solid #1e2923; border-radius: 10px; }"
        )
        fl = QVBoxLayout(footer)
        fl.setContentsMargins(12, 10, 12, 10)
        fl.setSpacing(2)
        fl.addWidget(lbl(session, 12, TEXT, "700"))
        fl.addWidget(lbl(user, 11, MUTED))
        layout.addWidget(footer)

    def set_current(self, index):
        btn = self.group.button(index)
        if btn is not None:
            btn.setChecked(True)


class MainWindow(QMainWindow):
    PAGE_META = [
        ("Rover Control", "Live camera feed, manual & autonomous 5s train drive, and sample capture"),
        ("Field Map", "Plot overview and capture locations"),
        ("Crop Records", "Captured samples and leaf color measurements"),
        ("Validation", "CV readings compared against manual field measurements"),
        ("Standards", "Thresholds for leaf color, plant height, tilt, and auto-drive interval"),
    ]

    def __init__(self):
        super().__init__()
        self.setWindowTitle("RiceTrack")
        self.resize(1280, 780)
        self.setMinimumSize(1020, 660)

        root = QWidget()
        root.setObjectName("root")
        self.setCentralWidget(root)
        layout = QHBoxLayout(root)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(16)

        self.sidebar = Sidebar()
        layout.addWidget(self.sidebar)

        right_col = QVBoxLayout()
        right_col.setContentsMargins(0, 0, 0, 0)
        right_col.setSpacing(12)

        header = QHBoxLayout()
        header.setContentsMargins(4, 2, 4, 4)
        title_box = QVBoxLayout()
        title_box.setSpacing(2)
        self.page_title = lbl(self.PAGE_META[0][0], 20, TEXT, "800")
        self.page_subtitle = lbl(self.PAGE_META[0][1], 12, MUTED)
        title_box.addWidget(self.page_title)
        title_box.addWidget(self.page_subtitle)
        header.addLayout(title_box, 1)
        right_col.addLayout(header)

        self.stack = QStackedWidget()
        right_col.addWidget(self.stack, 1)
        layout.addLayout(right_col, 1)

        self.control_page = ControlPage()
        self.map_page = FieldMapPage()
        self.records_page = RecordsPage()
        self.control_page.record_saved.connect(self._record_saved)
        self.control_page.auto_path_updated.connect(self.map_page.refresh_path)
        self.validation_page = ValidationPage()
        self.standards_page = StandardsPage()

        for page in (
            self.control_page,
            self.map_page,
            self.records_page,
            self.validation_page,
            self.standards_page,
        ):
            self.stack.addWidget(page)

        self.sidebar.navigate.connect(self._navigate)
        self.map_page.open_record.connect(self._open_record)
        self.standards_page.saved.connect(self._standards_saved)

    def _record_saved(self, _record_id):
        self.records_page.refresh()
        self.validation_page.refresh()

    def _navigate(self, index):
        self.stack.setCurrentIndex(index)
        if 0 <= index < len(self.PAGE_META):
            self.page_title.setText(self.PAGE_META[index][0])
            self.page_subtitle.setText(self.PAGE_META[index][1])
        if self.stack.widget(index) is self.records_page:
            self.records_page.refresh()
        elif self.stack.widget(index) is self.validation_page:
            self.validation_page.refresh()

    def _open_record(self, record_id):
        self.records_page.select(record_id)
        self.stack.setCurrentWidget(self.records_page)
        self.sidebar.set_current(2)
        self.page_title.setText(self.PAGE_META[2][0])
        self.page_subtitle.setText(self.PAGE_META[2][1])

    def _standards_saved(self):
        self.control_page.set_tilt_limit(STANDARDS["tilt"])
        self.control_page.set_auto_drive_interval(STANDARDS["auto_drive_s"])
        self.records_page.refresh()
        self.validation_page.refresh()


def main():
    app = QApplication(sys.argv)
    app.setStyleSheet(APP_QSS)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
