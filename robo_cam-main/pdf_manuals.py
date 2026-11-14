"""
pdf_manuals.py
--------------
Comprehensive, precise, and concise PDF generator for RiceTrack:
  1. Complete System & Hardware Manual (ESP32-WROOM-32, USB Camera,
     NEO-6M GPS, MPU-6050 IMU Gyroscope, 2x BTS7960 Drivers, 2x MY1016Z2 Motors,
     24V 12Ah Ebike Battery + 24V->5V Buck Converter).
  2. IRRI 4-Panel LCC Calibration & Mathematical Vision Specification
     (HSV + ExG Segmentation, CIELAB L*a*b*, Delta-E, Health Index, MAE/RMSE).
  3. Step-by-Step Overall Operating Instructions (Web App & Desktop App,
     5-Tab Workflow, 5s Autonomous Train Drive, Safety & Troubleshooting).
  4. Live Calibration Standards & Field Validation Summary Snapshot.
"""

from datetime import datetime
import io
from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.platypus import (
    HRFlowable,
    KeepTogether,
    PageBreak,
    Paragraph,
    SimpleDocTemplate,
    Spacer,
    Table,
    TableStyle,
)

from common import STANDARDS
import database
from vision import IRRI_LCC_REFERENCES

BASE_DIR = Path(__file__).resolve().parent
DEFAULT_PDF_PATH = BASE_DIR / "RiceTrack_Complete_Manual_Calibration_and_Instructions.pdf"

# Brand Palette for Print-Ready PDF
DARK_GREEN = colors.HexColor("#14532d")
EMERALD = colors.HexColor("#2ea043")
LIGHT_GREEN_BG = colors.HexColor("#ecfdf3")
SLATE_DARK = colors.HexColor("#0f172a")
SLATE_MUTED = colors.HexColor("#475569")
BORDER_GRAY = colors.HexColor("#cbd5e1")
ROW_ALT = colors.HexColor("#f8fafc")
WARN_RED = colors.HexColor("#b91c1c")


def _draw_header_footer(canvas, doc):
    """Render running header and page number footer on every page."""
    canvas.saveState()
    w, h = A4

    # Top header bar
    canvas.setStrokeColor(EMERALD)
    canvas.setLineWidth(1.2)
    canvas.line(15 * mm, h - 14 * mm, w - 15 * mm, h - 14 * mm)

    canvas.setFont("Helvetica-Bold", 8.5)
    canvas.setFillColor(DARK_GREEN)
    canvas.drawString(15 * mm, h - 11.5 * mm, "RICETRACK — SYSTEM MANUAL, IRRI LCC CALIBRATION & OPERATING INSTRUCTIONS")

    canvas.setFont("Helvetica", 8)
    canvas.setFillColor(SLATE_MUTED)
    canvas.drawRightString(w - 15 * mm, h - 11.5 * mm, f"PhilRice Muñoz · {STANDARDS['variety']}")

    # Bottom footer bar
    canvas.setStrokeColor(BORDER_GRAY)
    canvas.setLineWidth(0.6)
    canvas.line(15 * mm, 13 * mm, w - 15 * mm, 13 * mm)

    canvas.setFont("Helvetica", 8)
    canvas.setFillColor(SLATE_MUTED)
    canvas.drawString(
        15 * mm,
        8.5 * mm,
        f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M')} · Validated Reference: IRRI 4-Panel LCC (Sir Jasper Talada)",
    )
    canvas.setFont("Helvetica-Bold", 8)
    canvas.drawRightString(w - 15 * mm, 8.5 * mm, f"Page {doc.page}")
    canvas.restoreState()


def _make_table(data, col_widths=None, header_bg=DARK_GREEN):
    t = Table(data, colWidths=col_widths, repeatRows=1)
    style_cmds = [
        ("BACKGROUND", (0, 0), (-1, 0), header_bg),
        ("TEXTCOLOR", (0, 0), (-1, 0), colors.white),
        ("FONTNAME", (0, 0), (-1, 0), "Helvetica-Bold"),
        ("FONTSIZE", (0, 0), (-1, 0), 8.5),
        ("BOTTOMPADDING", (0, 0), (-1, 0), 6),
        ("TOPPADDING", (0, 0), (-1, 0), 6),
        ("FONTNAME", (0, 1), (-1, -1), "Helvetica"),
        ("FONTSIZE", (0, 1), (-1, -1), 8),
        ("TEXTCOLOR", (0, 1), (-1, -1), SLATE_DARK),
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
        ("LEFTPADDING", (0, 0), (-1, -1), 6),
        ("RIGHTPADDING", (0, 0), (-1, -1), 6),
        ("TOPPADDING", (0, 1), (-1, -1), 4.5),
        ("BOTTOMPADDING", (0, 1), (-1, -1), 4.5),
        ("GRID", (0, 0), (-1, -1), 0.5, BORDER_GRAY),
    ]
    for row_idx in range(1, len(data)):
        if row_idx % 2 == 0:
            style_cmds.append(("BACKGROUND", (0, row_idx), (-1, row_idx), ROW_ALT))
    t.setStyle(TableStyle(style_cmds))
    return t


def build_manual_pdf_bytes() -> bytes:
    """Build the complete RiceTrack Manual, Calibration & Instructions PDF and return raw bytes."""
    buf = io.BytesIO()
    doc = SimpleDocTemplate(
        buf,
        pagesize=A4,
        leftMargin=15 * mm,
        rightMargin=15 * mm,
        topMargin=18 * mm,
        bottomMargin=18 * mm,
        title="RiceTrack Complete Manual, Calibration & Operating Instructions",
        author="RiceTrack System (PhilRice Muñoz)",
    )

    base_styles = getSampleStyleSheet()
    title_style = ParagraphStyle(
        "DocTitle",
        parent=base_styles["Title"],
        fontName="Helvetica-Bold",
        fontSize=18,
        leading=22,
        textColor=DARK_GREEN,
        alignment=0,
        spaceAfter=4,
    )
    subtitle_style = ParagraphStyle(
        "DocSubtitle",
        parent=base_styles["Normal"],
        fontName="Helvetica",
        fontSize=9.5,
        leading=13,
        textColor=SLATE_MUTED,
        spaceAfter=10,
    )
    h1_style = ParagraphStyle(
        "SectionH1",
        parent=base_styles["Heading1"],
        fontName="Helvetica-Bold",
        fontSize=12.5,
        leading=16,
        textColor=DARK_GREEN,
        spaceBefore=10,
        spaceAfter=5,
    )
    h2_style = ParagraphStyle(
        "SectionH2",
        parent=base_styles["Heading2"],
        fontName="Helvetica-Bold",
        fontSize=10,
        leading=13,
        textColor=SLATE_DARK,
        spaceBefore=7,
        spaceAfter=3,
    )
    body_style = ParagraphStyle(
        "BodyCompact",
        parent=base_styles["Normal"],
        fontName="Helvetica",
        fontSize=8.5,
        leading=12,
        textColor=SLATE_DARK,
        spaceAfter=4,
    )
    bullet_style = ParagraphStyle(
        "BulletCompact",
        parent=body_style,
        leftIndent=12,
        firstLineIndent=-8,
        spaceAfter=2.5,
    )
    cell_style = ParagraphStyle(
        "CellText",
        parent=body_style,
        fontSize=7.8,
        leading=10.5,
        spaceAfter=0,
    )
    cell_bold = ParagraphStyle(
        "CellBold",
        parent=cell_style,
        fontName="Helvetica-Bold",
    )

    story = []

    # =========================================================================
    # COVER / TITLE BANNER
    # =========================================================================
    story.append(Paragraph("RICETRACK: COMPREHENSIVE SYSTEM MANUAL, CALIBRATION &amp; OPERATING GUIDE", title_style))
    story.append(
        Paragraph(
            f"<b>Target Crop Variety:</b> {STANDARDS['variety']} &nbsp;|&nbsp; "
            f"<b>Controller:</b> ESP32-WROOM-32 &nbsp;|&nbsp; "
            f"<b>Vision Reference:</b> Physical IRRI 4-Panel Leaf Color Chart (Panels 2–5)",
            subtitle_style,
        )
    )
    story.append(HRFlowable(width="100%", thickness=1.5, color=EMERALD, spaceAfter=8))

    # =========================================================================
    # SECTION 1: OVERALL SYSTEM ARCHITECTURE & HARDWARE MANUAL
    # =========================================================================
    story.append(Paragraph("1. HARDWARE ARCHITECTURE &amp; ESP32-WROOM-32 WIRING MANUAL", h1_style))
    story.append(
        Paragraph(
            "RiceTrack is an autonomous and remotely operated agricultural field rover engineered on the "
            "<b>ESP32-WROOM-32</b>. It integrates a USB Camera on the Host PC for real-time canopy and "
            "IRRI Leaf Color Chart (LCC) vision analysis, an <b>MPU-6050 6-axis IMU Gyroscope/Accelerometer</b> "
            "for slope/tilt protection, a <b>NEO-6M GNSS/GPS receiver</b> for plot geotagging, and dual "
            "<b>BTS7960 43A H-Bridge Motor Drivers</b> powering dual <b>MY1016Z2 24V DC geared wheel motors</b>.",
            body_style,
        )
    )

    hw_rows = [
        ["Hardware Module", "Module Pin", "ESP32-WROOM-32 Connection", "ESP32 GPIO / Bus / Spec"],
        [
            Paragraph("<b>USB Camera (Host PC)</b>", cell_bold),
            Paragraph("USB Webcam", cell_style),
            Paragraph("USB Camera (Host PC)", cell_style),
            Paragraph("OpenCV USB (1280×720 @ 25 FPS)", cell_style),
        ],
        [
            Paragraph("<b>IMU Gyroscope MPU-6050</b>", cell_bold),
            Paragraph("SDA / SCL<br/>VCC / GND", cell_style),
            Paragraph("GPIO 21 (SDA) / GPIO 22 (SCL)<br/>3.3V Rail &amp; Common GND", cell_style),
            Paragraph("GPIO 21 (I2C SDA), GPIO 22 (I2C SCL)<br/>I2C addr 0x68 via ESP32 (alt 0x69), 20 Hz", cell_style),
        ],
        [
            Paragraph("<b>NEO-6M GPS Module</b>", cell_bold),
            Paragraph("TX / RX<br/>VCC / GND", cell_style),
            Paragraph("GPIO 16 (UART2 RX) / GPIO 17 (UART2 TX)<br/>3.3V Rail &amp; Common GND", cell_style),
            Paragraph("GPIO 16 (UART2 RX) &lt;- GPS TX, GPIO 17 (UART2 TX) -&gt; GPS RX<br/>Serial USB @ 9600 baud (NMEA GGA/RMC/GSV)", cell_style),
        ],
        [
            Paragraph("<b>Left BTS7960 Motor Driver</b>", cell_bold),
            Paragraph("R_En, L_En<br/>RPWM, LPWM<br/>B+/B-, M+/M-", cell_style),
            Paragraph("Standard GPIOs (Enable)<br/>Hardware PWM0 GPIOs<br/>24V Battery -&gt; Left MY1016Z2", cell_style),
            Paragraph("GPIO 25 (R_En), GPIO 26 (L_En)<br/>GPIO 32 (RPWM), GPIO 33 (LPWM) @ 1 kHz PWM<br/>Drive duty 65% (~0.6 m/s), Turn duty 50%", cell_style),
        ],
        [
            Paragraph("<b>Right BTS7960 Motor Driver</b>", cell_bold),
            Paragraph("R_En, L_En<br/>RPWM, LPWM<br/>B+/B-, M+/M-", cell_style),
            Paragraph("Standard GPIOs (Enable)<br/>Hardware PWM0 GPIOs<br/>24V Battery -&gt; Right MY1016Z2", cell_style),
            Paragraph("GPIO 27 (R_En), GPIO 14 (L_En)<br/>GPIO 18 (RPWM), GPIO 19 (LPWM) @ 1 kHz PWM<br/>Differential skid-steering (FWD/REV/LEFT/RIGHT)", cell_style),
        ],
        [
            Paragraph("<b>24V 12Ah Ebike Battery &amp; Buck Converter (24V-&gt;5V)</b>", cell_bold),
            Paragraph("B+ / B- (Switch)<br/>In+/In- -&gt; Out+/Out-", cell_style),
            Paragraph("24V Direct to BTS7960 B+/B-<br/>5V Out to ESP32 VIN", cell_style),
            Paragraph("24V High-Current Motor Bus + 5V Supply to ESP32 VIN", cell_style),
        ],
    ]
    story.append(_make_table(hw_rows, col_widths=[42 * mm, 28 * mm, 50 * mm, 60 * mm]))
    story.append(Spacer(1, 6))

    # =========================================================================
    # SECTION 2: MPU-6050 GYROSCOPE & SENSOR FUSION SPECIFICATION
    # =========================================================================
    story.append(Paragraph("2. IMU GYROSCOPE (MPU-6050) &amp; SAFETY CUTOFF CALIBRATION", h2_style))
    story.append(
        Paragraph(
            "• <b>Register Initialization:</b> Wake command writes <b>0x00</b> to power management register "
            "<b>REG_PWR_MGMT_1 (0x6B)</b> on I2C address <b>0x68</b>.<br/>"
            "• <b>14-Byte Atomic Burst Read (20 Hz / 50 ms):</b> Reads registers <b>0x3B..0x48</b> unpacking "
            "signed 16-bit big-endian Accelerometer (<b>ax, ay, az</b> @ 16,384 LSB/g) and Gyroscope "
            "(<b>gx, gy, gz</b> @ 131.0 LSB/deg/s).<br/>"
            "• <b>Complementary Sensor Fusion Filter:</b> Eliminates gyroscope integration drift while rejecting "
            "high-frequency chassis vibration:<br/>"
            "&nbsp;&nbsp;&nbsp;&nbsp;<i>Roll = 0.80 × (Roll_prev + gx × dt) + 0.20 × atan2(ay, az)</i><br/>"
            "&nbsp;&nbsp;&nbsp;&nbsp;<i>Pitch = 0.80 × (Pitch_prev + gy × dt) + 0.20 × atan2(-ax, sqrt(ay² + az²))</i><br/>"
            f"• <b>Automatic Tilt Safety Cutoff:</b> Whenever <b>max(|Roll|, |Pitch|) &gt; {STANDARDS['tilt']}°</b>, "
            "RiceTrack immediately halts Autonomous Train Drive, disables capture to prevent distorted canopy geometry, "
            "and triggers a red high-priority operator alert.",
            body_style,
        )
    )

    # =========================================================================
    # SECTION 3: IRRI 4-PANEL LCC CALIBRATION & VISION MANUAL
    # =========================================================================
    story.append(Paragraph("3. IRRI 4-PANEL LEAF COLOR CHART (LCC) CALIBRATION MANUAL", h1_style))
    story.append(
        Paragraph(
            "RiceTrack's computer vision pipeline (<b>vision.py</b>) is grounded directly in the physical "
            "<b>IRRI 4-Panel Leaf Color Chart (Panels 2, 3, 4, and 5)</b> validated at PhilRice Muñoz:",
            body_style,
        )
    )
    story.append(
        Paragraph(
            "1. <b>Dual-Mode Automatic Detection:</b><br/>"
            "&nbsp;&nbsp;• <b>Mode A — 4-Panel IRRI LCC Chart Auto-Calibration:</b> Detects 4 aligned vertical rectangular "
            "strips (aspect ratio 1.3–4.5, rectangular fill ≥ 0.72), erodes panel borders by 3% to strip plastic frame "
            "shadows, and extracts each panel's core median HSV and CIELAB L*a*b*.<br/>"
            "&nbsp;&nbsp;• <b>Mode B — Rice Canopy &amp; Single-Panel Segmentation:</b> Combines HSV green-band filtering "
            "(Hue 25–85, Sat ≥ 30, Val 12–250) with <b>Excess Green (ExG = 2G − R − B)</b> Otsu thresholding. Automatically "
            "detects horizontal plastic rat-barrier fences (60%–92% frame height) and masks out foreground dike grass.",
            body_style,
        )
    )

    lcc_rows = [
        ["IRRI LCC Panel", "Hue Band (OpenCV 0–179)", "Calibrated Core HSV (H, S, V)", "Calibrated CIELAB (L*, a*, b*)", "Agronomic Status"],
    ]
    hue_bands_map = {
        2: "30 – 41 (Yellowish)",
        3: f"42 – {STANDARDS['hue'][0] - 1} (Light Green)",
        4: f"{STANDARDS['hue'][0]} – {STANDARDS['hue'][1]} (Healthy Optimal)",
        5: f"&gt; {STANDARDS['hue'][1]} (Dark Green / Excess N)",
    }
    for ref in IRRI_LCC_REFERENCES:
        pnum = ref["panel_num"]
        hsv_str = f"({ref['hsv'][0]:.0f}, {ref['hsv'][1]:.0f}, {ref['hsv'][2]:.0f})"
        lab_str = f"({ref['lab'][0]:.0f}, {ref['lab'][1]:.0f}, {ref['lab'][2]:.0f})"
        lcc_rows.append(
            [
                Paragraph(f"<b>{ref['panel']}</b>", cell_bold),
                Paragraph(hue_bands_map.get(pnum, "--"), cell_style),
                Paragraph(hsv_str, cell_style),
                Paragraph(lab_str, cell_style),
                Paragraph(f"<b>{ref['status']}</b>", cell_bold),
            ]
        )
    story.append(_make_table(lcc_rows, col_widths=[38 * mm, 40 * mm, 34 * mm, 36 * mm, 32 * mm]))
    story.append(Spacer(1, 5))

    story.append(
        Paragraph(
            "• <b>Perceptual Color Distance (CIELAB Delta-E, ΔE*):</b> "
            "<i>ΔE* = sqrt((L* − L*_ref)² + (a* − a*_ref)² + (b* − b*_ref)²)</i>. "
            "When <b>ΔE* &lt; 18.0</b>, the sample locks directly onto the physical IRRI LCC panel centroid; "
            "for natural sunlit canopies, classification uses the configured HSV Hue bands.<br/>"
            "• <b>Thesis Health Index (HI) &amp; Error Metrics:</b><br/>"
            "&nbsp;&nbsp;<i>ColorScore = max(0, min(100, 100 − (|IdealHue − Hue| / 27.5 × 100)))</i><br/>"
            "&nbsp;&nbsp;<i>HeightScore = max(0, min(100, 100 − (|IdealHeight − Height| / IdealHeight × 100)))</i><br/>"
            "&nbsp;&nbsp;<i>HealthIndex (HI) = 0.70 × ColorScore + 0.30 × HeightScore</i> &nbsp;|&nbsp; "
            "<i>MAE = (1/N) Σ |CV − Ref|</i> &nbsp;|&nbsp; <i>RMSE = sqrt((1/N) Σ (CV − Ref)²)</i>",
            body_style,
        )
    )

    story.append(PageBreak())

    # =========================================================================
    # SECTION 4: OVERALL OPERATING INSTRUCTIONS (WEB APP & DESKTOP APP)
    # =========================================================================
    story.append(Paragraph("4. OVERALL OPERATING INSTRUCTIONS (WEB APP &amp; DESKTOP APP)", h1_style))

    story.append(Paragraph("4.1 Starting the System", h2_style))
    story.append(
        Paragraph(
            "• <b>Option A — Web Application (Recommended for Field &amp; Headless ESP32):</b><br/>"
            "&nbsp;&nbsp;1. Double-click <b>run_web.bat</b> or run <b>py web_app.py</b> (Windows) / <b>python3 web_app.py</b> (ESP32 Host PC).<br/>"
            "&nbsp;&nbsp;2. The server automatically binds to <b>http://localhost:8000</b> (and <b>http://&lt;Host-IP&gt;:8000</b> over Wi-Fi/LAN) and opens your browser.<br/>"
            "• <b>Option B — Native Desktop GUI (PySide6):</b><br/>"
            "&nbsp;&nbsp;1. Double-click <b>run.bat</b> or run <b>py main.py</b> (Windows) / <b>python3 main.py</b> (ESP32 Host PC).",
            body_style,
        )
    )

    story.append(Paragraph("4.2 Step-by-Step Workflow Across All 5 Tabs", h2_style))
    tabs_guide = [
        [
            "Tab Name",
            "Primary Controls & Shortcuts",
            "Standard Operating Procedure (SOP)",
        ],
        [
            Paragraph("<b>1. Rover Control</b>", cell_bold),
            Paragraph(
                "• <b>WASD / Arrow Keys / D-Pad</b>: Manual &amp; Auto steering<br/>"
                "• <b>Spacebar / ▢</b>: Emergency Stop<br/>"
                "• <b>T</b>: Toggle 5s Auto Train Drive<br/>"
                "• <b>C</b>: Capture &amp; Evaluate Frame",
                cell_style,
            ),
            Paragraph(
                "1. Select active camera (Camera 0/1/2) or choose a sample/upload photo.<br/>"
                "2. Keep <b>LCC Overlay</b> enabled to view live segmented canopy masks and Hue/L*a*b*.<br/>"
                "3. For autonomous corridor runs, click <b>Auto Train (5s)</b> and choose either "
                "<i>Every 5s Run</i> (continuous pulse + GPS path log) or <i>5s + Sample</i> (stops every 5s, "
                "captures &amp; evaluates canopy, and logs to SQLite + auto_drive_path.json).<br/>"
                "4. Click the camera viewport after capture to toggle between <b>Segmented Overlay</b> and <b>Original Photo</b>.",
                cell_style,
            ),
        ],
        [
            Paragraph("<b>2. Field Map</b>", cell_bold),
            Paragraph(
                "• Interactive PhilRice GIS Canvas<br/>"
                "• Inspection Pins &amp; FOV Cones<br/>"
                "• Live Auto Train GPS Path",
                cell_style,
            ),
            Paragraph(
                "1. Inspect side-by-side experimental plots (<b>PLOT A-01 to A-04 · NSIC Rc 222</b>), perimeter rat-barrier fence, and capture corridor.<br/>"
                "2. View the blue <b>Auto Train GPS Path</b> waypoints updated every 5 seconds.<br/>"
                "3. Click any green (Healthy) or red (Needs Inspection) pin to view its popup, then click the popup to jump directly to its full entry in <b>Records</b>.",
                cell_style,
            ),
        ],
        [
            Paragraph("<b>3. Records</b>", cell_bold),
            Paragraph(
                "• Filters: Month, Plot, Variant, Status<br/>"
                "• Expandable Detail Drawer<br/>"
                "• Field Report (CSV) Export",
                cell_style,
            ),
            Paragraph(
                "1. Filter stored SQLite inspections by Month, Plot, Variant, or Health Status.<br/>"
                "2. Click any row to expand its stored JPEG thumbnail, GPS coordinates, Median Hue swatch, CIELAB (L*/a*/b*), Avg Height, and Agronomic Verdict.<br/>"
                "3. Click <b>Field Report (CSV)</b> to export all records for spreadsheet analysis.",
                cell_style,
            ),
        ],
        [
            Paragraph("<b>4. Validation</b>", cell_bold),
            Paragraph(
                "• + Add Manual Reading<br/>"
                "• Live MAE &amp; RMSE Cards<br/>"
                "• Export CSV",
                cell_style,
            ),
            Paragraph(
                "1. Click <b>+ Add Manual Reading</b> and select an existing CV capture record.<br/>"
                "2. Enter the ground-truth manual reference height (cm) and reference LCC hue (0–179).<br/>"
                "3. The system computes Height Error, Hue Error, Color Score, Height Score, Health Index (HI), and updates overall <b>MAE</b> and <b>RMSE</b> automatically.",
                cell_style,
            ),
        ],
        [
            Paragraph("<b>5. Standards</b>", cell_bold),
            Paragraph(
                "• Healthy Hue Band Steppers<br/>"
                "• Healthy Height Steppers<br/>"
                "• GPS N, Max Tilt (°), Auto Cycle (s)<br/>"
                "• Reset Defaults / Save Standards",
                cell_style,
            ),
            Paragraph(
                "1. Adjust the <b>Healthy Hue Band</b> (default 48–75) and <b>Healthy Height Range</b> (default 98–101 cm) using the -/+ steppers.<br/>"
                "2. Set <b>Max tilt angle</b> (default 15°) and <b>Train Auto-Drive cycle</b> (default 5s).<br/>"
                "3. Clicking <b>Save Standards</b> immediately updates live camera classification AND recomputes all existing record/validation statuses in SQLite.",
                cell_style,
            ),
        ],
    ]
    story.append(_make_table(tabs_guide, col_widths=[30 * mm, 52 * mm, 98 * mm]))
    story.append(Spacer(1, 8))

    # =========================================================================
    # SECTION 5: LIVE CALIBRATION & VALIDATION SNAPSHOT FROM DATABASE
    # =========================================================================
    story.append(Paragraph("5. ACTIVE CALIBRATION STANDARDS &amp; VALIDATION SNAPSHOT", h1_style))

    metrics = database.validation_metrics()
    records = database.fetch_records()
    auto_path = database.fetch_auto_drive_path(limit=10)

    h_mae = f"{metrics['height_mae']:.2f} cm" if metrics["height_mae"] is not None else "N/A"
    h_rmse = f"{metrics['height_rmse']:.2f} cm" if metrics["height_rmse"] is not None else "N/A"
    hue_mae = f"{metrics['hue_mae']:.2f}" if metrics["hue_mae"] is not None else "N/A"
    hue_rmse = f"{metrics['hue_rmse']:.2f}" if metrics["hue_rmse"] is not None else "N/A"

    snap_rows = [
        ["Active Parameter / Metric", "Configured / Measured Value", "Operational Notes"],
        [
            Paragraph("<b>Target Rice Variety</b>", cell_bold),
            Paragraph(str(STANDARDS["variety"]), cell_style),
            Paragraph("Calibrated with Sir Jasper Talada (PhilRice Muñoz)", cell_style),
        ],
        [
            Paragraph("<b>Healthy HSV Hue Band</b>", cell_bold),
            Paragraph(f"{STANDARDS['hue'][0]} to {STANDARDS['hue'][1]} (Ideal: {(STANDARDS['hue'][0] + STANDARDS['hue'][1]) / 2:.1f})", cell_style),
            Paragraph("Maps to IRRI LCC Panel 4 (Healthy Green)", cell_style),
        ],
        [
            Paragraph("<b>Healthy Plant Height Range</b>", cell_bold),
            Paragraph(f"{STANDARDS['height'][0]} to {STANDARDS['height'][1]} cm (Pole: {STANDARDS['pole']} m)", cell_style),
            Paragraph("Standard mature canopy height band for NSIC Rc 222", cell_style),
        ],
        [
            Paragraph("<b>Safety &amp; Auto-Drive Limits</b>", cell_bold),
            Paragraph(
                f"Max Tilt: {STANDARDS['tilt']}° | GPS Samples: {STANDARDS['gps_n']} | Auto Cycle: {STANDARDS.get('auto_drive_s', 5)}s",
                cell_style,
            ),
            Paragraph(f"Total Stored Records: {len(records)} | Logged Auto Cycles: {len(auto_path)}", cell_style),
        ],
        [
            Paragraph("<b>Live Validation Accuracy (N = " + str(metrics["n"]) + ")</b>", cell_bold),
            Paragraph(f"Height MAE: {h_mae} (RMSE: {h_rmse}) | Hue MAE: {hue_mae} (RMSE: {hue_rmse})", cell_style),
            Paragraph("Thresholds: Height Error <= 2.0 cm, Hue Error <= 5.0", cell_style),
        ],
    ]
    story.append(KeepTogether([_make_table(snap_rows, col_widths=[50 * mm, 62 * mm, 68 * mm])]))

    doc.build(story, onFirstPage=_draw_header_footer, onLaterPages=_draw_header_footer)
    return buf.getvalue()


def generate_pdf_file(output_path: Path | str = DEFAULT_PDF_PATH) -> Path:
    """Write the complete PDF manual to disk and return its Path."""
    out = Path(output_path)
    pdf_bytes = build_manual_pdf_bytes()
    out.write_bytes(pdf_bytes)
    return out


if __name__ == "__main__":
    saved = generate_pdf_file()
    print(f"Generated PDF Manual: {saved} ({saved.stat().st_size} bytes)")

