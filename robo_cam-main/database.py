"""
database.py
-------------
SQLite-backed storage for RiceTrack operational records and validation data.

Operational records contain the CV result. Validation readings are stored
separately so manual/reference measurements do not masquerade as rover/CV
measurements.
"""

import json
import math
import sqlite3
from datetime import datetime
from pathlib import Path

from common import STANDARDS

DB_PATH = Path(__file__).resolve().parent / "ricetrack.db"
AUTO_DRIVE_JSON_PATH = Path(__file__).resolve().parent / "auto_drive_path.json"
FARM_CAPTURES_DIR = Path(__file__).resolve().parent / "farm_captures"

DEFAULT_PLOT = "A-01"
DEFAULT_VARIANT = "NSIC Rc 222"
DEFAULT_GPS = "15.67170, 120.89280"
DEFAULT_AVG_HEIGHT = 100.0
DEFAULT_HEIGHT_REPORT = "healthy"

# Preliminary Health Index constants from the thesis methodology.
IDEAL_HUE = 58.5
MAX_HUE_DEVIATION = 27.5
IDEAL_HEIGHT = 99.5

_DISTINCT_ALLOWED = {"plot", "variant", "status"}


def _connect():
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    return conn


def _add_column_if_missing(conn, table: str, column: str, definition: str):
    existing = {
        row[1] for row in conn.execute(f"PRAGMA table_info({table})").fetchall()
    }
    if column not in existing:
        conn.execute(f"ALTER TABLE {table} ADD COLUMN {column} {definition}")


def init_db():
    with _connect() as conn:
        conn.execute(
            """
            CREATE TABLE IF NOT EXISTS records (
                id TEXT PRIMARY KEY,
                captured_at TEXT NOT NULL,
                plot TEXT NOT NULL,
                variant TEXT NOT NULL,
                gps TEXT NOT NULL,
                avg_height REAL NOT NULL,
                avg_hue REAL NOT NULL,
                height_report TEXT NOT NULL,
                leaf_report TEXT NOT NULL,
                status TEXT NOT NULL,
                image BLOB
            )
            """
        )

        # Demo color-calibration fields. They are nullable so existing DBs
        # remain compatible with older records.
        _add_column_if_missing(conn, "records", "lab_l", "REAL")
        _add_column_if_missing(conn, "records", "lab_a", "REAL")
        _add_column_if_missing(conn, "records", "lab_b", "REAL")
        _add_column_if_missing(conn, "records", "hue_method", "TEXT")
        _add_column_if_missing(conn, "records", "segmentation_v_min", "INTEGER")
        _add_column_if_missing(conn, "records", "file_path", "TEXT")

        conn.execute(
            """
            CREATE TABLE IF NOT EXISTS validation_readings (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                record_id TEXT NOT NULL,
                created_at TEXT NOT NULL,
                manual_height REAL NOT NULL,
                manual_hue REAL NOT NULL,
                height_error REAL NOT NULL,
                hue_error REAL NOT NULL,
                color_score REAL NOT NULL,
                height_score REAL NOT NULL,
                health_index REAL NOT NULL,
                status TEXT NOT NULL,
                notes TEXT NOT NULL DEFAULT '',
                FOREIGN KEY(record_id) REFERENCES records(id)
            )
            """
        )
        conn.execute(
            "CREATE INDEX IF NOT EXISTS idx_validation_record_id "
            "ON validation_readings(record_id)"
        )
        conn.execute(
            "CREATE INDEX IF NOT EXISTS idx_validation_created_at "
            "ON validation_readings(created_at)"
        )
        conn.execute(
            """
            CREATE TABLE IF NOT EXISTS auto_drive_cycles (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                cycle_num INTEGER NOT NULL,
                logged_at TEXT NOT NULL,
                mode TEXT NOT NULL,
                direction TEXT NOT NULL,
                interval_s INTEGER NOT NULL,
                gps TEXT NOT NULL,
                lat REAL NOT NULL,
                lon REAL NOT NULL,
                sats INTEGER NOT NULL DEFAULT 0,
                roll REAL NOT NULL DEFAULT 0.0,
                pitch REAL NOT NULL DEFAULT 0.0,
                speed_mps REAL NOT NULL DEFAULT 0.0,
                median_hue REAL,
                status TEXT NOT NULL DEFAULT 'Auto Drive',
                record_id TEXT
            )
            """
        )


def _compute_status(color_ok: bool, height_ok: bool) -> str:
    if color_ok and height_ok:
        return "Healthy"
    if not color_ok and not height_ok:
        return "Needs Inspection - both"
    return "Needs Inspection - " + ("color" if not color_ok else "height")


def compute_test_status(manual_height: float, manual_hue: float) -> str:
    """Return the requested Test-* status for a manual/reference reading."""
    color_ok = STANDARDS["hue"][0] <= manual_hue <= STANDARDS["hue"][1]
    height_ok = STANDARDS["height"][0] <= manual_height <= STANDARDS["height"][1]

    if color_ok and height_ok:
        return "Test - Healthy"
    if not color_ok and not height_ok:
        return "Test - Unhealthy - color + height"
    if not color_ok:
        return "Test - Unhealthy - color"
    return "Test - Unhealthy - height"


def _health_index_scores(manual_height: float, manual_hue: float):
    """Apply the preliminary thesis Health Index formulas using current STANDARDS."""
    ideal_hue = (STANDARDS["hue"][0] + STANDARDS["hue"][1]) / 2.0
    ideal_height = (STANDARDS["height"][0] + STANDARDS["height"][1]) / 2.0
    color_score = max(
        0.0,
        min(
            100.0,
            100.0
            - (abs(ideal_hue - manual_hue) / MAX_HUE_DEVIATION * 100.0),
        ),
    )
    height_score = max(
        0.0,
        min(
            100.0,
            100.0
            - (abs(ideal_height - manual_height) / max(1.0, ideal_height) * 100.0),
        ),
    )
    health_index = 0.7 * color_score + 0.3 * height_score
    return color_score, height_score, health_index


def recompute_all_statuses():
    """Re-evaluate record and validation statuses against the updated STANDARDS thresholds."""
    h_lo, h_hi = STANDARDS["height"]
    hue_lo, hue_hi = STANDARDS["hue"]
    with _connect() as conn:
        rec_rows = conn.execute("SELECT id, avg_height, avg_hue FROM records").fetchall()
        for r in rec_rows:
            height_ok = h_lo <= float(r["avg_height"]) <= h_hi
            color_ok = hue_lo <= float(r["avg_hue"]) <= hue_hi
            new_status = _compute_status(color_ok, height_ok)
            conn.execute("UPDATE records SET status = ? WHERE id = ?", (new_status, r["id"]))

        val_rows = conn.execute("SELECT id, manual_height, manual_hue FROM validation_readings").fetchall()
        for v in val_rows:
            mh = float(v["manual_height"])
            mhue = float(v["manual_hue"])
            c_score, h_score, h_idx = _health_index_scores(mh, mhue)
            new_v_status = compute_test_status(mh, mhue)
            conn.execute(
                "UPDATE validation_readings SET color_score = ?, height_score = ?, health_index = ?, status = ? WHERE id = ?",
                (c_score, h_score, h_idx, new_v_status, v["id"]),
            )


def _next_id(conn, plot: str) -> str:
    count = conn.execute(
        "SELECT COUNT(*) FROM records WHERE plot = ?", (plot,)
    ).fetchone()[0]
    return f"{plot}-{count + 1:02d}"


def insert_record(record: dict) -> str:
    """Insert one operational inspection record and return its ID."""
    plot = record.get("plot", DEFAULT_PLOT)
    variant = record.get("variant", DEFAULT_VARIANT)
    gps = record.get("gps", DEFAULT_GPS)
    avg_height = record.get("avg_height", DEFAULT_AVG_HEIGHT)
    height_report = record.get("height_report", DEFAULT_HEIGHT_REPORT)
    avg_hue = float(record["avg_hue"])
    leaf_report = record["leaf_report"]
    image_bytes = record.get("image")
    captured_at = record.get("captured_at") or datetime.now().isoformat(timespec="seconds")

    height_ok = STANDARDS["height"][0] <= avg_height <= STANDARDS["height"][1]
    color_ok = STANDARDS["hue"][0] <= avg_hue <= STANDARDS["hue"][1]
    status = record.get("status") or _compute_status(color_ok, height_ok)

    with _connect() as conn:
        record_id = _next_id(conn, plot)

        # Automatically save image to the farm_captures/ folder
        file_path_rel = None
        try:
            FARM_CAPTURES_DIR.mkdir(parents=True, exist_ok=True)
            ts_str = captured_at.replace(":", "").replace("-", "").replace("T", "_")
            filename = f"capture_{record_id}_{ts_str}.jpg"
            target_path = FARM_CAPTURES_DIR / filename
            raw_bgr = record.get("raw_bgr")
            if raw_bgr is not None:
                import cv2
                cv2.imwrite(str(target_path), raw_bgr, [cv2.IMWRITE_JPEG_QUALITY, 95])
                file_path_rel = f"farm_captures/{filename}"
            elif image_bytes:
                with open(target_path, "wb") as f:
                    f.write(image_bytes)
                file_path_rel = f"farm_captures/{filename}"

            # If annotated/segmented overlay is provided, also save it
            seg_bgr = record.get("seg_bgr")
            if seg_bgr is not None:
                import cv2
                seg_filename = f"capture_{record_id}_{ts_str}_seg.jpg"
                cv2.imwrite(str(FARM_CAPTURES_DIR / seg_filename), seg_bgr, [cv2.IMWRITE_JPEG_QUALITY, 92])
        except Exception:
            file_path_rel = None

        record["saved_file_path"] = file_path_rel

        conn.execute(
            """
            INSERT INTO records
                (id, captured_at, plot, variant, gps, avg_height, avg_hue,
                 height_report, leaf_report, status, image,
                 lab_l, lab_a, lab_b, hue_method, segmentation_v_min, file_path)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (
                record_id,
                captured_at,
                plot,
                variant,
                gps,
                avg_height,
                avg_hue,
                height_report,
                leaf_report,
                status,
                image_bytes,
                record.get("lab_l"),
                record.get("lab_a"),
                record.get("lab_b"),
                record.get("hue_method", "median"),
                record.get("segmentation_v_min", 12),
                file_path_rel,
            ),
        )
    return record_id


def fetch_records(month=None, plot=None, variant=None, status=None) -> list:
    """Return matching operational records, newest first."""
    query = "SELECT * FROM records WHERE 1=1"
    params = []

    if month and month != "All":
        query += " AND strftime('%Y-%m', captured_at) = ?"
        params.append(month)
    if plot and plot != "All":
        query += " AND plot = ?"
        params.append(plot)
    if variant and variant != "All":
        query += " AND variant = ?"
        params.append(variant)
    if status and status != "All":
        if status == "Needs Inspection":
            query += " AND status LIKE 'Needs Inspection%'"
        else:
            query += " AND status = ?"
            params.append(status)

    query += " ORDER BY captured_at DESC"

    with _connect() as conn:
        rows = conn.execute(query, params).fetchall()
    return [dict(row) for row in rows]


def distinct(column: str) -> list:
    if column not in _DISTINCT_ALLOWED:
        raise ValueError(f"Unsupported column for distinct(): {column}")
    with _connect() as conn:
        rows = conn.execute(
            f"SELECT DISTINCT {column} FROM records ORDER BY {column}"
        ).fetchall()
    return [row[0] for row in rows]


def distinct_months() -> list:
    """Distinct YYYY-MM values present in captured_at, newest first."""
    with _connect() as conn:
        rows = conn.execute(
            """
            SELECT DISTINCT strftime('%Y-%m', captured_at) AS m
            FROM records
            WHERE captured_at IS NOT NULL
            ORDER BY m DESC
            """
        ).fetchall()
    return [row[0] for row in rows if row[0]]


def insert_validation_reading(
    record_id: str,
    manual_height: float,
    manual_hue: float,
    notes: str = "",
) -> int:
    """Store one manual/reference measurement linked to a CV record."""
    try:
        manual_height = float(manual_height)
        manual_hue = float(manual_hue)
    except (TypeError, ValueError) as exc:
        raise ValueError("Manual height and hue must be numeric.") from exc

    if not math.isfinite(manual_height) or not math.isfinite(manual_hue):
        raise ValueError("Manual height and hue must be finite values.")
    if not (0.0 <= manual_hue <= 179.0):
        raise ValueError("Manual hue must be between 0 and 179.")
    if manual_height <= 0.0:
        raise ValueError("Manual height must be greater than 0 cm.")

    with _connect() as conn:
        cv_row = conn.execute(
            "SELECT avg_height, avg_hue FROM records WHERE id = ?",
            (record_id,),
        ).fetchone()
        if cv_row is None:
            raise ValueError(f"CV record {record_id} was not found.")

        color_score, height_score, health_index = _health_index_scores(
            manual_height, manual_hue
        )
        height_error = abs(float(cv_row["avg_height"]) - manual_height)
        hue_error = abs(float(cv_row["avg_hue"]) - manual_hue)
        status = compute_test_status(manual_height, manual_hue)

        cursor = conn.execute(
            """
            INSERT INTO validation_readings
                (record_id, created_at, manual_height, manual_hue,
                 height_error, hue_error, color_score, height_score,
                 health_index, status, notes)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (
                record_id,
                datetime.now().isoformat(timespec="seconds"),
                manual_height,
                manual_hue,
                height_error,
                hue_error,
                color_score,
                height_score,
                health_index,
                status,
                notes.strip(),
            ),
        )
        last_id = cursor.lastrowid
        return int(last_id) if last_id is not None else 0


def fetch_validation_pairs() -> list:
    """Return stored validation readings joined to their CV records."""
    with _connect() as conn:
        rows = conn.execute(
            """
            SELECT
                v.id AS validation_id,
                v.record_id,
                r.captured_at,
                r.plot,
                r.avg_height AS cv_height,
                r.avg_hue AS cv_hue,
                r.lab_l AS cv_lab_l,
                r.lab_a AS cv_lab_a,
                r.lab_b AS cv_lab_b,
                v.manual_height,
                v.manual_hue,
                v.height_error,
                v.hue_error,
                v.color_score,
                v.height_score,
                v.health_index,
                v.status,
                v.notes,
                v.created_at
            FROM validation_readings AS v
            JOIN records AS r ON r.id = v.record_id
            ORDER BY v.created_at DESC, v.id DESC
            """
        ).fetchall()
    return [dict(row) for row in rows]


def validation_metrics() -> dict:
    """Compute live MAE/RMSE values from all stored validation pairs."""
    rows = fetch_validation_pairs()
    n = len(rows)
    if not n:
        return {
            "n": 0,
            "height_mae": None,
            "height_rmse": None,
            "hue_mae": None,
            "hue_rmse": None,
        }

    height_errors = [float(row["height_error"]) for row in rows]
    hue_errors = [float(row["hue_error"]) for row in rows]

    return {
        "n": n,
        "height_mae": sum(abs(e) for e in height_errors) / n,
        "height_rmse": math.sqrt(sum(e * e for e in height_errors) / n),
        "hue_mae": sum(abs(e) for e in hue_errors) / n,
        "hue_rmse": math.sqrt(sum(e * e for e in hue_errors) / n),
    }


def log_auto_drive_cycle(entry: dict) -> dict:
    """
    Persist one 5-second Auto Train Drive cycle into BOTH:
      1. SQLite `auto_drive_cycles` GPS path table in `ricetrack.db`
      2. JSON file `auto_drive_path.json` (GPS Path + Telemetry log)
    """
    logged_at = entry.get("logged_at") or datetime.now().isoformat(timespec="seconds")
    cycle_num = int(entry.get("cycle_num", 1))
    mode = str(entry.get("mode", "train_pulse"))
    direction = str(entry.get("direction", "forward"))
    interval_s = int(entry.get("interval_s", STANDARDS.get("auto_drive_s", 5)))
    lat = float(entry.get("lat", 15.67170))
    lon = float(entry.get("lon", 120.89280))
    gps_str = str(entry.get("gps") or f"{lat:.5f}, {lon:.5f}")
    sats = int(entry.get("sats", 0))
    roll = float(entry.get("roll", 0.0))
    pitch = float(entry.get("pitch", 0.0))
    speed_mps = float(entry.get("speed_mps", 0.6))
    median_hue = entry.get("median_hue")
    median_hue_val = float(median_hue) if median_hue is not None else None
    status = str(entry.get("status") or "Auto Drive")
    record_id = entry.get("record_id")

    with _connect() as conn:
        cursor = conn.execute(
            """
            INSERT INTO auto_drive_cycles
                (cycle_num, logged_at, mode, direction, interval_s,
                 gps, lat, lon, sats, roll, pitch, speed_mps,
                 median_hue, status, record_id)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (
                cycle_num,
                logged_at,
                mode,
                direction,
                interval_s,
                gps_str,
                lat,
                lon,
                sats,
                roll,
                pitch,
                speed_mps,
                median_hue_val,
                status,
                record_id,
            ),
        )
        row_id = int(cursor.lastrowid or 0)
        rows = conn.execute(
            "SELECT * FROM auto_drive_cycles ORDER BY id ASC LIMIT 500"
        ).fetchall()

    path_list = [dict(r) for r in rows]
    payload = {
        "updated_at": logged_at,
        "total_cycles": len(path_list),
        "latest_cycle": cycle_num,
        "latest_direction": direction,
        "latest_gps": gps_str,
        "gps_path": path_list,
    }
    try:
        AUTO_DRIVE_JSON_PATH.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    except OSError:
        pass

    return {
        "id": row_id,
        "cycle_num": cycle_num,
        "logged_at": logged_at,
        "direction": direction,
        "gps": gps_str,
        "lat": lat,
        "lon": lon,
        "json_path": str(AUTO_DRIVE_JSON_PATH.name),
    }


def fetch_auto_drive_path(limit: int = 100) -> list:
    """Return logged Auto Train Drive GPS path waypoints ordered chronologically."""
    with _connect() as conn:
        rows = conn.execute(
            "SELECT * FROM auto_drive_cycles ORDER BY id DESC LIMIT ?",
            (int(limit),),
        ).fetchall()
    return [dict(r) for r in reversed(rows)]


init_db()
