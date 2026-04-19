#!/usr/bin/env python3
"""Capture board CSV output from a serial port.

The firmware prints one tagged CSV stream:
    device,tick_ms,...
    mic,...
    radar,...

This tool saves the combined stream and also splits rows into MIC and radar
CSV files by the first "device" column.
"""

from __future__ import annotations

import argparse
import csv
import math
import struct
import sys
import time
from datetime import datetime
from pathlib import Path


HEADER = [
    "device",
    "tick_ms",
    "sequence",
    "valid",
    "block_index",
    "sample_index",
    "left",
    "right",
    "sample_count",
    "min",
    "max",
    "peak_abs",
    "mean_abs",
    "frame_id",
    "radar_type",
    "frame_len",
    "data_len",
    "driver_drop",
    "msg_drop",
    "frame_hex",
    "radar_decoded",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture MIC/radar CSV rows from the board debug UART."
    )
    parser.add_argument(
        "--port",
        help="Serial port, for example COM7 or /dev/ttyACM0. Use 'auto' to pick a USB debug port.",
    )
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Seconds to capture. 0 means run until Ctrl+C.",
    )
    parser.add_argument(
        "--output",
        default="sensor_export.csv",
        help="Combined CSV output file name.",
    )
    parser.add_argument("--mic-output", default="mic.csv", help="MIC CSV output path.")
    parser.add_argument(
        "--radar-output", default="radar.csv", help="Radar CSV output path."
    )
    parser.add_argument(
        "--raw-output",
        default="serial_raw.log",
        help="Raw serial text log path. Use an empty value to disable.",
    )
    parser.add_argument(
        "--output-dir",
        default="log_csv",
        help="Directory for relative output files.",
    )
    parser.add_argument(
        "--timestamp-format",
        default="%Y%m%d_%H%M%S",
        help="strftime pattern appended to output file names.",
    )
    parser.add_argument(
        "--no-timestamp",
        action="store_true",
        help="Do not append a timestamp to output file names.",
    )
    parser.add_argument(
        "--no-split",
        action="store_true",
        help="Only write the combined CSV file.",
    )
    parser.add_argument(
        "--list-ports",
        action="store_true",
        help="List serial ports and exit.",
    )
    return parser.parse_args()


def import_serial():
    try:
        import serial
        from serial.tools import list_ports
    except ImportError as exc:
        raise SystemExit(
            "pyserial is required. Install it with: python -m pip install pyserial"
        ) from exc

    return serial, list_ports


def normalize_row(row: list[str]) -> list[str]:
    if len(row) < len(HEADER):
        row = row + [""] * (len(HEADER) - len(row))
    return row[: len(HEADER)]


def build_output_path(path_value: str, output_dir: str, timestamp: str | None) -> Path:
    path = Path(path_value)
    parent = path.parent if str(path.parent) != "." else Path(output_dir)
    suffix = path.suffix
    stem = path.stem
    name = f"{stem}_{timestamp}{suffix}" if timestamp else path.name
    return parent / name


def le_u16(data: bytes, offset: int = 0) -> int:
    return int.from_bytes(data[offset : offset + 2], "little", signed=False)


def le_u32(data: bytes, offset: int = 0) -> int:
    return int.from_bytes(data[offset : offset + 4], "little", signed=False)


def le_i32(data: bytes, offset: int = 0) -> int:
    return int.from_bytes(data[offset : offset + 4], "little", signed=True)


def le_float(data: bytes, offset: int = 0) -> float:
    return struct.unpack_from("<f", data, offset)[0]


def fmt_float(value: float) -> str:
    if not math.isfinite(value):
        return ""
    return f"{value:.3f}"


def sanitize_text(data: bytes) -> str:
    chars: list[str] = []
    for byte in data:
        if byte in (ord(","), ord("\r"), ord("\n")):
            chars.append(";")
        elif 32 <= byte <= 126:
            chars.append(chr(byte))
        else:
            chars.append(".")
    return "".join(chars).strip()


def decode_radar_payload(row: list[str]) -> str:
    device = row[0].strip().lower()
    if device != "radar" or row[20]:
        return row[20]

    frame_hex = row[19].strip().replace(" ", "")
    if not frame_hex:
        return ""

    try:
        frame = bytes.fromhex(frame_hex)
    except ValueError:
        return ""

    if len(frame) < 8:
        return ""

    try:
        radar_type = int(row[14], 16)
    except ValueError:
        radar_type = int.from_bytes(frame[5:7], "big", signed=False)

    data_len = int.from_bytes(frame[3:5], "big", signed=False)
    payload_end = min(8 + data_len, len(frame))
    data = frame[8:payload_end]

    try:
        if radar_type == 0x0100:
            return f"text={sanitize_text(data)}"
        if radar_type == 0x0F09 and len(data) >= 2:
            return f"human_present={1 if le_u16(data) else 0}"
        if radar_type == 0x0A04 and len(data) >= 4:
            target_count = le_u32(data)
            parts = [f"target_count={target_count}"]
            if target_count and len(data) >= 24:
                parts.extend(
                    [
                        f"x_m={fmt_float(le_float(data, 4))}",
                        f"y_m={fmt_float(le_float(data, 8))}",
                        f"z_m={fmt_float(le_float(data, 12))}",
                        f"dop_idx={le_i32(data, 16)}",
                        f"cluster_id={le_i32(data, 20)}",
                    ]
                )
            return ";".join(parts)
        if radar_type == 0x0A13 and len(data) >= 12:
            return (
                f"total_phase={fmt_float(le_float(data, 0))};"
                f"breath_phase={fmt_float(le_float(data, 4))};"
                f"heart_phase={fmt_float(le_float(data, 8))}"
            )
        if radar_type == 0x0A14 and len(data) >= 4:
            return f"breath_rate_per_min={fmt_float(le_float(data))}"
        if radar_type == 0x0A15 and len(data) >= 4:
            return f"heart_rate_bpm={fmt_float(le_float(data))}"
        if radar_type == 0x0A16 and len(data) >= 8:
            range_flag = le_u32(data)
            parts = [f"range_flag={range_flag}"]
            if range_flag:
                parts.append(f"range_cm={fmt_float(le_float(data, 4))}")
            return ";".join(parts)
        if radar_type == 0x0A17 and len(data) >= 12:
            return (
                f"x_m={fmt_float(le_float(data, 0))};"
                f"y_m={fmt_float(le_float(data, 4))};"
                f"z_m={fmt_float(le_float(data, 8))}"
            )
    except (struct.error, ValueError):
        return ""

    return ""


def choose_auto_port(list_ports) -> str:
    ports = list(list_ports.comports())
    if not ports:
        raise SystemExit("No serial ports found. Connect the board and try again.")

    preferred_keywords = (
        "kitprog",
        "cmsis-dap",
        "cypress",
        "infineon",
        "usb serial",
        "usb-uart",
        "usb uart",
    )

    for port in ports:
        text = " ".join(
            str(value or "")
            for value in (port.device, port.description, port.manufacturer, port.hwid)
        ).lower()
        if any(keyword in text for keyword in preferred_keywords):
            return port.device

    non_builtin_ports = [
        port
        for port in ports
        if "communications port" not in str(port.description).lower()
        and "通信端口" not in str(port.description).lower()
    ]
    if len(non_builtin_ports) == 1:
        return non_builtin_ports[0].device

    if len(ports) == 1:
        return ports[0].device

    available = ", ".join(f"{port.device} ({port.description})" for port in ports)
    raise SystemExit(
        "Could not auto-select a serial port. "
        f"Available ports: {available}. Pass --port COMx explicitly."
    )


def open_writer(path: str):
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    csv_file = output_path.open("w", newline="", encoding="utf-8")
    writer = csv.writer(csv_file)
    writer.writerow(HEADER)
    return csv_file, writer


def main() -> int:
    args = parse_args()
    serial, list_ports = import_serial()

    if args.list_ports:
        for port in list_ports.comports():
            print(f"{port.device}\t{port.description}")
        return 0

    if not args.port:
        print("error: --port is required unless --list-ports is used", file=sys.stderr)
        return 2

    if args.port.lower() == "auto":
        args.port = choose_auto_port(list_ports)
        print(f"Auto-selected serial port: {args.port}")

    timestamp = None if args.no_timestamp else datetime.now().strftime(args.timestamp_format)
    combined_path = build_output_path(args.output, args.output_dir, timestamp)
    mic_path = build_output_path(args.mic_output, args.output_dir, timestamp)
    radar_path = build_output_path(args.radar_output, args.output_dir, timestamp)
    raw_path = (
        build_output_path(args.raw_output, args.output_dir, timestamp)
        if args.raw_output
        else None
    )

    combined_file, combined_writer = open_writer(str(combined_path))
    raw_file = None
    mic_file = mic_writer = None
    radar_file = radar_writer = None

    if raw_path is not None:
        raw_path.parent.mkdir(parents=True, exist_ok=True)
        raw_file = raw_path.open("w", encoding="utf-8", newline="")

    if not args.no_split:
        mic_file, mic_writer = open_writer(str(mic_path))
        radar_file, radar_writer = open_writer(str(radar_path))

    deadline = None if args.duration <= 0 else time.monotonic() + args.duration
    rows = 0
    skipped = 0

    try:
        with serial.Serial(args.port, args.baud, timeout=1) as ser:
            print(f"Capturing {args.port} at {args.baud} baud...")
            print(f"Combined CSV: {combined_path}")
            if not args.no_split:
                print(f"MIC CSV: {mic_path}")
                print(f"Radar CSV: {radar_path}")
            if raw_path is not None:
                print(f"Raw log: {raw_path}")
            while deadline is None or time.monotonic() < deadline:
                raw = ser.readline()
                if not raw:
                    continue

                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                if raw_file is not None:
                    raw_file.write(line + "\n")

                if line.startswith("device,"):
                    continue

                try:
                    row = normalize_row(next(csv.reader([line])))
                except csv.Error:
                    skipped += 1
                    continue

                device = row[0].strip().lower()
                if device not in {"mic", "radar"}:
                    skipped += 1
                    continue

                if device == "radar":
                    row[20] = decode_radar_payload(row)

                combined_writer.writerow(row)
                if device == "mic" and mic_writer is not None:
                    mic_writer.writerow(row)
                elif device == "radar" and radar_writer is not None:
                    radar_writer.writerow(row)

                rows += 1
                if rows % 200 == 0:
                    combined_file.flush()
                    if mic_file is not None:
                        mic_file.flush()
                    if radar_file is not None:
                        radar_file.flush()
                    if raw_file is not None:
                        raw_file.flush()
                    print(f"\rrows={rows} skipped={skipped}", end="", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        combined_file.close()
        if raw_file is not None:
            raw_file.close()
        if mic_file is not None:
            mic_file.close()
        if radar_file is not None:
            radar_file.close()

    print(f"\nDone. rows={rows} skipped={skipped}")
    if rows == 0 and raw_path is not None:
        print(
            f"No MIC/radar CSV rows were captured. Check {raw_path} "
            "to see the raw serial output."
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
