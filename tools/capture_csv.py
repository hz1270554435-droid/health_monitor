#!/usr/bin/env python3
"""Capture board training export output from a serial port.

The firmware prints one tagged CSV stream for radar and may send MIC data as
compact binary PCM blocks:
    device,tick_ms,...
    radar,...
    PCMB + 32-byte header + int16 PCM payload

This tool converts the stream into WAV/JSON audio sessions and CSV/JSON radar
sessions.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import struct
import sys
import time
import wave
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

AUDIO_BINARY_MAGIC = b"PCMB"
AUDIO_BINARY_HEADER = struct.Struct("<4sBBBBIIHHIIHH")
AUDIO_BINARY_VERSION = 1

RADAR_SESSION_CSV_HEADER = [
    "session_id",
    "t_ms",
    "type",
    "human_present",
    "range_cm",
    "total_phase",
    "breath_phase",
    "heart_phase",
    "breath_rate",
    "heart_rate",
    "quality_flag",
]

RADAR_UART_RAW_CSV_HEADER = [
    "session_id",
    "t_ms",
    "frame_id",
    "radar_type",
    "frame_len",
    "data_len",
    "frame_hex",
]

RADAR_TYPE_NAMES = {
    "0x0100": "text",
    "0x0f09": "human_status",
    "0x0a04": "human_position",
    "0x0a13": "phase",
    "0x0a14": "breath_rate",
    "0x0a15": "heart_rate",
    "0x0a16": "target_range",
    "0x0a17": "track_position",
}

RADAR_KNOWN_TYPES = set(RADAR_TYPE_NAMES)
RADAR_FRAME_ID_MODULUS = 0x8000
RADAR_MAX_REASONABLE_FRAME_GAP = 512


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture MIC/radar CSV rows from the board debug UART."
    )
    parser.add_argument(
        "--port",
        help="Serial port, for example COM7 or /dev/ttyACM0. Use 'auto' to pick a USB debug port.",
    )
    parser.add_argument("--baud", type=int, default=2000000, help="PC debug serial baud rate")
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
        "--radar-output",
        default="radar.csv",
        help=(
            "Radar debug CSV output path. In session export mode, relative paths "
            "are written under the session radar directory."
        ),
    )
    parser.add_argument(
        "--raw-output",
        default="serial_raw.bin",
        help="Raw serial byte capture path. Use an empty value to disable.",
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
        "--no-session-export",
        action="store_true",
        help="Disable training session WAV/JSON and radar CSV/JSON export.",
    )
    parser.add_argument(
        "--session-output-dir",
        default="sessions",
        help="Directory for training sessions. Relative paths are under --output-dir.",
    )
    parser.add_argument(
        "--audio-session-sec",
        type=int,
        default=60,
        help="Audio session length in seconds.",
    )
    parser.add_argument(
        "--radar-session-sec",
        type=int,
        default=100,
        help="Radar session length in seconds.",
    )
    parser.add_argument(
        "--person-id",
        default="p01",
        help="Person ID written to each session JSON file.",
    )
    parser.add_argument(
        "--scene",
        default="bedside_quiet",
        help="Scene label written to each session JSON file.",
    )
    parser.add_argument(
        "--distance-cm",
        type=int,
        default=80,
        help="Subject distance in cm written to each session JSON file.",
    )
    parser.add_argument(
        "--radar-angle",
        default="front",
        help="Radar angle label written to radar session JSON files.",
    )
    parser.add_argument(
        "--radar-posture",
        default="lying",
        help="Subject posture label written to radar session JSON files.",
    )
    parser.add_argument(
        "--radar-calibrated",
        action="store_true",
        help="Write calibrated=true to radar session JSON files.",
    )
    parser.add_argument(
        "--radar-notes",
        default="first trial, radar not fully tuned",
        help="Notes written to radar session JSON files.",
    )
    parser.add_argument(
        "--audio-fs",
        type=int,
        default=16000,
        help="Audio sample rate used for WAV headers and session timing.",
    )
    parser.add_argument(
        "--audio-channels",
        type=int,
        default=2,
        choices=(1, 2),
        help="Audio channel count used for WAV headers.",
    )
    parser.add_argument(
        "--audio-bit-depth",
        type=int,
        default=16,
        choices=(16,),
        help="Audio bit depth used for WAV headers. Board PCM export is int16.",
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


def make_run_output_dir(output_dir: str, now: datetime) -> Path:
    root = Path(output_dir)
    run_dir = root / now.strftime("%Y-%m-%d_%H-%M-%S")
    suffix = 1
    while run_dir.exists():
        run_dir = root / f"{now.strftime('%Y-%m-%d_%H-%M-%S')}_{suffix:02d}"
        suffix += 1
    run_dir.mkdir(parents=True, exist_ok=False)
    return run_dir


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


def parse_int(value: str, default: int | None = None) -> int | None:
    try:
        return int(value.strip(), 0)
    except (AttributeError, ValueError):
        return default


def parse_decoded_fields(decoded: str) -> dict[str, str]:
    fields: dict[str, str] = {}
    for part in decoded.split(";"):
        key, sep, value = part.partition("=")
        if sep:
            fields[key.strip()] = value.strip()
    return fields


def radar_type_name(type_value: str) -> str:
    key = type_value.strip().lower()
    if not key:
        return ""
    return RADAR_TYPE_NAMES.get(key, key)


def resolve_session_root(path_value: str, output_dir: str) -> Path:
    path = Path(path_value)
    if path.is_absolute():
        return path
    return Path(output_dir) / path


def build_session_radar_output_path(
    path_value: str, session_root: Path, timestamp: str | None
) -> Path:
    path = Path(path_value)
    if path.is_absolute():
        parent = path.parent
    else:
        parent = session_root / "radar" / path.parent

    suffix = path.suffix
    stem = path.stem
    name = f"{stem}_{timestamp}{suffix}" if timestamp else path.name
    return parent / name


def session_timestamp(now: datetime) -> str:
    return now.strftime("%Y%m%d_%H%M%S")


def checksum_u8(data: bytes) -> int:
    return sum(data) & 0xFFFF


class AudioSessionWriter:
    def __init__(self, args: argparse.Namespace, root: Path):
        self.person_id = args.person_id
        self.scene = args.scene
        self.distance_cm = args.distance_cm
        self.fs = args.audio_fs
        self.channels = args.audio_channels
        self.bit_depth = args.audio_bit_depth
        self.session_sec = args.audio_session_sec
        self.max_frames = self.fs * self.session_sec
        self.root = root / "audio"
        self.session_index = 0
        self.session_id = ""
        self.start_tick_ms = 0
        self.frames = 0
        self.wav_file = None
        self.json_path: Path | None = None

    def write_row(self, row: list[str], now: datetime) -> None:
        if row[0].strip().lower() != "mic" or row[3].strip() != "1":
            return

        tick_ms = parse_int(row[1])
        sample_index = parse_int(row[5])
        left = parse_int(row[6])
        right = parse_int(row[7])
        if tick_ms is None or sample_index is None or left is None or right is None:
            return

        self._write_sample(tick_ms, left, right, now)

    def write_pcm_block(
        self,
        tick_ms: int,
        samples: tuple[int, ...],
        channels: int,
        now: datetime,
    ) -> None:
        if channels not in (1, 2):
            return

        for offset in range(0, len(samples), channels):
            left = samples[offset]
            right = samples[offset + 1] if channels == 2 else 0
            self._write_sample(tick_ms, left, right, now)

    def _write_sample(self, tick_ms: int, left: int, right: int, now: datetime) -> None:
        if self.wav_file is None:
            self._open_session(tick_ms, now)
        elif self.frames >= self.max_frames:
            self.close(complete=True)
            self._open_session(tick_ms, now)

        if self.channels == 1:
            self.wav_file.writeframesraw(struct.pack("<h", left))
        else:
            self.wav_file.writeframesraw(struct.pack("<hh", left, right))

        self.frames += 1

    def close(self, complete: bool = False) -> None:
        if self.wav_file is None:
            return

        duration_s = self.session_sec if complete else round(self.frames / self.fs, 3)
        meta = {
            "session_id": self.session_id,
            "person_id": self.person_id,
            "fs": self.fs,
            "channels": self.channels,
            "bit_depth": self.bit_depth,
            "scene": self.scene,
            "distance_cm": self.distance_cm,
            "start_tick_ms": self.start_tick_ms,
            "duration_s": duration_s,
        }

        self.wav_file.close()
        with self.json_path.open("w", encoding="utf-8") as json_file:
            json.dump(meta, json_file, indent=2)
            json_file.write("\n")

        self.wav_file = None
        self.json_path = None
        self.frames = 0

    def _open_session(self, start_tick_ms: int, now: datetime) -> None:
        self.session_index += 1
        self.session_id = f"session_{self.session_index:04d}"
        timestamp = session_timestamp(now)
        base = f"{self.session_id}_{timestamp}_audio"
        self.root.mkdir(parents=True, exist_ok=True)

        wav_path = self.root / f"{base}.wav"
        self.json_path = self.root / f"{base}.json"
        self.start_tick_ms = start_tick_ms
        self.frames = 0

        self.wav_file = wave.open(str(wav_path), "wb")
        self.wav_file.setnchannels(self.channels)
        self.wav_file.setsampwidth(self.bit_depth // 8)
        self.wav_file.setframerate(self.fs)


class RadarSessionWriter:
    def __init__(self, args: argparse.Namespace, root: Path):
        self.person_id = args.person_id
        self.scene = args.scene
        self.distance_cm = args.distance_cm
        self.angle = args.radar_angle
        self.posture = args.radar_posture
        self.calibrated = args.radar_calibrated
        self.notes = args.radar_notes
        self.session_sec = args.radar_session_sec
        self.session_ms = self.session_sec * 1000
        self.root = root / "radar"
        self.session_index = 0
        self.session_id = ""
        self.start_tick_ms = 0
        self.last_tick_ms = 0
        self.last_driver_drop: int | None = None
        self.last_msg_drop: int | None = None
        self.last_frame_id: int | None = None
        self.csv_file = None
        self.csv_writer = None
        self.raw_file = None
        self.raw_writer = None
        self.json_path: Path | None = None

    def write_row(self, row: list[str], now: datetime) -> None:
        if row[0].strip().lower() != "radar":
            return

        tick_ms = parse_int(row[1])
        if tick_ms is None:
            return

        if self.csv_file is None:
            self._open_session(tick_ms, now)
        elif tick_ms - self.start_tick_ms >= self.session_ms:
            self.close(complete=True)
            self._open_session(tick_ms, now)

        valid = row[3].strip() == "1"
        driver_drop = parse_int(row[17], 0)
        msg_drop = parse_int(row[18], 0)
        no_new_drop = (
            self.last_driver_drop is None
            or (
                driver_drop == self.last_driver_drop
                and msg_drop == self.last_msg_drop
            )
        )
        quality_flag = 1 if valid and no_new_drop else 0
        frame_id = parse_int(row[13])
        if not self._frame_id_is_continuous(frame_id):
            quality_flag = 0
        self.last_driver_drop = driver_drop
        self.last_msg_drop = msg_drop
        self.last_frame_id = frame_id if frame_id is not None else self.last_frame_id

        self.raw_writer.writerow(
            [
                self.session_id,
                tick_ms - self.start_tick_ms,
                row[13],
                row[14],
                row[15],
                row[16],
                row[19],
            ]
        )

        type_key = row[14].strip().lower()
        if type_key not in RADAR_KNOWN_TYPES:
            return

        type_name = radar_type_name(type_key)
        fields = parse_decoded_fields(row[20])
        values = self._values_from_fields(fields)

        self.csv_writer.writerow(
            [
                self.session_id,
                tick_ms - self.start_tick_ms,
                type_name,
                values["human_present"],
                values["range_cm"],
                values["total_phase"],
                values["breath_phase"],
                values["heart_phase"],
                values["breath_rate"],
                values["heart_rate"],
                quality_flag,
            ]
        )

    def close(self, complete: bool = False) -> None:
        if self.csv_file is None:
            return

        meta = {
            "session_id": self.session_id,
            "person_id": self.person_id,
            "duration_s": self.session_sec,
            "distance_cm": self.distance_cm,
            "angle": self.angle,
            "posture": self.posture,
            "scene": self.scene,
            "calibrated": self.calibrated,
            "notes": self.notes,
        }

        self.csv_file.close()
        self.raw_file.close()
        with self.json_path.open("w", encoding="utf-8") as json_file:
            json.dump(meta, json_file, indent=2)
            json_file.write("\n")

        self.csv_file = None
        self.csv_writer = None
        self.raw_file = None
        self.raw_writer = None
        self.json_path = None
        self.last_driver_drop = None
        self.last_msg_drop = None
        self.last_frame_id = None

    def _open_session(self, start_tick_ms: int, now: datetime) -> None:
        self.session_index += 1
        self.session_id = f"radar_s{self.session_index:03d}"
        timestamp = session_timestamp(now)
        base = f"{self.session_id}_{timestamp}_radar"
        self.root.mkdir(parents=True, exist_ok=True)

        csv_path = self.root / f"{base}.csv"
        raw_path = self.root / f"{base}_uart_raw.csv"
        self.json_path = self.root / f"{base}.json"
        self.start_tick_ms = start_tick_ms
        self.last_tick_ms = start_tick_ms
        self.last_driver_drop = None
        self.last_msg_drop = None
        self.last_frame_id = None

        self.csv_file = csv_path.open("w", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(RADAR_SESSION_CSV_HEADER)
        self.raw_file = raw_path.open("w", newline="", encoding="utf-8")
        self.raw_writer = csv.writer(self.raw_file)
        self.raw_writer.writerow(RADAR_UART_RAW_CSV_HEADER)

    def _values_from_fields(self, fields: dict[str, str]) -> dict[str, str]:
        values = {
            "human_present": "",
            "range_cm": "",
            "total_phase": "",
            "breath_phase": "",
            "heart_phase": "",
            "breath_rate": "",
            "heart_rate": "",
        }
        mapping = {
            "human_present": "human_present",
            "range_cm": "range_cm",
            "total_phase": "total_phase",
            "breath_phase": "breath_phase",
            "heart_phase": "heart_phase",
            "breath_rate_per_min": "breath_rate",
            "heart_rate_bpm": "heart_rate",
        }
        for source, target in mapping.items():
            if source in fields:
                values[target] = fields[source]
        return values

    def _frame_id_is_continuous(self, frame_id: int | None) -> bool:
        if frame_id is None or self.last_frame_id is None:
            return True

        if frame_id >= self.last_frame_id:
            delta = frame_id - self.last_frame_id
        else:
            delta = frame_id + RADAR_FRAME_ID_MODULUS - self.last_frame_id

        return 0 < delta <= RADAR_MAX_REASONABLE_FRAME_GAP


class TrainingSessionExporter:
    def __init__(self, args: argparse.Namespace):
        root = resolve_session_root(args.session_output_dir, args.output_dir)
        self.root = root
        (root / "audio").mkdir(parents=True, exist_ok=True)
        (root / "radar").mkdir(parents=True, exist_ok=True)
        self.audio = AudioSessionWriter(args, root)
        self.radar = RadarSessionWriter(args, root)

    def write_row(self, row: list[str]) -> None:
        now = datetime.now()
        device = row[0].strip().lower()
        if device == "mic":
            self.audio.write_row(row, now)
        elif device == "radar":
            self.radar.write_row(row, now)

    def close(self) -> None:
        self.audio.close()
        self.radar.close()


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


def mic_rows_from_binary_frame(
    tick_ms: int,
    sequence: int,
    block_index: int,
    samples: tuple[int, ...],
    channels: int,
    samples_per_channel: int,
    driver_drop: int,
    msg_drop: int,
):
    for sample_index in range(samples_per_channel):
        base = sample_index * channels
        left = samples[base]
        right = samples[base + 1] if channels == 2 else 0
        yield [
            "mic",
            str(tick_ms),
            str(sequence),
            "1",
            str(block_index),
            str(sample_index),
            str(left),
            str(right),
            "",
            "",
            "",
            "",
            "",
            "",
            "",
            "",
            "",
            str(driver_drop),
            str(msg_drop),
            "",
            "",
        ]


def print_raw_capture_hint(raw_path: Path) -> None:
    if not raw_path.exists() or raw_path.stat().st_size == 0:
        print("Raw capture is empty: check the selected COM port and board power.")
        return

    sample = raw_path.read_bytes()[:4096]
    if not sample:
        return

    zero_ratio = sample.count(0) / len(sample)
    ascii_text = sample.decode("ascii", errors="ignore")
    if AUDIO_BINARY_MAGIC not in sample and "radar," not in ascii_text:
        if zero_ratio > 0.9:
            print(
                "Raw capture is mostly 0x00 bytes. This usually means the "
                "debug UART baud rate is not supported by the USB serial bridge "
                "or does not match the firmware."
            )
        else:
            print(
                "Raw capture does not contain PCMB/radar markers. Check baud "
                "rate first, then confirm the selected COM port is the KitProg "
                "USB-UART port."
            )


def main() -> int:
    args = parse_args()
    serial, list_ports = import_serial()

    if args.audio_session_sec <= 0 or args.radar_session_sec <= 0:
        print("error: session lengths must be positive", file=sys.stderr)
        return 2
    if args.audio_fs <= 0:
        print("error: --audio-fs must be positive", file=sys.stderr)
        return 2

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

    run_started_at = datetime.now()
    run_output_dir = make_run_output_dir(args.output_dir, run_started_at)
    args.output_dir = str(run_output_dir)
    timestamp = None
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
    session_exporter = None

    if not args.no_session_export:
        session_exporter = TrainingSessionExporter(args)
        radar_path = build_session_radar_output_path(
            args.radar_output, session_exporter.root, timestamp
        )

    if raw_path is not None:
        raw_path.parent.mkdir(parents=True, exist_ok=True)
        raw_file = raw_path.open("wb")

    if not args.no_split:
        radar_file, radar_writer = open_writer(str(radar_path))

    deadline = None if args.duration <= 0 else time.monotonic() + args.duration
    rows = 0
    skipped = 0
    binary_frames = 0

    def process_text_line(line: str) -> None:
        nonlocal rows, skipped

        line = line.strip()
        if not line:
            return
        if line.startswith("device,") or line.startswith("#"):
            return

        try:
            row = normalize_row(next(csv.reader([line])))
        except csv.Error:
            skipped += 1
            return

        device = row[0].strip().lower()
        if device not in {"mic", "radar"}:
            skipped += 1
            return

        if device == "radar":
            row[20] = decode_radar_payload(row)

        if session_exporter is not None:
            session_exporter.write_row(row)

        combined_writer.writerow(row)
        if device == "mic" and mic_writer is not None:
            mic_writer.writerow(row)
        elif device == "radar" and radar_writer is not None:
            radar_writer.writerow(row)

        rows += 1

    def process_audio_binary_frame(header_values, payload: bytes) -> None:
        nonlocal rows, skipped, binary_frames

        (
            _magic,
            version,
            channels,
            bits_per_sample,
            block_index,
            tick_ms,
            sequence,
            sample_count,
            samples_per_channel,
            driver_drop,
            msg_drop,
            payload_bytes,
            checksum,
        ) = header_values

        if (
            version != AUDIO_BINARY_VERSION
            or bits_per_sample != 16
            or channels not in (1, 2)
            or payload_bytes != len(payload)
            or payload_bytes != sample_count * 2
            or sample_count != samples_per_channel * channels
            or checksum_u8(payload) != checksum
        ):
            skipped += 1
            return

        samples = struct.unpack_from(f"<{sample_count}h", payload)
        now = datetime.now()

        if session_exporter is not None:
            session_exporter.audio.write_pcm_block(tick_ms, samples, channels, now)

        rows += samples_per_channel
        binary_frames += 1

    try:
        with serial.Serial(args.port, args.baud, timeout=1) as ser:
            print(f"Capturing {args.port} at {args.baud} baud...")
            print(f"Run output directory: {run_output_dir}")
            print(f"Combined CSV: {combined_path}")
            if not args.no_split:
                print(f"Radar debug CSV: {radar_path}")
            if session_exporter is not None:
                print(f"Training sessions: {session_exporter.root}")
            if raw_path is not None:
                print(f"Raw byte capture: {raw_path}")
            rx_buffer = bytearray()
            while deadline is None or time.monotonic() < deadline:
                raw = ser.read(4096)
                if not raw:
                    continue

                if raw_file is not None:
                    raw_file.write(raw)

                rx_buffer.extend(raw)
                while rx_buffer:
                    magic_index = rx_buffer.find(AUDIO_BINARY_MAGIC)
                    newline_index = rx_buffer.find(b"\n")

                    if magic_index == 0:
                        if len(rx_buffer) < AUDIO_BINARY_HEADER.size:
                            break

                        header_values = AUDIO_BINARY_HEADER.unpack(
                            bytes(rx_buffer[: AUDIO_BINARY_HEADER.size])
                        )
                        payload_bytes = header_values[11]
                        frame_len = AUDIO_BINARY_HEADER.size + payload_bytes
                        if len(rx_buffer) < frame_len:
                            break

                        payload = bytes(rx_buffer[AUDIO_BINARY_HEADER.size:frame_len])
                        del rx_buffer[:frame_len]
                        process_audio_binary_frame(header_values, payload)
                        continue

                    if newline_index >= 0 and (
                        magic_index < 0 or newline_index < magic_index
                    ):
                        line_bytes = bytes(rx_buffer[: newline_index + 1])
                        del rx_buffer[: newline_index + 1]
                        process_text_line(line_bytes.decode("utf-8", errors="replace"))
                        continue

                    if magic_index > 0:
                        prefix = bytes(rx_buffer[:magic_index])
                        del rx_buffer[:magic_index]
                        if prefix.strip():
                            skipped += 1
                        continue

                    if len(rx_buffer) > 8192:
                        del rx_buffer[:-3]
                        skipped += 1
                    break

                if rows % 200 == 0:
                    combined_file.flush()
                    if mic_file is not None:
                        mic_file.flush()
                    if radar_file is not None:
                        radar_file.flush()
                    if raw_file is not None:
                        raw_file.flush()
                    print(
                        f"\rrows={rows} binary_frames={binary_frames} skipped={skipped}",
                        end="",
                        flush=True,
                    )
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
        if session_exporter is not None:
            session_exporter.close()

    print(f"\nDone. rows={rows} binary_frames={binary_frames} skipped={skipped}")
    if rows == 0 and raw_path is not None:
        print(
            f"No MIC/radar CSV rows were captured. Check {raw_path} "
            "to see the raw serial output."
        )
        print_raw_capture_hint(raw_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
