#!/usr/bin/env python3
"""BLE Stage 2 golden-vector and nRF Connect helper.

This script intentionally has no third-party dependencies and does not create
files. It mirrors the firmware wire contract used by app_ble_protocol.c.
"""

from __future__ import annotations

import argparse
import re
from dataclasses import dataclass

MAGIC = 0xE8
PROTO = 0x01

MSG_REALTIME = 0x10
MSG_EVENT = 0x20
MSG_COMMAND = 0x30
MSG_COMMAND_RESPONSE = 0x31

ERR_OK = 0
ERR_INVALID_ARG = 5
ERR_CRC_FAILED = 7
ERR_CMD_UNSUPPORTED = 8
ERR_CMD_DENIED = 9
ERR_CMD_NOT_READY = 11
ERR_BUSY = 12

CMD_PING = 0x01
CMD_TIME_SYNC = 0x02
CMD_START_MONITOR = 0x03
CMD_STOP_MONITOR = 0x04
CMD_SET_ALERT_THRESHOLD = 0x05
CMD_GET_DEVICE_INFO = 0x06
CMD_GET_NIGHT_SUMMARY = 0x07
CMD_CLEAR_NIGHT_SUMMARY = 0x08
CMD_SET_LOG_LEVEL = 0x09

REALTIME_FRAME_LEN = 19
EVENT_FRAME_LEN = 19
COMMAND_FRAME_MIN_LEN = 12
COMMAND_MAX_PAYLOAD_LEN = 16
CMD_RESPONSE_FRAME_MIN_LEN = 13
ATT_NOTIFY_PAYLOAD_MAX_DEFAULT = 20
DEVICE_INFO_PAYLOAD_LEN_V2 = 7

CAPABILITY_FLAG_ENABLE = 1 << 0
CAPABILITY_FLAG_FAKE_DATA = 1 << 1
CAPABILITY_FLAG_STACK = 1 << 2
CAPABILITY_FLAG_DIAG = 1 << 3

RUNTIME_FLAG_MONITOR_REQUESTED = 1 << 0
RUNTIME_FLAG_TIME_SYNCED = 1 << 1

EXPECTED_REALTIME_HEX = (
    "e8 01 10 00 40 e2 01 00 00 10 4b 64 05 57 ff 02 02 9f 54"
)
EXPECTED_EVENT_HEX = (
    "e8 01 20 00 41 e2 01 00 00 01 00 00 00 01 02 57 03 05 32"
)
EXPECTED_PING_HEX = "e8 01 30 00 40 e2 01 00 00 01 00 48"


@dataclass(frozen=True)
class CommandVector:
    name: str
    command_id: int
    payload: bytes


def crc8_atm(data: bytes) -> int:
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def le32(value: int) -> bytes:
    return value.to_bytes(4, "little", signed=False)


def frame(body_without_crc: bytes) -> bytes:
    return body_without_crc + bytes([crc8_atm(body_without_crc)])


def header(msg_type: int, seq: int, ts_s: int, error: int = ERR_OK) -> bytes:
    return bytes([MAGIC, PROTO, msg_type, seq]) + le32(ts_s) + bytes([error])


def pack_realtime() -> bytes:
    body = (
        header(MSG_REALTIME, seq=0, ts_s=123456)
        + bytes(
            [
                16,  # rr_bpm
                75,  # hr_bpm
                100,  # presence
                5,  # motion
                87,  # cough_prob
                0xFF,  # snore_prob invalid
                2,  # fusion warning
                2,  # alert warning
                0x9F,  # quality flags
            ]
        )
    )
    return frame(body)


def pack_event() -> bytes:
    body = (
        header(MSG_EVENT, seq=0, ts_s=123457)
        + le32(1)
        + bytes(
            [
                1,  # cough
                2,  # severity
                87,  # confidence
                3,  # duration_s
                0x05,  # audio + fusion
            ]
        )
    )
    return frame(body)


def pack_command(seq: int, ts_s: int, command_id: int, payload: bytes = b"") -> bytes:
    if len(payload) > COMMAND_MAX_PAYLOAD_LEN:
        raise ValueError("payload too large")
    body = (
        header(MSG_COMMAND, seq=seq, ts_s=ts_s)
        + bytes([command_id, len(payload)])
        + payload
    )
    return frame(body)


def pack_cmd_response(
    seq: int, ts_s: int, command_id: int, status: int, payload: bytes = b""
) -> bytes:
    body = (
        header(MSG_COMMAND_RESPONSE, seq=seq, ts_s=ts_s, error=status)
        + bytes([command_id, status, len(payload)])
        + payload
    )
    return frame(body)


def pack_device_info_payload(
    capability_flags: int,
    runtime_flags: int,
    cough_threshold: int,
    warning_threshold: int,
    high_threshold: int,
) -> bytes:
    return bytes(
        [
            PROTO,
            capability_flags,
            runtime_flags,
            cough_threshold,
            warning_threshold,
            high_threshold,
            0,
        ]
    )


def stack_debug_capability_flags() -> int:
    return (
        CAPABILITY_FLAG_ENABLE
        | CAPABILITY_FLAG_FAKE_DATA
        | CAPABILITY_FLAG_STACK
        | CAPABILITY_FLAG_DIAG
    )


def command_vectors() -> list[CommandVector]:
    return [
        CommandVector("PING", CMD_PING, b""),
        CommandVector("TIME_SYNC", CMD_TIME_SYNC, le32(1735689600)),
        CommandVector("START_MONITOR", CMD_START_MONITOR, b""),
        CommandVector("STOP_MONITOR", CMD_STOP_MONITOR, b""),
        CommandVector(
            "SET_ALERT_THRESHOLD_DEFAULT",
            CMD_SET_ALERT_THRESHOLD,
            bytes([75, 2, 4]),
        ),
        CommandVector(
            "SET_ALERT_THRESHOLD_MIN",
            CMD_SET_ALERT_THRESHOLD,
            bytes([0, 0, 0]),
        ),
        CommandVector(
            "SET_ALERT_THRESHOLD_MAX",
            CMD_SET_ALERT_THRESHOLD,
            bytes([100, 100, 100]),
        ),
        CommandVector(
            "SET_ALERT_THRESHOLD_BAD_RANGE",
            CMD_SET_ALERT_THRESHOLD,
            bytes([101, 2, 4]),
        ),
        CommandVector(
            "SET_ALERT_THRESHOLD_BAD_LEN",
            CMD_SET_ALERT_THRESHOLD,
            bytes([75, 2]),
        ),
        CommandVector("GET_DEVICE_INFO", CMD_GET_DEVICE_INFO, b""),
        CommandVector("GET_NIGHT_SUMMARY", CMD_GET_NIGHT_SUMMARY, b""),
        CommandVector("CLEAR_NIGHT_SUMMARY", CMD_CLEAR_NIGHT_SUMMARY, b""),
        CommandVector("SET_LOG_LEVEL", CMD_SET_LOG_LEVEL, b""),
        CommandVector("UNKNOWN_VALID_CRC", 0xFF, b""),
    ]


def decode_command(data: bytes) -> tuple[int, dict[str, int]]:
    if len(data) < COMMAND_FRAME_MIN_LEN:
        return ERR_INVALID_ARG, {}
    if data[0] != MAGIC or data[1] != PROTO or data[2] != MSG_COMMAND:
        return ERR_INVALID_ARG, {}
    payload_len = data[10]
    if payload_len > COMMAND_MAX_PAYLOAD_LEN:
        return ERR_INVALID_ARG, {}
    if len(data) != COMMAND_FRAME_MIN_LEN + payload_len:
        return ERR_INVALID_ARG, {}
    if data[-1] != crc8_atm(data[:-1]):
        return ERR_CRC_FAILED, {}
    command_id = data[9]
    if not (CMD_PING <= command_id <= CMD_SET_LOG_LEVEL):
        return ERR_CMD_UNSUPPORTED, {}
    return ERR_OK, {
        "seq": data[3],
        "ts_s": int.from_bytes(data[4:8], "little"),
        "command_id": command_id,
        "payload_len": payload_len,
    }


def decode_cmd_response(data: bytes) -> dict[str, object]:
    if len(data) < CMD_RESPONSE_FRAME_MIN_LEN:
        raise ValueError("response frame too short")
    if data[0] != MAGIC:
        raise ValueError("invalid magic")
    if data[1] != PROTO:
        raise ValueError("invalid proto")
    if data[2] != MSG_COMMAND_RESPONSE:
        raise ValueError("invalid msg_type")
    if data[-1] != crc8_atm(data[:-1]):
        raise ValueError(
            f"crc mismatch: got 0x{data[-1]:02x}, expected 0x{crc8_atm(data[:-1]):02x}"
        )

    payload_len = data[11]
    expected_len = CMD_RESPONSE_FRAME_MIN_LEN + payload_len
    if len(data) != expected_len:
        raise ValueError(f"invalid response length: got {len(data)}, expected {expected_len}")

    payload = data[12 : 12 + payload_len]
    decoded: dict[str, object] = {
        "seq": data[3],
        "ts_s": int.from_bytes(data[4:8], "little"),
        "frame_error": data[8],
        "command_id": data[9],
        "command_name": command_name(data[9]),
        "status": data[10],
        "status_name": status_name(data[10]),
        "payload_len": payload_len,
        "payload_hex": payload.hex(" "),
        "crc": data[-1],
        "total_len": len(data),
    }

    if data[9] == CMD_GET_DEVICE_INFO and payload_len == DEVICE_INFO_PAYLOAD_LEN_V2:
        if len(data) != ATT_NOTIFY_PAYLOAD_MAX_DEFAULT:
            raise ValueError("GET_DEVICE_INFO response must be 20 bytes at default MTU")
        decoded["device_info"] = decode_device_info_payload(payload)

    return decoded


def decode_device_info_payload(payload: bytes) -> dict[str, object]:
    if len(payload) != DEVICE_INFO_PAYLOAD_LEN_V2:
        raise ValueError("GET_DEVICE_INFO payload must be 7 bytes")
    return {
        "proto_ver": payload[0],
        "capability_flags": payload[1],
        "capabilities": capability_names(payload[1]),
        "runtime_flags": payload[2],
        "runtime": runtime_names(payload[2]),
        "cough_threshold": payload[3],
        "warning_threshold": payload[4],
        "high_threshold": payload[5],
        "reserved": payload[6],
    }


def parse_hex_bytes(text: str) -> bytes:
    cleaned = text.strip().replace("0x", "").replace("0X", "")
    cleaned = re.sub(r"[\s:,_-]+", "", cleaned)
    if len(cleaned) % 2 != 0:
        raise ValueError("hex string has an odd number of digits")
    if not re.fullmatch(r"[0-9a-fA-F]*", cleaned):
        raise ValueError("hex string contains non-hex characters")
    return bytes.fromhex(cleaned)


def stage2_command_status(command_id: int, payload: bytes = b"") -> int:
    if not (CMD_PING <= command_id <= CMD_SET_LOG_LEVEL):
        return ERR_CMD_UNSUPPORTED
    if command_id in (
        CMD_PING,
        CMD_START_MONITOR,
        CMD_STOP_MONITOR,
        CMD_GET_DEVICE_INFO,
    ):
        return ERR_OK
    if command_id == CMD_TIME_SYNC:
        if len(payload) != 4:
            return ERR_INVALID_ARG
        epoch_s = int.from_bytes(payload, "little")
        return ERR_OK if 1577836800 <= epoch_s <= 4102444800 else ERR_INVALID_ARG
    if command_id == CMD_SET_ALERT_THRESHOLD:
        if len(payload) != 3:
            return ERR_INVALID_ARG
        return ERR_OK if all(value <= 100 for value in payload) else ERR_INVALID_ARG
    if command_id == CMD_CLEAR_NIGHT_SUMMARY:
        return ERR_CMD_DENIED
    return ERR_CMD_NOT_READY


def status_name(status: int) -> str:
    return {
        ERR_OK: "OK",
        ERR_INVALID_ARG: "INVALID_ARG",
        ERR_CRC_FAILED: "CRC_FAILED",
        ERR_CMD_UNSUPPORTED: "CMD_UNSUPPORTED",
        ERR_CMD_DENIED: "CMD_DENIED",
        ERR_CMD_NOT_READY: "CMD_NOT_READY",
        ERR_BUSY: "BUSY",
    }.get(status, f"ERR_{status}")


def command_name(command_id: int) -> str:
    return {
        CMD_PING: "PING",
        CMD_TIME_SYNC: "TIME_SYNC",
        CMD_START_MONITOR: "START_MONITOR",
        CMD_STOP_MONITOR: "STOP_MONITOR",
        CMD_SET_ALERT_THRESHOLD: "SET_ALERT_THRESHOLD",
        CMD_GET_DEVICE_INFO: "GET_DEVICE_INFO",
        CMD_GET_NIGHT_SUMMARY: "GET_NIGHT_SUMMARY",
        CMD_CLEAR_NIGHT_SUMMARY: "CLEAR_NIGHT_SUMMARY",
        CMD_SET_LOG_LEVEL: "SET_LOG_LEVEL",
    }.get(command_id, f"UNKNOWN_0x{command_id:02x}")


def capability_names(flags: int) -> list[str]:
    names = []
    if flags & CAPABILITY_FLAG_ENABLE:
        names.append("APP_BLE_ENABLE")
    if flags & CAPABILITY_FLAG_FAKE_DATA:
        names.append("APP_BLE_FAKE_DATA_ENABLE")
    if flags & CAPABILITY_FLAG_STACK:
        names.append("APP_BLE_STACK_ENABLE")
    if flags & CAPABILITY_FLAG_DIAG:
        names.append("APP_BLE_DIAG_ENABLE")
    return names


def runtime_names(flags: int) -> list[str]:
    names = []
    if flags & RUNTIME_FLAG_MONITOR_REQUESTED:
        names.append("monitor_requested")
    if flags & RUNTIME_FLAG_TIME_SYNCED:
        names.append("time_synced")
    return names


def default_device_info_payload() -> bytes:
    return pack_device_info_payload(
        stack_debug_capability_flags(),
        runtime_flags=0,
        cough_threshold=75,
        warning_threshold=2,
        high_threshold=4,
    )


def started_max_device_info_payload() -> bytes:
    return pack_device_info_payload(
        stack_debug_capability_flags(),
        runtime_flags=RUNTIME_FLAG_MONITOR_REQUESTED,
        cough_threshold=100,
        warning_threshold=100,
        high_threshold=100,
    )


def started_synced_max_device_info_payload() -> bytes:
    return pack_device_info_payload(
        stack_debug_capability_flags(),
        runtime_flags=RUNTIME_FLAG_MONITOR_REQUESTED | RUNTIME_FLAG_TIME_SYNCED,
        cough_threshold=100,
        warning_threshold=100,
        high_threshold=100,
    )


def default_device_info_response() -> bytes:
    return pack_cmd_response(
        7, 123456, CMD_GET_DEVICE_INFO, ERR_OK, default_device_info_payload()
    )


def started_synced_max_device_info_response() -> bytes:
    return pack_cmd_response(
        7,
        123456,
        CMD_GET_DEVICE_INFO,
        ERR_OK,
        started_synced_max_device_info_payload(),
    )


def run_assertions() -> None:
    realtime = pack_realtime()
    event = pack_event()
    ping = pack_command(0, 123456, CMD_PING)
    default_response = default_device_info_response()

    assert len(realtime) == REALTIME_FRAME_LEN
    assert len(event) == EVENT_FRAME_LEN
    assert len(default_device_info_payload()) == DEVICE_INFO_PAYLOAD_LEN_V2
    assert (
        CMD_RESPONSE_FRAME_MIN_LEN + DEVICE_INFO_PAYLOAD_LEN_V2
        <= ATT_NOTIFY_PAYLOAD_MAX_DEFAULT
    )
    assert len(default_response) == ATT_NOTIFY_PAYLOAD_MAX_DEFAULT
    assert decode_cmd_response(default_response)["payload_len"] == DEVICE_INFO_PAYLOAD_LEN_V2
    assert realtime.hex(" ") == EXPECTED_REALTIME_HEX
    assert event.hex(" ") == EXPECTED_EVENT_HEX
    assert ping.hex(" ") == EXPECTED_PING_HEX
    assert crc8_atm(b"123456789") == 0xF4

    bad_magic = bytearray(ping)
    bad_magic[0] = 0x00
    bad_proto = bytearray(ping)
    bad_proto[1] = 0x02
    bad_crc = bytearray(ping)
    bad_crc[-1] ^= 0x01

    assert decode_command(bytes(bad_magic))[0] == ERR_INVALID_ARG
    assert decode_command(bytes(bad_proto))[0] == ERR_INVALID_ARG
    assert decode_command(bytes(bad_crc))[0] == ERR_CRC_FAILED
    assert decode_command(pack_command(0, 123456, 0xFF))[0] == ERR_CMD_UNSUPPORTED


def print_summary() -> None:
    run_assertions()
    print("BLE protocol golden checks: PASS")
    print(f"realtime golden hex: {pack_realtime().hex(' ')}")
    print(f"event golden hex:    {pack_event().hex(' ')}")
    print(f"ping command hex:    {pack_command(0, 123456, CMD_PING).hex(' ')}")
    print("crc8_atm('123456789'): f4")
    print(
        "GET_DEVICE_INFO default response len="
        f"{len(default_device_info_response())} "
        f"hex={default_device_info_response().hex(' ')}"
    )
    print(
        "GET_DEVICE_INFO START+TIME_SYNC+MAX response len="
        f"{len(started_synced_max_device_info_response())} "
        f"hex={started_synced_max_device_info_response().hex(' ')}"
    )


def print_commands() -> None:
    run_assertions()
    print("nRF Connect command hex:")
    for vector in command_vectors():
        encoded = pack_command(7, 123456, vector.command_id, vector.payload)
        decode_status, decoded = decode_command(encoded)
        handle_status = stage2_command_status(vector.command_id, vector.payload)
        print(f"  {vector.name}: {encoded.hex(' ')}")
        print(
            "    decode="
            f"{status_name(decode_status)} handle={status_name(handle_status)} "
            f"fields={decoded}"
        )

    ping_bad_crc = bytearray(pack_command(0, 123456, CMD_PING))
    ping_bad_crc[-1] ^= 0x01
    print(f"  PING_BAD_CRC: {bytes(ping_bad_crc).hex(' ')}")
    print("GET_DEVICE_INFO response examples:")
    print(f"  DEFAULT: {default_device_info_response().hex(' ')}")
    print(f"  START_TIME_SYNC_MAX: {started_synced_max_device_info_response().hex(' ')}")


def print_decoded_response(hex_text: str) -> None:
    run_assertions()
    data = parse_hex_bytes(hex_text)
    decoded = decode_cmd_response(data)
    print("Command Response decode:")
    for key in (
        "total_len",
        "seq",
        "ts_s",
        "frame_error",
        "command_id",
        "command_name",
        "status",
        "status_name",
        "payload_len",
        "payload_hex",
        "crc",
    ):
        print(f"  {key}: {decoded[key]}")

    if "device_info" in decoded:
        print("GET_DEVICE_INFO payload v2:")
        device_info = decoded["device_info"]
        assert isinstance(device_info, dict)
        for key, value in device_info.items():
            print(f"  {key}: {value}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="BLE Stage 2 golden-vector and nRF Connect helper"
    )
    parser.add_argument(
        "--print-commands",
        action="store_true",
        help="print all nRF Connect command hex vectors",
    )
    parser.add_argument(
        "--decode-response",
        metavar="HEX",
        help="decode one Command Response notify frame",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.decode_response:
        print_decoded_response(args.decode_response)
    elif args.print_commands:
        print_commands()
    else:
        print_summary()


if __name__ == "__main__":
    main()
