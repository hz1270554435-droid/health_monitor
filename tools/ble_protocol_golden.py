#!/usr/bin/env python3
"""Minimal BLE Stage 1 protocol golden-vector checks.

This script intentionally has no third-party dependencies and does not create
files. It mirrors the Stage 1 wire contract used by app_ble_protocol.c.
"""

from __future__ import annotations

MAGIC = 0xE8
PROTO = 0x01

MSG_REALTIME = 0x10
MSG_EVENT = 0x20
MSG_COMMAND = 0x30

ERR_OK = 0
ERR_INVALID_ARG = 5
ERR_CRC_FAILED = 7
ERR_CMD_UNSUPPORTED = 8
ERR_CMD_DENIED = 9
ERR_CMD_NOT_READY = 11

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

EXPECTED_REALTIME_HEX = (
    "e8 01 10 00 40 e2 01 00 00 10 4b 64 05 57 ff 02 02 9f 54"
)
EXPECTED_EVENT_HEX = (
    "e8 01 20 00 41 e2 01 00 00 01 00 00 00 01 02 57 03 05 32"
)
EXPECTED_PING_HEX = "e8 01 30 00 40 e2 01 00 00 01 00 48"


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
    body = header(MSG_COMMAND, seq=seq, ts_s=ts_s) + bytes([command_id, len(payload)]) + payload
    return frame(body)


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


def stage1_command_status(command_id: int, payload: bytes = b"") -> int:
    if not (CMD_PING <= command_id <= CMD_SET_LOG_LEVEL):
        return ERR_CMD_UNSUPPORTED
    if command_id in (CMD_PING, CMD_GET_DEVICE_INFO):
        return ERR_OK
    if command_id == CMD_TIME_SYNC:
        if len(payload) != 4:
            return ERR_INVALID_ARG
        epoch_s = int.from_bytes(payload, "little")
        return ERR_OK if 1577836800 <= epoch_s <= 4102444800 else ERR_INVALID_ARG
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
    }.get(status, f"ERR_{status}")


def main() -> None:
    realtime = pack_realtime()
    event = pack_event()
    ping = pack_command(0, 123456, CMD_PING)

    assert len(realtime) == REALTIME_FRAME_LEN
    assert len(event) == EVENT_FRAME_LEN
    assert realtime.hex(" ") == EXPECTED_REALTIME_HEX
    assert event.hex(" ") == EXPECTED_EVENT_HEX
    assert ping.hex(" ") == EXPECTED_PING_HEX
    assert crc8_atm(b"123456789") == 0xF4

    print(f"realtime golden hex: {realtime.hex(' ')}")
    print(f"event golden hex:    {event.hex(' ')}")
    print(f"ping command hex:    {ping.hex(' ')}")
    print("crc8_atm('123456789'): f4")

    for name, cmd_id, payload in [
        ("PING", CMD_PING, b""),
        ("TIME_SYNC", CMD_TIME_SYNC, le32(1735689600)),
        ("START_MONITOR", CMD_START_MONITOR, b""),
        ("CLEAR_NIGHT_SUMMARY", CMD_CLEAR_NIGHT_SUMMARY, b""),
        ("UNKNOWN", 0xFF, b""),
    ]:
        encoded = pack_command(7, 123456, cmd_id, payload)
        decode_status, decoded = decode_command(encoded)
        handle_status = stage1_command_status(cmd_id, payload)
        print(
            "command decode case: "
            f"{name} decode={status_name(decode_status)} "
            f"handle={status_name(handle_status)} fields={decoded}"
        )

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
    print("invalid cases: magic/proto/crc/cmd rejected")


if __name__ == "__main__":
    main()
