# BLE Stage 2 Protocol Contract

Date: 2026-05-17
Firmware HEAD observed during Stage 2D planning: `837d4bd`

This document freezes the firmware-local BLE Stage 2 protocol that has been
validated with nRF Connect. Stage 2 is a monitoring and lightweight control
layer only. It does not carry raw PCM, raw radar bytes, OTA payloads, cloud
sync data, or medical diagnosis state.

## GATT Layout

Device name:

```text
E84-HealthMonitor
```

Health Monitor Service:

```text
7E840000-1A2B-4C3D-9E10-000000000001
```

Characteristics:

| Characteristic | UUID | Property | Value handle | CCCD handle |
| --- | --- | --- | --- | --- |
| Realtime Status | `7E840002-1A2B-4C3D-9E10-000000000001` | Notify | `0x0102` | `0x0103` |
| Alert Event | `7E840003-1A2B-4C3D-9E10-000000000001` | Notify | `0x0105` | `0x0106` |
| Command Write | `7E840004-1A2B-4C3D-9E10-000000000001` | Write | `0x0108` | N/A |
| Command Response | `7E840005-1A2B-4C3D-9E10-000000000001` | Notify | `0x010A` | `0x010B` |

Service handle:

```text
0x0100
```

Characteristic declaration handles:

```text
Realtime declaration:        0x0101
Alert Event declaration:     0x0104
Command declaration:         0x0107
Command Response declaration:0x0109
```

UUID byte order in the static WICED GATT database is ATT little-endian. nRF
Connect should display the UUIDs in the canonical `7E84xxxx-...` form above.

## Frame Contract

All multibyte values are little-endian. All frames end with CRC-8/ATM over all
bytes before the CRC byte.

CRC parameters:

```text
Name: CRC-8/ATM
Polynomial: 0x07
Initial value: 0x00
Check vector: crc8("123456789") = 0xF4
```

Common header:

| Byte | Field |
| --- | --- |
| 0 | magic `0xE8` |
| 1 | protocol version `0x01` |
| 2 | message type |
| 3 | sequence |
| 4..7 | timestamp seconds |
| 8 | error/status field |

Message types:

| Message | `msg_type` | Length |
| --- | --- | --- |
| Realtime Status | `0x10` | 19 bytes |
| Alert Event | `0x20` | 19 bytes |
| Command Write | `0x30` | `12 + payload_len` bytes |
| Command Response | `0x31` | `13 + payload_len` bytes |

Default ATT MTU is 23. The Stage 2 notify payload limit is therefore
`mtu - 3 = 20` bytes. Stage 2 does not implement fragmentation. Oversize
Realtime/Event/Command Response frames are rejected or dropped according to the
path-specific diagnostics counters.

## Command Matrix

All commands are handled in the BLE stream task context after the GATT write
callback copies raw bytes into a fixed queue. No command directly touches real
PDM, CM55, radar, model inference, fusion, or shared memory.

| Command | Result | Side effect | Real hardware touched |
| --- | --- | --- | --- |
| `PING` | `OK` | None | No |
| `TIME_SYNC` | `OK` or `INVALID_ARG` | Stores BLE-local time offset and `time_synced` flag only | No |
| `START_MONITOR` | `OK` | Sets BLE-local `monitor_requested=1` only | No |
| `STOP_MONITOR` | `OK` | Sets BLE-local `monitor_requested=0` only | No |
| `SET_ALERT_THRESHOLD` | `OK` or `INVALID_ARG` | Saves 3-byte BLE-local RAM shadow config only | No |
| `GET_DEVICE_INFO` | `OK` | Returns 7-byte BLE-local status payload v2 | No |
| `GET_NIGHT_SUMMARY` | `CMD_NOT_READY` | None | No |
| `CLEAR_NIGHT_SUMMARY` | `CMD_DENIED` | None | No |
| `SET_LOG_LEVEL` | `CMD_NOT_READY` | None | No |
| Unknown command with valid CRC | `CMD_UNSUPPORTED` | None | No |
| Bad CRC | `CRC_FAILED` | Counted as command CRC error; response is sent only when enough header/id context is available | No |

`SET_ALERT_THRESHOLD` payload v1 is exactly 3 bytes:

| Byte | Field | Range | Default |
| --- | --- | --- | --- |
| 0 | cough probability threshold | 0..100 | 75 |
| 1 | warning event threshold | 0..100 | 2 |
| 2 | high-risk event threshold | 0..100 | 4 |

The shadow threshold config is RAM-only and resets to defaults after reboot.

## GET_DEVICE_INFO v2 Payload

`GET_DEVICE_INFO` returns `OK` with a fixed 7-byte payload. The full Command
Response notify frame length is `13 + 7 = 20` bytes at default MTU.

| Byte | Field |
| --- | --- |
| 0 | `proto_ver` |
| 1 | `capability_flags` |
| 2 | `runtime_flags` |
| 3 | `cough_prob_threshold` |
| 4 | `warning_event_threshold` |
| 5 | `high_risk_event_threshold` |
| 6 | reserved, always `0x00` |

`capability_flags`:

| Bit | Meaning |
| --- | --- |
| 0 | `APP_BLE_ENABLE` |
| 1 | `APP_BLE_FAKE_DATA_ENABLE` |
| 2 | `APP_BLE_STACK_ENABLE` |
| 3 | `APP_BLE_DIAG_ENABLE` |

`runtime_flags`:

| Bit | Meaning |
| --- | --- |
| 0 | `monitor_requested` |
| 1 | `time_synced` |

Examples:

```text
Default:
01 0f 00 4b 02 04 00

START_MONITOR + TIME_SYNC + SET_ALERT_THRESHOLD_MAX:
01 0f 03 64 64 64 00
```

`GET_DEVICE_INFO OK` only means the BLE-local status query succeeded. It does
not indicate true PDM, CM55, radar, model, fusion, or shared-memory health.

## Stage 2 Boundaries

Stage 2 uses fake realtime/event samples and BLE-local shadow command state.
It is safe for nRF Connect and mobile-app protocol bring-up, but it is not yet
the real health-monitor summary path. Real model/radar/fusion summaries are
reserved for Stage 3.
