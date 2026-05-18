# BLE Stage 2 Smoke Report

Date: 2026-05-17
Firmware HEAD observed during Stage 2D planning: `837d4bd`
Working tree state: BLE Stage 2 development changes are present.

Build macros used for board smoke:

```text
APP_AUDIO_MODEL_SELECT=3
APP_BLE_ENABLE=1
APP_BLE_FAKE_DATA_ENABLE=1
APP_BLE_STACK_ENABLE=1
APP_BLE_DEBUG_DISABLE_INFERENCE=1
```

Stage 1 regression build macro set:

```text
APP_AUDIO_MODEL_SELECT=3
APP_BLE_ENABLE=1
APP_BLE_FAKE_DATA_ENABLE=1
APP_BLE_STACK_ENABLE=0
```

## Summary

Stage 2 validates BLE stack bring-up, advertising, static GATT discovery,
Realtime/Event notify, Command Write, Command Response notify, and BLE-local
shadow state. Stage 2 does not validate real model result, real radar summary,
fusion summary, PDM/CM55/radar realtime behavior, shared memory, BLE
fragmentation, or mobile app UI.

## Smoke Matrix

| Item | Status | Evidence |
| --- | --- | --- |
| Advertising scan | PASS | nRF Connect scanned `E84-HealthMonitor`; UART showed `[BLE_ADV] started name=E84-HealthMonitor`. |
| BLE connect | PASS | nRF Connect connected; UART showed `[BLE_CONN] connected conn_id=32768`. |
| Disconnect and advertising restart | PASS | Earlier Stage 2A smoke showed disconnect followed by advertising restart. Reconfirm before Stage 3 if needed. |
| Health Monitor Service discovery | PASS | nRF Connect discovered service `7E840000-1A2B-4C3D-9E10-000000000001`. |
| Four characteristic discovery | PASS | nRF Connect discovered Realtime, Alert Event, Command Write, and Command Response characteristics. |
| Realtime notify | PASS | Realtime notify observed at 1 Hz with 19-byte frames; `notify_ok` increased and `notify_fail=0`. |
| Alert Event notify | PASS | Event notify observed about every 15 s with 19-byte frames; no transport errors reported. |
| Command Write raw queue | PASS | UART showed `[BLE_CMD_RAW] ...`; `cmd_drop=0`. |
| PING response | PASS | UART showed `[BLE_CMD] id=PING result=OK` and `[BLE_CMD_RSP] notify ok len=13`. |
| Command matrix | PASS | PING/TIME_SYNC/START/STOP/SET_ALERT_THRESHOLD/GET_DEVICE_INFO/error paths were exercised through nRF Connect. |
| START/STOP shadow state | PASS | UART showed `START_MONITOR result=OK monitor=1`; stats showed `monitor=1`. |
| SET_ALERT_THRESHOLD shadow config | PASS | UART showed `SET_ALERT_THRESHOLD result=OK cough=100 warn=100 high=100`; stats showed thresholds updated. |
| GET_DEVICE_INFO v2 | PASS | UART showed `[BLE_CMD] id=GET_DEVICE_INFO result=OK` and `[BLE_CMD_RSP] notify ok len=20`; phone payload matched `01 0f 03 64 64 64 00`. |
| Command Response not subscribed path | NOT_TESTED | TODO: write `GET_DEVICE_INFO` without subscribing Command Response and confirm only `cmd_rsp_no_sub` increases. |
| 10 repeated GET_DEVICE_INFO reads | TODO | TODO: run before Stage 3 and confirm `cmd_rsp_tx_busy` does not stick. |
| Real model result summary | N/A | Stage 3 scope. |
| Real radar summary | N/A | Stage 3 scope. |
| Fusion summary | N/A | Stage 3 scope. |
| BLE fragmentation | N/A | Stage 2 explicitly does not fragment. |
| Mobile app UI | N/A | Stage 2 uses nRF Connect. |

## Key UART Evidence

```text
[BLE_CORE] stack enabled
[BLE_GATT] db init ok
[BLE_ADV] started name=E84-HealthMonitor
[BLE_CONN] connected conn_id=32768
[BLE_CMD] id=GET_DEVICE_INFO result=OK
[BLE_CMD_RSP] notify ok len=20
```

Latest user-reported healthy stats included:

```text
notify_fail=0
transport_err=0
cmd_rsp_fail=0
cmd_drop=0
cmd_rsp_tx_busy=0
monitor=1
thr_cough=100
thr_warn=100
thr_high=100
```

## Known Limits

- Realtime and Event payloads are fake Stage 2 data.
- START/STOP and thresholds are BLE-local shadow state only.
- GET_DEVICE_INFO reports BLE-local state only, not true hardware health.
- Default ATT MTU is treated as 23; notify payload max is 20 bytes.
- No BLE fragmentation is implemented.
- Night summary, SET_LOG_LEVEL, real fusion, real model/radar summary, and app UI are not part of Stage 2.

## Commands

Protocol helper:

```powershell
cd D:\e84_health_monitor\firmware
python tools\ble_protocol_golden.py
python tools\ble_protocol_golden.py --print-commands
```

Build:

```powershell
cd D:\e84_health_monitor\firmware
$env:PATH = 'D:\modustoolbox\ModusToolbox\tools_3.7\modus-shell\bin;' + $env:PATH
D:\modustoolbox\ModusToolbox\tools_3.7\modus-shell\bin\make.exe build OS=Windows_NT APP_AUDIO_MODEL_SELECT=3 APP_BLE_ENABLE=1 APP_BLE_FAKE_DATA_ENABLE=1 APP_BLE_STACK_ENABLE=1 APP_BLE_DEBUG_DISABLE_INFERENCE=1
D:\modustoolbox\ModusToolbox\tools_3.7\modus-shell\bin\make.exe build OS=Windows_NT APP_AUDIO_MODEL_SELECT=3 APP_BLE_ENABLE=1 APP_BLE_FAKE_DATA_ENABLE=1 APP_BLE_STACK_ENABLE=0
```

Program, only when board smoke needs to be refreshed:

```powershell
D:\modustoolbox\ModusToolbox\tools_3.7\modus-shell\bin\make.exe program OS=Windows_NT APP_AUDIO_MODEL_SELECT=3 APP_BLE_ENABLE=1 APP_BLE_FAKE_DATA_ENABLE=1 APP_BLE_STACK_ENABLE=1 APP_BLE_DEBUG_DISABLE_INFERENCE=1
```
