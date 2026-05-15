# BLE Stage 1 Notes

Stage 1 is firmware-only and intentionally does not add Bluetooth middleware,
GATT tables, HCI code, mobile-app code, or top-level workspace contracts.

## Build Macros

- `APP_BLE_ENABLE=1`
- `APP_BLE_FAKE_DATA_ENABLE=1`
- `APP_BLE_STACK_ENABLE=0`

With those macros, CM33_NS creates a low-priority fake BLE stream task that
prints protocol summaries over the existing debug UART.

## Windows Build Environment

Build from the firmware repository root:

```powershell
cd D:\e84_health_monitor\firmware
```

Before invoking ModusToolbox `make.exe`, ensure the ModusToolbox shell is first
in `PATH`:

```powershell
$env:PATH = 'D:\modustoolbox\ModusToolbox\tools_3.7\modus-shell\bin;' + $env:PATH
```

This is required because a different MSYS/MINGW shell earlier in `PATH` can make
GNU make run under that shell instead of the ModusToolbox Cygwin shell. In that
case `cygpath` is unavailable, `/cygdrive/...` paths can reach the Windows
`mtbninja.exe`, and `mtbninja` may incorrectly report that `proj_cm33_s` does
not contain `Makefile`.

Expected environment checks after the `PATH` update:

```powershell
where make
where sh
where cygpath
D:\modustoolbox\ModusToolbox\tools_3.7\modus-shell\bin\bash.exe -lc 'uname; which cygpath'
```

`make`, `sh`, and `cygpath` should resolve first under
`D:\modustoolbox\ModusToolbox\tools_3.7\modus-shell\bin`, and `uname` should
report `CYGWIN_NT...`.

## Stage 1 Scope

- Pack realtime frames.
- Pack event frames.
- Decode command frames.
- Generate command responses.
- Run fake realtime/event samples through the queue.
- Keep all BLE behavior independent from PDM capture, radar UART, CM55
  inference, and shared memory.

## Deferred

- Real AIROC/WICED BLE stack.
- GATT database.
- nRF Connect validation.
- Real model/radar/fusion summaries.
- Top-level `shared/contracts/` sync.
