# Board Debug Policy

These rules apply to PSoC Edge E84 board debugging.

## Platform

- Board: PSoC Edge E84 / `KIT_PSE84_EVAL` or `KIT_PSE84_AI`.
- Cores: CM33 secure, CM33 non-secure, CM55.
- Accelerator path: Ethos-U55 / NPU where supported by the model/runtime.
- OS/runtime: ModusToolbox, FreeRTOS, TFLite Micro / ML middleware.

## Debug Priorities

- Reproduce the symptom with the smallest command or UART capture.
- Save key logs, not massive unfiltered dumps.
- Distinguish build failure, boot failure, capture failure, preprocessing failure, inference failure, and result-reporting failure.
- Compare PC and board preprocessing before blaming the model.

## Shared Memory / IPC

- Treat CM33/CM55 shared memory as high risk.
- Check state transitions, sequence counters, result slots, timeouts, and buffer sizes.
- Cache consistency and memory barriers require architecture review.
- Do not change shared-memory layout from one core only.

## UART Logs

- Keep baud rate and stream type explicit.
- Avoid mixing binary frames and text logs unless the capture tool expects it.
- For radar, separate radar UART input from retarget/debug UART output.

## Smoke Tests

- Firmware changes must include a build or smoke-test recommendation.
- Board smoke tests should state expected UART markers or observed result fields.
- A model smoke test proves integration only; it does not prove model performance.

## Real Commands Currently Seen

```powershell
make build
make program
tools\run_capture_csv.bat --list-ports
tools\run_capture_csv.bat --port COM7 --baud 2000000
```

Unknown commands must be marked TODO until verified.
