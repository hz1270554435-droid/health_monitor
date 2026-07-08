# Stable Build Configuration — 2026-07-08

Tag: `stable-display-inference-ble-20260708`

## Overview

Full-feature firmware: CM55 LVGL touch multi-page UI + Hz2 audio inference + BLE + radar bridge.

Based on Hz2 stable baseline (`1550f12`) with display updates cherry-picked from main.

## Build Commands

Requires ModusToolbox shell. Run from repo root:

```bash
/d/modustoolbox/ModusToolbox/tools_3.7/modus-shell/bin/bash.exe -lc '
set -e
cd /cygdrive/d/modustoolbox/project_shuimianhuxi_workspace/PSOC_Edge_PDM_to_I2S

# CM33_S (secure boot)
make -C proj_cm33_s build_proj OS=Windows_NT \
  MTB_APPLICATION_SUBPROJECTS="proj_cm33_s proj_cm33_ns proj_cm55" \
  MTB_APPLICATION_NAME="PSOC_Edge_PDM_to_I2S" MTB_APPLICATION_PROMOTE=true

# CM33_NS (audio pipeline + BLE + display bridge + radar)
make -C proj_cm33_ns build_proj OS=Windows_NT \
  APP_BUILD_PROFILE=legacy_manual \
  APP_DISPLAY_ENABLE=1 \
  APP_DISPLAY_SUMMARY_ENABLE=1 \
  APP_MONITOR_SUMMARY_ENABLE=1 \
  APP_DISPLAY_CM55_SNAPSHOT_BRIDGE_ENABLE=1 \
  APP_DISPLAY_CM55_SNAPSHOT_BRIDGE_DIAG_ENABLE=1 \
  APP_RADAR_BRIDGE_ENABLE=1 \
  APP_MODEL_RESULT_MONITOR_ENABLE=1 \
  APP_BLE_ENABLE=1 \
  APP_BLE_STACK_ENABLE=1 \
  APP_BLE_SUMMARY_ENABLE=1 \
  APP_AUDIO_MODEL_SELECT=7 \
  APP_MODEL_EVENT_THRESHOLD=0.70f \
  MTB_APPLICATION_SUBPROJECTS="proj_cm33_s proj_cm33_ns proj_cm55" \
  MTB_APPLICATION_NAME="PSOC_Edge_PDM_to_I2S" MTB_APPLICATION_PROMOTE=true

# CM55 (display LVGL + touch + inference)
make -C proj_cm55 build_proj OS=Windows_NT \
  APP_BUILD_PROFILE=legacy_manual \
  APP_DISPLAY_OFFICIAL_CM55_BRINGUP_ENABLE=1 \
  APP_DISPLAY_LVGL_ENABLE=1 \
  APP_DISPLAY_LVGL_TOUCH_ENABLE=1 \
  APP_DISPLAY_CM55_SNAPSHOT_BRIDGE_ENABLE=1 \
  APP_CM55_INFERENCE_ENABLE=1 \
  APP_AUDIO_MODEL_SELECT=7 \
  MTB_APPLICATION_SUBPROJECTS="proj_cm33_s proj_cm33_ns proj_cm55" \
  MTB_APPLICATION_NAME="PSOC_Edge_PDM_to_I2S" MTB_APPLICATION_PROMOTE=true

# Post-build (combine hex)
make -C proj_cm33_s application_postbuild \
  MTB_APPLICATION_SUBPROJECTS="proj_cm33_s proj_cm33_ns proj_cm55"

# Program
make -C proj_cm33_s qprogram_proj \
  MTB_APPLICATION_SUBPROJECTS="proj_cm33_s proj_cm33_ns proj_cm55" \
  MTB_APPLICATION_NAME="PSOC_Edge_PDM_to_I2S"
'
```

## Key Configuration

| Parameter | Value | Notes |
|-----------|-------|-------|
| `APP_AUDIO_MODEL_SELECT` | 7 | Hz2 (hz2mix_bssilver_bw1_1w) |
| `APP_MODEL_EVENT_THRESHOLD` | 0.70f | Matches Hz2 recommended threshold |
| `APP_MODEL_EVENT_MIN_HITS` | 2 | Default |
| `APP_MODEL_EVENT_WINDOW_MS` | 1200 | Default |
| `APP_CM55_INFERENCE_ENABLE` | 1 | Model inference on CM55 |
| `APP_DISPLAY_LVGL_ENABLE` | 1 | LVGL dashboard |
| `APP_DISPLAY_LVGL_TOUCH_ENABLE` | 1 | FT5406 touch, 4-page swipe UI |
| `APP_BLE_ENABLE` | 1 | BLE data export |
| `APP_BLE_STACK_ENABLE` | 1 | BLE stack |
| `APP_RADAR_BRIDGE_ENABLE` | 1 | LD6002 radar bridge |
| `APP_BUILD_PROFILE` | legacy_manual | For both CM33_NS and CM55 |

## Important: Do NOT use `display_bringup` profile

The `display_bringup` profile contains `override APP_CM55_INFERENCE_ENABLE:=0` which
forcibly disables model inference. Use `legacy_manual` for both cores and set display
flags manually.

## Verified Features

- [x] CM55 LVGL health dashboard (4 pages: Home, RR, HR, Night Cough)
- [x] Touch swipe navigation (FT5406)
- [x] 180° display rotation
- [x] Hz2 audio model inference (cough/non_cough)
- [x] Cough event detection (threshold 0.70, 2 hits / 1.2s)
- [x] BLE device discoverable
- [x] Radar bridge (LD6002 breath rate, heart rate)
- [x] Display snapshot bridge (CM33 → CM55)
