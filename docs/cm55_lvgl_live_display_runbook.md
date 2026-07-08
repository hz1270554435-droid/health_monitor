# CM55 LVGL Live Display Build and Flash Runbook

Date: 2026-07-08 (updated)

## Purpose

The CM55 LCD dashboard uses the CM55 display bring-up/LVGL path, while live
values are produced on CM33_NS and published through the CM33 -> CM55 display
snapshot bridge. Build these two sides with different profiles.

Use this runbook when flashing the live LCD dashboard after CM55 display changes
such as framebuffer rotation, LVGL port updates, touch input, or dashboard UI
changes.

## Important pitfall

Do **not** pass `APP_BUILD_PROFILE=display_bringup` as a single global top-level
`make program` argument for the live dashboard image.

That profile is correct for CM55 display bring-up, but when it is also applied to
CM33_NS it disables the CM33 live-data path, including display summary, monitor
summary, radar bridge, and the snapshot publisher. The visible symptom is:

- the screen layout and orientation are normal;
- all live numeric values stay as `--`.

**Additionally**, `display_bringup` contains `override APP_CM55_INFERENCE_ENABLE:=0`
which forcibly disables model inference even if you pass `APP_CM55_INFERENCE_ENABLE=1`
on the command line. Use `legacy_manual` for both cores when you need inference.

## Required profile split

### CM33_NS live-data producer

CM33_NS should use `legacy_manual` with the live-data path enabled:

- `APP_DISPLAY_ENABLE=1`
- `APP_DISPLAY_SUMMARY_ENABLE=1`
- `APP_MONITOR_SUMMARY_ENABLE=1`
- `APP_DISPLAY_CM55_SNAPSHOT_BRIDGE_ENABLE=1`
- `APP_RADAR_BRIDGE_ENABLE=1`
- `APP_MODEL_RESULT_MONITOR_ENABLE=1`

This keeps CM33_NS as the owner of sensor/summary collection and the producer of
`app_display_cm55_snapshot_t` in `m33_m55_shared`.

### CM55 LCD/LVGL consumer

CM55 should use `legacy_manual` with display, touch, inference, and snapshot
reading enabled:

- `APP_BUILD_PROFILE=legacy_manual`
- `APP_DISPLAY_OFFICIAL_CM55_BRINGUP_ENABLE=1`
- `APP_DISPLAY_LVGL_ENABLE=1`
- `APP_DISPLAY_LVGL_TOUCH_ENABLE=1`
- `APP_DISPLAY_CM55_SNAPSHOT_BRIDGE_ENABLE=1`
- `APP_CM55_INFERENCE_ENABLE=1`
- `APP_AUDIO_MODEL_SELECT=7`

## Build and flash command

Run from ModusToolbox `bash.exe` or from a shell that executes the commands
inside ModusToolbox bash. Directly invoking Modus `make.exe` from Git Bash can
produce incorrect `/cygdrive/...` path mapping and false `Makefile` lookup
failures.

```bash
/d/modustoolbox/ModusToolbox/tools_3.7/modus-shell/bin/bash.exe -lc '
set -e
cd /cygdrive/d/modustoolbox/project_shuimianhuxi_workspace/PSOC_Edge_PDM_to_I2S

make -C proj_cm33_s build_proj \
  OS=Windows_NT \
  MTB_APPLICATION_SUBPROJECTS="proj_cm33_s proj_cm33_ns proj_cm55" \
  MTB_APPLICATION_NAME="PSOC_Edge_PDM_to_I2S" \
  MTB_APPLICATION_PROMOTE=true

make -C proj_cm33_ns build_proj \
  OS=Windows_NT \
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
  MTB_APPLICATION_NAME="PSOC_Edge_PDM_to_I2S" \
  MTB_APPLICATION_PROMOTE=true

make -C proj_cm55 build_proj \
  OS=Windows_NT \
  APP_BUILD_PROFILE=legacy_manual \
  APP_DISPLAY_OFFICIAL_CM55_BRINGUP_ENABLE=1 \
  APP_DISPLAY_LVGL_ENABLE=1 \
  APP_DISPLAY_LVGL_TOUCH_ENABLE=1 \
  APP_DISPLAY_CM55_SNAPSHOT_BRIDGE_ENABLE=1 \
  APP_CM55_INFERENCE_ENABLE=1 \
  APP_AUDIO_MODEL_SELECT=7 \
  MTB_APPLICATION_SUBPROJECTS="proj_cm33_s proj_cm33_ns proj_cm55" \
  MTB_APPLICATION_NAME="PSOC_Edge_PDM_to_I2S" \
  MTB_APPLICATION_PROMOTE=true

make -C proj_cm33_s application_postbuild \
  MTB_APPLICATION_SUBPROJECTS="proj_cm33_s proj_cm33_ns proj_cm55"

make -C proj_cm33_s qprogram_proj \
  MTB_APPLICATION_SUBPROJECTS="proj_cm33_s proj_cm33_ns proj_cm55" \
  MTB_APPLICATION_NAME="PSOC_Edge_PDM_to_I2S"
'
```

## Expected build checks

CM33_NS ninja defines should include:

```text
APP_DISPLAY_ENABLE=1
APP_DISPLAY_SUMMARY_ENABLE=1
APP_MONITOR_SUMMARY_ENABLE=1
APP_DISPLAY_CM55_SNAPSHOT_BRIDGE_ENABLE=1
APP_RADAR_BRIDGE_ENABLE=1
APP_BLE_ENABLE=1
APP_MODEL_RESULT_MONITOR_ENABLE=1
APP_AUDIO_MODEL_SELECT=7
APP_MODEL_EVENT_THRESHOLD=0.70f
```

CM55 ninja defines should include:

```text
APP_DISPLAY_LVGL_ENABLE=1
APP_DISPLAY_LVGL_TOUCH_ENABLE=1
APP_DISPLAY_CM55_SNAPSHOT_BRIDGE_ENABLE=1
APP_DISPLAY_OFFICIAL_CM55_BRINGUP_ENABLE=1
APP_CM55_INFERENCE_ENABLE=1
APP_AUDIO_MODEL_SELECT=7
```

Expected symbols:

```text
# CM33_NS
app_display_cm55_bridge_init
app_display_summary_adapter_tick
app_monitor_summary_update_radar
app_radar_bridge_init

# CM55
app_cm55_display_rotate_180_rgb565_inplace
lv_port_disp_init
lv_port_indev_init
ui_health_dashboard_update
```

## Board smoke checklist

After programming:

1. LCD orientation is physically correct.
2. Dashboard layout is stable: no obvious flicker, tearing, diagonal artifacts,
   or double-rotation.
3. Live values no longer remain all `--`.
4. Radar-derived fields such as breath rate, heart rate, distance, and presence
   update when valid radar data is available.
5. Touch input responds to swipe gestures.
6. Page navigation works (swipe left/right to cycle through 4 pages).
7. Cough events trigger on microphone input.

A non-fatal OpenOCD message like the following can appear while programming:

```text
Error: couldn't bind gdb to socket on port 3333
```

If erase, program, and verify complete afterward, this is not a flash failure.

## If values are still `--`

Enable/inspect the CM55 dashboard diagnostics or snapshot probe and distinguish:

- invalid snapshot magic/version;
- sequence not moving;
- heartbeat not moving;
- heartbeat moving but radar/vital fields remain zero;
- CM33 publisher not built or not running;
- radar bridge not producing valid source data.

---

# CM55 LVGL Touch Multi-Page UI

Date: 2026-07-08

## Overview

The CM55 LVGL dashboard now supports **touch-based page navigation** with **4
pages** of health monitoring data. The FT5406 capacitive touch controller on the
Waveshare 4.3" LCD enables swipe gestures for page switching.

## Hardware

- **Display**: Waveshare 4.3" LCD (800x480, RGB565)
- **Touch controller**: FocalTech FT5406 (I2C)
- **Driver**: ModusToolbox `mtb_ctp_ft5406` component
- **LVGL input type**: `LV_INDEV_TYPE_POINTER`

## Touch Coordinate Mapping

Raw touch coordinates from FT5406 are inverted in both axes:

```c
x_mapped = 799 - raw_x;  // clamp to [0, 799]
y_mapped = 479 - raw_y;  // clamp to [0, 479]
```

Display visible area: 800x480 (with 832-wide frame, 32px horizontal offset).

## Swipe Gesture Detection

The touch driver implements custom swipe detection outside LVGL's gesture system.

**Recognition criteria:**
- `|dx| > 80 pixels` (minimum horizontal travel)
- `|dy| < 120 pixels` (maximum vertical deviation — must be mostly horizontal)

**Direction mapping:**
- `dx > 0` (swipe right) → `dir = +1` → previous page
- `dx < 0` (swipe left) → `dir = -1` → next page

**Dual navigation mechanism:**
1. **LVGL built-in gesture** (`LV_EVENT_GESTURE`): reads `lv_indev_get_gesture_dir()`
2. **Custom swipe callback** (`touch_swipe_cb`): from FT5406 read callback

Both mechanisms call `switch_page()` with cyclic wrap-around.

## Page Structure

### Page 0: Home (UI_PAGE_HOME)

Main dashboard with 2x2 card grid:

| Card | Title | Data | Status Badge |
|------|-------|------|--------------|
| Top-left | 呼吸率 (RR) | Current bpm | "平稳" / "暂无数据" |
| Top-right | 心率 (HR) | Current bpm | "正常" (≤100) / "偏高" (>100) |
| Bottom-left | 咳嗽事件 | Night count + 30min count | "检测到咳嗽" / "无明显咳嗽" |
| Bottom-right | 雷达 | Distance in cm | "已检测到人体" / "未检测到人体" |

**Additional elements:**
- Top-left capsule: "起夜N次" — wake-up count (radar distance > baseline + 50cm, confirmed by 2 absence hits within 30s)
- Top-right capsule: "实时监测中" — real-time monitoring indicator with green LED
- Top-center: clock (HH:MM, UTC+8, color-coded: green=BLE synced, amber=NVM loaded, dim=no time source)
- Footer: health trend text, diagnostic line, status LEDs (MIC, Radar, BLE)

### Page 1: Respiratory Rate Detail (UI_PAGE_RR_DETAIL)

- Title: "呼吸 近30min"
- Current RR value in large text with "bpm" unit
- State label: "实时中" (live) / "等待数据" (waiting)
- Smooth line chart (Catmull-Rom interpolation) — 30-minute history
- Statistics: Min, Max, Avg below chart
- Page indicator: "2/4"

### Page 2: Heart Rate Detail (UI_PAGE_HR_DETAIL)

- Title: "心率 近30min"
- Same layout as RR detail, rose/red accent color
- Page indicator: "3/4"

### Page 3: Cough Night Summary (UI_PAGE_NIGHT)

- Title: "整夜"
- Total night cough count (large amber number, "次")
- Recent 30-minute cough count
- Bar chart: 12 hourly buckets (0h-12h), amber bars
- "暂无咳嗽事件" placeholder when no events
- Page indicator: "4/4"

## History and Charting Constants

| Constant | Value | Purpose |
|----------|-------|---------|
| `HISTORY_SAMPLE_PERIOD_MS` | 30,000 ms (30s) | History sample interval for RR/HR |
| `HISTORY_SAMPLE_COUNT` | 60 | Ring buffer — 60 samples × 30s = 30 min |
| `HISTORY_RENDER_SUBDIV` | 4 | Catmull-Rom subdivision factor |
| `HISTORY_RENDER_MAX_POINTS` | 241 | Max render points: (60-1)×4 + 1 |
| `COUGH_RECENT_WINDOW_MS` | 1,800,000 ms (30 min) | "Recent" cough window |
| `COUGH_EVENT_RING_COUNT` | 128 | Cough event timestamp ring buffer |
| `COUGH_NIGHT_BUCKET_COUNT` | 12 | Hourly buckets for night bar chart |
| `COUGH_NIGHT_BUCKET_MS` | 3,600,000 ms (1 hour) | Duration per bucket |
| `APP_DISPLAY_LVGL_COUGH_EVENT_HOLD_MS` | 3,000 ms (3s) | "Cough detected" badge hold time |
| `WAKE_FAR_DELTA_CM` | 50 cm | Distance threshold for wake-up detection |
| `WAKE_WINDOW_MS` | 30,000 ms (30s) | Wake-up absence confirmation window |
| `WAKE_ABSENT_CONFIRM_HITS` | 2 | Consecutive absence hits for wake-up |

## Build Flags for Touch

Touch is enabled via `APP_DISPLAY_LVGL_TOUCH_ENABLE=1` in the CM55 build.

**Required dependencies:**
- `APP_DISPLAY_LVGL_ENABLE=1` (LVGL must be enabled)
- `APP_DISPLAY_OFFICIAL_CM55_BRINGUP_ENABLE=1` (display hardware init)
- FT5406 driver dependency: `proj_cm55/deps/touch-ctp-ft5406.mtb`

**Optional diagnostics:**
- `APP_DISPLAY_LVGL_TOUCH_DIAG_ENABLE=1` — enables touch coordinate logging

**Excluded touch drivers** (via `CY_IGNORE` in Makefile):
- `gt911` (GT911 capacitive touch)
- `ili2511` (ILI2511 capacitive touch)

## Source Files

| File | Purpose |
|------|---------|
| `proj_cm55/source/app_cm55_lvgl/lv_port_indev.c` | FT5406 touch input port, swipe detection |
| `proj_cm55/source/app_cm55_lvgl/lv_port_indev.h` | Touch port header |
| `proj_cm55/source/app_cm55_lvgl/ui_health_dashboard.c` | 4-page dashboard UI |
| `proj_cm55/source/app_cm55_lvgl/ui_health_dashboard.h` | Dashboard header |
| `proj_cm55/source/app_cm55_display_bringup/app_cm55_display_rotation.c` | 180° RGB565 rotation helper |
| `proj_cm55/source/app_cm55_display_bringup/app_cm55_display_rotation.h` | Rotation header |
| `proj_cm55/deps/touch-ctp-ft5406.mtb` | FT5406 driver dependency |

## Gesture Bubbling

All child widgets on each page have `LV_OBJ_FLAG_GESTURE_BUBBLE` set recursively
via `add_gesture_bubble_to_children()`. This ensures swipe gestures on any widget
propagate up to the page root where the gesture callback is registered.

**Navigation is exclusively swipe-based** — there is no tap/click handler for
page switching.
