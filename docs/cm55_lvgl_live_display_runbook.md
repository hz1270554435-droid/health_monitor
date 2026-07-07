# CM55 LVGL Live Display Build and Flash Runbook

Date: 2026-07-07

## Purpose

The CM55 LCD dashboard uses the CM55 display bring-up/LVGL path, while live
values are produced on CM33_NS and published through the CM33 -> CM55 display
snapshot bridge. Build these two sides with different profiles.

Use this runbook when flashing the live LCD dashboard after CM55 display changes
such as framebuffer rotation, LVGL port updates, or dashboard UI changes.

## Important pitfall

Do **not** pass `APP_BUILD_PROFILE=display_bringup` as a single global top-level
`make program` argument for the live dashboard image.

That profile is correct for CM55 display bring-up, but when it is also applied to
CM33_NS it disables the CM33 live-data path, including display summary, monitor
summary, radar bridge, and the snapshot publisher. The visible symptom is:

- the screen layout and orientation are normal;
- all live numeric values stay as `--`.

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

CM55 should use `display_bringup` with LVGL and snapshot reading enabled:

- `APP_BUILD_PROFILE=display_bringup`
- `APP_DISPLAY_LVGL_ENABLE=1`
- `APP_DISPLAY_CM55_SNAPSHOT_BRIDGE_ENABLE=1`

This keeps CM55 focused on LCD bring-up/LVGL rendering and reading the published
snapshot.

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
  MTB_APPLICATION_SUBPROJECTS="proj_cm33_s proj_cm33_ns proj_cm55" \
  MTB_APPLICATION_NAME="PSOC_Edge_PDM_to_I2S" \
  MTB_APPLICATION_PROMOTE=true

make -C proj_cm55 build_proj \
  OS=Windows_NT \
  APP_BUILD_PROFILE=display_bringup \
  APP_DISPLAY_LVGL_ENABLE=1 \
  APP_DISPLAY_CM55_SNAPSHOT_BRIDGE_ENABLE=1 \
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
APP_DISPLAY_OFFICIAL_CM55_BRINGUP_ENABLE=0
```

CM55 ninja defines should include:

```text
APP_DISPLAY_LVGL_ENABLE=1
APP_DISPLAY_CM55_SNAPSHOT_BRIDGE_ENABLE=1
APP_DISPLAY_OFFICIAL_CM55_BRINGUP_ENABLE=1
APP_CM55_INFERENCE_ENABLE=0
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
