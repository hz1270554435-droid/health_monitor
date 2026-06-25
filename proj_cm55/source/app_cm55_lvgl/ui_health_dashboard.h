/**
 * @file ui_health_dashboard.h
 * Health monitor dashboard UI — LVGL implementation.
 *
 * Dark-theme night monitoring layout:
 *   Header:  "夜间健康监测"  +  green "实时监测中" capsule
 *   Cards:   2×2 grid — 呼吸率 | 心率 | 咳嗽事件 | 雷达
 *   Footer:  trend text + MIC/Radar/BLE status dots
 *
 * Data source: app_display_cm55_snapshot_t (read-only, seq_begin/end protected)
 */

#ifndef UI_HEALTH_DASHBOARD_H
#define UI_HEALTH_DASHBOARD_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Create all dashboard widgets (call once after lv_init).
 *
 * Must be called from the GFX task after lv_port_disp_init().
 */
void ui_health_dashboard_init(void);

/**
 * @brief Refresh dashboard labels from the latest snapshot.
 *
 * Call at ~1 Hz from the GFX task loop.  Reads the shared-memory snapshot
 * and updates label text, colors, and visibility.
 */
void ui_health_dashboard_update(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*UI_HEALTH_DASHBOARD_H*/
