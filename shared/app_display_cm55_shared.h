/*******************************************************************************
* File Name : app_display_cm55_shared.h
*
* Description : CM33->CM55 display snapshot bridge over m33_m55_shared.
*
* This header defines a fixed-offset region in m33_m55_shared where CM33
* publishes health display snapshots for CM55 to render on the LCD.
*
* Memory layout:
*   m33_m55_shared + 0x00000 : (legacy) radar frame buffer, other users
*   m33_m55_shared + 0x08000 : app_model_shared_region_t (model IPC)
*   m33_m55_shared + 0x3E000 : app_display_cm55_snapshot_t (THIS BRIDGE, 4KB)
*   m33_m55_shared + 0x3F000 : app_display_diag_region_t (display diag)
*
* This bridge does NOT change app_model_shared_region_t or its version.
* Fields use fixed-width integers for ABI stability across CM33/CM55 builds.
*******************************************************************************/

#ifndef __APP_DISPLAY_CM55_SHARED_H__
#define __APP_DISPLAY_CM55_SHARED_H__

#include <stdint.h>

#include "cy_pdl.h"
#if defined(COMPONENT_CM55)
#include "cymem_CM55_0.h"
#else
#include "cymem_CM33_0.h"
#endif

#if defined(__cplusplus)
extern "C" {
#endif

/* "DSNP" = DiSplay SNaPshot */
#define APP_DISPLAY_CM55_SNAPSHOT_MAGIC        (0x44534E50u)
#define APP_DISPLAY_CM55_SNAPSHOT_VERSION      (1u)

/* Fixed offset within m33_m55_shared. Must not overlap model IPC (0x8000)
 * or display diag (0x3F000). 4KB budget is generous for this small struct. */
#define APP_DISPLAY_CM55_SNAPSHOT_OFFSET_BYTES (0x0003E000u)

#if defined(COMPONENT_CM55)
#define APP_DISPLAY_CM55_SNAPSHOT_BASE_ADDR    (CYMEM_CM55_0_m33_m55_shared_START)
#define APP_DISPLAY_CM55_SNAPSHOT_REGION_SIZE  (CYMEM_CM55_0_m33_m55_shared_SIZE)
#else
#define APP_DISPLAY_CM55_SNAPSHOT_BASE_ADDR    (CYMEM_CM33_0_m33_m55_shared_START)
#define APP_DISPLAY_CM55_SNAPSHOT_REGION_SIZE  (CYMEM_CM33_0_m33_m55_shared_SIZE)
#endif

#define APP_DISPLAY_CM55_SNAPSHOT_ADDR \
    (APP_DISPLAY_CM55_SNAPSHOT_BASE_ADDR + \
     APP_DISPLAY_CM55_SNAPSHOT_OFFSET_BYTES)

/* Health states — must match e84_display_health_state_t values. */
#define DISPLAY_CM55_HEALTH_INIT          (0u)
#define DISPLAY_CM55_HEALTH_NORMAL        (1u)
#define DISPLAY_CM55_HEALTH_ATTENTION     (2u)
#define DISPLAY_CM55_HEALTH_WARNING       (3u)
#define DISPLAY_CM55_HEALTH_SENSOR_LOST   (4u)
#define DISPLAY_CM55_HEALTH_ERROR         (5u)

/* Alert codes — must match e84_display_alert_t values. */
#define DISPLAY_CM55_ALERT_NONE               (0u)
#define DISPLAY_CM55_ALERT_COUGH_BURST        (1u)
#define DISPLAY_CM55_ALERT_RESP_RATE_ABNORMAL (2u)
#define DISPLAY_CM55_ALERT_HEART_RATE_ABNORMAL (3u)
#define DISPLAY_CM55_ALERT_BREATHING_GAP      (4u)
#define DISPLAY_CM55_ALERT_SENSOR_LOST        (5u)
#define DISPLAY_CM55_ALERT_SYSTEM_ERROR       (6u)

/* Radar source states — must match e84_display_radar_source_state_t values. */
#define DISPLAY_CM55_RADAR_NORMAL         (0u)
#define DISPLAY_CM55_RADAR_UNAVAILABLE    (1u)
#define DISPLAY_CM55_RADAR_STALE          (2u)
#define DISPLAY_CM55_RADAR_INVALID        (3u)
#define DISPLAY_CM55_RADAR_LOW_QUALITY    (4u)

/* Flags — must match E84_DISPLAY_FLAG_* values. */
#define DISPLAY_CM55_FLAG_AUDIO_VALID              (1u << 0)
#define DISPLAY_CM55_FLAG_RADAR_VALID              (1u << 1)
#define DISPLAY_CM55_FLAG_FUSION_VALID             (1u << 2)
#define DISPLAY_CM55_FLAG_BLE_VALID                (1u << 3)
#define DISPLAY_CM55_FLAG_RR_VALID                 (1u << 4)
#define DISPLAY_CM55_FLAG_HR_VALID                 (1u << 5)
#define DISPLAY_CM55_FLAG_ALERT_LATCHED            (1u << 6)
#define DISPLAY_CM55_FLAG_COUGH_MODEL_NOT_VERIFIED (1u << 7)

/* Wall-clock time flags. The epoch field is valid only when bit 0 is set. */
#define DISPLAY_CM55_TIME_FLAG_VALID      (1u << 0)
#define DISPLAY_CM55_TIME_FLAG_NVM_LOADED (1u << 1)
#define DISPLAY_CM55_TIME_FLAG_NVM_SAVED  (1u << 2)
#define DISPLAY_CM55_TIME_FLAG_BLE_SYNCED (1u << 3)

/* Display snapshot bridge structure.
 * All fields are fixed-width for ABI stability. CM33 writes, CM55 reads.
 * seq_begin/seq_end bracket the write; CM55 reads seq_end first, then
 * data, then seq_begin to detect torn reads. __DMB() on both sides. */
typedef struct
{
    /* Magic + version for validity check. */
    volatile uint32_t magic;
    volatile uint32_t version;
    /* Sequence bracket: CM33 writes seq_begin before data, seq_end after. */
    volatile uint32_t seq_begin;
    volatile uint32_t seq_end;
    /* Timestamp of last update from CM33. */
    volatile uint32_t timestamp_ms;
    /* Heartbeat counter: CM33 increments on each publish. */
    volatile uint32_t heartbeat;

    /* Health and alert state. */
    volatile uint8_t  health_state;      /* DISPLAY_CM55_HEALTH_* */
    volatile uint8_t  alert_code;        /* 0=NONE, see e84_display_alert_t */
    volatile uint8_t  radar_source;      /* DISPLAY_CM55_RADAR_* */
    volatile uint8_t  flags;             /* DISPLAY_CM55_FLAG_* bitmask */

    /* MIC audio data. */
    volatile uint16_t mic_cough_prob_x1000; /* 0..1000 = 0.0..100.0% */
    volatile uint8_t  audio_quality;        /* 0..100 */
    volatile uint8_t  cough_model_not_verified; /* 1 = show 模型未验证 */

    /* Radar vitals. */
    volatile uint16_t rr_bpm_x10;        /* breath rate * 10, 0=N/A */
    volatile uint16_t hr_bpm_x10;        /* heart rate * 10, 0=N/A */
    volatile uint8_t  radar_quality;     /* 0..100 */
    volatile uint8_t  radar_presence;    /* 1=present */
    volatile uint16_t distance_cm;       /* 0=N/A */

    /* Cough counts. */
    volatile uint16_t cough_count_1min;
    volatile uint16_t cough_count_5min;
    volatile uint32_t cough_event_count_total;

    /* Fusion and BLE. */
    volatile uint8_t  fusion_confidence; /* 0..100 */
    volatile uint8_t  ble_connected;     /* 1=connected */
    volatile uint16_t reserved0;

    /* Wall-clock time, supplied by CM33 board-time service. */
    volatile uint32_t wall_epoch_s;       /* Unix epoch seconds, 0=N/A */
    volatile uint32_t wall_time_flags;    /* DISPLAY_CM55_TIME_FLAG_* */

    /* Last APP_MONITOR_EVENT_CONFIRMED_COUGH event id mirrored to BLE. */
    volatile uint32_t last_cough_event_id;

    /* Reserved for future use. */
    volatile uint32_t reserved[4];
} app_display_cm55_snapshot_t;

#define APP_DISPLAY_CM55_SNAPSHOT \
    ((volatile app_display_cm55_snapshot_t *)APP_DISPLAY_CM55_SNAPSHOT_ADDR)

/* Compile-time size check: snapshot must fit before diag region at 0x3F000. */
typedef char app_display_cm55_snapshot_size_check[
    ((APP_DISPLAY_CM55_SNAPSHOT_OFFSET_BYTES +
      sizeof(app_display_cm55_snapshot_t)) <=
     0x3F000u) ? 1 : -1];

/* Cache maintenance hooks. Override if m33_m55_shared is cacheable. */
#ifndef APP_DISPLAY_CM55_CLEAN_CACHE
#define APP_DISPLAY_CM55_CLEAN_CACHE(address, size) \
    do { (void)(address); (void)(size); } while (0)
#endif

#ifndef APP_DISPLAY_CM55_INVALIDATE_CACHE
#define APP_DISPLAY_CM55_INVALIDATE_CACHE(address, size) \
    do { (void)(address); (void)(size); } while (0)
#endif

#if defined(__cplusplus)
}
#endif

#endif /* __APP_DISPLAY_CM55_SHARED_H__ */
