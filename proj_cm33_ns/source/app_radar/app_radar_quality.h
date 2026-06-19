#ifndef __APP_RADAR_QUALITY_H__
#define __APP_RADAR_QUALITY_H__

#if defined(__cplusplus)
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* ---------------------------------------------------------------------------
 * P1B-01 radar_quality producer — frozen constants from P1B-01A policy.
 *
 * This module implements the radar_quality calculation as a pure function.
 * It does NOT integrate with Monitor Summary (P1B-02 scope).
 * It does NOT change shared-memory ABI or BLE wire format.
 * --------------------------------------------------------------------------- */

/* Base quality score — start at maximum, penalties only reduce. */
#define APP_RADAR_QUALITY_BASE                  (100u)

/* TTL for radar observation freshness (ms). */
#define APP_RADAR_QUALITY_TTL_MS                (3000u)

/* Rolling error window duration (ms). */
#define APP_RADAR_QUALITY_ERROR_WINDOW_MS       (5000u)

/* bad_frame rate threshold: above this, apply penalty. */
#define APP_RADAR_QUALITY_BAD_FRAME_THRESHOLD   (5u)   /* percent */
#define APP_RADAR_QUALITY_BAD_FRAME_PENALTY     (30u)

/* uart_err rate threshold: above this, apply penalty. */
#define APP_RADAR_QUALITY_UART_ERR_THRESHOLD    (1u)    /* percent */
#define APP_RADAR_QUALITY_UART_ERR_PENALTY      (20u)

/* Drop rate threshold (bridge profile only): above this, apply penalty. */
#define APP_RADAR_QUALITY_DROP_THRESHOLD        (10u)   /* percent */
#define APP_RADAR_QUALITY_DROP_PENALTY          (20u)

/* Per required-field penalty when field is invalid. */
#define APP_RADAR_QUALITY_FIELD_PENALTY         (20u)

/* Quality below this triggers radar_quality_poor reason flag. */
#define APP_RADAR_QUALITY_POOR_THRESHOLD        (35u)

/* Minimum quality for valid=true. */
#define APP_RADAR_QUALITY_VALID_MIN             (1u)

/* Invalid age sentinel when no frame exists. */
#define APP_RADAR_QUALITY_AGE_INVALID           (0xFFFFFFFFu)

/* Maximum percent value. */
#define APP_RADAR_QUALITY_MAX                   (100u)

/* ---------------------------------------------------------------------------
 * Profile enum — controls drop penalty behavior.
 * --------------------------------------------------------------------------- */
typedef enum
{
    APP_RADAR_QUALITY_PROFILE_TEST = 0,   /* Test decoder: drops are artifact */
    APP_RADAR_QUALITY_PROFILE_BRIDGE      /* Bridge: drops are real backpressure */
} app_radar_quality_profile_t;

/* ---------------------------------------------------------------------------
 * Required field indices for penalty tracking.
 * --------------------------------------------------------------------------- */
typedef enum
{
    APP_RADAR_QUALITY_FIELD_PRESENCE = 0,
    APP_RADAR_QUALITY_FIELD_RR_BPM,
    APP_RADAR_QUALITY_FIELD_HR_BPM,
    APP_RADAR_QUALITY_FIELD_DISTANCE,
    APP_RADAR_QUALITY_FIELD_COUNT  /* must be last */
} app_radar_quality_field_t;

/* ---------------------------------------------------------------------------
 * Input: parser counters and field validity for quality computation.
 * --------------------------------------------------------------------------- */
typedef struct
{
    /* Parser health counters (lifetime or windowed — caller manages window). */
    uint32_t received_count;
    uint32_t bad_frame_count;
    uint32_t uart_err_count;
    uint32_t drop_count;
    uint32_t resync_count;

    /* Timestamps. */
    uint32_t newest_frame_timestamp_ms;  /* monotonic ms of newest accepted frame */
    uint32_t current_timestamp_ms;       /* monotonic ms of current snapshot */

    /* Per-field validity (true = valid, false = invalid/missing/OOR/NaN). */
    bool field_valid[APP_RADAR_QUALITY_FIELD_COUNT];

    /* Profile selection. */
    app_radar_quality_profile_t profile;
} app_radar_quality_input_t;

/* ---------------------------------------------------------------------------
 * Output: quality result with state flags and reason.
 * --------------------------------------------------------------------------- */
typedef struct
{
    uint8_t quality;        /* 0..100 */
    bool valid;             /* true only if quality >= VALID_MIN and not stale */
    bool stale;             /* true if age > TTL or no frame */
    bool ready;             /* true if producer path can report state */
    uint32_t age_ms;        /* age of newest frame, or AGE_INVALID */
    uint32_t reason_flags;  /* radar reason band bits */
} app_radar_quality_result_t;

/* ---------------------------------------------------------------------------
 * Reason flag bits (radar band, bits 8..15 per monitor_summary contract).
 * --------------------------------------------------------------------------- */
#define APP_RADAR_QUALITY_REASON_UNAVAILABLE    (1u << 8)
#define APP_RADAR_QUALITY_REASON_STALE          (1u << 14)
#define APP_RADAR_QUALITY_REASON_QUALITY_POOR   (1u << 15)

/* ---------------------------------------------------------------------------
 * API
 * --------------------------------------------------------------------------- */

/**
 * Compute radar_quality from parser counters and field validity.
 *
 * This is a pure function with no side effects. It does NOT write to
 * Monitor Summary, shared memory, or BLE. It does NOT consume the
 * parser queue.
 *
 * @param[in]  input   Parser counters, timestamps, field validity, profile.
 * @param[out] result  Quality, valid, stale, ready, age, reason flags.
 */
void app_radar_quality_compute(const app_radar_quality_input_t *input,
                               app_radar_quality_result_t *result);

/**
 * Initialize a default input struct with zero counters and invalid fields.
 */
void app_radar_quality_input_default(app_radar_quality_input_t *input);

#if defined(__cplusplus)
}
#endif

#endif /* __APP_RADAR_QUALITY_H__ */
