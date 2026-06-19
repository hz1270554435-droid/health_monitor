#include "app_radar_quality.h"

#include <string.h>

/* ---------------------------------------------------------------------------
 * P1B-01 radar_quality producer — frozen formula from P1B-01A policy.
 *
 * This module is a pure calculation engine. It has no side effects, does not
 * access shared memory, does not call Monitor Summary APIs, does not consume
 * the parser queue, and does not produce BLE payloads.
 *
 * Profile-aware drop penalty:
 *   - TEST profile: drops are test consumer artifact, no penalty.
 *   - BRIDGE profile: drops above threshold reduce quality.
 * --------------------------------------------------------------------------- */

void app_radar_quality_input_default(app_radar_quality_input_t *input)
{
    if (NULL == input)
    {
        return;
    }

    memset(input, 0, sizeof(*input));
    input->newest_frame_timestamp_ms = 0u;
    input->current_timestamp_ms = 0u;
    input->profile = APP_RADAR_QUALITY_PROFILE_TEST;

    for (uint32_t i = 0u; i < APP_RADAR_QUALITY_FIELD_COUNT; i++)
    {
        input->field_valid[i] = false;
    }
}

static uint8_t app_radar_quality_clamp_u8(int32_t value, uint8_t max)
{
    if (value > (int32_t)max)
    {
        return max;
    }
    if (value < 0)
    {
        return 0u;
    }
    return (uint8_t)value;
}

static uint32_t app_radar_quality_compute_age_ms(
    uint32_t current_ms, uint32_t newest_frame_ms, bool has_frame)
{
    if (!has_frame)
    {
        return APP_RADAR_QUALITY_AGE_INVALID;
    }

    /* Monotonic delta — safe for uint32 wraparound. */
    return current_ms - newest_frame_ms;
}

static bool app_radar_quality_is_stale(uint32_t age_ms, bool has_frame)
{
    if (!has_frame)
    {
        return true;
    }
    return (age_ms > APP_RADAR_QUALITY_TTL_MS);
}

/* Compute error rate as percentage: (error_count * 100) / received_count. */
static uint32_t app_radar_quality_error_rate_pct(uint32_t error_count,
                                                  uint32_t received_count)
{
    if (0u == received_count)
    {
        return 0u;
    }
    return (error_count * 100u) / received_count;
}

void app_radar_quality_compute(const app_radar_quality_input_t *input,
                               app_radar_quality_result_t *result)
{
    if ((NULL == input) || (NULL == result))
    {
        if (NULL != result)
        {
            result->quality = 0u;
            result->valid = false;
            result->stale = true;
            result->ready = false;
            result->age_ms = APP_RADAR_QUALITY_AGE_INVALID;
            result->reason_flags = APP_RADAR_QUALITY_REASON_UNAVAILABLE;
        }
        return;
    }

    memset(result, 0, sizeof(*result));
    result->ready = true;  /* Producer path is initialized. */

    /* --- Step 1: Check if any frame has been received --- */
    bool has_frame = (input->received_count > 0u);
    uint32_t age_ms = app_radar_quality_compute_age_ms(
        input->current_timestamp_ms,
        input->newest_frame_timestamp_ms,
        has_frame);
    result->age_ms = age_ms;

    if (!has_frame)
    {
        /* Unavailable: no frames received. */
        result->quality = 0u;
        result->valid = false;
        result->stale = true;
        result->reason_flags = APP_RADAR_QUALITY_REASON_UNAVAILABLE;
        return;
    }

    /* --- Step 2: Check stale --- */
    if (app_radar_quality_is_stale(age_ms, has_frame))
    {
        result->quality = 0u;
        result->valid = false;
        result->stale = true;
        result->reason_flags = APP_RADAR_QUALITY_REASON_STALE;
        return;
    }

    /* --- Step 3: Start with base quality --- */
    int32_t raw_quality = (int32_t)APP_RADAR_QUALITY_BASE;

    /* --- Step 4: Error penalties --- */
    uint32_t bad_frame_rate = app_radar_quality_error_rate_pct(
        input->bad_frame_count, input->received_count);
    if (bad_frame_rate > APP_RADAR_QUALITY_BAD_FRAME_THRESHOLD)
    {
        raw_quality -= APP_RADAR_QUALITY_BAD_FRAME_PENALTY;
    }

    uint32_t uart_err_rate = app_radar_quality_error_rate_pct(
        input->uart_err_count, input->received_count);
    if (uart_err_rate > APP_RADAR_QUALITY_UART_ERR_THRESHOLD)
    {
        raw_quality -= APP_RADAR_QUALITY_UART_ERR_PENALTY;
    }

    /* --- Step 5: Drop penalty (profile-aware) --- */
    if (APP_RADAR_QUALITY_PROFILE_BRIDGE == input->profile)
    {
        uint32_t total = input->received_count + input->drop_count;
        uint32_t drop_rate = (0u < total) ?
            (input->drop_count * 100u) / total : 0u;
        if (drop_rate > APP_RADAR_QUALITY_DROP_THRESHOLD)
        {
            raw_quality -= APP_RADAR_QUALITY_DROP_PENALTY;
        }
    }
    /* TEST profile: drop_penalty = 0 (drops are test consumer artifact). */

    /* --- Step 6: Required field penalties --- */
    for (uint32_t i = 0u; i < APP_RADAR_QUALITY_FIELD_COUNT; i++)
    {
        if (!input->field_valid[i])
        {
            raw_quality -= APP_RADAR_QUALITY_FIELD_PENALTY;
        }
    }

    /* --- Step 7: Clamp --- */
    result->quality = app_radar_quality_clamp_u8(
        raw_quality, APP_RADAR_QUALITY_MAX);

    /* --- Step 8: Determine valid --- */
    result->stale = false;
    result->valid = (result->quality >= APP_RADAR_QUALITY_VALID_MIN);

    /* --- Step 9: Reason flags --- */
    if (result->quality < APP_RADAR_QUALITY_POOR_THRESHOLD)
    {
        result->reason_flags |= APP_RADAR_QUALITY_REASON_QUALITY_POOR;
    }
}
