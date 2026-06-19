#include "app_radar_bridge.h"

#if (APP_RADAR_BRIDGE_ENABLE)

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "app_uart_radar.h"
#include "app_radar_quality.h"
#include "app_monitor_summary.h"
#include "app_monitor_summary_types.h"
#include "mtb_hal.h"

/* ---------------------------------------------------------------------------
 * Internal observation accumulator.
 *
 * This struct holds the latest decoded radar fields from parser frames.
 * It is NOT a shared-memory ABI — it is firmware-internal to the bridge task.
 * --------------------------------------------------------------------------- */
typedef struct
{
    /* Timestamps. */
    uint32_t newest_frame_timestamp_ms;
    uint32_t snapshot_timestamp_ms;

    /* Decoded fields from latest frames. */
    uint16_t presence_raw;          /* 0x0F09 raw u16 */
    int32_t  rr_centi;              /* 0x0A14 breath rate * 100 */
    int32_t  hr_centi;              /* 0x0A15 heart rate * 100 */
    uint32_t range_flag;            /* 0x0A16 flag */
    int32_t  distance_centi;        /* 0x0A16 distance * 100 */
    int32_t  x_centi;               /* 0x0A17 X * 100 */
    int32_t  y_centi;               /* 0x0A17 Y * 100 */
    int32_t  z_centi;               /* 0x0A17 Z * 100 */

    /* Field freshness flags (true if decoded from a valid frame). */
    bool has_presence;
    bool has_rr;
    bool has_hr;
    bool has_distance;
    bool has_position;

    /* TYPE coverage counters. */
    uint32_t type_0x0F09_count;
    uint32_t type_0x0A13_count;     /* semantic TBD — structural only */
    uint32_t type_0x0A14_count;
    uint32_t type_0x0A15_count;
    uint32_t type_0x0A16_count;
    uint32_t type_0x0A17_count;
    uint32_t type_0x0A04_count;     /* raw-only / non-product */
    uint32_t type_unsupported_count;

    /* Total frames processed. */
    uint32_t total_frames;
} app_radar_bridge_accumulator_t;

static TaskHandle_t bridge_task_handle = NULL;
static app_radar_bridge_accumulator_t accumulator;

/* ---------------------------------------------------------------------------
 * Helpers (mirror app_uart_radar.c decode logic)
 * --------------------------------------------------------------------------- */

static uint16_t bridge_read_le_u16(const uint8_t *data)
{
    return ((uint16_t)data[1] << 8) | data[0];
}

static uint32_t bridge_read_le_u32(const uint8_t *data)
{
    return ((uint32_t)data[3] << 24) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[1] << 8) |
           data[0];
}

static int32_t bridge_float_to_centi(const uint8_t *data)
{
    uint32_t raw = bridge_read_le_u32(data);
    float value;

    memcpy(&value, &raw, sizeof(value));

    if (0.0f <= value)
    {
        return (int32_t)((value * 100.0f) + 0.5f);
    }
    return (int32_t)((value * 100.0f) - 0.5f);
}

/* Check if a float value is finite and in range. */
static bool bridge_float_is_finite(const uint8_t *data)
{
    uint32_t raw = bridge_read_le_u32(data);
    float value;

    memcpy(&value, &raw, sizeof(value));
    return isfinite(value);
}

/* ---------------------------------------------------------------------------
 * Frame decoder — update accumulator from a single parser block.
 * --------------------------------------------------------------------------- */
static void bridge_decode_frame(app_radar_bridge_accumulator_t *acc,
                                const app_uart_radar_block_t *block,
                                uint32_t now_ms)
{
    const uint8_t *data = block->data;
    uint16_t data_len = block->data_len;

    acc->total_frames++;
    acc->newest_frame_timestamp_ms = now_ms;

    switch (block->type)
    {
        case APP_RADAR_BRIDGE_TYPE_HUMAN_STATUS:
            acc->type_0x0F09_count++;
            if (data_len >= 2u)
            {
                acc->presence_raw = bridge_read_le_u16(data);
                acc->has_presence = true;
            }
            break;

        case APP_RADAR_BRIDGE_TYPE_BREATH_RATE:
            acc->type_0x0A14_count++;
            if ((data_len >= 4u) && bridge_float_is_finite(data))
            {
                int32_t centi = bridge_float_to_centi(data);
                /* Contract range: 4..60 bpm → 400..6000 centi */
                if ((centi >= 400) && (centi <= 6000))
                {
                    acc->rr_centi = centi;
                    acc->has_rr = true;
                }
                else
                {
                    acc->has_rr = false;  /* out of range */
                }
            }
            break;

        case APP_RADAR_BRIDGE_TYPE_HEART_RATE:
            acc->type_0x0A15_count++;
            if ((data_len >= 4u) && bridge_float_is_finite(data))
            {
                int32_t centi = bridge_float_to_centi(data);
                /* Contract range: 30..220 bpm → 3000..22000 centi */
                if ((centi >= 3000) && (centi <= 22000))
                {
                    acc->hr_centi = centi;
                    acc->has_hr = true;
                }
                else
                {
                    acc->has_hr = false;  /* out of range */
                }
            }
            break;

        case APP_RADAR_BRIDGE_TYPE_TARGET_RANGE:
            acc->type_0x0A16_count++;
            if (data_len >= 8u)
            {
                acc->range_flag = bridge_read_le_u32(data);
                if (bridge_float_is_finite(&data[4]))
                {
                    int32_t centi = bridge_float_to_centi(&data[4]);
                    /* Contract range: 0.0..10.0 m → 0..1000 centi */
                    if ((centi >= 0) && (centi <= 1000))
                    {
                        acc->distance_centi = centi;
                        acc->has_distance = true;
                    }
                    else
                    {
                        acc->has_distance = false;
                    }
                }
            }
            break;

        case APP_RADAR_BRIDGE_TYPE_TRACK_POSITION:
            acc->type_0x0A17_count++;
            if (data_len >= 12u)
            {
                acc->x_centi = bridge_float_to_centi(data);
                acc->y_centi = bridge_float_to_centi(&data[4]);
                acc->z_centi = bridge_float_to_centi(&data[8]);
                acc->has_position = true;
            }
            break;

        case APP_RADAR_BRIDGE_TYPE_PHASE:
            /* 0x0A13: structural presence only, semantic TBD */
            acc->type_0x0A13_count++;
            break;

        case APP_RADAR_BRIDGE_TYPE_HUMAN_POSITION:
            /* 0x0A04: raw-only / non-product */
            acc->type_0x0A04_count++;
            break;

        default:
            acc->type_unsupported_count++;
            break;
    }
}

/* ---------------------------------------------------------------------------
 * Map accumulator + quality → app_monitor_radar_input_t
 * --------------------------------------------------------------------------- */
static void bridge_map_to_radar_input(
    const app_radar_bridge_accumulator_t *acc,
    const app_radar_quality_result_t *quality,
    app_monitor_radar_input_t *out)
{
    memset(out, 0, sizeof(*out));

    /* State flags from quality producer. */
    out->valid = quality->valid;
    out->ready = quality->ready;
    out->stale = quality->stale;
    out->radar_quality = quality->quality;
    out->age_ms = quality->age_ms;
    out->radar_reason_flags = quality->reason_flags;

    /* If not valid, all product fields remain at invalid sentinels. */
    if (!quality->valid)
    {
        out->presence_state = APP_MONITOR_PRESENCE_UNKNOWN;
        out->motion_state = 0;      /* unknown */
        out->breath_state = APP_MONITOR_VITAL_UNKNOWN;
        out->heart_state = APP_MONITOR_VITAL_UNKNOWN;
        out->motion_x100 = 0;
        out->rr_bpm_x10 = 0;
        out->hr_bpm_x10 = 0;
        out->distance_cm = 0;
        return;
    }

    /* Presence mapping (conservative). */
    if (acc->has_presence)
    {
        switch (acc->presence_raw)
        {
            case APP_RADAR_BRIDGE_PRESENCE_ABSENT_RAW:
                out->presence_state = APP_MONITOR_PRESENCE_ABSENT;
                break;
            case APP_RADAR_BRIDGE_PRESENCE_PRESENT_RAW:
                out->presence_state = APP_MONITOR_PRESENCE_PRESENT;
                break;
            default:
                out->presence_state = APP_MONITOR_PRESENCE_UNKNOWN;
                break;
        }
    }
    else
    {
        out->presence_state = APP_MONITOR_PRESENCE_UNKNOWN;
    }

    /* Breath rate mapping. */
    if (acc->has_rr)
    {
        out->rr_bpm_x10 = (uint16_t)(acc->rr_centi / 10);
        out->breath_state = APP_MONITOR_VITAL_NORMAL;
    }
    else
    {
        out->rr_bpm_x10 = 0;
        out->breath_state = APP_MONITOR_VITAL_INVALID;
    }

    /* Heart rate mapping. */
    if (acc->has_hr)
    {
        out->hr_bpm_x10 = (uint16_t)(acc->hr_centi / 10);
        out->heart_state = APP_MONITOR_VITAL_NORMAL;
    }
    else
    {
        out->hr_bpm_x10 = 0;
        out->heart_state = APP_MONITOR_VITAL_INVALID;
    }

    /* Distance mapping. */
    if (acc->has_distance)
    {
        out->distance_cm = (uint16_t)(acc->distance_centi);
    }
    else
    {
        out->distance_cm = 0;
    }

    /* Motion: P1B does NOT map 0x0A17 to product motion.
     * motion_state remains unknown, motion_x100 remains 0.
     * 0x0A13 and 0x0A04 are NOT used for motion. */
    out->motion_state = 0;  /* unknown */
    out->motion_x100 = 0;
}

/* ---------------------------------------------------------------------------
 * Source marker log
 * --------------------------------------------------------------------------- */
static void bridge_log_source_markers(const app_radar_bridge_accumulator_t *acc,
                                      const app_radar_quality_result_t *quality)
{
    printf("[RADAR_BRIDGE] mock=0 host_mock=0 fake=0 projection=0 "
           "real_radar=%u bridge=1 test_decoder=0 "
           "quality=%u valid=%u stale=%u age_ms=%lu "
           "frames=%lu 0x0F09=%lu 0x0A13=%lu 0x0A14=%lu 0x0A15=%lu "
           "0x0A16=%lu 0x0A17=%lu 0x0A04=%lu unsup=%lu "
           "presence_raw=%u has_rr=%u rr_centi=%ld has_hr=%u "
           "hr_centi=%ld has_dist=%u dist_centi=%ld\r\n",
           quality->valid ? 1u : 0u,
           (unsigned int)quality->quality,
           quality->valid ? 1u : 0u,
           quality->stale ? 1u : 0u,
           (unsigned long)quality->age_ms,
           (unsigned long)acc->total_frames,
           (unsigned long)acc->type_0x0F09_count,
           (unsigned long)acc->type_0x0A13_count,
           (unsigned long)acc->type_0x0A14_count,
           (unsigned long)acc->type_0x0A15_count,
           (unsigned long)acc->type_0x0A16_count,
           (unsigned long)acc->type_0x0A17_count,
           (unsigned long)acc->type_0x0A04_count,
           (unsigned long)acc->type_unsupported_count,
           (unsigned int)acc->presence_raw,
           acc->has_rr ? 1u : 0u,
           (long)acc->rr_centi,
           acc->has_hr ? 1u : 0u,
           (long)acc->hr_centi,
           acc->has_distance ? 1u : 0u,
           (long)acc->distance_centi);
}

/* ---------------------------------------------------------------------------
 * Bridge task main loop
 * --------------------------------------------------------------------------- */
static void app_radar_bridge_task(void *pvParameters)
{
    (void)pvParameters;
    app_uart_radar_block_t block;
    app_radar_quality_input_t quality_input;
    app_radar_quality_result_t quality_result;
    app_monitor_radar_input_t radar_input;
    uint32_t last_log_tick = xTaskGetTickCount();

    memset(&accumulator, 0, sizeof(accumulator));
    app_radar_quality_input_default(&quality_input);
    quality_input.profile = APP_RADAR_QUALITY_PROFILE_BRIDGE;

    printf("[RADAR_BRIDGE] task started, profile=bridge, "
           "test_decoder=disabled\r\n");

    for (;;)
    {
        bool got_block = app_uart_radar_receive_block(
            &block, pdMS_TO_TICKS(APP_RADAR_BRIDGE_RX_WAIT_MS));

        uint32_t now_ms = (uint32_t)(xTaskGetTickCount() *
                                      portTICK_PERIOD_MS);

        if (got_block)
        {
            /* Decode frame into accumulator. */
            bridge_decode_frame(&accumulator, &block, now_ms);

            /* Release block back to parser. */
            app_uart_radar_release_block(block.block_index);

            /* Build quality input from parser counters. */
            quality_input.received_count =
                app_uart_radar_get_received_count();
            quality_input.bad_frame_count =
                app_uart_radar_get_bad_frame_count();
            quality_input.uart_err_count =
                app_uart_radar_get_uart_error_count();
            quality_input.drop_count =
                app_uart_radar_get_dropped_count();
            quality_input.resync_count =
                app_uart_radar_get_resync_count();
            quality_input.newest_frame_timestamp_ms =
                accumulator.newest_frame_timestamp_ms;
            quality_input.current_timestamp_ms = now_ms;

            /* Required field validity. */
            quality_input.field_valid[APP_RADAR_QUALITY_FIELD_PRESENCE] =
                accumulator.has_presence;
            quality_input.field_valid[APP_RADAR_QUALITY_FIELD_RR_BPM] =
                accumulator.has_rr;
            quality_input.field_valid[APP_RADAR_QUALITY_FIELD_HR_BPM] =
                accumulator.has_hr;
            quality_input.field_valid[APP_RADAR_QUALITY_FIELD_DISTANCE] =
                accumulator.has_distance;

            /* Compute quality. */
            app_radar_quality_compute(&quality_input, &quality_result);

            /* Map to Monitor Summary radar input. */
            bridge_map_to_radar_input(&accumulator,
                                      &quality_result,
                                      &radar_input);

            /* Update Monitor Summary and materialize a fresh snapshot for
             * Display/BLE consumers. update_radar() only copies the input. */
            (void)app_monitor_summary_update_radar(&radar_input);
            (void)app_monitor_summary_tick(now_ms);
        }

        /* Periodic source marker log. */
        if (pdMS_TO_TICKS(APP_RADAR_BRIDGE_LOG_MS) <=
            (xTaskGetTickCount() - last_log_tick))
        {
            bridge_log_source_markers(&accumulator, &quality_result);
            last_log_tick = xTaskGetTickCount();
        }
    }
}

/* ---------------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------------- */

cy_rslt_t app_radar_bridge_init(void)
{
    BaseType_t ret;

    if (NULL != bridge_task_handle)
    {
        return CY_RSLT_SUCCESS;
    }

    /* Safety check: if test decoder is enabled, refuse to start.
     * Bridge and test decoder MUST be mutually exclusive. */
#if (APP_UART_RADAR_TEST_ENABLE)
    #error "APP_RADAR_BRIDGE_ENABLE and APP_UART_RADAR_TEST_ENABLE cannot both be 1"
#endif

    ret = xTaskCreate(app_radar_bridge_task,
                      "radar_bridge",
                      APP_RADAR_BRIDGE_TASK_STACK_SIZE,
                      NULL,
                      APP_RADAR_BRIDGE_TASK_PRIORITY,
                      &bridge_task_handle);

    return (pdPASS == ret) ? CY_RSLT_SUCCESS : CY_RSLT_TYPE_ERROR;
}

TaskHandle_t app_radar_bridge_get_task_handle(void)
{
    return bridge_task_handle;
}

#endif /* APP_RADAR_BRIDGE_ENABLE */
