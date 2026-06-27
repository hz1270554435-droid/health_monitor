/*******************************************************************************
* File Name : app_display_cm55_bridge.c
*
* Description : CM33-side producer for the display snapshot bridge.
*
* Publishes e84_display_snapshot_t to a fixed-offset region in m33_m55_shared
* so CM55 can read it and render on the LCD.
*******************************************************************************/

#include "app_display_cm55_bridge.h"

#if (APP_DISPLAY_CM55_SNAPSHOT_BRIDGE_ENABLE)

#include "app_build_config.h"
#include "app_display_cm55_shared.h"
#include "app_board_time.h"

#include <stdio.h>
#include <string.h>

static uint32_t bridge_heartbeat;

cy_rslt_t app_display_cm55_bridge_init(void)
{
    volatile app_display_cm55_snapshot_t *snap = APP_DISPLAY_CM55_SNAPSHOT;

    memset((void *)snap, 0, sizeof(*snap));
    snap->magic = APP_DISPLAY_CM55_SNAPSHOT_MAGIC;
    snap->version = APP_DISPLAY_CM55_SNAPSHOT_VERSION;
    snap->seq_begin = 0u;
    snap->seq_end = 0u;
    snap->heartbeat = 0u;
    bridge_heartbeat = 0u;

    APP_DISPLAY_CM55_CLEAN_CACHE(
        (uint32_t)snap, sizeof(app_display_cm55_snapshot_t));

#if (APP_DISPLAY_CM55_SNAPSHOT_BRIDGE_DIAG_ENABLE)
    printf("[DISPLAY_BRIDGE] init magic=0x%08lx ver=%lu addr=0x%08lx\r\n",
           (unsigned long)snap->magic,
           (unsigned long)snap->version,
           (unsigned long)(uintptr_t)snap);
    fflush(stdout);
#endif

    return CY_RSLT_SUCCESS;
}

cy_rslt_t app_display_cm55_bridge_publish(
    const e84_display_snapshot_t *snapshot)
{
    volatile app_display_cm55_snapshot_t *snap = APP_DISPLAY_CM55_SNAPSHOT;

    if ((NULL == snapshot) ||
        (APP_DISPLAY_CM55_SNAPSHOT_MAGIC != snap->magic) ||
        (APP_DISPLAY_CM55_SNAPSHOT_VERSION != snap->version))
    {
        return CY_RSLT_TYPE_ERROR;
    }

    bridge_heartbeat++;

    /* Seq begin: signal CM55 that write is starting. */
    snap->seq_begin = bridge_heartbeat;
    __DMB();

    /* Copy fields. */
    snap->timestamp_ms = snapshot->timestamp_ms;
    snap->health_state = (uint8_t)snapshot->health_state;
    snap->alert_code = (uint8_t)snapshot->active_alert;
    snap->radar_source = (uint8_t)snapshot->radar_source_state;
    snap->flags = (uint8_t)(snapshot->flags & 0xFFu);

    snap->mic_cough_prob_x1000 =
        (uint16_t)(snapshot->mic_cough_prob * 1000.0f + 0.5f);
    snap->audio_quality = snapshot->audio_quality;
    snap->cough_model_not_verified = snapshot->cough_model_not_verified ? 1u : 0u;

    snap->rr_bpm_x10 = (uint16_t)(snapshot->breath_rate_bpm * 10.0f + 0.5f);
    snap->hr_bpm_x10 = (uint16_t)(snapshot->heart_rate_bpm * 10.0f + 0.5f);
    snap->radar_quality = snapshot->radar_quality;
    snap->radar_presence = snapshot->radar_presence ? 1u : 0u;
    snap->distance_cm = snapshot->distance_cm;

    snap->cough_count_1min = snapshot->cough_count_1min;
    snap->cough_count_5min = snapshot->cough_count_5min;
    snap->cough_event_count_total = snapshot->cough_event_count_total;

    snap->fusion_confidence = snapshot->fusion_confidence;
    snap->ble_connected = snapshot->ble_connected ? 1u : 0u;
    uint32_t wall_epoch_s = 0u;
    if (app_board_time_now_epoch_s(&wall_epoch_s))
    {
        snap->wall_epoch_s = wall_epoch_s;
        snap->wall_time_flags = app_board_time_get_flags() &
                                (DISPLAY_CM55_TIME_FLAG_VALID |
                                 DISPLAY_CM55_TIME_FLAG_NVM_LOADED |
                                 DISPLAY_CM55_TIME_FLAG_NVM_SAVED |
                                 DISPLAY_CM55_TIME_FLAG_BLE_SYNCED);
    }
    else
    {
        snap->wall_epoch_s = 0u;
        snap->wall_time_flags = 0u;
    }

    snap->heartbeat = bridge_heartbeat;

    __DMB();

    /* Seq end: signal CM55 that write is complete. */
    snap->seq_end = bridge_heartbeat;

    APP_DISPLAY_CM55_CLEAN_CACHE(
        (uint32_t)snap, sizeof(app_display_cm55_snapshot_t));

#if (APP_DISPLAY_CM55_SNAPSHOT_BRIDGE_DIAG_ENABLE)
    if ((bridge_heartbeat % 10u) == 1u)
    {
        printf("[DISPLAY_BRIDGE] seq=%lu health=%u prob=%u radar=%u\r\n",
               (unsigned long)bridge_heartbeat,
               (unsigned int)snap->health_state,
               (unsigned int)snap->mic_cough_prob_x1000,
               (unsigned int)snap->radar_source);
        fflush(stdout);
    }
#endif

    return CY_RSLT_SUCCESS;
}

#endif /* APP_DISPLAY_CM55_SNAPSHOT_BRIDGE_ENABLE */
