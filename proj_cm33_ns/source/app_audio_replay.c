/*******************************************************************************
 * File Name : app_audio_replay.c
 *
 * Description : Default-off board replay / PCM injection provider.
 *******************************************************************************/

#include "app_audio_replay.h"

#include <stdio.h>
#include <string.h>

#include "app_audio_replay_selected_asset.h"

typedef struct
{
    bool initialized;
    bool completed;
    bool pacing_started;
    TickType_t last_wake_tick;
    uint32_t mono_cursor;
    uint32_t block_sequence;
    int16_t stereo_block[APP_PDM_PCM_BLOCK_SAMPLES];
} app_audio_replay_state_t;

static app_audio_replay_state_t app_audio_replay_state;

#if (APP_AUDIO_REPLAY_TEST_ENABLE)
static bool app_audio_replay_asset_is_valid(
    const app_audio_replay_asset_t *asset);
static void app_audio_replay_print_ready_marker(
    const app_audio_replay_asset_t *asset);
#endif

cy_rslt_t app_audio_replay_init(void)
{
#if (!APP_AUDIO_REPLAY_TEST_ENABLE)
    return CY_RSLT_SUCCESS;
#else
    const app_audio_replay_asset_t *asset = app_audio_replay_get_selected_asset();

    memset(&app_audio_replay_state, 0, sizeof(app_audio_replay_state));

    if (!app_audio_replay_asset_is_valid(asset))
    {
        printf("[BOOT] replay_provider_invalid asset_name=%s chunk_id=%s "
               "sample_rate_hz=%lu sample_count=%lu\r\n",
               (NULL != asset) && (NULL != asset->asset_name) ?
                   asset->asset_name : "null",
               (NULL != asset) && (NULL != asset->chunk_id) ?
                   asset->chunk_id : "null",
               (unsigned long)((NULL != asset) ? asset->sample_rate_hz : 0u),
               (unsigned long)((NULL != asset) ? asset->sample_count : 0u));
        fflush(stdout);
        return CY_RSLT_TYPE_ERROR;
    }

    if ((APP_AUDIO_REPLAY_ASSET_ID_ANY != APP_AUDIO_REPLAY_ASSET_ID) &&
        (asset->asset_id != APP_AUDIO_REPLAY_ASSET_ID))
    {
        printf("[BOOT] replay_provider_asset_mismatch expected_id=%lu "
               "actual_id=%lu chunk_id=%s\r\n",
               (unsigned long)APP_AUDIO_REPLAY_ASSET_ID,
               (unsigned long)asset->asset_id,
               (NULL != asset->chunk_id) ? asset->chunk_id : "null");
        fflush(stdout);
        return CY_RSLT_TYPE_ERROR;
    }

    app_audio_replay_state.initialized = true;
    app_audio_replay_print_ready_marker(asset);
    return CY_RSLT_SUCCESS;
#endif
}

bool app_audio_replay_receive_block(app_pdm_pcm_block_t *block,
                                    TickType_t ticks_to_wait)
{
#if (!APP_AUDIO_REPLAY_TEST_ENABLE)
    (void)block;
    (void)ticks_to_wait;
    return false;
#else
    const app_audio_replay_asset_t *asset = app_audio_replay_get_selected_asset();
    uint32_t remaining_samples;
    uint32_t samples_to_copy;
    uint32_t i;

    if ((!app_audio_replay_state.initialized) || (NULL == block) ||
        (!app_audio_replay_asset_is_valid(asset)))
    {
        return false;
    }

    if (app_audio_replay_state.completed)
    {
        vTaskDelay((ticks_to_wait > 0) ? ticks_to_wait : pdMS_TO_TICKS(250u));
        return false;
    }

    if (!app_audio_replay_state.pacing_started)
    {
        app_audio_replay_state.last_wake_tick = xTaskGetTickCount();
        app_audio_replay_state.pacing_started = true;
    }
    else
    {
        vTaskDelayUntil(&app_audio_replay_state.last_wake_tick,
                        pdMS_TO_TICKS(APP_PDM_PCM_BLOCK_MS));
    }

    memset(app_audio_replay_state.stereo_block,
           0,
           sizeof(app_audio_replay_state.stereo_block));

    remaining_samples = asset->sample_count - app_audio_replay_state.mono_cursor;
    samples_to_copy = remaining_samples;
    if (samples_to_copy > APP_PDM_PCM_SAMPLES_PER_CH_PER_BLOCK)
    {
        samples_to_copy = APP_PDM_PCM_SAMPLES_PER_CH_PER_BLOCK;
    }

    for (i = 0u; i < samples_to_copy; i++)
    {
        int16_t mono = asset->pcm_mono[app_audio_replay_state.mono_cursor + i];
        uint32_t base = i * NUM_CHANNELS;

        app_audio_replay_state.stereo_block[base] = mono;
        app_audio_replay_state.stereo_block[base + 1u] = mono;
    }

    block->data = &app_audio_replay_state.stereo_block[0];
    block->sample_count = APP_PDM_PCM_BLOCK_SAMPLES;
    block->samples_per_channel = APP_PDM_PCM_SAMPLES_PER_CH_PER_BLOCK;
    block->block_index = 0u;
    block->sequence = app_audio_replay_state.block_sequence++;
    block->dropped_count = 0u;

    app_audio_replay_state.mono_cursor += samples_to_copy;
    if (app_audio_replay_state.mono_cursor >= asset->sample_count)
    {
        app_audio_replay_state.completed = true;
        printf("[BOOT] replay_provider_complete chunk_id=%s total_blocks=%lu\r\n",
               (NULL != asset->chunk_id) ? asset->chunk_id : "null",
               (unsigned long)asset->total_blocks);
        fflush(stdout);
    }

    return true;
#endif
}

void app_audio_replay_release_block(uint8_t block_index)
{
    (void)block_index;
}

bool app_audio_replay_get_runtime_info(app_audio_replay_runtime_info_t *info)
{
    const app_audio_replay_asset_t *asset = app_audio_replay_get_selected_asset();

    if ((NULL == info) || (NULL == asset))
    {
        return false;
    }

    memset(info, 0, sizeof(*info));
    info->asset_id = asset->asset_id;
    info->chunk_id = asset->chunk_id;
    info->asset_name = asset->asset_name;
    info->sample_rate_hz = asset->sample_rate_hz;
    info->total_samples = asset->sample_count;
    info->total_blocks = asset->total_blocks;
    info->duration_ms = asset->duration_ms;
    info->next_block_sequence = app_audio_replay_state.block_sequence;
    info->initialized = app_audio_replay_state.initialized;
    info->completed = app_audio_replay_state.completed;
    return true;
}

bool app_audio_replay_get_window_info(uint32_t input_sequence,
                                      app_audio_replay_window_info_t *info)
{
    uint32_t start_ms;

    if ((0u == input_sequence) || (NULL == info))
    {
        return false;
    }

    start_ms = (input_sequence - 1u) * 500u;

    memset(info, 0, sizeof(*info));
    info->replay_window_index = input_sequence;
    info->start_ms = start_ms;
    info->end_ms = start_ms + 1000u;
    return true;
}

const app_audio_replay_asset_t *app_audio_replay_get_selected_asset(void)
{
    return &g_app_audio_replay_selected_asset;
}

bool app_audio_replay_is_enabled(void)
{
#if (APP_AUDIO_REPLAY_TEST_ENABLE)
    return true;
#else
    return false;
#endif
}

void app_audio_replay_format_seconds(char *buffer,
                                     size_t buffer_size,
                                     uint32_t time_ms)
{
    uint32_t whole_seconds;
    uint32_t millis;

    if ((NULL == buffer) || (0u == buffer_size))
    {
        return;
    }

    whole_seconds = time_ms / 1000u;
    millis = time_ms % 1000u;
    (void)snprintf(buffer,
                   buffer_size,
                   "%lu.%03lu",
                   (unsigned long)whole_seconds,
                   (unsigned long)millis);
}

#if (APP_AUDIO_REPLAY_TEST_ENABLE)
static bool app_audio_replay_asset_is_valid(
    const app_audio_replay_asset_t *asset)
{
    if ((NULL == asset) || (NULL == asset->pcm_mono) ||
        (NULL == asset->chunk_id) || (NULL == asset->asset_name))
    {
        return false;
    }

    if ((16000u != asset->sample_rate_hz) || (0u == asset->sample_count) ||
        (0u == asset->total_blocks) || (0u == asset->duration_ms))
    {
        return false;
    }

    return true;
}

static void app_audio_replay_print_ready_marker(
    const app_audio_replay_asset_t *asset)
{
    char duration_sec[24];

    app_audio_replay_format_seconds(duration_sec,
                                    sizeof(duration_sec),
                                    asset->duration_ms);
    printf("[BOOT] replay_provider_ready chunk_id=%s asset_name=%s "
           "total_samples=%lu total_blocks=%lu duration_sec=%s\r\n",
           asset->chunk_id,
           asset->asset_name,
           (unsigned long)asset->sample_count,
           (unsigned long)asset->total_blocks,
           duration_sec);
    fflush(stdout);
}
#endif
