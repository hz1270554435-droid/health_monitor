/*******************************************************************************
 * File Name : app_audio_replay.h
 *
 * Description : Default-off board replay / PCM injection provider.
 *******************************************************************************/

#ifndef APP_AUDIO_REPLAY_H
#define APP_AUDIO_REPLAY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "app_build_config.h"
#include "app_pdm_pcm.h"

#if defined(__cplusplus)
extern "C" {
#endif

#define APP_AUDIO_REPLAY_ASSET_ID_ANY             (0xFFFFFFFFu)

#define APP_AUDIO_REPLAY_ASSET_MODE_PCM                 (0u)
#define APP_AUDIO_REPLAY_ASSET_MODE_FEATURE_SEQUENCE    (1u)

#define APP_AUDIO_REPLAY_FEATURE_PHASE_NONE      (0u)
#define APP_AUDIO_REPLAY_FEATURE_PHASE_PREV2     (1u)
#define APP_AUDIO_REPLAY_FEATURE_PHASE_PREV1     (2u)
#define APP_AUDIO_REPLAY_FEATURE_PHASE_CURRENT   (3u)

typedef struct
{
    uint16_t reference_tick_id;
    uint8_t phase;
    uint8_t reserved0;
    uint32_t window_start_ms;
    uint32_t window_end_ms;
    uint32_t prev2_start_ms;
    uint32_t prev1_start_ms;
    uint32_t current_start_ms;
    uint32_t current_end_ms;
} app_audio_replay_feature_entry_t;

typedef struct
{
    uint8_t phase;
    uint8_t reference_tick_id;
    uint8_t reserved0;
    uint8_t reserved1;
    uint32_t window_start_ms;
    uint32_t window_end_ms;
} app_audio_replay_feature_publish_info_t;

typedef struct
{
    uint32_t asset_id;
    const char *chunk_id;
    const char *asset_name;
    uint32_t sample_rate_hz;
    uint32_t sample_count;
    uint32_t total_blocks;
    uint32_t duration_ms;
    const int16_t *pcm_mono;
    uint8_t asset_mode;
    uint8_t reserved0;
    uint16_t reserved1;
    uint32_t feature_entry_count;
    uint32_t feature_stride_floats;
    const app_audio_replay_feature_entry_t *feature_entries;
    const float *feature_payload_f32;
} app_audio_replay_asset_t;

typedef struct
{
    uint32_t asset_id;
    const char *chunk_id;
    const char *asset_name;
    uint32_t sample_rate_hz;
    uint32_t total_samples;
    uint32_t total_blocks;
    uint32_t duration_ms;
    uint32_t next_block_sequence;
    bool initialized;
    bool completed;
} app_audio_replay_runtime_info_t;

typedef struct
{
    uint32_t replay_window_index;
    uint32_t start_ms;
    uint32_t end_ms;
    uint32_t reference_tick_id;
    uint32_t prev2_start_ms;
    uint32_t prev1_start_ms;
    uint32_t current_start_ms;
    uint32_t current_end_ms;
    uint8_t phase;
} app_audio_replay_window_info_t;

cy_rslt_t app_audio_replay_init(void);
bool app_audio_replay_receive_block(app_pdm_pcm_block_t *block,
                                    TickType_t ticks_to_wait);
bool app_audio_replay_receive_feature(float *feature,
                                      uint32_t feature_capacity_floats,
                                      app_audio_replay_feature_publish_info_t *info,
                                      TickType_t ticks_to_wait);
void app_audio_replay_release_block(uint8_t block_index);
bool app_audio_replay_get_runtime_info(app_audio_replay_runtime_info_t *info);
bool app_audio_replay_get_window_info(uint32_t input_sequence,
                                      app_audio_replay_window_info_t *info);
const app_audio_replay_asset_t *app_audio_replay_get_selected_asset(void);
bool app_audio_replay_is_enabled(void);
bool app_audio_replay_is_feature_sequence_mode(void);
void app_audio_replay_format_seconds(char *buffer,
                                     size_t buffer_size,
                                     uint32_t time_ms);

#if defined(__cplusplus)
}
#endif

#endif /* APP_AUDIO_REPLAY_H */
