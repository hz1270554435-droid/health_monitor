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
} app_audio_replay_window_info_t;

cy_rslt_t app_audio_replay_init(void);
bool app_audio_replay_receive_block(app_pdm_pcm_block_t *block,
                                    TickType_t ticks_to_wait);
void app_audio_replay_release_block(uint8_t block_index);
bool app_audio_replay_get_runtime_info(app_audio_replay_runtime_info_t *info);
bool app_audio_replay_get_window_info(uint32_t input_sequence,
                                      app_audio_replay_window_info_t *info);
const app_audio_replay_asset_t *app_audio_replay_get_selected_asset(void);
bool app_audio_replay_is_enabled(void);
void app_audio_replay_format_seconds(char *buffer,
                                     size_t buffer_size,
                                     uint32_t time_ms);

#if defined(__cplusplus)
}
#endif

#endif /* APP_AUDIO_REPLAY_H */
