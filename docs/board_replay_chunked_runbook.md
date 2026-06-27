# Board Replay Chunked Runbook

## Scope

This runbook covers the implemented chunked board replay flow:

1. generate replay chunks in `ml/results/board_live_structured_20260626/replay_chunks`
2. stage one selected chunk into firmware
3. build either normal live firmware or replay firmware
4. flash one replay image
5. capture UART as real board replay evidence
6. merge chunk logs back onto the global replay timeline

This runbook does not change:

- B0-current model weights
- `board_htk_no_norm_v1`
- live firmware C1/C2 policy
- PC offline safe-3 / C2-lite-3 policy

## Implemented Files

- generator:
  `D:\e84_health_monitor\ml\scripts\generate_board_replay_chunks.py`
- firmware staging:
  `D:\e84_health_monitor\ml\scripts\install_board_replay_chunk_for_firmware.py`
- UART merge:
  `D:\e84_health_monitor\ml\scripts\merge_board_replay_uart_logs.py`
- replay provider:
  `D:\e84_health_monitor\firmware\proj_cm33_ns\source\app_audio_replay.c`
  `D:\e84_health_monitor\firmware\proj_cm33_ns\source\app_audio_replay.h`
- staged selected asset:
  `D:\e84_health_monitor\firmware\proj_cm33_ns\source\app_audio_replay_selected_asset.c`
  `D:\e84_health_monitor\firmware\proj_cm33_ns\source\app_audio_replay_selected_asset.h`

## Chunk Outputs

Generated output directory:

- `D:\cough_model_train\health_monitor_ai\results\board_live_structured_20260626\replay_chunks`

Generated artifacts:

- `chunk_000.wav` / `.pcm` / `_asset.c` / `_asset.h`
- `chunk_001.wav` / `.pcm` / `_asset.c` / `_asset.h`
- `...`
- `chunk_manifest.csv`
- `chunk_plan.md`

## Current Flash-Fit Boundary

Source and build evidence now show:

- a raw `int16 mono 16 kHz` C array at `65 s` overflowed `m33_nvm_C` by `129148` bytes
- a `60 s` selected chunk links successfully

Practical consequence:

- this Phase 0 implementation keeps replay assets at about `60 s`
- combined `shell_knock + table_knock` and broader mixed hard-negative packs were split into buildable single-segment chunks
- if longer combined chunks are required later, that will need a separate storage/layout review before implementation

## Generate Chunks

Command:

```powershell
python D:\e84_health_monitor\ml\scripts\generate_board_replay_chunks.py
```

Current generated chunk set:

- `chunk_000`: `normal_cough` sanity
- `chunk_001`: `weak_cough`
- `chunk_002`: `throat_clear`
- `chunk_003`: `multi_speaker_speech`
- `chunk_004`: `cable_touch`
- `chunk_005`: `shell_knock`
- `chunk_006`: `table_knock`
- `chunk_007`: `slipper_walk` fallback hard negative

## Stage One Selected Chunk

Install `chunk_000` into firmware:

```powershell
python D:\e84_health_monitor\ml\scripts\install_board_replay_chunk_for_firmware.py --chunk-id chunk_000
```

This rewrites only the selected-asset pair:

- `D:\e84_health_monitor\firmware\proj_cm33_ns\source\app_audio_replay_selected_asset.h`
- `D:\e84_health_monitor\firmware\proj_cm33_ns\source\app_audio_replay_selected_asset.c`

The replay build must also pass the matching numeric asset id:

- `chunk_000` -> `APP_AUDIO_REPLAY_ASSET_ID=0`
- `chunk_001` -> `APP_AUDIO_REPLAY_ASSET_ID=1`
- `chunk_002` -> `APP_AUDIO_REPLAY_ASSET_ID=2`
- `...`

## Build Commands

Default live build:

```powershell
& 'D:\e84_health_monitor\tools\build_firmware.ps1' -Profile inference_default -Action build
```

Replay build example for `chunk_000` with explicit B0-current:

```powershell
& 'D:\e84_health_monitor\tools\build_firmware.ps1' -Profile inference_default -Action build -ExtraMakeArgs @(
  'APP_AUDIO_REPLAY_TEST_ENABLE=1',
  'APP_AUDIO_MODEL_SELECT=5',
  'APP_AUDIO_REPLAY_ASSET_ID=0'
)
```

## Flash Command Template

Wrapper-supported flash path:

```powershell
& 'D:\e84_health_monitor\tools\build_firmware.ps1' -Profile inference_default -Action qprogram -ExtraMakeArgs @(
  'APP_AUDIO_REPLAY_TEST_ENABLE=1',
  'APP_AUDIO_MODEL_SELECT=5',
  'APP_AUDIO_REPLAY_ASSET_ID=0'
)
```

## UART Run Notes

Replay mode uses the normal audio-preprocess runtime path, so the current UART
default is the normal inference baud:

- `115200 8N1`

Log naming rule:

- `board_replay_chunk_000_uart.log`
- `board_replay_chunk_001_uart.log`
- `board_replay_chunk_002_uart.log`
- `...`

Expected replay markers:

- `[BOOT] replay_provider_ready chunk_id asset_name total_samples total_blocks duration_sec`
- `[MIC_WIN] ... replay_mode=1 chunk_id replay_window_index replay_time_sec`
- `[C1_CAND] ... replay_mode=1 chunk_id replay_window_index_start/end replay_time_sec_start/end`
- `[C1_EVENT] ... replay_mode=1 chunk_id replay_window_index_start/end replay_time_sec_start/end`
- `[C2_EVENT] ... replay_mode=1 chunk_id replay_window_index_start/end replay_time_sec_start/end`
- `[COUGH_EDGE] ... replay_mode=1 chunk_id replay_window_index_start/end replay_time_sec_start/end`

End-of-chunk behavior:

- provider prints `[BOOT] replay_provider_complete ...`
- provider stops producing new PCM blocks
- it does not loop fake data

## Merge Logs Back to Global Replay

Place real board logs under a directory such as:

- `D:\cough_model_train\health_monitor_ai\results\board_live_structured_20260626\board_replay_uart_logs`

Merge command:

```powershell
python D:\e84_health_monitor\ml\scripts\merge_board_replay_uart_logs.py `
  --chunk-dir D:\cough_model_train\health_monitor_ai\results\board_live_structured_20260626\replay_chunks `
  --log-dir D:\cough_model_train\health_monitor_ai\results\board_live_structured_20260626\board_replay_uart_logs `
  --out-dir D:\cough_model_train\health_monitor_ai\results\board_live_structured_20260626 `
  --offline-summary D:\cough_model_train\health_monitor_ai\results\board_live_structured_20260626\event_summary_by_subtype.csv
```

Merged outputs:

- `board_replay_windows_merged.csv`
- `board_replay_events_merged.csv`
- `board_replay_event_summary_by_subtype.csv`
- `board_replay_vs_offline_comparison.md`

Important boundary:

- merged board replay logs are real board replay evidence
- they are not proof that firmware policy equals PC safe-3 / C2-lite-3
