# Board Replay / PCM Injection Design

Update:

- the design below has now been implemented in a chunked replay form
- operational details live in
  `firmware/docs/board_replay_chunked_runbook.md`
- build evidence shows the current raw-C-array flash-fit ceiling is about
  `60 s` per selected replay chunk under the present `m33_nvm_C` layout

## Goal

Design a default-off firmware replay mode that bypasses live PDM hardware
capture but still reuses the real downstream path:

- 16 kHz mono source material
- 1.0 s window
- 0.5 s hop
- 40-bin HTK Log-Mel
- `board_htk_no_norm_v1`
- CM55 B0-current inference
- existing firmware C1/C2 logic
- existing UART event chain

This is a design document only. It does not change firmware policy and does not
claim parity with the PC safe-3 / lite-3 policy.

## Recommended Injection Point

Inject at the narrow boundary inside `app_audio_preprocess_task()` where the
task currently blocks on `app_pdm_pcm_receive_block()`.

Primary source anchor:

- `proj_cm33_ns/source/app_audio_preprocess/app_audio_preprocess.c:413-471`

Why this insertion point is preferred:

1. It is the narrowest boundary between live PDM hardware capture and the real
   preprocess pipeline.
2. Everything after that point can stay unchanged:
   - channel select / mono mix
   - ring buffer accumulation
   - `1.0 s` / `0.5 s` sliding windows
   - `board_htk_no_norm_v1`
   - shared-memory publish
   - CM55 inference
   - `app_model_result_monitor`
   - UART markers
3. It does not change the math definition of `board_htk_no_norm_v1`.
4. It does not require any CM55 model change.

## Compile-Time Control

Add a new shared build macro:

- file: `shared/app_build_config.h`
- macro: `APP_AUDIO_REPLAY_TEST_ENABLE`
- default: `0`

Recommended constraints:

- only valid when `APP_RUNTIME_MODE == APP_RUNTIME_MODE_AUDIO_PREPROCESS`
- compile-time error if enabled together with CSV export mode
- compile-time error if enabled with any future incompatible capture mode

Recommended behavior:

- `APP_AUDIO_REPLAY_TEST_ENABLE=0`
  - keep the current live PDM path unchanged
- `APP_AUDIO_REPLAY_TEST_ENABLE=1`
  - do not start `app_pdm_pcm_task_init()` in `main.c`
  - initialize a replay provider instead
  - let `app_audio_preprocess_task()` read 10 ms blocks from replay provider
    rather than from the PDM queue

Relevant task wiring anchors:

- `shared/app_build_config.h:14-29`
- `proj_cm33_ns/main.c:333-395`

## Replay Provider Module

Add a dedicated CM33-side module, for example:

- `proj_cm33_ns/source/app_audio_replay/app_audio_replay.h`
- `proj_cm33_ns/source/app_audio_replay/app_audio_replay.c`

Keep the external shape close to `app_pdm_pcm` so downstream code stays small:

- `cy_rslt_t app_audio_replay_init(void);`
- `bool app_audio_replay_receive_block(app_pdm_pcm_block_t *block,
  TickType_t ticks_to_wait);`
- `void app_audio_replay_release_block(uint8_t block_index);`

Recommended internal model:

- store a static replay asset in flash
- expose one synthetic `app_pdm_pcm_block_t` at a time
- maintain a small local ring of generated 10 ms blocks so the descriptor shape
  stays compatible with the current preprocess loop

Compatibility target:

- reuse `app_pdm_pcm_block_t` exactly as defined in
  `proj_cm33_ns/source/app_pdm_pcm/app_pdm_pcm.h:144-163`

That descriptor already gives us the required metadata:

- `data`
- `sample_count`
- `samples_per_channel`
- `block_index`
- `sequence`
- `dropped_count`

## PCM Organization

Preferred first input mode:

- mini replay PCM converted to a generated C array and compiled into firmware

Reason:

- lowest system risk
- fastest path to proving end-to-end replay through the real firmware chain
- no UART/USB framing, host-stream timing, or large buffering work needed yet

Current preprocess expectation:

- `app_audio_preprocess` consumes 10 ms stereo PCM blocks from `app_pdm_pcm`
- source anchor:
  `proj_cm33_ns/source/app_audio_preprocess/app_audio_preprocess.h:6-15`

Mini replay source material is 16 kHz mono, so the provider should synthesize
stereo blocks by duplicating mono samples:

- `L = mono`
- `R = mono`

Why this is safe for Phase 0:

- `SELECT_BEST` sees effectively equal channels
- average mix stays equal to the mono source
- downstream mono content remains unchanged
- no math change is introduced into `board_htk_no_norm_v1`

Each synthetic block should preserve existing descriptor semantics:

- `sample_count = 320`
- `samples_per_channel = 160`
- `sequence` monotonic
- `dropped_count = 0`

## Mini Replay Asset Strategy

Do not start with the full `42.7 min` replay.

Phase 0 target:

- build a stitched mini replay pack that covers:
  - `normal_cough`
  - `weak_cough`
  - `throat_clear`
  - `multi_speaker_speech`
  - `shell_knock`
  - `table_knock`
  - `cable_touch`
  - `quiet gap`

Risk that must be called out explicitly:

- raw `int16 mono 16k` footprint is about:
  - `~3.8 MiB` for `2 min`
  - `~9.6 MiB` for `5 min`
- a direct C array may exceed practical flash or link budget depending on the
  active image layout

Recommended staged rollout:

1. Phase 0
   - start with a shorter stitched subset purely to validate the replay path
   - confirm build/link/map viability first
2. Phase 1
   - expand toward the requested `2-5 min` pack if flash budget allows
   - if link pressure becomes too high, split into chunked arrays and keep the
     same provider API
3. Next priority only after that
   - UART/USB streaming

UART/USB streaming is intentionally not first because it adds transport timing,
framing, and buffering complexity before the replay chain itself is proven.

## Suggested Generated Asset Files

Keep generated replay assets separate from handwritten code. For example:

- `proj_cm33_ns/source/app_audio_replay/app_audio_replay_assets.h`
- `proj_cm33_ns/source/app_audio_replay/app_audio_replay_assets.c`

Recommended metadata to generate with the asset:

- sample rate
- total sample count
- total synthetic 10 ms block count
- clip name / build note
- optional segment table for the stitched mini pack

This keeps the provider simple and makes later provenance checks easier.

## UART Design

Do not replace the current marker family. Extend it.

Required marker family:

- `[MIC_WIN]`
- `[C1_CAND]`
- `[C1_EVENT]`
- `[C2_EVENT]`
- `[COUGH_EDGE]`

Recommended replay-only fields:

- `[MIC_WIN]`
  - `replay_mode=1`
  - `replay_window_index`
  - `replay_time_sec`
- `[C1_CAND]`, `[C1_EVENT]`, `[C2_EVENT]`, `[COUGH_EDGE]`
  - `replay_window_index_start`
  - `replay_window_index_end`
  - `replay_time_sec_start`
  - `replay_time_sec_end`

Most economical implementation:

- reuse `input_sequence` as `replay_window_index`
- derive replay time as:
  - `replay_time_sec = (input_sequence - 1) * 0.5`

Why this is preferred:

- no shared-memory contract expansion required
- window indexing already flows through the existing path
- easy to align with the offline replay report

Important boundary:

- these would be real board UART lines for replay mode, not the PC offline
  pseudo UART
- the tag names can stay the same because replay mode is explicitly marked by
  `replay_mode=1`

## Boot and Task Flow

Recommended control flow when replay mode is enabled:

1. boot CM33 as usual
2. initialize UART and shared-memory path as usual
3. skip `app_pdm_pcm_task_init()`
4. call `app_audio_replay_init()`
5. create `app_audio_preprocess_task()`
6. create `app_model_result_monitor_task()`
7. preprocess task consumes replay provider blocks and publishes live features
8. CM55 inference and existing event logic run unchanged

This preserves the real post-capture chain while keeping live PDM mode
untouched when replay is disabled.

## Validation Order

Recommended validation sequence:

1. Boot path
   - confirm a replay init boot marker such as `[BOOT] replay_provider_ready`
2. Window progression
   - confirm `[MIC_WIN]` appears
   - confirm `replay_window_index` increments by one
   - confirm `replay_time_sec` increments by `0.5`
3. Event chain
   - inspect `[C1_CAND]`
   - inspect `[C1_EVENT]`
   - inspect `[C2_EVENT]`
   - inspect `[COUGH_EDGE]`
4. Offline comparison
   - align replay-mode board UART against
     `ml/results/board_live_structured_20260626/offline_replay_eval_report.md`
   - compare by subtype, not just by aggregate counts

## Explicit Non-Goals for Phase 0

- do not modify B0-current model weights
- do not modify `board_htk_no_norm_v1`
- do not change firmware C1/C2 policy in the same patch
- do not implement full-length `42.7 min` storage first
- do not introduce UART/USB streaming first
- do not treat replay-mode board results as proof of PC-policy parity

## Recommended Next Implementation Order

1. Implement `APP_AUDIO_REPLAY_TEST_ENABLE` and replay provider only.
2. Validate that replayed PCM traverses the real firmware preprocess/inference/
   event chain.
3. Compare replay-mode UART against the existing offline replay report.
4. Only after that, if still needed, prepare a separate safe-3 / lite-3 parity
   proposal for firmware.

This keeps two questions separate:

- can the board replay the real chain reliably?
- is the firmware event policy the same as the PC replay package?
