# Firmware vs PC Event Policy Audit

## Scope

This audit is read-only. It compares the current firmware MIC event chain in
`firmware/` against the known offline replay policy used by:

- `D:\cough_model_train\health_monitor_ai\results\board_live_structured_20260626\offline_replay_eval_report.md`
- `D:\cough_model_train\health_monitor_ai\exports\v3_3_2_b0_current\threshold_policy.json`
- `D:\cough_model_train\health_monitor_ai\exports\v3_3_2_b0_current\event_policy_c1_safe3.json`
- `D:\cough_model_train\health_monitor_ai\exports\v3_3_2_b0_current\event_policy_c2_lite.json`

This document does not claim anything about a specific flashed board image
unless separate build/flash provenance is supplied.

## Firmware Source Evidence

### B0-current selector and contract anchor

- `shared/app_audio_deployment_config.h:7` defines
  `APP_AUDIO_MODEL_SELECT_V3_3_2_B0_CURRENT` as `5`.
- `shared/app_audio_deployment_config.h:34-39` defines the B0-current deployment
  contract as:
  - model version `v3_3_1_planA_A5`
  - frontend `board_htk_no_norm_v1`
  - class order `non_cough,cough`
  - cough threshold `0.10f`
- `shared/app_audio_deployment_config.h:75-91` maps selector `5` to the active
  model metadata and header.

### Default source build does not prove B0-current

- `proj_cm33_ns/Makefile:77-93` defaults `APP_BUILD_PROFILE` to
  `inference_default` and defaults `APP_AUDIO_MODEL_SELECT` to `3`.
- `proj_cm33_ns/Makefile:172-175` shows `inference_default` only fixes runtime
  mode and radar-test behavior. It does not override `APP_AUDIO_MODEL_SELECT` to
  `5`.
- `proj_cm55/Makefile:192` also defaults `APP_AUDIO_MODEL_SELECT` to `3`.
- `proj_cm55/Makefile:222-240` shows selector-specific source inclusion, with
  selector `5` only enabled when `APP_AUDIO_MODEL_SELECT=5` is passed in.

Conclusion from source only: the default source build path is not B0-current.

### Frontend parity for B0-current

- `proj_cm33_ns/source/app_audio_preprocess/app_audio_preprocess.h:99-126`
  defines `APP_AUDIO_PREPROCESS_FRONTEND_PROFILE_BOARD_HTK_NO_NORM_V1` and makes
  it the default frontend profile for selector `5`.
- `proj_cm33_ns/source/app_audio_preprocess/app_audio_preprocess.c:1557-1565`
  states that `board_htk_no_norm_v1` stops at `power_to_db(ref=max, top_db=80)`
  and does not perform feature standardization.
- `proj_cm33_ns/source/app_audio_preprocess/app_audio_preprocess.c:1624-1643`
  explicitly disables RMS gain and feature z-score when the active frontend
  profile is `BOARD_HTK_NO_NORM_V1`.
- `proj_cm33_ns/source/app_audio_preprocess/app_audio_preprocess.c:1135-1151`
  keeps the live windowing contract at `1.0 s` window and `0.5 s` hop.

Conclusion: for selector `5`, the firmware frontend contract matches
`board_htk_no_norm_v1` and satisfies the no-normalization requirement.

### Current firmware live event chain

- `proj_cm33_ns/source/app_model_result_monitor/app_model_result_monitor.c:79-97`
  defines the current live event macros:
  - `APP_MODEL_EVENT_THRESHOLD = 0.95f`
  - `APP_MODEL_EVENT_MIN_HITS = 2u`
  - `APP_MODEL_EVENT_WINDOW_MS = 1200u`
  - `APP_MODEL_EVENT_COOLDOWN_MS = 1500u`
  - `APP_MODEL_EVENT_MIN_ENERGY = 0.0f`
- `proj_cm33_ns/source/app_model_result_monitor/app_model_result_monitor.c:1220-1395`
  implements the entire live C1/C2 chain:
  - candidate starts only when `scores[2] >= APP_MODEL_EVENT_THRESHOLD`
  - confirm occurs when hit count reaches `APP_MODEL_EVENT_MIN_HITS`
  - cooldown is event-to-event only
  - rejection path is only `candidate_energy < APP_MODEL_EVENT_MIN_ENERGY`
- `proj_cm33_ns/source/app_model_result_monitor/app_model_result_monitor.c:920-922`
  and `759-761` show that the event chain consumes:
  - `scores[2]` as `cough_prob`
  - `scores[3]` as `energy`

Conclusion: current firmware live logic is a simpler threshold/min-hits/window/
cooldown/low-energy chain, not the PC safe-3 + lite-3 chain.

### UART marker entry points

- `[MIC_WIN]` prints in
  `proj_cm33_ns/source/app_model_result_monitor/app_model_result_monitor.c:1625-1663`
- `[C1_CAND]`, `[C1_EVENT]`, `[C2_EVENT]` print in
  `proj_cm33_ns/source/app_model_result_monitor/app_model_result_monitor.c:1304-1385`
- `[COUGH_EDGE]` is printed on accepted event path from the result-monitor main
  loop in the same module, referenced at `464-507`

The tag set matches the offline pseudo UART naming, but source evidence shows
that the decision policy behind those tags is different.

## Known PC Offline Export Policy

From the current PC export package used by the offline replay report:

- `threshold_policy.json`
  - `raw_model_threshold = 0.10`
  - `event_threshold = 0.10`
- `event_policy_c1_safe3.json`
  - `candidate_threshold = 0.10`
  - `peak_threshold = 0.20`
  - `min_positive_windows = 2`
  - `max_event_duration_sec = 2.5`
  - `context_windows = 5`
  - `background_median_reject = 0.06`
  - `min_peakiness = 0.07`
  - `cooldown_sec = 2.0`
  - `merge_gap_sec = 1.0`
- `event_policy_c2_lite.json`
  - duration / short-burst / attack / peakiness based acceptance-rejection rules
  - not just a low-energy gate

## Parity Table

| Item | Firmware source | PC offline replay policy | Parity |
|---|---|---|---|
| B0 model select | Selector `5` exists in `shared/app_audio_deployment_config.h`, but default CM33/CM55 Makefiles still default to `APP_AUDIO_MODEL_SELECT?=3` | Offline replay explicitly used B0-current export package | No |
| Frontend | Selector `5` maps to `board_htk_no_norm_v1`; no RMS gain, no z-score | `board_htk_no_norm_v1` | Yes for selector `5` |
| Class order | `non_cough,cough` | `non_cough,cough` | Yes |
| Raw/event threshold | Live event threshold macro is `0.95f` | `0.10` | No |
| C1 candidate threshold | Enter candidate only when `scores[2] >= 0.95` | Candidate threshold `0.10`, peak threshold `0.20` | No |
| C1 confirm rule | `min_hits=2` within `1200 ms` | `min_positive_windows=2` plus safe-3 context/peakiness/background rules | No |
| C1 cooldown | `1500 ms` | `2.0 s` and `merge_gap_sec=1.0` | No |
| C1 merge / context / peakiness | Not present in live source | Present in safe-3 | No |
| C2 reject/accept rules | Only `low_energy`; default floor is `0.0f` | duration / attack / short-burst / peakiness rules | No |
| UART markers | `[MIC_WIN]`, `[C1_CAND]`, `[C1_EVENT]`, `[C2_EVENT]`, `[COUGH_EDGE]` exist in source | Offline pseudo UART intentionally mirrors these tags | Tag names yes, policy no |

## Conclusions

1. The current source default build is not B0-current.
   `inference_default` does not automatically switch `APP_AUDIO_MODEL_SELECT`
   from `3` to `5`.

2. The current firmware event logic is not `C1-A5-safe-3 + C2-lite-3`.
   The live source still runs a simpler threshold/min-hits/window/cooldown gate
   with an optional low-energy reject step.

3. The current firmware tag names are policy-compatible in format only.
   They are not proof that the board is already running the same logic as the PC
   offline replay package.

4. Without an explicit build override such as `APP_AUDIO_MODEL_SELECT=5` and
   without flashed-image provenance, it is not valid to claim that a board is
   currently deployed with the same B0-current + C1-safe-3 + C2-lite-3 policy
   used by the offline replay.

## Practical Readout

- `board_htk_no_norm_v1` contract: source evidence says yes
- current firmware event policy parity with PC replay policy: no
- default `inference_default` source build equals B0-current: no
- actual deployed board image proof available in this audit: no
