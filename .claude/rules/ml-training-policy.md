# ML Training Policy

This project is MIC-first. The current ML path is `cough` vs `non_cough`.

## Current MIC Route

1. Freeze `audio_labels_v3`.
2. Verify grouped `person_id` split.
3. Run tiny dataset check.
4. Run preprocessing smoke test.
5. Run one-epoch training smoke test.
6. Train DS-CNN / small CNN v3 baseline.
7. Evaluate.
8. Run threshold sweep.
9. Report FP/hour.
10. Only then discuss teacher models or distillation.

## Board-Compatible Preprocessing

Training preprocessing must match the CM33 contract:

- 16 kHz sample rate.
- 1 second window.
- Existing overlap config only.
- HTK Mel.
- No normalization unless the project has a clear PC/board contract.

Before changing preprocessing, inspect board code in `proj_cm33_ns/source/app_audio_preprocess/` and shared metadata in `shared/`.

## Required Checks

- Tiny dataset check catches label and shape errors.
- Preprocessing smoke test confirms feature shape/range.
- One-epoch training smoke test catches training-loop and export issues.
- Evaluation reports metrics on held-out `person_id` groups.
- Threshold sweep chooses operating points.
- FP/hour estimates false alarms on non-cough streams.

## Deferred Work

- OPERA, YAMNet, and HeAR may be used later as teachers, mining tools, or research baselines.
- Do not use them to bypass `audio_labels_v3` or the board-compatible v3 baseline.

## Reporting

- Do not present smoke-test success as final model quality.
- Include data version, label version, split seed, config path, and metric script path.
