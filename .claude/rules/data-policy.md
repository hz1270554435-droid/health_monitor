# Data Policy

These rules apply to raw captures, labels, processed features, model artifacts, logs, and experiment results.

## Raw Data

- Do not modify `data/raw/`.
- Do not rewrite, normalize, trim, or relabel raw files in place.
- Do not commit new raw audio, raw radar, or raw UART byte captures.
- Existing root-level capture artifacts such as `mic.csv`, `radar.csv`, `sensor_export.csv`, and `serial_raw.log` should not be expanded or regenerated without explicit approval.

## Labels

- Label changes must be traceable.
- Human-reviewed hard negatives must be consolidated into `audio_labels_v3` before v3 training.
- Label files should include enough information to recover source sample, label, review status, and reviewer decision when available.
- Do not silently drop ambiguous samples; mark them explicitly.

## Splits

- Train/val/test splits must be grouped by `person_id`.
- The same `person_id` must not appear in more than one split.
- Split generation must be deterministic from config and seed.

## Generated Artifacts

Do not commit large generated artifacts:

- processed audio features,
- radar features,
- trained models,
- exported model binaries,
- evaluation result dumps,
- long raw logs,
- cache directories.

Generated artifacts must be reproducible from `configs/*.yaml` or a documented equivalent config.

## Experiment Records

- Record config path, commit/hash if available, data version, label version, split seed, model version, and output directory.
- Separate smoke-test results from full evaluation results.
