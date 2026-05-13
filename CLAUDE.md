# PSoC Edge E84 Respiratory Health Monitor

This repository is a competition/school prototype for edge AI respiratory health monitoring on Infineon PSoC Edge E84. Treat it as an embedded + ML co-design project: small, reproducible steps matter more than impressive but unverifiable experiments.

## Project Overview

- Target board: Infineon PSoC Edge E84, especially `KIT_PSE84_EVAL` / `KIT_PSE84_AI`.
- Chip architecture: Cortex-M33, Cortex-M55, Ethos-U55 / NPU, shared-memory handoff between cores.
- Software stack: ModusToolbox, FreeRTOS, C/C++, Python 3.10+ tooling for capture, preprocessing, and training.
- Sensors:
  - MIC: onboard microphones / audio capture for `cough` vs `non_cough`.
  - Radar: HLK-LD6002 breathing/heart-rate radar over UART.
- Current stage:
  - MIC-first baseline is the main path.
  - v2 audio model has board-side smoke-test support.
  - Real-time inference still needs end-to-end integration and debugging.
  - Negative samples have been scanned and hard samples need human review consolidation.
  - Next ML step is to freeze `audio_labels_v3`, then train a board-compatible v3 baseline.

## Current Roadmap

1. Freeze `audio_labels_v3` from reviewed cough/non-cough labels and hard negatives.
2. Train board-compatible MIC v3 baseline using DS-CNN or small CNN.
3. Verify PC/board preprocessing and inference consistency.
4. Stabilize LD6002 radar capture, parser, CSV/features, and quality scoring.
5. Build a simple explainable fusion rule state machine.
6. Consider a small MLP fusion model only after rule behavior is documented.
7. Defer OPERA / YAMNet / HeAR distillation until the board-compatible baseline is stable.

## Non-Negotiable Rules

- Do not modify `data/raw/`.
- Do not commit raw audio, raw radar, processed features, trained models, result dumps, or other large experiment artifacts.
- Split train/val/test by `person_id`; the same person must not appear in multiple splits.
- Generated training artifacts must be reproducible from `configs/*.yaml` or an explicitly documented equivalent config.
- Hard samples from human review must be consolidated into `audio_labels_v3` before training v3.
- Do not skip label freezing and jump directly to OPERA or other distillation work.
- Radar models must use parsed radar CSV/features, not raw UART bytes.
- Firmware changes touching FreeRTOS, ISR, DMA, ring buffers, shared memory, CM33/CM55 IPC, cache behavior, startup, linker, clock, power, or secure boot require architecture review first.
- Prefer existing `cy_rslt_t` / project-local error handling patterns. Do not introduce a new error system casually.

## MIC Training Contract

- Current task: binary `cough` / `non_cough`.
- Preferred baseline: board-compatible DS-CNN or small CNN v3.
- Preprocessing must match CM33 contract:
  - 16 kHz audio.
  - 1 second window; overlap only as configured.
  - HTK Mel scale.
  - No normalization unless there is a clear project contract saying otherwise.
- Training task completion must include:
  - tiny dataset check,
  - preprocessing smoke test,
  - one-epoch training smoke test,
  - evaluation,
  - threshold sweep,
  - false positives per hour.

## Radar Contract

- LD6002 UART parser must handle SOF, ID, LEN, TYPE, HEAD_CKSUM, DATA, and DATA_CKSUM.
- Header fields are big-endian; DATA payload values are little-endian.
- Known frame types include:
  - `0x0A13`: phase,
  - `0x0A14`: breathing rate,
  - `0x0A15`: heart rate,
  - `0x0A16`: distance.
- First radar stage is stable capture, parser correctness, quality scoring, and simple feature baselines.
- Do not start with a complex deep radar model.

## Fusion Contract

- Start with an explainable rule state machine.
- Candidate inputs:
  - `mic_cough_prob`
  - `radar_rr`
  - `radar_hr`
  - `radar_presence`
  - `radar_motion_score`
  - `radar_quality`
  - `audio_quality`
- Initial outputs:
  - `normal`
  - `attention`
  - `warning`
- Fusion must remain easy to explain in a competition defense.

## Build/Test Commands

These are based on files currently present in this repository. Do not invent new commands without checking the repo first.

Firmware build/program:

```powershell
make build
make build TOOLCHAIN=GCC_ARM
make program
make program TOOLCHAIN=GCC_ARM
```

Board capture helpers:

```powershell
tools\run_capture_csv.bat --list-ports
tools\run_capture_csv.bat --port COM7 --baud 2000000
python tools\capture_csv.py --help
```

Known project utilities:

```powershell
python tools\compare_mel_consistency.py --help
```

TODO commands to identify before use:

- TODO: identify firmware clean/rebuild command for this local ModusToolbox setup.
- TODO: identify board smoke-test command and expected UART markers.
- TODO: identify audio label-freeze command for `audio_labels_v3`.
- TODO: identify audio preprocessing smoke-test command.
- TODO: identify one-epoch training smoke-test command.
- TODO: identify evaluation, threshold sweep, and FP/hour commands.
- TODO: identify radar parser unit-test command.

## Agent Routing Guide

- Use `embedded-architect` for high-risk embedded design, CM33/CM55 ownership, FreeRTOS task topology, ISR/DMA/ring buffer/shared-memory/cache questions, and error-code architecture.
- Use `firmware-worker` only for clear, local firmware edits after scope is known.
- Use `radar-protocol-engineer` for LD6002 parser, checksum, endian conversion, radar CSV/features, and radar quality metrics.
- Use `audio-ml-engineer` for `audio_labels_v3`, MIC data processing, board-compatible DS-CNN/small CNN training, threshold sweep, and FP/hour.
- Use `ml-pipeline-reviewer` before full training, model export, or accepting reported metrics.
- Use `board-debugger` for ModusToolbox build/program/debug, UART logs, CM33 preprocessing to CM55 inference flow, and PC/board consistency.
- Use `log-analyzer` for long build, serial, Python, or evaluation logs.
- Use `code-reviewer` before commits or before merging firmware/ML workflow changes.
- Use `docs-writer` for README, experiment notes, weekly reports, defense material, and workflow docs.
- Use `learning-tutor` when learning a concept from this project codebase.

## Safe Development Workflow

1. Plan first and list files before editing.
2. Ask `embedded-architect` for high-risk embedded changes.
3. Make a small patch with a narrow file scope.
4. Run the smallest relevant smoke test.
5. Send long logs to `log-analyzer`.
6. Send ML pipelines to `ml-pipeline-reviewer` before full training or export.
7. Send final changes to `code-reviewer`.
8. Commit only after commands, logs, and docs are aligned.

## Existing Documentation

- `README.md` is still the upstream Infineon PDM-to-I2S example README and contains valid ModusToolbox setup/build context.
- `docs/design_and_implementation.md` and `docs/using_the_code_example.md` contain upstream design and CLI build/program instructions.
- `AGENTS.md` was not present when this file was created. If one is added later, merge its non-conflicting rules here or reference it explicitly.
