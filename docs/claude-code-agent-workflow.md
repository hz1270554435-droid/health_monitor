# Claude Code Agent Workflow

This workflow is tailored to the PSoC Edge E84 respiratory health monitoring project. It is meant for learning, development, and competition delivery.

## 1. Learning Workflow

### FreeRTOS

Goal: understand how tasks, priorities, queues, and timeouts shape the firmware.

Suggested prompt:

```text
Use learning-tutor. Teach me FreeRTOS tasks through this project. First explain the concept, then map it to proj_cm33_ns and proj_cm55, point to code locations, give one small exercise, and list common mistakes.
```

Recommended path:

1. Read `proj_cm33_ns/main.c` and `proj_cm55/main.c`.
2. Identify task creation points.
3. Find any queue, notification, or timeout use.
4. Ask why each task can block or must not block.

### UART Parser

Goal: learn frame parsing through LD6002 radar.

Suggested prompt:

```text
Use learning-tutor. Explain UART frame parsing using the LD6002 radar path in this repository. Cover SOF, ID, LEN, TYPE, header checksum, data checksum, endian conversion, incomplete frames, and resync.
```

Then ask:

```text
Use radar-protocol-engineer. Review the current radar parser and list parser tests needed for 0x0A13, 0x0A14, 0x0A15, 0x0A16, checksum error, incomplete frame, and endian conversion.
```

### PDM/PCM

Goal: understand microphone capture and buffering.

Suggested prompt:

```text
Use learning-tutor. Explain PDM to PCM audio capture in this project. Map the concept to app_pdm_pcm and explain where ISR/DMA/ring-buffer risks usually appear.
```

### Mel Features

Goal: understand the PC/board feature contract.

Suggested prompt:

```text
Use learning-tutor. Explain log-Mel features for this project. Connect 16 kHz, 1 s window, HTK Mel, and normalization policy to the CM33 preprocessing code and the training pipeline requirements.
```

### Model Training

Goal: learn a reproducible MIC baseline workflow.

Suggested prompt:

```text
Use audio-ml-engineer. Explain the MIC v3 baseline workflow for cough/non_cough. Include audio_labels_v3, person_id split, tiny check, preprocessing smoke test, one-epoch smoke test, evaluation, threshold sweep, and FP/hour.
```

### Board Deployment

Goal: understand how model outputs reach the board demo.

Suggested prompt:

```text
Use learning-tutor. Explain board deployment in this project: CM33 preprocessing, shared memory, CM55 inference, result reporting, and what a smoke test can and cannot prove.
```

## 2. Development Workflow

### New Feature

1. Write a short feature goal.
2. If it touches RTOS, ISR, DMA, ring buffer, shared memory, CM33/CM55, IPC, or cache, start with `embedded-architect`.
3. Give `firmware-worker` a narrow implementation task.
4. Run the smallest build or smoke test.
5. Send logs to `log-analyzer`.
6. Send final diff to `code-reviewer`.

Prompt:

```text
Use embedded-architect. Review this feature plan before implementation: <plan>. Output risk list, recommended design, task split, firmware-worker tasks, and human-confirmation items.
```

### Bug Fix

1. Reproduce the symptom.
2. Capture the smallest useful log.
3. Use `log-analyzer` for long output.
4. Use `board-debugger` for board behavior.
5. Use `firmware-worker` only after the likely file scope is clear.

Prompt:

```text
Use board-debugger. Analyze this symptom and log. Restate the symptom, extract key logs, rank possible causes, give next validation commands, and say whether embedded-architect is needed.
```

### Log Analysis

Prompt:

```text
Use log-analyzer. Summarize this output. Give key errors, direct cause, possible root cause, suggested files to inspect, and whether another agent should handle it.
```

### Training Experiment

1. Confirm label version.
2. Confirm grouped `person_id` split.
3. Confirm config path.
4. Run tiny check.
5. Run preprocessing smoke test.
6. Run one-epoch smoke test.
7. Run full training only after review.
8. Evaluate, sweep threshold, and compute FP/hour.

Prompt:

```text
Use audio-ml-engineer. Plan this MIC training experiment. Enforce audio_labels_v3, grouped person_id split, board-compatible preprocessing, tiny check, preprocessing smoke test, one-epoch smoke test, evaluation, threshold sweep, and FP/hour. Do not start distillation.
```

Review prompt:

```text
Use ml-pipeline-reviewer. Review whether this training pipeline can enter full training and whether it can export to board. Focus on leakage, labels, split, config reproducibility, board-compatible preprocessing, threshold sweep, and FP/hour.
```

### Board Bring-Up / Joint Debug

1. Build firmware.
2. Program board.
3. Capture UART log.
4. Verify boot markers.
5. Verify CM33 feature metadata.
6. Verify CM55 inference result sequence.
7. Compare PC and board features.

Known commands from this repo:

```powershell
make build
make program
tools\run_capture_csv.bat --list-ports
tools\run_capture_csv.bat --port COM7 --baud 2000000
```

Prompt:

```text
Use board-debugger. Help me debug board real-time inference. Use the log below and focus on build/program status, UART markers, CM33 preprocessing, CM55 inference, shared-memory state, and timeout symptoms.
```

### Commit Review

Prompt:

```text
Use code-reviewer. Review my staged changes before commit. Lead with findings. Focus on embedded real-time risks, CM33/CM55 shared-memory contract, ML reproducibility, data policy, tests, and docs.
```

## 3. Current Project Recommended Flow

### Step 1: Freeze `audio_labels_v3`

- Merge hard negative review results.
- Keep ambiguous samples traceable.
- Confirm `person_id` availability.
- Do not train v3 before this step is done.

Prompt:

```text
Use audio-ml-engineer. Plan audio_labels_v3 freezing. List input files, output files, validation checks, and how to verify person_id split readiness. Do not edit until file scope is clear.
```

### Step 2: MIC v3 Baseline

- Use DS-CNN / small CNN.
- Match CM33 preprocessing.
- Run tiny check, preprocessing smoke test, one epoch, evaluation, threshold sweep, FP/hour.

Prompt:

```text
Use audio-ml-engineer. Prepare a board-compatible MIC v3 baseline plan. Use DS-CNN or small CNN. Include exact checks and mark unknown commands as TODO.
```

### Step 3: Board Consistency

- Compare PC and CM33 feature output.
- Confirm shared metadata such as sample rate, frame count, mel bins, payload size, and result sequence.

Prompt:

```text
Use board-debugger. Plan a PC/board consistency check for MIC preprocessing and CM55 inference. Include expected UART markers and what logs to capture.
```

### Step 4: Radar Parser / Features

- Stabilize LD6002 parser.
- Add tests for frame types and error handling.
- Export parsed CSV and simple quality/motion features.

Prompt:

```text
Use radar-protocol-engineer. Plan the LD6002 parser validation and radar feature baseline. Training must use parsed CSV/features, not raw UART bytes.
```

### Step 5: Fusion State Machine

- Start with rules.
- Inputs: `mic_cough_prob`, `radar_rr`, `radar_hr`, `radar_presence`, `radar_motion_score`, `radar_quality`, `audio_quality`.
- Outputs: `normal`, `attention`, `warning`.
- Keep every state transition explainable.

Prompt:

```text
Use embedded-architect. Review a simple fusion rule state machine for MIC + radar. Keep it explainable for competition defense and split safe implementation tasks for firmware-worker.
```

### Step 6: Demo Polish

- Show what is real, what is smoke-tested, and what is planned.
- Avoid claiming medical-grade performance.
- Keep result screens/logs easy to explain.

Prompt:

```text
Use docs-writer. Draft competition demo notes. Separate completed, in progress, planned, and TODO. Do not exaggerate smoke-test results.
```

## 4. Copy-Paste Chinese Prompts

架构审查：

```text
请使用 embedded-architect。审查下面这个嵌入式改动方案，重点看 FreeRTOS、ISR、DMA、ring buffer、shared memory、CM33/CM55 IPC、cache consistency 和错误码风险。请输出：风险清单、推荐方案、可执行任务拆分、哪些任务可交给 firmware-worker、哪些问题必须人工确认。
```

小范围固件修改：

```text
请使用 firmware-worker。只实现这个明确的小改动：<写具体任务>。修改前先列出准备编辑的文件。不要碰 startup、linker、clock、power、secure boot、BSP 或无关驱动。修改后给出 build/smoke test 建议。
```

MIC v3：

```text
请使用 audio-ml-engineer。规划 MIC cough/non_cough v3 baseline。必须先固化 audio_labels_v3，按 person_id 分组切分，前处理对齐板端 CM33：16 kHz、1 s window、HTK Mel、默认 no norm。必须包含 tiny check、preprocessing smoke test、one-epoch smoke test、evaluation、threshold sweep 和 FP/hour。不要进入 OPERA 蒸馏。
```

ML 审查：

```text
请使用 ml-pipeline-reviewer。审查这个训练流程是否可以进入 full training，以及是否可以导出到板端。重点检查数据泄漏、person_id split、label 版本、config 可复现、board-compatible preprocessing、threshold sweep 和 FP/hour。
```

雷达协议：

```text
请使用 radar-protocol-engineer。审查 LD6002 UART parser。必须考虑 SOF、ID、LEN、TYPE、HEAD_CKSUM、DATA、DATA_CKSUM，头部大端、DATA 小端，并覆盖 0x0A13 phase、0x0A14 breathing rate、0x0A15 heart rate、0x0A16 distance、checksum error、incomplete frame、endian conversion。训练只能用解析后的 radar CSV/features。
```

板端调试：

```text
请使用 board-debugger。分析下面的板端现象和 UART 日志。请输出现象复述、关键日志、可能原因排序、下一步验证命令、是否需要升级给 embedded-architect。
```

日志摘要：

```text
请使用 log-analyzer。摘要下面的长日志，只提取关键错误、直接原因、可能根因、建议检查文件，以及是否需要转交其他 agent。
```

提交前审查：

```text
请使用 code-reviewer。审查我的 staged changes。请优先列出必须修改的问题，再列建议修改、可以暂缓、测试是否足够、是否建议提交。重点看嵌入式实时性、数据可复现性、测试覆盖和文档同步。
```

学习：

```text
请使用 learning-tutor。结合当前项目解释 <FreeRTOS/DMA/ring buffer/UART/PDM/PCM/Mel/模型部署>。先讲概念，再对应到本项目，再指出代码位置，再给最小练习任务，最后列出常见错误。
```
