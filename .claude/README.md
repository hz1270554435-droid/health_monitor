# Claude Code Agents for This Project

This directory configures Claude Code subagents and project rules for the PSoC Edge E84 respiratory health monitor.

## Agent Responsibilities

- `embedded-architect`: read-only architecture review for FreeRTOS, ISR, DMA, ring buffers, shared memory, CM33/CM55, cache, IPC, and Ethos-U55 inference flow.
- `firmware-worker`: small scoped firmware patches after files and behavior are clear.
- `radar-protocol-engineer`: LD6002 parser, checksum/endian handling, parsed radar CSV/features, radar quality and motion score.
- `audio-ml-engineer`: `audio_labels_v3`, MIC cough/non-cough v3 baseline, board-compatible preprocessing, threshold sweep, FP/hour.
- `ml-pipeline-reviewer`: read-only ML leakage/reproducibility/board-export review.
- `board-debugger`: build/program/UART/debug and PC-board consistency investigation.
- `log-analyzer`: low-cost summary of build, serial, training, and evaluation logs.
- `code-reviewer`: pre-commit review for firmware safety, ML reproducibility, tests, and docs.
- `docs-writer`: README, experiment notes, weekly reports, defense material, and workflow docs.
- `learning-tutor`: beginner-friendly explanations tied to this repository.

## Model Layering

- `opus`: architecture, review, high-risk judgment.
- `sonnet`: implementation, debugging, training-script work, documentation with technical context.
- `haiku`: logs, summaries, lightweight documentation.

The agent frontmatter uses readable aliases: `opus`, `sonnet`, and `haiku`. If your Claude Code installation does not support these aliases, replace them with the model names available in your environment.

## Common Calls

Ask Claude Code to delegate explicitly:

```text
Use the embedded-architect subagent to review this CM33/CM55 shared-memory change plan. Do not edit files.
```

```text
Use the audio-ml-engineer subagent to plan audio_labels_v3 consolidation and list the exact files it would change before editing.
```

```text
Use the radar-protocol-engineer subagent to review the LD6002 parser and propose unit tests for 0x0A13, 0x0A14, 0x0A15, 0x0A16, checksum error, incomplete frame, and endian conversion.
```

```text
Use the log-analyzer subagent to summarize this build log. Extract key errors, likely direct cause, and files to inspect.
```

```text
Use the code-reviewer subagent to review my staged changes before commit. Focus on embedded real-time risk, data policy, tests, and docs.
```

## Prompt Templates

Architecture review:

```text
Use embedded-architect. Review the proposed change below for FreeRTOS, DMA/ring buffer, shared memory, cache, and CM33/CM55 risks. Output risk list, recommended design, task split, firmware-worker tasks, and human-confirmation items.
```

Firmware implementation:

```text
Use firmware-worker. Implement only this scoped change: <task>. First list files to edit. Do not touch startup, linker, clock, power, secure boot, or unrelated drivers. After editing, give build/smoke-test commands.
```

MIC training:

```text
Use audio-ml-engineer. Plan the next MIC v3 step. Enforce audio_labels_v3, person_id split, board-compatible preprocessing, tiny check, smoke test, one epoch, eval, threshold sweep, and FP/hour. Do not start OPERA distillation.
```

Radar parser:

```text
Use radar-protocol-engineer. Inspect the LD6002 parser path and propose tests for phase, breathing rate, heart rate, distance, checksum error, incomplete frame, and endian conversion. Use parsed CSV/features only for training.
```

Board debug:

```text
Use board-debugger. Analyze this board symptom and UART log. Restate the symptom, extract key logs, rank causes, give validation commands, and say whether embedded-architect is needed.
```

## Avoiding Token Waste

- Send logs to `log-analyzer` first, then route the short summary.
- Use `embedded-architect` before implementation only for high-risk embedded changes.
- Use `firmware-worker` only after file scope is clear.
- Ask `ml-pipeline-reviewer` for go/no-go decisions instead of asking it to rewrite code.
- Keep prompts focused on one subsystem: MIC, radar, board, docs, or review.

## Human Confirmation Required

Confirm manually before:

- modifying `data/raw/`,
- adding or committing large data/model/result artifacts,
- changing train/val/test split policy,
- changing CM33/CM55 shared-memory layout,
- changing cache/IPC behavior,
- touching startup, linker, clock, power, secure boot, or BSP files,
- starting full training,
- exporting a new model to board firmware,
- starting OPERA/YAMNet/HeAR distillation.
