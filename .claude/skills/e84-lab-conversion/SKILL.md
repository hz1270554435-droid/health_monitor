---
name: e84-lab-conversion
description: "Use when the user wants to convert a main E84 project file, module, or mechanism into a small e84_embedded_lab exercise. Produces a bounded learning-lab plan with APIs, state machine, tests, abnormal cases, and acceptance criteria. Read-only by default."
tools: Read, Grep, Glob, Bash
---

# E84 Lab Conversion Skill

## Purpose

Convert a real mechanism from the main E84 firmware project into a small, controlled, testable learning-lab exercise.

The goal is not to copy the production code. The goal is to reproduce the key engineering mechanism so the user learns how to write similar project-level embedded code.

---

## When to Use

Use this skill when the user says:

- “把这个文件转成学习项目实验”
- “这个机制我该怎么在 e84_embedded_lab 里重写”
- “帮我设计一个学习 lab”
- “我想把主项目代码学习转化成练习”
- “这个模块的关键机制怎么复现”

---

## Inputs

Accept one of:

- a source file path,
- a module name,
- a mechanism name,
- a runtime chain,
- a symptom/log that suggests a mechanism.

Examples:

- `app_model_inference.c`
- `app_ble_protocol.c`
- `radar parser`
- `audio ring buffer`
- `CM33/CM55 shared snapshot`
- `BLE command response`
- `display summary consumer`

---

## Inspection Flow

1. Read the target file/module.
2. Identify the key mechanism.
3. Identify what is essential and what is hardware-specific.
4. Identify contracts and public APIs.
5. Identify state machine and error paths.
6. Convert only the mechanism into a small lab.
7. Do not copy production code unless the user explicitly asks.

---

## Lab Selection Rules

Choose one lab type:

| Main project mechanism | Learning lab |
|---|---|
| BLE frame/command/response | `ble_protocol_lab` |
| Radar UART parser | `radar_parser_lab` |
| Audio buffer/window | `audio_buffer_lab` |
| CM33/CM55 handoff | `ipc_snapshot_lab` |
| Model threshold/cooldown/energy | `event_gate_lab` |
| Display reads summary | `display_consumer_lab` |
| Build macro selection | `build_macro_lab` |
| Generated model wrapper | `model_api_boundary_lab` |
| Runtime log/debug counters | `observability_lab` |

If unclear, choose the smallest lab that exercises the key mechanism.

---

## Output Format

```markdown
# Lab Conversion: <source mechanism>

## 1. Source Mechanism

- Main project file/module:
- Runtime chain:
- What this mechanism does:
- Why it matters in a real embedded project:
- What learning objective it supports:

## 2. What to Reproduce

Reproduce only:

- core data structure:
- public API:
- state machine:
- validation rules:
- error handling:
- observability:
- tests:

Do not reproduce:

- real hardware:
- vendor HAL:
- real BLE stack:
- real PDM/DMA:
- real CM55/cache:
- real generated model weights:
- full App/display UI:
- unrelated project integration:

## 3. Lab Name and Directory

```text
e84_embedded_lab/labs/<lab_name>/
├── include/
│   └── <lab>.h
├── src/
│   └── <lab>.c
├── tests/
│   └── test_<lab>.c
├── docs/
│   └── <lab>_notes.md
└── README.md
```

## 4. Public API Design

```c
// Example shape only; adapt to actual mechanism.
lab_result_t lab_init(void);
lab_result_t lab_reset(void);
lab_result_t lab_feed(...);
lab_result_t lab_get_output(...);
lab_result_t lab_get_stats(...);
```

Explain each API:

| API | Purpose | Input | Output | Error cases |
|---|---|---|---|---|

## 5. Core Data Structures

```c
typedef struct {
    ...
} lab_state_t;
```

| Field | Meaning | Producer | Consumer | Why needed |
|---|---|---|---|---|

## 6. State Machine

```text
STATE_A
  -> event
STATE_B
  -> event
STATE_C
```

Explain:

- legal transitions,
- invalid transitions,
- recovery behavior,
- what counters/logs observe state changes.

## 7. Normal Path Tests

| Test | Input | Expected output | What it proves |
|---|---|---|---|

At least 3 normal tests.

## 8. Abnormal Path Tests

| Test | Fault injected | Expected behavior | What it proves |
|---|---|---|---|

Include as applicable:

- NULL input,
- bad length,
- bad CRC/checksum,
- unsupported command/type,
- buffer too small,
- stale data,
- state mismatch,
- overflow,
- repeated init,
- no data,
- half packet,
- out-of-order sequence.

## 9. Observability

Include at least:

- stats struct,
- error counter,
- last error,
- sequence or count,
- optional debug dump.

Do not add high-frequency blocking logs in real-time-like paths.

## 10. Acceptance Criteria

A lab is complete only if:

- public header exists,
- source implementation exists,
- tests cover normal and abnormal paths,
- README explains the production mechanism,
- docs explain what was intentionally not reproduced,
- the lab can be understood without main project hardware,
- the user can compare it back to the main project.

## 11. Compare Back to Main Project

After completing the lab, answer:

1. What did the production project do beyond the lab?
2. Which production complexity was hardware-specific?
3. Which production complexity was for compatibility?
4. Which production complexity was for observability/debug?
5. Which production complexity was for safety or concurrency?
6. What would be dangerous to copy back directly?

## 12. Concrete Next Step

Give one exact next action, for example:

```text
Create e84_embedded_lab/labs/ipc_snapshot_lab with a header, source, and tests for READY -> READING -> EMPTY and RESULT_WRITING -> RESULT_READY transitions.
```
```

---

## Lab Templates

### IPC Snapshot Lab

Use for CM33/CM55 shared memory or summary snapshot.

Must include:

- version,
- valid flag,
- sequence number,
- producer update,
- consumer read,
- read-stability check,
- stale/unstable counter,
- result status.

### BLE Protocol Lab

Use for command/response/realtime/event frame.

Must include:

- frame layout,
- command ID,
- payload length,
- CRC-8/ATM or project CRC,
- decode,
- encode,
- unsupported command,
- output buffer too small,
- golden vector style tests.

Do not use real BLE stack.

### Radar Parser Lab

Use for LD6002 or UART parser.

Must include:

- byte feed API,
- parser state,
- header sync,
- length,
- checksum,
- half packet,
- sticky packet,
- resynchronization,
- error counters.

Do not use real UART hardware.

### Audio Buffer Lab

Use for PDM/PCM/windowing.

Must include:

- simulated PCM input,
- ring buffer or block buffer,
- window extraction,
- sample count,
- overflow/drop counters,
- simple energy calculation.

Do not run real PDM or full Log-Mel unless explicitly requested.

### Event Gate Lab

Use for model event decisions.

Must include:

- threshold,
- candidate window,
- cooldown,
- energy guard,
- event count,
- suppress count,
- test input sequence.

Do not run real model.

### Display Consumer Lab

Use for screen/display integration.

Must include:

- summary snapshot input,
- display state mapping,
- refresh interval,
- dirty flag,
- mock display backend,
- no direct access to raw audio/radar/model internals.

---

## Quality Rules

- Keep labs small.
- Avoid reimplementing the full product.
- Prefer 1–3 day exercises.
- Include abnormal tests.
- Separate mechanism from hardware.
- Do not claim production equivalence.
- The lab trains understanding; it is not automatically production code.
