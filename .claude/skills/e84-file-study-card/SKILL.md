---
name: e84-file-study-card
description: "Use when the user gives a source file path and wants a detailed learning card grounded in the E84 firmware repository. Produces a file-level study card focused on role, layer, runtime chain, contracts, states, risks, observability, and a lab exercise. Read-only by default."
tools: Read, Grep, Glob, Bash
---

# E84 File Study Card Skill

## Purpose

Generate a detailed, repository-grounded learning card for one source file in the E84 respiratory health monitoring project.

This skill is for learning and code ownership recovery. It must help the user understand:

- what the file does,
- where it sits in the system,
- how it interacts with other modules,
- which embedded concepts it demonstrates,
- what can safely be changed,
- what should not be changed,
- how to convert the file into a small learning-lab exercise.

Do not modify files.

---

## When to Use

Use this skill when the user asks things like:

- “帮我学习这个文件”
- “给这个文件生成学习卡”
- “这个文件在项目里负责什么”
- “我想通过这个文件学习嵌入式工程写法”
- “帮我读懂 `<path>`”

---

## Repository Context

Assume the project is an E84 edge-AI respiratory health monitoring project involving:

- CM33 / CM55 split,
- PDM / PCM audio capture and preprocessing,
- Log-Mel feature extraction,
- CM55 model inference,
- CM33 result monitoring,
- shared memory / IPC contracts,
- LD6002 radar UART protocol parsing,
- BLE GATT / Notify / Command protocol,
- App integration,
- later display integration.

If the repository content contradicts this assumption, report the actual evidence from the files.

---

## Read-Only Inspection Flow

Given target file `<TARGET_FILE>`:

1. Read `<TARGET_FILE>`.
2. If present, read its corresponding `.h`.
3. Search who includes the `.h`.
4. Search important public functions from the target file.
5. Search important macros used by the file.
6. Inspect nearby Makefile / build config only if the file behavior depends on macros or target selection.
7. Inspect docs/tests/logs only if they directly verify the file’s behavior.
8. Do not expand indefinitely. Stop when the file’s role, inputs, outputs, contracts, and risks are clear.

Allowed Bash examples:

```bash
grep -R "<symbol>" -n firmware docs tools
find firmware -name "*<module>*"
git grep "<symbol>"
```

Do not run build, clean, flash, install, or modify commands unless the user explicitly asks.

---

## Output Format

```markdown
# File Study Card: <TARGET_FILE>

## 0. Quick Review Card

- One-sentence role:
- Runtime chain:
- Layer:
- Upstream input:
- Downstream output:
- 3–5 key mechanisms:
- Biggest risks:
- Best learning-lab conversion:

## 1. File Responsibility

Explain:

- what this file is responsible for,
- what it is not responsible for,
- why it exists as a separate module,
- which engineering problem it solves.

Do not translate code line by line.

## 2. System Position

```text
upstream
  -> this file / function
  -> downstream
```

Mark whether each step is:

- task,
- callback,
- timer,
- ISR,
- normal function call,
- build-time selection,
- shared-memory handoff.

## 3. Layer and Boundary

Classify the file as one or more:

- configuration,
- build system,
- startup/deployment,
- driver,
- parser/protocol,
- service,
- application logic,
- preprocessing,
- inference,
- shared contract,
- result monitor,
- reporter/output,
- display,
- test/tooling,
- generated code.

Explain:

- why this classification is correct,
- what this layer should do,
- what it should not do,
- how it differs from a simple demo implementation.

## 4. File Structure

Break the file into logical regions:

| Region | Lines / Symbols | Purpose | Why it exists |
|---|---|---|---|

Include:

- includes,
- macros,
- types,
- static state,
- public API,
- static helpers,
- task/callback/main loop,
- state machine,
- error handling,
- debug/test hooks.

## 5. Public API and Internal Helpers

### Public API

| Symbol | Called by | Input | Output | Precondition | Side effect |
|---|---|---|---|---|---|

### Static Helpers

| Symbol | Why static | Encapsulates | Risk if changed |
|---|---|---|---|

## 6. Key Data Structures and State

| Data / State | Producer | Consumer | Lifetime | Contract or internal? | Risk |
|---|---|---|---|---|---|

Include buffers, descriptors, stats, status flags, handles, sequence IDs, task handles, parser states, and result structures.

## 7. Core Macros and Configuration

Group by category:

| Macro | Category | Defined in | Consumed by | Effect | Risk if wrong |
|---|---|---|---|---|---|

Categories:

- feature switch,
- task/priority/stack/timing,
- buffer/payload/length,
- protocol/frame/CRC/version,
- model/feature/shape/quantization,
- log/debug/profile,
- platform/selector/build.

## 8. Include and Build Impact

### This file includes

| Include | Why needed |
|---|---|

### Who includes its header

| File | Dependency | Strength |
|---|---|---|

### Build impact

- firmware target:
- core affected:
- link/build artifact:
- relevant Makefile/CMake/macros:
- minimum build checks after modification:

## 9. Dataflow and Control Flow

### Dataflow

```text
source
  -> buffer / struct / shared memory
  -> processing
  -> output
```

Explain validation, copy, ownership, overwrite/queue/drop semantics.

### Control flow

```text
trigger
  -> init/start/callback/task
  -> state transition
  -> output/error path
```

Explain whether the file is polling, event-driven, callback-driven, ISR-driven, or build-selected.

## 10. State Machine

If applicable:

```text
STATE_A
  -> event / condition
STATE_B
  -> event / condition
STATE_C
```

For each state:

| State | Meaning | Set by | Read by | Valid transitions | Error handling |
|---|---|---|---|---|---|

## 11. Contract Table

List contracts touched by this file.

| Contract | Defined in | Written by | Read by | Can modify? | Risk if changed |
|---|---|---|---|---|---|

Contracts may include:

- BLE frame layout,
- command ID,
- UUID/handle,
- CRC algorithm,
- radar frame format,
- audio feature shape,
- shared memory layout,
- model input/output shape,
- result status enum,
- display summary format,
- App decode expectations.

## 12. Error Handling and Defensive Programming

List checks for:

- NULL,
- length,
- state,
- CRC/checksum,
- version/magic,
- buffer overflow,
- unsupported command,
- init failure,
- task creation failure,
- model compute failure,
- timeout/stale data,
- cache/sync risk.

Use table:

| Error case | Detection | Response | Observable? | Missing check? |
|---|---|---|---|---|

## 13. Observability

Identify:

- logs,
- counters,
- sequence numbers,
- timestamps,
- stats structs,
- debug dumps,
- smoke tests,
- golden tests,
- board/app/nRF/display observation points.

Answer:

- If this file fails, what should be checked first?
- Where should a low-frequency log be added?
- Which counters are worth adding?
- How to avoid blocking real-time paths?

## 14. Embedded Concepts Demonstrated

Explain only concepts visible in this file, such as:

- C module design,
- public API vs static helper,
- pointer/buffer safety,
- struct layout,
- enum state machine,
- FreeRTOS task,
- callback/ISR,
- DMA/ring buffer,
- UART parser,
- BLE GATT/Notify,
- CRC/checksum,
- shared memory,
- cache clean/invalidate,
- memory barrier,
- producer-consumer,
- generated model API,
- numerical stability.

For each concept:

| Concept | Where shown | Why it matters |
|---|---|---|

## 15. Project-Level Coding Patterns

Explain how the code differs from a demo.

Include patterns such as:

- thin main / fat module,
- task wrapper,
- protocol contract,
- producer-consumer,
- state machine,
- defensive checks,
- default-off feature macro,
- fake/real data-source switch,
- generated-code boundary,
- result monitor,
- low-frequency observability,
- minimum-intrusion patching.

## 16. Risk List

Group risks:

| Category | Risk | Trigger | Symptom | First check | Better verification |
|---|---|---|---|---|---|

Categories:

- function,
- timing,
- memory,
- concurrency,
- contract,
- build,
- debug/observability.

## 17. Safe and Dangerous Changes

### Safe Learning Changes

| Change | File area | Expected observation | Verification | Should not affect |
|---|---|---|---|---|

Examples:

- add counter,
- add rate-limited log,
- add stats getter,
- add input check,
- add mock path,
- add smoke test.

### Dangerous Changes

| Change | Why dangerous | What else must be synchronized |
|---|---|---|

Examples:

- shared memory layout,
- BLE UUID/handle/wire format,
- model input shape,
- buffer size,
- cache maintenance order,
- state write order,
- ISR heavy logic,
- task priority.

## 18. Learning-Lab Conversion

Design a small `e84_embedded_lab` exercise.

### Lab name

### Main project mechanism being reproduced

### What not to reproduce

Examples:

- no real BLE stack,
- no real PDM,
- no real CM55,
- no real cache operation,
- no real display hardware,
- no generated model weights.

### Suggested directory

```text
e84_embedded_lab/labs/<lab_name>/
  include/
  src/
  tests/
  docs/
```

### Public API

### Internal state

### State machine

### Normal tests

### Abnormal tests

### Acceptance criteria

### How to compare back to the main project

## 19. Checkpoint Questions

Give 10–20 questions:

- basic role,
- runtime chain,
- API/contract,
- embedded concept,
- risk/debug,
- extension design.

Questions must test understanding, not memorization.

## 20. Final One-Sentence Summary

`<TARGET_FILE>` is a <layer> module in the <runtime chain> responsible for <responsibility>; its engineering value is <contract/state/verification/boundary>.
```

---

## Quality Rules

- Ground claims in files, functions, macros, structs, build rules, or logs.
- State uncertainty explicitly.
- Do not invent runtime behavior.
- Do not explain every generated array/operator table.
- Do not produce a generic textbook explanation.
- Do not modify files.
