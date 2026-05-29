---
name: e84-call-chain-trace
description: "Use when the user wants to trace how an event, log, data object, callback, command, notify, inference, radar frame, audio window, or display update flows through the E84 firmware. Produces an evidence-grounded runtime chain with files, functions, data, state, errors, and verification points."
tools: Read, Grep, Glob, Bash
---

# E84 Call Chain Trace Skill

## Purpose

Trace a runtime event through the E84 firmware repository.

This skill helps answer:

- where does this data come from?
- who calls this function?
- how does this log appear?
- how does this BLE notify get sent?
- how does an audio feature become a model result?
- how does radar raw data become a feature?
- how does summary data reach BLE/display?
- where can the chain fail?

Do not modify files.

---

## When to Use

Use this skill when the user asks:

- “从入口开始追踪”
- “这个 log 是哪里打印的”
- “这个数据从哪里来”
- “一次推理是怎么发生的”
- “BLE notify 是怎么发出来的”
- “雷达帧是怎么解析的”
- “屏幕显示数据从哪里来”
- “App command 进入板端后发生了什么”

---

## Inspection Strategy

1. Start from the user-provided event/log/symbol/path.
2. If it is a log string, search exact string first.
3. If it is a function, search callers and callees.
4. If it is data, search producer and consumer.
5. If it is a macro-controlled path, inspect build macro definitions.
6. If it crosses modules, identify contracts and states.
7. Stop when the trigger, path, output, and verification point are clear.

---

## Output Format

```markdown
# Call Chain Trace: <event / symbol / log>

## 1. Trigger

- Trigger type:
  - boot,
  - task,
  - callback,
  - timer,
  - ISR,
  - BLE write,
  - BLE subscribe,
  - UART RX,
  - audio block,
  - shared-memory ready flag,
  - model result,
  - display refresh,
  - build macro,
  - test script.
- First confirmed entry:
- Evidence:

## 2. End-to-End Chain

```text
entry
  -> function_a()
  -> function_b()
  -> state/data update
  -> output/log/result
```

Mark each step:

- `[task]`
- `[callback]`
- `[timer]`
- `[ISR]`
- `[normal]`
- `[shared-memory]`
- `[build-time]`
- `[test/tool]`

## 3. Step Details

| Step | File | Function / Symbol | Input | Output | Side effect | Evidence |
|---|---|---|---|---|---|---|

## 4. Data Objects

| Data | Producer | Consumer | Format | Lifetime | Ownership |
|---|---|---|---|---|---|

Include buffers, frames, descriptors, shared structs, snapshots, counters, stats.

## 5. State Transitions

```text
STATE_A
  -> condition
STATE_B
  -> condition
STATE_C
```

| State | Set by | Read by | Meaning |
|---|---|---|---|

## 6. Contracts Crossed

| Contract | Defined in | Producer | Consumer | Risk |
|---|---|---|---|---|

Examples:

- BLE frame layout,
- radar frame format,
- audio feature shape,
- shared memory status,
- model output format,
- display summary fields.

## 7. Failure Points

| Rank | Failure point | Symptom | How to check | Non-invasive evidence |
|---|---|---|---|---|

## 8. Verification Plan

List from lowest-risk to higher-risk:

1. code evidence,
2. build macro evidence,
3. test/golden evidence,
4. board log evidence,
5. external tool evidence,
6. App/display observation.

## 9. Minimal Log / Breakpoint Locations

| Location | Why useful | Expected value |
|---|---|---|

Prefer low-frequency logs or counters.

## 10. What the User Should Understand

After reading, the user should be able to explain:

- trigger,
- chain,
- key data,
- state transitions,
- failure points,
- how to verify.

## 11. One-Sentence Summary

`<event>` is triggered by <trigger>, travels through <key modules>, crosses <contracts>, and produces <output>; the most important risk is <risk>.
```

---

## Specialized Trace Patterns

### BLE Command Trace

Trace:

```text
App / nRF Connect write
  -> GATT callback
  -> command queue/parser
  -> command handler
  -> response frame build
  -> response notify/write-back
```

Check:

- command characteristic,
- CCCD/subscription if notify is used,
- frame length,
- CRC,
- command ID,
- response format,
- golden script.

### BLE Notify Trace

Trace:

```text
subscribe
  -> notify enabled state
  -> timer/task
  -> data source
  -> frame builder
  -> notify API
  -> App/nRF Connect
```

Check:

- handle/UUID,
- payload length <= MTU payload,
- notify enabled,
- sequence,
- CRC,
- data source fake/real.

### CM33/CM55 Inference Trace

Trace:

```text
CM33 feature ready
  -> shared input slot
  -> CM55 consumer task
  -> model compute
  -> shared result slot
  -> CM33 result monitor
```

Check:

- magic/version,
- input status,
- output status,
- shape,
- cache maintenance,
- sequence,
- threshold.

### Radar Parser Trace

Trace:

```text
UART RX byte/block
  -> parser state machine
  -> frame validation
  -> payload decode
  -> radar feature/status
  -> fusion/summary/log
```

Check:

- SOF,
- length,
- checksum,
- endian conversion,
- resynchronization,
- error counters.

### Audio Pipeline Trace

Trace:

```text
PDM/PCM capture
  -> buffer
  -> window
  -> preprocessing
  -> feature matrix
  -> shared memory / model input
```

Check:

- sample rate,
- block size,
- window length,
- hop,
- Mel bins,
- normalization,
- dropped/overflow counters.

### Display Update Trace

Trace:

```text
summary snapshot
  -> display state mapping
  -> refresh task/timer
  -> draw/update backend
```

Check:

- display should consume summary,
- no direct raw audio/radar/model access,
- refresh interval,
- dirty flag,
- stale data behavior.

---

## Quality Rules

- Always distinguish confirmed and unconfirmed links.
- Do not assume a caller exists; search for it.
- Do not hide macro-gated paths.
- If there are multiple possible paths, list them and state which one is built/enabled if known.
- Prefer exact log strings and symbol references.
- Do not modify files.
