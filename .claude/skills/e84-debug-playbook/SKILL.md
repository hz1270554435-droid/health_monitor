---
name: e84-debug-playbook
description: "Use when the user reports a symptom in the E84 project and wants a structured, non-invasive debug plan. Covers BLE, radar, audio, CM33/CM55 shared memory, model inference, display, build macros, and board logs."
tools: Read, Grep, Glob, Bash
---

# E84 Debug Playbook Skill

## Purpose

Create a structured debug plan for E84 firmware symptoms.

The priority is:

1. observe,
2. localize,
3. prove,
4. only then modify.

Do not immediately suggest code changes unless evidence points to a small, safe modification.

---

## When to Use

Use this skill when the user says:

- “这个现象怎么排查”
- “BLE 收不到 notify”
- “模型没有结果”
- “雷达没有数据”
- “音频不对”
- “CM55 没跑”
- “屏幕没显示”
- “日志不符合预期”
- “build 好像用了错的模型”
- “板子运行异常”

---

## Output Format

```markdown
# Debug Playbook: <symptom>

## 1. Symptom

- Observed behavior:
- Expected behavior:
- Where observed:
  - board log,
  - nRF Connect,
  - App,
  - display,
  - build output,
  - test script,
  - PC replay.
- When it started:
- Related recent changes:

## 2. First Classification

Classify as likely:

- build/config issue,
- startup/task issue,
- protocol/contract issue,
- data-source issue,
- parser issue,
- buffer/timing issue,
- shared-memory issue,
- model/inference issue,
- threshold/event-gate issue,
- display consumer issue,
- logging/observability issue,
- hardware/connection issue.

## 3. Most Likely Causes

| Rank | Cause | Why plausible | How to check | Non-invasive evidence |
|---|---|---|---|---|

## 4. Runtime Chain to Inspect

```text
trigger
  -> step A
  -> step B
  -> output
```

Mark key breakpoints/log points.

## 5. Minimal Non-Invasive Checks

List in order:

1. build/config proof,
2. startup proof,
3. data-source proof,
4. contract proof,
5. output proof.

Do not start with broad refactoring.

## 6. Code Locations to Inspect

| File | Function / symbol | Why inspect |
|---|---|---|

## 7. What Not to Do First

Examples:

- do not change model before proving preprocessing path,
- do not change BLE protocol before proving subscription state,
- do not change shared memory layout before proving producer/consumer status,
- do not add high-frequency printf in real-time path,
- do not retrain before verifying board/PC feature alignment.

## 8. Suggested Next Experiment

Must be small and observable.

- Change:
- Expected evidence:
- Verification:
- Rollback:
- Should not affect:

## 9. Evidence Needed to Close the Issue

| Evidence | Pass criteria |
|---|---|

## 10. Current Conclusion

State:

- verified,
- partially verified,
- unverified,
- most likely next check.
```

---

## Domain Debug Guides

### BLE Notify Missing

Check in order:

1. correct firmware flashed,
2. advertising visible,
3. service/characteristics discovered,
4. CCCD subscribed,
5. notify enabled state,
6. timer/task producing frame,
7. payload length within MTU,
8. notify API return,
9. App/nRF decode,
10. no UUID/handle/wire format drift.

### BLE Command No Response

Check:

1. command characteristic write,
2. GATT callback triggered,
3. raw queue/parser receives bytes,
4. CRC/length/command ID valid,
5. handler produces response,
6. response notify/write path,
7. golden vector matches.

### CM55 / Model No Result

Check:

1. CM33 releases/enables CM55,
2. CM55 `main()` starts,
3. inference task created,
4. shared input status becomes READY,
5. magic/version match,
6. input shape valid,
7. model runtime init succeeds,
8. compute returns success,
9. result status becomes READY,
10. CM33 result monitor consumes it.

### Board/PC Model Mismatch

Check:

1. active model selector,
2. generated model source linked,
3. input shape,
4. class order,
5. preprocessing parameters,
6. normalization,
7. fixed-vector smoke,
8. board feature dump replay,
9. threshold/event gate.

### Radar No Feature

Check:

1. UART config/pins,
2. RX callback/data arrival,
3. parser sync,
4. length/checksum,
5. endian decode,
6. message type support,
7. stale/timeout flag,
8. feature consumer.

### Audio Pipeline Problem

Check:

1. PDM/PCM input starts,
2. buffer fills,
3. block/window length,
4. sample rate,
5. overflow/drop counter,
6. feature shape,
7. feature values,
8. shared memory handoff.

### Display Missing / Stale

Check:

1. display init,
2. refresh task/timer,
3. summary provider available,
4. stale flag,
5. dirty flag,
6. mock snapshot works,
7. display does not read raw internals.

### Build / Macro Suspicion

Check:

1. Makefile value,
2. generated `.defines`,
3. compiler command,
4. source conditional path,
5. linked map,
6. startup log.

---

## Quality Rules

- Always rank causes.
- Prefer checks that do not modify firmware.
- Separate “symptom observed” from “hypothesis”.
- Do not recommend retraining/model changes before proving deployment contracts.
- Do not recommend protocol changes before proving current protocol state.
- Do not suggest high-frequency blocking logs in ISR/audio/realtime paths.
