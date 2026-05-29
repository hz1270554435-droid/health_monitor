---
name: e84-module-study-card
description: "Use when the user wants to understand a whole module or subsystem in the E84 project rather than one file. Produces a module-level study card with files, responsibility, interfaces, dataflow, contracts, risks, tests, reading order, and learning exercises."
tools: Read, Grep, Glob, Bash
---

# E84 Module Study Card Skill

## Purpose

Generate a module-level learning card for a subsystem in the E84 project.

A module may be:

- BLE protocol,
- radar parser,
- audio preprocessing,
- CM55 inference,
- CM33 result monitor,
- shared memory contract,
- event gate,
- summary snapshot,
- display runtime,
- generated model integration,
- build/model selection.

Do not modify files.

---

## When to Use

Use this skill when the user asks:

- “帮我学习这个模块”
- “这个子系统怎么组织”
- “帮我读 BLE / radar / audio / display 这一块”
- “我该先读哪些文件”
- “这个模块有哪些边界和风险”

---

## Inspection Flow

1. Find all files under the module directory.
2. Identify public headers.
3. Identify entry points.
4. Identify internal helpers.
5. Identify producer/consumer data.
6. Identify contracts and config macros.
7. Identify tests/docs/logs.
8. Produce a reading order and exercises.

---

## Output Format

```markdown
# Module Study Card: <module>

## 1. Module Purpose

- What this module does:
- What it does not do:
- Which runtime chain it belongs to:
- Why it exists as a separate module:

## 2. File Inventory

| File | Type | Role | Read priority |
|---|---|---|---|

Types:

- public header,
- implementation,
- config,
- contract,
- generated,
- test,
- tool,
- doc,
- log/report.

## 3. Layering

```text
driver
  -> parser/protocol
  -> service/state
  -> application logic
  -> reporter/output
```

Mark which files belong to which layer.

## 4. External Interface

| API / data | Defined in | Called/read by | Purpose |
|---|---|---|---|

## 5. Internal Flow

```text
entry
  -> internal processing
  -> state update
  -> output
```

## 6. Data Objects and Ownership

| Data | Owner | Producer | Consumer | Lifetime |
|---|---|---|---|---|

## 7. Contracts

| Contract | Definition | Producer | Consumer | Risk |
|---|---|---|---|---|

## 8. Configuration and Macros

| Macro | Defined in | Effect | Risk |
|---|---|---|---|

## 9. Error Handling and Observability

| Error / status | Where handled | Observable evidence |
|---|---|---|

Include logs, counters, timestamps, stats, debug dumps, smoke tests.

## 10. Tests / Verification Evidence

| Evidence | What it proves | Gap |
|---|---|---|

## 11. Risks

| Risk | Impact | First check |
|---|---|---|

## 12. Recommended Reading Order

| Order | File/topic | Why read now | What to learn |
|---|---|---|---|

## 13. Learning Exercises

Give 1–3 exercises:

- reading exercise,
- trace exercise,
- lab conversion exercise.

## 14. Checkpoint Questions

Questions should test:

- responsibility,
- chain,
- APIs,
- contracts,
- risks,
- debugging,
- safe extension.

## 15. One-Sentence Summary

`<module>` is responsible for <responsibility>; its engineering value is <boundary/contract/state/verification>.
```

---

## Module-Specific Focus

### BLE Module

Focus on:

- GATT DB,
- UUID/handle,
- CCCD,
- notify enable,
- frame build,
- command parser,
- response frame,
- golden vector,
- App compatibility.

### Radar Module

Focus on:

- UART receive,
- byte stream parser,
- frame sync,
- checksum,
- endian conversion,
- feature status,
- stale/timeout,
- fusion input.

### Audio Module

Focus on:

- PDM/PCM capture,
- DMA/callback,
- buffer,
- window,
- feature shape,
- normalization,
- overflow/drop,
- board/PC alignment.

### Shared Memory / IPC Module

Focus on:

- magic/version,
- layout,
- alignment,
- status enum,
- producer/consumer,
- sequence,
- cache/barrier,
- ABI stability.

### Display Module

Focus on:

- display init,
- refresh task/timer,
- summary consumer,
- dirty/stale flags,
- no raw internals access,
- multi-output architecture.

---

## Quality Rules

- Do not average-read all files.
- Identify entry points first.
- Public headers matter more than private helpers.
- Always distinguish contract files from implementation files.
- Always give a reading order.
- Do not modify files.
