---
name: e84-contract-audit
description: "Use when the user wants to audit contracts between modules in the E84 project: BLE wire format, shared memory layout, audio feature shape, radar frame/feature format, model input/output, summary snapshot, display data, or App decode expectations. Produces a contract table, producer/consumer map, risks, and verification checks."
tools: Read, Grep, Glob, Bash
---

# E84 Contract Audit Skill

## Purpose

Audit the engineering contracts that connect modules in the E84 project.

A contract is any stable interface that multiple modules depend on, such as:

- BLE frame layout,
- BLE UUID/handle,
- command ID,
- CRC algorithm,
- shared memory struct layout,
- status enum,
- audio feature shape,
- model input/output shape,
- radar frame or feature format,
- summary snapshot fields,
- display state fields,
- App decode expectations,
- build selector values.

Do not modify files.

---

## When to Use

Use this skill when the user asks:

- “这个协议能不能改”
- “这个 struct 改了会影响谁”
- “帮我审计契约”
- “BLE/App/firmware 是否一致”
- “CM33/CM55 shared memory 有什么风险”
- “模型输入输出契约在哪里”
- “屏幕应该读什么数据源”
- “新增字段怎么保持兼容”

---

## Audit Flow

1. Identify the contract name and scope.
2. Find definition file.
3. Find producers.
4. Find consumers.
5. Find tests/golden/docs verifying the contract.
6. Identify fields/values that must remain stable.
7. Identify extension strategy.
8. Identify minimum verification after changes.

---

## Output Format

```markdown
# Contract Audit: <contract name>

## 1. Contract Scope

- Contract:
- Definition location:
- Runtime chain:
- Producers:
- Consumers:
- External dependents:
- Is it public/stable or internal/private?

## 2. Contract Table

| Item | Definition | Producer | Consumer | Stability | Change risk |
|---|---|---|---|---|---|

Stability values:

- fixed,
- versioned,
- internal,
- experimental,
- generated,
- unknown.

## 3. Producer / Consumer Map

```text
producer
  -> contract object / frame / struct
  -> consumer A
  -> consumer B
```

## 4. Field-Level Review

| Field / Value | Meaning | Who writes | Who reads | Default | Can extend? | Risk |
|---|---|---|---|---|---|---|

## 5. Versioning / Compatibility

Answer:

- Is there a version field?
- Is there a magic value?
- Are there reserved fields?
- Is payload length fixed?
- Is order fixed?
- Are enum numeric values stable?
- Is endianess specified?
- Is alignment/padding relevant?
- Is there a golden test?
- Is App/firmware/documentation synchronized?

## 6. Contract Risks

| Risk | Trigger | Symptom | Affected modules | Check |
|---|---|---|---|---|

Examples:

- CM33 and CM55 use different struct layout,
- App decodes old frame format,
- model shape mismatch,
- radar parser endian mismatch,
- display reads stale summary,
- BLE payload exceeds MTU,
- enum numeric value changed.

## 7. Extension Strategy

If adding fields or behavior:

1. Prefer reserved fields if available.
2. Add version only if contract already supports versioning or a migration is planned.
3. Keep existing wire layout stable when App depends on it.
4. Add new command/field instead of changing old meaning when possible.
5. Add default values for old consumers.
6. Update docs and golden tests.
7. Verify old path still works.

## 8. Verification Checklist

| Check | Tool / Method | Required? |
|---|---|---|

Include as applicable:

- compile both cores,
- run golden protocol script,
- fixed-vector model smoke,
- nRF Connect decode,
- App decode,
- board log,
- parser replay,
- snapshot sequence check,
- display mock/smoke,
- docs updated.

## 9. Safe and Unsafe Changes

### Usually Safe

- add internal counter,
- add reserved-field interpretation while preserving old layout,
- add new command ID,
- add non-breaking getter,
- add debug-only log.

### Usually Unsafe

- reorder struct fields,
- change enum numeric values,
- change frame length,
- change CRC,
- change UUID/handle,
- change model input shape,
- change endianess,
- change shared memory status sequence,
- remove reserved fields.

## 10. Final Recommendation

- Current contract status:
  - verified,
  - partially verified,
  - unverified,
  - stale,
  - risky.
- Recommended next action:
```

---

## Contract-Specific Rules

### BLE Contract

Check:

- service UUID,
- characteristic UUIDs,
- handles,
- properties,
- MTU/payload length,
- command IDs,
- response IDs,
- CRC,
- frame length,
- sequence,
- App decoder,
- golden.py,
- docs.

Never suggest changing UUID/handle/wire format casually.

### Shared Memory Contract

Check:

- magic,
- version,
- struct layout,
- alignment,
- cache line considerations,
- status enum,
- producer/consumer ownership,
- sequence,
- valid flags,
- result overwrite semantics.

Never suggest changing layout without dual-core compatibility verification.

### Audio Feature Contract

Check:

- sample rate,
- window length,
- hop,
- Mel bins,
- time bins,
- quantization,
- normalization,
- class order,
- payload bytes.

Must compare PC training/export contract to board preprocessing when possible.

### Radar Contract

Check:

- SOF,
- ID,
- LEN,
- TYPE,
- header checksum,
- payload checksum,
- endianess,
- decoded feature units,
- feature validity flags,
- timeout/stale behavior.

### Display/Summary Contract

Check:

- display consumes summary, not raw internals,
- stale data field,
- validity flags,
- update timestamp,
- refresh interval,
- multiple consumers do not fight over ownership.

---

## Quality Rules

- Do not infer contracts only from comments.
- Prefer definitions and consumers.
- If definition and docs disagree, state the mismatch.
- If no test exists, state “unverified”.
- Do not modify files.
