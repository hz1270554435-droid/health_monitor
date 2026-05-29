---
name: e84-build-macro-trace
description: "Use when the user wants to understand build variables, Makefile macros, selector values, compile-time feature switches, generated defines, or why a certain model/path/code was or was not included in the E84 firmware. Produces a build variable flow, definition/consumption table, proof plan, and risk list."
tools: Read, Grep, Glob, Bash
---

# E84 Build Macro Trace Skill

## Purpose

Trace build variables and compile-time macros through the E84 firmware repository.

Use this to understand:

- which model is selected,
- whether smoke test is enabled,
- whether fake/real path is compiled,
- which source files are linked,
- how Makefile variables become `-D` macros,
- why a code path did or did not execute.

Read-only by default.

---

## When to Use

Use this skill when the user asks:

- “这个宏在哪里定义”
- “APP_AUDIO_MODEL_SELECT 是怎么传进去的”
- “为什么这个模型没被编译”
- “这次 build 到底用了哪个 selector”
- “stack=0/stack=1 有什么差别”
- “这个功能开关是否启用”
- “为什么 smoke test 没跑”
- “怎么证明当前固件编译了哪个模型”

---

## Inspection Flow

1. Identify macro/build variable.
2. Search where it is defined.
3. Search where it is consumed.
4. Inspect Makefile/CMake fragments.
5. Inspect generated defines if available.
6. Inspect compile commands/map/link evidence if available.
7. Inspect startup logs if available.
8. State what is proven and what is not.

Allowed safe commands:

```bash
grep -R "<MACRO>" -n firmware
find . -name "*.defines" -o -name "compile_commands.json" -o -name "*.map"
make -n <target>
git grep "<MACRO>"
```

Do not clean, rebuild, flash, or modify unless explicitly requested.

---

## Output Format

```markdown
# Build/Macro Trace: <macro or feature>

## 1. Question

- Macro/feature:
- User question:
- Expected behavior:
- Current evidence status:

## 2. Definition Sites

| Macro / Variable | Defined in | Value | Condition | Evidence |
|---|---|---|---|---|

## 3. Consumption Sites

| File | Symbol | How used | Effect |
|---|---|---|---|

## 4. Build Variable Flow

```text
make command / default
  -> Makefile variable
  -> generated define / compiler -D
  -> source conditional compilation
  -> linked source / runtime behavior
```

## 5. Source Selection

If relevant:

| Selector / macro value | Included source | Active API | Expected model/path |
|---|---|---|---|

## 6. Proof Evidence

| Evidence | Found? | Meaning |
|---|---|---|

Check:

- `.defines`,
- `compile_commands.json`,
- map file,
- linked object/source,
- startup log,
- `[MODEL_INFO]` or equivalent,
- smoke report,
- golden output.

## 7. Failure Points

| Failure point | Symptom | How to check |
|---|---|---|

Examples:

- macro default differs from Makefile value,
- `.defines` stale,
- wrong build directory,
- selector maps to wrong model,
- source compiled but not linked,
- stack variant uses different config,
- runtime log comes from old flashed firmware.

## 8. Minimal Verification Plan

List the smallest proof chain:

1. inspect Makefile default,
2. inspect generated defines / compile command,
3. inspect source conditional path,
4. inspect linked map or object,
5. inspect runtime startup log.

## 9. Safe Next Action

Give one non-invasive action, e.g.:

```text
Run a dry build or inspect the generated .defines file for APP_AUDIO_MODEL_SELECT.
```

## 10. One-Sentence Summary

`<macro>` flows from <definition> through <build artifact> into <source path>, affecting <runtime behavior>; current proof is <verified/partial/unverified>.
```

---

## Common E84 Macros / Areas

Prioritize these when present:

- `APP_AUDIO_MODEL_SELECT`
- `APP_MODEL_SMOKE_TEST_ENABLE`
- `APP_MODEL_RUNTIME_PROFILE_ENABLE`
- `APP_MODEL_LOG_LEVEL`
- `APP_MODEL_LOG_RATE_LIMIT_MS`
- `APP_MODEL_PRINT_FLOAT_ENABLE`
- `APP_MODEL_EVENT_MIN_ENERGY`
- BLE fake/real data source macros
- display enable macros
- radar enable/parser macros
- CM33/CM55 shared memory macros
- active model header/API macros

---

## Quality Rules

- Do not assume the macro value from source defaults if build artifacts override it.
- Distinguish source default, Makefile value, compiler define, and runtime evidence.
- If there is no build artifact, say proof is incomplete.
- Do not modify files.
