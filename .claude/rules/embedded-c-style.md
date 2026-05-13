# Embedded C/C++ Style Rules

These rules apply to firmware in `proj_cm33_s/`, `proj_cm33_ns/`, `proj_cm55/`, `shared/`, and BSP-adjacent code.

## General Style

- Prefer small functions with one clear responsibility.
- Keep ownership and lifetime of buffers explicit.
- Use fixed-width integer types when data crosses hardware, wire, file, or core boundaries.
- Check pointer arguments, sizes, enum ranges, and state transitions near API boundaries.
- Keep comments short and useful. Explain non-obvious timing, concurrency, memory, or hardware assumptions.
- Preserve local naming and formatting style.

## Error Handling

- Prefer existing `cy_rslt_t` or project-local error-code conventions.
- Do not introduce a parallel error handling framework without architecture review.
- Return errors early and clearly.
- Avoid hiding errors behind generic `false` unless the local API already uses `bool`.

## Logging

- Keep logs gated or rate-limited in real-time paths.
- Do not print from high-frequency ISR paths.
- Include enough context to debug state, sequence, and timeout problems.
- Do not mix binary capture streams and text logs unless the framing contract explicitly supports it.

## FreeRTOS

- Do not block inside ISR callbacks.
- Do not perform heavy compute in ISR context.
- Use `FromISR` APIs from ISR context.
- Keep task priorities conservative and documented.
- Treat queue, semaphore, and notification timeouts as part of the API contract.

## DMA / Ring Buffer

- Validate buffer size, alignment, and ownership.
- Avoid overwriting unread data unless loss behavior is explicit and logged.
- Keep producer/consumer state transitions simple.
- Be careful with `volatile`; use it for hardware/ISR visibility, not as a replacement for synchronization.

## CM33 / CM55 / Shared Memory

- Shared structures must have stable layout and explicit version/size expectations.
- Sequence counters, state flags, and result slots must be updated in a documented order.
- Cache maintenance and memory barriers require `embedded-architect` review.
- Do not change shared-memory layout without checking both cores.

## High-Risk Files

Architecture review is required before changing startup, linker, clock, power, secure boot, BSP, IPC, cache, ISR, DMA, ring buffer, or shared-memory code.
