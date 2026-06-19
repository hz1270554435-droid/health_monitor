/*******************************************************************************
* File Name : app_model_ipc_smoke.h
*
* Description : Default-off CM33/CM55 IPC coherency smoke helpers.
*
* These helpers intentionally do not change app_model_shared_region_t. They only
* generate deterministic payload patterns and signatures for board smoke builds.
*******************************************************************************/

#ifndef __APP_MODEL_IPC_SMOKE_H__
#define __APP_MODEL_IPC_SMOKE_H__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef APP_MODEL_IPC_SMOKE_PAYLOAD_ENABLE
#define APP_MODEL_IPC_SMOKE_PAYLOAD_ENABLE      (0u)
#endif

#ifndef APP_MODEL_IPC_SMOKE_RESET_DIAG_ENABLE
#define APP_MODEL_IPC_SMOKE_RESET_DIAG_ENABLE   (0u)
#endif

#ifndef APP_MODEL_IPC_SMOKE_EXCLUSIVE_ENABLE
#define APP_MODEL_IPC_SMOKE_EXCLUSIVE_ENABLE    (0u)
#endif

#ifndef APP_MODEL_IPC_SMOKE_LOG_LIMIT
#define APP_MODEL_IPC_SMOKE_LOG_LIMIT           (64u)
#endif

#ifndef APP_MODEL_IPC_SMOKE_CM55_DELAY_MS
#define APP_MODEL_IPC_SMOKE_CM55_DELAY_MS       (0u)
#endif

#ifndef APP_MODEL_IPC_SMOKE_FORCE_WARM_STALE_READY_ENABLE
#define APP_MODEL_IPC_SMOKE_FORCE_WARM_STALE_READY_ENABLE (0u)
#endif

#ifndef APP_MODEL_IPC_SMOKE_FORCE_WARM_STALE_INPUT_ENABLE
#define APP_MODEL_IPC_SMOKE_FORCE_WARM_STALE_INPUT_ENABLE (0u)
#endif

#define APP_MODEL_IPC_SMOKE_HASH_OFFSET         (2166136261u)
#define APP_MODEL_IPC_SMOKE_HASH_PRIME          (16777619u)
#define APP_MODEL_IPC_SMOKE_BOOT_EPOCH          (1u)

#if ((APP_MODEL_IPC_SMOKE_EXCLUSIVE_ENABLE != 0u) && \
     (APP_MODEL_IPC_SMOKE_EXCLUSIVE_ENABLE != 1u))
#error "APP_MODEL_IPC_SMOKE_EXCLUSIVE_ENABLE must be 0 or 1"
#endif

#if (APP_MODEL_IPC_SMOKE_EXCLUSIVE_ENABLE) && \
    ((!APP_MODEL_IPC_SMOKE_PAYLOAD_ENABLE) || \
     (!APP_MODEL_IPC_SMOKE_RESET_DIAG_ENABLE))
#error "Exclusive IPC smoke requires payload and reset diagnostics"
#endif

#if ((APP_MODEL_IPC_SMOKE_FORCE_WARM_STALE_READY_ENABLE != 0u) && \
     (APP_MODEL_IPC_SMOKE_FORCE_WARM_STALE_READY_ENABLE != 1u))
#error "APP_MODEL_IPC_SMOKE_FORCE_WARM_STALE_READY_ENABLE must be 0 or 1"
#endif

#if (APP_MODEL_IPC_SMOKE_FORCE_WARM_STALE_READY_ENABLE) && \
    (!APP_MODEL_IPC_SMOKE_EXCLUSIVE_ENABLE)
#error "Warm stale READY trigger requires exclusive IPC smoke"
#endif

#if ((APP_MODEL_IPC_SMOKE_FORCE_WARM_STALE_INPUT_ENABLE != 0u) && \
     (APP_MODEL_IPC_SMOKE_FORCE_WARM_STALE_INPUT_ENABLE != 1u))
#error "APP_MODEL_IPC_SMOKE_FORCE_WARM_STALE_INPUT_ENABLE must be 0 or 1"
#endif

#if (APP_MODEL_IPC_SMOKE_FORCE_WARM_STALE_INPUT_ENABLE) && \
    (!APP_MODEL_IPC_SMOKE_EXCLUSIVE_ENABLE)
#error "Warm stale input trigger requires exclusive IPC smoke"
#endif

#if (APP_MODEL_IPC_SMOKE_FORCE_WARM_STALE_READY_ENABLE) && \
    (APP_MODEL_IPC_SMOKE_FORCE_WARM_STALE_INPUT_ENABLE)
#error "Warm stale result and stale input triggers are mutually exclusive"
#endif

#if (APP_MODEL_IPC_SMOKE_EXCLUSIVE_ENABLE)
void app_model_ipc_smoke_log_init(void);
void app_model_ipc_smoke_log_lock(void);
void app_model_ipc_smoke_log_unlock(void);
#else
static inline void app_model_ipc_smoke_log_init(void)
{
}

static inline void app_model_ipc_smoke_log_lock(void)
{
}

static inline void app_model_ipc_smoke_log_unlock(void)
{
}
#endif

static inline uint32_t app_model_ipc_smoke_hash_bytes(uint32_t hash,
                                                       const void *data,
                                                       uint32_t size)
{
    const uint8_t *bytes = (const uint8_t *)data;

    for (uint32_t i = 0u; i < size; i++)
    {
        hash ^= (uint32_t)bytes[i];
        hash *= APP_MODEL_IPC_SMOKE_HASH_PRIME;
    }

    return hash;
}

static inline float app_model_ipc_smoke_payload_value(uint32_t sequence,
                                                       uint32_t index)
{
    uint32_t raw = ((sequence * 37u) ^
                    (index * 73u) ^
                    ((index + 1u) * 17u)) & 0x000003ffu;

    return ((float)raw) * (1.0f / 1024.0f);
}

static inline uint32_t app_model_ipc_smoke_mid_index(uint32_t count)
{
    return (0u == count) ? 0u : (count / 2u);
}

static inline uint32_t app_model_ipc_smoke_last_index(uint32_t count)
{
    return (0u == count) ? 0u : (count - 1u);
}

static inline void app_model_ipc_smoke_fill_payload(float *payload,
                                                     uint32_t count,
                                                     uint32_t sequence)
{
    if (NULL == payload)
    {
        return;
    }

    for (uint32_t i = 0u; i < count; i++)
    {
        payload[i] = app_model_ipc_smoke_payload_value(sequence, i);
    }
}

static inline uint32_t app_model_ipc_smoke_hash_payload(const float *payload,
                                                        uint32_t count)
{
    uint32_t hash = APP_MODEL_IPC_SMOKE_HASH_OFFSET;

    if (NULL == payload)
    {
        return hash;
    }

    for (uint32_t i = 0u; i < count; i++)
    {
        hash = app_model_ipc_smoke_hash_bytes(hash,
                                              (const void *)&payload[i],
                                              (uint32_t)sizeof(payload[i]));
    }

    return hash;
}

static inline uint32_t app_model_ipc_smoke_expected_hash(uint32_t sequence,
                                                         uint32_t count)
{
    uint32_t hash = APP_MODEL_IPC_SMOKE_HASH_OFFSET;

    for (uint32_t i = 0u; i < count; i++)
    {
        float value = app_model_ipc_smoke_payload_value(sequence, i);
        hash = app_model_ipc_smoke_hash_bytes(hash,
                                              (const void *)&value,
                                              (uint32_t)sizeof(value));
    }

    return hash;
}

static inline bool app_model_ipc_smoke_float_equal(float a, float b)
{
    return a == b;
}

static inline uint32_t app_model_ipc_smoke_hash_low16(uint32_t hash)
{
    return hash & 0x0000ffffu;
}

static inline uint32_t app_model_ipc_smoke_hash_high16(uint32_t hash)
{
    return (hash >> 16u) & 0x0000ffffu;
}

static inline uint32_t app_model_ipc_smoke_hash_from_halves(float low,
                                                            float high)
{
    uint32_t low16 = (uint32_t)(low + 0.5f) & 0x0000ffffu;
    uint32_t high16 = (uint32_t)(high + 0.5f) & 0x0000ffffu;

    return low16 | (high16 << 16u);
}

#endif /* __APP_MODEL_IPC_SMOKE_H__ */
