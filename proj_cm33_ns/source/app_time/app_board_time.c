#include "app_board_time.h"
#include "app_build_config.h"

#include <stddef.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "cymem_CM33_0.h"

#define APP_BOARD_TIME_RECORD_MAGIC   (0x4254494Du) /* "BTIM" */
#define APP_BOARD_TIME_RECORD_VERSION (1u)
#define APP_BOARD_TIME_RECORD_SLOTS   (2u)
#define APP_BOARD_TIME_RECORD_SIZE    (64u)
#define APP_BOARD_TIME_NVM_ADDR       (CYMEM_CM33_0_user_nvm_START)

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t sequence;
    uint32_t epoch_s;
    uint32_t source;
    uint32_t flags;
    uint32_t checksum;
    uint32_t reserved[9];
} app_board_time_record_t;

typedef char app_board_time_record_size_check[
    (sizeof(app_board_time_record_t) == APP_BOARD_TIME_RECORD_SIZE) ? 1 : -1];
typedef char app_board_time_nvm_size_check[
    ((APP_BOARD_TIME_RECORD_SLOTS * APP_BOARD_TIME_RECORD_SIZE) <=
     CYMEM_CM33_0_user_nvm_SIZE) ? 1 : -1];

static bool s_initialized;
static bool s_time_valid;
static uint32_t s_base_epoch_s;
static uint32_t s_base_uptime_s;
static uint32_t s_sequence;
static uint32_t s_source;
static uint32_t s_flags;
static uint32_t s_next_slot;

static uint32_t board_time_uptime_s(void);
static uint32_t board_time_checksum(const app_board_time_record_t *record);
static bool board_time_record_is_valid(const app_board_time_record_t *record);
static bool board_time_read_slot(uint32_t slot, app_board_time_record_t *record);
static cy_rslt_t board_time_write_slot(uint32_t slot,
                                       const app_board_time_record_t *record);
static bool board_time_source_should_persist(uint32_t source);
static void board_time_load_from_nvm(void);
static void board_time_ensure_init(void);

cy_rslt_t app_board_time_init(void)
{
    if (s_initialized)
    {
        return CY_RSLT_SUCCESS;
    }

    s_initialized = true;
    s_time_valid = false;
    s_base_epoch_s = 0u;
    s_base_uptime_s = board_time_uptime_s();
    s_sequence = 0u;
    s_source = APP_BOARD_TIME_SOURCE_NONE;
    s_flags = 0u;
    s_next_slot = 0u;

#if (APP_BOARD_TIME_NVM_ENABLE)
    board_time_load_from_nvm();
#endif

    if ((!s_time_valid) && (0u != (uint32_t)APP_BOARD_TIME_BOOTSTRAP_EPOCH_S))
    {
        (void)app_board_time_calibrate_epoch_s(
            (uint32_t)APP_BOARD_TIME_BOOTSTRAP_EPOCH_S,
            APP_BOARD_TIME_SOURCE_BOOTSTRAP);
    }

    return CY_RSLT_SUCCESS;
}

cy_rslt_t app_board_time_calibrate_epoch_s(uint32_t epoch_s, uint32_t source)
{
    app_board_time_record_t record;
    cy_rslt_t result = CY_RSLT_SUCCESS;

    board_time_ensure_init();

    s_base_epoch_s = epoch_s;
    s_base_uptime_s = board_time_uptime_s();
    s_sequence++;
    if (0u == s_sequence)
    {
        s_sequence = 1u;
    }
    s_source = source;
    s_time_valid = true;
    s_flags = APP_BOARD_TIME_FLAG_VALID;
    if (APP_BOARD_TIME_SOURCE_BLE == source)
    {
        s_flags |= APP_BOARD_TIME_FLAG_BLE_SYNCED;
    }

#if (APP_BOARD_TIME_NVM_ENABLE)
    if (board_time_source_should_persist(source))
    {
        memset(&record, 0, sizeof(record));
        record.magic = APP_BOARD_TIME_RECORD_MAGIC;
        record.version = APP_BOARD_TIME_RECORD_VERSION;
        record.size = (uint16_t)sizeof(record);
        record.sequence = s_sequence;
        record.epoch_s = epoch_s;
        record.source = source;
        record.flags = s_flags;
        record.checksum = board_time_checksum(&record);

        result = board_time_write_slot(s_next_slot, &record);
        if (CY_RSLT_SUCCESS == result)
        {
            s_flags |= APP_BOARD_TIME_FLAG_NVM_SAVED;
            s_next_slot = (s_next_slot + 1u) % APP_BOARD_TIME_RECORD_SLOTS;
        }
    }
#endif

    return result;
}

bool app_board_time_now_epoch_s(uint32_t *epoch_s)
{
    uint32_t now_s;

    board_time_ensure_init();

    if ((!s_time_valid) || (NULL == epoch_s))
    {
        return false;
    }

    now_s = board_time_uptime_s();
    *epoch_s = s_base_epoch_s + (now_s - s_base_uptime_s);
    return true;
}

bool app_board_time_is_synced(void)
{
    board_time_ensure_init();
    return s_time_valid;
}

uint32_t app_board_time_get_offset_s(void)
{
    uint32_t epoch_s;

    if (!app_board_time_now_epoch_s(&epoch_s))
    {
        return 0u;
    }

    return epoch_s - board_time_uptime_s();
}

uint32_t app_board_time_get_flags(void)
{
    board_time_ensure_init();
    return s_flags;
}

uint32_t app_board_time_get_source(void)
{
    board_time_ensure_init();
    return s_source;
}

static uint32_t board_time_uptime_s(void)
{
    return (uint32_t)((xTaskGetTickCount() * portTICK_PERIOD_MS) / 1000u);
}

static uint32_t board_time_checksum(const app_board_time_record_t *record)
{
    const uint8_t *bytes = (const uint8_t *)record;
    uint32_t hash = 2166136261u;

    for (uint32_t i = 0u; i < sizeof(*record); ++i)
    {
        uint8_t value = bytes[i];
        if ((i >= offsetof(app_board_time_record_t, checksum)) &&
            (i < (offsetof(app_board_time_record_t, checksum) +
                  sizeof(record->checksum))))
        {
            value = 0u;
        }
        hash ^= value;
        hash *= 16777619u;
    }

    return hash;
}

static bool board_time_record_is_valid(const app_board_time_record_t *record)
{
    if (NULL == record)
    {
        return false;
    }

    return (APP_BOARD_TIME_RECORD_MAGIC == record->magic) &&
           (APP_BOARD_TIME_RECORD_VERSION == record->version) &&
           (sizeof(*record) == record->size) &&
           (0u != record->sequence) &&
           (0u != record->epoch_s) &&
           (record->checksum == board_time_checksum(record));
}

static bool board_time_read_slot(uint32_t slot, app_board_time_record_t *record)
{
    uint32_t addr = APP_BOARD_TIME_NVM_ADDR +
                    (slot * APP_BOARD_TIME_RECORD_SIZE);

    if ((slot >= APP_BOARD_TIME_RECORD_SLOTS) || (NULL == record))
    {
        return false;
    }

#if (APP_BOARD_TIME_NVM_ENABLE)
    if (CY_RRAM_SUCCESS !=
        Cy_RRAM_NvmReadByteArray(RRAMC0,
                                 addr,
                                 (uint8_t *)record,
                                 sizeof(*record)))
    {
        return false;
    }
#else
    (void)addr;
    memset(record, 0, sizeof(*record));
#endif

    return true;
}

static cy_rslt_t board_time_write_slot(uint32_t slot,
                                       const app_board_time_record_t *record)
{
    uint32_t addr = APP_BOARD_TIME_NVM_ADDR +
                    (slot * APP_BOARD_TIME_RECORD_SIZE);

    if ((slot >= APP_BOARD_TIME_RECORD_SLOTS) || (NULL == record))
    {
        return CY_RSLT_TYPE_ERROR;
    }

#if (APP_BOARD_TIME_NVM_ENABLE)
    if (CY_RRAM_SUCCESS !=
        Cy_RRAM_NvmWriteByteArray(RRAMC0,
                                  addr,
                                  (const uint8_t *)record,
                                  sizeof(*record)))
    {
        return CY_RSLT_TYPE_ERROR;
    }
#else
    (void)addr;
#endif

    return CY_RSLT_SUCCESS;
}

static bool board_time_source_should_persist(uint32_t source)
{
    if (APP_BOARD_TIME_SOURCE_BLE == source)
    {
        return true;
    }

#if (APP_BOARD_TIME_BOOTSTRAP_SAVE_NVM_ENABLE)
    if (APP_BOARD_TIME_SOURCE_BOOTSTRAP == source)
    {
        return true;
    }
#endif

    return false;
}

static void board_time_load_from_nvm(void)
{
    app_board_time_record_t best;
    bool have_best = false;
    uint32_t best_slot = 0u;

    memset(&best, 0, sizeof(best));

    for (uint32_t slot = 0u; slot < APP_BOARD_TIME_RECORD_SLOTS; ++slot)
    {
        app_board_time_record_t candidate;

        if ((!board_time_read_slot(slot, &candidate)) ||
            (!board_time_record_is_valid(&candidate)))
        {
            continue;
        }

        if ((!have_best) ||
            ((int32_t)(candidate.sequence - best.sequence) > 0))
        {
            best = candidate;
            best_slot = slot;
            have_best = true;
        }
    }

    if (have_best)
    {
        s_time_valid = true;
        s_base_epoch_s = best.epoch_s;
        s_base_uptime_s = board_time_uptime_s();
        s_sequence = best.sequence;
        s_source = best.source;
        s_flags = best.flags | APP_BOARD_TIME_FLAG_VALID |
                  APP_BOARD_TIME_FLAG_NVM_LOADED;
        s_next_slot = (best_slot + 1u) % APP_BOARD_TIME_RECORD_SLOTS;
    }
}

static void board_time_ensure_init(void)
{
    if (!s_initialized)
    {
        (void)app_board_time_init();
    }
}
