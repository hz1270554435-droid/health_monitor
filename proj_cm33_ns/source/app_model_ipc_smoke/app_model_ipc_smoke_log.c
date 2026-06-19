#include "app_model_ipc_smoke.h"

#if (APP_MODEL_IPC_SMOKE_EXCLUSIVE_ENABLE)

#include "FreeRTOS.h"
#include "semphr.h"

static StaticSemaphore_t ipc_smoke_log_mutex_storage;
static SemaphoreHandle_t ipc_smoke_log_mutex;

void app_model_ipc_smoke_log_init(void)
{
    if (NULL == ipc_smoke_log_mutex)
    {
        ipc_smoke_log_mutex =
            xSemaphoreCreateMutexStatic(&ipc_smoke_log_mutex_storage);
        configASSERT(NULL != ipc_smoke_log_mutex);
    }
}

void app_model_ipc_smoke_log_lock(void)
{
    configASSERT(NULL != ipc_smoke_log_mutex);
    (void)xSemaphoreTake(ipc_smoke_log_mutex, portMAX_DELAY);
}

void app_model_ipc_smoke_log_unlock(void)
{
    configASSERT(NULL != ipc_smoke_log_mutex);
    (void)xSemaphoreGive(ipc_smoke_log_mutex);
}

#endif
