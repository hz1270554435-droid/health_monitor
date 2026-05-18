#include "app_ble_core.h"

#include <stdio.h>
#include <string.h>

#include "app_ble_config.h"
#include "app_ble_diag.h"
#include "app_ble_gatt.h"

#if (APP_BLE_ENABLE && APP_BLE_STACK_ENABLE)

#include "cycfg_peripherals.h"
#include "cycfg_pins.h"
#include "wiced_bt_adv_scan_common.h"
#include "wiced_bt_adv_scan_legacy.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_stack.h"

#if defined(CY_USING_HAL)
#error "BLE Stage 2A uses the HAL-next HCI-UART BSP path. CY_USING_HAL needs a separate platform config review."
#endif

#if !defined(CYBSP_BT_UART_HW) || !defined(CYBSP_BT_UART_IRQ)
#error "BLE Stage 2A requires CYBSP_BT_UART_HW and CYBSP_BT_UART_IRQ"
#endif

#if !defined(CYBSP_BT_UART_RX_PORT) || !defined(CYBSP_BT_UART_RX_PIN)
#error "BLE Stage 2A requires CYBSP_BT_UART_RX pin macros"
#endif

#if !defined(CYBSP_BT_UART_TX_PORT) || !defined(CYBSP_BT_UART_TX_PIN)
#error "BLE Stage 2A requires CYBSP_BT_UART_TX pin macros"
#endif

#if !defined(CYBSP_BT_UART_CTS_PORT) || !defined(CYBSP_BT_UART_CTS_PIN)
#error "BLE Stage 2A requires CYBSP_BT_UART_CTS pin macros"
#endif

#if !defined(CYBSP_BT_UART_RTS_PORT) || !defined(CYBSP_BT_UART_RTS_PIN)
#error "BLE Stage 2A requires CYBSP_BT_UART_RTS pin macros"
#endif

#if !defined(CYBSP_BT_POWER_PORT) || !defined(CYBSP_BT_POWER_PIN)
#error "BLE Stage 2A requires CYBSP_BT_POWER pin macros"
#endif

#if !defined(CYBSP_BT_HOST_WAKE_PORT) || !defined(CYBSP_BT_HOST_WAKE_PIN)
#error "BLE Stage 2A requires CYBSP_BT_HOST_WAKE pin macros"
#endif

#if !defined(CYBSP_BT_DEVICE_WAKE_PORT) || !defined(CYBSP_BT_DEVICE_WAKE_PIN)
#error "BLE Stage 2A requires CYBSP_BT_DEVICE_WAKE pin macros"
#endif

#define APP_BLE_CORE_DEFAULT_MTU          (23u)
#define APP_BLE_ADV_ELEMENT_COUNT         (2u)

static uint8_t ble_core_device_name[] = APP_BLE_DEVICE_NAME;
static uint8_t ble_core_adv_flags =
    (uint8_t)(BTM_BLE_GENERAL_DISCOVERABLE_FLAG |
              BTM_BLE_BREDR_NOT_SUPPORTED);

static const wiced_bt_cfg_ble_scan_settings_t ble_core_scan_cfg = {
    .scan_mode = BTM_BLE_SCAN_MODE_NONE,
    .high_duty_scan_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_SCAN_INTERVAL,
    .high_duty_scan_window = WICED_BT_CFG_DEFAULT_HIGH_DUTY_SCAN_WINDOW,
    .high_duty_scan_duration = 0u,
    .low_duty_scan_interval = WICED_BT_CFG_DEFAULT_LOW_DUTY_SCAN_INTERVAL,
    .low_duty_scan_window = WICED_BT_CFG_DEFAULT_LOW_DUTY_SCAN_WINDOW,
    .low_duty_scan_duration = 0u,
    .high_duty_conn_scan_interval =
        WICED_BT_CFG_DEFAULT_HIGH_DUTY_CONN_SCAN_INTERVAL,
    .high_duty_conn_scan_window =
        WICED_BT_CFG_DEFAULT_HIGH_DUTY_CONN_SCAN_WINDOW,
    .high_duty_conn_duration = 0u,
    .low_duty_conn_scan_interval =
        WICED_BT_CFG_DEFAULT_LOW_DUTY_CONN_SCAN_INTERVAL,
    .low_duty_conn_scan_window =
        WICED_BT_CFG_DEFAULT_LOW_DUTY_CONN_SCAN_WINDOW,
    .low_duty_conn_duration = 0u,
    .conn_min_interval = WICED_BT_CFG_DEFAULT_CONN_MIN_INTERVAL,
    .conn_max_interval = WICED_BT_CFG_DEFAULT_CONN_MAX_INTERVAL,
    .conn_latency = WICED_BT_CFG_DEFAULT_CONN_LATENCY,
    .conn_supervision_timeout =
        WICED_BT_CFG_DEFAULT_CONN_SUPERVISION_TIMEOUT
};

static const wiced_bt_cfg_ble_advert_settings_t ble_core_advert_cfg = {
    .channel_map = BTM_BLE_ADVERT_CHNL_37 |
                   BTM_BLE_ADVERT_CHNL_38 |
                   BTM_BLE_ADVERT_CHNL_39,
    .high_duty_min_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_ADV_MIN_INTERVAL,
    .high_duty_max_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_ADV_MAX_INTERVAL,
    .high_duty_duration = 0u,
    .low_duty_min_interval = WICED_BT_CFG_DEFAULT_LOW_DUTY_ADV_MIN_INTERVAL,
    .low_duty_max_interval = WICED_BT_CFG_DEFAULT_LOW_DUTY_ADV_MAX_INTERVAL,
    .low_duty_duration = 0u,
    .high_duty_directed_min_interval =
        WICED_BT_CFG_DEFAULT_HIGH_DUTY_DIRECTED_ADV_MIN_INTERVAL,
    .high_duty_directed_max_interval =
        WICED_BT_CFG_DEFAULT_HIGH_DUTY_DIRECTED_ADV_MAX_INTERVAL,
    .low_duty_directed_min_interval =
        WICED_BT_CFG_DEFAULT_LOW_DUTY_DIRECTED_ADV_MIN_INTERVAL,
    .low_duty_directed_max_interval =
        WICED_BT_CFG_DEFAULT_LOW_DUTY_DIRECTED_ADV_MAX_INTERVAL,
    .low_duty_directed_duration = 0u,
    .high_duty_nonconn_min_interval =
        WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MIN_INTERVAL,
    .high_duty_nonconn_max_interval =
        WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MAX_INTERVAL,
    .high_duty_nonconn_duration = 0u,
    .low_duty_nonconn_min_interval =
        WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MIN_INTERVAL,
    .low_duty_nonconn_max_interval =
        WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MAX_INTERVAL,
    .low_duty_nonconn_duration = 0u
};

static const wiced_bt_cfg_gatt_t ble_core_gatt_cfg = {
    .max_db_service_modules = 0u,
    .max_eatt_bearers = 0u
};

static const wiced_bt_cfg_ble_t ble_core_ble_cfg = {
    .ble_max_simultaneous_links = 1u,
    .ble_max_rx_pdu_size = 65u,
    .appearance = 0u,
    .rpa_refresh_timeout = 0u,
    .host_addr_resolution_db_size = 0u,
    .p_ble_scan_cfg = &ble_core_scan_cfg,
    .p_ble_advert_cfg = &ble_core_advert_cfg,
    .default_ble_power_level = 0
};

static const wiced_bt_cfg_settings_t ble_core_stack_cfg = {
    .device_name = ble_core_device_name,
    .security_required = BTM_SEC_BEST_EFFORT,
    .p_br_cfg = NULL,
    .p_ble_cfg = &ble_core_ble_cfg,
    .p_gatt_cfg = &ble_core_gatt_cfg,
    .p_isoc_cfg = NULL,
    .p_l2cap_app_cfg = NULL
};

static bool ble_core_init_started;
static bool ble_core_enabled;
static bool ble_core_connected;
static bool ble_core_advertising;
static bool ble_core_gatt_registered;
static uint16_t ble_core_conn_id;
static uint16_t ble_core_mtu = APP_BLE_CORE_DEFAULT_MTU;

static wiced_result_t app_ble_core_management_callback(
    wiced_bt_management_evt_t event,
    wiced_bt_management_evt_data_t *event_data);
static wiced_bt_gatt_status_t app_ble_core_gatt_callback(
    wiced_bt_gatt_evt_t event,
    wiced_bt_gatt_event_data_t *event_data);
static cy_rslt_t app_ble_core_start_advertising(bool restarted);
static cy_rslt_t app_ble_core_set_adv_data(void);
static void app_ble_core_note_stack_error(const char *detail,
                                          wiced_result_t result);

cy_rslt_t app_ble_core_init(void)
{
    wiced_result_t result;

    if (ble_core_init_started)
    {
        return CY_RSLT_SUCCESS;
    }

    ble_core_init_started = true;
    ble_core_enabled = false;
    ble_core_connected = false;
    ble_core_advertising = false;
    ble_core_gatt_registered = false;
    ble_core_conn_id = 0u;
    ble_core_mtu = APP_BLE_CORE_DEFAULT_MTU;
    app_ble_diag_set_connected(false);
    app_ble_diag_set_mtu(ble_core_mtu);

#if (APP_BLE_LOG_LEVEL >= APP_BLE_LOG_LEVEL_STAT)
    printf("[BLE_CORE] init\r\n");
#endif

    result = wiced_bt_stack_init(app_ble_core_management_callback,
                                 &ble_core_stack_cfg);
    if (WICED_BT_SUCCESS != result)
    {
        app_ble_core_note_stack_error("stack_init", result);
        ble_core_init_started = false;
        return CY_RSLT_TYPE_ERROR;
    }

    return CY_RSLT_SUCCESS;
}

bool app_ble_core_is_enabled(void)
{
    return ble_core_enabled;
}

bool app_ble_core_is_stack_enabled(void)
{
    return app_ble_core_is_enabled();
}

bool app_ble_core_is_connected(void)
{
    return ble_core_connected;
}

bool app_ble_core_is_advertising(void)
{
    return ble_core_advertising;
}

uint16_t app_ble_core_get_conn_id(void)
{
    return ble_core_conn_id;
}

uint16_t app_ble_core_get_mtu(void)
{
    return ble_core_mtu;
}

static wiced_result_t app_ble_core_management_callback(
    wiced_bt_management_evt_t event,
    wiced_bt_management_evt_data_t *event_data)
{
    switch (event)
    {
        case BTM_ENABLED_EVT:
        {
            wiced_result_t status =
                (NULL != event_data) ? event_data->enabled.status :
                                       WICED_BT_ERROR;

            if (WICED_BT_SUCCESS != status)
            {
                app_ble_core_note_stack_error("enabled_evt", status);
                return status;
            }

            ble_core_enabled = true;
            app_ble_diag_set_mtu(ble_core_mtu);

#if (APP_BLE_LOG_LEVEL >= APP_BLE_LOG_LEVEL_STAT)
            printf("[BLE_CORE] stack enabled\r\n");
#endif

            if (CY_RSLT_SUCCESS != app_ble_core_set_adv_data())
            {
                return WICED_BT_ERROR;
            }

            if (!ble_core_gatt_registered)
            {
                wiced_bt_gatt_status_t gatt_status =
                    wiced_bt_gatt_register(app_ble_core_gatt_callback);

                if (WICED_BT_GATT_SUCCESS != gatt_status)
                {
                    app_ble_diag_set_last_error(APP_BLE_ERR_STACK_FAILED);
#if (APP_BLE_LOG_LEVEL >= APP_BLE_LOG_LEVEL_STAT)
                    printf("[BLE_ERR] code=STACK_FAILED detail=gatt_register "
                           "result=0x%lx\r\n",
                           (unsigned long)gatt_status);
#endif
                    return WICED_BT_ERROR;
                }

                ble_core_gatt_registered = true;
            }

            if (CY_RSLT_SUCCESS != app_ble_gatt_init())
            {
                return WICED_BT_ERROR;
            }

            if (CY_RSLT_SUCCESS != app_ble_core_start_advertising(false))
            {
                return WICED_BT_ERROR;
            }
            break;
        }

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
            if (NULL != event_data)
            {
                ble_core_advertising =
                    (BTM_BLE_ADVERT_OFF !=
                     event_data->ble_advert_state_changed);
            }
            break;

        case BTM_DISABLED_EVT:
            ble_core_enabled = false;
            ble_core_connected = false;
            ble_core_advertising = false;
            app_ble_diag_set_connected(false);
            break;

        default:
            break;
    }

    return WICED_BT_SUCCESS;
}

static wiced_bt_gatt_status_t app_ble_core_gatt_callback(
    wiced_bt_gatt_evt_t event,
    wiced_bt_gatt_event_data_t *event_data)
{
    if ((GATT_CONNECTION_STATUS_EVT == event) && (NULL != event_data))
    {
        const wiced_bt_gatt_connection_status_t *status =
            &event_data->connection_status;

        if (status->connected)
        {
            ble_core_connected = true;
            ble_core_advertising = false;
            ble_core_conn_id = status->conn_id;
            app_ble_diag_set_connected(true);
#if (APP_BLE_LOG_LEVEL >= APP_BLE_LOG_LEVEL_STAT)
            printf("[BLE_CONN] connected conn_id=%u\r\n",
                   (unsigned int)ble_core_conn_id);
#endif
        }
        else
        {
            ble_core_connected = false;
            ble_core_advertising = false;
            ble_core_conn_id = 0u;
            ble_core_mtu = APP_BLE_CORE_DEFAULT_MTU;
            app_ble_diag_set_mtu(ble_core_mtu);
            app_ble_diag_note_disconnect();
            app_ble_gatt_on_disconnected();
#if (APP_BLE_LOG_LEVEL >= APP_BLE_LOG_LEVEL_STAT)
            printf("[BLE_CONN] disconnected reason=0x%02x\r\n",
                   (unsigned int)status->reason);
#endif
            (void)app_ble_core_start_advertising(true);
        }
    }

    if (GATT_CONNECTION_STATUS_EVT != event)
    {
        return app_ble_gatt_on_event(event, event_data);
    }

    return WICED_BT_GATT_SUCCESS;
}

static cy_rslt_t app_ble_core_start_advertising(bool restarted)
{
    wiced_result_t result;

    if (!ble_core_enabled)
    {
        return CY_RSLT_TYPE_ERROR;
    }

    result = wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH,
                                           0u,
                                           NULL);
    if (WICED_BT_SUCCESS != result)
    {
        app_ble_core_note_stack_error("adv_start", result);
        ble_core_advertising = false;
        return CY_RSLT_TYPE_ERROR;
    }

    ble_core_advertising = true;

#if (APP_BLE_LOG_LEVEL >= APP_BLE_LOG_LEVEL_STAT)
    if (restarted)
    {
        printf("[BLE_ADV] restarted\r\n");
    }
    else
    {
        printf("[BLE_ADV] started name=%s\r\n", APP_BLE_DEVICE_NAME);
    }
#else
    (void)restarted;
#endif

    return CY_RSLT_SUCCESS;
}

static cy_rslt_t app_ble_core_set_adv_data(void)
{
    wiced_bt_ble_advert_elem_t adv_data[APP_BLE_ADV_ELEMENT_COUNT];
    wiced_result_t result;

    memset(adv_data, 0, sizeof(adv_data));
    adv_data[0].advert_type = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_data[0].len = sizeof(ble_core_adv_flags);
    adv_data[0].p_data = &ble_core_adv_flags;
    adv_data[1].advert_type = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_data[1].len = (uint16_t)(sizeof(ble_core_device_name) - 1u);
    adv_data[1].p_data = ble_core_device_name;

    result = wiced_bt_ble_set_raw_advertisement_data(APP_BLE_ADV_ELEMENT_COUNT,
                                                     adv_data);
    if (WICED_BT_SUCCESS != result)
    {
        app_ble_core_note_stack_error("adv_data", result);
        return CY_RSLT_TYPE_ERROR;
    }

    return CY_RSLT_SUCCESS;
}

static void app_ble_core_note_stack_error(const char *detail,
                                          wiced_result_t result)
{
    app_ble_diag_set_last_error(APP_BLE_ERR_STACK_FAILED);

#if (APP_BLE_LOG_LEVEL >= APP_BLE_LOG_LEVEL_STAT)
    printf("[BLE_ERR] code=STACK_FAILED detail=%s result=0x%lx\r\n",
           (NULL != detail) ? detail : "unknown",
           (unsigned long)result);
#else
    (void)detail;
    (void)result;
#endif
}

#endif /* APP_BLE_ENABLE && APP_BLE_STACK_ENABLE */
