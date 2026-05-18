#include "app_ble_gatt.h"

#include <stdio.h>
#include <string.h>

#include "app_ble_core.h"
#include "app_ble_diag.h"
#include "app_ble_stream.h"

#if (APP_BLE_ENABLE && APP_BLE_STACK_ENABLE)

#include "gattdefs.h"

#define APP_BLE_GATT_DEFAULT_MTU             (23u)

#define APP_BLE_GATT_HANDLE_SERVICE          (0x0100u)
#define APP_BLE_GATT_HANDLE_RT_DECL          (0x0101u)
/* Handles from static GATT DB:
 * 0x0102 = Realtime characteristic value handle
 * 0x0103 = Realtime CCCD handle
 */
#define APP_BLE_HANDLE_REALTIME_VALUE        (0x0102u)
#define APP_BLE_HANDLE_REALTIME_CCCD         (0x0103u)
#define APP_BLE_GATT_HANDLE_EVT_DECL         (0x0104u)
/* Handles from static GATT DB:
 * 0x0105 = Alert Event characteristic value handle
 * 0x0106 = Alert Event CCCD handle
 */
#define APP_BLE_HANDLE_EVENT_VALUE           (0x0105u)
#define APP_BLE_HANDLE_EVENT_CCCD            (0x0106u)
/* Handles from static GATT DB:
 * 0x0108 = Command characteristic value handle
 */
#define APP_BLE_GATT_HANDLE_CMD_DECL         (0x0107u)
#define APP_BLE_GATT_HANDLE_CMD_VALUE        (0x0108u)
#define APP_BLE_GATT_HANDLE_RSP_DECL         (0x0109u)
/* Handles from static GATT DB:
 * 0x010A = Command Response characteristic value handle
 * 0x010B = Command Response CCCD handle
 */
#define APP_BLE_HANDLE_CMD_RESPONSE_VALUE    (0x010Au)
#define APP_BLE_HANDLE_CMD_RESPONSE_CCCD     (0x010Bu)

#define APP_BLE_UUID128_LE(id) \
    0x01u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x10u, 0x9Eu, \
    0x3Du, 0x4Cu, 0x2Bu, 0x1Au, \
    (uint8_t)((id) & 0xFFu), (uint8_t)(((id) >> 8u) & 0xFFu), \
    0x84u, 0x7Eu

static const uint8_t app_ble_gatt_db[] = {
    PRIMARY_SERVICE_UUID128(APP_BLE_GATT_HANDLE_SERVICE,
                            APP_BLE_UUID128_LE(0x0000u)),

    CHARACTERISTIC_UUID128(APP_BLE_GATT_HANDLE_RT_DECL,
                           APP_BLE_HANDLE_REALTIME_VALUE,
                           APP_BLE_UUID128_LE(0x0002u),
                           GATTDB_CHAR_PROP_NOTIFY,
                           GATTDB_PERM_NONE),
    CHAR_DESCRIPTOR_UUID16_WRITABLE(APP_BLE_HANDLE_REALTIME_CCCD,
                                    GATT_UUID_CHAR_CLIENT_CONFIG,
                                    (GATTDB_PERM_READABLE |
                                     GATTDB_PERM_WRITE_REQ)),

    CHARACTERISTIC_UUID128(APP_BLE_GATT_HANDLE_EVT_DECL,
                           APP_BLE_HANDLE_EVENT_VALUE,
                           APP_BLE_UUID128_LE(0x0003u),
                           GATTDB_CHAR_PROP_NOTIFY,
                           GATTDB_PERM_NONE),
    CHAR_DESCRIPTOR_UUID16_WRITABLE(APP_BLE_HANDLE_EVENT_CCCD,
                                    GATT_UUID_CHAR_CLIENT_CONFIG,
                                    (GATTDB_PERM_READABLE |
                                     GATTDB_PERM_WRITE_REQ)),

    CHARACTERISTIC_UUID128_WRITABLE(APP_BLE_GATT_HANDLE_CMD_DECL,
                                    APP_BLE_GATT_HANDLE_CMD_VALUE,
                                    APP_BLE_UUID128_LE(0x0004u),
                                    GATTDB_CHAR_PROP_WRITE,
                                    GATTDB_PERM_WRITE_REQ),

    CHARACTERISTIC_UUID128(APP_BLE_GATT_HANDLE_RSP_DECL,
                           APP_BLE_HANDLE_CMD_RESPONSE_VALUE,
                           APP_BLE_UUID128_LE(0x0005u),
                           GATTDB_CHAR_PROP_NOTIFY,
                           GATTDB_PERM_NONE),
    CHAR_DESCRIPTOR_UUID16_WRITABLE(APP_BLE_HANDLE_CMD_RESPONSE_CCCD,
                                    GATT_UUID_CHAR_CLIENT_CONFIG,
                                    (GATTDB_PERM_READABLE |
                                     GATTDB_PERM_WRITE_REQ))
};

static bool app_ble_gatt_ready;
static uint16_t app_ble_gatt_rt_cccd = GATT_CLIENT_CONFIG_NONE;
static uint16_t app_ble_gatt_evt_cccd = GATT_CLIENT_CONFIG_NONE;
static uint16_t app_ble_gatt_rsp_cccd = GATT_CLIENT_CONFIG_NONE;
static uint8_t app_ble_gatt_read_rsp[2];
static uint8_t app_ble_gatt_rt_tx_buf[APP_BLE_REALTIME_FRAME_LEN];
static uint8_t app_ble_gatt_evt_tx_buf[APP_BLE_EVENT_FRAME_LEN];
static uint8_t app_ble_gatt_rsp_tx_buf[APP_BLE_CMD_RESPONSE_FRAME_MAX_LEN];
static const uint8_t app_ble_gatt_rt_context = APP_BLE_MSG_REALTIME;
static const uint8_t app_ble_gatt_evt_context = APP_BLE_MSG_EVENT;
static const uint8_t app_ble_gatt_rsp_context = APP_BLE_MSG_COMMAND_RESPONSE;
static volatile bool app_ble_gatt_rt_tx_busy;
static volatile bool app_ble_gatt_evt_tx_busy;
static volatile bool app_ble_gatt_rsp_tx_busy;

static wiced_bt_gatt_status_t app_ble_gatt_handle_attribute_request(
    wiced_bt_gatt_attribute_request_t *request);
static wiced_bt_gatt_status_t app_ble_gatt_handle_read(
    const wiced_bt_gatt_attribute_request_t *request);
static wiced_bt_gatt_status_t app_ble_gatt_handle_write(
    const wiced_bt_gatt_attribute_request_t *request);
static uint16_t *app_ble_gatt_find_cccd(uint16_t handle);
static wiced_bt_gatt_status_t app_ble_gatt_send_error(
    const wiced_bt_gatt_attribute_request_t *request,
    uint16_t handle,
    wiced_bt_gatt_status_t status);
static bool app_ble_gatt_is_realtime_tx_done(
    const wiced_bt_gatt_buffer_transmitted_t *tx);
static bool app_ble_gatt_is_event_tx_done(
    const wiced_bt_gatt_buffer_transmitted_t *tx);
static bool app_ble_gatt_is_cmd_response_tx_done(
    const wiced_bt_gatt_buffer_transmitted_t *tx);

cy_rslt_t app_ble_gatt_init(void)
{
    wiced_bt_gatt_status_t status;

    if (app_ble_gatt_ready)
    {
        return CY_RSLT_SUCCESS;
    }

    status = wiced_bt_gatt_db_init(app_ble_gatt_db,
                                   (uint16_t)sizeof(app_ble_gatt_db),
                                   NULL);
    if (WICED_BT_GATT_SUCCESS != status)
    {
        app_ble_diag_set_last_error(APP_BLE_ERR_STACK_FAILED);
#if (APP_BLE_LOG_LEVEL >= APP_BLE_LOG_LEVEL_STAT)
        printf("[BLE_ERR] code=STACK_FAILED detail=gatt_db_init "
               "result=0x%lx\r\n",
               (unsigned long)status);
#endif
        return CY_RSLT_TYPE_ERROR;
    }

    app_ble_gatt_rt_cccd = GATT_CLIENT_CONFIG_NONE;
    app_ble_gatt_evt_cccd = GATT_CLIENT_CONFIG_NONE;
    app_ble_gatt_rsp_cccd = GATT_CLIENT_CONFIG_NONE;
    app_ble_gatt_rt_tx_busy = false;
    app_ble_gatt_evt_tx_busy = false;
    app_ble_gatt_rsp_tx_busy = false;
    app_ble_gatt_ready = true;

#if (APP_BLE_LOG_LEVEL >= APP_BLE_LOG_LEVEL_STAT)
    printf("[BLE_GATT] db init ok\r\n");
#endif

    return CY_RSLT_SUCCESS;
}

bool app_ble_gatt_is_ready(void)
{
    return app_ble_gatt_ready;
}

bool app_ble_gatt_is_available(void)
{
    return app_ble_gatt_is_ready();
}

bool app_ble_gatt_is_realtime_subscribed(void)
{
    return app_ble_gatt_ready &&
           (GATT_CLIENT_CONFIG_NOTIFICATION == app_ble_gatt_rt_cccd);
}

bool app_ble_gatt_is_realtime_tx_busy(void)
{
    return app_ble_gatt_rt_tx_busy;
}

bool app_ble_gatt_is_event_subscribed(void)
{
    return app_ble_gatt_ready &&
           (GATT_CLIENT_CONFIG_NOTIFICATION == app_ble_gatt_evt_cccd);
}

bool app_ble_gatt_is_event_tx_busy(void)
{
    return app_ble_gatt_evt_tx_busy;
}

bool app_ble_gatt_is_cmd_response_subscribed(void)
{
    return app_ble_gatt_ready &&
           (GATT_CLIENT_CONFIG_NOTIFICATION == app_ble_gatt_rsp_cccd);
}

bool app_ble_gatt_is_cmd_response_tx_busy(void)
{
    return app_ble_gatt_rsp_tx_busy;
}

void app_ble_gatt_on_disconnected(void)
{
    app_ble_gatt_rt_cccd = GATT_CLIENT_CONFIG_NONE;
    app_ble_gatt_evt_cccd = GATT_CLIENT_CONFIG_NONE;
    app_ble_gatt_rsp_cccd = GATT_CLIENT_CONFIG_NONE;
    app_ble_gatt_rt_tx_busy = false;
    app_ble_gatt_evt_tx_busy = false;
    app_ble_gatt_rsp_tx_busy = false;
}

cy_rslt_t app_ble_gatt_notify_realtime(const uint8_t *data, uint16_t len)
{
    uint16_t conn_id;
    uint16_t mtu;
    wiced_bt_gatt_status_t status;

    if (!app_ble_gatt_ready || !app_ble_core_is_enabled())
    {
        app_ble_diag_note_realtime_no_sub();
        return (cy_rslt_t)APP_BLE_ERR_NOT_INIT;
    }

    if (!app_ble_core_is_connected())
    {
        app_ble_diag_note_realtime_no_sub();
        return (cy_rslt_t)APP_BLE_ERR_NOT_CONNECTED;
    }

    conn_id = app_ble_core_get_conn_id();
    if (0u == conn_id)
    {
        app_ble_diag_note_realtime_no_sub();
        return (cy_rslt_t)APP_BLE_ERR_NOT_CONNECTED;
    }

    if (!app_ble_gatt_is_realtime_subscribed())
    {
        app_ble_diag_note_realtime_no_sub();
        return (cy_rslt_t)APP_BLE_ERR_NOT_SUBSCRIBED;
    }

    if ((NULL == data) || (0u == len))
    {
        app_ble_diag_note_realtime_oversize_drop();
        return (cy_rslt_t)APP_BLE_ERR_INVALID_ARG;
    }

    mtu = app_ble_core_get_mtu();
    if ((3u > mtu) || (len > (uint16_t)(mtu - 3u)) ||
        (sizeof(app_ble_gatt_rt_tx_buf) < len))
    {
        app_ble_diag_note_realtime_oversize_drop();
        return (cy_rslt_t)APP_BLE_ERR_INVALID_ARG;
    }

    if (app_ble_gatt_rt_tx_busy)
    {
        app_ble_diag_note_realtime_busy_drop();
        return (cy_rslt_t)APP_BLE_ERR_BUSY;
    }

    memcpy(app_ble_gatt_rt_tx_buf, data, len);
    app_ble_gatt_rt_tx_busy = true;
    status = wiced_bt_gatt_server_send_notification(
        conn_id,
        APP_BLE_HANDLE_REALTIME_VALUE,
        len,
        app_ble_gatt_rt_tx_buf,
        (wiced_bt_gatt_app_context_t)&app_ble_gatt_rt_context);

    if (WICED_BT_GATT_SUCCESS != status)
    {
        app_ble_gatt_rt_tx_busy = false;
        app_ble_diag_note_transport_error(APP_BLE_ERR_STACK_FAILED);
        return (cy_rslt_t)APP_BLE_ERR_STACK_FAILED;
    }

    return CY_RSLT_SUCCESS;
}

cy_rslt_t app_ble_gatt_notify_event(const uint8_t *data, uint16_t len)
{
    uint16_t conn_id;
    uint16_t mtu;
    wiced_bt_gatt_status_t status;

    if (!app_ble_gatt_ready || !app_ble_core_is_enabled())
    {
        app_ble_diag_note_event_no_sub();
        return (cy_rslt_t)APP_BLE_ERR_NOT_INIT;
    }

    if (!app_ble_core_is_connected())
    {
        app_ble_diag_note_event_no_sub();
        return (cy_rslt_t)APP_BLE_ERR_NOT_CONNECTED;
    }

    conn_id = app_ble_core_get_conn_id();
    if (0u == conn_id)
    {
        app_ble_diag_note_event_no_sub();
        return (cy_rslt_t)APP_BLE_ERR_NOT_CONNECTED;
    }

    if (!app_ble_gatt_is_event_subscribed())
    {
        app_ble_diag_note_event_no_sub();
        return (cy_rslt_t)APP_BLE_ERR_NOT_SUBSCRIBED;
    }

    if ((NULL == data) || (0u == len))
    {
        app_ble_diag_note_event_oversize_drop();
        return (cy_rslt_t)APP_BLE_ERR_INVALID_ARG;
    }

    mtu = app_ble_core_get_mtu();
    if ((3u > mtu) || (len > (uint16_t)(mtu - 3u)) ||
        (sizeof(app_ble_gatt_evt_tx_buf) < len))
    {
        app_ble_diag_note_event_oversize_drop();
        return (cy_rslt_t)APP_BLE_ERR_INVALID_ARG;
    }

    if (app_ble_gatt_evt_tx_busy)
    {
        app_ble_diag_note_event_busy_drop();
        return (cy_rslt_t)APP_BLE_ERR_BUSY;
    }

    memcpy(app_ble_gatt_evt_tx_buf, data, len);
    app_ble_gatt_evt_tx_busy = true;
    status = wiced_bt_gatt_server_send_notification(
        conn_id,
        APP_BLE_HANDLE_EVENT_VALUE,
        len,
        app_ble_gatt_evt_tx_buf,
        (wiced_bt_gatt_app_context_t)&app_ble_gatt_evt_context);

    if (WICED_BT_GATT_SUCCESS != status)
    {
        app_ble_gatt_evt_tx_busy = false;
        app_ble_diag_note_transport_error(APP_BLE_ERR_STACK_FAILED);
        return (cy_rslt_t)APP_BLE_ERR_STACK_FAILED;
    }

    return CY_RSLT_SUCCESS;
}

cy_rslt_t app_ble_gatt_notify_cmd_response(const uint8_t *data, uint16_t len)
{
    uint16_t conn_id;
    uint16_t mtu;
    wiced_bt_gatt_status_t status;

    if (!app_ble_gatt_ready || !app_ble_core_is_enabled())
    {
        app_ble_diag_note_cmd_rsp_no_sub();
        return (cy_rslt_t)APP_BLE_ERR_NOT_INIT;
    }

    if (!app_ble_core_is_connected())
    {
        app_ble_diag_note_cmd_rsp_no_sub();
        return (cy_rslt_t)APP_BLE_ERR_NOT_CONNECTED;
    }

    conn_id = app_ble_core_get_conn_id();
    if (0u == conn_id)
    {
        app_ble_diag_note_cmd_rsp_no_sub();
        return (cy_rslt_t)APP_BLE_ERR_NOT_CONNECTED;
    }

    if (!app_ble_gatt_is_cmd_response_subscribed())
    {
        app_ble_diag_note_cmd_rsp_no_sub();
        return (cy_rslt_t)APP_BLE_ERR_NOT_SUBSCRIBED;
    }

    if ((NULL == data) || (0u == len))
    {
        app_ble_diag_note_cmd_rsp_oversize_drop();
        return (cy_rslt_t)APP_BLE_ERR_INVALID_ARG;
    }

    mtu = app_ble_core_get_mtu();
    if ((3u > mtu) || (len > (uint16_t)(mtu - 3u)) ||
        (sizeof(app_ble_gatt_rsp_tx_buf) < len))
    {
        app_ble_diag_note_cmd_rsp_oversize_drop();
        return (cy_rslt_t)APP_BLE_ERR_INVALID_ARG;
    }

    if (app_ble_gatt_rsp_tx_busy)
    {
        app_ble_diag_note_cmd_rsp_busy_drop();
        return (cy_rslt_t)APP_BLE_ERR_BUSY;
    }

    memcpy(app_ble_gatt_rsp_tx_buf, data, len);
    app_ble_gatt_rsp_tx_busy = true;
    status = wiced_bt_gatt_server_send_notification(
        conn_id,
        APP_BLE_HANDLE_CMD_RESPONSE_VALUE,
        len,
        app_ble_gatt_rsp_tx_buf,
        (wiced_bt_gatt_app_context_t)&app_ble_gatt_rsp_context);

    if (WICED_BT_GATT_SUCCESS != status)
    {
        app_ble_gatt_rsp_tx_busy = false;
        app_ble_diag_note_cmd_rsp_transport_error(APP_BLE_ERR_STACK_FAILED);
        return (cy_rslt_t)APP_BLE_ERR_STACK_FAILED;
    }

    return CY_RSLT_SUCCESS;
}

wiced_bt_gatt_status_t app_ble_gatt_on_event(
    wiced_bt_gatt_evt_t event,
    wiced_bt_gatt_event_data_t *event_data)
{
    if ((GATT_ATTRIBUTE_REQUEST_EVT == event) && (NULL != event_data))
    {
        return app_ble_gatt_handle_attribute_request(
            &event_data->attribute_request);
    }

    if ((GATT_APP_BUFFER_TRANSMITTED_EVT == event) && (NULL != event_data))
    {
        if (app_ble_gatt_is_realtime_tx_done(&event_data->buffer_xmitted))
        {
            app_ble_gatt_rt_tx_busy = false;
        }
        else if (app_ble_gatt_is_event_tx_done(&event_data->buffer_xmitted))
        {
            app_ble_gatt_evt_tx_busy = false;
        }
        else if (app_ble_gatt_is_cmd_response_tx_done(
                     &event_data->buffer_xmitted))
        {
            app_ble_gatt_rsp_tx_busy = false;
        }
        return WICED_BT_GATT_SUCCESS;
    }

    return WICED_BT_GATT_SUCCESS;
}

static wiced_bt_gatt_status_t app_ble_gatt_handle_attribute_request(
    wiced_bt_gatt_attribute_request_t *request)
{
    if (NULL == request)
    {
        return WICED_BT_GATT_INVALID_PDU;
    }

    switch (request->opcode)
    {
        case GATT_REQ_MTU:
            return wiced_bt_gatt_server_send_mtu_rsp(
                request->conn_id,
                request->data.remote_mtu,
                APP_BLE_GATT_DEFAULT_MTU);

        case GATT_REQ_READ:
        case GATT_REQ_READ_BLOB:
            return app_ble_gatt_handle_read(request);

        case GATT_REQ_WRITE:
        case GATT_CMD_WRITE:
            return app_ble_gatt_handle_write(request);

        default:
            return app_ble_gatt_send_error(request,
                                           0u,
                                           WICED_BT_GATT_REQ_NOT_SUPPORTED);
    }
}

static wiced_bt_gatt_status_t app_ble_gatt_handle_read(
    const wiced_bt_gatt_attribute_request_t *request)
{
    uint16_t *cccd;
    uint16_t handle;

    handle = request->data.read_req.handle;
    cccd = app_ble_gatt_find_cccd(handle);

    if (NULL == cccd)
    {
        return app_ble_gatt_send_error(request,
                                       handle,
                                       WICED_BT_GATT_ATTRIBUTE_NOT_FOUND);
    }

    if (0u != request->data.read_req.offset)
    {
        return app_ble_gatt_send_error(request,
                                       handle,
                                       WICED_BT_GATT_INVALID_OFFSET);
    }

    app_ble_gatt_read_rsp[0] = (uint8_t)(*cccd & 0xFFu);
    app_ble_gatt_read_rsp[1] = (uint8_t)((*cccd >> 8u) & 0xFFu);

    return wiced_bt_gatt_server_send_read_handle_rsp(
        request->conn_id,
        request->opcode,
        sizeof(app_ble_gatt_read_rsp),
        app_ble_gatt_read_rsp,
        NULL);
}

static wiced_bt_gatt_status_t app_ble_gatt_handle_write(
    const wiced_bt_gatt_attribute_request_t *request)
{
    uint16_t *cccd;
    uint16_t handle;
    uint16_t value;

    handle = request->data.write_req.handle;

    if (APP_BLE_GATT_HANDLE_CMD_VALUE == handle)
    {
        if ((NULL == request->data.write_req.p_val) ||
            (0u == request->data.write_req.val_len) ||
            (APP_BLE_COMMAND_FRAME_MAX_LEN < request->data.write_req.val_len))
        {
            app_ble_diag_note_cmd_drop(APP_BLE_ERR_INVALID_ARG);
            return app_ble_gatt_send_error(request,
                                           handle,
                                           WICED_BT_GATT_INVALID_ATTR_LEN);
        }

        if (CY_RSLT_SUCCESS != app_ble_stream_enqueue_raw_command(
                request->data.write_req.p_val,
                request->data.write_req.val_len))
        {
            return app_ble_gatt_send_error(request,
                                           handle,
                                           WICED_BT_GATT_INSUF_RESOURCE);
        }

        if (GATT_REQ_WRITE == request->opcode)
        {
            return wiced_bt_gatt_server_send_write_rsp(request->conn_id,
                                                      request->opcode,
                                                      handle);
        }

        return WICED_BT_GATT_SUCCESS;
    }

    cccd = app_ble_gatt_find_cccd(handle);
    if (NULL == cccd)
    {
        return app_ble_gatt_send_error(request,
                                       handle,
                                       WICED_BT_GATT_ATTRIBUTE_NOT_FOUND);
    }

    if ((2u != request->data.write_req.val_len) ||
        (NULL == request->data.write_req.p_val))
    {
        return app_ble_gatt_send_error(request,
                                       handle,
                                       WICED_BT_GATT_INVALID_ATTR_LEN);
    }

    value = (uint16_t)request->data.write_req.p_val[0] |
            ((uint16_t)request->data.write_req.p_val[1] << 8u);
    if ((GATT_CLIENT_CONFIG_NONE != value) &&
        (GATT_CLIENT_CONFIG_NOTIFICATION != value))
    {
        return app_ble_gatt_send_error(request,
                                       handle,
                                       WICED_BT_GATT_WRITE_REQ_REJECTED);
    }

    *cccd = value;

    if (GATT_REQ_WRITE == request->opcode)
    {
        return wiced_bt_gatt_server_send_write_rsp(request->conn_id,
                                                  request->opcode,
                                                  handle);
    }

    return WICED_BT_GATT_SUCCESS;
}

static uint16_t *app_ble_gatt_find_cccd(uint16_t handle)
{
    switch (handle)
    {
        case APP_BLE_HANDLE_REALTIME_CCCD:
            return &app_ble_gatt_rt_cccd;

        case APP_BLE_HANDLE_EVENT_CCCD:
            return &app_ble_gatt_evt_cccd;

        case APP_BLE_HANDLE_CMD_RESPONSE_CCCD:
            return &app_ble_gatt_rsp_cccd;

        default:
            return NULL;
    }
}

static wiced_bt_gatt_status_t app_ble_gatt_send_error(
    const wiced_bt_gatt_attribute_request_t *request,
    uint16_t handle,
    wiced_bt_gatt_status_t status)
{
    if (NULL == request)
    {
        return status;
    }

    if (GATT_CMD_WRITE == request->opcode)
    {
        return status;
    }

    return wiced_bt_gatt_server_send_error_rsp(request->conn_id,
                                              request->opcode,
                                              handle,
                                              status);
}

static bool app_ble_gatt_is_realtime_tx_done(
    const wiced_bt_gatt_buffer_transmitted_t *tx)
{
    return (NULL != tx) &&
           (tx->p_app_data == app_ble_gatt_rt_tx_buf) &&
           (tx->p_app_ctxt ==
            (wiced_bt_gatt_app_context_t)&app_ble_gatt_rt_context);
}

static bool app_ble_gatt_is_event_tx_done(
    const wiced_bt_gatt_buffer_transmitted_t *tx)
{
    return (NULL != tx) &&
           (tx->p_app_data == app_ble_gatt_evt_tx_buf) &&
           (tx->p_app_ctxt ==
            (wiced_bt_gatt_app_context_t)&app_ble_gatt_evt_context);
}

static bool app_ble_gatt_is_cmd_response_tx_done(
    const wiced_bt_gatt_buffer_transmitted_t *tx)
{
    return (NULL != tx) &&
           (tx->p_app_data == app_ble_gatt_rsp_tx_buf) &&
           (tx->p_app_ctxt ==
            (wiced_bt_gatt_app_context_t)&app_ble_gatt_rsp_context);
}

#endif /* APP_BLE_ENABLE && APP_BLE_STACK_ENABLE */
