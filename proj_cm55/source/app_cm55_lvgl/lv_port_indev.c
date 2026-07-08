/**
 * @file lv_port_indev.c
 * LVGL touch input port for the Waveshare 4.3" FT5406 controller.
 */

#include "lv_port_indev.h"

#include "lvgl.h"
#include "mtb_ctp_ft5406.h"

#define TOUCH_VISIBLE_W (800)
#define TOUCH_VISIBLE_H (480)
#define TOUCH_MAX_X     (TOUCH_VISIBLE_W - 1)
#define TOUCH_MAX_Y     (TOUCH_VISIBLE_H - 1)
#define TOUCH_SWIPE_MIN_X (80)
#define TOUCH_SWIPE_MAX_Y (120)

static mtb_ctp_ft5406_config_t s_touch_config;
static lv_indev_t *s_touch_indev;
static lv_point_t s_last_point;
static lv_port_indev_status_t s_touch_status;
static bool s_touch_pressed;
static lv_point_t s_press_start;
static void (*s_swipe_cb)(int32_t dir);

static int32_t clamp_i32(int32_t value, int32_t min_value, int32_t max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static void map_touch_point(int raw_x, int raw_y, lv_point_t *out)
{
    int32_t x = (int32_t)TOUCH_MAX_X - (int32_t)raw_x;
    int32_t y = (int32_t)TOUCH_MAX_Y - (int32_t)raw_y;

    out->x = (lv_coord_t)clamp_i32(x, 0, TOUCH_MAX_X);
    out->y = (lv_coord_t)clamp_i32(y, 0, TOUCH_MAX_Y);
}

static void touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    mtb_ctp_touch_event_t touch_event = MTB_CTP_TOUCH_RESERVED;
    int raw_x = 0;
    int raw_y = 0;
    cy_en_scb_i2c_status_t status;
    bool now_pressed = false;
    lv_point_t point = s_last_point;

    (void)indev;

    data->point = s_last_point;
    data->state = LV_INDEV_STATE_RELEASED;
    s_touch_status.read_count++;

    status = mtb_ctp_ft5406_get_single_touch(&touch_event, &raw_x, &raw_y);
    s_touch_status.last_i2c_status = (uint32_t)status;
    if (CY_SCB_I2C_SUCCESS != status)
    {
        s_touch_status.error_count++;
        return;
    }

    s_touch_status.success_count++;
    if (s_touch_config.touch_buff[1] <= MTB_CTP_FT5406_MAX_TOUCHES)
    {
        s_touch_status.last_touch_count = (uint32_t)s_touch_config.touch_buff[1];
    }
    s_touch_status.last_event = (uint32_t)touch_event;

    if ((MTB_CTP_TOUCH_DOWN == touch_event) ||
        (MTB_CTP_TOUCH_CONTACT == touch_event))
    {
        map_touch_point(raw_x, raw_y, &point);
        s_touch_status.last_raw_x = (int16_t)raw_x;
        s_touch_status.last_raw_y = (int16_t)raw_y;
        s_touch_status.last_x = (int16_t)point.x;
        s_touch_status.last_y = (int16_t)point.y;
        now_pressed = true;
    }

    if (now_pressed)
    {
        if (!s_touch_pressed)
        {
            s_press_start = point;
        }

        s_last_point = point;
        s_touch_pressed = true;
        s_touch_status.press_count++;
        data->point = s_last_point;
        data->state = LV_INDEV_STATE_PRESSED;
    }
    else
    {
        s_touch_status.last_event = (uint32_t)MTB_CTP_TOUCH_RESERVED;
        if (s_touch_pressed)
        {
            int32_t dx = (int32_t)s_last_point.x - (int32_t)s_press_start.x;
            int32_t dy = (int32_t)s_last_point.y - (int32_t)s_press_start.y;

            if (((dx > TOUCH_SWIPE_MIN_X) || (dx < -TOUCH_SWIPE_MIN_X)) &&
                (dy < TOUCH_SWIPE_MAX_Y) && (dy > -TOUCH_SWIPE_MAX_Y))
            {
                int32_t dir = (dx > 0) ? 1 : -1;

                s_touch_status.swipe_count++;
                s_touch_status.last_swipe_dir = dir;
                if (NULL != s_swipe_cb)
                {
                    s_swipe_cb(dir);
                }
            }
        }
        s_touch_pressed = false;
    }
}

cy_rslt_t lv_port_indev_init(CySCB_Type *i2c_base,
                             cy_stc_scb_i2c_context_t *i2c_context)
{
    cy_en_scb_i2c_status_t status;

    s_touch_status.initialized = false;
    s_touch_status.init_result = CY_RSLT_TYPE_ERROR;
    s_touch_status.read_count = 0U;
    s_touch_status.success_count = 0U;
    s_touch_status.error_count = 0U;
    s_touch_status.press_count = 0U;
    s_touch_status.swipe_count = 0U;
    s_touch_status.last_i2c_status = 0U;
    s_touch_status.last_event = (uint32_t)MTB_CTP_TOUCH_RESERVED;
    s_touch_status.last_touch_count = 0U;
    s_touch_status.last_swipe_dir = 0;
    s_touch_status.last_raw_x = 0;
    s_touch_status.last_raw_y = 0;
    s_touch_status.last_x = 0;
    s_touch_status.last_y = 0;
    s_touch_pressed = false;

    if ((NULL == i2c_base) || (NULL == i2c_context))
    {
        return CY_RSLT_TYPE_ERROR;
    }

    s_last_point.x = 0;
    s_last_point.y = 0;
    s_touch_config.i2c_base = i2c_base;
    s_touch_config.i2c_context = i2c_context;

    status = mtb_ctp_ft5406_init(&s_touch_config);
    s_touch_status.last_i2c_status = (uint32_t)status;
    if (CY_SCB_I2C_SUCCESS != status)
    {
        s_touch_status.init_result = (cy_rslt_t)status;
        return (cy_rslt_t)status;
    }

    s_touch_indev = lv_indev_create();
    if (NULL == s_touch_indev)
    {
        s_touch_status.init_result = CY_RSLT_TYPE_ERROR;
        return CY_RSLT_TYPE_ERROR;
    }

    lv_indev_set_type(s_touch_indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(s_touch_indev, touch_read_cb);

    s_touch_status.initialized = true;
    s_touch_status.init_result = CY_RSLT_SUCCESS;
    return CY_RSLT_SUCCESS;
}

void lv_port_indev_get_status(lv_port_indev_status_t *status)
{
    if (NULL != status)
    {
        *status = s_touch_status;
    }
}

void lv_port_indev_set_swipe_cb(void (*cb)(int32_t dir))
{
    s_swipe_cb = cb;
}
