/**
 * @file lv_port_indev.h
 * LVGL touch input port for the Waveshare 4.3" FT5406 controller.
 */

#ifndef LV_PORT_INDEV_H
#define LV_PORT_INDEV_H

#include "cy_result.h"
#include "cy_scb_i2c.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    bool initialized;
    cy_rslt_t init_result;
    uint32_t read_count;
    uint32_t success_count;
    uint32_t error_count;
    uint32_t press_count;
    uint32_t swipe_count;
    uint32_t last_i2c_status;
    uint32_t last_event;
    uint32_t last_touch_count;
    int32_t last_swipe_dir;
    int16_t last_raw_x;
    int16_t last_raw_y;
    int16_t last_x;
    int16_t last_y;
} lv_port_indev_status_t;

/**
 * @brief Initialize the LVGL pointer input device.
 *
 * Uses the already-initialized display I2C controller. A non-success return
 * means touch is unavailable; the display/UI can continue without input.
 */
cy_rslt_t lv_port_indev_init(CySCB_Type *i2c_base,
                             cy_stc_scb_i2c_context_t *i2c_context);

void lv_port_indev_get_status(lv_port_indev_status_t *status);

void lv_port_indev_set_swipe_cb(void (*cb)(int32_t dir));

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* LV_PORT_INDEV_H */
