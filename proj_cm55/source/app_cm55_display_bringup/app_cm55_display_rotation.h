/*******************************************************************************
* File Name : app_cm55_display_rotation.h
*
* Description : CM55 display framebuffer rotation helpers.
*******************************************************************************/

#ifndef __APP_CM55_DISPLAY_ROTATION_H__
#define __APP_CM55_DISPLAY_ROTATION_H__

#include <stdint.h>

#if defined(__cplusplus)
extern "C" {
#endif

/**
 * Rotate an RGB565 framebuffer by 180 degrees in place.
 *
 * @param fb           RGB565 framebuffer, one uint16_t per pixel.
 * @param pixel_count  Number of pixels in the framebuffer, not bytes.
 */
void app_cm55_display_rotate_180_rgb565_inplace(uint16_t *fb,
                                                uint32_t pixel_count);

#if defined(__cplusplus)
}
#endif

#endif /* __APP_CM55_DISPLAY_ROTATION_H__ */
