/*******************************************************************************
* File Name : app_cm55_display_rotation.c
*
* Description : CM55 display framebuffer rotation helpers.
*******************************************************************************/

#include "app_cm55_display_rotation.h"

#include <stddef.h>
#include <string.h>

static inline uint32_t reverse_rgb565_pair_order(uint32_t pair)
{
    return ((pair & 0x0000FFFFu) << 16) | ((pair & 0xFFFF0000u) >> 16);
}

static void rotate_180_rgb565_u16(uint16_t *fb, uint32_t pixel_count)
{
    uint32_t i = 0U;
    uint32_t j = pixel_count - 1U;

    while (i < j)
    {
        uint16_t tmp = fb[i];
        fb[i] = fb[j];
        fb[j] = tmp;
        ++i;
        --j;
    }
}

static void rotate_180_rgb565_u32_pairs(uint16_t *fb, uint32_t pixel_count)
{
    uint32_t left = 0U;
    uint32_t right = (pixel_count / 2U) - 1U;

    while (left < right)
    {
        uint32_t left_pair;
        uint32_t right_pair;
        uint32_t new_left_pair;
        uint32_t new_right_pair;

        (void)memcpy(&left_pair, &fb[left * 2U], sizeof(left_pair));
        (void)memcpy(&right_pair, &fb[right * 2U], sizeof(right_pair));

        /* A 180° rotation reverses pixel order, not only 32-bit word order. */
        new_left_pair = reverse_rgb565_pair_order(right_pair);
        new_right_pair = reverse_rgb565_pair_order(left_pair);

        (void)memcpy(&fb[left * 2U], &new_left_pair, sizeof(new_left_pair));
        (void)memcpy(&fb[right * 2U], &new_right_pair, sizeof(new_right_pair));

        ++left;
        --right;
    }

    if (left == right)
    {
        uint32_t middle_pair;

        (void)memcpy(&middle_pair, &fb[left * 2U], sizeof(middle_pair));
        middle_pair = reverse_rgb565_pair_order(middle_pair);
        (void)memcpy(&fb[left * 2U], &middle_pair, sizeof(middle_pair));
    }
}

void app_cm55_display_rotate_180_rgb565_inplace(uint16_t *fb,
                                                uint32_t pixel_count)
{
    if ((NULL == fb) || (pixel_count < 2U))
    {
        return;
    }

    if ((((uintptr_t)fb & 0x3U) == 0U) && ((pixel_count & 0x1U) == 0U))
    {
        rotate_180_rgb565_u32_pairs(fb, pixel_count);
        return;
    }

    rotate_180_rgb565_u16(fb, pixel_count);
}
