/*******************************************************************************
* File Name : app_cm55_display_buffers.c
*
* Description : Framebuffer storage for the CM55 display proof path.
*******************************************************************************/

#include "cy_utils.h"

#include <stdint.h>

#define CM55_DISP_HOR_RES               (832U)
#define CM55_DISP_VER_RES               (480U)
#define CM55_DISP_APP_BUFFER_COUNT      (2U)
#define CM55_DISP_GPU_CMD_BUFFER_SIZE   ((64U) * (1024U))
#define CM55_DISP_GPU_TESS_BUFFER_SIZE  (CM55_DISP_VER_RES * 128U)
#define CM55_DISP_VGLITE_HEAP_SIZE      \
    ((CM55_DISP_GPU_CMD_BUFFER_SIZE * CM55_DISP_APP_BUFFER_COUNT) + \
     (CM55_DISP_GPU_TESS_BUFFER_SIZE * CM55_DISP_APP_BUFFER_COUNT))

CY_SECTION(".cy_gpu_buf") uint16_t frame_buffer1[
    CM55_DISP_HOR_RES * CM55_DISP_VER_RES];
CY_SECTION(".cy_gpu_buf") uint16_t frame_buffer2[
    CM55_DISP_HOR_RES * CM55_DISP_VER_RES];
CY_SECTION(".cy_gpu_buf") uint8_t contiguous_mem[
    CM55_DISP_VGLITE_HEAP_SIZE];
