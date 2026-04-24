#ifndef __APP_CSV_EXPORT_H__
#define __APP_CSV_EXPORT_H__

#if defined(__cplusplus)
extern "C" {
#endif

#include "cy_pdl.h"
#include "mtb_hal.h"
#include "cybsp.h"
#include "FreeRTOS.h"
#include "task.h"

/* Enable this task to make the board stream one CSV over the debug UART.
 * The first CSV column is "device", so PC tools can split MIC and radar rows.
 */
#ifndef APP_CSV_EXPORT_ENABLE
#define APP_CSV_EXPORT_ENABLE                 (1u)
#endif

/* Raw MIC CSV is bandwidth-heavy: at 16 kHz stereo it prints 16000 rows/s.
 * Use a fast debug UART, or set this to 0 for one summary row per MIC block.
 */
#ifndef APP_CSV_EXPORT_MIC_RAW_ENABLE
#define APP_CSV_EXPORT_MIC_RAW_ENABLE         (1u)
#endif

/* Send MIC samples as compact binary PCM blocks instead of per-sample CSV rows.
 * The PC capture tool recognizes these frames by the "PCMB" magic and converts
 * them back into WAV/CSV/JSON training sessions.
 */
#ifndef APP_CSV_EXPORT_MIC_BINARY_ENABLE
#define APP_CSV_EXPORT_MIC_BINARY_ENABLE      (1u)
#endif

/* Include the complete radar frame as hex without spaces. */
#ifndef APP_CSV_EXPORT_RADAR_FRAME_HEX_ENABLE
#define APP_CSV_EXPORT_RADAR_FRAME_HEX_ENABLE (1u)
#endif

/* Keep radar collection running but suppress radar rows on the debug UART.
 * This lets us test high-speed MIC export without interleaved radar text.
 */
#ifndef APP_CSV_EXPORT_RADAR_PRINT_ENABLE
#define APP_CSV_EXPORT_RADAR_PRINT_ENABLE     (1u)
#endif

#define APP_CSV_EXPORT_TASK_STACK_SIZE        (2048u)
#define APP_CSV_EXPORT_TASK_PRIORITY          (configMAX_PRIORITIES - 4u)
#define APP_CSV_EXPORT_IDLE_DELAY_MS          (1u)
#define APP_CSV_EXPORT_RADAR_RX_WAIT_MS       (1u)

cy_rslt_t app_csv_export_task_init(void);
void app_csv_export_task(void *pvParameters);

#if defined(__cplusplus)
}
#endif

#endif /* __APP_CSV_EXPORT_H__ */
