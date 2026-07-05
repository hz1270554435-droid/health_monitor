/*******************************************************************************
* File Name : app_cm55_fe_bench.h
*
* Description : Default-off CM55 audio frontend benchmark probe.
*******************************************************************************/

#ifndef __APP_CM55_FE_BENCH_H__
#define __APP_CM55_FE_BENCH_H__

#include <stdint.h>

#include "app_model_shared.h"

#if defined(__cplusplus)
extern "C" {
#endif

void app_cm55_fe_bench_maybe_run(uint32_t input_sequence,
                                 app_model_inference_result_t *result);

#if defined(__cplusplus)
}
#endif

#endif /* __APP_CM55_FE_BENCH_H__ */
