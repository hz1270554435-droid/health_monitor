/*******************************************************************************
* File Name : app_cm55_fe_bench_result.h
*
* Description : Score-slot mapping for the default-off CM55 frontend benchmark.
*******************************************************************************/

#ifndef __APP_CM55_FE_BENCH_RESULT_H__
#define __APP_CM55_FE_BENCH_RESULT_H__

#include "app_model_shared.h"

#define APP_CM55_FE_BENCH_SCORE_MARKER_VALUE       (55055.0f)
#define APP_CM55_FE_BENCH_SCORE_MARKER_INDEX       (5u)
#define APP_CM55_FE_BENCH_SCORE_RUN_INDEX          (6u)
#define APP_CM55_FE_BENCH_SCORE_OK_INDEX           (7u)
#define APP_CM55_FE_BENCH_SCORE_TOTAL_MS_INDEX     (8u)
#define APP_CM55_FE_BENCH_SCORE_CONDITION_MS_INDEX (9u)
#define APP_CM55_FE_BENCH_SCORE_SPECTRUM_MS_INDEX  (10u)
#define APP_CM55_FE_BENCH_SCORE_MELBANK_MS_INDEX   (11u)
#define APP_CM55_FE_BENCH_SCORE_DB_MS_INDEX        (12u)
#define APP_CM55_FE_BENCH_SCORE_TIME_BINS_INDEX    (13u)
#define APP_CM55_FE_BENCH_SCORE_MODE_INDEX         (14u)
#define APP_CM55_FE_BENCH_SCORE_HASH_INDEX         (15u)

#define APP_CM55_FE_BENCH_MODE_PROXY_NO_RFFT       (0u)
#define APP_CM55_FE_BENCH_MODE_RFFT_FAST_F32       (1u)

#if (APP_MODEL_INFERENCE_MAX_SCORES <= APP_CM55_FE_BENCH_SCORE_HASH_INDEX)
#error "APP_CM55_FE_BENCH result mapping requires at least 16 score slots"
#endif

#endif /* __APP_CM55_FE_BENCH_RESULT_H__ */
