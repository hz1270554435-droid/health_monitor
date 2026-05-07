#include "app_model_smoke.h"

#if (APP_MODEL_SMOKE_TEST_ENABLE)

#include <math.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app_model_shared.h"
#include "audio_model_v2_float.h"
#include "audio_test_vectors.h"

#endif /* APP_MODEL_SMOKE_TEST_ENABLE */

#if (APP_MODEL_SMOKE_TEST_ENABLE)

/* PC expected_output.json 中 14 条 curated 测试向量的参考值。
 * audio_test_vectors.h 来自：
 * D:\cough_model_train\health_monitor_ai\deploy\test_vectors\audio
 *
 * 注意：这里使用 PC 侧已经生成好的 Log-Mel float32 特征，目的是先验证模型部署
 * 和推理 API，而不是验证板端 PDM/PCM 前处理。
 *
 * 当前样本组成：
 * - 5 条 PC 判定正确的 cough；
 * - 5 条 PC 判定正确的 non_cough；
 * - 4 条 PC 侧已知误分类/边界样本。
 * smoke test 只要求板端输出复现 PC/ONNX expected，不在这里重新定义业务阈值。
 */

typedef struct
{
    const char *sample_id;
    float expected_output0;
    float expected_output1;
    float expected_cough_prob;
} app_model_smoke_expected_t;

/* smoke test 使用独立的 result_sequence 区间。
 *
 * result.input_sequence 仍然保持 1、2、3...，方便 CM33 日志按 sample 编号打印；
 * shared->result_sequence 使用高位区间，避免 smoke sample 1/2 和后续正式推理
 * input_seq 1/2 冲突。否则 CM33 监控任务如果漏看某条 smoke，后续同序号的正式
 * MODEL_RESULT 也可能被当成“已打印过的结果”跳过。
 */
static const app_model_smoke_expected_t app_model_smoke_expected[] =
{
    {
        "clip_cough_0025_0003",
        -2.518211841583252f,
        2.848555326461792f,
        0.9953525066375732f
    },
    {
        "clip_cough_0025_0002",
        -1.9881658554077148f,
        2.286250352859497f,
        0.9862709045410156f
    },
    {
        "clip_cough_0025_0008",
        -1.7914918661117554f,
        2.0597808361053467f,
        0.9791896343231201f
    },
    {
        "clip_cough_0025_0012",
        -1.79100501537323f,
        1.9779932498931885f,
        0.9774452447891235f
    },
    {
        "clip_cough_0025_0013",
        -1.7110023498535156f,
        1.8903207778930664f,
        0.9734372496604919f
    },
    {
        "clip_non_cough_0025_0018",
        3.1650824546813965f,
        -2.9453911781311035f,
        0.002214584033936262f
    },
    {
        "clip_non_cough_0025_0021",
        2.749448299407959f,
        -2.542973756790161f,
        0.005004392471164465f
    },
    {
        "clip_non_cough_0025_0030",
        2.6354711055755615f,
        -2.454615592956543f,
        0.006119801662862301f
    },
    {
        "clip_non_cough_0025_0017",
        2.637068510055542f,
        -2.435283899307251f,
        0.00622861972078681f
    },
    {
        "clip_non_cough_0025_0022",
        2.5412745475769043f,
        -2.3253591060638428f,
        0.007640416268259287f
    },
    {
        "clip_cough_0023_0001",
        0.2716721296310425f,
        -0.1315980702638626f,
        0.40052688121795654f
    },
    {
        "clip_cough_0023_0002",
        -0.0011171624064445496f,
        0.19802121818065643f,
        0.5496206879615784f
    },
    {
        "clip_non_cough_0023_v2_05_0001",
        -1.0244779586791992f,
        1.2235107421875f,
        0.9044768810272217f
    },
    {
        "clip_non_cough_0023_v2_05_0002",
        -0.9391075968742371f,
        0.8851758241653442f,
        0.8610793352127075f
    }
};

static float app_model_smoke_softmax_cough_prob(float output0,
                                                float output1);
static float app_model_smoke_absf(float value);
static void app_model_smoke_publish_result(uint32_t sample_index,
                                           const float *output,
                                           float cough_prob,
                                           float expected_cough_prob,
                                           uint32_t elapsed_ms,
                                           uint8_t status);
static uint32_t app_model_smoke_now_ms(void);

#endif /* APP_MODEL_SMOKE_TEST_ENABLE */

bool app_model_smoke_run_once(void)
{
#if (APP_MODEL_SMOKE_TEST_ENABLE)
    bool all_passed = true;
    int init_ret;

    if (AUDIO_TEST_VECTOR_COUNT !=
        (sizeof(app_model_smoke_expected) / sizeof(app_model_smoke_expected[0])))
    {
        return false;
    }

    init_ret = AUDIO_init();
    if (AUDIO_RET_SUCCESS != init_ret)
    {
        float empty_output[AUDIO_DATA_OUT_COUNT] = { 0.0f, 0.0f };

        app_model_smoke_publish_result(0u,
                                       empty_output,
                                       0.0f,
                                       app_model_smoke_expected[0].expected_cough_prob,
                                       0u,
                                       (uint8_t)APP_MODEL_INFERENCE_STATUS_INVALID_INPUT);
        return false;
    }

    for (uint32_t i = 0u; i < AUDIO_TEST_VECTOR_COUNT; i++)
    {
        float output[AUDIO_DATA_OUT_COUNT] = { 0.0f, 0.0f };
        uint32_t start_ms;
        uint32_t elapsed_ms;
        float cough_prob;
        float diff;

        (void)AUDIO_soft_reset();

        start_ms = app_model_smoke_now_ms();
        AUDIO_compute(audio_test_vectors[i], output);
        elapsed_ms = app_model_smoke_now_ms() - start_ms;

        cough_prob = app_model_smoke_softmax_cough_prob(output[0], output[1]);
        diff = app_model_smoke_absf(cough_prob -
                                    app_model_smoke_expected[i].expected_cough_prob);

        if (0.01f < diff)
        {
            all_passed = false;
        }

        app_model_smoke_publish_result(i,
                                       output,
                                       cough_prob,
                                       app_model_smoke_expected[i].expected_cough_prob,
                                       elapsed_ms,
                                       (uint8_t)APP_MODEL_INFERENCE_STATUS_OK);

        /* 给 CM33 结果观察任务留出打印时间；结果槽采用“最新结果覆盖旧结果”语义。 */
        vTaskDelay(pdMS_TO_TICKS(500u));
    }

    return all_passed;
#else
    return true;
#endif
}

#if (APP_MODEL_SMOKE_TEST_ENABLE)

static float app_model_smoke_softmax_cough_prob(float output0,
                                                float output1)
{
    float max_logit = (output0 > output1) ? output0 : output1;
    float exp0 = expf(output0 - max_logit);
    float exp1 = expf(output1 - max_logit);
    float denom = exp0 + exp1;

    return (0.0f < denom) ? (exp1 / denom) : 0.0f;
}

static float app_model_smoke_absf(float value)
{
    return (0.0f <= value) ? value : -value;
}

static void app_model_smoke_publish_result(uint32_t sample_index,
                                           const float *output,
                                           float cough_prob,
                                           float expected_cough_prob,
                                           uint32_t elapsed_ms,
                                           uint8_t status)
{
    volatile app_model_shared_region_t *shared = APP_MODEL_SHARED_REGION;
    app_model_inference_result_t result;

    if ((APP_MODEL_SHARED_MAGIC != shared->magic) ||
        (APP_MODEL_SHARED_VERSION != shared->version))
    {
        memset((void *)shared, 0, sizeof(*shared));
        shared->magic = APP_MODEL_SHARED_MAGIC;
        shared->version = APP_MODEL_SHARED_VERSION;
        shared->input_state = APP_MODEL_SHARED_INPUT_EMPTY;
    }

    memset(&result, 0, sizeof(result));
    result.input_sequence = sample_index + 1u;
    result.timestamp_ms = app_model_smoke_now_ms();
    result.inference_time_ms = elapsed_ms;
    result.class_count = 2u;
    result.status = status;
    result.scores[0] = output[0];
    result.scores[1] = output[1];
    result.scores[2] = cough_prob;
    result.scores[3] = app_model_smoke_expected[sample_index].expected_output0;
    result.scores[4] = app_model_smoke_expected[sample_index].expected_output1;
    result.scores[5] = expected_cough_prob;
    result.scores[6] = app_model_smoke_absf(cough_prob - expected_cough_prob);

    shared->result_state = APP_MODEL_SHARED_RESULT_WRITING;
    shared->result = result;
    shared->result_sequence = APP_MODEL_RESULT_SEQUENCE_SMOKE_BASE +
                              result.input_sequence;
    __DMB();
    shared->result_state = APP_MODEL_SHARED_RESULT_READY;
    APP_MODEL_SHARED_CLEAN_CACHE((void *)shared, sizeof(*shared));
}

static uint32_t app_model_smoke_now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

#endif /* APP_MODEL_SMOKE_TEST_ENABLE */
