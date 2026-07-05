#include "app_cm55_fe_bench.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app_cm55_fe_bench_result.h"
#include "app_model_shared.h"

#ifndef APP_CM55_FE_BENCH_ENABLE
#define APP_CM55_FE_BENCH_ENABLE              (0u)
#endif

#ifndef APP_CM55_FE_BENCH_PERIOD_WINDOWS
#define APP_CM55_FE_BENCH_PERIOD_WINDOWS      (20u)
#endif

#ifndef APP_CM55_FE_BENCH_MAX_RUNS
#define APP_CM55_FE_BENCH_MAX_RUNS            (8u)
#endif

#ifndef APP_CM55_FE_BENCH_TIME_BIN_LIMIT
#define APP_CM55_FE_BENCH_TIME_BIN_LIMIT      (0u)
#endif

#ifndef APP_CM55_FE_BENCH_RFFT_ENABLE
#define APP_CM55_FE_BENCH_RFFT_ENABLE         (0u)
#endif

#ifndef APP_CM55_FE_BENCH_STAGE_LOG_ENABLE
#define APP_CM55_FE_BENCH_STAGE_LOG_ENABLE    (0u)
#endif

#ifndef APP_CM55_FE_BENCH_UART_LOG_ENABLE
#define APP_CM55_FE_BENCH_UART_LOG_ENABLE     (0u)
#endif

#if (APP_CM55_FE_BENCH_ENABLE)

#if (APP_CM55_FE_BENCH_RFFT_ENABLE)
#include "arm_math.h"
#endif

#define CM55_FE_BENCH_PI                      (3.14159265358979323846f)
#define CM55_FE_BENCH_SAMPLE_RATE_HZ          (16000u)
#define CM55_FE_BENCH_WINDOW_SAMPLES          (16000u)
#define CM55_FE_BENCH_FRAME_LEN_SAMPLES       (1024u)
#define CM55_FE_BENCH_FRAME_HOP_SAMPLES       (160u)
#define CM55_FE_BENCH_FFT_SIZE                (1024u)
#define CM55_FE_BENCH_SPECTRUM_BINS           ((CM55_FE_BENCH_FFT_SIZE / 2u) + 1u)
#define CM55_FE_BENCH_MEL_BINS                (APP_MODEL_AUDIO_MODEL_MEL_BINS)
#define CM55_FE_BENCH_TIME_BINS               (APP_MODEL_AUDIO_MODEL_TIME_BINS)
#define CM55_FE_BENCH_MEL_LOW_HZ              (50.0f)
#define CM55_FE_BENCH_MEL_HIGH_HZ             (7600.0f)
#define CM55_FE_BENCH_ENERGY_GATE             (0.000001f)
#define CM55_FE_BENCH_LOG_EPSILON             (0.000001f)

typedef struct
{
    uint16_t mel_edges[CM55_FE_BENCH_MEL_BINS + 2u];
    float frame_window[CM55_FE_BENCH_FRAME_LEN_SAMPLES];
} app_cm55_fe_bench_plan_t;

static app_cm55_fe_bench_plan_t cm55_fe_bench_plan;
#if (APP_CM55_FE_BENCH_RFFT_ENABLE)
static arm_rfft_fast_instance_f32 cm55_fe_bench_rfft;
static float cm55_fe_bench_rfft_in[CM55_FE_BENCH_FFT_SIZE];
static float cm55_fe_bench_rfft_out[CM55_FE_BENCH_FFT_SIZE];
#endif
static float cm55_fe_bench_spectrum[CM55_FE_BENCH_SPECTRUM_BINS];
static float cm55_fe_bench_feature[APP_MODEL_AUDIO_FEATURE_MAX_ELEMENTS];
static bool cm55_fe_bench_ready;
static bool cm55_fe_bench_init_attempted;
static uint32_t cm55_fe_bench_seen_windows;
static uint32_t cm55_fe_bench_runs;
static volatile float cm55_fe_bench_sink;

static bool app_cm55_fe_bench_init_once(uint32_t input_sequence);
static void app_cm55_fe_bench_run_once(uint32_t input_sequence,
                                       app_model_inference_result_t *result);
static uint32_t app_cm55_fe_bench_now_ms(void);
static float app_cm55_fe_bench_hz_to_mel(float hz);
static float app_cm55_fe_bench_mel_to_hz(float mel);
static float app_cm55_fe_bench_mel_energy(uint16_t mel_index);
#if !(APP_CM55_FE_BENCH_RFFT_ENABLE)
static void app_cm55_fe_bench_fill_spectrum_proxy(uint16_t time_index,
                                                  float energy);
#endif
static uint32_t app_cm55_fe_bench_hash_feature(void);
static float app_cm55_fe_bench_synthetic_sample(uint32_t index);
static uint16_t app_cm55_fe_bench_active_time_bins(void);
#if (APP_CM55_FE_BENCH_UART_LOG_ENABLE)
static const char *app_cm55_fe_bench_mode(void);
#endif
static void app_cm55_fe_bench_log_stage(const char *stage,
                                        uint32_t input_sequence);
static void app_cm55_fe_bench_store_result(
    app_model_inference_result_t *result,
    uint32_t ok,
    uint32_t total_ms,
    uint32_t copy_condition_ms,
    uint32_t spectrum_ms,
    uint32_t melbank_ms,
    uint32_t db_ms,
    uint32_t hash);

void app_cm55_fe_bench_maybe_run(uint32_t input_sequence,
                                 app_model_inference_result_t *result)
{
    if (0u == APP_CM55_FE_BENCH_MAX_RUNS)
    {
        return;
    }

    if (cm55_fe_bench_runs >= APP_CM55_FE_BENCH_MAX_RUNS)
    {
        return;
    }

    cm55_fe_bench_seen_windows++;
    if (APP_CM55_FE_BENCH_PERIOD_WINDOWS > 1u)
    {
        if (0u != (cm55_fe_bench_seen_windows %
                   APP_CM55_FE_BENCH_PERIOD_WINDOWS))
        {
            return;
        }
    }

    app_cm55_fe_bench_log_stage("trigger", input_sequence);
    if (!app_cm55_fe_bench_init_once(input_sequence))
    {
        if (0u == cm55_fe_bench_runs)
        {
#if (APP_CM55_FE_BENCH_UART_LOG_ENABLE)
            printf("[CM55_FE_BENCH] t_ms=%lu run=0 input_seq=%lu ok=0 "
                   "reason=init_failed\r\n",
                   (unsigned long)app_cm55_fe_bench_now_ms(),
                   (unsigned long)input_sequence);
            fflush(stdout);
#endif
            cm55_fe_bench_runs++;
            app_cm55_fe_bench_store_result(result, 0u, 0u, 0u, 0u, 0u, 0u, 0u);
        }
        return;
    }

    app_cm55_fe_bench_run_once(input_sequence, result);
}

static bool app_cm55_fe_bench_init_once(uint32_t input_sequence)
{
    float low_mel;
    float high_mel;
    uint16_t spectrum_last_bin = (uint16_t)(CM55_FE_BENCH_SPECTRUM_BINS - 1u);

    if (cm55_fe_bench_ready)
    {
        return true;
    }
    if (cm55_fe_bench_init_attempted)
    {
        return false;
    }
    cm55_fe_bench_init_attempted = true;
    app_cm55_fe_bench_log_stage("init_begin", input_sequence);

#if (APP_CM55_FE_BENCH_RFFT_ENABLE)
    app_cm55_fe_bench_log_stage("rfft_init_begin", input_sequence);
    if (ARM_MATH_SUCCESS !=
        arm_rfft_fast_init_f32(&cm55_fe_bench_rfft, CM55_FE_BENCH_FFT_SIZE))
    {
        app_cm55_fe_bench_log_stage("rfft_init_failed", input_sequence);
        return false;
    }
    app_cm55_fe_bench_log_stage("rfft_init_end", input_sequence);
#endif

    for (uint16_t i = 0; i < CM55_FE_BENCH_FRAME_LEN_SAMPLES; i++)
    {
        float phase = (2.0f * CM55_FE_BENCH_PI * (float)i) /
                      (float)CM55_FE_BENCH_FRAME_LEN_SAMPLES;
        cm55_fe_bench_plan.frame_window[i] = 0.5f - (0.5f * cosf(phase));
    }

    low_mel = app_cm55_fe_bench_hz_to_mel(CM55_FE_BENCH_MEL_LOW_HZ);
    high_mel = app_cm55_fe_bench_hz_to_mel(CM55_FE_BENCH_MEL_HIGH_HZ);
    for (uint16_t i = 0; i < (uint16_t)(CM55_FE_BENCH_MEL_BINS + 2u); i++)
    {
        float mel = low_mel + ((high_mel - low_mel) *
                               ((float)i / (float)(CM55_FE_BENCH_MEL_BINS + 1u)));
        float hz = app_cm55_fe_bench_mel_to_hz(mel);
        uint32_t bin = (uint32_t)(((float)(CM55_FE_BENCH_FFT_SIZE + 1u) * hz) /
                                  (float)CM55_FE_BENCH_SAMPLE_RATE_HZ);
        if (spectrum_last_bin < bin)
        {
            bin = spectrum_last_bin;
        }
        cm55_fe_bench_plan.mel_edges[i] = (uint16_t)bin;
    }

    cm55_fe_bench_ready = true;
    app_cm55_fe_bench_log_stage("init_end", input_sequence);
    return true;
}

static void app_cm55_fe_bench_run_once(uint32_t input_sequence,
                                       app_model_inference_result_t *result)
{
    uint32_t start_ms;
    uint32_t stage_ms;
    uint32_t copy_condition_ms;
    uint32_t spectrum_ms = 0u;
    uint32_t melbank_ms = 0u;
    uint32_t db_ms;
    uint32_t total_ms;
    uint32_t hash;
    double sum = 0.0;
    double square_sum = 0.0;
    float mean;
    float energy;
    float max_mel_energy = 0.0f;
    uint16_t active_time_bins = app_cm55_fe_bench_active_time_bins();

    start_ms = app_cm55_fe_bench_now_ms();
    app_cm55_fe_bench_log_stage("run_begin", input_sequence);

    for (uint16_t i = 0; i < CM55_FE_BENCH_WINDOW_SAMPLES; i++)
    {
        sum += app_cm55_fe_bench_synthetic_sample(i);
    }
    mean = (float)(sum / (double)CM55_FE_BENCH_WINDOW_SAMPLES);

    for (uint16_t i = 0; i < CM55_FE_BENCH_WINDOW_SAMPLES; i++)
    {
        float sample = app_cm55_fe_bench_synthetic_sample(i) - mean;
        square_sum += ((double)sample * (double)sample);
    }
    energy = (float)(square_sum / (double)CM55_FE_BENCH_WINDOW_SAMPLES);
    copy_condition_ms = app_cm55_fe_bench_now_ms() - start_ms;
    app_cm55_fe_bench_log_stage("condition_end", input_sequence);

    if (energy < CM55_FE_BENCH_ENERGY_GATE)
    {
        cm55_fe_bench_runs++;
#if (APP_CM55_FE_BENCH_UART_LOG_ENABLE)
        printf("[CM55_FE_BENCH] t_ms=%lu run=%lu input_seq=%lu ok=0 "
               "reason=energy_gate total_ms=%lu energy_x1e9=%lu\r\n",
               (unsigned long)app_cm55_fe_bench_now_ms(),
               (unsigned long)cm55_fe_bench_runs,
               (unsigned long)input_sequence,
               (unsigned long)copy_condition_ms,
               (unsigned long)(energy * 1000000000.0f));
        fflush(stdout);
#endif
        app_cm55_fe_bench_store_result(result, 0u, copy_condition_ms,
                                       copy_condition_ms, 0u, 0u, 0u, 0u);
        return;
    }

    app_cm55_fe_bench_log_stage("frames_begin", input_sequence);
    for (uint16_t t = 0; t < active_time_bins; t++)
    {
        uint16_t out_t = t;
        uint32_t frame_start =
            (uint32_t)t * (uint32_t)CM55_FE_BENCH_FRAME_HOP_SAMPLES;
#if !(APP_CM55_FE_BENCH_RFFT_ENABLE)
        float frame_accum = 0.0f;
#endif

        for (uint16_t n = 0; n < CM55_FE_BENCH_FRAME_LEN_SAMPLES; n++)
        {
            float frame_sample =
                (app_cm55_fe_bench_synthetic_sample(frame_start + n) - mean) *
                cm55_fe_bench_plan.frame_window[n];
#if (APP_CM55_FE_BENCH_RFFT_ENABLE)
            cm55_fe_bench_rfft_in[n] = frame_sample;
#else
            frame_accum += frame_sample * frame_sample;
#endif
        }
        if (0u == t)
        {
            app_cm55_fe_bench_log_stage("frame0_fill_end", input_sequence);
        }

        stage_ms = app_cm55_fe_bench_now_ms();
#if (APP_CM55_FE_BENCH_RFFT_ENABLE)
        if (0u == t)
        {
            app_cm55_fe_bench_log_stage("rfft_begin", input_sequence);
        }
        arm_rfft_fast_f32(&cm55_fe_bench_rfft,
                          cm55_fe_bench_rfft_in,
                          cm55_fe_bench_rfft_out,
                          0u);
        if (0u == t)
        {
            app_cm55_fe_bench_log_stage("rfft_end", input_sequence);
        }

        cm55_fe_bench_spectrum[0] =
            cm55_fe_bench_rfft_out[0] * cm55_fe_bench_rfft_out[0];
        for (uint16_t k = 1u; k < (CM55_FE_BENCH_FFT_SIZE / 2u); k++)
        {
            float real = cm55_fe_bench_rfft_out[2u * k];
            float imag = cm55_fe_bench_rfft_out[(2u * k) + 1u];
            cm55_fe_bench_spectrum[k] = (real * real) + (imag * imag);
        }
        cm55_fe_bench_spectrum[CM55_FE_BENCH_FFT_SIZE / 2u] =
            cm55_fe_bench_rfft_out[1] * cm55_fe_bench_rfft_out[1];
#else
        if (0u == t)
        {
            app_cm55_fe_bench_log_stage("spectrum_proxy_begin", input_sequence);
        }
        app_cm55_fe_bench_fill_spectrum_proxy(t, energy + (frame_accum * 0.000001f));
        if (0u == t)
        {
            app_cm55_fe_bench_log_stage("spectrum_proxy_end", input_sequence);
        }
#endif
        spectrum_ms += app_cm55_fe_bench_now_ms() - stage_ms;

        stage_ms = app_cm55_fe_bench_now_ms();
        for (uint16_t mel = 0; mel < CM55_FE_BENCH_MEL_BINS; mel++)
        {
            float mel_energy = app_cm55_fe_bench_mel_energy(mel);
            uint16_t out_index =
                (uint16_t)((mel * CM55_FE_BENCH_TIME_BINS) + out_t);
            cm55_fe_bench_feature[out_index] = mel_energy;
            if (max_mel_energy < mel_energy)
            {
                max_mel_energy = mel_energy;
            }
        }
        melbank_ms += app_cm55_fe_bench_now_ms() - stage_ms;
        if (0u == t)
        {
            app_cm55_fe_bench_log_stage("frame0_melbank_end", input_sequence);
        }
    }
    app_cm55_fe_bench_log_stage("frames_end", input_sequence);

    stage_ms = app_cm55_fe_bench_now_ms();
    app_cm55_fe_bench_log_stage("db_begin", input_sequence);
    {
        const float amin = 1.0e-10f;
        const float top_db = 80.0f;
        uint32_t element_count =
            (uint32_t)CM55_FE_BENCH_MEL_BINS *
            (uint32_t)active_time_bins;
        float ref_power = (max_mel_energy > amin) ? max_mel_energy : amin;
        float ref_db = 10.0f * log10f(ref_power);

        for (uint32_t i = 0; i < element_count; i++)
        {
            float power = (cm55_fe_bench_feature[i] > amin) ?
                          cm55_fe_bench_feature[i] : amin;
            float db = (10.0f * log10f(power)) - ref_db;
            if (-top_db > db)
            {
                db = -top_db;
            }
            cm55_fe_bench_feature[i] = db;
        }
    }
    db_ms = app_cm55_fe_bench_now_ms() - stage_ms;
    app_cm55_fe_bench_log_stage("db_end", input_sequence);
    total_ms = app_cm55_fe_bench_now_ms() - start_ms;
    hash = app_cm55_fe_bench_hash_feature();

    cm55_fe_bench_sink = cm55_fe_bench_feature[hash %
        ((uint32_t)CM55_FE_BENCH_MEL_BINS * (uint32_t)active_time_bins)];

    cm55_fe_bench_runs++;
    app_cm55_fe_bench_store_result(result, 1u, total_ms, copy_condition_ms,
                                   spectrum_ms, melbank_ms, db_ms, hash);
#if (APP_CM55_FE_BENCH_UART_LOG_ENABLE)
    printf("[CM55_FE_BENCH] t_ms=%lu run=%lu input_seq=%lu ok=1 "
           "mode=%s source=synthetic "
           "total_ms=%lu copy_condition_ms=%lu spectrum_ms=%lu "
           "melbank_ms=%lu db_ms=%lu infer_gate_ms=180 "
           "mel_bins=%lu time_bins=%lu full_time_bins=%lu fft=%lu "
           "energy_x1e9=%lu hash=0x%08lx\r\n",
           (unsigned long)app_cm55_fe_bench_now_ms(),
           (unsigned long)cm55_fe_bench_runs,
           (unsigned long)input_sequence,
           app_cm55_fe_bench_mode(),
           (unsigned long)total_ms,
           (unsigned long)copy_condition_ms,
           (unsigned long)spectrum_ms,
           (unsigned long)melbank_ms,
           (unsigned long)db_ms,
           (unsigned long)CM55_FE_BENCH_MEL_BINS,
           (unsigned long)active_time_bins,
           (unsigned long)CM55_FE_BENCH_TIME_BINS,
           (unsigned long)CM55_FE_BENCH_FFT_SIZE,
           (unsigned long)(energy * 1000000000.0f),
           (unsigned long)hash);
    fflush(stdout);
#endif
}

static uint32_t app_cm55_fe_bench_now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static float app_cm55_fe_bench_hz_to_mel(float hz)
{
    return 2595.0f * log10f(1.0f + (hz / 700.0f));
}

static float app_cm55_fe_bench_mel_to_hz(float mel)
{
    return 700.0f * (powf(10.0f, mel / 2595.0f) - 1.0f);
}

static float app_cm55_fe_bench_mel_energy(uint16_t mel_index)
{
    uint16_t start = cm55_fe_bench_plan.mel_edges[mel_index];
    uint16_t center = cm55_fe_bench_plan.mel_edges[mel_index + 1u];
    uint16_t end = cm55_fe_bench_plan.mel_edges[mel_index + 2u];
    float energy = 0.0f;

    if ((start >= center) || (center >= end))
    {
        return 0.0f;
    }

    for (uint16_t k = start; k < center; k++)
    {
        float weight = (float)(k - start) / (float)(center - start);
        energy += cm55_fe_bench_spectrum[k] * weight;
    }

    for (uint16_t k = center; k <= end; k++)
    {
        float weight = (float)(end - k) / (float)(end - center);
        energy += cm55_fe_bench_spectrum[k] * weight;
    }

    return energy;
}

#if !(APP_CM55_FE_BENCH_RFFT_ENABLE)
static void app_cm55_fe_bench_fill_spectrum_proxy(uint16_t time_index,
                                                  float energy)
{
    float base = (energy > CM55_FE_BENCH_LOG_EPSILON) ?
                 energy : CM55_FE_BENCH_LOG_EPSILON;

    for (uint16_t k = 0u; k < CM55_FE_BENCH_SPECTRUM_BINS; k++)
    {
        uint32_t mix =
            (((uint32_t)k + 1u) * 37u) ^
            (((uint32_t)time_index + 3u) * 131u);
        float shaped = 0.25f + (float)(mix & 0x7Fu) / 127.0f;
        cm55_fe_bench_spectrum[k] = base * shaped;
    }
}
#endif

static uint32_t app_cm55_fe_bench_hash_feature(void)
{
    const uint32_t fnv_prime = 16777619u;
    uint32_t hash = 2166136261u;
    uint32_t element_count =
        (uint32_t)CM55_FE_BENCH_MEL_BINS *
        (uint32_t)app_cm55_fe_bench_active_time_bins();

    for (uint32_t i = 0; i < element_count; i += 31u)
    {
        uint32_t word;
        memcpy(&word, &cm55_fe_bench_feature[i], sizeof(word));
        hash ^= word;
        hash *= fnv_prime;
    }

    return hash;
}

static float app_cm55_fe_bench_synthetic_sample(uint32_t index)
{
    int32_t value = ((int32_t)((index * 73u) & 0x7FFFu) - 16384);
    value += ((int32_t)((index * 17u) & 0x0FFFu) - 2048) / 4;
    if (32767 < value)
    {
        value = 32767;
    }
    else if (-32768 > value)
    {
        value = -32768;
    }
    return (float)value / 32768.0f;
}

static uint16_t app_cm55_fe_bench_active_time_bins(void)
{
    if ((0u == APP_CM55_FE_BENCH_TIME_BIN_LIMIT) ||
        (APP_CM55_FE_BENCH_TIME_BIN_LIMIT > CM55_FE_BENCH_TIME_BINS))
    {
        return (uint16_t)CM55_FE_BENCH_TIME_BINS;
    }
    return (uint16_t)APP_CM55_FE_BENCH_TIME_BIN_LIMIT;
}

#if (APP_CM55_FE_BENCH_UART_LOG_ENABLE)
static const char *app_cm55_fe_bench_mode(void)
{
#if (APP_CM55_FE_BENCH_RFFT_ENABLE)
    return "rfft_fast_f32";
#else
    return "synthetic_spectrum_no_rfft";
#endif
}
#endif

static void app_cm55_fe_bench_log_stage(const char *stage,
                                        uint32_t input_sequence)
{
#if ((APP_CM55_FE_BENCH_STAGE_LOG_ENABLE) && \
     (APP_CM55_FE_BENCH_UART_LOG_ENABLE))
    printf("[CM55_FE_BENCH] t_ms=%lu run=%lu input_seq=%lu ok=2 "
           "mode=%s stage=%s\r\n",
           (unsigned long)app_cm55_fe_bench_now_ms(),
           (unsigned long)(cm55_fe_bench_runs + 1u),
           (unsigned long)input_sequence,
           app_cm55_fe_bench_mode(),
           stage);
    fflush(stdout);
#else
    (void)stage;
    (void)input_sequence;
#endif
}

static void app_cm55_fe_bench_store_result(
    app_model_inference_result_t *result,
    uint32_t ok,
    uint32_t total_ms,
    uint32_t copy_condition_ms,
    uint32_t spectrum_ms,
    uint32_t melbank_ms,
    uint32_t db_ms,
    uint32_t hash)
{
    if (NULL == result)
    {
        return;
    }

    result->scores[APP_CM55_FE_BENCH_SCORE_MARKER_INDEX] =
        APP_CM55_FE_BENCH_SCORE_MARKER_VALUE;
    result->scores[APP_CM55_FE_BENCH_SCORE_RUN_INDEX] =
        (float)cm55_fe_bench_runs;
    result->scores[APP_CM55_FE_BENCH_SCORE_OK_INDEX] = (float)ok;
    result->scores[APP_CM55_FE_BENCH_SCORE_TOTAL_MS_INDEX] = (float)total_ms;
    result->scores[APP_CM55_FE_BENCH_SCORE_CONDITION_MS_INDEX] =
        (float)copy_condition_ms;
    result->scores[APP_CM55_FE_BENCH_SCORE_SPECTRUM_MS_INDEX] =
        (float)spectrum_ms;
    result->scores[APP_CM55_FE_BENCH_SCORE_MELBANK_MS_INDEX] =
        (float)melbank_ms;
    result->scores[APP_CM55_FE_BENCH_SCORE_DB_MS_INDEX] = (float)db_ms;
    result->scores[APP_CM55_FE_BENCH_SCORE_TIME_BINS_INDEX] =
        (float)app_cm55_fe_bench_active_time_bins();
#if (APP_CM55_FE_BENCH_RFFT_ENABLE)
    result->scores[APP_CM55_FE_BENCH_SCORE_MODE_INDEX] =
        (float)APP_CM55_FE_BENCH_MODE_RFFT_FAST_F32;
#else
    result->scores[APP_CM55_FE_BENCH_SCORE_MODE_INDEX] =
        (float)APP_CM55_FE_BENCH_MODE_PROXY_NO_RFFT;
#endif
    result->scores[APP_CM55_FE_BENCH_SCORE_HASH_INDEX] =
        (float)(hash & 0xffffu);
}

#else

void app_cm55_fe_bench_maybe_run(uint32_t input_sequence,
                                 app_model_inference_result_t *result)
{
    (void)input_sequence;
    (void)result;
}

#endif /* APP_CM55_FE_BENCH_ENABLE */
