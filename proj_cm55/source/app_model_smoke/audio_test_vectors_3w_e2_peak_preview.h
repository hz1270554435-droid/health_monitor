#ifndef AUDIO_TEST_VECTORS_3W_E2_PEAK_PREVIEW_H
#define AUDIO_TEST_VECTORS_3W_E2_PEAK_PREVIEW_H

#include "e2_3w_fixed_vector.h"

#define AUDIO_TEST_VECTOR_BATCH 1
#define AUDIO_TEST_VECTOR_CHANNELS E2_3W_VECTOR_CHANNELS
#define AUDIO_TEST_VECTOR_N_MELS E2_3W_VECTOR_N_MELS
#define AUDIO_TEST_VECTOR_TIME_FRAMES E2_3W_VECTOR_TIME_FRAMES
#define AUDIO_TEST_VECTOR_SIZE E2_3W_VECTOR_SIZE
#define AUDIO_TEST_VECTOR_COUNT 1
#define AUDIO_TEST_VECTOR_EXPECTED_COUNT AUDIO_TEST_VECTOR_COUNT

static const float* const audio_test_vectors[AUDIO_TEST_VECTOR_COUNT] = {
    e2_3w_fixed_vector
};

static const float audio_test_vector_expected_output0[AUDIO_TEST_VECTOR_COUNT] = {
    -0.61893719f
};

static const float audio_test_vector_expected_output1[AUDIO_TEST_VECTOR_COUNT] = {
    0.639714f
};

static const float audio_test_vector_expected_cough_prob[AUDIO_TEST_VECTOR_COUNT] = {
    0.77879381f
};

#endif /* AUDIO_TEST_VECTORS_3W_E2_PEAK_PREVIEW_H */
