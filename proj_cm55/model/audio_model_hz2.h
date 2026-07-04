/*
* ImagiNet Compiler 5.9.4927.65534+e5b229f227997e1d3e75ed83c30a1bcf08976921
* Copyright © 2023- Imagimob AB, All Rights Reserved.
* 
* Generated at 07/04/2026 02:54:02 UTC. Any changes will be lost.
* 
* Model ID  b53fa698-7cdf-45fe-b1a8-11963e6b9293
* 
* Memory    Size                      Efficiency
* Buffers   8 bytes (RAM)             100 %
* State     569344 bytes (RAM)        100 %
* Readonly  106088 bytes (Flash)      100 %
* 
* Exported functions:
* 
*  @param data_in args_0. Input float[1,40,94].
*  @param aux_logits aux_logits. Output float[5].
*  @param binary_logits binary_logits. Output float[2].
*  @return IPWIN_RET_SUCCESS (0) or IPWIN_RET_NODATA (-1), IPWIN_RET_ERROR (-2), IPWIN_RET_STREAMEND (-3)
*  int AUDIO_compute(const float *data_in, float *aux_logits, float *binary_logits);
* 
*  @description: Closes and flushes streams, free any heap allocated memory.
*  void AUDIO_finalize(void);
* 
*  @description: Resets windows and neural networks(i.e. RNNs) to initial state.
*  @return IPWIN_RET_SUCCESS (0) or IPWIN_RET_NODATA (-1), IPWIN_RET_ERROR (-2), IPWIN_RET_STREAMEND (-3)
*  int AUDIO_soft_reset(void);
* 
*  @description: Initializes buffers to initial state.
*  @return IPWIN_RET_SUCCESS (0) or IPWIN_RET_NODATA (-1), IPWIN_RET_ERROR (-2), IPWIN_RET_STREAMEND (-3)
*  int AUDIO_init(void);
* 
* 
* Disclaimer:
*   The generated code relies on the optimizations done by the C compiler.
*   For example many for-loops of length 1 must be removed by the optimizer.
*   This can only be done if the functions are inlined and simplified.
*   Check disassembly if unsure.
*   tl;dr Compile using gcc with -O3 or -Ofast
* 
* Notes:
* 	-> This code was generated with DEEPCRAFT™ Model Converter using:
* 		ml-coretools 3.1.0.9404.
* 		tensorflow 2.19.0.
* 		ethos-u-vela 4.5.0.
* 	-> This code requires the following Modus Toolbox libraries (add them to your
* 	project using the Library Manager):
* 		ml-middleware 3.2.0.
* 		ml-tflite-micro 3.2.0.
*/

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "mtb_ml_model.h"
#define AUDIO_API_USER_DEFINED

typedef int8_t q7_t;         // 8-bit fractional data type in Q1.7 format.
typedef int16_t q15_t;       // 16-bit fractional data type in Q1.15 format.
typedef int32_t q31_t;       // 32-bit fractional data type in Q1.31 format.
typedef int64_t q63_t;       // 64-bit fractional data type in Q1.63 format.
typedef float timestamp_t;

// Model GUID (16 bytes)
#define AUDIO_MODEL_ID {0x98, 0xa6, 0x3f, 0xb5, 0xdf, 0x7c, 0xfe, 0x45, 0xb1, 0xa8, 0x11, 0x96, 0x3e, 0x6b, 0x92, 0x93}


// First nibble is bit encoding, second nibble is number of bytes
#define IMAGINET_TYPES_NONE	(0x0)
#define IMAGINET_TYPES_FLOAT32	(0x14)
#define IMAGINET_TYPES_FLOAT64	(0x18)
#define IMAGINET_TYPES_INT8	(0x21)
#define IMAGINET_TYPES_INT16	(0x22)
#define IMAGINET_TYPES_INT32	(0x24)
#define IMAGINET_TYPES_INT64	(0x28)
#define IMAGINET_TYPES_Q7	(0x31)
#define IMAGINET_TYPES_Q15	(0x32)
#define IMAGINET_TYPES_Q31	(0x34)
#define IMAGINET_TYPES_BOOL	(0x41)
#define IMAGINET_TYPES_STRING	(0x54)
#define IMAGINET_TYPES_D8	(0x61)
#define IMAGINET_TYPES_D16	(0x62)
#define IMAGINET_TYPES_D32	(0x64)
#define IMAGINET_TYPES_UINT8	(0x71)
#define IMAGINET_TYPES_UINT16	(0x72)
#define IMAGINET_TYPES_UINT32	(0x74)
#define IMAGINET_TYPES_UINT64	(0x78)


#define AUDIO_COMPUTE_INPUTS (1)
#define AUDIO_COMPUTE_OUTPUTS (2)
#define AUDIO_COMPUTE_IN_TYPE float
#define AUDIO_COMPUTE_IN_TYPE_ID IMAGINET_TYPES_FLOAT32
#define AUDIO_COMPUTE_OUT_TYPE float
#define AUDIO_COMPUTE_OUT_TYPE_ID IMAGINET_TYPES_FLOAT32
#define AUDIO_COMPUTE_OUT_NO_COPY false

// data_in [1,40,94] (15040 bytes)
#define AUDIO_DATA_IN_RANK (3)
#define AUDIO_DATA_IN_SHAPE ((int[]){94, 40, 1})
#define AUDIO_DATA_IN_COUNT (3760)
#define AUDIO_DATA_IN_BYTES (15040)
#define AUDIO_DATA_IN_TYPE float
#define AUDIO_DATA_IN_TYPE_ID IMAGINET_TYPES_FLOAT32
#define AUDIO_DATA_IN_SHIFT 0
#define AUDIO_DATA_IN_OFFSET 0
#define AUDIO_DATA_IN_SCALE 1
#define AUDIO_DATA_IN_SYMBOLS { }

// aux_logits [5] (20 bytes)
#define AUDIO_AUX_LOGITS_RANK (1)
#define AUDIO_AUX_LOGITS_SHAPE ((int[]){5})
#define AUDIO_AUX_LOGITS_COUNT (5)
#define AUDIO_AUX_LOGITS_BYTES (20)
#define AUDIO_AUX_LOGITS_TYPE float
#define AUDIO_AUX_LOGITS_TYPE_ID IMAGINET_TYPES_FLOAT32
#define AUDIO_AUX_LOGITS_SHIFT 0
#define AUDIO_AUX_LOGITS_OFFSET 0
#define AUDIO_AUX_LOGITS_SCALE 1
#define AUDIO_AUX_LOGITS_SYMBOLS { }

// binary_logits [2] (8 bytes)
#define AUDIO_BINARY_LOGITS_RANK (1)
#define AUDIO_BINARY_LOGITS_SHAPE ((int[]){2})
#define AUDIO_BINARY_LOGITS_COUNT (2)
#define AUDIO_BINARY_LOGITS_BYTES (8)
#define AUDIO_BINARY_LOGITS_TYPE float
#define AUDIO_BINARY_LOGITS_TYPE_ID IMAGINET_TYPES_FLOAT32
#define AUDIO_BINARY_LOGITS_SHIFT 0
#define AUDIO_BINARY_LOGITS_OFFSET 0
#define AUDIO_BINARY_LOGITS_SCALE 1
#define AUDIO_BINARY_LOGITS_SYMBOLS { }

#define AUDIO_KEY_MAX (7)

// Return codes
#define AUDIO_RET_SUCCESS 0
#define AUDIO_RET_NODATA -1
#define AUDIO_RET_ERROR -2
#define AUDIO_RET_STREAMEND -3

#define IPWIN_RET_SUCCESS 0
#define IPWIN_RET_NODATA -1
#define IPWIN_RET_ERROR -2
#define IPWIN_RET_STREAMEND -3

// Exported methods
int AUDIO_compute(const float *restrict data_in, float *restrict aux_logits, float *restrict binary_logits);
void AUDIO_finalize(void);
int AUDIO_soft_reset(void);
int AUDIO_init(void);

// Symbol AUDIO_PROFILING must be defined to enable profiling of models
// Symbol AUDIO_PROFILING_LOG will enable printing the raw outputs of neural networks
/// @brief This method will print the region profiling results
void AUDIO_print_region_profiling(void);
#ifdef AUDIO_PROFILING
/// @brief Implement this method to perform profiling, should populate the value pointed to by val with the current tick count
int AUDIO_get_ticks(uint64_t *val);
/// @brief Only re-implement this method if you want to perform custom profiling of regions in the generated code
void AUDIO_hook_region(bool entered, int32_t region_id);
#endif

// Quantization helpers
/// @brief Dequantize a quantized integer to a floating-point value
int AUDIO_dequantize(const int8_t *restrict src, float *restrict dst, int32_t type_id, int32_t count, float scale, int32_t offset);
/// @brief Quantize a floating-point value to a quantized integer
int AUDIO_quantize(const float *restrict src, int8_t *restrict dst, int32_t type_id, int32_t count, float scale, int32_t offset);

/// @brief This method will print neural network inference profiling results
void AUDIO_mtb_models_profile_log();
/// @brief This method will print neural network information
void AUDIO_mtb_models_print_info();
#define AUDIO_MAX_MTB_MODELS 4
extern int32_t AUDIO_mtb_models_count;
extern mtb_ml_model_t* AUDIO_mtb_models[AUDIO_MAX_MTB_MODELS];

// Profiling regions
#ifdef AUDIO_PROFILING
    #define AUDIO_REGIONS_COUNT 4
    #define AUDIO_REGIONS_NAMES {\
    	"SET INPUT 0",\
    	"INVOKE HZ2_B0",\
    	"GET OUTPUT 0",\
    	"GET OUTPUT 1",\
    }
#else
    #define AUDIO_REGIONS_COUNT 0
    #define AUDIO_REGIONS_NAMES {}
#endif
// Call macros — invoke any exported function via a void* array
#define AUDIO_COMPUTE_PTR(a) AUDIO_compute((const float *)(a)[0], (float *)(a)[1], (float *)(a)[2])
#define AUDIO_FINALIZE_PTR(a) AUDIO_finalize()
#define AUDIO_SOFT_RESET_PTR(a) AUDIO_soft_reset()
#define AUDIO_INIT_PTR(a) AUDIO_init()


typedef enum {
    AUDIO_PARAM_UNDEFINED = 0,
    AUDIO_PARAM_INPUT = 1,
    AUDIO_PARAM_OUTPUT = 2,
    AUDIO_PARAM_REFERENCE = 3,
    AUDIO_PARAM_HANDLE = 7,
    AUDIO_PARAM_CALLBACK = 8,
    AUDIO_PARAM_OUTPUT_REF = 18,
} AUDIO_param_attrib;

typedef char *label_text_t;

typedef struct {
    char* name;
    int size;
    label_text_t *labels;
} AUDIO_shape_dim;

typedef struct {
    char* name;
    AUDIO_param_attrib attrib;
    int32_t rank;
    AUDIO_shape_dim *shape;
    int32_t count;
    int32_t bytes;
    int32_t type_id;
    float frequency;
    int shift;
    float scale;
    long offset;
} AUDIO_param_def;

typedef enum {
    AUDIO_FUNC_ATTRIB_NONE = 0,
    AUDIO_FUNC_ATTRIB_CAN_FAIL = 1,
    AUDIO_FUNC_ATTRIB_PUBLIC = 2,
    AUDIO_FUNC_ATTRIB_INIT = 4,
    AUDIO_FUNC_ATTRIB_DESTRUCTOR = 8,
} AUDIO_func_attrib;

typedef struct {
    char* name;
    char* description;
    void* fn_ptr;
    AUDIO_func_attrib attrib;
    int32_t param_count;
    AUDIO_param_def *param_list;
} AUDIO_func_def;

typedef struct {
    uint32_t size;
    uint32_t peak_usage;
} AUDIO_mem_usage;

typedef enum {
    AUDIO_API_TYPE_UNDEFINED = 0,
    AUDIO_API_TYPE_FUNCTION = 1,
    AUDIO_API_TYPE_QUEUE = 2,
    AUDIO_API_TYPE_QUEUE_TIME = 3,
    AUDIO_API_TYPE_USER_DEFINED = 4,
} AUDIO_api_type;

typedef struct {
    uint32_t api_ver;
    uint8_t id[16];
    AUDIO_api_type api_type;
    char* prefix;
    AUDIO_mem_usage buffer_mem;
    AUDIO_mem_usage static_mem;
    AUDIO_mem_usage readonly_mem;
    int32_t func_count;
    AUDIO_func_def *func_list;
} AUDIO_api_def;

AUDIO_api_def *AUDIO_api(void);

