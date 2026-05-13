/*
* ImagiNet Compiler 5.9.4927.65534+e5b229f227997e1d3e75ed83c30a1bcf08976921
* Copyright © 2023- Imagimob AB, All Rights Reserved.
* 
* Generated at 05/12/2026 05:34:07 UTC. Any changes will be lost.
* 
* Model ID  34aa8590-2e66-463c-8467-6810223dd86f
* 
* Memory    Size                      Efficiency
* State     582672 bytes (RAM)        100 %
* Readonly  23864 bytes (Flash)       100 %
* 
* Exported functions:
* 
*  @param data_in Input features. Input float[1,40,101].
*  @param data_out Output features. Output float[2].
*  void COUGH_compute(const float *data_in, float *data_out);
* 
*  @description: Closes and flushes streams, free any heap allocated memory.
*  void COUGH_finalize(void);
* 
*  @description: Resets windows and neural networks(i.e. RNNs) to initial state.
*  @return IPWIN_RET_SUCCESS (0) or IPWIN_RET_NODATA (-1), IPWIN_RET_ERROR (-2), IPWIN_RET_STREAMEND (-3)
*  int COUGH_soft_reset(void);
* 
*  @description: Initializes buffers to initial state.
*  @return IPWIN_RET_SUCCESS (0) or IPWIN_RET_NODATA (-1), IPWIN_RET_ERROR (-2), IPWIN_RET_STREAMEND (-3)
*  int COUGH_init(void);
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
#define COUGH_API_FUNCTION

typedef int8_t q7_t;         // 8-bit fractional data type in Q1.7 format.
typedef int16_t q15_t;       // 16-bit fractional data type in Q1.15 format.
typedef int32_t q31_t;       // 32-bit fractional data type in Q1.31 format.
typedef int64_t q63_t;       // 64-bit fractional data type in Q1.63 format.
typedef float timestamp_t;

// Model GUID (16 bytes)
#define COUGH_MODEL_ID {0x90, 0x85, 0xaa, 0x34, 0x66, 0x2e, 0x3c, 0x46, 0x84, 0x67, 0x68, 0x10, 0x22, 0x3d, 0xd8, 0x6f}


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


#define COUGH_COMPUTE_INPUTS (1)
#define COUGH_COMPUTE_OUTPUTS (1)
#define COUGH_COMPUTE_IN_TYPE float
#define COUGH_COMPUTE_IN_TYPE_ID IMAGINET_TYPES_FLOAT32
#define COUGH_COMPUTE_OUT_TYPE float
#define COUGH_COMPUTE_OUT_TYPE_ID IMAGINET_TYPES_FLOAT32
#define COUGH_COMPUTE_OUT_NO_COPY false

// data_in [1,40,101] (16160 bytes)
#define COUGH_DATA_IN_RANK (3)
#define COUGH_DATA_IN_SHAPE ((int[]){101, 40, 1})
#define COUGH_DATA_IN_COUNT (4040)
#define COUGH_DATA_IN_BYTES (16160)
#define COUGH_DATA_IN_TYPE float
#define COUGH_DATA_IN_TYPE_ID IMAGINET_TYPES_FLOAT32
#define COUGH_DATA_IN_SHIFT 0
#define COUGH_DATA_IN_OFFSET 0
#define COUGH_DATA_IN_SCALE 0
#define COUGH_DATA_IN_SYMBOLS { }

// data_out [2] (8 bytes)
#define COUGH_DATA_OUT_RANK (1)
#define COUGH_DATA_OUT_SHAPE ((int[]){2})
#define COUGH_DATA_OUT_COUNT (2)
#define COUGH_DATA_OUT_BYTES (8)
#define COUGH_DATA_OUT_TYPE float
#define COUGH_DATA_OUT_TYPE_ID IMAGINET_TYPES_FLOAT32
#define COUGH_DATA_OUT_SHIFT 0
#define COUGH_DATA_OUT_OFFSET 0
#define COUGH_DATA_OUT_SCALE 0
#define COUGH_DATA_OUT_SYMBOLS { }

#define COUGH_KEY_MAX (5)

// Return codes
#define COUGH_RET_SUCCESS 0
#define COUGH_RET_NODATA -1
#define COUGH_RET_ERROR -2
#define COUGH_RET_STREAMEND -3

#define IPWIN_RET_SUCCESS 0
#define IPWIN_RET_NODATA -1
#define IPWIN_RET_ERROR -2
#define IPWIN_RET_STREAMEND -3

// Exported methods
void COUGH_compute(const float *restrict data_in, float *restrict data_out);
void COUGH_finalize(void);
int COUGH_soft_reset(void);
int COUGH_init(void);

// Symbol COUGH_PROFILING must be defined to enable profiling of models
// Symbol COUGH_PROFILING_LOG will enable printing the raw outputs of neural networks
/// @brief This method will print the region profiling results
void COUGH_print_region_profiling(void);
#ifdef COUGH_PROFILING
/// @brief Implement this method to perform profiling, should populate the value pointed to by val with the current tick count
int COUGH_get_ticks(uint64_t *val);
/// @brief Only re-implement this method if you want to perform custom profiling of regions in the generated code
void COUGH_hook_region(bool entered, int32_t region_id);
#endif

// Quantization helpers
/// @brief Dequantize a quantized integer to a floating-point value
int COUGH_dequantize(const int8_t *restrict src, float *restrict dst, int32_t type_id, int32_t count, float scale, int32_t offset);
/// @brief Quantize a floating-point value to a quantized integer
int COUGH_quantize(const float *restrict src, int8_t *restrict dst, int32_t type_id, int32_t count, float scale, int32_t offset);

/// @brief This method will print neural network inference profiling results
void COUGH_mtb_models_profile_log();
/// @brief This method will print neural network information
void COUGH_mtb_models_print_info();
#define COUGH_MAX_MTB_MODELS 4
extern int32_t COUGH_mtb_models_count;
extern mtb_ml_model_t* COUGH_mtb_models[COUGH_MAX_MTB_MODELS];

// Profiling regions
#ifdef COUGH_PROFILING
    #define COUGH_REGIONS_COUNT 1
    #define COUGH_REGIONS_NAMES {\
    	"AUDIO_MODEL_V3_BOARD_HTK_HARDNEG",\
    }
#else
    #define COUGH_REGIONS_COUNT 0
    #define COUGH_REGIONS_NAMES {}
#endif
// Call macros — invoke any exported function via a void* array
#define COUGH_COMPUTE_PTR(a) COUGH_compute((const float *)(a)[0], (float *)(a)[1])
#define COUGH_FINALIZE_PTR(a) COUGH_finalize()
#define COUGH_SOFT_RESET_PTR(a) COUGH_soft_reset()
#define COUGH_INIT_PTR(a) COUGH_init()


typedef enum {
    COUGH_PARAM_UNDEFINED = 0,
    COUGH_PARAM_INPUT = 1,
    COUGH_PARAM_OUTPUT = 2,
    COUGH_PARAM_REFERENCE = 3,
    COUGH_PARAM_HANDLE = 7,
    COUGH_PARAM_CALLBACK = 8,
    COUGH_PARAM_OUTPUT_REF = 18,
} COUGH_param_attrib;

typedef char *label_text_t;

typedef struct {
    char* name;
    int size;
    label_text_t *labels;
} COUGH_shape_dim;

typedef struct {
    char* name;
    COUGH_param_attrib attrib;
    int32_t rank;
    COUGH_shape_dim *shape;
    int32_t count;
    int32_t bytes;
    int32_t type_id;
    float frequency;
    int shift;
    float scale;
    long offset;
} COUGH_param_def;

typedef enum {
    COUGH_FUNC_ATTRIB_NONE = 0,
    COUGH_FUNC_ATTRIB_CAN_FAIL = 1,
    COUGH_FUNC_ATTRIB_PUBLIC = 2,
    COUGH_FUNC_ATTRIB_INIT = 4,
    COUGH_FUNC_ATTRIB_DESTRUCTOR = 8,
} COUGH_func_attrib;

typedef struct {
    char* name;
    char* description;
    void* fn_ptr;
    COUGH_func_attrib attrib;
    int32_t param_count;
    COUGH_param_def *param_list;
} COUGH_func_def;

typedef struct {
    uint32_t size;
    uint32_t peak_usage;
} COUGH_mem_usage;

typedef enum {
    COUGH_API_TYPE_UNDEFINED = 0,
    COUGH_API_TYPE_FUNCTION = 1,
    COUGH_API_TYPE_QUEUE = 2,
    COUGH_API_TYPE_QUEUE_TIME = 3,
    COUGH_API_TYPE_USER_DEFINED = 4,
} COUGH_api_type;

typedef struct {
    uint32_t api_ver;
    uint8_t id[16];
    COUGH_api_type api_type;
    char* prefix;
    COUGH_mem_usage buffer_mem;
    COUGH_mem_usage static_mem;
    COUGH_mem_usage readonly_mem;
    int32_t func_count;
    COUGH_func_def *func_list;
} COUGH_api_def;

COUGH_api_def *COUGH_api(void);

