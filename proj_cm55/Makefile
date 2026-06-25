################################################################################
# \file Makefile
# \version 1.0
#
# \brief
# Makefile for configuring the Cortex-M55 core project
#
################################################################################
# \copyright
# Copyright 2025, Cypress Semiconductor Corporation (an Infineon company)
# SPDX-License-Identifier: Apache-2.0
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

include ../common.mk

################################################################################
# Basic Configuration
################################################################################

# Name of application (used to derive name of final linked file).
#
# If APPNAME is edited, ensure to update or regenerate launch
# configurations for your IDE.
APPNAME=proj_cm55

# If set to "true" or "1", display full command-lines when building.
VERBOSE=


################################################################################
# Advanced Configuration
################################################################################

# Enable optional code that is ordinarily disabled by default.
#
# Available components depend on the specific targeted hardware and firmware
# in use. In general, if you have
#
#    COMPONENTS=foo bar
#
# ... then code in directories named COMPONENT_foo and COMPONENT_bar will be
# added to the build
#
COMPONENTS+=FREERTOS GFXSS

# Display bring-up on CM55 (follows official LVGL demo architecture)
APP_DISPLAY_OFFICIAL_CM55_BRINGUP_ENABLE?=0
ifeq ($(APP_DISPLAY_OFFICIAL_CM55_BRINGUP_ENABLE),1)
APP_CM55_INFERENCE_ENABLE?=0
else
APP_CM55_INFERENCE_ENABLE?=1
endif
DEFINES+=APP_DISPLAY_OFFICIAL_CM55_BRINGUP_ENABLE=$(APP_DISPLAY_OFFICIAL_CM55_BRINGUP_ENABLE)
DEFINES+=APP_CM55_INFERENCE_ENABLE=$(APP_CM55_INFERENCE_ENABLE)
APP_DISPLAY_DIAG_ENABLE?=0
APP_DISPLAY_CM55_UART_LOG_ENABLE?=1
APP_DISPLAY_PANEL_I2C_SUBDIAG_ENABLE?=0
APP_DISPLAY_PANEL_I2C_BYPASS_ID_ENABLE?=0
APP_DISPLAY_PANEL_I2C_LOWLEVEL_PROBE_ENABLE?=0
APP_DISPLAY_CM55_SNAPSHOT_BRIDGE_ENABLE?=0
APP_DISPLAY_CM55_SNAPSHOT_BRIDGE_DIAG_ENABLE?=0
APP_DISPLAY_LVGL_ENABLE?=0
APP_DISPLAY_LVGL_SKIP_SNAPSHOT_READ_ENABLE?=0
APP_DISPLAY_LVGL_SNAPSHOT_PROBE_ENABLE?=0
APP_DISPLAY_LVGL_MINIMAL_REALTIME_ENABLE?=0
DEFINES+=APP_DISPLAY_DIAG_ENABLE=$(APP_DISPLAY_DIAG_ENABLE)
DEFINES+=APP_DISPLAY_CM55_UART_LOG_ENABLE=$(APP_DISPLAY_CM55_UART_LOG_ENABLE)
DEFINES+=APP_DISPLAY_PANEL_I2C_SUBDIAG_ENABLE=$(APP_DISPLAY_PANEL_I2C_SUBDIAG_ENABLE)
DEFINES+=APP_DISPLAY_PANEL_I2C_BYPASS_ID_ENABLE=$(APP_DISPLAY_PANEL_I2C_BYPASS_ID_ENABLE)
DEFINES+=APP_DISPLAY_PANEL_I2C_LOWLEVEL_PROBE_ENABLE=$(APP_DISPLAY_PANEL_I2C_LOWLEVEL_PROBE_ENABLE)
DEFINES+=APP_DISPLAY_CM55_SNAPSHOT_BRIDGE_ENABLE=$(APP_DISPLAY_CM55_SNAPSHOT_BRIDGE_ENABLE)
DEFINES+=APP_DISPLAY_CM55_SNAPSHOT_BRIDGE_DIAG_ENABLE=$(APP_DISPLAY_CM55_SNAPSHOT_BRIDGE_DIAG_ENABLE)
DEFINES+=APP_DISPLAY_LVGL_ENABLE=$(APP_DISPLAY_LVGL_ENABLE)
DEFINES+=APP_DISPLAY_LVGL_SKIP_SNAPSHOT_READ_ENABLE=$(APP_DISPLAY_LVGL_SKIP_SNAPSHOT_READ_ENABLE)
DEFINES+=APP_DISPLAY_LVGL_SNAPSHOT_PROBE_ENABLE=$(APP_DISPLAY_LVGL_SNAPSHOT_PROBE_ENABLE)
DEFINES+=APP_DISPLAY_LVGL_MINIMAL_REALTIME_ENABLE=$(APP_DISPLAY_LVGL_MINIMAL_REALTIME_ENABLE)

# When display bring-up is enabled, add GFXSS component and display defines
ifeq ($(APP_DISPLAY_OFFICIAL_CM55_BRINGUP_ENABLE),1)
COMPONENTS+=RTOS_AWARE
CONFIG_DISPLAY?=W4P3INCH_DISP
ifeq ($(CONFIG_DISPLAY),W4P3INCH_DISP)
DEFINES+=MTB_DISPLAY_W4P3INCH_RPI
CY_IGNORE+=../../mtb_shared/display-tft-ek79007ad3
else
$(error Unsupported CONFIG_DISPLAY=$(CONFIG_DISPLAY); display proof supports W4P3INCH_DISP only)
endif
else
CY_IGNORE+=source/app_cm55_display_bringup
CY_IGNORE+=$(SEARCH_display-dsi-waveshare-4-3-lcd)
CY_IGNORE+=$(SEARCH_display-tft-ek79007ad3)
endif

# LVGL integration — replaces CPU pixel rendering with LVGL widget framework.
# Requires APP_DISPLAY_OFFICIAL_CM55_BRINGUP_ENABLE=1 for GFXSS hardware init.
# Also requires snapshot bridge for live data binding.
ifeq ($(APP_DISPLAY_LVGL_ENABLE),1)
ifeq ($(APP_DISPLAY_OFFICIAL_CM55_BRINGUP_ENABLE),1)
LVGL_PATH=$(SEARCH_lvgl)
DEFINES+=LV_CONF_INCLUDE_SIMPLE
INCLUDES+=source/app_cm55_lvgl
# Use -isystem for LVGL root to prevent its subdirectories from shadowing
# system headers like <stdint.h> (the ThorVG rapidjson has an MSVC-only one).
CFLAGS+=-isystem $(LVGL_PATH)
include lvgl_sources.mk
# Disable LVGL demos and examples — only core src is compiled
CY_IGNORE+=$(LVGL_PATH)/demos
CY_IGNORE+=$(LVGL_PATH)/examples
CY_IGNORE+=$(LVGL_PATH)/lv_conf_template.h
CY_IGNORE+=$(LVGL_PATH)/env_support
# ThorVG rapidjson has MSVC-only headers that conflict with GCC arm-none-eabi
CY_IGNORE+=$(LVGL_PATH)/src/libs/thorvg
# Exclude Helium/NEON/arm2d assembly — we use LV_DRAW_SW_ASM_NONE
CY_IGNORE+=$(LVGL_PATH)/src/draw/sw/blend/helium
CY_IGNORE+=$(LVGL_PATH)/src/draw/sw/blend/neon
CY_IGNORE+=$(LVGL_PATH)/src/draw/sw/blend/arm2d
# Exclude LVGL tests — not needed for embedded build
CY_IGNORE+=$(LVGL_PATH)/tests
# Exclude LVGL's vg_lite_tvg — conflicts with PDL VGLite headers
CY_IGNORE+=$(LVGL_PATH)/src/others/vg_lite_tvg
CY_IGNORE+=$(LVGL_PATH)/src/draw/vg_lite
else
$(error APP_DISPLAY_LVGL_ENABLE=1 requires APP_DISPLAY_OFFICIAL_CM55_BRINGUP_ENABLE=1)
endif
else
CY_IGNORE+=source/app_cm55_lvgl
CY_IGNORE+=$(SEARCH_lvgl)
endif

ifeq ($(APP_CM55_INFERENCE_ENABLE),1)
COMPONENTS+=ML_TFLM ML_FLOAT32 U55
else
CY_IGNORE+=source/app_model_inference source/app_model_smoke model
CY_IGNORE+=../../mtb_shared/ml-middleware
CY_IGNORE+=../../mtb_shared/ml-middleware/release-v3.2.0
CY_IGNORE+=../../mtb_shared/ml-tflite-micro
CY_IGNORE+=../../mtb_shared/ml-tflite-micro/release-v3.2.0
endif

# Like COMPONENTS, but disable optional code that was enabled by default.
DISABLE_COMPONENTS=
ifneq ($(APP_CM55_INFERENCE_ENABLE),1)
DISABLE_COMPONENTS+=MW_ML_MIDDLEWARE MW_ML_TFLITE_MICRO
endif

CORE=CM55
CORE_NAME=CM55_0

# By default the build system automatically looks in the Makefile's directory
# tree for source code and builds it. The SOURCES variable can be used to
# manually add source code to the build process from a location not searched
# by default, or otherwise not found by the build system.
APP_AUDIO_MODEL_SELECT?=3
APP_MODEL_SMOKE_TEST_ENABLE?=0
APP_MODEL_RUNTIME_PROFILE_ENABLE?=1
APP_MODEL_LOG_LEVEL?=1
APP_MODEL_LOG_RATE_LIMIT_MS?=1000
APP_MODEL_PRINT_FLOAT_ENABLE?=1
APP_MODEL_INPUT_DUMP_ENABLE?=0
APP_MODEL_INPUT_DUMP_WINDOWS?=3
APP_MODEL_IPC_SMOKE_PAYLOAD_ENABLE?=0
APP_MODEL_IPC_SMOKE_RESET_DIAG_ENABLE?=0
APP_MODEL_IPC_SMOKE_EXCLUSIVE_ENABLE?=0
APP_MODEL_IPC_SMOKE_LOG_LIMIT?=64
APP_MODEL_IPC_SMOKE_CM55_DELAY_MS?=0
APP_MODEL_IPC_SMOKE_FORCE_WARM_STALE_READY_ENABLE?=0
APP_MODEL_IPC_SMOKE_FORCE_WARM_STALE_INPUT_ENABLE?=0
ifeq ($(APP_CM55_INFERENCE_ENABLE),1)
ifeq ($(APP_MODEL_IPC_SMOKE_EXCLUSIVE_ENABLE),1)
ifneq ($(APP_MODEL_IPC_SMOKE_PAYLOAD_ENABLE),1)
$(error APP_MODEL_IPC_SMOKE_EXCLUSIVE_ENABLE=1 requires APP_MODEL_IPC_SMOKE_PAYLOAD_ENABLE=1)
endif
ifneq ($(APP_MODEL_IPC_SMOKE_RESET_DIAG_ENABLE),1)
$(error APP_MODEL_IPC_SMOKE_EXCLUSIVE_ENABLE=1 requires APP_MODEL_IPC_SMOKE_RESET_DIAG_ENABLE=1)
endif
override APP_MODEL_SMOKE_TEST_ENABLE:=0
override APP_MODEL_RUNTIME_PROFILE_ENABLE:=0
override APP_MODEL_LOG_LEVEL:=0
override APP_MODEL_PRINT_FLOAT_ENABLE:=0
override APP_MODEL_INPUT_DUMP_ENABLE:=0
endif
ifeq ($(APP_AUDIO_MODEL_SELECT),2)
CY_IGNORE+=model/audio_model_v3_board_htk_hardneg_float.c
CY_IGNORE+=model/audio_model_v3.1_board_fpfix_float.c
DEFINES+=APP_AUDIO_MODEL_SELECT=2
else ifeq ($(APP_AUDIO_MODEL_SELECT),3)
CY_IGNORE+=model/audio_model_v2_float.c
CY_IGNORE+=model/audio_model_v3.1_board_fpfix_float.c
DEFINES+=APP_AUDIO_MODEL_SELECT=3
else ifeq ($(APP_AUDIO_MODEL_SELECT),4)
CY_IGNORE+=model/audio_model_v2_float.c
CY_IGNORE+=model/audio_model_v3_board_htk_hardneg_float.c
DEFINES+=APP_AUDIO_MODEL_SELECT=4
else
$(error Unsupported APP_AUDIO_MODEL_SELECT=$(APP_AUDIO_MODEL_SELECT); expected 2, 3, or 4)
endif
endif
DEFINES+=APP_MODEL_SMOKE_TEST_ENABLE=$(APP_MODEL_SMOKE_TEST_ENABLE)
DEFINES+=APP_MODEL_RUNTIME_PROFILE_ENABLE=$(APP_MODEL_RUNTIME_PROFILE_ENABLE)
DEFINES+=APP_MODEL_LOG_LEVEL=$(APP_MODEL_LOG_LEVEL)
DEFINES+=APP_MODEL_LOG_RATE_LIMIT_MS=$(APP_MODEL_LOG_RATE_LIMIT_MS)
DEFINES+=APP_MODEL_PRINT_FLOAT_ENABLE=$(APP_MODEL_PRINT_FLOAT_ENABLE)
DEFINES+=APP_MODEL_INPUT_DUMP_ENABLE=$(APP_MODEL_INPUT_DUMP_ENABLE)
DEFINES+=APP_MODEL_INPUT_DUMP_WINDOWS=$(APP_MODEL_INPUT_DUMP_WINDOWS)
DEFINES+=APP_MODEL_IPC_SMOKE_PAYLOAD_ENABLE=$(APP_MODEL_IPC_SMOKE_PAYLOAD_ENABLE)
DEFINES+=APP_MODEL_IPC_SMOKE_RESET_DIAG_ENABLE=$(APP_MODEL_IPC_SMOKE_RESET_DIAG_ENABLE)
DEFINES+=APP_MODEL_IPC_SMOKE_EXCLUSIVE_ENABLE=$(APP_MODEL_IPC_SMOKE_EXCLUSIVE_ENABLE)
DEFINES+=APP_MODEL_IPC_SMOKE_LOG_LIMIT=$(APP_MODEL_IPC_SMOKE_LOG_LIMIT)
DEFINES+=APP_MODEL_IPC_SMOKE_CM55_DELAY_MS=$(APP_MODEL_IPC_SMOKE_CM55_DELAY_MS)
DEFINES+=APP_MODEL_IPC_SMOKE_FORCE_WARM_STALE_READY_ENABLE=$(APP_MODEL_IPC_SMOKE_FORCE_WARM_STALE_READY_ENABLE)
DEFINES+=APP_MODEL_IPC_SMOKE_FORCE_WARM_STALE_INPUT_ENABLE=$(APP_MODEL_IPC_SMOKE_FORCE_WARM_STALE_INPUT_ENABLE)

# Like SOURCES, but for include directories. Value should be paths to
# directories (without a leading -I).
INCLUDES+=../shared source/app_cm55_display_bringup
ifeq ($(APP_CM55_INFERENCE_ENABLE),1)
INCLUDES+=model
endif

# Add additional defines to the build process (without a leading -D).
DEFINES+=CY_RETARGET_IO_CONVERT_LF_TO_CRLF
# TFLite Micro runtime 需要静态内存布局；模型为 float32 输入/输出，量化类型由
# COMPONENTS+=ML_FLOAT32 选择。U55 组件用于让构建系统链接 CM55/U55 对应的 TFLM 库。
DEFINES+=TF_LITE_STATIC_MEMORY
# audio_model_v2_float.c 由工具生成，当前版本里有一个旧返回码名 IMAI_RET_ERROR。
# 这里用编译宏做兼容，避免直接修改生成文件，后续重新导出模型时也更容易替换。
DEFINES+=IMAI_RET_ERROR=IPWIN_RET_ERROR

# Select softfp or hardfp floating point. Default is softfp.
VFP_SELECT+=

# Additional / custom C compiler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
CFLAGS+=-O3

# Additional / custom C++ compiler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
CXXFLAGS+=

# Additional / custom assembler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
ASFLAGS+=

# Additional / custom linker flags.
LDFLAGS+=

# Additional / custom libraries to link in to the application.
LDLIBS+=-lm

# Path to the linker script to use (if empty, use the default linker script).
LINKER_SCRIPT=

# Custom pre-build commands to run.
PREBUILD=

# Custom post-build commands to run.
POSTBUILD=


################################################################################
# Paths
################################################################################

# Relative path to the project directory (default is the Makefile's directory).
#
# This controls where automatic source code discovery looks for code.
CY_APP_PATH=

# Relative path to the shared repo location.
#
# All .mtb files have the format, <URI>#<COMMIT>#<LOCATION>. If the <LOCATION> 
# field begins with $$ASSET_REPO$$, then the repo is deposited in the path 
# specified by the CY_GETLIBS_SHARED_PATH variable. The default location is one 
# directory level above the current app directory. This is used with 
# CY_GETLIBS_SHARED_NAME variable, which specifies the directory name.
CY_GETLIBS_SHARED_PATH=../..

# Directory name of the shared repo location.
#
CY_GETLIBS_SHARED_NAME=mtb_shared

include $(CY_TOOLS_DIR)/make/start.mk
