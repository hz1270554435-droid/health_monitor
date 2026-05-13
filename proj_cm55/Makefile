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
COMPONENTS+=FREERTOS ML_TFLM ML_FLOAT32 U55

# Like COMPONENTS, but disable optional code that was enabled by default.
DISABLE_COMPONENTS=

CORE=CM55
CORE_NAME=CM55_0

# By default the build system automatically looks in the Makefile's directory
# tree for source code and builds it. The SOURCES variable can be used to
# manually add source code to the build process from a location not searched
# by default, or otherwise not found by the build system.
APP_AUDIO_MODEL_SELECT?=3
APP_MODEL_SMOKE_TEST_ENABLE?=0
APP_MODEL_RUNTIME_PROFILE_ENABLE?=0
APP_MODEL_LOG_LEVEL?=1
APP_MODEL_LOG_RATE_LIMIT_MS?=1000
APP_MODEL_PRINT_FLOAT_ENABLE?=1
ifeq ($(APP_AUDIO_MODEL_SELECT),3)
CY_IGNORE+=model/audio_model_v2_float.c
DEFINES+=APP_AUDIO_MODEL_SELECT=3
else
CY_IGNORE+=model/audio_model_v3_board_htk_hardneg_float.c
DEFINES+=APP_AUDIO_MODEL_SELECT=2
endif
DEFINES+=APP_MODEL_SMOKE_TEST_ENABLE=$(APP_MODEL_SMOKE_TEST_ENABLE)
DEFINES+=APP_MODEL_RUNTIME_PROFILE_ENABLE=$(APP_MODEL_RUNTIME_PROFILE_ENABLE)
DEFINES+=APP_MODEL_LOG_LEVEL=$(APP_MODEL_LOG_LEVEL)
DEFINES+=APP_MODEL_LOG_RATE_LIMIT_MS=$(APP_MODEL_LOG_RATE_LIMIT_MS)
DEFINES+=APP_MODEL_PRINT_FLOAT_ENABLE=$(APP_MODEL_PRINT_FLOAT_ENABLE)

# Like SOURCES, but for include directories. Value should be paths to
# directories (without a leading -I).
INCLUDES+=../shared model

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
