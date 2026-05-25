################################################################################
# \file Makefile
# \version 1.0
#
# \brief
# Makefile for configuring the Cortex-M33 core project
#
################################################################################
# \copyright
# Copyright 2025, Cypress Semiconductor Corporation (an Infineon company)
# SPDX-License-Identifier: Apache-2.0
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 对应子agent
#     http://ww模型w.apache.org/licenses/LICENSE-2.0
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
APPNAME=proj_cm33_ns

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
COMPONENTS+=NON_SECURE_DEVICE FREERTOS

# Like COMPONENTS, but disable optional code that was enabled by default.
DISABLE_COMPONENTS=

CORE=CM33
CORE_NAME=CM33_0

# By default the build system automatically looks in the Makefile's directory
# tree for source code and builds it. The SOURCES variable can be used to
# manually add source code to the build process from a location not searched
# by default, or otherwise not found by the build system.
SOURCES+=

# Like SOURCES, but for include directories. Value should be paths to
# directories (without a leading -I).
INCLUDES+=../shared source/app_ble source/app_display

# Add additional defines to the build process (without a leading -D).
# Keep CR/LF conversion disabled: training export sends binary PCM frames over
# retarget-io, and translating 0x0A bytes would corrupt the payload.
APP_AUDIO_MODEL_SELECT?=3
APP_MODEL_SMOKE_TEST_ENABLE?=0
APP_MODEL_RUNTIME_PROFILE_ENABLE?=1
APP_MODEL_LOG_LEVEL?=1
APP_MODEL_LOG_RATE_LIMIT_MS?=1000
APP_MODEL_PRINT_FLOAT_ENABLE?=1
APP_MODEL_RESULT_MONITOR_ENABLE?=1
APP_RUNTIME_MODE?=0
APP_AUDIO_SPECTRUM_BACKEND?=1
APP_AUDIO_SPECTRUM_COMPARE_ENABLE?=0
APP_AUDIO_SPECTRUM_COMPARE_WINDOWS?=3
APP_AUDIO_FEATURE_DUMP_ENABLE?=0
APP_AUDIO_FEATURE_DUMP_WINDOWS?=3
APP_AUDIO_EVENT_FEATURE_DUMP_ENABLE?=0
APP_AUDIO_EVENT_FEATURE_DUMP_MAX_EVENTS?=1
APP_AUDIO_EVENT_FEATURE_DUMP_RING_DEPTH?=2
APP_AUDIO_EVENT_PCM_DUMP_ENABLE?=0
APP_AUDIO_EVENT_PCM_DUMP_MAX_EVENTS?=1
APP_AUDIO_EVENT_PCM_DUMP_RING_DEPTH?=1
APP_AUDIO_EVENT_PCM_DUMP_INCLUDE_FEATURE?=1
APP_MODEL_INPUT_DUMP_ENABLE?=0
APP_MODEL_INPUT_DUMP_WINDOWS?=3
APP_PDM_PCM_BLOCK_COUNT?=32
APP_MODEL_EVENT_THRESHOLD?=0.95f
APP_MODEL_EVENT_MIN_HITS?=2
APP_MODEL_EVENT_WINDOW_MS?=1200
APP_MODEL_EVENT_COOLDOWN_MS?=1500
APP_MODEL_EVENT_MIN_ENERGY?=0.0f
APP_CSV_EXPORT_CAPTURE_MODE?=0
APP_BLE_ENABLE?=0
APP_BLE_FAKE_DATA_ENABLE?=0
APP_BLE_STACK_ENABLE?=0
APP_BLE_DEBUG_DISABLE_INFERENCE?=0
APP_DISPLAY_ENABLE?=0
APP_DISPLAY_SMOKE_ENABLE?=0

ifneq ($(strip $(APP_DEBUG_UART_BAUD_RATE)),)
DEFINES+=APP_DEBUG_UART_BAUD_RATE=$(APP_DEBUG_UART_BAUD_RATE)
endif
DEFINES+=APP_AUDIO_MODEL_SELECT=$(APP_AUDIO_MODEL_SELECT)
DEFINES+=APP_MODEL_SMOKE_TEST_ENABLE=$(APP_MODEL_SMOKE_TEST_ENABLE)
DEFINES+=APP_MODEL_RUNTIME_PROFILE_ENABLE=$(APP_MODEL_RUNTIME_PROFILE_ENABLE)
DEFINES+=APP_MODEL_LOG_LEVEL=$(APP_MODEL_LOG_LEVEL)
DEFINES+=APP_MODEL_LOG_RATE_LIMIT_MS=$(APP_MODEL_LOG_RATE_LIMIT_MS)
DEFINES+=APP_MODEL_PRINT_FLOAT_ENABLE=$(APP_MODEL_PRINT_FLOAT_ENABLE)
DEFINES+=APP_MODEL_RESULT_MONITOR_ENABLE=$(APP_MODEL_RESULT_MONITOR_ENABLE)
DEFINES+=APP_RUNTIME_MODE=$(APP_RUNTIME_MODE)
DEFINES+=APP_AUDIO_SPECTRUM_BACKEND=$(APP_AUDIO_SPECTRUM_BACKEND)
DEFINES+=APP_AUDIO_SPECTRUM_COMPARE_ENABLE=$(APP_AUDIO_SPECTRUM_COMPARE_ENABLE)
DEFINES+=APP_AUDIO_SPECTRUM_COMPARE_WINDOWS=$(APP_AUDIO_SPECTRUM_COMPARE_WINDOWS)
DEFINES+=APP_AUDIO_FEATURE_DUMP_ENABLE=$(APP_AUDIO_FEATURE_DUMP_ENABLE)
DEFINES+=APP_AUDIO_FEATURE_DUMP_WINDOWS=$(APP_AUDIO_FEATURE_DUMP_WINDOWS)
DEFINES+=APP_AUDIO_EVENT_FEATURE_DUMP_ENABLE=$(APP_AUDIO_EVENT_FEATURE_DUMP_ENABLE)
DEFINES+=APP_AUDIO_EVENT_FEATURE_DUMP_MAX_EVENTS=$(APP_AUDIO_EVENT_FEATURE_DUMP_MAX_EVENTS)
DEFINES+=APP_AUDIO_EVENT_FEATURE_DUMP_RING_DEPTH=$(APP_AUDIO_EVENT_FEATURE_DUMP_RING_DEPTH)
DEFINES+=APP_AUDIO_EVENT_PCM_DUMP_ENABLE=$(APP_AUDIO_EVENT_PCM_DUMP_ENABLE)
DEFINES+=APP_AUDIO_EVENT_PCM_DUMP_MAX_EVENTS=$(APP_AUDIO_EVENT_PCM_DUMP_MAX_EVENTS)
DEFINES+=APP_AUDIO_EVENT_PCM_DUMP_RING_DEPTH=$(APP_AUDIO_EVENT_PCM_DUMP_RING_DEPTH)
DEFINES+=APP_AUDIO_EVENT_PCM_DUMP_INCLUDE_FEATURE=$(APP_AUDIO_EVENT_PCM_DUMP_INCLUDE_FEATURE)
DEFINES+=APP_MODEL_INPUT_DUMP_ENABLE=$(APP_MODEL_INPUT_DUMP_ENABLE)
DEFINES+=APP_MODEL_INPUT_DUMP_WINDOWS=$(APP_MODEL_INPUT_DUMP_WINDOWS)
DEFINES+=APP_PDM_PCM_BLOCK_COUNT=$(APP_PDM_PCM_BLOCK_COUNT)
DEFINES+=APP_MODEL_EVENT_THRESHOLD=$(APP_MODEL_EVENT_THRESHOLD)
DEFINES+=APP_MODEL_EVENT_MIN_HITS=$(APP_MODEL_EVENT_MIN_HITS)
DEFINES+=APP_MODEL_EVENT_WINDOW_MS=$(APP_MODEL_EVENT_WINDOW_MS)
DEFINES+=APP_MODEL_EVENT_COOLDOWN_MS=$(APP_MODEL_EVENT_COOLDOWN_MS)
DEFINES+=APP_MODEL_EVENT_MIN_ENERGY=$(APP_MODEL_EVENT_MIN_ENERGY)
DEFINES+=APP_CSV_EXPORT_CAPTURE_MODE=$(APP_CSV_EXPORT_CAPTURE_MODE)
DEFINES+=APP_BLE_ENABLE=$(APP_BLE_ENABLE)
DEFINES+=APP_BLE_FAKE_DATA_ENABLE=$(APP_BLE_FAKE_DATA_ENABLE)
DEFINES+=APP_BLE_STACK_ENABLE=$(APP_BLE_STACK_ENABLE)
DEFINES+=APP_BLE_DEBUG_DISABLE_INFERENCE=$(APP_BLE_DEBUG_DISABLE_INFERENCE)
DEFINES+=APP_DISPLAY_ENABLE=$(APP_DISPLAY_ENABLE)
DEFINES+=APP_DISPLAY_SMOKE_ENABLE=$(APP_DISPLAY_SMOKE_ENABLE)

ifneq ($(filter 1,$(APP_AUDIO_SPECTRUM_BACKEND) $(APP_AUDIO_SPECTRUM_COMPARE_ENABLE)),)
COMPONENTS+=CMSIS_DSP
endif

ifeq ($(APP_BLE_ENABLE),1)
ifeq ($(APP_BLE_STACK_ENABLE),1)
COMPONENTS+=WICED_BLE
endif
endif

ifeq ($(APP_BLE_STACK_ENABLE),0)
DISABLE_COMPONENTS+=HCI-UART WICED_BLE WICED_DUALMODE
CY_IGNORE+=../../mtb_shared/btstack-integration
CY_IGNORE+=../../mtb_shared/btstack
endif

ifeq ($(APP_BLE_ENABLE),0)
CY_IGNORE+=source/app_ble/app_ble_core.c
CY_IGNORE+=source/app_ble/app_ble_cmd.c
CY_IGNORE+=source/app_ble/app_ble_diag.c
CY_IGNORE+=source/app_ble/app_ble_gatt.c
CY_IGNORE+=source/app_ble/app_ble_protocol.c
CY_IGNORE+=source/app_ble/app_ble_stream.c
else
ifeq ($(APP_BLE_STACK_ENABLE),0)
CY_IGNORE+=source/app_ble/app_ble_core.c
CY_IGNORE+=source/app_ble/app_ble_gatt.c
endif
endif

ifeq ($(APP_DISPLAY_ENABLE),0)
CY_IGNORE+=source/app_display/app_display.c
CY_IGNORE+=source/app_display/app_display_backend_null.c
endif

# Select softfp or hardfp floating point. Default is softfp.
VFP_SELECT+=

# Additional / custom C compiler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
CFLAGS+=

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
# directory level above the current app directory.
# This is used with CY_GETLIBS_SHARED_NAME variable, which specifies the 
# directory name.
CY_GETLIBS_SHARED_PATH=../..

# Directory name of the shared repo location.
#
CY_GETLIBS_SHARED_NAME=mtb_shared

include $(CY_TOOLS_DIR)/make/start.mk
