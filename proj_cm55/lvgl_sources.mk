# lvgl_sources.mk — LVGL source integration for ModusToolbox
# Include this from the project Makefile when APP_DISPLAY_LVGL_ENABLE=1.
#
# The MTB build system doesn't support recursive SOURCES+= for external libs.
# This snippet uses $(shell find) to discover all LVGL .c files under src/
# and adds them as CSRCS.

LVGL_PATH ?= $(SEARCH_lvgl)

# Core LVGL sources needed for the health dashboard
LVGL_CSRCS := $(shell find $(LVGL_PATH)/src -type f -name '*.c')

# Filter out platform-specific drivers we don't need (saves compile time)
LVGL_CSRCS := $(filter-out %/src/draw/nxp/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/draw/renesas/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/draw/sdl/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/draw/vg_lite/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/libs/thorvg/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/drivers/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/libs/ffmpeg/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/libs/freetype/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/libs/gif/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/libs/libjpeg_turbo/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/libs/libpng/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/libs/lodepng/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/libs/rlottie/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/libs/tiny_ttf/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/libs/tjpgd/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/libs/barcode/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/libs/bin_decoder/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/libs/qrcode/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/libs/bmp/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/libs/fsdrv/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/others/vg_lite_tvg/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/stdlib/micropython/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/stdlib/rtthread/%,$(LVGL_CSRCS))

# Filter out unused widget types to reduce flash
LVGL_CSRCS := $(filter-out %/src/widgets/calendar/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/widgets/canvas/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/widgets/chart/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/widgets/checkbox/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/widgets/dropdown/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/widgets/image/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/widgets/imagebutton/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/widgets/keyboard/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/widgets/list/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/widgets/lottie/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/widgets/menu/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/widgets/msgbox/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/widgets/roller/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/widgets/slider/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/widgets/span/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/widgets/spinbox/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/widgets/spinner/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/widgets/switch/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/widgets/table/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/widgets/tabview/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/widgets/textarea/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/widgets/tileview/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/widgets/win/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/widgets/animimage/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/widgets/buttonmatrix/%,$(LVGL_CSRCS))

# Other unused features
LVGL_CSRCS := $(filter-out %/src/others/file_explorer/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/others/fragment/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/others/gridnav/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/others/ime/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/others/imgfont/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/others/monkey/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/others/sysmon/%,$(LVGL_CSRCS))

# These are the only themes we enable
LVGL_CSRCS := $(filter-out %/src/themes/mono/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/themes/simple/%,$(LVGL_CSRCS))

# Exclude Helium/NEON assembly — we use LV_DRAW_SW_ASM_NONE
LVGL_CSRCS := $(filter-out %/src/draw/sw/blend/helium/%,$(LVGL_CSRCS))
LVGL_CSRCS := $(filter-out %/src/draw/sw/blend/neon/%,$(LVGL_CSRCS))

# Add the concrete LVGL source files to SOURCES.
# Passing only directories here lets the MTB auto-discovery skip the external
# LVGL implementation units, which then leaves ui_health_dashboard.o with
# unresolved lv_* symbols at link time.
SOURCES += $(LVGL_CSRCS)
