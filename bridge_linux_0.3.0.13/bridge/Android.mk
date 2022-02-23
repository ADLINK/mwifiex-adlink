LOCAL_PATH := $(my-dir)
include $(CLEAR_VARS)

LOCAL_MODULE := bridge-android
LOCAL_SRC_FILES := ../drvwrapper/drv_wrapper.c mfgbridge.c mfgdebug.c
#LOCAL_MODULE_TAGS := eng development
DRV_DIR=../drvwrapper
LOCAL_CFLAGS += -Wno-date-time -Wno-parentheses -Wno-format -Wno-fortify-source -Wno-format-security -Wno-missing-field-initializers -Wno-missing-braces -Wno-missing-field-initializers -w -Wno-unused-parameter -Wno-writable-strings -Wno-pragma-pack -Wno-macro-redefined -Wno-pragma-pack-suspicious-include -Wno-pedantic -Wno-return-type -Wno-error -DMFG_UPDATE -I. -I$(DRV_DIR) -DNONPLUG_SUPPORT -DRAWUR_BT_STACK -D_ANDROID_ -DSPI_SUPPORT -D_SPINEL_SUPPORT_
LOCAL_C_INCLUDES := $(INCLUDES)
#LOCAL_LDLIBS += -lpthread
include $(BUILD_EXECUTABLE)

