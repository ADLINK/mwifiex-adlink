LOCAL_PATH := $(my-dir)
include $(CLEAR_VARS)

LOCAL_MODULE := mfgbridge
LOCAL_SRC_FILES := mfgbridge.c mfgdebug.c ../drvwrapper/drv_wrapper.c
LOCAL_MODULE_TAGS := eng development
LOCAL_CFLAGS += -DNONPLUG_SUPPORT -DMARVELL_BT_STACK -DMFG_UPDATE
include $(BUILD_EXECUTABLE)

