LOCAL_PATH := $(my-dir)
include $(CLEAR_VARS)

INCLUDES = external/bluetooth/bluez/include

LOCAL_MODULE := mfgbridge
LOCAL_SRC_FILES := mfgbridge.c mfgdebug.c ../drvwrapper/drv_wrapper.c
LOCAL_MODULE_TAGS := eng development
LOCAL_SHARED_LIBRARIES := libbluetooth 
LOCAL_CFLAGS += -DNONPLUG_SUPPORT -DMFG_UPDATE -DWEXT
LOCAL_C_INCLUDES := $(INCLUDES)
include $(BUILD_EXECUTABLE)


