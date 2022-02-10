ifeq ($(strip $(SUPPORT_GNSS_HARDWARE)), true)
LOCAL_PATH := $(call my-dir)

#$(warning shell echo "it should $(TARGET_BUILD_VARIANT)")


GNSS_LCS_FLAG := TRUE

include $(CLEAR_VARS)
LOCAL_SRC_FILES := \
	common/src/gps_hal.c


LOCAL_C_INCLUDES := \
	$(LOCAL_PATH)/common/inc \

LOCAL_SHARED_LIBRARIES := \
	libc \
	libutils \
	libcutils \
	libhardware \
	liblog \
	libpower \
	libm   \
	libdl \
	libandroid_net \
	libbase

LOCAL_CFLAGS +=  -Wall -Wno-missing-field-initializers  -Wunreachable-code -Wpointer-arith -Wshadow 
LOCAL_PRELINK_MODULE := false
LOCAL_MODULE_RELATIVE_PATH := hw
LOCAL_MODULE := gps.default
LOCAL_MODULE_TAGS := optional
LOCAL_PROPRIETARY_MODULE := true
#$(warning shell echo "it build end libgps")
include $(BUILD_SHARED_LIBRARY)

endif
