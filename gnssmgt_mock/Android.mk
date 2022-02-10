LOCAL_PATH := $(call my-dir)
#$(warning shell echo "it build gnssmgtmock")

include $(CLEAR_VARS)
LOCAL_SRC_FILES := \
	common/src/gps.c \


LOCAL_C_INCLUDES := \
	$(LOCAL_PATH)/common/inc \
	hardware/libhardware_legacy \
	hardware/libhardware_legacy/include/hardware_legacy \
	hardware/libhardware/include/hardware \
	external/openssl/include  \
	frameworks/native/include

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
LOCAL_CFLAGS +=  -Wno-unused-parameter  -Wno-date-time -Wno-enum-conversion -Wno-unused-function
LOCAL_PRELINK_MODULE := false
#LOCAL_MODULE_RELATIVE_PATH := libgnssmgtmock
LOCAL_MODULE := libgnssmgtmock
LOCAL_MODULE_TAGS := optional
LOCAL_PROPRIETARY_MODULE := true
#$(warning shell echo "it build end gnssmgtmock")
include $(BUILD_SHARED_LIBRARY)


