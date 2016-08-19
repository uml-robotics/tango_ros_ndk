LOCAL_PATH := $(call my-dir)
PROJECT_ROOT:= $(call my-dir)/../../../../..

include $(CLEAR_VARS)
LOCAL_MODULE := tango_client_api
LOCAL_SRC_FILES := $(PROJECT_ROOT)/EXTRACT_TANGO_CLIENT_LIBRARY_HERE/libtango_client_api/armeabi-v7a/libtango_client_api.so
LOCAL_EXPORT_C_INCLUDES += $(PROJECT_ROOT)/EXTRACT_TANGO_CLIENT_LIBRARY_HERE/libtango_client_api/armeabi-v7a
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_DISABLE_FATAL_LINKER_WARNINGS=true
LOCAL_MODULE    := libtango_native_streaming
LOCAL_SHARED_LIBRARIES := tango_client_api tango_support_api
LOCAL_CFLAGS    := -std=c++11

LOCAL_SRC_FILES := jni_interface.cpp \
                   tango_native_streaming.cpp

LOCAL_LDLIBS    := -llog -L$(SYSROOT)/usr/lib
LOCAL_STATIC_LIBRARIES := roscpp_android_ndk
include $(BUILD_SHARED_LIBRARY)

$(call import-add-path, $(PROJECT_ROOT))
$(call import-module,tango_support_api)
$(call import-module,roscpp_android_ndk)
