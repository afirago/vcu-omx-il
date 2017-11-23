#
# Copyright (C) 2017 Mentor Graphics Inc.
# Copyright (C) 2017 Xilinx Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

LOCAL_PATH := $(call my-dir)

##################################
# libOMX.allegro.core
##################################
include $(CLEAR_VARS)

LOCAL_SHARED_LIBRARIES := liblog libcutils

LOCAL_CFLAGS := \
	-DLOG_TAG=\"libOMX.allegro.core\" \
	-DAL_USE_VCU \
	-DAL_USE_MCU \
	-std=gnu99

LOCAL_CPPFLAGS += -fexceptions -std=c++11

LOCAL_C_INCLUDES := \
	$(LOCAL_PATH)/omx_header \

LIB_OMX_CORE_SRC := \
	core/omx_core/omx_core.cpp

LOCAL_SRC_FILES := \
	$(LIB_OMX_CORE_SRC)

LOCAL_MODULE := libOMX.allegro.core
include $(BUILD_SHARED_LIBRARY)

##################################
# libOMX.allegro.video_decoder
##################################
include $(CLEAR_VARS)

LOCAL_SHARED_LIBRARIES := liblog libcutils liballegro_decode

CONFIG_H := $(LOCAL_PATH)/../vcu-ctrl-sw/include/config.h

LOCAL_CFLAGS := \
	-include $(CONFIG_H) \
	-DLOG_TAG=\"libOMX.allegro.video_decoder\" \
	-DAL_USE_VCU \
	-DAL_USE_MCU \
	-std=gnu99

LOCAL_CPPFLAGS += -fexceptions -std=c++11

LOCAL_C_INCLUDES := \
	$(LOCAL_PATH)/omx_header \
	hardware/xilinx/vcu/vcu-ctrl-sw/include

LIB_OMX_DECODE_SRC := \
	base/omx_port/omx_port.cpp \
	base/omx_port/omx_buffer_manager.cpp \
	base/omx_wrapper/omx_wrapper.cpp \
	base/omx_wrapper/omx_dec_wrapper.cpp \
	base/omx_checker/omx_checker.cpp \
	base/omx_videocodec/omx_videocodec.cpp \
	base/omx_processtype/omx_process.cpp \
	base/omx_processtype/omx_dec_process.cpp \
	base/omx_processtype/omx_dec_ipdevice.cpp \
	base/omx_buffer/omx_buffer.cpp \
	base/omx_buffer/omx_buffer_meta.cpp \
	base/omx_codectype/omx_hevc_codec.cpp \
	base/omx_codectype/omx_avc_codec.cpp

LOCAL_SRC_FILES := \
	$(LIB_OMX_DECODE_SRC)

LOCAL_MODULE := libOMX.allegro.video_decoder
include $(BUILD_SHARED_LIBRARY)

##################################
# libOMX.allegro.video_encoder
##################################
include $(CLEAR_VARS)

LOCAL_SHARED_LIBRARIES := liblog libcutils liballegro_encode liballegro_decode

CONFIG_H := $(LOCAL_PATH)/../vcu-ctrl-sw/include/config.h

LOCAL_CFLAGS := \
	-include $(CONFIG_H) \
	-DLOG_TAG=\"libOMX.allegro.video_encoder\" \
	-DAL_USE_VCU \
	-DAL_USE_MCU \
	-std=gnu99

LOCAL_CPPFLAGS += -fexceptions -std=c++11

LOCAL_C_INCLUDES := \
	$(LOCAL_PATH)/omx_header \
	hardware/xilinx/vcu/vcu-ctrl-sw/include

LIB_OMX_ENCODE_SRC := \
	base/omx_port/omx_port.cpp \
	base/omx_port/omx_buffer_manager.cpp \
	base/omx_wrapper/omx_wrapper.cpp \
	base/omx_wrapper/omx_enc_wrapper.cpp \
	base/omx_checker/omx_checker.cpp \
	base/omx_videocodec/omx_videocodec.cpp \
	base/omx_processtype/omx_process.cpp \
	base/omx_processtype/omx_enc_process.cpp \
	base/omx_processtype/omx_enc_ipdevice.cpp \
	base/omx_buffer/omx_buffer.cpp \
	base/omx_buffer/omx_buffer_meta.cpp \
	base/omx_codectype/omx_hevc_codec.cpp \
	base/omx_codectype/omx_avc_codec.cpp \
	base/omx_codectype/omx_enc_param.cpp

LOCAL_SRC_FILES := \
	$(LIB_OMX_ENCODE_SRC)

LOCAL_MODULE := libOMX.allegro.video_encoder
include $(BUILD_SHARED_LIBRARY)

##################################
# omx_decoder
##################################
include $(CLEAR_VARS)

LOCAL_SHARED_LIBRARIES := liblog libcutils liballegro_decode libOMX.allegro.core

CONFIG_H := $(LOCAL_PATH)/../vcu-ctrl-sw/include/config.h

LOCAL_CFLAGS := \
	-include $(CONFIG_H) \
	-DLOG_TAG=\"omx_decoder\" \
	-DAL_USE_VCU \
	-DAL_USE_MCU \
	-std=gnu99

LOCAL_CPPFLAGS += -fexceptions -std=c++11

LOCAL_C_INCLUDES := \
	$(LOCAL_PATH)/omx_header \
	hardware/xilinx/vcu/vcu-ctrl-sw/include

EXE_TEST_DECODE_OMX_SRC := \
	exe_omx/decoder/main.cpp \
	exe_omx/common/helpers.cpp \
	exe_omx/common/setters.cpp

LOCAL_SRC_FILES := \
	$(EXE_TEST_DECODE_OMX_SRC)

LOCAL_MODULE := omx_decoder
include $(BUILD_EXECUTABLE)

##################################
# omx_encoder
##################################
include $(CLEAR_VARS)

LOCAL_SHARED_LIBRARIES := liblog libcutils liballegro_encode libOMX.allegro.core

CONFIG_H := $(LOCAL_PATH)/../vcu-ctrl-sw/include/config.h

LOCAL_CFLAGS := \
	-include $(CONFIG_H) \
	-DLOG_TAG=\"omx_encoder\" \
	-DAL_USE_VCU \
	-DAL_USE_MCU \
	-std=gnu99

LOCAL_CPPFLAGS += -fexceptions -std=c++11

LOCAL_C_INCLUDES := \
	$(LOCAL_PATH)/omx_header \
	hardware/xilinx/vcu/vcu-ctrl-sw/include

EXE_TEST_ENCODE_OMX_SRC:= \
	exe_omx/encoder/main.cpp \
	exe_omx/common/helpers.cpp \
	exe_omx/common/setters.cpp \
	exe_omx/common/getters.cpp

LOCAL_SRC_FILES := \
	$(EXE_TEST_ENCODE_OMX_SRC)

LOCAL_MODULE := omx_encoder
include $(BUILD_EXECUTABLE)
