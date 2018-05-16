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

# Common src files for all libs
OMX_CHECKER_SRCS := \
        base/omx_checker/omx_checker.cpp

OMX_CODEC_COMMON_SRCS := \
	base/omx_codec/omx_codec.cpp \
	base/omx_codec/omx_convert_omx_module.cpp

OMX_MODULE_COMMON_SRCS := \

OMX_WRAPPER_COMMON_SRCS := \
	base/omx_wrapper/omx_wrapper.cpp

##################################
# libOMX.allegro.core
##################################
include $(CLEAR_VARS)

LOCAL_SHARED_LIBRARIES := liblog libcutils

LOCAL_CFLAGS := \
	-DLOG_TAG=\"libOMX.allegro.core\" \
	-std=gnu99

LOCAL_CPPFLAGS += -fexceptions -std=c++11

LOCAL_C_INCLUDES := \
	frameworks/native/include/media/openmax \
	$(LOCAL_PATH)/omx_header_android

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
	-std=gnu99

LOCAL_CPPFLAGS += -fexceptions -std=c++11 -frtti

LOCAL_C_INCLUDES := \
	frameworks/native/include/media/openmax \
	hardware/xilinx/vcu/vcu-ctrl-sw/include \
	$(LOCAL_PATH)/omx_header_android

OMX_CODEC_DEC_SRCS := \
	base/omx_codec/omx_codec_dec.cpp \
	base/omx_codec/omx_expertise_dec_avc.cpp \
	base/omx_codec/omx_expertise_dec_hevc.cpp

OMX_MEDIATYPE_DEC_SRCS := \
	base/omx_mediatype/omx_mediatype_dec_avc.cpp \
	base/omx_mediatype/omx_mediatype_dec_hevc.cpp \

OMX_MODULE_DEC_SRCS := \
	base/omx_module/omx_module_dec.cpp \
	base/omx_module/omx_device_dec_hardware_mcu.cpp

OMX_WRAPPER_DEC_SRCS := \
	base/omx_wrapper/omx_wrapper_dec.cpp \

LOCAL_SRC_FILES := \
	$(OMX_CODEC_DEC_SRCS) \
	$(OMX_CHECKER_SRCS) \
	$(OMX_CODEC_COMMON_SRCS) \
	$(OMX_MODULE_COMMON_SRCS) \
	$(OMX_WRAPPER_COMMON_SRCS) \
	$(OMX_MEDIATYPE_DEC_SRCS) \
	$(OMX_MODULE_DEC_SRCS) \
	$(OMX_WRAPPER_DEC_SRCS)

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
	-std=gnu99

LOCAL_CPPFLAGS += -fexceptions -std=c++11 -frtti

LOCAL_C_INCLUDES := \
	frameworks/native/include/media/openmax \
	hardware/xilinx/vcu/vcu-ctrl-sw/include \
	$(LOCAL_PATH)/omx_header_android

OMX_CODEC_ENC_SRCS := \
	base/omx_codec/omx_codec_enc.cpp \
	base/omx_codec/omx_expertise_enc_avc.cpp \
	base/omx_codec/omx_expertise_enc_hevc.cpp

OMX_MEDIATYPE_ENC_SRCS := \
	base/omx_mediatype/omx_mediatype_enc_avc.cpp \
	base/omx_mediatype/omx_mediatype_enc_hevc.cpp \

OMX_MODULE_ENC_SRCS := \
	base/omx_module/omx_module_enc.cpp \
	base/omx_module/omx_device_enc_hardware_mcu.cpp

OMX_WRAPPER_ENC_SRCS := \
	base/omx_wrapper/omx_wrapper_enc.cpp \

LOCAL_SRC_FILES := \
	$(OMX_CODEC_ENC_SRCS) \
	$(OMX_CHECKER_SRCS) \
	$(OMX_CODEC_COMMON_SRCS) \
	$(OMX_MODULE_COMMON_SRCS) \
	$(OMX_WRAPPER_COMMON_SRCS) \
	$(OMX_MEDIATYPE_ENC_SRCS) \
	$(OMX_MODULE_ENC_SRCS) \
	$(OMX_WRAPPER_ENC_SRCS)

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
	-std=gnu99

LOCAL_CPPFLAGS += -fexceptions -std=c++11

LOCAL_C_INCLUDES := \
	frameworks/native/include/media/openmax \
	hardware/xilinx/vcu/vcu-ctrl-sw/include \
	$(LOCAL_PATH)/omx_header_android

EXE_OMX_DECODER_SRCS := \
	exe_omx/decoder/main.cpp \
	exe_omx/common/helpers.cpp \
	exe_omx/common/setters.cpp \
	exe_omx/common/getters.cpp

LOCAL_SRC_FILES := \
	$(EXE_OMX_DECODER_SRCS)

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
	-std=gnu99

LOCAL_CPPFLAGS += -fexceptions -std=c++11

LOCAL_C_INCLUDES := \
	frameworks/native/include/media/openmax \
	hardware/xilinx/vcu/vcu-ctrl-sw/include \
	$(LOCAL_PATH)/omx_header_android

EXE_OMX_ENCODER_SRCS:= \
	exe_omx/encoder/main.cpp \
	exe_omx/encoder/CommandsSender.cpp\
	exe_omx/encoder/EncCmdMngr.cpp\
	exe_omx/common/helpers.cpp \
	exe_omx/common/setters.cpp \
	exe_omx/common/getters.cpp

LOCAL_SRC_FILES := \
	$(EXE_TEST_ENCODE_OMX_SRC)

LOCAL_MODULE := omx_encoder
include $(BUILD_EXECUTABLE)
