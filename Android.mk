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

ifeq ($(TARGET_USES_XILINX_VCU),true)

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

OMX_SETTINGS_COMMON_SRCS := \
	base/omx_settings/omx_convert_module_soft.cpp \
	base/omx_settings/omx_convert_module_soft_avc.cpp \
	base/omx_settings/omx_convert_module_soft_hevc.cpp \
	base/omx_settings/omx_settings_common.cpp \
	base/omx_settings/omx_settings_common_avc.cpp \
	base/omx_settings/omx_settings_common_hevc.cpp \
	base/omx_settings/omx_settings_checks.cpp

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
	frameworks/native/include/media/openmax

LIB_OMX_CORE_SRC := \
	core/omx_core/omx_core.cpp

LOCAL_SRC_FILES := \
	$(LIB_OMX_CORE_SRC)

LOCAL_VENDOR_MODULE := true
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

OMX_SETTINGS_DEC_SRCS := \
	base/omx_settings/omx_settings_dec_avc.cpp \
	base/omx_settings/omx_settings_dec_hevc.cpp \
	base/omx_settings/omx_settings_dec_common.cpp \
	base/omx_settings/omx_convert_module_soft_dec.cpp

LOCAL_SRC_FILES := \
	$(OMX_CODEC_DEC_SRCS) \
	$(OMX_CHECKER_SRCS) \
	$(OMX_CODEC_COMMON_SRCS) \
	$(OMX_MODULE_COMMON_SRCS) \
	$(OMX_WRAPPER_COMMON_SRCS) \
	$(OMX_SETTINGS_COMMON_SRCS) \
	$(OMX_MEDIATYPE_DEC_SRCS) \
	$(OMX_MODULE_DEC_SRCS) \
	$(OMX_WRAPPER_DEC_SRCS) \
	$(OMX_SETTINGS_DEC_SRCS)

LOCAL_VENDOR_MODULE := true
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
	base/omx_module/omx_device_enc_hardware_mcu.cpp \
	base/omx_module/ROIMngr.cpp \
	base/omx_module/omx_convert_module_soft_roi.cpp

OMX_WRAPPER_ENC_SRCS := \
	base/omx_wrapper/omx_wrapper_enc.cpp \

OMX_SETTINGS_ENC_SRCS := \
        base/omx_settings/omx_settings_enc_avc.cpp \
        base/omx_settings/omx_settings_enc_hevc.cpp \
        base/omx_settings/omx_settings_enc_common.cpp \
        base/omx_settings/omx_convert_module_soft_enc.cpp

LOCAL_SRC_FILES := \
	$(OMX_CODEC_ENC_SRCS) \
	$(OMX_CHECKER_SRCS) \
	$(OMX_CODEC_COMMON_SRCS) \
	$(OMX_MODULE_COMMON_SRCS) \
	$(OMX_WRAPPER_COMMON_SRCS) \
	$(OMX_SETTINGS_COMMON_SRCS) \
	$(OMX_MEDIATYPE_ENC_SRCS) \
	$(OMX_MODULE_ENC_SRCS) \
	$(OMX_SETTINGS_ENC_SRCS) \
	$(OMX_WRAPPER_ENC_SRCS)

LOCAL_VENDOR_MODULE := true
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

LOCAL_VENDOR_MODULE := true
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

EXE_OMX_ENCODER_SRCS := \
	exe_omx/encoder/main.cpp \
	exe_omx/encoder/CommandsSender.cpp\
	exe_omx/encoder/EncCmdMngr.cpp\
	exe_omx/common/helpers.cpp \
	exe_omx/common/setters.cpp \
	exe_omx/common/getters.cpp

LOCAL_SRC_FILES := \
	$(EXE_OMX_ENCODER_SRCS)

LOCAL_VENDOR_MODULE := true
LOCAL_MODULE := omx_encoder
include $(BUILD_EXECUTABLE)

##################################
# libstagefrighthw
##################################

include $(CLEAR_VARS)

LOCAL_SRC_FILES := \
	libstagefrighthw/XilinxOMXPlugin.cpp

LOCAL_CFLAGS := $(PV_CFLAGS_MINUS_VISIBILITY)

LOCAL_C_INCLUDES:= \
	frameworks/native/include/media/openmax \
	frameworks/native/include/media/hardware

LOCAL_SHARED_LIBRARIES := \
	libbinder \
	libutils \
	libcutils \
	libdl \
	libui                   \

LOCAL_VENDOR_MODULE := true
LOCAL_MODULE := libstagefrighthw
include $(BUILD_SHARED_LIBRARY)

endif #TARGET_USES_XILINX_VCU
