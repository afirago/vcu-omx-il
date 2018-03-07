#
# Copyright (C) 2018 Mentor Graphics Inc.
# Copyright (C) 2018 Xilinx Inc.
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

OMX_COMPONENT_COMMON_SRCS := \
	base/omx_component/omx_component_interface.cpp \
	base/omx_component/omx_component.cpp \
	base/omx_component/omx_convert_omx_media.cpp \
	base/omx_component/omx_buffer_handle.cpp \
	base/omx_component/omx_component_getset.cpp \
	base/omx_component/omx_expertise_interface.cpp \
	base/omx_component/omx_expertise_avc.cpp \
	base/omx_component/omx_expertise_hevc.cpp

OMX_MODULE_CODEC_SRCS := \
	base/omx_module/convert_module_soft.cpp \
	base/omx_module/convert_module_soft_avc.cpp \
	base/omx_module/convert_module_soft_hevc.cpp \
	base/omx_module/mediatype_interface.cpp \
	base/omx_module/mediatype_checks.cpp \
	base/omx_module/mediatype_codec_itu.cpp \
	base/omx_module/mediatype_codec_avc.cpp \
	base/omx_module/mediatype_codec_hevc.cpp \
	base/omx_module/mediatype_dummy.cpp \
	base/omx_module/module_interface.cpp \
	base/omx_module/module_dummy.cpp \
	base/omx_module/buffer_handle_interface.cpp

OMX_WRAPPER_COMMON_SRCS := \
	base/omx_wrapper/omx_wrapper_common_entry_point.cpp

EXE_OMX_COMMON_SRCS := \
	exe_omx/common//getters.cpp \
	exe_omx/common//helpers.cpp

##################################
# libOMX.allegro.core
##################################
include $(CLEAR_VARS)

LOCAL_SHARED_LIBRARIES := liblog libcutils

LOCAL_CFLAGS := \
	-DLOG_TAG=\"libOMX.allegro.core\"

LOCAL_CPPFLAGS := \
	-fexceptions \
	-std=c++11

LOCAL_C_INCLUDES := \
	frameworks/native/include/media/openmax

LIB_OMX_CORE_SRC := \
	core/omx_core.cpp

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
	-DAL_USE_VCU \
	-DAL_USE_MCU

LOCAL_CPPFLAGS := \
	-std=c++11 \
	-fexceptions \
	-frtti

LOCAL_C_INCLUDES := \
	frameworks/native/include/media/openmax \
	hardware/xilinx/vcu/vcu-ctrl-sw/include \
	$(LOCAL_PATH)/omx_header_for_android

OMX_COMPONENT_DEC_SRCS := \
	base/omx_component/omx_component_dec.cpp

OMX_MODULE_DEC_SRCS := \
	base/omx_module/mediatype_dec_avc.cpp \
	base/omx_module/mediatype_dec_hevc.cpp \
	base/omx_module/mediatype_dec_itu.cpp \
	base/omx_module/convert_module_soft_dec.cpp \
	base/omx_module/module_dec.cpp \
	base/omx_module/device_dec_interface.cpp \
	base/omx_module/device_dec_hardware_mcu.cpp

OMX_WRAPPER_DEC_SRCS := \
	base/omx_wrapper/omx_wrapper_dec.cpp \
	base/omx_wrapper/omx_wrapper_dec_entry_point.cpp

LOCAL_SRC_FILES := \
	$(OMX_CHECKER_SRCS) \
	$(OMX_WRAPPER_COMMON_SRCS) \
	$(OMX_COMPONENT_COMMON_SRCS) \
	$(OMX_COMPONENT_DEC_SRCS) \
	$(OMX_MODULE_DEC_SRCS) \
	$(OMX_WRAPPER_DEC_SRCS) \
	$(OMX_MODULE_CODEC_SRCS) \

LOCAL_VENDOR_MODULE := true
LOCAL_MODULE := libOMX.allegro.video_decoder
include $(BUILD_SHARED_LIBRARY)

##################################
# libOMX.allegro.video_encoder
##################################
include $(CLEAR_VARS)

LOCAL_SHARED_LIBRARIES := liblog libcutils liballegro_encode

CONFIG_H := $(LOCAL_PATH)/../vcu-ctrl-sw/include/config.h

LOCAL_CFLAGS := \
	-include $(CONFIG_H) \
	-DLOG_TAG=\"libOMX.allegro.video_encoder\" \
	-std=gnu99 \
	-DAL_USE_VCU \
	-DAL_USE_MCU

LOCAL_CPPFLAGS := \
 	-std=c++11 \
 	-fexceptions \
 	-frtti

LOCAL_C_INCLUDES := \
	frameworks/native/include/media/openmax \
	hardware/xilinx/vcu/vcu-ctrl-sw/include \
	$(LOCAL_PATH)/omx_header_for_android

OMX_COMPONENT_ENC_SRCS := \
	base/omx_component/omx_component_enc.cpp

OMX_MODULE_ENC_SRCS := \
	base/omx_module/mediatype_enc_avc.cpp \
	base/omx_module/mediatype_enc_hevc.cpp \
	base/omx_module/mediatype_enc_itu.cpp \
	base/omx_module/convert_module_soft_enc.cpp \
	base/omx_module/module_enc.cpp \
	base/omx_module/memory_interface.cpp \
	base/omx_module/dma_memory.cpp \
	base/omx_module/cpp_memory.cpp \
	base/omx_module/device_enc_interface.cpp \
	base/omx_module/device_enc_hardware_mcu.cpp \
	base/omx_module/ROIMngr.cpp \
	base/omx_module/TwoPassMngr.cpp \
	base/omx_module/convert_module_soft_roi.cpp

OMX_WRAPPER_ENC_SRCS := \
	base/omx_wrapper/omx_wrapper_enc.cpp \
	base/omx_wrapper/omx_wrapper_enc_entry_point.cpp

LOCAL_SRC_FILES := \
	$(OMX_CHECKER_SRCS) \
	$(OMX_WRAPPER_COMMON_SRCS) \
	$(OMX_COMPONENT_COMMON_SRCS) \
	$(OMX_COMPONENT_ENC_SRCS) \
	$(OMX_MODULE_ENC_SRCS) \
	$(OMX_WRAPPER_ENC_SRCS) \
	$(OMX_MODULE_CODEC_SRCS)

LOCAL_VENDOR_MODULE := true
LOCAL_MODULE := libOMX.allegro.video_encoder
include $(BUILD_SHARED_LIBRARY)

##################################
# omx_decoder
##################################
include $(CLEAR_VARS)

LOCAL_SHARED_LIBRARIES := \
	liblog \
	libcutils \
	liballegro_decode \
	libOMX.allegro.core

CONFIG_H := $(LOCAL_PATH)/../vcu-ctrl-sw/include/config.h

LOCAL_CFLAGS := \
	-include $(CONFIG_H) \
	-DLOG_TAG=\"omx_decoder\" \
	-std=gnu99

LOCAL_CPPFLAGS := \
	-std=c++11 \
	-fexceptions

LOCAL_C_INCLUDES := \
	frameworks/native/include/media/openmax \
	hardware/xilinx/vcu/vcu-ctrl-sw/include \
	$(LOCAL_PATH)/omx_header_for_android

EXE_OMX_DECODER_SRCS := \
	exe_omx/decoder/main.cpp

LOCAL_SRC_FILES := \
	$(EXE_OMX_COMMON_SRCS) \
	$(EXE_OMX_DECODER_SRCS)

LOCAL_VENDOR_MODULE := true
LOCAL_MODULE := omx_decoder
include $(BUILD_EXECUTABLE)

##################################
# omx_encoder
##################################
include $(CLEAR_VARS)

LOCAL_SHARED_LIBRARIES := \
	liblog \
	libcutils \
	liballegro_encode \
	libOMX.allegro.core

CONFIG_H := $(LOCAL_PATH)/../vcu-ctrl-sw/include/config.h

LOCAL_CFLAGS := \
	-include $(CONFIG_H) \
	-DLOG_TAG=\"omx_encoder\" \
	-std=gnu99

LOCAL_CPPFLAGS := \
	-fexceptions \
	-std=c++11

LOCAL_C_INCLUDES := \
	frameworks/native/include/media/openmax \
	hardware/xilinx/vcu/vcu-ctrl-sw/include \
	$(LOCAL_PATH)/omx_header_for_android

EXE_OMX_ENCODER_SRCS := \
	exe_omx/encoder/main.cpp \
	exe_omx/encoder/CommandsSender.cpp \
	exe_omx/encoder/EncCmdMngr.cpp \

LOCAL_SRC_FILES := \
	$(EXE_OMX_COMMON_SRCS) \
	$(EXE_OMX_ENCODER_SRCS)

LOCAL_MODULE := omx_encoder
LOCAL_VENDOR_MODULE := true
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

LOCAL_MODULE := libstagefrighthw
include $(BUILD_SHARED_LIBRARY)

endif #TARGET_USES_XILINX_VCU
