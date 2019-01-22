/******************************************************************************
*
* Copyright (C) 2018 Allegro DVT2.  All rights reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* Use of the Software is limited solely to applications:
* (a) running on a Xilinx device, or
* (b) that interact with a Xilinx device through a bus or interconnect.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
* XILINX OR ALLEGRO DVT2 BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
* OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* Except as contained in this notice, the name of  Xilinx shall not be used
* in advertising or otherwise to promote the sale, use or other dealings in
* this Software without prior written authorization from Xilinx.
*
*
* Except as contained in this notice, the name of Allegro DVT2 shall not be used
* in advertising or otherwise to promote the sale, use or other dealings in
* this Software without prior written authorization from Allegro DVT2.
*
******************************************************************************/

#include "omx_component_dec.h"

#include <OMX_VideoExt.h>
#include <OMX_ComponentAlg.h>
#include <OMX_IVCommonAlg.h>

#include <cmath>

#include "base/omx_checker/omx_checker.h"
#include "base/omx_utils/omx_log.h"
#include "base/omx_utils/omx_translate.h"

#include "omx_component_getset.h"

#ifdef ANDROID
#include <media/hardware/HardwareAPI.h>
#include <system/graphics.h>
#include <nativebase/nativebase.h>
#include <hardware/gralloc.h>
#include <gralloc_priv.h>
#include <sys/mman.h>
#endif

using namespace std;

static DecModule& ToDecModule(ModuleInterface& module)
{
  return dynamic_cast<DecModule &>(module);
}

DecComponent::DecComponent(OMX_HANDLETYPE component, shared_ptr<MediatypeInterface> media, std::unique_ptr<DecModule>&& module, OMX_STRING name, OMX_STRING role, std::unique_ptr<Expertise>&& expertise) :
  Component(component, media, std::move(module), std::move(expertise), name, role)
{
}

DecComponent::~DecComponent() = default;

void DecComponent::EmptyThisBufferCallBack(BufferHandleInterface* handle)
{
  auto header = (OMX_BUFFERHEADERTYPE*)((OMXBufferHandle*)handle)->header;
  delete handle;
  ClearPropagatedData(header);

  if(callbacks.EmptyBufferDone)
    callbacks.EmptyBufferDone(component, app, header);
}

void DecComponent::AssociateCallBack(BufferHandleInterface*, BufferHandleInterface* fill)
{
  std::lock_guard<std::mutex> lock(mutex);

  if(transmit.empty())
    return;

  auto emptyHeader = transmit.front();
  auto fillHeader = (OMX_BUFFERHEADERTYPE*)((OMXBufferHandle*)fill)->header;
  assert(fillHeader);
  fillHeader->hMarkTargetComponent = emptyHeader.hMarkTargetComponent;
  fillHeader->pMarkData = emptyHeader.pMarkData;
  fillHeader->nTimeStamp = emptyHeader.nTimeStamp;
  transmit.pop_front();

  if(IsEOSDetected(emptyHeader.nFlags))
  {
    callbacks.EventHandler(component, app, OMX_EventBufferFlag, output.index, emptyHeader.nFlags, nullptr);
    transmit.clear();
  }

  if(IsCompMarked(emptyHeader.hMarkTargetComponent, component))
    callbacks.EventHandler(component, app, OMX_EventMark, 0, 0, emptyHeader.pMarkData);
}

void DecComponent::FillThisBufferCallBack(BufferHandleInterface* filled, int offset, int size)
{
  assert(filled);
  auto header = (OMX_BUFFERHEADERTYPE*)((OMXBufferHandle*)filled)->header;
  delete filled;

  header->nOffset = offset;
  header->nFilledLen = size;
  switch(ToDecModule(*module).GetDisplayPictureType())
  {
  case 0:
  {
    // Do nothing : PROGRESSIVE
    break;
  }
  case 1:
  {
    header->nFlags |= OMX_ALG_BUFFERFLAG_TOP_FIELD;
    break;
  }
  case 2:
  {
    header->nFlags |= OMX_ALG_BUFFERFLAG_BOT_FIELD;
    break;
  }
  default: break;
  }

  if(offset == 0 && size == 0)
    header->nFlags = OMX_BUFFERFLAG_EOS;

  if(callbacks.FillBufferDone)
    callbacks.FillBufferDone(component, app, header);
}

void DecComponent::EventCallBack(CallbackEventType type, void* data)
{
  assert(type <= CALLBACK_EVENT_MAX);
  switch(type)
  {
  case CALLBACK_EVENT_RESOLUTION_CHANGE:
  {
    LOGI("%s", ToStringCallbackEvent.at(type));

    auto port = GetPort(1);

    if(port->isTransientToEnable || !port->enable)
      callbacks.EventHandler(component, app, OMX_EventPortSettingsChanged, 1, 0, nullptr);
    break;
  }
  default:
    Component::EventCallBack(type, data);
    break;
  }
}

OMX_ERRORTYPE DecComponent::GetExtensionIndex(OMX_IN OMX_STRING name, OMX_OUT OMX_INDEXTYPE* index)
{
  OMX_TRY();
  OMXChecker::CheckNotNull(name);
  OMXChecker::CheckNotNull(index);
  OMXChecker::CheckStateOperation(AL_GetExtensionIndex, state);
  if (!strcmp(name, "OMX.google.android.index.enableAndroidNativeBuffers"))
  {
    *index = static_cast<OMX_INDEXTYPE>(OMX_ALG_IndexExtEnableNativeBuffer);
    return OMX_ErrorNone;
  }

  if (!strcmp(name, "OMX.google.android.index.getAndroidNativeBufferUsage"))
  {
    *index = static_cast<OMX_INDEXTYPE>(OMX_ALG_IndexExtGetNativeBufferUsage);
    return OMX_ErrorNone;
  }

  if (!strcmp(name, "OMX.google.android.index.useAndroidNativeBuffer"))
  {
    *index = static_cast<OMX_INDEXTYPE>(OMX_ALG_IndexExtUseNativeBuffer);
    return OMX_ErrorNone;
  }
  return OMX_ErrorNoMore;
  OMX_CATCH();
}

static OMX_BUFFERHEADERTYPE* AllocateHeader(OMX_PTR app, int size, OMX_U8* buffer, bool isBufferAllocatedByModule, int index)
{
  auto header = new OMX_BUFFERHEADERTYPE;
  OMXChecker::SetHeaderVersion(*header);
  header->pBuffer = buffer;
  header->nAllocLen = size;
  header->pAppPrivate = app;
  header->pInputPortPrivate = new bool(isBufferAllocatedByModule);
  header->pOutputPortPrivate = new bool(isBufferAllocatedByModule);
  auto& p = IsInputPort(index) ? header->nInputPortIndex : header->nOutputPortIndex;
  p = index;

  return header;
}

static inline bool isBufferAllocatedByModule(OMX_BUFFERHEADERTYPE const* header)
{
  if(!header->pInputPortPrivate || !header->pOutputPortPrivate)
    return false;

  auto isInputAllocated = *(static_cast<bool*>(header->pInputPortPrivate));
  auto isOutputAllocated = *(static_cast<bool*>(header->pOutputPortPrivate));

  return isInputAllocated || isOutputAllocated;
}

static void DeleteHeader(OMX_BUFFERHEADERTYPE* header)
{
  delete static_cast<bool*>(header->pInputPortPrivate);
  delete static_cast<bool*>(header->pOutputPortPrivate);
  delete header;
}

OMX_ERRORTYPE DecComponent::AllocateBuffer(OMX_INOUT OMX_BUFFERHEADERTYPE** header, OMX_IN OMX_U32 index, OMX_IN OMX_PTR app, OMX_IN OMX_U32 size)
{
  OMX_TRY();
  OMXChecker::CheckNotNull(header);
  OMXChecker::CheckNotNull(size);
  CheckPortIndex(index);

  auto port = GetPort(index);

  if(transientState != TransientLoadedToIdle && !(port->isTransientToEnable))
    throw OMX_ErrorIncorrectStateOperation;

  BufferHandles handles;
  media->Get(SETTINGS_INDEX_BUFFER_HANDLES, &handles);
  auto bufferHandlePort = IsInputPort(index) ? handles.input : handles.output;
  bool dmaOnPort = (bufferHandlePort == BufferHandleType::BUFFER_HANDLE_FD);
  auto buffer = dmaOnPort ? reinterpret_cast<OMX_U8*>(ToDecModule(*module).AllocateDMA(size * sizeof(OMX_U8))) : static_cast<OMX_U8*>(module->Allocate(size * sizeof(OMX_U8)));

  if(dmaOnPort ? (static_cast<int>((intptr_t)buffer) < 0) : !buffer)
    throw OMX_ErrorInsufficientResources;

  *header = AllocateHeader(app, size, buffer, true, index);
  assert(*header);
  port->Add(*header);

  return OMX_ErrorNone;
  OMX_CATCH_L([&](OMX_ERRORTYPE& e)
  {
    if(e != OMX_ErrorBadPortIndex)
      GetPort(index)->ErrorOccured();
  });
}

OMX_ERRORTYPE DecComponent::FreeBuffer(OMX_IN OMX_U32 index, OMX_IN OMX_BUFFERHEADERTYPE* header)
{
  OMX_TRY();
  OMXChecker::CheckNotNull(header);

  CheckPortIndex(index);
  auto port = GetPort(index);

  if((transientState != TransientIdleToLoaded) && (!port->isTransientToDisable))
    callbacks.EventHandler(component, app, OMX_EventError, OMX_ErrorPortUnpopulated, 0, nullptr);

  BufferHandles handles;
  media->Get(SETTINGS_INDEX_BUFFER_HANDLES, &handles);
  auto bufferHandlePort = IsInputPort(index) ? handles.input : handles.output;
  bool dmaOnPort = (bufferHandlePort == BufferHandleType::BUFFER_HANDLE_FD);
  dmaOnPort ? ToDecModule(*module).FreeDMA(static_cast<int>((intptr_t)header->pBuffer)) : module->Free(header->pBuffer);

  port->Remove(header);
  DeleteHeader(header);

  return OMX_ErrorNone;
  OMX_CATCH_L([&](OMX_ERRORTYPE& e)
  {
    if(e != OMX_ErrorBadPortIndex)
      GetPort(index)->ErrorOccured();
  });
}

void DecComponent::TreatEmptyBufferCommand(Task* task)
{
  std::lock_guard<std::mutex> lock(mutex);
  assert(task);
  assert(task->cmd == EmptyBuffer);
  assert(static_cast<int>((intptr_t)task->data) == input.index);
  auto header = static_cast<OMX_BUFFERHEADERTYPE*>(task->opt.get());
  assert(header);

  if(state == OMX_StateInvalid)
  {
    callbacks.EmptyBufferDone(component, app, header);
    return;
  }

  AttachMark(header);

  if(header->nFlags & OMX_BUFFERFLAG_ENDOFFRAME)
    transmit.push_back(PropagatedData(header->hMarkTargetComponent, header->pMarkData, header->nTimeStamp, header->nFlags));

  auto handle = new OMXBufferHandle(header);
  auto success = module->Empty(handle);
  assert(success);
}

OMX_ERRORTYPE DecComponent::DecSetVideoPortFormat(OMX_VIDEO_PARAM_PORTFORMATTYPE const& format, Port const& port, shared_ptr<MediatypeInterface> media)
{
  OMX_VIDEO_PARAM_PORTFORMATTYPE rollback;
  ConstructVideoPortCurrentFormat(rollback, port, media);

  auto ret = SetClock(format.xFramerate, media);

  if(ret != OMX_ErrorNone)
  {
    DecSetVideoPortFormat(rollback, port, media);
    throw ret;
  }

  if (!IsInputPort(port.index))
  {
    ret = SetFormat(format.eColorFormat, media);

    if(ret != OMX_ErrorNone)
    {
      DecSetVideoPortFormat(rollback, port, media);
      throw ret;
    }
  }
  return OMX_ErrorNone;
}

OMX_ERRORTYPE DecComponent::DecSetPortDefinition(OMX_PARAM_PORTDEFINITIONTYPE const& settings, Port& port, ModuleInterface& module, shared_ptr<MediatypeInterface> media)
{
  OMX_PARAM_PORTDEFINITIONTYPE rollback;
  ConstructPortDefinition(rollback, port, module, media);
  auto video = settings.format.video;
  OMX_ERRORTYPE ret;

  if (!IsInputPort(port.index))
  {
    ret = SetFormat(video.eColorFormat, media);

    if(ret != OMX_ErrorNone)
    {
      DecSetPortDefinition(rollback, port, module, media);
      throw ret;
    }
  }

  ret = SetClock(video.xFramerate, media);

  if(ret != OMX_ErrorNone)
  {
    DecSetPortDefinition(rollback, port, module, media);
    throw ret;
  }

  ret = SetResolution(video, media);

  if(ret != OMX_ErrorNone)
  {
    DecSetPortDefinition(rollback, port, module, media);
    throw ret;
  }

  // Set Target is only used for encoder, ignored for decoder
  ret = SetTargetBitrate(video.nBitrate, media);

  if(ret != OMX_ErrorNone && ret != OMX_ErrorUnsupportedIndex)
  {
    DecSetPortDefinition(rollback, port, module, media);
    throw ret;
  }

  return OMX_ErrorNone;
}


OMX_ERRORTYPE DecComponent::GetParameter(OMX_IN OMX_INDEXTYPE index, OMX_INOUT OMX_PTR param)
{
  OMX_TRY();
  OMXChecker::CheckNotNull(param);
  OMXChecker::CheckHeaderVersion(GetVersion(param));
  OMXChecker::CheckStateOperation(AL_GetParameter, state);

  auto getCurrentPort = [=](OMX_PTR param) -> Port*
                        {
                          auto index = *(((OMX_U32*)param) + 2);
                          return GetPort(index);
                        };
  switch(static_cast<OMX_U32>(index)) // all indexes are 32u
  {
  case OMX_IndexParamVideoInit:
  {
    *(OMX_PORT_PARAM_TYPE*)param = videoPortParams;
    return OMX_ErrorNone;
  }
  case OMX_IndexParamStandardComponentRole:
  {
    auto p = (OMX_PARAM_COMPONENTROLETYPE*)param;
    strncpy((char*)p->cRole, (char*)role, OMX_MAX_STRINGNAME_SIZE);
    return OMX_ErrorNone;
  }
  case OMX_IndexParamPortDefinition:
  {
    auto port = getCurrentPort(param);
    auto def = static_cast<OMX_PARAM_PORTDEFINITIONTYPE*>(param);
    OMX_ERRORTYPE err = ConstructPortDefinition(*def, *port, *module, media);
    if (err)
      return err;
#ifdef ANDROID
    OMX_ALG_PORT_PARAM_BUFFER_MODE buf_mode;
    err = ConstructPortBufferMode(buf_mode, *port, media);
    if (err)
      return err;
    if (buf_mode.eMode == OMX_ALG_BUF_DMA)
    {
      if (def->format.video.eColorFormat == OMX_COLOR_FormatYUV420SemiPlanar)
        def->format.video.eColorFormat = (OMX_COLOR_FORMATTYPE)HAL_PIXEL_FORMAT_YCbCr_420_888;
    }
#endif
    return OMX_ErrorNone;
  }
  case OMX_IndexParamCompBufferSupplier:
  {
    auto port = getCurrentPort(param);
    auto s = static_cast<OMX_PARAM_BUFFERSUPPLIERTYPE*>(param);
    return ConstructPortSupplier(*s, *port);
  }
  case OMX_IndexParamVideoPortFormat:
  {
    auto p = (OMX_VIDEO_PARAM_PORTFORMATTYPE*)param;
    auto port = getCurrentPort(param);
    OMX_ERRORTYPE err = GetVideoPortFormatSupported(*p, media);
    if (err)
      return err;
#ifdef ANDROID
    OMX_ALG_PORT_PARAM_BUFFER_MODE buf_mode;
    err = ConstructPortBufferMode(buf_mode, *port, media);
    if (err)
      return err;
    if (buf_mode.eMode == OMX_ALG_BUF_DMA)
    {
      if (p->eColorFormat == OMX_COLOR_FormatYUV420SemiPlanar)
        p->eColorFormat = (OMX_COLOR_FORMATTYPE)HAL_PIXEL_FORMAT_YCbCr_420_888;
    }    
#endif
    if (IsInputPort(port->index))
      p->eColorFormat = OMX_COLOR_FormatUnused;
    return OMX_ErrorNone;

  }
  case OMX_IndexParamVideoProfileLevelCurrent:
  {
    auto port = getCurrentPort(param);
    return expertise->GetProfileLevel(param, *port, media);
  }
  case OMX_IndexParamVideoProfileLevelQuerySupported: // GetParameter only
  {
    return expertise->GetProfileLevelSupported(param, media);
  }
  case OMX_IndexParamVideoAvc:
  case OMX_ALG_IndexParamVideoHevc:
  {
    auto port = getCurrentPort(param);
    return expertise->GetExpertise(param, *port, media);
  }
  case OMX_ALG_IndexParamReportedLatency: // GetParameter only
  {
    auto lat = static_cast<OMX_ALG_PARAM_REPORTED_LATENCY*>(param);
    return ConstructReportedLatency(*lat, media);
  }
  case OMX_ALG_IndexPortParamBufferMode:
  {
    auto port = getCurrentPort(param);
    auto mode = static_cast<OMX_ALG_PORT_PARAM_BUFFER_MODE*>(param);
    return ConstructPortBufferMode(*mode, *port, media);
  }
  case OMX_ALG_IndexParamVideoSubframe:
  {
    auto port = getCurrentPort(param);
    auto subframe = static_cast<OMX_ALG_VIDEO_PARAM_SUBFRAME*>(param);
    return ConstructVideoSubframe(*subframe, *port, media);
  }
  // only encoder
  case OMX_IndexParamVideoQuantization:
  {
    auto port = getCurrentPort(param);
    auto q = static_cast<OMX_VIDEO_PARAM_QUANTIZATIONTYPE*>(param);
    return ConstructVideoQuantization(*q, *port, media);
  }
  case OMX_IndexParamVideoBitrate:
  {
    auto port = getCurrentPort(param);
    auto b = static_cast<OMX_VIDEO_PARAM_BITRATETYPE*>(param);
    return ConstructVideoBitrate(*b, *port, media);
  }
  case OMX_ALG_IndexParamVideoInterlaceFormatSupported: // GetParameter only
  {
    auto port = getCurrentPort(param);
    auto interlace = static_cast<OMX_INTERLACEFORMATTYPE*>(param);
    return ConstructVideoModesSupported(*interlace, *port, media);
  }
  case OMX_ALG_IndexParamVideoInterlaceFormatCurrent:
  {
    auto port = getCurrentPort(param);
    auto interlace = static_cast<OMX_INTERLACEFORMATTYPE*>(param);
    return ConstructVideoModeCurrent(*interlace, *port, media);
  }
  case OMX_ALG_IndexParamVideoQuantizationControl:
  {
    auto port = getCurrentPort(param);
    auto q = static_cast<OMX_ALG_VIDEO_PARAM_QUANTIZATION_CONTROL*>(param);
    return ConstructVideoQuantizationControl(*q, *port, media);
  }
  case OMX_ALG_IndexParamVideoQuantizationExtension:
  {
    auto port = getCurrentPort(param);
    auto q = static_cast<OMX_ALG_VIDEO_PARAM_QUANTIZATION_EXTENSION*>(param);
    return ConstructVideoQuantizationExtension(*q, *port, media);
  }
  case OMX_ALG_IndexParamVideoAspectRatio:
  {
    auto port = getCurrentPort(param);
    auto a = static_cast<OMX_ALG_VIDEO_PARAM_ASPECT_RATIO*>(param);
    return ConstructVideoAspectRatio(*a, *port, media);
  }
  case OMX_ALG_IndexParamVideoMaxBitrate:
  {
    auto port = getCurrentPort(param);
    auto b = static_cast<OMX_ALG_VIDEO_PARAM_MAX_BITRATE*>(param);
    return ConstructVideoMaxBitrate(*b, *port, media);
  }
  case OMX_ALG_IndexParamVideoLowBandwidth:
  {
    auto port = getCurrentPort(param);
    auto bw = static_cast<OMX_ALG_VIDEO_PARAM_LOW_BANDWIDTH*>(param);
    return ConstructVideoLowBandwidth(*bw, *port, media);
  }
  case OMX_ALG_IndexParamVideoGopControl:
  {
    auto port = getCurrentPort(param);
    auto gc = static_cast<OMX_ALG_VIDEO_PARAM_GOP_CONTROL*>(param);
    return ConstructVideoGopControl(*gc, *port, media);
  }
  case OMX_ALG_IndexParamVideoSceneChangeResilience:
  {
    auto port = getCurrentPort(param);
    auto r = static_cast<OMX_ALG_VIDEO_PARAM_SCENE_CHANGE_RESILIENCE*>(param);
    return ConstructVideoSceneChangeResilience(*r, *port, media);
  }
  case OMX_ALG_IndexParamVideoInstantaneousDecodingRefresh:
  {
    auto port = getCurrentPort(param);
    auto idr = static_cast<OMX_ALG_VIDEO_PARAM_INSTANTANEOUS_DECODING_REFRESH*>(param);
    return ConstructVideoInstantaneousDecodingRefresh(*idr, *port, media);
  }
  case OMX_ALG_IndexParamVideoCodedPictureBuffer:
  {
    auto port = getCurrentPort(param);
    auto cpb = static_cast<OMX_ALG_VIDEO_PARAM_CODED_PICTURE_BUFFER*>(param);
    return ConstructVideoCodedPictureBuffer(*cpb, *port, media);
  }
  case OMX_ALG_IndexParamVideoPrefetchBuffer:
  {
    auto port = getCurrentPort(param);
    auto pb = static_cast<OMX_ALG_VIDEO_PARAM_PREFETCH_BUFFER*>(param);
    return ConstructVideoPrefetchBuffer(*pb, *port, media);
  }
  case OMX_ALG_IndexParamVideoScalingList:
  {
    auto port = getCurrentPort(param);
    auto scl = static_cast<OMX_ALG_VIDEO_PARAM_SCALING_LIST*>(param);
    return ConstructVideoScalingList(*scl, *port, media);
  }
  case OMX_ALG_IndexParamVideoFillerData:
  {
    auto port = getCurrentPort(param);
    auto f = static_cast<OMX_ALG_VIDEO_PARAM_FILLER_DATA*>(param);
    return ConstructVideoFillerData(*f, *port, media);
  }
  case OMX_ALG_IndexParamVideoSlices:
  {
    auto port = getCurrentPort(param);
    auto s = static_cast<OMX_ALG_VIDEO_PARAM_SLICES*>(param);
    return ConstructVideoSlices(*s, *port, media);
  }
  case OMX_ALG_IndexParamVideoLongTerm:
  {
    auto port = getCurrentPort(param);
    auto longTerm = static_cast<OMX_ALG_VIDEO_PARAM_LONG_TERM*>(param);
    return ConstructVideoLongTerm(*longTerm, *port, media);
  }
  case OMX_ALG_IndexParamVideoLookAhead:
  {
    auto port = getCurrentPort(param);
    auto la = static_cast<OMX_ALG_VIDEO_PARAM_LOOKAHEAD*>(param);
    return ConstructVideoLookAhead(*la, *port, media);
  }
  // only decoder
  case OMX_ALG_IndexParamPreallocation:
  {
    auto prealloc = static_cast<OMX_ALG_PARAM_PREALLOCATION*>(param);
    return ConstructPreallocation(*prealloc, this->shouldPrealloc);
  }
  case OMX_ALG_IndexParamVideoDecodedPictureBuffer:
  {
    auto port = getCurrentPort(param);
    auto dpb = static_cast<OMX_ALG_VIDEO_PARAM_DECODED_PICTURE_BUFFER*>(param);
    return ConstructVideoDecodedPictureBuffer(*dpb, *port, media);
  }
  case OMX_ALG_IndexParamVideoInternalEntropyBuffers:
  {
    auto port = getCurrentPort(param);
    auto ieb = static_cast<OMX_ALG_VIDEO_PARAM_INTERNAL_ENTROPY_BUFFERS*>(param);
    return ConstructVideoInternalEntropyBuffers(*ieb, *port, media);
  }
  case OMX_ALG_IndexParamCommonSequencePictureModeQuerySupported:
  {
    auto mode = (OMX_ALG_COMMON_PARAM_SEQUENCE_PICTURE_MODE*)param;
    return ConstructCommonSequencePictureModesSupported(mode, media);
  }
  case OMX_ALG_IndexParamCommonSequencePictureModeCurrent:
  {
    auto port = getCurrentPort(param);
    auto mode = static_cast<OMX_ALG_COMMON_PARAM_SEQUENCE_PICTURE_MODE*>(param);
    return ConstructCommonSequencePictureMode(*mode, *port, media);
  }
#ifdef ANDROID
  case OMX_ALG_IndexExtGetNativeBufferUsage:
  {
    auto const port = getCurrentPort(param);
    android::GetAndroidNativeBufferUsageParams* usg_param = (android::GetAndroidNativeBufferUsageParams*)param;
    if (IsInputPort(port->index))
    {
      LOGE("OMX_ALG_IndexExtGetNativeBufferUsage called on input port. Not supported");
      return OMX_ErrorBadParameter;
    }
    usg_param->nUsage = GRALLOC_USAGE_PRIVATE_2;
    return OMX_ErrorNone;
  }
#endif
  default:
    LOGE("index 0x%x is unsupported", index);
    return OMX_ErrorUnsupportedIndex;
  }

  LOGE("index 0x%x is unsupported", index);
  return OMX_ErrorUnsupportedIndex;
  OMX_CATCH_PARAMETER();
}

OMX_ERRORTYPE DecComponent::SetParameter(OMX_IN OMX_INDEXTYPE index, OMX_IN OMX_PTR param)
{
  OMX_TRY();
  OMXChecker::CheckNotNull(param);
  OMXChecker::CheckHeaderVersion(GetVersion(param));

  auto getCurrentPort = [=](OMX_PTR param) -> Port*
                        {
                          auto index = *(((OMX_U32*)param) + 2);
                          return GetPort(index);
                        };

  Port* port;
  isSettingsInit = false;

  if(OMX_U32(index) != OMX_IndexParamStandardComponentRole)
  {
    port = getCurrentPort(param);

    if(!port->isTransientToDisable && port->enable)
      OMXChecker::CheckStateOperation(AL_SetParameter, state);
  }
  switch(static_cast<OMX_U32>(index)) // all indexes are 32u
  {
  case OMX_IndexParamStandardComponentRole:
  {
    OMXChecker::CheckStateOperation(AL_SetParameter, state);
    auto p = (OMX_PARAM_COMPONENTROLETYPE*)param;

    if(!strncmp((char*)role, (char*)p->cRole, strlen((char*)role)))
    {
      media->Reset();
      return OMX_ErrorNone;
    }
    throw OMX_ErrorBadParameter;
  }
  case OMX_IndexParamPortDefinition:
  {
    auto settings = static_cast<OMX_PARAM_PORTDEFINITIONTYPE*>(param);
    SetPortExpectedBuffer(*settings, const_cast<Port &>(*port), *module);
#ifdef ANDROID
    OMX_ALG_PORT_PARAM_BUFFER_MODE buf_mode;
    OMX_ERRORTYPE err;
    err = ConstructPortBufferMode(buf_mode, *port, media);
    if (err)
      return err;
    if (buf_mode.eMode == OMX_ALG_BUF_DMA)
    {
      if (settings->format.video.eColorFormat == (OMX_COLOR_FORMATTYPE)HAL_PIXEL_FORMAT_YCbCr_420_888)
        settings->format.video.eColorFormat = OMX_COLOR_FormatYUV420SemiPlanar;
    }
#endif
    return DecSetPortDefinition(*settings, *port, *module, media);
  }
  case OMX_IndexParamCompBufferSupplier:
  {
    // Do nothing
    return OMX_ErrorNone;
  }
  case OMX_IndexParamVideoPortFormat:
  {
    auto format = static_cast<OMX_VIDEO_PARAM_PORTFORMATTYPE*>(param);
#if ANDROID
    OMX_ALG_PORT_PARAM_BUFFER_MODE buf_mode;
    OMX_ERRORTYPE err;
    err = ConstructPortBufferMode(buf_mode, *port, media);
    if (err)
      return err;
    if (buf_mode.eMode == OMX_ALG_BUF_DMA)
    {
      if (format->eColorFormat == (OMX_COLOR_FORMATTYPE)HAL_PIXEL_FORMAT_YCbCr_420_888)
        format->eColorFormat = OMX_COLOR_FormatYUV420SemiPlanar;
    }
#endif    
    return DecSetVideoPortFormat(*format, *port, media);
  }
  case OMX_IndexParamVideoProfileLevelCurrent:
  {
    return expertise->SetProfileLevel(param, *port, media);
  }
  case OMX_IndexParamVideoAvc:
  case OMX_ALG_IndexParamVideoHevc:
  {
    return expertise->SetExpertise(param, *port, media);
  }
  case OMX_ALG_IndexParamVideoSubframe:
  {
    auto subframe = static_cast<OMX_ALG_VIDEO_PARAM_SUBFRAME*>(param);
    return SetVideoSubframe(*subframe, *port, media);
  }
  case OMX_ALG_IndexPortParamBufferMode:
  {
    auto portBufferMode = static_cast<OMX_ALG_PORT_PARAM_BUFFER_MODE*>(param);
    return SetPortBufferMode(*portBufferMode, *port, media);
  }
  // only encoder
  case OMX_IndexParamVideoQuantization:
  {
    auto quantization = static_cast<OMX_VIDEO_PARAM_QUANTIZATIONTYPE*>(param);
    return SetVideoQuantization(*quantization, *port, media);
  }
  case OMX_IndexParamVideoBitrate:
  {
    auto bitrate = static_cast<OMX_VIDEO_PARAM_BITRATETYPE*>(param);
    return SetVideoBitrate(*bitrate, *port, media);
  }
  case OMX_ALG_IndexParamVideoInterlaceFormatCurrent:
  {
    auto interlaced = static_cast<OMX_INTERLACEFORMATTYPE*>(param);
    return SetVideoModeCurrent(*interlaced, *port, media);
  }
  case OMX_ALG_IndexParamVideoQuantizationControl:
  {
    auto quantizationControl = static_cast<OMX_ALG_VIDEO_PARAM_QUANTIZATION_CONTROL*>(param);
    return SetVideoQuantizationControl(*quantizationControl, *port, media);
  }
  case OMX_ALG_IndexParamVideoQuantizationExtension:
  {
    auto quantizationExtension = static_cast<OMX_ALG_VIDEO_PARAM_QUANTIZATION_EXTENSION*>(param);
    return SetVideoQuantizationExtension(*quantizationExtension, *port, media);
  }
  case OMX_ALG_IndexParamVideoAspectRatio:
  {
    auto aspectRatio = static_cast<OMX_ALG_VIDEO_PARAM_ASPECT_RATIO*>(param);
    return SetVideoAspectRatio(*aspectRatio, *port, media);
  }
  case OMX_ALG_IndexParamVideoMaxBitrate:
  {
    auto maxBitrate = static_cast<OMX_ALG_VIDEO_PARAM_MAX_BITRATE*>(param);
    return SetVideoMaxBitrate(*maxBitrate, *port, media);
  }
  case OMX_ALG_IndexParamVideoLowBandwidth:
  {
    auto lowBandwidth = static_cast<OMX_ALG_VIDEO_PARAM_LOW_BANDWIDTH*>(param);
    return SetVideoLowBandwidth(*lowBandwidth, *port, media);
  }
  case OMX_ALG_IndexParamVideoGopControl:
  {
    auto gopControl = static_cast<OMX_ALG_VIDEO_PARAM_GOP_CONTROL*>(param);
    return SetVideoGopControl(*gopControl, *port, media);
  }
  case OMX_ALG_IndexParamVideoSceneChangeResilience:
  {
    auto sceneChangeResilience = static_cast<OMX_ALG_VIDEO_PARAM_SCENE_CHANGE_RESILIENCE*>(param);
    return SetVideoSceneChangeResilience(*sceneChangeResilience, *port, media);
  }
  case OMX_ALG_IndexParamVideoInstantaneousDecodingRefresh:
  {
    auto instantaneousDecodingRefresh = static_cast<OMX_ALG_VIDEO_PARAM_INSTANTANEOUS_DECODING_REFRESH*>(param);
    return SetVideoInstantaneousDecodingRefresh(*instantaneousDecodingRefresh, *port, media);
  }
  case OMX_ALG_IndexParamVideoCodedPictureBuffer:
  {
    auto codedPictureBuffer = static_cast<OMX_ALG_VIDEO_PARAM_CODED_PICTURE_BUFFER*>(param);
    return SetVideoCodedPictureBuffer(*codedPictureBuffer, *port, media);
  }
  case OMX_ALG_IndexParamVideoPrefetchBuffer:
  {
    auto prefetchBuffer = static_cast<OMX_ALG_VIDEO_PARAM_PREFETCH_BUFFER*>(param);
    return SetVideoPrefetchBuffer(*prefetchBuffer, *port, media);
  }
  case OMX_ALG_IndexParamVideoScalingList:
  {
    auto scalingList = static_cast<OMX_ALG_VIDEO_PARAM_SCALING_LIST*>(param);
    return SetVideoScalingList(*scalingList, *port, media);
  }
  case OMX_ALG_IndexParamVideoFillerData:
  {
    auto fillerData = static_cast<OMX_ALG_VIDEO_PARAM_FILLER_DATA*>(param);
    return SetVideoFillerData(*fillerData, *port, media);
  }
  case OMX_ALG_IndexParamVideoSlices:
  {
    auto slices = static_cast<OMX_ALG_VIDEO_PARAM_SLICES*>(param);
    return SetVideoSlices(*slices, *port, media);
  }
  case OMX_ALG_IndexParamVideoLongTerm:
  {
    auto longTerm = static_cast<OMX_ALG_VIDEO_PARAM_LONG_TERM*>(param);
    return SetVideoLongTerm(*longTerm, *port, media);
  }
  case OMX_ALG_IndexParamVideoLookAhead:
  {
    auto la = static_cast<OMX_ALG_VIDEO_PARAM_LOOKAHEAD*>(param);
    return SetVideoLookAhead(*la, *port, media);
  }
  // only decoder
  case OMX_ALG_IndexParamPreallocation:
  {
    auto p = (OMX_ALG_PARAM_PREALLOCATION*)param;
    this->shouldPrealloc = p->bDisablePreallocation == OMX_FALSE;
    return OMX_ErrorNone;
  }
  case OMX_ALG_IndexParamVideoDecodedPictureBuffer:
  {
    auto dpb = static_cast<OMX_ALG_VIDEO_PARAM_DECODED_PICTURE_BUFFER*>(param);

    return SetVideoDecodedPictureBuffer(*dpb, *port, media);
  }
  case OMX_ALG_IndexParamVideoInternalEntropyBuffers:
  {
    auto ieb = static_cast<OMX_ALG_VIDEO_PARAM_INTERNAL_ENTROPY_BUFFERS*>(param);

    return SetVideoInternalEntropyBuffers(*ieb, *port, media);
  }
  case OMX_ALG_IndexParamCommonSequencePictureModeCurrent:
  {
    auto spm = static_cast<OMX_ALG_COMMON_PARAM_SEQUENCE_PICTURE_MODE*>(param);

    return SetCommonSequencePictureMode(*spm, *port, media);
  }
#ifdef ANDROID
  case OMX_ALG_IndexExtEnableNativeBuffer:
  {
    auto const port = getCurrentPort(param);
    if(!port->isTransientToDisable && port->enable)
      OMXChecker::CheckStateOperation(AL_SetParameter, state);
    auto const android_buf_mode = (android::EnableAndroidNativeBuffersParams*)param;
    OMX_ALG_PORT_PARAM_BUFFER_MODE alg_buf_mode;
    OMXChecker::SetHeaderVersion(alg_buf_mode);
    alg_buf_mode.nPortIndex = port->index;

    if (android_buf_mode->enable == OMX_TRUE)
      alg_buf_mode.eMode = OMX_ALG_BUF_DMA;
    else
      alg_buf_mode.eMode = OMX_ALG_BUF_NORMAL;

    auto portBufferMode = static_cast<OMX_ALG_PORT_PARAM_BUFFER_MODE*>(param);
    return SetPortBufferMode(alg_buf_mode, *port, media);
  }
  case OMX_ALG_IndexExtUseNativeBuffer:
  {
    auto const port = getCurrentPort(param);
    if(IsInputPort(port->index))
      return OMX_ErrorBadParameter;
    
    auto buf_params = (android::UseAndroidNativeBufferParams *)(param);
    if (buf_params->nativeBuffer == NULL || buf_params->nativeBuffer->handle == NULL)
      return OMX_ErrorBadParameter;
    
    android::sp<android_native_buffer_t> nBuf = buf_params->nativeBuffer;
    private_handle_t *handle = (private_handle_t *)nBuf->handle;
    OMX_U8 *buffer = NULL;
    buffer = (OMX_U8*)(uintptr_t)(handle->share_fd);
    return UseBuffer(buf_params->bufferHeader, buf_params->nPortIndex, buf_params->pAppPrivate, handle->size, buffer);
  }
#endif
  default:
    LOGE("index 0x%x is unsupported", index);
    return OMX_ErrorUnsupportedIndex;
  }

  LOGE("index 0x%x is unsupported", index);
  return OMX_ErrorUnsupportedIndex;
  OMX_CATCH_PARAMETER();
}
