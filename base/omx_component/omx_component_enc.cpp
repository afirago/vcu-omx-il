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

#include "omx_component_enc.h"

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
#endif

using namespace std;

static EncModule& ToEncModule(ModuleInterface& module)
{
  return dynamic_cast<EncModule &>(module);
}

EncComponent::EncComponent(OMX_HANDLETYPE component, shared_ptr<MediatypeInterface> media, std::unique_ptr<EncModule>&& module, OMX_STRING name, OMX_STRING role, std::unique_ptr<Expertise>&& expertise, std::shared_ptr<SyncIpInterface> syncIp) :
  Component(component, media, std::move(module), std::move(expertise), name, role), syncIp(syncIp)
{
}

EncComponent::~EncComponent() = default;

void EncComponent::EmptyThisBufferCallBack(BufferHandleInterface* handle)
{
  auto header = ((OMXBufferHandle*)(handle))->header;
  delete handle;

  ClearPropagatedData(header);

  if(roiMap.Exist(header))
  {
    auto roiBuffer = roiMap.Pop(header);
    roiFreeBuffers.push(roiBuffer);
  }

  if(callbacks.EmptyBufferDone)
    callbacks.EmptyBufferDone(component, app, header);
}

static void AddEncoderFlags(OMXBufferHandle* handle, EncModule& module)
{
  auto flags = module.GetFlags(handle);

  if(flags.isSync)
    handle->header->nFlags |= OMX_BUFFERFLAG_SYNCFRAME;

  if(flags.isEndOfFrame)
    handle->header->nFlags |= OMX_BUFFERFLAG_ENDOFFRAME;

  if(flags.isEndOfSlice)
    handle->header->nFlags |= OMX_BUFFERFLAG_CODECCONFIG;
}

void EncComponent::AssociateCallBack(BufferHandleInterface* empty_, BufferHandleInterface* fill_)
{
  auto empty = (OMXBufferHandle*)(empty_);
  auto fill = (OMXBufferHandle*)(fill_);
  auto emptyHeader = empty->header;
  auto fillHeader = fill->header;

  PropagateHeaderData(emptyHeader, fillHeader);
  AddEncoderFlags(fill, ToEncModule(*module));

  if(IsEOSDetected(emptyHeader->nFlags))
    callbacks.EventHandler(component, app, OMX_EventBufferFlag, output.index, emptyHeader->nFlags, nullptr);

  if(IsCompMarked(emptyHeader->hMarkTargetComponent, component))
    callbacks.EventHandler(component, app, OMX_EventMark, 0, 0, emptyHeader->pMarkData);
}

void EncComponent::FillThisBufferCallBack(BufferHandleInterface* filled, int offset, int size)
{
  assert(filled);
  auto header = (OMX_BUFFERHEADERTYPE*)(((OMXBufferHandle*)(filled))->header);
  delete filled;

  header->nOffset = offset;
  header->nFilledLen = size;

  if(header->nFlags & OMX_BUFFERFLAG_ENDOFFRAME)
    syncIp->addBuffer(nullptr);

  if(callbacks.FillBufferDone)
    callbacks.FillBufferDone(component, app, header);
}

OMX_ERRORTYPE EncComponent::GetExtensionIndex(OMX_IN OMX_STRING name, OMX_OUT OMX_INDEXTYPE* index)
{
  OMX_TRY();
  OMXChecker::CheckNotNull(name);
  OMXChecker::CheckNotNull(index);
  OMXChecker::CheckStateOperation(AL_GetExtensionIndex, state);
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

uint8_t* EncComponent::AllocateROIBuffer()
{
  int roiSize;
  module->GetDynamic(DYNAMIC_INDEX_REGION_OF_INTEREST_QUALITY_BUFFER_SIZE, &roiSize);
  return static_cast<uint8_t*>(calloc(roiSize, sizeof(uint8_t)));
}

OMX_ERRORTYPE EncComponent::UseBuffer(OMX_OUT OMX_BUFFERHEADERTYPE** header, OMX_IN OMX_U32 index, OMX_IN OMX_PTR app, OMX_IN OMX_U32 size, OMX_IN OMX_U8* buffer)
{
  OMX_TRY();
  OMXChecker::CheckNotNull(header);
  OMXChecker::CheckNotNull(size);

  CheckPortIndex(index);
  auto port = GetPort(index);

  if(transientState != TransientLoadedToIdle && !(port->isTransientToEnable))
    throw OMX_ErrorIncorrectStateOperation;

  *header = AllocateHeader(app, size, buffer, false, index);
  assert(*header);
  port->Add(*header);

  if(IsInputPort(index))
  {
    auto roiBuffer = AllocateROIBuffer();
    roiFreeBuffers.push(roiBuffer);
    roiDestroyMap.Add(*header, roiBuffer);

    auto bufferHandlePort = IsInputPort(index) ? ToEncModule(*module).GetBufferHandles().input : ToEncModule(*module).GetBufferHandles().output;
    bool dmaOnPort = (bufferHandlePort == BufferHandleType::BUFFER_HANDLE_FD);

    if(dmaOnPort)
    {
      auto handle = OMXBufferHandle(*header);
      syncIp->addBuffer(&handle);
    }
  }

  if(port->playable && IsInputPort(index))
    syncIp->enable();

  return OMX_ErrorNone;
  OMX_CATCH_L([&](OMX_ERRORTYPE& e)
  {
    if(e != OMX_ErrorBadPortIndex)
      GetPort(index)->ErrorOccured();
  });
}

OMX_ERRORTYPE EncComponent::AllocateBuffer(OMX_INOUT OMX_BUFFERHEADERTYPE** header, OMX_IN OMX_U32 index, OMX_IN OMX_PTR app, OMX_IN OMX_U32 size)
{
  OMX_TRY();
  OMXChecker::CheckNotNull(header);
  OMXChecker::CheckNotNull(size);
  CheckPortIndex(index);

  auto port = GetPort(index);

  if(transientState != TransientLoadedToIdle && !(port->isTransientToEnable))
    throw OMX_ErrorIncorrectStateOperation;

  auto bufferHandlePort = IsInputPort(index) ? ToEncModule(*module).GetBufferHandles().input : ToEncModule(*module).GetBufferHandles().output;
  bool dmaOnPort = (bufferHandlePort == BufferHandleType::BUFFER_HANDLE_FD);
  auto buffer = dmaOnPort ? reinterpret_cast<OMX_U8*>(ToEncModule(*module).AllocateDMA(size * sizeof(OMX_U8))) : static_cast<OMX_U8*>(module->Allocate(size * sizeof(OMX_U8)));

  if(dmaOnPort ? (static_cast<int>((intptr_t)buffer) < 0) : !buffer)
    throw OMX_ErrorInsufficientResources;

  *header = AllocateHeader(app, size, buffer, true, index);
  assert(*header);
  port->Add(*header);

  if(IsInputPort(index))
  {
    auto roiBuffer = AllocateROIBuffer();
    roiFreeBuffers.push(roiBuffer);
    roiDestroyMap.Add(*header, roiBuffer);

    if(dmaOnPort)
    {
      auto handle = OMXBufferHandle(*header);
      syncIp->addBuffer(&handle);
    }
  }

  if(port->playable && IsInputPort(index))
    syncIp->enable();

  return OMX_ErrorNone;
  OMX_CATCH_L([&](OMX_ERRORTYPE& e)
  {
    if(e != OMX_ErrorBadPortIndex)
      GetPort(index)->ErrorOccured();
  });
}

void EncComponent::DestroyROIBuffer(uint8_t* roiBuffer)
{
  free(roiBuffer);
}

OMX_ERRORTYPE EncComponent::FreeBuffer(OMX_IN OMX_U32 index, OMX_IN OMX_BUFFERHEADERTYPE* header)
{
  OMX_TRY();
  OMXChecker::CheckNotNull(header);

  CheckPortIndex(index);
  auto port = GetPort(index);

  if((transientState != TransientIdleToLoaded) && (!port->isTransientToDisable))
    callbacks.EventHandler(component, app, OMX_EventError, OMX_ErrorPortUnpopulated, 0, nullptr);

  if(isBufferAllocatedByModule(header))
  {
    auto bufferHandlePort = IsInputPort(index) ? ToEncModule(*module).GetBufferHandles().input : ToEncModule(*module).GetBufferHandles().output;
    bool dmaOnPort = (bufferHandlePort == BufferHandleType::BUFFER_HANDLE_FD);
    dmaOnPort ? ToEncModule(*module).FreeDMA(static_cast<int>((intptr_t)header->pBuffer)) : module->Free(header->pBuffer);
  }

  if(IsInputPort(index))
  {
    if(roiDestroyMap.Exist(header))
    {
      auto roiBuffer = roiDestroyMap.Pop(header);
      DestroyROIBuffer(roiBuffer);
    }
  }

  port->Remove(header);
  DeleteHeader(header);

  return OMX_ErrorNone;
  OMX_CATCH();
}

void EncComponent::TreatEmptyBufferCommand(Task* task)
{
  assert(task);
  assert(task->cmd == EmptyBuffer);
  assert(static_cast<int>((intptr_t)task->data) == input.index);
  auto header = static_cast<OMX_BUFFERHEADERTYPE*>(task->opt.get());
  assert(header);
  AttachMark(header);

  if(shouldPushROI && header->nFilledLen)
  {
    auto roiBuffer = roiFreeBuffers.pop();
    module->GetDynamic(DYNAMIC_INDEX_REGION_OF_INTEREST_QUALITY_BUFFER_FILL, roiBuffer);
    module->SetDynamic(DYNAMIC_INDEX_REGION_OF_INTEREST_QUALITY_BUFFER_EMPTY, roiBuffer);
    roiMap.Add(header, roiBuffer);
  }

  auto handle = new OMXBufferHandle(header);
  auto success = module->Empty(handle);

  shouldClearROI = true;
  assert(success);
}

OMX_ERRORTYPE EncComponent::GetParameter(OMX_IN OMX_INDEXTYPE index, OMX_INOUT OMX_PTR param)
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
    return ConstructPortDefinition(*def, *port, *module, media);
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
    if (!IsInputPort(port->index))
      p->eColorFormat = OMX_COLOR_FormatUnused;
    else
      p->eCompressionFormat = OMX_VIDEO_CodingUnused;
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
  case OMX_IndexParamVideoHevc:
  {
    auto port = getCurrentPort(param);
    auto hevcType = static_cast<OMX_VIDEO_PARAM_HEVCTYPE*>(param);
    hevcType->nKeyFrameInterval = 0; // unspecified; FIXME

    OMX_VIDEO_PARAM_PROFILELEVELTYPE levelType;
    auto ret = expertise->GetProfileLevel((OMX_PTR)(&levelType), *port, media);
    if (ret != OMX_ErrorNone)
      return ret;
    hevcType->eProfile = static_cast<OMX_VIDEO_HEVCPROFILETYPE>(levelType.eProfile);
    hevcType->eLevel = static_cast<OMX_VIDEO_HEVCLEVELTYPE>(levelType.eLevel);
    return OMX_ErrorNone;
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
  default:
    LOGE("index 0x%x is unsupported", index);
    return OMX_ErrorUnsupportedIndex;
  }

  LOGE("index 0x%x is unsupported", index);
  return OMX_ErrorUnsupportedIndex;
  OMX_CATCH_PARAMETER();
}

OMX_ERRORTYPE EncComponent::EncSetPortDefinition(OMX_PARAM_PORTDEFINITIONTYPE const& settings, Port& port, ModuleInterface& module, shared_ptr<MediatypeInterface> media)
{
  OMX_PARAM_PORTDEFINITIONTYPE rollback;
  ConstructPortDefinition(rollback, port, module, media);
  auto video = settings.format.video;
  OMX_ERRORTYPE ret;

  if (IsInputPort(port.index))
  {
    ret = SetFormat(video.eColorFormat, media);

    if(ret != OMX_ErrorNone)
    {
      EncSetPortDefinition(rollback, port, module, media);
      throw ret;
    }

    ret = SetClock(video.xFramerate, media);

    if(ret != OMX_ErrorNone)
    {
      EncSetPortDefinition(rollback, port, module, media);
      throw ret;
    }
  }

  ret = SetResolution(video, media);

  if(ret != OMX_ErrorNone)
  {
    EncSetPortDefinition(rollback, port, module, media);
    throw ret;
  }

  if (!IsInputPort(port.index))
  {
    // Set Target is only used for encoder, ignored for decoder
    ret = SetTargetBitrate(video.nBitrate, media);

    if(ret != OMX_ErrorNone && ret != OMX_ErrorUnsupportedIndex)
    {
      EncSetPortDefinition(rollback, port, module, media);
      throw ret;
    }
  }
  return OMX_ErrorNone;
}

OMX_ERRORTYPE EncComponent::EncSetVideoPortFormat(OMX_VIDEO_PARAM_PORTFORMATTYPE const& format, Port const& port, shared_ptr<MediatypeInterface> media)
{
  OMX_VIDEO_PARAM_PORTFORMATTYPE rollback;
  ConstructVideoPortCurrentFormat(rollback, port, media);
  if (IsInputPort(port.index))
  {
    auto ret = SetClock(format.xFramerate, media);

    if(ret != OMX_ErrorNone)
    {
      EncSetVideoPortFormat(rollback, port, media);
      throw ret;
    }

    ret = SetFormat(format.eColorFormat, media);

    if(ret != OMX_ErrorNone)
    {
      EncSetVideoPortFormat(rollback, port, media);
      throw ret;
    }
  }
  return OMX_ErrorNone;
}

OMX_ERRORTYPE EncComponent::EncSetVideoBitrate(OMX_VIDEO_PARAM_BITRATETYPE const& bitrate, Port const& port, shared_ptr<MediatypeInterface> media)
{
  OMX_VIDEO_PARAM_BITRATETYPE rollback;
  ConstructVideoBitrate(rollback, port, media);

  if (!IsInputPort(port.index))
  {
    auto ret = SetModeBitrate(bitrate.nTargetBitrate, bitrate.eControlRate, media);

    if(ret != OMX_ErrorNone)
    {
      EncSetVideoBitrate(rollback, port, media);
      throw ret;
    }
  }
  return OMX_ErrorNone;
}

OMX_ERRORTYPE EncComponent::SetParameter(OMX_IN OMX_INDEXTYPE index, OMX_IN OMX_PTR param)
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
    return EncSetPortDefinition(*settings, *port, *module, media);
  }
  case OMX_IndexParamCompBufferSupplier:
  {
    // Do nothing
    return OMX_ErrorNone;
  }
  case OMX_IndexParamVideoPortFormat:
  {
    auto format = static_cast<OMX_VIDEO_PARAM_PORTFORMATTYPE*>(param);
    return EncSetVideoPortFormat(*format, *port, media);
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
  case OMX_IndexParamVideoHevc:
  {
    auto hevcType = static_cast<OMX_VIDEO_PARAM_HEVCTYPE*>(param);

    OMX_VIDEO_PARAM_PROFILELEVELTYPE levelType;
    auto ret = expertise->GetProfileLevel((OMX_PTR)(&levelType), *port, media);
    if (ret != OMX_ErrorNone)
      return ret;

    levelType.eProfile = hevcType->eProfile;
    levelType.eLevel = hevcType->eLevel;

    return expertise->SetProfileLevel((OMX_PTR)(&levelType), *port, media);
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
    if (!IsInputPort(port->index))
    {
      auto bitrate = static_cast<OMX_VIDEO_PARAM_BITRATETYPE*>(param);
      return EncSetVideoBitrate(*bitrate, *port, media);
    }
    return OMX_ErrorNone;
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
  default:
    LOGE("index 0x%x is unsupported", index);
    return OMX_ErrorUnsupportedIndex;
  }

  LOGE("index 0x%x is unsupported", index);
  return OMX_ErrorUnsupportedIndex;
  OMX_CATCH_PARAMETER();
}
