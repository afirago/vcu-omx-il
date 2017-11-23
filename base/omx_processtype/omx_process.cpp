/******************************************************************************
*
* Copyright (C) 2017 Allegro DVT2.  All rights reserved.
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

#include "omx_processtype.h"
#include "base/omx_utils/omx_log.h"

void ProcessType::checkTransitions()
{
  if (m_state != m_TargetState) {

    bool transitionComplete = true;

    if (m_state == OMX_StateLoaded) {
      // OMX requires that all buffers should be allocated
      // before transition from Loaded to Idle completes
      if (m_TargetState == OMX_StateIdle)
        if (!AllocateDone())
          transitionComplete = false;
    } else if (m_TargetState == OMX_StateLoaded) {
      // TODO
    }

    if (transitionComplete) {
      LOGV("state transition from %d to %d complete", m_state, m_TargetState);
      m_state = m_TargetState;

      m_pCallback->EventHandler(m_hComponent, m_pAppData, OMX_EventCmdComplete, OMX_CommandStateSet, m_state, nullptr);
    } else {
      LOGV("state transition from %d to %d not yet complete", m_state, m_TargetState);
    }
  }
}

OMX_ERRORTYPE ProcessType::FreeBuffer(OMX_IN OMX_U32 nPortIndex, OMX_IN OMX_BUFFERHEADERTYPE* pBufferHdr)
{
  auto port = GetPort(nPortIndex);

  if(!port)
    return OMX_ErrorBadPortIndex;

  port->destroyBuffer(pBufferHdr);

  checkTransitions();
  return OMX_ErrorNone;
}

Port* ProcessType::GetPort(OMX_U32 const nPortIndex)
{
  if(nPortIndex == inPort.getDefinition().nPortIndex)
    return &inPort;

  if(nPortIndex == outPort.getDefinition().nPortIndex)
    return &outPort;

  return nullptr;
}

bool ProcessType::AllocateInputDone()
{
  // we consider allocation is done if port is disabled
  if (inPort.getDefinition().bEnabled == OMX_FALSE)
    return true;

  // if no buffers to be allocated we consider allocation is done
  if (inPort.getDefinition().nBufferCountActual == 0)
    return true;

  // otherwise it should be populated
  if (inPort.getDefinition().bPopulated == OMX_TRUE)
    return true;

  return false;
}

bool ProcessType::AllocateOutputDone()
{
  // we consider allocation is done if port is disabled
  if (outPort.getDefinition().bEnabled == OMX_FALSE)
    return true;

  // if no buffers to be allocated we consider allocation is done
  if (outPort.getDefinition().nBufferCountActual == 0)
    return true;

  // otherwise it should be populated
  if (outPort.getDefinition().bPopulated == OMX_TRUE)
    return true;

  return false;
}

bool ProcessType::AllocateDone()
{
  if (AllocateInputDone() && AllocateOutputDone())
    return true;
  return false;
}
