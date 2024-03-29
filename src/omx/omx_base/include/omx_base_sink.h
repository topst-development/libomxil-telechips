/**
  @file src/base/omx_base_sink.h

  OpenMAX base sink component. This component does not perform any multimedia
  processing. It derives from base component and contains a single port. It can be used
  as base class for sink components.

  Copyright (C) 2007-2008 STMicroelectronics
  Copyright (C) 2007-2008 Nokia Corporation and/or its subsidiary(-ies).

  This library is free software; you can redistribute it and/or modify it under
  the terms of the GNU Lesser General Public License as published by the Free
  Software Foundation; either version 2.1 of the License, or (at your option)
  any later version.

  This library is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
  FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
  details.

  You should have received a copy of the GNU Lesser General Public License
  along with this library; if not, write to the Free Software Foundation, Inc.,
  51 Franklin St, Fifth Floor, Boston, MA
  02110-1301  USA

  $Date: 2008-08-28 17:50:08 +0200 (Thu, 28 Aug 2008) $
  Revision $Rev: 583 $
  Author $Author: gsent $

*/

#ifndef OMX_BASE_SINK_COMPONENT_H_
#define OMX_BASE_SINK_COMPONENT_H_

#include <OMX_Types.h>
#include <OMX_Component.h>
#include <OMX_Core.h>
#include <pthread.h>
#include "omx_base_component.h"
#include <stdlib.h>


#define OMX_BASE_SINK_INPUTPORT_INDEX   0     /* The index of the input port for the derived components */
#define OMX_BASE_SINK_CLOCKPORT_INDEX   1     /* The index of the clock port for the dervied components */

/** OMX_BASE_SINK_ALLPORT_INDEX as the standard specifies, the -1 value for port index is used to point to all the ports
 */
#define OMX_BASE_SINK_ALLPORT_INDEX -1

/** base sink component private structure.
 */
DERIVEDCLASS(omx_base_sink_PrivateType, omx_base_component_PrivateType)
#define omx_base_sink_PrivateType_FIELDS omx_base_component_PrivateType_FIELDS \
  /** @param BufferMgmtCallback function pointer for algorithm callback */ \
  void (*BufferMgmtCallback)(OMX_COMPONENTTYPE* openmaxStandComp, OMX_BUFFERHEADERTYPE* inputbuffer);
ENDCLASS(omx_base_sink_PrivateType)

/** Base sink contructor
 */
OMX_ERRORTYPE omx_base_sink_Constructor(OMX_COMPONENTTYPE *openmaxStandComp,OMX_STRING cComponentName);

/** The base sink destructor. It simply calls the base destructor
 */
OMX_ERRORTYPE omx_base_sink_Destructor(OMX_COMPONENTTYPE *openmaxStandComp);

/** This is the central function for component processing. It
 * is executed in a separate thread, is synchronized with
 * semaphores at each port, those are released each time a new buffer
 * is available on the given port.
 */
void* omx_base_sink_BufferMgmtFunction(void* param);

#endif
