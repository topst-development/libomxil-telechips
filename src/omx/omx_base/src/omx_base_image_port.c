/**
  @file src/base/omx_base_image_port.c

  Base Image Port class for OpenMAX ports to be used in derived components.

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

  $Date: 2008-08-29 06:15:38 +0200 (Fri, 29 Aug 2008) $
  Revision $Rev: 585 $
  Author $Author: pankaj_sen $
*/

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <omxcore.h>
#include <OMX_Core.h>
#include <OMX_Component.h>

#include "omx_base_component.h"
#include "omx_base_image_port.h"

/** 
  * @brief The base contructor for the generic OpenMAX ST Image port
  * 
  * This function is executed by the component that uses a port.
  * The parameter contains the info about the component.
  * It takes care of constructing the instance of the port and 
  * every object needed by the base port.
  *
  * @param openmaxStandComp pointer to the Handle of the component
  * @param openmaxStandPort the ST port to be initialized
  * @param nPortIndex Index of the port to be constructed
  * @param isInput specifices if the port is an input or an output
  * 
  * @return OMX_ErrorInsufficientResources if a memory allocation fails
  */

OMX_ERRORTYPE base_image_port_Constructor(OMX_COMPONENTTYPE *openmaxStandComp,omx_base_PortType **openmaxStandPort,OMX_U32 nPortIndex, OMX_BOOL isInput) {
  
  omx_base_image_PortType *omx_base_image_Port;

  if (!(*openmaxStandPort)) {
    *openmaxStandPort = calloc(1,sizeof (omx_base_image_PortType));
  }

  if (!(*openmaxStandPort)) {
    return OMX_ErrorInsufficientResources;
  }

  base_port_Constructor(openmaxStandComp,openmaxStandPort,nPortIndex, isInput);

  omx_base_image_Port = (omx_base_image_PortType *)*openmaxStandPort;

  setHeader(&omx_base_image_Port->sImageParam, sizeof(OMX_IMAGE_PARAM_PORTFORMATTYPE));
  omx_base_image_Port->sImageParam.nPortIndex = nPortIndex;
  omx_base_image_Port->sImageParam.nIndex = 0;
  omx_base_image_Port->sImageParam.eCompressionFormat = OMX_IMAGE_CodingUnused;
  
  omx_base_image_Port->sPortParam.eDomain = OMX_PortDomainImage;
  omx_base_image_Port->sPortParam.format.image.cMIMEType = malloc(DEFAULT_MIME_STRING_LENGTH);
  if (omx_base_image_Port->sPortParam.format.image.cMIMEType != NULL) {
    const char image_MIME[] = "raw/image";
    (void)strncpy(omx_base_image_Port->sPortParam.format.image.cMIMEType, image_MIME, sizeof(image_MIME));
  }
  omx_base_image_Port->sPortParam.format.image.pNativeRender = 0;
  omx_base_image_Port->sPortParam.format.image.bFlagErrorConcealment = OMX_FALSE;
  omx_base_image_Port->sPortParam.format.image.eCompressionFormat = OMX_IMAGE_CodingUnused;

  omx_base_image_Port->sPortParam.nBufferSize = (isInput == OMX_TRUE)?DEFAULT_IN_BUFFER_SIZE:DEFAULT_OUT_BUFFER_SIZE ;

  omx_base_image_Port->PortDestructor = &base_image_port_Destructor;

  return OMX_ErrorNone;
}

/** 
  * @brief The base image port destructor for the generic OpenMAX ST Image port
  * 
  * This function is executed by the component that uses a port.
  * The parameter contains the info about the port.
  * It takes care of destructing the instance of the port
  * 
  * @param openmaxStandPort the ST port to be destructed
  * 
  * @return OMX_ErrorNone 
  */

OMX_ERRORTYPE base_image_port_Destructor(omx_base_PortType *openmaxStandPort){

  if(openmaxStandPort->sPortParam.format.image.cMIMEType) {
    free(openmaxStandPort->sPortParam.format.image.cMIMEType);
    openmaxStandPort->sPortParam.format.image.cMIMEType = NULL;
  }

  base_port_Destructor(openmaxStandPort);

  return OMX_ErrorNone;
}
