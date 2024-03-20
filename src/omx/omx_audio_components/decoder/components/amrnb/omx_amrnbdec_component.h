/**

  @file omx_amrnbdec_component.h

  This file is header of amrnb decoder component.

  Copyright (C) 2007-2008  STMicroelectronics
  Copyright (C) 2007-2008 Nokia Corporation and/or its subsidiary(-ies).
  Copyright (C) 2009-2010 Telechips Inc.

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

*/

#ifndef OMX_AMRNBDEC_COMPONENT_H_
#define OMX_AMRNBDEC_COMPONENT_H_

#include <omx_audiodec_component.h>

#define TCC_OMX_COMPONENT_AMRNBDEC_MIME	("audio/AMR")
#define TCC_OMX_COMPONENT_AMRNBDEC_NAME	("OMX.TCC.amrnbdec")
#define TCC_OMX_COMPONENT_AMRNBDEC_ROLE	("audio_decoder.amrnb")

#define TCC_AMRNBDEC_LIB_NAME	("libtccamrnbdec.so")

// amrnb decoder class
DERIVEDCLASS(omx_amrnbdec_component_PrivateType, omx_audiodec_component_PrivateType)
#define omx_amrnbdec_component_PrivateType_FIELDS omx_audiodec_component_PrivateType_FIELDS \
	OMX_AUDIO_PARAM_AMRTYPE stAudioAmrnb;
ENDCLASS(omx_amrnbdec_component_PrivateType)

#endif /* OMX_amrnbDEC_COMPONENT_H_ */

