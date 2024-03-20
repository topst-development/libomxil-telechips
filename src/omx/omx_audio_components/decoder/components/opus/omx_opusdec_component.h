/**

  @file omx_opusdec_component.h

  This file is header of OPUS decoder component.

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

#ifndef OMX_OPUSDEC_COMPONENT_H_
#define OMX_OPUSDEC_COMPONENT_H_

#include <omx_audiodec_component.h>

#define TCC_OMX_COMPONENT_OPUSDEC_MIME	("audio/x-opus")
#define TCC_OMX_COMPONENT_OPUSDEC_NAME	("OMX.TCC.opusdec")
#define TCC_OMX_COMPONENT_OPUSDEC_ROLE	("audio_decoder.opus")

#define TCC_OPUSDEC_LIB_NAME	("libtccopusdec.so")

//! Data structure for OPUS decoding
typedef struct OpusDecoderPrivate
{
  int isHeader;
  void* (*m_pfCalloc) (unsigned int, unsigned int);					//!< cdk_ogg_malloc

  //!   reserved for future needs
  int	reserved[32-2];
} OpusDecoderPrivate;

// opus decoder class
DERIVEDCLASS(omx_opusdec_component_PrivateType, omx_audiodec_component_PrivateType)
#define omx_opusdec_component_PrivateType_FIELDS omx_audiodec_component_PrivateType_FIELDS \
	OMX_AUDIO_PARAM_OPUSTYPE stAudioOpus;
ENDCLASS(omx_opusdec_component_PrivateType)

#endif /* OMX_OPUSDEC_COMPONENT_H_ */
