/**

  @file omx_flacdec_component.h

  This file is header of FLAC decoder component.

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

#ifndef OMX_FLACDEC_COMPONENT_H_
#define OMX_FLACDEC_COMPONENT_H_

#include <omx_audiodec_component.h>

#define TCC_OMX_COMPONENT_FLACDEC_MIME	("audio/flac")
#define TCC_OMX_COMPONENT_FLACDEC_NAME	("OMX.TCC.flacdec")
#define TCC_OMX_COMPONENT_FLACDEC_ROLE	("audio_decoder.flac")

#define TCC_FLACDEC_LIB_NAME	("libtccflacdec.so")

//! Data structure for FLAC decoding
typedef struct FlacDecoderPrivate
{
	//!		Init mode select ( 0: file decoding mode,  1: buffer decoding mode, 2: demuxer&decode mode)
	int		m_iFlacInitMode;
	//!		reserved for future needs
	int		reserved[32-1];
} FlacDecoderPrivate;

// flac decoder class
DERIVEDCLASS(omx_flacdec_component_PrivateType, omx_audiodec_component_PrivateType)
#define omx_flacdec_component_PrivateType_FIELDS omx_audiodec_component_PrivateType_FIELDS \
	OMX_AUDIO_PARAM_FLACTYPE stAudioFlac;
ENDCLASS(omx_flacdec_component_PrivateType)

#endif /* OMX_FLACDEC_COMPONENT_H_ */

