/**

  @file omx_mp3dec_component.h

  This file is header of MP3 decoder component.

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

#ifndef OMX_MP3DEC_COMPONENT_H_
#define OMX_MP3DEC_COMPONENT_H_

#include <omx_audiodec_component.h>

#define TCC_OMX_COMPONENT_MP3DEC_MIME	("audio/mpeg")
#define TCC_OMX_COMPONENT_MP3DEC_NAME	("OMX.TCC.mp3dec")
#define TCC_OMX_COMPONENT_MP3DEC_ROLE	("audio_decoder.mp3")

#define TCC_MP3DEC_LIB_NAME	("libtccmp3dec.so")

//! Data structure for MPEG Audio Layer 3/2/1 decoding
typedef struct Mp3DecoderPrivate
{
	//!		DAB mode selection for layer 2 decoding ( 0: OFF,  1: ON)
	int		m_iDABMode;

	//!		reserved for future needs
	int		reserved[32-1];
} Mp3DecoderPrivate;

// mp3 decoder class
DERIVEDCLASS(omx_mp3dec_component_PrivateType, omx_audiodec_component_PrivateType)
#define omx_mp3dec_component_PrivateType_FIELDS omx_audiodec_component_PrivateType_FIELDS \
	OMX_AUDIO_PARAM_MP3TYPE stAudioMp3;
ENDCLASS(omx_mp3dec_component_PrivateType)

#endif /* OMX_MP3DEC_COMPONENT_H_ */

