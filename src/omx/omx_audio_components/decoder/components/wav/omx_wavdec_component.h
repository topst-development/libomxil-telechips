/**

  @file omx_wavdec_component.h

  This file is header of WAV decoder component.

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

#ifndef OMX_WAVDEC_COMPONENT_H_
#define OMX_WAVDEC_COMPONENT_H_

#include <omx_audiodec_component.h>

#define TCC_OMX_COMPONENT_WAVDEC_MIME	("audio/x-pcm")
#define TCC_OMX_COMPONENT_WAVDEC_NAME	("OMX.TCC.pcmdec")
#define TCC_OMX_COMPONENT_WAVDEC_ROLE	("audio_decoder.pcm_dec")

#define TCC_WAVDEC_LIB_NAME	("libtccpcmdec.so")

//! Data structure for WAV decoding
typedef struct WavDecoderPrivate
{
	//!		Format ID
	int		m_iWAVForm_TagID;
	//!		Block size in bytes of the WAV stream.	  
	unsigned int	m_uiNBlockAlign;		
	//!		Endian  ( 0 : little endian,  1: big endian )
	int		m_iEndian;	
	//!		Container type ( 0 : Audio,  1: MPG,  2 : TS)
	int		m_iContainerType;	
	//!		reserved for future needs
	int		reserved[32-4];
} WavDecoderPrivate;

// wav decoder class
DERIVEDCLASS(omx_wavdec_component_PrivateType, omx_audiodec_component_PrivateType)
#define omx_wavdec_component_PrivateType_FIELDS omx_audiodec_component_PrivateType_FIELDS \
	OMX_AUDIO_PARAM_PCMMODETYPE stAudioWav;
ENDCLASS(omx_wavdec_component_PrivateType)

#endif /* OMX_WAVDEC_COMPONENT_H_ */

