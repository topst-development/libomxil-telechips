/**

  @file omx_aacdec_component.h

  This file is header of AAC decoder component.

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

#ifndef OMX_AACDEC_COMPONENT_H_
#define OMX_AACDEC_COMPONENT_H_

#include <omx_audiodec_component.h>

#define TCC_OMX_COMPONENT_AACDEC_MIME	("audio/aac")
#define TCC_OMX_COMPONENT_AACDEC_NAME	("OMX.TCC.aacdec")
#define TCC_OMX_COMPONENT_AACDEC_ROLE	("audio_decoder.aac")

#define TCC_AACDEC_LIB_NAME	("libtccaacdec.so")

//! Data structure for AAC decoding
typedef struct AacDecoderPrivate
{
	//!     AAC Object type : (2 : AAC_LC, 4: LTP, 5: SBR, 22: BSAC, ...)
	int		m_iAACObjectType;
	//!     AAC Stream Header type : ( 0 : RAW-AAC, 1: ADTS, 2: ADIF)
	int		m_iAACHeaderType;
	//!     m_iAACForceUpsampling -> deprecated
	int		m_iAACForceUpsampling;
	//!		upmix (mono to stereo) flag (0 : disable, 1: enable)
	//!     only, if( ( m_iAACForceUpmix == 1 ) && ( channel == mono ) ), then out_channel = 2;
	int		m_iAACForceUpmix;
	//!		Dynamic Range Control
	//!		Dynamic Range Control, Enable Dynamic Range Control (0 : disable (default), 1: enable)	
	int		m_iEnableDRC;
	//!		Dynamic Range Control, Scaling factor for boosting gain value, range: 0 (not apply) ~ 127 (fully apply)
	int		m_iDrcBoostFactor;
	//!		Dynamic Range Control, Scaling factor for cutting gain value, range: 0 (not apply) ~ 127 (fully apply)
	int		m_iDrcCutFactor;
	//!		Dynamic Range Control, Target reference level, range: 0 (full scale) ~ 127 (31.75 dB below full-scale)
	int		m_iDrcReferenceLevel;
	//!		Dynamic Range Control, Enable DVB specific heavy compression (aka RF mode), (0 : disable (default), 1: enable)
	int		m_iDrcHeavyCompression;
	//!		m_uiChannelMasking -> deprecated
	int		m_uiChannelMasking;
	//!		m_uiDisableHEAACDecoding -> deprecated
	int		m_uiDisableHEAACDecoding;
	//!		Disable signal level limiting. \n
	//!     1: Turn off PCM limiter, Otherwise: Auto-config. Enable limiter for all non-lowdelay configurations by default.
	int		m_uiDisablePCMLimiter;
	//!		RESERVED
	int		reserved[32-12];
} AacDecoderPrivate;

// for LATM Demuxer
typedef enum
{	
	// LATM/LOAS (Low Overhead Audio Stream): LATM with sync information
	TF_AAC_LOAS			= 0,	// default value

	// LATM (Low-overhead MPEG-4 Audio Transport Multiplex), without LOAS Sync-Layer, No random access is possible
	TF_AAC_LATM_MCP1	= 1,	// LATM wiht muxConfigPresent = 1
	TF_AAC_LATM_MCP0	= 2,	// LATM wiht muxConfigPresent = 0

	// ADTS (Audio Data Transport Stream)
	TF_AAC_ADTS			= 3,	

	// ADIF (Audio Data Interchange Format)
	TF_AAC_ADIF			= 4,	// not supported

	TF_UNKNOWN			= 0x7FFFFFFF	// Unknown format
}TransportFormat;

//! Callback Func
typedef struct latm_callback_func_t
{
	void* (*m_pfMalloc			) ( size_t );								//!< malloc
	void* (*m_pfNonCacheMalloc	) ( size_t );								//!< non-cacheable malloc
	void  (*m_pfFree			) ( void* );									//!< free
	void  (*m_pfNonCacheFree	) ( void* );									//!< non-cacheable free
	void* (*m_pfMemcpy			) ( void*, const void*, size_t );			//!< memcpy
	void  (*m_pfMemset			) ( void*, int, size_t );					//!< memset
	void* (*m_pfRealloc			) ( void*, size_t );						//!< realloc
	void* (*m_pfMemmove			) ( void*, const void*, size_t );			//!< memmove
	void* (*m_pfCalloc			) ( size_t , size_t );				//!< calloc
	int*  (*m_pfMemcmp			) ( const void* , const void*, size_t );	//!< memcmp

	int m_Reserved1[4];
} latm_callback_func_t;

typedef void* pfLatmParseInit(OMX_U8*, OMX_U32, OMX_S32*, OMX_S32*, void*, TransportFormat);
typedef int pfLatmParseGetFrame(void*,	OMX_U8*, OMX_S32, OMX_U8**, OMX_S32*, OMX_U32);
typedef int pfLatmParseGetHeaderType(void*);
typedef int pfLatmParseClose(void*);

// aac decoder class
DERIVEDCLASS(omx_aacdec_component_PrivateType, omx_audiodec_component_PrivateType)
#define omx_aacdec_component_PrivateType_FIELDS omx_audiodec_component_PrivateType_FIELDS \
	OMX_AUDIO_PARAM_AACPROFILETYPE stAudioAac; \
	latm_callback_func_t stCallbackFunc; \
	pfLatmParseInit  *pfLatmInit; \
	pfLatmParseGetFrame *pfLatmGetFrame; \
	pfLatmParseGetHeaderType *pfLatmGetHeaderType; \
	pfLatmParseClose *pfLatmClose; \
	OMX_U32 uiNeedLatmParsing; \
	OMX_U8* pAACRawData; \
	OMX_S32 iAACRawDataSize; \
	OMX_PTR pvSubParser;
ENDCLASS(omx_aacdec_component_PrivateType)

#endif /* OMX_AACDEC_COMPONENT_H_ */

