/**

  @file omx_audiodec_component.h

  This file is header of audio decoder component.

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

#ifndef OMX_AUDIODEC_COMPONENT_H_
#define OMX_AUDIODEC_COMPONENT_H_

#include <OMX_Types.h>
#include <OMX_Component.h>
#include <OMX_Core.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <omx_base_filter.h>
#include "TCCMemory.h"

#include "OMX_TCC_Index.h"

#include "tcc_audio_common.h"

#define OUTPUT_SPLIT_TIME_LIMIT		(50000)	// 50ms
#define OUTPUT_SILENT_TIME_LIMIT	(1000000)	// 1sec
#define MAX_DIFF_TIMESTAMP 			(100000) // 100msec

// audio decoder base class
DERIVEDCLASS(omx_audiodec_component_PrivateType, omx_base_filter_PrivateType)
#define omx_audiodec_component_PrivateType_FIELDS omx_base_filter_PrivateType_FIELDS \
	/** audio decoder functions */ \
	OMX_S32 (*pfCodecInit  ) (OMX_PTR pAudioDecComponentPrivate, OMX_BUFFERHEADERTYPE* pInbuffer); \
	OMX_S32 (*pfCodecDecode) (OMX_PTR pAudioDecComponentPrivate, OMX_BUFFERHEADERTYPE* pInbuffer, OMX_BUFFERHEADERTYPE* pOutbuffer); \
	OMX_S32 (*pfCodecFlush ) (OMX_PTR pAudioDecComponentPrivate, OMX_BUFFERHEADERTYPE* pInbuffer); \
	OMX_S32 (*pfCodecClose ) (OMX_PTR pAudioDecComponentPrivate); \
	OMX_S32 (*pfCdkAudioDec) (int iOpCode, OMX_S32* pHandle, void* pParam1, void* pParam2 ); \
	OMX_S32 (*pfSetInternalParam) (OMX_PTR pAudioDecComponentPrivate); \
	/** */ \
	adec_init_t stADecInit; \
	audio_pcminfo_t stPcmInfo; \
	audio_streaminfo_t stStreamInfo; \
	OMX_S32 hDecoderHandle; \
	OMX_S32 iDecodedSamplePerCh; \
	/** for input buffer management */ \
	OMX_U8*	pucStreamBuffer; \
	OMX_S32	iStreamBufferSize; \
	OMX_S32 iEmptyBufferDone; \
	OMX_S32 iMinStreamSize; \	
	OMX_TICKS iStartTS; \
	OMX_TICKS iPrevTS; \
	OMX_TICKS iPrevOriginalTS; \
	OMX_TICKS iDuration; \
	OMX_U32	uiInputBufferCount; \
	OMX_BOOL isEndOfFile; \
	OMX_BOOL isDecoderReady; \
	OMX_TICKS uiNumSamplesOutput; \	
	/** for output buffer split & silence insertion */ \
	OMX_BOOL isPcmSplitEnabled; \
	OMX_U8* pRest; \
	OMX_U32 iRestSamples; \
	OMX_U32 iSplitLength; \
	OMX_U32 iSplitPosition; \
	OMX_U32 uiSplitBufferSampleCapacity; \
	OMX_BOOL isSilenceInsertionEnable; \
	OMX_TICKS iMuteSamples; \
	OMX_TICKS iGuardSamples; \
	OMX_U32 iNumOfSeek; \	
	OMX_BOOL isPrevDecFail; \
	/** for audio decoder*/ \	
	OMX_PTR pvCodecSpecific; \
	OMX_U32 uiAudioProcessMode; \
	OMX_U32 uiCodecType; \
	OMX_BOOL isBitstreamOut; \
	/** for output port*/ \	
	OMX_U32 nMaxSupportSamplerate; \
	OMX_AUDIO_PARAM_PCMMODETYPE stOutPcmMode; \
	decoder_func_list_t* pDecFunctionList; \
	OMX_PTR pAudioDLLModule; \
	OMX_S32 iDebugDuration;
ENDCLASS(omx_audiodec_component_PrivateType)

// common functions
OMX_ERRORTYPE omx_audiodec_component_Init(
	OMX_COMPONENTTYPE *pOpenmaxStandComp, 
	OMX_STRING cComponentName, 
	OMX_U32 nComponentPrivateSize) ;

OMX_ERRORTYPE omx_audiodec_component_library_Load(
	omx_audiodec_component_PrivateType* pADecCompPrivate, 
	OMX_STRING cAudioLibName);

void omx_audiodec_component_BufferMgmtCallback(
	OMX_COMPONENTTYPE *openmaxStandComp, 
	OMX_BUFFERHEADERTYPE* inputbuffer, 
	OMX_BUFFERHEADERTYPE* outputbuffer);

OMX_ERRORTYPE omx_audiodec_component_LibInit(omx_audiodec_component_PrivateType* omx_audiodec_component_Private);

OMX_ERRORTYPE omx_audiodec_component_LibDeinit(omx_audiodec_component_PrivateType* omx_audiodec_component_Private);

OMX_ERRORTYPE omx_audiodec_component_MessageHandler(
	OMX_COMPONENTTYPE* openmaxStandComp, 
	internalRequestMessageType *message);

OMX_ERRORTYPE omx_audiodec_component_GetExtensionIndex(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_STRING cParameterName,
  OMX_OUT OMX_INDEXTYPE* pIndexType);

OMX_ERRORTYPE omx_audiodec_component_Destructor(OMX_COMPONENTTYPE *openmaxStandComp);

OMX_ERRORTYPE omx_audiodec_component_GetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_INOUT OMX_PTR ComponentParameterStructure);

OMX_ERRORTYPE omx_audiodec_component_SetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_IN  OMX_PTR ComponentParameterStructure);


/* common audio decoder functions */
OMX_S32 InitDecoder(omx_audiodec_component_PrivateType* pAudioDecComponentPrivate);
OMX_S32 DecodeFrame(omx_audiodec_component_PrivateType* pAudioDecComponentPrivate, OMX_BUFFERHEADERTYPE* pInputBuffer, OMX_BUFFERHEADERTYPE* pOutputBuffer);
OMX_S32 FlushCodec(omx_audiodec_component_PrivateType* pAudioDecComponentPrivate, OMX_BUFFERHEADERTYPE* pInputBuffer);
OMX_S32 CloseCodec(omx_audiodec_component_PrivateType* pAudioDecComponentPrivate);


#endif /* OMX_AUDIODEC_COMPONENT_H_ */

