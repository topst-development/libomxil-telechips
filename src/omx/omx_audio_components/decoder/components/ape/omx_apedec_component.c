/**

  @file omx_apedec_component.c

  This component implement APE decoder.

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

#include <omxcore.h>
#include <omx_base_audio_port.h>

#include <omx_apedec_component.h>
#include <OMX_TCC_Index.h>
#include "TCCFile.h"
#include "TCCMemory.h"

#ifdef HAVE_ANDROID_OS
#define USE_EXTERNAL_BUFFER 0
#define LOG_TAG	"OMX_TCC_APEDEC"
#include <utils/Log.h>

#define LOGV(...)	ALOGV(__VA_ARGS__)
#define LOGD(...)	ALOGD(__VA_ARGS__)
#define LOGI(...)	ALOGI(__VA_ARGS__)
#define LOGW(...)	ALOGW(__VA_ARGS__)
#define LOGE(...)	ALOGE(__VA_ARGS__)
#else
#include "omx_comp_debug_levels.h"

#define LOGV(...)	{\
		if (gs_iDbgmsgLevel_Adec >= 5) {\
			tcc_printf(T_DEFAULT "[OMXAPEDEC:V]" __VA_ARGS__);\
			tcc_printf(" - [%s - %d]\n"T_DEFAULT,__func__,__LINE__);\
		}\
	}
#define LOGD(...)	{\
		if (gs_iDbgmsgLevel_Adec >= 4) {\
			tcc_printf(TC_CYAN "[OMXAPEDEC:D]" __VA_ARGS__);\
			tcc_printf(" - [%s - %d]\n"T_DEFAULT,__func__,__LINE__);\
		}\
	}
#define LOGI(...)	{\
		if (gs_iDbgmsgLevel_Adec >= 3) {\
			tcc_printf(TC_GREEN "[OMXAPEDEC:I]" __VA_ARGS__);\
			tcc_printf(" - [%s - %d]\n"T_DEFAULT,__func__,__LINE__);\
		}\
	}
#define LOGW(...)	{\
		if (gs_iDbgmsgLevel_Adec >= 2) {\
			tcc_printf(TC_MAGENTA "[OMXAPEDEC:W]" __VA_ARGS__);\
			tcc_printf(" - [%s - %d]\n"T_DEFAULT,__func__,__LINE__);\
		}\
	}
#define LOGE(...)	{\
		if (gs_iDbgmsgLevel_Adec >= 1) {\
			tcc_printf(TC_RED "[OMXAPEDEC:E]" __VA_ARGS__);\
			tcc_printf(" - [%s - %d]\n"T_DEFAULT,__func__,__LINE__);\
		}\
	}
#endif /* HAVE_ANDROID_OS */

#define CODEC_SPECIFIC_MARKER (0x12345678)
#define CODEC_SPECIFIC_MARKER_OFFSET (8)
#define CODEC_SPECIFIC_INFO_OFFSET (4)

typedef struct ape_dmx_exinput_t
{
	unsigned int m_uiInputBufferByteOffset;
	unsigned int m_uiCurrentFrames;
} ape_dmx_exinput_t;

typedef struct ape_dmx_exinfo_t
{
	short     m_nVersion;
	short     padding1;
	unsigned short nCompressionLevel;
	unsigned short nFormatFlags;
	unsigned short nBitsPerSample;
	unsigned short temp_align;
	unsigned int finalframeblocks;
	unsigned int blocksperframe;
	unsigned int totalframes;
} ape_dmx_exinfo_t;

static OMX_S32 InitApeDec(omx_apedec_component_PrivateType* pApeDecCompPrivate, OMX_BUFFERHEADERTYPE* pInputBuffer)
{
	OMX_S32 nRetValue	= 0;
	ape_dmx_exinfo_t stInitExinfo;

	/* setting callback functions */
	// set already

	/* set pcm-info struct */
	// set already

	/* set stream-info struct */
	// set already

	memset(&stInitExinfo,0,sizeof(ape_dmx_exinfo_t));

	if (!(pInputBuffer->nFilledLen > 10))
	{
		unsigned char *pp = pInputBuffer->pBuffer;
		stInitExinfo.m_nVersion	= (pp[1] << 8) | pp[0];
		stInitExinfo.padding1	= 0;
		stInitExinfo.nCompressionLevel = (pp[3] << 8) | pp[2];

		stInitExinfo.nFormatFlags 	= (pp[5] << 8) | pp[4];
		stInitExinfo.nBitsPerSample	= pApeDecCompPrivate->stAudioApe.nBitPerSample;
		stInitExinfo.totalframes		= 0;	//6;
		stInitExinfo.finalframeblocks	= 0;	//2048;
		stInitExinfo.blocksperframe	= 0;

		// setting extra info
		pApeDecCompPrivate->stADecInit.m_pucExtraData = &stInitExinfo;			//pInputBuffer->pBuffer;
		pApeDecCompPrivate->stADecInit.m_iExtraDataLen = sizeof(ape_dmx_exinfo_t);	//pInputBuffer->nFilledLen;
	}

	nRetValue = pApeDecCompPrivate->pfCdkAudioDec(AUDIO_INIT, &pApeDecCompPrivate->hDecoderHandle, &pApeDecCompPrivate->stADecInit, NULL);

	LOGI("%s result %ld",__func__, nRetValue);

	return nRetValue;

}

static OMX_S32 DecodeApe(omx_apedec_component_PrivateType* pApeDecCompPrivate, OMX_BUFFERHEADERTYPE* pInputBuffer, OMX_BUFFERHEADERTYPE* pOutputBuffer)
{
	OMX_S32 nRetValue = TCAS_SUCCESS;
	int ByteofPCMSample = (pApeDecCompPrivate->stPcmInfo.m_uiBitsPerSample <= 16) ? 2 : 4;
	int iOutFrameSize;

	iOutFrameSize = 0;
	pApeDecCompPrivate->iDecodedSamplePerCh = 0;

	//pApeDecCompPrivate->stStreamInfo.m_pvExtraInfo = pApeDecCompPrivate->pvCodecSpecific;

	LOGD("%s: input length %d", __func__, pApeDecCompPrivate->stStreamInfo.m_iStreamLength);

	while(pApeDecCompPrivate->stStreamInfo.m_iStreamLength > 4/*pPrivate->iMinStreamSize*/)
	{
		pApeDecCompPrivate->stPcmInfo.m_pvChannel[0]  = (void *)(pOutputBuffer->pBuffer + iOutFrameSize);

		nRetValue = pApeDecCompPrivate->pfCdkAudioDec(AUDIO_DECODE, &pApeDecCompPrivate->hDecoderHandle,
																	 &pApeDecCompPrivate->stStreamInfo, &pApeDecCompPrivate->stPcmInfo);

		if(nRetValue == TCAS_SUCCESS)
		{
			iOutFrameSize += pApeDecCompPrivate->stPcmInfo.m_uiNumberOfChannel * pApeDecCompPrivate->stPcmInfo.m_uiSamplesPerChannel * ByteofPCMSample;
			pApeDecCompPrivate->iDecodedSamplePerCh += pApeDecCompPrivate->stPcmInfo.m_uiSamplesPerChannel;
			//break;
		}
		else
		{
			if(nRetValue != TCAS_ERROR_SKIP_FRAME){
				break;
			}
		}
	}
	pApeDecCompPrivate->stStreamInfo.m_iStreamLength = 0;

	return nRetValue;
}

static OMX_S32 FlushApeDec(omx_apedec_component_PrivateType* pApeDecCompPrivate,  OMX_BUFFERHEADERTYPE* pInputBuffer)
{
	ape_dmx_exinput_t *codecSpecific = 0;
	OMX_U8* p = pInputBuffer->pBuffer;

	LOGD("In %s",__func__);

	if(*((OMX_U32*)(p+pInputBuffer->nFilledLen-CODEC_SPECIFIC_MARKER_OFFSET)) == CODEC_SPECIFIC_MARKER)
	{
		LOGD(" %s codec specific data!", __func__);
		codecSpecific = (ape_dmx_exinput_t *)(*((OMX_U32*)(p+pInputBuffer->nFilledLen-CODEC_SPECIFIC_INFO_OFFSET)));
		pInputBuffer->nFilledLen -= CODEC_SPECIFIC_MARKER_OFFSET;
	}
	else
	{
		LOGW(" %s No codec specific data", __func__);
	}

	pApeDecCompPrivate->pvCodecSpecific = codecSpecific;
	pApeDecCompPrivate->pfCdkAudioDec(AUDIO_FLUSH, &pApeDecCompPrivate->hDecoderHandle, NULL, NULL);

	LOGD("Out %s", __func__);

	return 0;
}

static OMX_S32 SetInternalParam(omx_apedec_component_PrivateType* pApeDecCompPrivate)
{
	/* reset OMX_AUDIO_PARAM_PCMMODETYPE Info */
	pApeDecCompPrivate->stOutPcmMode.nChannels = pApeDecCompPrivate->stPcmInfo.m_uiNumberOfChannel;
	pApeDecCompPrivate->stOutPcmMode.nSamplingRate = pApeDecCompPrivate->stPcmInfo.m_eSampleRate;

	/* reset OMX_AUDIO_PARAM_APETYPE Info */
	pApeDecCompPrivate->stAudioApe.nSampleRate = pApeDecCompPrivate->stPcmInfo.m_eSampleRate;
	pApeDecCompPrivate->stAudioApe.nChannels = pApeDecCompPrivate->stPcmInfo.m_uiNumberOfChannel;
	return 0;
}

/** this function sets the parameter values regarding audio format & index */
static OMX_ERRORTYPE omx_apedec_component_SetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_IN  OMX_PTR ComponentParameterStructure)
{

	OMX_ERRORTYPE eError = OMX_ErrorNone;
	OMX_PARAM_COMPONENTROLETYPE * pComponentRole;
	OMX_U32 portIndex;

	/* Check which structure we are being fed and make control its header */
	OMX_COMPONENTTYPE *pOpenmaxStandComp = (OMX_COMPONENTTYPE *)hComponent;
	omx_apedec_component_PrivateType* pApeDecCompPrivate = pOpenmaxStandComp->pComponentPrivate;

	if (ComponentParameterStructure == NULL)
	{
		return OMX_ErrorBadParameter;
	}

	LOGD(" %s nParamIndex [0x%x]", __func__, nParamIndex);

	switch(nParamIndex)
	{
		case OMX_IndexParamAudioPcm:
		{
			OMX_AUDIO_PARAM_PCMMODETYPE* pAudioSrcPcmMode = (OMX_AUDIO_PARAM_PCMMODETYPE*)ComponentParameterStructure;
			portIndex = pAudioSrcPcmMode->nPortIndex;

			if (pAudioSrcPcmMode->nSamplingRate > pApeDecCompPrivate->nMaxSupportSamplerate)
			{
				LOGE(" This system doesn't support over %luHz sampling rate!", pApeDecCompPrivate->nMaxSupportSamplerate);
				eError = OMX_ErrorUnsupportedSetting;
				break;
			}

			/*Check Structure Header and verify component state*/
			eError = omx_base_component_ParameterSanityCheck(hComponent, portIndex, pAudioSrcPcmMode, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));
			if (eError!= OMX_ErrorNone)
			{
				LOGE("In %s Parameter Check Error=%x",__func__,eError);
				break;
			}

			if (pAudioSrcPcmMode->nPortIndex == 0)
			{
				eError = OMX_ErrorBadPortIndex;
				break;
			}
			else
			{
				LOGD("@@@@ :: nChannels = %lu, nBitPerSample = %lu, nSamplingRate = %lu",
								pAudioSrcPcmMode->nChannels, pAudioSrcPcmMode->nBitPerSample, pAudioSrcPcmMode->nSamplingRate);
				(void)memcpy(&pApeDecCompPrivate->stOutPcmMode, pAudioSrcPcmMode, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));
			}
			break;
		}

		case OMX_IndexParamStandardComponentRole:
		{
			pComponentRole = (OMX_PARAM_COMPONENTROLETYPE*)ComponentParameterStructure;
			if (strncmp( (char*) pComponentRole->cRole, TCC_OMX_COMPONENT_APEDEC_ROLE, sizeof(TCC_OMX_COMPONENT_APEDEC_ROLE)))
			{
				eError = OMX_ErrorBadParameter;
			}
			break;
		}

		// codec specific parameters -------------------------------------------------------
		case OMX_IndexParamAudioAPE:
		{
			OMX_AUDIO_PARAM_PCMMODETYPE *pAudioParamPCM = &pApeDecCompPrivate->stOutPcmMode;
			portIndex = pApeDecCompPrivate->stAudioApe.nPortIndex;
			eError = omx_base_component_ParameterSanityCheck(hComponent,portIndex,ComponentParameterStructure,sizeof(OMX_AUDIO_PARAM_APETYPE));
			if(eError!=OMX_ErrorNone)
			{
				LOGE("In %s Parameter Check Error=%x",__func__,eError);
				break;
			}

			if (((OMX_AUDIO_PARAM_APETYPE *)(ComponentParameterStructure))->nPortIndex != portIndex)
			{
				eError = OMX_ErrorBadPortIndex;
				break;
			}

			(void)memcpy(&pApeDecCompPrivate->stAudioApe, ComponentParameterStructure, sizeof(OMX_AUDIO_PARAM_APETYPE));

			pAudioParamPCM->nSamplingRate = pApeDecCompPrivate->stAudioApe.nSampleRate;
			if (pApeDecCompPrivate->stAudioApe.nChannels == 1)
			{
				pAudioParamPCM->nChannels = 1;
			}

			LOGD("setparam samplerate = %lu, channels = %lu", pApeDecCompPrivate->stAudioApe.nSampleRate, pApeDecCompPrivate->stAudioApe.nChannels);

			break;
		}

		default: /*Call the base component function*/
			return omx_audiodec_component_SetParameter(hComponent, nParamIndex, ComponentParameterStructure);

	}
	return eError;
}

/** this function gets the parameters regarding audio formats and index */
static OMX_ERRORTYPE omx_apedec_component_GetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_INOUT OMX_PTR ComponentParameterStructure)
{

	OMX_ERRORTYPE eError = OMX_ErrorNone;
	OMX_COMPONENTTYPE *pOpenmaxStandComp = (OMX_COMPONENTTYPE *)hComponent;
	omx_apedec_component_PrivateType* pApeDecCompPrivate = pOpenmaxStandComp->pComponentPrivate;
	OMX_PARAM_COMPONENTROLETYPE * pComponentRole;

	if (ComponentParameterStructure == NULL)
	{
		return OMX_ErrorBadParameter;
	}

	LOGD(" %s nParamIndex [0x%x]", __func__, nParamIndex);
	/* Check which structure we are being fed and fill its header */

	switch(nParamIndex)
	{
		case OMX_IndexParamAudioPcm:
		{
			if ((eError = checkHeader(ComponentParameterStructure, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE))) != OMX_ErrorNone)
			{
				break;
			}
			if (((OMX_AUDIO_PARAM_PCMMODETYPE *)(ComponentParameterStructure))->nPortIndex == 1)
			{
				// output is PCM
				(void)memcpy(ComponentParameterStructure, &pApeDecCompPrivate->stOutPcmMode, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));
			}
			else
			{
				eError = OMX_ErrorBadPortIndex;
			}

			break;
		}

		case OMX_IndexParamAudioAPE:
		{
			if ((eError = checkHeader(ComponentParameterStructure, sizeof(OMX_AUDIO_PARAM_APETYPE))) != OMX_ErrorNone)
			{
				break;
			}

			if (((OMX_AUDIO_PARAM_APETYPE *)(ComponentParameterStructure))->nPortIndex != pApeDecCompPrivate->stAudioApe.nPortIndex)
			{
				return OMX_ErrorBadPortIndex;
			}

			(void)memcpy(ComponentParameterStructure, &pApeDecCompPrivate->stAudioApe, sizeof(OMX_AUDIO_PARAM_APETYPE));
			LOGD("getparam samplerate = %lu, channels = %lu", pApeDecCompPrivate->stAudioApe.nSampleRate, pApeDecCompPrivate->stAudioApe.nChannels);
			break;
		}

		case OMX_IndexParamStandardComponentRole:
		{
			pComponentRole = (OMX_PARAM_COMPONENTROLETYPE*)ComponentParameterStructure;
			if ((eError = checkHeader(ComponentParameterStructure, sizeof(OMX_PARAM_COMPONENTROLETYPE))) != OMX_ErrorNone)
			{
				break;
			}

			(void)strncpy( (char*) pComponentRole->cRole, TCC_OMX_COMPONENT_APEDEC_ROLE, sizeof(TCC_OMX_COMPONENT_APEDEC_ROLE));
			break;
		}

		default: /*Call the audiodec component function*/
			return omx_audiodec_component_GetParameter(hComponent, nParamIndex, ComponentParameterStructure);

	}

	return eError;

}


OMX_ERRORTYPE OMX_ComponentInit(OMX_COMPONENTTYPE *pOpenmaxStandComp, OMX_STRING cComponentName)
{
	OMX_ERRORTYPE eError = OMX_ErrorNone;
	omx_base_audio_PortType *inPort, *outPort;
	omx_apedec_component_PrivateType* pApeDecCompPrivate;

	if(strncmp(cComponentName, TCC_OMX_COMPONENT_APEDEC_NAME, sizeof(TCC_OMX_COMPONENT_APEDEC_NAME)))
	{
		// IL client specified an invalid component name
		LOGE("OMX_ErrorInvalidComponentName %s", cComponentName);
		eError = OMX_ErrorInvalidComponentName;
		goto EXIT;
	}

	eError = omx_audiodec_component_Init(pOpenmaxStandComp, cComponentName, sizeof(omx_apedec_component_PrivateType));
	if(eError != OMX_ErrorNone)
	{
		goto EXIT;
	}

	pApeDecCompPrivate = pOpenmaxStandComp->pComponentPrivate;

	/** parameters related to output port */
	outPort = (omx_base_audio_PortType *) pApeDecCompPrivate->ports[OMX_BASE_FILTER_OUTPUTPORT_INDEX];
	outPort->sPortParam.nBufferSize = 1152*1024*2;	//AUDIO_DEC_OUT_BUFFER_SIZE;		//1024*1024

	/** parameters related to output audio param port */
	// use default setting (in omx_audiodec_component_Init)

	/** parameters related to input port */
	inPort = (omx_base_audio_PortType *) pApeDecCompPrivate->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];
	inPort->sPortParam.nBufferSize = 2048*1024*2;	//DEFAULT_IN_BUFFER_SIZE*2;		//DEFAULT_IN_BUFFER_SIZE = 32 * 1024
	(void)strncpy(inPort->sPortParam.format.audio.cMIMEType, TCC_OMX_COMPONENT_APEDEC_MIME, sizeof(TCC_OMX_COMPONENT_APEDEC_MIME));
	inPort->sPortParam.format.audio.eEncoding = OMX_AUDIO_CodingAPE;
	inPort->sAudioParam.eEncoding = OMX_AUDIO_CodingAPE;

	/** parameters related to input audio param port */
	setHeader(&pApeDecCompPrivate->stAudioApe, sizeof(OMX_AUDIO_PARAM_APETYPE));
	pApeDecCompPrivate->stAudioApe.nPortIndex = 0;
	pApeDecCompPrivate->stAudioApe.nChannels = 2;
	pApeDecCompPrivate->stAudioApe.nBitRate = 0;
	pApeDecCompPrivate->stAudioApe.nSampleRate = 44100;
	pApeDecCompPrivate->stAudioApe.eChannelMode = OMX_AUDIO_ChannelModeStereo;

	/** load audio decoder library */
	if (omx_audiodec_component_library_Load(pApeDecCompPrivate, TCC_APEDEC_LIB_NAME) != OMX_ErrorNone)
	{
		eError = OMX_ErrorInsufficientResources;
		goto EXIT;
	}

	/** overwrite omx component functions */
	pOpenmaxStandComp->SetParameter = omx_apedec_component_SetParameter;
	pOpenmaxStandComp->GetParameter = omx_apedec_component_GetParameter;
	pApeDecCompPrivate->pfSetInternalParam = SetInternalParam;

	/** overwrite audio decoder functions */
	pApeDecCompPrivate->pfCodecInit = InitApeDec; // use own init function
	pApeDecCompPrivate->pfCodecFlush = FlushApeDec;
	pApeDecCompPrivate->pfCodecDecode = DecodeApe;

	pApeDecCompPrivate->uiAudioProcessMode = AUDIO_NORMAL_MODE; /* decoded pcm mode */
	pApeDecCompPrivate->uiCodecType = AUDIO_ID_APE;
	pApeDecCompPrivate->iMinStreamSize = 4;

	LOGD("constructor of ape decoder component is completed!");

EXIT:
	return eError;

}

