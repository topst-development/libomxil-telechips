/**

  @file omx_amrnbdec_component.c

  This component implement amrnb decoder.

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

#include <omx_amrnbdec_component.h>
#include <OMX_TCC_Index.h>
#include "TCCFile.h"
#include "TCCMemory.h"

#ifdef HAVE_ANDROID_OS
#define USE_EXTERNAL_BUFFER 0
#define LOG_TAG	"OMX_TCC_AMRNBDEC"
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
			tcc_printf(T_DEFAULT "[OMXAMRNBDEC:V]" __VA_ARGS__);\
			tcc_printf(" - [%s - %d]\n"T_DEFAULT,__func__,__LINE__);\
		}\
	}
#define LOGD(...)	{\
		if (gs_iDbgmsgLevel_Adec >= 4) {\
			tcc_printf(TC_CYAN "[OMXAMRNBDEC:D]" __VA_ARGS__);\
			tcc_printf(" - [%s - %d]\n"T_DEFAULT,__func__,__LINE__);\
		}\
	}
#define LOGI(...)	{\
		if (gs_iDbgmsgLevel_Adec >= 3) {\
			tcc_printf(TC_GREEN "[OMXAMRNBDEC:I]" __VA_ARGS__);\
			tcc_printf(" - [%s - %d]\n"T_DEFAULT,__func__,__LINE__);\
		}\
	}
#define LOGW(...)	{\
		if (gs_iDbgmsgLevel_Adec >= 2) {\
			tcc_printf(TC_MAGENTA "[OMXAMRNBDEC:W]" __VA_ARGS__);\
			tcc_printf(" - [%s - %d]\n"T_DEFAULT,__func__,__LINE__);\
		}\
	}
#define LOGE(...)	{\
		if (gs_iDbgmsgLevel_Adec >= 1) {\
			tcc_printf(TC_RED "[OMXAMRNBDEC:E]" __VA_ARGS__);\
			tcc_printf(" - [%s - %d]\n"T_DEFAULT,__func__,__LINE__);\
		}\
	}
#endif /* HAVE_ANDROID_OS */

static OMX_S32 InitAmrnbDec(omx_amrnbdec_component_PrivateType* pAmrnbDecCompPrivate, OMX_BUFFERHEADERTYPE* pInputBuffer)
{
	OMX_S32 nRetValue	= 0;

	/* setting callback functions */
	// set already

	/* set pcm-info struct */
	// set already

	/* set stream-info struct */
	pAmrnbDecCompPrivate->stStreamInfo.m_eSampleRate = 8000;
	pAmrnbDecCompPrivate->stStreamInfo.m_uiNumberOfChannel = 1;

	nRetValue = pAmrnbDecCompPrivate->pfCdkAudioDec(AUDIO_INIT, &pAmrnbDecCompPrivate->hDecoderHandle, &pAmrnbDecCompPrivate->stADecInit, NULL);

	LOGI("%s result %ld",__func__, nRetValue);

	return nRetValue;

}

static OMX_S32 DecodeAmrnb(omx_amrnbdec_component_PrivateType* pAmrnbDecCompPrivate, OMX_BUFFERHEADERTYPE* pInputBuffer, OMX_BUFFERHEADERTYPE* pOutputBuffer)
{
	OMX_S32 iOutFrameSize, nRetValue = TCAS_SUCCESS;

	iOutFrameSize = 0;
	pAmrnbDecCompPrivate->iDecodedSamplePerCh = 0;

	LOGD("%s: input length %d", __func__, pAmrnbDecCompPrivate->stStreamInfo.m_iStreamLength);

	while(pAmrnbDecCompPrivate->stStreamInfo.m_iStreamLength > 4/*pPrivate->iMinStreamSize*/)
	{
		pAmrnbDecCompPrivate->stPcmInfo.m_pvChannel[0]  = (void *)(pOutputBuffer->pBuffer + iOutFrameSize);

		nRetValue = pAmrnbDecCompPrivate->pfCdkAudioDec(AUDIO_DECODE, &pAmrnbDecCompPrivate->hDecoderHandle,
																		&pAmrnbDecCompPrivate->stStreamInfo, &pAmrnbDecCompPrivate->stPcmInfo);

		if(nRetValue == TCAS_SUCCESS)
		{
			iOutFrameSize += pAmrnbDecCompPrivate->stPcmInfo.m_uiNumberOfChannel * pAmrnbDecCompPrivate->stPcmInfo.m_uiSamplesPerChannel * sizeof(short);
			pAmrnbDecCompPrivate->iDecodedSamplePerCh += pAmrnbDecCompPrivate->stPcmInfo.m_uiSamplesPerChannel;
		//	break;
		}
		else
		{
			if(nRetValue != TCAS_ERROR_SKIP_FRAME){
				break;
			}
		}
	}
	pAmrnbDecCompPrivate->stStreamInfo.m_iStreamLength = 0;

	return nRetValue;
}

static OMX_S32 FlushAmrnbDec(omx_amrnbdec_component_PrivateType* pAmrnbDecCompPrivate,  OMX_BUFFERHEADERTYPE* pInputBuffer)
{
	LOGD("%s ",__func__);
	pAmrnbDecCompPrivate->pfCdkAudioDec(AUDIO_FLUSH, &pAmrnbDecCompPrivate->hDecoderHandle, NULL, NULL);
	return 0;
}

static OMX_S32 SetInternalParam(omx_amrnbdec_component_PrivateType* pAmrnbDecCompPrivate)
{
	/* reset OMX_AUDIO_PARAM_PCMMODETYPE Info */
	pAmrnbDecCompPrivate->stOutPcmMode.nChannels = pAmrnbDecCompPrivate->stPcmInfo.m_uiNumberOfChannel;
	pAmrnbDecCompPrivate->stOutPcmMode.nSamplingRate = pAmrnbDecCompPrivate->stPcmInfo.m_eSampleRate;
	return 0;
}

/** this function sets the parameter values regarding audio format & index */
static OMX_ERRORTYPE omx_amrnbdec_component_SetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_IN  OMX_PTR ComponentParameterStructure)
{

	OMX_ERRORTYPE eError = OMX_ErrorNone;
	OMX_PARAM_COMPONENTROLETYPE * pComponentRole;
	OMX_U32 portIndex;

	/* Check which structure we are being fed and make control its header */
	OMX_COMPONENTTYPE *pOpenmaxStandComp = (OMX_COMPONENTTYPE *)hComponent;
	omx_amrnbdec_component_PrivateType* pAmrnbDecCompPrivate = pOpenmaxStandComp->pComponentPrivate;

	if (ComponentParameterStructure == NULL)
	{
		LOGE("ComponentParameterStructure is NULL");
		eError = OMX_ErrorBadParameter;
	}
	else 
	{
		LOGD(" %s nParamIndex [0x%x]", __func__, nParamIndex);
		switch(nParamIndex)
		{
			case OMX_IndexParamAudioPcm:
			{
				OMX_AUDIO_PARAM_PCMMODETYPE* pAudioSrcPcmMode = (OMX_AUDIO_PARAM_PCMMODETYPE*)ComponentParameterStructure;
				portIndex = pAudioSrcPcmMode->nPortIndex;

				if (pAudioSrcPcmMode->nSamplingRate > pAmrnbDecCompPrivate->nMaxSupportSamplerate)
				{
					LOGE(" This system doesn't support over %luHz sampling rate!", pAmrnbDecCompPrivate->nMaxSupportSamplerate);
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
					LOGD("@@@@ :: nChannels = %lu, nBitPerSample = %lu, nSamplingRate = %lu", pAudioSrcPcmMode->nChannels, pAudioSrcPcmMode->nBitPerSample, pAudioSrcPcmMode->nSamplingRate);
					(void)memcpy(&pAmrnbDecCompPrivate->stOutPcmMode, pAudioSrcPcmMode, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));
				}
				break;
			}

			case OMX_IndexParamStandardComponentRole:
			{
				pComponentRole = (OMX_PARAM_COMPONENTROLETYPE*)ComponentParameterStructure;
				if (strncmp( (char*) pComponentRole->cRole, TCC_OMX_COMPONENT_AMRNBDEC_ROLE, sizeof(TCC_OMX_COMPONENT_AMRNBDEC_ROLE)))
				{
					eError = OMX_ErrorBadParameter;
				}
				break;
			}

			// codec specific parameters -------------------------------------------------------
			case OMX_IndexParamAudioAmr:
			{
				OMX_AUDIO_PARAM_PCMMODETYPE *pAudioParamPCM = &pAmrnbDecCompPrivate->stOutPcmMode;
				portIndex = pAmrnbDecCompPrivate->stAudioAmrnb.nPortIndex;
				eError = omx_base_component_ParameterSanityCheck(hComponent,portIndex,ComponentParameterStructure,sizeof(OMX_AUDIO_PARAM_AMRTYPE));
				if(eError!=OMX_ErrorNone)
				{
					LOGE("In %s Parameter Check Error=%x",__func__,eError);
					break;
				}

				if (((OMX_AUDIO_PARAM_AMRTYPE *)(ComponentParameterStructure))->nPortIndex != portIndex)
				{
					eError = OMX_ErrorBadPortIndex;
					break;
				}

				(void)memcpy(&pAmrnbDecCompPrivate->stAudioAmrnb, ComponentParameterStructure, sizeof(OMX_AUDIO_PARAM_AMRTYPE));
				pAudioParamPCM->nSamplingRate = 8000;
				pAudioParamPCM->nChannels = 1;

				pAmrnbDecCompPrivate->stStreamInfo.m_eSampleRate = 8000;
				pAmrnbDecCompPrivate->stStreamInfo.m_uiNumberOfChannel = 1;

				LOGD("setparam channels = %lu", pAmrnbDecCompPrivate->stAudioAmrnb.nChannels);
				break;
			}

			default: /*Call the base component function*/
				eError = omx_audiodec_component_SetParameter(hComponent, nParamIndex, ComponentParameterStructure);
				break;
		}
	}
	return eError;
}

/** this function gets the parameters regarding audio formats and index */
static OMX_ERRORTYPE omx_amrnbdec_component_GetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_INOUT OMX_PTR ComponentParameterStructure)
{

	OMX_ERRORTYPE eError = OMX_ErrorNone;
	OMX_COMPONENTTYPE *pOpenmaxStandComp = (OMX_COMPONENTTYPE *)hComponent;
	omx_amrnbdec_component_PrivateType* pAmrnbDecCompPrivate = pOpenmaxStandComp->pComponentPrivate;
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
				(void)memcpy(ComponentParameterStructure, &pAmrnbDecCompPrivate->stOutPcmMode, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));
			}
			else
			{
				eError = OMX_ErrorBadPortIndex;
			}

			break;
		}

		case OMX_IndexParamAudioAmr:
		{
			if ((eError = checkHeader(ComponentParameterStructure, sizeof(OMX_AUDIO_PARAM_AMRTYPE))) != OMX_ErrorNone)
			{
				break;
			}

			if (((OMX_AUDIO_PARAM_AMRTYPE *)(ComponentParameterStructure))->nPortIndex != pAmrnbDecCompPrivate->stAudioAmrnb.nPortIndex)
			{
				return OMX_ErrorBadPortIndex;
			}

			(void)memcpy(ComponentParameterStructure, &pAmrnbDecCompPrivate->stAudioAmrnb, sizeof(OMX_AUDIO_PARAM_AMRTYPE));
			LOGD("getparam channels = %lu", pAmrnbDecCompPrivate->stAudioAmrnb.nChannels);
			break;
		}

		case OMX_IndexParamStandardComponentRole:
		{
			pComponentRole = (OMX_PARAM_COMPONENTROLETYPE*)ComponentParameterStructure;
			if ((eError = checkHeader(ComponentParameterStructure, sizeof(OMX_PARAM_COMPONENTROLETYPE))) != OMX_ErrorNone)
			{
				break;
			}

			(void)strncpy( (char*) pComponentRole->cRole, TCC_OMX_COMPONENT_AMRNBDEC_ROLE, sizeof(TCC_OMX_COMPONENT_AMRNBDEC_ROLE));
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
	omx_amrnbdec_component_PrivateType* pAmrnbDecCompPrivate;

	if(strncmp(cComponentName, TCC_OMX_COMPONENT_AMRNBDEC_NAME, sizeof(TCC_OMX_COMPONENT_AMRNBDEC_NAME)))
	{
		// IL client specified an invalid component name
		LOGE("OMX_ErrorInvalidComponentName %s", cComponentName);
		eError = OMX_ErrorInvalidComponentName;
		goto EXIT;
	}

	eError = omx_audiodec_component_Init(pOpenmaxStandComp, cComponentName, sizeof(omx_amrnbdec_component_PrivateType));
	if(eError != OMX_ErrorNone)
	{
		goto EXIT;
	}

	pAmrnbDecCompPrivate = pOpenmaxStandComp->pComponentPrivate;

	/** parameters related to output port */
	// use default setting (in omx_audiodec_component_Init)

	/** parameters related to output audio param port */
	// use default setting (in omx_audiodec_component_Init)

	/** parameters related to input port */
	inPort = (omx_base_audio_PortType *) pAmrnbDecCompPrivate->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];
	inPort->sPortParam.nBufferSize = DEFAULT_IN_BUFFER_SIZE*2;		//DEFAULT_IN_BUFFER_SIZE = 32 * 1024
	(void)strncpy(inPort->sPortParam.format.audio.cMIMEType, TCC_OMX_COMPONENT_AMRNBDEC_MIME, sizeof(TCC_OMX_COMPONENT_AMRNBDEC_MIME));
	inPort->sPortParam.format.audio.eEncoding = OMX_AUDIO_CodingAMR;
	inPort->sAudioParam.eEncoding = OMX_AUDIO_CodingAMR;

	/** parameters related to input audio param port */
	setHeader(&pAmrnbDecCompPrivate->stAudioAmrnb, sizeof(OMX_AUDIO_PARAM_AMRTYPE));
	pAmrnbDecCompPrivate->stAudioAmrnb.nPortIndex = 0;
	pAmrnbDecCompPrivate->stAudioAmrnb.nChannels = 1;
	pAmrnbDecCompPrivate->stAudioAmrnb.nBitRate = 0;
	pAmrnbDecCompPrivate->stAudioAmrnb.eAMRBandMode = OMX_AUDIO_AMRBandModeNB0;
	pAmrnbDecCompPrivate->stAudioAmrnb.eAMRDTXMode = OMX_AUDIO_AMRDTXModeOff;
	pAmrnbDecCompPrivate->stAudioAmrnb.eAMRFrameFormat = OMX_AUDIO_AMRFrameFormatConformance;

	/** load audio decoder library */
	if (omx_audiodec_component_library_Load(pAmrnbDecCompPrivate, TCC_AMRNBDEC_LIB_NAME) != OMX_ErrorNone)
	{
		eError = OMX_ErrorInsufficientResources;
		goto EXIT;
	}

	/** overwrite omx component functions */
	pOpenmaxStandComp->SetParameter = omx_amrnbdec_component_SetParameter;
	pOpenmaxStandComp->GetParameter = omx_amrnbdec_component_GetParameter;
	pAmrnbDecCompPrivate->pfSetInternalParam = SetInternalParam;

	/** overwrite audio decoder functions */
	pAmrnbDecCompPrivate->pfCodecInit = InitAmrnbDec; // use own init function
	pAmrnbDecCompPrivate->pfCodecFlush = FlushAmrnbDec;
	pAmrnbDecCompPrivate->pfCodecDecode = DecodeAmrnb;

	pAmrnbDecCompPrivate->uiAudioProcessMode = AUDIO_NORMAL_MODE; /* decoded pcm mode */
	pAmrnbDecCompPrivate->uiCodecType = AUDIO_ID_AMR;
	pAmrnbDecCompPrivate->iMinStreamSize = 4;

	LOGD("constructor of amrnb decoder component is completed!");

EXIT:
	return eError;
}
