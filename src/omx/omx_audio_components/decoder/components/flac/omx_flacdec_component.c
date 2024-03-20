/**

  @file omx_flacdec_component.c

  This component implement FLAC decoder.

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

#include <omx_flacdec_component.h>
#include <OMX_TCC_Index.h>
#include "TCCFile.h"
#include "TCCMemory.h"

#ifdef HAVE_ANDROID_OS
#define USE_EXTERNAL_BUFFER 0
#define LOG_TAG	"OMX_TCC_FLACDEC"
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
			tcc_printf(T_DEFAULT "[OMXFLACDEC:V]" __VA_ARGS__);\
			tcc_printf(" - [%s - %d]\n"T_DEFAULT,__func__,__LINE__);\
		}\
	}
#define LOGD(...)	{\
		if (gs_iDbgmsgLevel_Adec >= 4) {\
			tcc_printf(TC_CYAN "[OMXFLACDEC:D]" __VA_ARGS__);\
			tcc_printf(" - [%s - %d]\n"T_DEFAULT,__func__,__LINE__);\
		}\
	}
#define LOGI(...)	{\
		if (gs_iDbgmsgLevel_Adec >= 3) {\
			tcc_printf(TC_GREEN "[OMXFLACDEC:I]" __VA_ARGS__);\
			tcc_printf(" - [%s - %d]\n"T_DEFAULT,__func__,__LINE__);\
		}\
	}
#define LOGW(...)	{\
		if (gs_iDbgmsgLevel_Adec >= 2) {\
			tcc_printf(TC_MAGENTA "[OMXFLACDEC:W]" __VA_ARGS__);\
			tcc_printf(" - [%s - %d]\n"T_DEFAULT,__func__,__LINE__);\
		}\
	}
#define LOGE(...)	{\
		if (gs_iDbgmsgLevel_Adec >= 1) {\
			tcc_printf(TC_RED "[OMXFLACDEC:E]" __VA_ARGS__);\
			tcc_printf(" - [%s - %d]\n"T_DEFAULT,__func__,__LINE__);\
		}\
	}
#endif /* HAVE_ANDROID_OS */

#if 0
static OMX_S32 InitFlacDec(omx_flacdec_component_PrivateType* pFlacDecCompPrivate, OMX_BUFFERHEADERTYPE* pInputBuffer)
{
	FlacDecoderPrivate *pFlacDecPrivate = (FlacDecoderPrivate *)pFlacDecCompPrivate->stADecInit.m_unAudioCodecParams;
	OMX_S32 nRetValue	= 0;

	/* setting callback functions */
	// set already

	/* set pcm-info struct */
	// set already

	/* set stream-info struct */
	// set already

	pFlacDecPrivate->m_iFlacInitMode = 1;

	nRetValue = pADecCompPrivate->pfCdkAudioDec(AUDIO_INIT, &pADecCompPrivate->hDecoderHandle, &pADecCompPrivate->stADecInit, NULL);

	LOGD("%s result %ld",__func__, nRetValue);

	return nRetValue;

}
#endif

static OMX_S32 SetInternalParam(omx_flacdec_component_PrivateType* pFlacDecCompPrivate)
{
	/* reset OMX_AUDIO_PARAM_PCMMODETYPE Info */
	pFlacDecCompPrivate->stOutPcmMode.nChannels = pFlacDecCompPrivate->stPcmInfo.m_uiNumberOfChannel;
	pFlacDecCompPrivate->stOutPcmMode.nSamplingRate = pFlacDecCompPrivate->stPcmInfo.m_eSampleRate;

	/* reset OMX_AUDIO_PARAM_FLACTYPE Info */
	pFlacDecCompPrivate->stAudioFlac.nSampleRate = pFlacDecCompPrivate->stPcmInfo.m_eSampleRate;
	pFlacDecCompPrivate->stAudioFlac.nChannels = pFlacDecCompPrivate->stPcmInfo.m_uiNumberOfChannel;
	return 0;
}

/** this function sets the parameter values regarding audio format & index */
static OMX_ERRORTYPE omx_flacdec_component_SetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_IN  OMX_PTR ComponentParameterStructure)
{

	OMX_ERRORTYPE eError = OMX_ErrorNone;
	OMX_PARAM_COMPONENTROLETYPE * pComponentRole;
	OMX_U32 portIndex;

	/* Check which structure we are being fed and make control its header */
	OMX_COMPONENTTYPE *pOpenmaxStandComp = (OMX_COMPONENTTYPE *)hComponent;
	omx_flacdec_component_PrivateType* pFlacDecCompPrivate = pOpenmaxStandComp->pComponentPrivate;

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

			if (pAudioSrcPcmMode->nSamplingRate > pFlacDecCompPrivate->nMaxSupportSamplerate)
			{
				LOGE(" This system doesn't support over %luHz sampling rate!", pFlacDecCompPrivate->nMaxSupportSamplerate);
				eError = OMX_ErrorUnsupportedSetting;
				break;
			}

			/*Check Structure Header and verify component state*/
			eError = omx_base_component_ParameterSanityCheck(hComponent, portIndex, pAudioSrcPcmMode, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));
			if (eError!= OMX_ErrorNone)
			{
				LOGD("In %s Parameter Check Error=%x",__func__,eError);
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
				(void)memcpy(&pFlacDecCompPrivate->stOutPcmMode, pAudioSrcPcmMode, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));
			}
			break;
		}

		case OMX_IndexParamStandardComponentRole:
		{
			pComponentRole = (OMX_PARAM_COMPONENTROLETYPE*)ComponentParameterStructure;
			if (strncmp( (char*) pComponentRole->cRole, TCC_OMX_COMPONENT_FLACDEC_ROLE, sizeof(TCC_OMX_COMPONENT_FLACDEC_ROLE)))
			{
				eError = OMX_ErrorBadParameter;
			}
			break;
		}

		// codec specific parameters -------------------------------------------------------
		case OMX_IndexParamAudioFlac:
		{
			OMX_AUDIO_PARAM_PCMMODETYPE *pAudioParamPCM = &pFlacDecCompPrivate->stOutPcmMode;
			portIndex = pFlacDecCompPrivate->stAudioFlac.nPortIndex;
			eError = omx_base_component_ParameterSanityCheck(hComponent,portIndex,ComponentParameterStructure,sizeof(OMX_AUDIO_PARAM_FLACTYPE));
			if(eError!=OMX_ErrorNone)
			{
				LOGD("In %s Parameter Check Error=%x",__func__,eError);
				break;
			}

			if (((OMX_AUDIO_PARAM_FLACTYPE *)(ComponentParameterStructure))->nPortIndex != portIndex)
			{
				eError = OMX_ErrorBadPortIndex;
				break;
			}

			(void)memcpy(&pFlacDecCompPrivate->stAudioFlac, ComponentParameterStructure, sizeof(OMX_AUDIO_PARAM_FLACTYPE));

			pFlacDecCompPrivate->stStreamInfo.m_eSampleRate = (TCAS_U32)pFlacDecCompPrivate->stAudioFlac.nSampleRate;
			pFlacDecCompPrivate->stStreamInfo.m_uiNumberOfChannel = pFlacDecCompPrivate->stAudioFlac.nChannels;

			if(pAudioParamPCM->nSamplingRate != pFlacDecCompPrivate->stAudioFlac.nSampleRate)
			{
				pAudioParamPCM->nSamplingRate = pFlacDecCompPrivate->stAudioFlac.nSampleRate;
			}

			if (pAudioParamPCM->nChannels != pFlacDecCompPrivate->stAudioFlac.nChannels)
			{
				pAudioParamPCM->nChannels = pFlacDecCompPrivate->stAudioFlac.nChannels;
			}

			if (pAudioParamPCM->nChannels > 2)
			{
				LOGD("flac ch: %lu", pAudioParamPCM->nChannels);
				if (pFlacDecCompPrivate->stADecInit.m_iDownMixMode == 1)
				{
					LOGD("downmix 2ch");
					pAudioParamPCM->nChannels = 2;
				}
				else
				{
					LOGI("multichannel out");
				}
			}

			break;
		}

		default: /*Call the base component function*/
			eError = omx_audiodec_component_SetParameter(hComponent, nParamIndex, ComponentParameterStructure);
			break;
	}

	return eError;
}

/** this function gets the parameters regarding audio formats and index */
static OMX_ERRORTYPE omx_flacdec_component_GetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_INOUT OMX_PTR ComponentParameterStructure)
{

	OMX_ERRORTYPE eError = OMX_ErrorNone;
	OMX_COMPONENTTYPE *pOpenmaxStandComp = (OMX_COMPONENTTYPE *)hComponent;
	omx_flacdec_component_PrivateType* pFlacDecCompPrivate = pOpenmaxStandComp->pComponentPrivate;
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
				(void)memcpy(ComponentParameterStructure, &pFlacDecCompPrivate->stOutPcmMode, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));
			}
			else
			{
				eError = OMX_ErrorBadPortIndex;
			}

			break;
		}

		case OMX_IndexParamAudioFlac:
		{
			if ((eError = checkHeader(ComponentParameterStructure, sizeof(OMX_AUDIO_PARAM_FLACTYPE))) != OMX_ErrorNone)
			{
				break;
			}

			if (((OMX_AUDIO_PARAM_FLACTYPE *)(ComponentParameterStructure))->nPortIndex != pFlacDecCompPrivate->stAudioFlac.nPortIndex)
			{
				return OMX_ErrorBadPortIndex;
			}

			(void)memcpy(ComponentParameterStructure, &pFlacDecCompPrivate->stAudioFlac, sizeof(OMX_AUDIO_PARAM_FLACTYPE));
			break;
		}

		case OMX_IndexParamStandardComponentRole:
		{
			pComponentRole = (OMX_PARAM_COMPONENTROLETYPE*)ComponentParameterStructure;
			if ((eError = checkHeader(ComponentParameterStructure, sizeof(OMX_PARAM_COMPONENTROLETYPE))) != OMX_ErrorNone)
			{
				break;
			}

			(void)strncpy( (char*) pComponentRole->cRole, TCC_OMX_COMPONENT_FLACDEC_ROLE, sizeof(TCC_OMX_COMPONENT_FLACDEC_ROLE));
			break;
		}

		default: /*Call the audiodec component function*/
			eError = omx_audiodec_component_GetParameter(hComponent, nParamIndex, ComponentParameterStructure);
			break;

	}

	return eError;

}


OMX_ERRORTYPE OMX_ComponentInit(OMX_COMPONENTTYPE *pOpenmaxStandComp, OMX_STRING cComponentName)
{
	OMX_ERRORTYPE eError = OMX_ErrorNone;
	omx_base_audio_PortType *inPort;
	omx_flacdec_component_PrivateType* pFlacDecCompPrivate;

	if(strncmp(cComponentName, TCC_OMX_COMPONENT_FLACDEC_NAME, sizeof(TCC_OMX_COMPONENT_FLACDEC_NAME)))
	{
		// IL client specified an invalid component name
		LOGE("OMX_ErrorInvalidComponentName %s", cComponentName);
		eError = OMX_ErrorInvalidComponentName;
		goto EXIT;
	}

	eError = omx_audiodec_component_Init(pOpenmaxStandComp, cComponentName, sizeof(omx_flacdec_component_PrivateType));
	if(eError != OMX_ErrorNone)
	{
		goto EXIT;
	}

	pFlacDecCompPrivate = pOpenmaxStandComp->pComponentPrivate;

	/** parameters related to output port */
	// use default setting (in omx_audiodec_component_Init)

	/** parameters related to output audio param port */
	// use default setting (in omx_audiodec_component_Init)

	/** parameters related to input port */
	inPort = (omx_base_audio_PortType *) pFlacDecCompPrivate->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];
	inPort->sPortParam.nBufferSize = 65536;//DEFAULT_IN_BUFFER_SIZE*4;
	(void)strncpy(inPort->sPortParam.format.audio.cMIMEType, TCC_OMX_COMPONENT_FLACDEC_MIME, sizeof(TCC_OMX_COMPONENT_FLACDEC_MIME));
	inPort->sPortParam.format.audio.eEncoding = OMX_AUDIO_CodingFLAC;
	inPort->sAudioParam.eEncoding = OMX_AUDIO_CodingFLAC;

	/** parameters related to input audio param port */
	setHeader(&pFlacDecCompPrivate->stAudioFlac, sizeof(OMX_AUDIO_PARAM_FLACTYPE));
	pFlacDecCompPrivate->stAudioFlac.nPortIndex = 0;
	pFlacDecCompPrivate->stAudioFlac.nChannels = 2;
	pFlacDecCompPrivate->stAudioFlac.nSampleRate = 44100;

	/** load audio decoder library */
	if (omx_audiodec_component_library_Load(pFlacDecCompPrivate, TCC_FLACDEC_LIB_NAME) != OMX_ErrorNone)
	{
		eError = OMX_ErrorInsufficientResources;
		goto EXIT;
	}

	/** overwrite omx component functions */
	pOpenmaxStandComp->SetParameter = omx_flacdec_component_SetParameter;
	pOpenmaxStandComp->GetParameter = omx_flacdec_component_GetParameter;
	pFlacDecCompPrivate->pfSetInternalParam = SetInternalParam;

	/** overwrite audio decoder functions */
	// pFlacDecCompPrivate->pfCodecInit = InitFlacDec; // use own init function

	pFlacDecCompPrivate->uiAudioProcessMode = AUDIO_NORMAL_MODE; /* decoded pcm mode */
	pFlacDecCompPrivate->uiCodecType = AUDIO_ID_FLAC;
	pFlacDecCompPrivate->iMinStreamSize = 0;

	LOGD("constructor of flac decoder component is completed!");

EXIT:
	return eError;

}
