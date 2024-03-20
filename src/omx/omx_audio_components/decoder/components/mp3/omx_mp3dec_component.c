/**

  @file omx_mp3dec_component.c

  This component implement MP3 decoder.

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

#include <omx_mp3dec_component.h>
#include <OMX_TCC_Index.h>
#include "TCCFile.h"
#include "TCCMemory.h"

#ifdef HAVE_ANDROID_OS
#define USE_EXTERNAL_BUFFER 0
#define LOG_TAG	"OMX_TCC_MP3DEC"
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
			tcc_printf(T_DEFAULT "[OMXMP3DEC:V]" __VA_ARGS__);\
			tcc_printf(" - [%s - %d]\n"T_DEFAULT,__func__,__LINE__);\
		}\
	}
#define LOGD(...)	{\
		if (gs_iDbgmsgLevel_Adec >= 4) {\
			tcc_printf(TC_CYAN "[OMXMP3DEC:D]" __VA_ARGS__);\
			tcc_printf(" - [%s - %d]\n"T_DEFAULT,__func__,__LINE__);\
		}\
	}
#define LOGI(...)	{\
		if (gs_iDbgmsgLevel_Adec >= 3) {\
			tcc_printf(TC_GREEN "[OMXMP3DEC:I]" __VA_ARGS__);\
			tcc_printf(" - [%s - %d]\n"T_DEFAULT,__func__,__LINE__);\
		}\
	}
#define LOGW(...)	{\
		if (gs_iDbgmsgLevel_Adec >= 2) {\
			tcc_printf(TC_MAGENTA "[OMXMP3DEC:W]" __VA_ARGS__);\
			tcc_printf(" - [%s - %d]\n"T_DEFAULT,__func__,__LINE__);\
		}\
	}
#define LOGE(...)	{\
		if (gs_iDbgmsgLevel_Adec >= 1) {\
			tcc_printf(TC_RED "[OMXMP3DEC:E]" __VA_ARGS__);\
			tcc_printf(" - [%s - %d]\n"T_DEFAULT,__func__,__LINE__);\
		}\
	}
#endif /* HAVE_ANDROID_OS */

static OMX_S32 InitMp3Dec(omx_mp3dec_component_PrivateType* pMp3DecCompPrivate, OMX_BUFFERHEADERTYPE* pInputBuffer)
{
	Mp3DecoderPrivate *pMp3DecPrivate = (Mp3DecoderPrivate *)pMp3DecCompPrivate->stADecInit.m_unAudioCodecParams;
	OMX_S32 nRetValue = 0;
	OMX_AUDIO_MP3STREAMFORMATTYPE InputMPEGAudioType = pMp3DecCompPrivate->stAudioMp3.eFormat;
	OMX_S32 nLayer;

	/* setting callback functions */
	// set already

	/* set pcm-info struct */
	// set already

	/* set stream-info struct */
	// set already
	if ((InputMPEGAudioType >= OMX_AUDIO_MP3StreamFormatMP1Layer1) && (InputMPEGAudioType <= OMX_AUDIO_MP3StreamFormatMP2_5Layer1)) {
		nLayer = 1;
	} else if ((InputMPEGAudioType >= OMX_AUDIO_MP3StreamFormatMP1Layer2) && (InputMPEGAudioType <= OMX_AUDIO_MP3StreamFormatMP2_5Layer2)) {
		nLayer = 2;
	} else {
		nLayer = 3;
	}

	pMp3DecCompPrivate->pfCdkAudioDec = pMp3DecCompPrivate->pDecFunctionList->pfCodecSpecific[nLayer-1];
	if (pMp3DecCompPrivate->pfCdkAudioDec == NULL) {
		LOGE("MPEG Audio Layer%ld decoder is not yet implemented\n", nLayer);
		return OMX_ErrorNotImplemented;
	}

	pMp3DecPrivate->m_iDABMode = (pMp3DecCompPrivate->uiAudioProcessMode == AUDIO_BROADCAST_MODE) ? 1 : 0;	// for future use

	nRetValue = pMp3DecCompPrivate->pfCdkAudioDec(AUDIO_INIT, &pMp3DecCompPrivate->hDecoderHandle, &pMp3DecCompPrivate->stADecInit, NULL);

	LOGI("%s MP%ld Decoder init: result %ld", __func__, nLayer, nRetValue);

	return nRetValue;

}

static OMX_S32 SetInternalParam(omx_mp3dec_component_PrivateType* pMp3DecCompPrivate)
{
	/* reset OMX_AUDIO_PARAM_PCMMODETYPE Info */
	pMp3DecCompPrivate->stOutPcmMode.nChannels = pMp3DecCompPrivate->stPcmInfo.m_uiNumberOfChannel;
	pMp3DecCompPrivate->stOutPcmMode.nSamplingRate = pMp3DecCompPrivate->stPcmInfo.m_eSampleRate;

	/* reset OMX_AUDIO_PARAM_MP3TYPE Info */
	pMp3DecCompPrivate->stAudioMp3.nSampleRate = pMp3DecCompPrivate->stPcmInfo.m_eSampleRate;
	pMp3DecCompPrivate->stAudioMp3.nChannels = pMp3DecCompPrivate->stPcmInfo.m_uiNumberOfChannel;
	return 0;
}

/** this function sets the parameter values regarding audio format & index */
static OMX_ERRORTYPE omx_mp3dec_component_SetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_IN  OMX_PTR ComponentParameterStructure)
{

	OMX_ERRORTYPE eError = OMX_ErrorNone;
	OMX_PARAM_COMPONENTROLETYPE * pComponentRole;
	OMX_U32 portIndex;

	/* Check which structure we are being fed and make control its header */
	OMX_COMPONENTTYPE *pOpenmaxStandComp = (OMX_COMPONENTTYPE *)hComponent;
	omx_mp3dec_component_PrivateType* pMp3DecCompPrivate = pOpenmaxStandComp->pComponentPrivate;

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

			if (pAudioSrcPcmMode->nSamplingRate > pMp3DecCompPrivate->nMaxSupportSamplerate)
			{
				LOGE(" This system doesn't support over %luHz sampling rate!", pMp3DecCompPrivate->nMaxSupportSamplerate);
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
				LOGV(" %s Param = Pcm:: nCh = %lu, %lu bit Pcm, SampleRate = %lu", __func__,
							pAudioSrcPcmMode->nChannels, pAudioSrcPcmMode->nBitPerSample, pAudioSrcPcmMode->nSamplingRate);
				(void)memcpy(&pMp3DecCompPrivate->stOutPcmMode, pAudioSrcPcmMode, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));
			}
			break;
		}

		case OMX_IndexParamStandardComponentRole:
		{
			pComponentRole = (OMX_PARAM_COMPONENTROLETYPE*)ComponentParameterStructure;
			if (strncmp( (char*) pComponentRole->cRole, TCC_OMX_COMPONENT_MP3DEC_ROLE, sizeof(TCC_OMX_COMPONENT_MP3DEC_ROLE)))
			{
				eError = OMX_ErrorBadParameter;
			}
			break;
		}

		// codec specific parameters -------------------------------------------------------
		case OMX_IndexParamAudioMp3:
		{
			OMX_AUDIO_PARAM_PCMMODETYPE *pAudioParamPCM = &pMp3DecCompPrivate->stOutPcmMode;
			portIndex = pMp3DecCompPrivate->stAudioMp3.nPortIndex;
			eError = omx_base_component_ParameterSanityCheck(hComponent,portIndex,ComponentParameterStructure,sizeof(OMX_AUDIO_PARAM_MP3TYPE));
			if(eError!=OMX_ErrorNone)
			{
				LOGD("In %s Parameter Check Error=%x",__func__,eError);
				break;
			}

			if (((OMX_AUDIO_PARAM_MP3TYPE *)(ComponentParameterStructure))->nPortIndex != portIndex)
			{
				eError = OMX_ErrorBadPortIndex;
				break;
			}

			(void)memcpy(&pMp3DecCompPrivate->stAudioMp3, ComponentParameterStructure, sizeof(OMX_AUDIO_PARAM_MP3TYPE));

			pMp3DecCompPrivate->stStreamInfo.m_eSampleRate = (TCAS_U32)pMp3DecCompPrivate->stAudioMp3.nSampleRate;
			pMp3DecCompPrivate->stStreamInfo.m_uiNumberOfChannel = pMp3DecCompPrivate->stAudioMp3.nChannels;

			if(pAudioParamPCM->nSamplingRate != pMp3DecCompPrivate->stAudioMp3.nSampleRate)
			{
				pAudioParamPCM->nSamplingRate = pMp3DecCompPrivate->stAudioMp3.nSampleRate;
			}

			if (pAudioParamPCM->nChannels != pMp3DecCompPrivate->stAudioMp3.nChannels)
			{
				pAudioParamPCM->nChannels = pMp3DecCompPrivate->stAudioMp3.nChannels;
			}
			LOGV(" %s Param = MP3:: nCh = %lu, SampleRate = %lu", __func__, pMp3DecCompPrivate->stAudioMp3.nChannels, pMp3DecCompPrivate->stAudioMp3.nSampleRate);
			break;
		}

		default: /*Call the base component function*/
			return omx_audiodec_component_SetParameter(hComponent, nParamIndex, ComponentParameterStructure);

	}

	return eError;
}

/** this function gets the parameters regarding audio formats and index */
static OMX_ERRORTYPE omx_mp3dec_component_GetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_INOUT OMX_PTR ComponentParameterStructure)
{

	OMX_ERRORTYPE eError = OMX_ErrorNone;
	OMX_COMPONENTTYPE *pOpenmaxStandComp = (OMX_COMPONENTTYPE *)hComponent;
	omx_mp3dec_component_PrivateType* pMp3DecCompPrivate = pOpenmaxStandComp->pComponentPrivate;
	OMX_PARAM_COMPONENTROLETYPE * pComponentRole;

	if (ComponentParameterStructure == NULL)
	{
		return OMX_ErrorBadParameter;
	}

	LOGD(" %s nParamIndex [0x%x]", __func__, nParamIndex);

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
				LOGD(" %s Pcm:: nCh = %lu, %lu bit Pcm, SampleRate = %lu",
						__func__,
						pMp3DecCompPrivate->stOutPcmMode.nChannels,
						pMp3DecCompPrivate->stOutPcmMode.nBitPerSample,
						pMp3DecCompPrivate->stOutPcmMode.nSamplingRate);
				(void)memcpy(ComponentParameterStructure, &pMp3DecCompPrivate->stOutPcmMode, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));
			}
			else
			{
				eError = OMX_ErrorBadPortIndex;
			}

			break;
		}

		case OMX_IndexParamAudioMp3:
		{
			if ((eError = checkHeader(ComponentParameterStructure, sizeof(OMX_AUDIO_PARAM_MP3TYPE))) != OMX_ErrorNone)
			{
				break;
			}

			if (((OMX_AUDIO_PARAM_MP3TYPE *)(ComponentParameterStructure))->nPortIndex != pMp3DecCompPrivate->stAudioMp3.nPortIndex)
			{
				return OMX_ErrorBadPortIndex;
			}

			LOGD(" %s MP3:: nCh = %lu, SampleRate = %lu", __func__,
						pMp3DecCompPrivate->stAudioMp3.nChannels, pMp3DecCompPrivate->stAudioMp3.nSampleRate);
			(void)memcpy(ComponentParameterStructure, &pMp3DecCompPrivate->stAudioMp3, sizeof(OMX_AUDIO_PARAM_MP3TYPE));
			break;
		}

		case OMX_IndexParamStandardComponentRole:
		{
			pComponentRole = (OMX_PARAM_COMPONENTROLETYPE*)ComponentParameterStructure;
			if ((eError = checkHeader(ComponentParameterStructure, sizeof(OMX_PARAM_COMPONENTROLETYPE))) != OMX_ErrorNone)
			{
				break;
			}

			(void)strncpy( (char*) pComponentRole->cRole, TCC_OMX_COMPONENT_MP3DEC_ROLE, sizeof(TCC_OMX_COMPONENT_MP3DEC_ROLE));
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
	omx_base_audio_PortType *inPort;
	omx_mp3dec_component_PrivateType* pMp3DecCompPrivate;

	if(strncmp(cComponentName, TCC_OMX_COMPONENT_MP3DEC_NAME, sizeof(TCC_OMX_COMPONENT_MP3DEC_NAME)))
	{
		// IL client specified an invalid component name
		LOGE("OMX_ErrorInvalidComponentName %s", cComponentName);
		eError = OMX_ErrorInvalidComponentName;
		goto EXIT;
	}

	eError = omx_audiodec_component_Init(pOpenmaxStandComp, cComponentName, sizeof(omx_mp3dec_component_PrivateType));
	if(eError != OMX_ErrorNone)
	{
		goto EXIT;
	}

	pMp3DecCompPrivate = pOpenmaxStandComp->pComponentPrivate;

	/** parameters related to output port */
	// use default setting (in omx_audiodec_component_Init)

	/** parameters related to output audio param port */
	// use default setting (in omx_audiodec_component_Init)

	/** parameters related to input port */
	inPort = (omx_base_audio_PortType *) pMp3DecCompPrivate->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];
	inPort->sPortParam.nBufferSize = DEFAULT_IN_BUFFER_SIZE*2;
	(void)strncpy(inPort->sPortParam.format.audio.cMIMEType, TCC_OMX_COMPONENT_MP3DEC_MIME, sizeof(TCC_OMX_COMPONENT_MP3DEC_MIME));
	inPort->sPortParam.format.audio.eEncoding = OMX_AUDIO_CodingMP3;
	inPort->sAudioParam.eEncoding = OMX_AUDIO_CodingMP3;

	/** parameters related to input audio param port */
	setHeader(&pMp3DecCompPrivate->stAudioMp3, sizeof(OMX_AUDIO_PARAM_MP3TYPE));
	pMp3DecCompPrivate->stAudioMp3.nPortIndex = 0;
	pMp3DecCompPrivate->stAudioMp3.nChannels = 2;
	pMp3DecCompPrivate->stAudioMp3.nBitRate = 0;
	pMp3DecCompPrivate->stAudioMp3.nSampleRate = 44100;
	pMp3DecCompPrivate->stAudioMp3.eChannelMode = OMX_AUDIO_ChannelModeStereo;

	/** load audio decoder library */
	if (omx_audiodec_component_library_Load(pMp3DecCompPrivate, TCC_MP3DEC_LIB_NAME) != OMX_ErrorNone)
	{
		eError = OMX_ErrorInsufficientResources;
		goto EXIT;
	}

	/** overwrite omx component functions */
	pOpenmaxStandComp->SetParameter = omx_mp3dec_component_SetParameter;
	pOpenmaxStandComp->GetParameter = omx_mp3dec_component_GetParameter;
	pMp3DecCompPrivate->pfSetInternalParam = SetInternalParam;

	/** overwrite audio decoder functions */
	pMp3DecCompPrivate->pfCodecInit = InitMp3Dec; // use own init function
	pMp3DecCompPrivate->isPcmSplitEnabled = OMX_FALSE;
	pMp3DecCompPrivate->isSilenceInsertionEnable = OMX_TRUE;

	pMp3DecCompPrivate->uiAudioProcessMode = AUDIO_NORMAL_MODE; /* decoded pcm mode */
	pMp3DecCompPrivate->uiCodecType = AUDIO_ID_MP3;
	pMp3DecCompPrivate->iMinStreamSize = 32;

	LOGD("constructor of mp3 decoder component is completed!");

EXIT:
	return eError;

}
