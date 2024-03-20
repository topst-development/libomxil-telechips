/**

  @file omx_wavdec_component.c

  This component implement WAV decoder.

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

#include <omx_wavdec_component.h>
#include <OMX_TCC_Index.h>
#include "TCCFile.h"
#include "TCCMemory.h"

#ifdef HAVE_ANDROID_OS
#define USE_EXTERNAL_BUFFER 0
#define LOG_TAG	"OMX_TCC_PCMDEC"
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
			tcc_printf(T_DEFAULT "[OMXWAVDEC:V]" __VA_ARGS__);\
			tcc_printf(" - [%s - %d]\n"T_DEFAULT,__func__,__LINE__);\
		}\
	}
#define LOGD(...)	{\
		if (gs_iDbgmsgLevel_Adec >= 4) {\
			tcc_printf(TC_CYAN "[OMXWAVDEC:D]" __VA_ARGS__);\
			tcc_printf(" - [%s - %d]\n"T_DEFAULT,__func__,__LINE__);\
		}\
	}
#define LOGI(...)	{\
		if (gs_iDbgmsgLevel_Adec >= 3) {\
			tcc_printf(TC_GREEN "[OMXWAVDEC:I]" __VA_ARGS__);\
			tcc_printf(" - [%s - %d]\n"T_DEFAULT,__func__,__LINE__);\
		}\
	}
#define LOGW(...)	{\
		if (gs_iDbgmsgLevel_Adec >= 2) {\
			tcc_printf(TC_MAGENTA "[OMXWAVDEC:W]" __VA_ARGS__);\
			tcc_printf(" - [%s - %d]\n"T_DEFAULT,__func__,__LINE__);\
		}\
	}
#define LOGE(...)	{\
		if (gs_iDbgmsgLevel_Adec >= 1) {\
			tcc_printf(TC_RED "[OMXWAVDEC:E]" __VA_ARGS__);\
			tcc_printf(" - [%s - %d]\n"T_DEFAULT,__func__,__LINE__);\
		}\
	}
#endif /* HAVE_ANDROID_OS */

static OMX_S32 InitWavDec(omx_wavdec_component_PrivateType* pWavDecCompPrivate, OMX_BUFFERHEADERTYPE* pInputbuffer)
{
	WavDecoderPrivate* pWavDecPrivate = (WavDecoderPrivate *)pWavDecCompPrivate->stADecInit.m_unAudioCodecParams;
	OMX_S32 nRetValue = 0;

	/* setting callback functions */
	// set already

	/* set pcm-info struct */
	// set already

	/* set stream-info struct */
	// set already

	// to pass original data to A/V receiver
//	if(pWavDecCompPrivate->isBitstreamOut == OMX_TRUE)
//	{
//		pWavDecCompPrivate->stADecInit.m_iDownMixMode = 0;
//	}
//	else
	{
		pWavDecCompPrivate->stADecInit.m_iDownMixMode = 1;
	}

#if 0
	if(pWavDecCompPrivate->iExtractorType == OMX_BUFFERFLAG_EXTRACTORTYPE_TCC)
	{
		// setting stream info
		pWavDecCompPrivate->stStreamInfo.m_eSampleRate = pWavDecCompPrivate->cdmx_info.m_sAudioInfo.m_iSamplePerSec;
		pWavDecCompPrivate->stStreamInfo.m_uiNumberOfChannel = pWavDecCompPrivate->cdmx_info.m_sAudioInfo.m_iChannels;
		pWavDecCompPrivate->stStreamInfo.m_uiBitsPerSample = pWavDecCompPrivate->cdmx_info.m_sAudioInfo.m_iBitsPerSample;
		// setting extra info
		pWavDecCompPrivate->stADecInit.m_pucExtraData = pWavDecCompPrivate->cdmx_info.m_sAudioInfo.m_pExtraData;
		pWavDecCompPrivate->stADecInit.m_iExtraDataLen = pWavDecCompPrivate->cdmx_info.m_sAudioInfo.m_iExtraDataLen;

		/* WAV (PCM, ADPCM_MS, ADPCM_IMA, ALAW, MULAW) Decoder */
		pWavDecPrivate->m_iWAVForm_TagID  =  pWavDecCompPrivate->cdmx_info.m_sAudioInfo.m_iFormatId;
		pWavDecPrivate->m_uiNBlockAlign  =  pWavDecCompPrivate->cdmx_info.m_sAudioInfo.m_iBlockAlign;		//20091121_Helena

		switch(pWavDecCompPrivate->iCtype)
		{
			case CONTAINER_TYPE_TS:                     //Blu-ray
			{
				pWavDecPrivate->m_iEndian = 1;
				pWavDecPrivate->m_iContainerType = 2;
				break;
			}
			case CONTAINER_TYPE_MPG:        //DVD
			{
				pWavDecPrivate->m_iEndian = 1;
				pWavDecPrivate->m_iContainerType = 1;
				break;
			}
			case CONTAINER_TYPE_AUDIO:
			default:
			{
				pWavDecPrivate->m_iEndian = 0;
				pWavDecPrivate->m_iContainerType = 0;

				if(pWavDecCompPrivate->cdmx_info.m_sAudioInfo.m_iFormatId == AV_AUDIO_MS_PCM_SWAP)
					pWavDecPrivate->m_iEndian = 1;

				break;
			}
		}

		if(pWavDecCompPrivate->cdmx_info.m_sAudioInfo.m_iFormatId == AV_AUDIO_MS_PCM)
		{
			//output sample is greater than 4
			pWavDecCompPrivate->iMinStreamSize = pWavDecCompPrivate->cdmx_info.m_sAudioInfo.m_iChannels * pWavDecCompPrivate->cdmx_info.m_sAudioInfo.m_iBitsPerSample / 2;
		}
	}
	else
#endif
	{
		pWavDecPrivate->m_uiNBlockAlign = pInputbuffer->nFilledLen;

		if (pWavDecCompPrivate->stAudioWav.ePCMMode == OMX_AUDIO_PCMModeDVDLinear)
		{
			pWavDecPrivate->m_iEndian = 1;
			pWavDecPrivate->m_iContainerType = 1;	// 1 = DVD
		}
		else if (pWavDecCompPrivate->stAudioWav.ePCMMode == OMX_AUDIO_PCMModeWDLinear)
		{
			pWavDecPrivate->m_iEndian = 1;
			pWavDecPrivate->m_iContainerType = 2;	// 2 = WiFi-Display
		}
		else	// little endian
		{
			pWavDecPrivate->m_iEndian = (pWavDecCompPrivate->stAudioWav.eEndian == 4321) ? 1 : 0;
			pWavDecPrivate->m_iContainerType = 0;
		}
		pWavDecCompPrivate->stStreamInfo.m_uiBitsPerSample = pWavDecCompPrivate->stAudioWav.nBitPerSample;

		// setting stream info
		// set already (see OpenAudioDecoder)
		//pWavDecCompPrivate->stStreamInfo.m_eSampleRate = pWavDecCompPrivate->stAudioWav.nSamplingRate;
		pWavDecCompPrivate->stStreamInfo.m_uiNumberOfChannel = pWavDecCompPrivate->stAudioWav.nChannels;

		// setting extra info
		//pWavDecCompPrivate->stADecInit.m_pucExtraData = pInputbuffer->pBuffer;
		pWavDecCompPrivate->stADecInit.m_iExtraDataLen = 0; //pInputbuffer->nFilledLen;

		pWavDecPrivate->m_iWAVForm_TagID = 1;	//TC_WAVE_FORMAT_PCM;
	}

	nRetValue = pWavDecCompPrivate->pfCdkAudioDec(AUDIO_INIT, &pWavDecCompPrivate->hDecoderHandle, &pWavDecCompPrivate->stADecInit, NULL);

	pWavDecCompPrivate->stOutPcmMode.nChannels = pWavDecCompPrivate->stPcmInfo.m_uiNumberOfChannel;
	pWavDecCompPrivate->stOutPcmMode.nSamplingRate = pWavDecCompPrivate->stPcmInfo.m_eSampleRate;

	LOGD("%s result %ld",__func__, nRetValue);

	return nRetValue;

}


// decode one frame of wav data
static OMX_S32 DecodeWav(omx_wavdec_component_PrivateType* pWavDecCompPrivate, OMX_BUFFERHEADERTYPE* inputbuffer, OMX_BUFFERHEADERTYPE* outputbuffer)
{
	OMX_S32 iOutFrameSize, iRet = TCAS_SUCCESS;
	int ByteofPCMSample = (pWavDecCompPrivate->stPcmInfo.m_uiBitsPerSample <= 16) ? 2 : 4;

	iOutFrameSize = 0;
	pWavDecCompPrivate->iDecodedSamplePerCh = 0;

	LOGD("start: input length %d", pWavDecCompPrivate->stStreamInfo.m_iStreamLength);

	while(pWavDecCompPrivate->stStreamInfo.m_iStreamLength > pWavDecCompPrivate->iMinStreamSize)
	{
		pWavDecCompPrivate->stPcmInfo.m_pvChannel[0]  = (void *)(outputbuffer->pBuffer + iOutFrameSize);
		if ((outputbuffer->nAllocLen - iOutFrameSize) < pWavDecCompPrivate->stPcmInfo.m_uiNumberOfChannel * pWavDecCompPrivate->stPcmInfo.m_uiSamplesPerChannel * ByteofPCMSample)
		{
			LOGW ("input frame will be split due to memory shortage on the outbuffer");
			break;
		}

		//LOGD("S: iRet %lu, input length %d", iRet, pWavDecCompPrivate->stStreamInfo.m_iStreamLength);
		iRet = pWavDecCompPrivate->pfCdkAudioDec(AUDIO_DECODE, &pWavDecCompPrivate->hDecoderHandle, &pWavDecCompPrivate->stStreamInfo, &pWavDecCompPrivate->stPcmInfo);

		//LOGD("E: iRet %lu, input length %d", iRet, pWavDecCompPrivate->stStreamInfo.m_iStreamLength);

		if(iRet == TCAS_SUCCESS)
		{
			if(pWavDecCompPrivate->stPcmInfo.m_uiSamplesPerChannel)
			{
				//LOGD("ch %lu, samples %lu", pWavDecCompPrivate->stPcmInfo.m_uiNumberOfChannel, pWavDecCompPrivate->stPcmInfo.m_uiSamplesPerChannel);
				iOutFrameSize += pWavDecCompPrivate->stPcmInfo.m_uiNumberOfChannel * pWavDecCompPrivate->stPcmInfo.m_uiSamplesPerChannel * ByteofPCMSample;
				pWavDecCompPrivate->iDecodedSamplePerCh += pWavDecCompPrivate->stPcmInfo.m_uiSamplesPerChannel;
			}
		}
		else
		{
			if(((int)iRet != TCAS_ERROR_SKIP_FRAME) && ((int)iRet != TCAS_ERROR_MORE_DATA))
			{
				LOGE("In %s error %ld", __func__, iRet);
			}

			if((int)iRet != TCAS_ERROR_SKIP_FRAME)
			{
				if((int)iRet != TCAS_ERROR_MORE_DATA /*&& iRet != TCAS_ERROR_INVALID_BUFSTATE*/)
				{
					pWavDecCompPrivate->stStreamInfo.m_iStreamLength = 0;
				}
				break;
			}
		}
	}

	LOGD("end: input length %d, out: %ld", pWavDecCompPrivate->stStreamInfo.m_iStreamLength, iOutFrameSize);

	return iRet;
}

static OMX_S32 SetInternalParam(omx_wavdec_component_PrivateType* pWavDecCompPrivate)
{
	/* reset OMX_AUDIO_PARAM_PCMMODETYPE Info */
	pWavDecCompPrivate->stOutPcmMode.nChannels = pWavDecCompPrivate->stPcmInfo.m_uiNumberOfChannel;
	pWavDecCompPrivate->stOutPcmMode.nSamplingRate = pWavDecCompPrivate->stPcmInfo.m_eSampleRate;

	/* reset OMX_AUDIO_PARAM_PCMMODETYPE Info */
	pWavDecCompPrivate->stAudioWav.nSamplingRate = pWavDecCompPrivate->stPcmInfo.m_eSampleRate;
//	pWavDecCompPrivate->stAudioWav.nChannels = pWavDecCompPrivate->stPcmInfo.m_uiNumberOfChannel;
	return 0;
}

/** this function sets the parameter values regarding audio format & index */
static OMX_ERRORTYPE omx_wavdec_component_SetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_IN  OMX_PTR ComponentParameterStructure)
{

	OMX_ERRORTYPE eError = OMX_ErrorNone;
	OMX_PARAM_COMPONENTROLETYPE * pComponentRole;
	OMX_U32 portIndex;

	/* Check which structure we are being fed and make control its header */
	OMX_COMPONENTTYPE *pOpenmaxStandComp = (OMX_COMPONENTTYPE *)hComponent;
	omx_wavdec_component_PrivateType* pWavDecCompPrivate = pOpenmaxStandComp->pComponentPrivate;

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

			if (pAudioSrcPcmMode->nSamplingRate > pWavDecCompPrivate->nMaxSupportSamplerate)
			{
				LOGE(" This system doesn't support over %luHz sampling rate!", pWavDecCompPrivate->nMaxSupportSamplerate);
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

			if (pAudioSrcPcmMode->nPortIndex == 0) // set input PCM type
			{
				(void)memcpy(&pWavDecCompPrivate->stAudioWav, ComponentParameterStructure, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));
				pWavDecCompPrivate->stOutPcmMode.nSamplingRate = pWavDecCompPrivate->stAudioWav.nSamplingRate;
				if (pWavDecCompPrivate->isBitstreamOut == OMX_TRUE) {
					pWavDecCompPrivate->stOutPcmMode.nChannels = pWavDecCompPrivate->stAudioWav.nChannels;
				} else {
					if (pWavDecCompPrivate->stAudioWav.nChannels == 1) {
						pWavDecCompPrivate->stOutPcmMode.nChannels = 1;
					}

					if (pWavDecCompPrivate->stOutPcmMode.nChannels > 2)
					{
						if (pWavDecCompPrivate->stADecInit.m_iDownMixMode == 1)
						{
							pWavDecCompPrivate->stOutPcmMode.nChannels = 2;
						}
						else
						{
							LOGI("multichannel out");
						}
					}
				}
				pWavDecCompPrivate->stStreamInfo.m_eSampleRate = (TCAS_U32)pWavDecCompPrivate->stAudioWav.nSamplingRate;
				pWavDecCompPrivate->stStreamInfo.m_uiNumberOfChannel = pWavDecCompPrivate->stAudioWav.nChannels;

			}
			else
			{
				// set output PCM type
				LOGI("@@@@ :: nChannels = %lu, nBitPerSample = %lu, nSamplingRate = %lu", pAudioSrcPcmMode->nChannels, pAudioSrcPcmMode->nBitPerSample, pAudioSrcPcmMode->nSamplingRate);
				(void)memcpy(&pWavDecCompPrivate->stOutPcmMode, pAudioSrcPcmMode, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));
			}
			break;
		}

		case OMX_IndexParamStandardComponentRole:
		{
			pComponentRole = (OMX_PARAM_COMPONENTROLETYPE*)ComponentParameterStructure;
			if (strncmp( (char*) pComponentRole->cRole, TCC_OMX_COMPONENT_WAVDEC_ROLE, sizeof(TCC_OMX_COMPONENT_WAVDEC_ROLE)))
			{
				eError = OMX_ErrorBadParameter;
			}
			break;
		}

		default: /*Call the base component function*/
			return omx_audiodec_component_SetParameter(hComponent, nParamIndex, ComponentParameterStructure);

	}

	return eError;
}

/** this function gets the parameters regarding audio formats and index */
static OMX_ERRORTYPE omx_wavdec_component_GetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_INOUT OMX_PTR ComponentParameterStructure)
{

	OMX_ERRORTYPE eError = OMX_ErrorNone;
	OMX_COMPONENTTYPE *pOpenmaxStandComp = (OMX_COMPONENTTYPE *)hComponent;
	omx_wavdec_component_PrivateType* pWavDecCompPrivate = pOpenmaxStandComp->pComponentPrivate;
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
				// get output PCM type
				(void)memcpy(ComponentParameterStructure, &pWavDecCompPrivate->stOutPcmMode, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));
			}
			else
			{
				OMX_U32 portIndex = pWavDecCompPrivate->stAudioWav.nPortIndex;
				if (((OMX_AUDIO_PARAM_PCMMODETYPE *)(ComponentParameterStructure))->nPortIndex != portIndex)
				{
					LOGE(" %s port index error", __func__);
					return OMX_ErrorBadPortIndex;
				}

				(void)memcpy(ComponentParameterStructure, &pWavDecCompPrivate->stAudioWav, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));
			}

			break;
		}

		case OMX_IndexParamStandardComponentRole:
		{
			pComponentRole = (OMX_PARAM_COMPONENTROLETYPE*)ComponentParameterStructure;
			if ((eError = checkHeader(ComponentParameterStructure, sizeof(OMX_PARAM_COMPONENTROLETYPE))) != OMX_ErrorNone)
			{
				break;
			}

			(void)strncpy( (char*) pComponentRole->cRole, TCC_OMX_COMPONENT_WAVDEC_ROLE, sizeof(TCC_OMX_COMPONENT_WAVDEC_ROLE));
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
	omx_wavdec_component_PrivateType* pWavDecCompPrivate;

	if(strncmp(cComponentName, TCC_OMX_COMPONENT_WAVDEC_NAME, sizeof(TCC_OMX_COMPONENT_WAVDEC_NAME)))
	{
		// IL client specified an invalid component name
		LOGE("OMX_ErrorInvalidComponentName %s", cComponentName);
		eError = OMX_ErrorInvalidComponentName;
		goto EXIT;
	}

	eError = omx_audiodec_component_Init(pOpenmaxStandComp, cComponentName, sizeof(omx_wavdec_component_PrivateType));
	if(eError != OMX_ErrorNone)
	{
		goto EXIT;
	}

	pWavDecCompPrivate = pOpenmaxStandComp->pComponentPrivate;

	/** parameters related to output port */
	// use default setting (in omx_audiodec_component_Init)

	/** parameters related to output audio param port */
	// use default setting (in omx_audiodec_component_Init)

	/** parameters related to input port */
	inPort = (omx_base_audio_PortType *) pWavDecCompPrivate->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];
	inPort->sPortParam.nBufferSize = 8192*8*2*2;
	(void)strncpy(inPort->sPortParam.format.audio.cMIMEType, TCC_OMX_COMPONENT_WAVDEC_MIME, sizeof(TCC_OMX_COMPONENT_WAVDEC_MIME));
	inPort->sPortParam.format.audio.eEncoding = OMX_AUDIO_CodingPCM;
	inPort->sAudioParam.eEncoding = OMX_AUDIO_CodingPCM;

	/** parameters related to input audio param port */
	setHeader(&pWavDecCompPrivate->stAudioWav, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));
	pWavDecCompPrivate->stAudioWav.nPortIndex = 0;
	pWavDecCompPrivate->stAudioWav.nChannels = 2;
	pWavDecCompPrivate->stAudioWav.nSamplingRate = 44100;
	pWavDecCompPrivate->stAudioWav.ePCMMode = OMX_AUDIO_PCMModeLinear;

	/** load audio decoder library */
	if (omx_audiodec_component_library_Load(pWavDecCompPrivate, TCC_WAVDEC_LIB_NAME) != OMX_ErrorNone)
	{
		eError = OMX_ErrorInsufficientResources;
		goto EXIT;
	}

	/** overwrite omx component functions */
	pOpenmaxStandComp->SetParameter = omx_wavdec_component_SetParameter;
	pOpenmaxStandComp->GetParameter = omx_wavdec_component_GetParameter;
	pWavDecCompPrivate->pfSetInternalParam = SetInternalParam;

	/** overwrite audio decoder functions */
	pWavDecCompPrivate->pfCodecInit = InitWavDec; // use own init function
	pWavDecCompPrivate->pfCodecDecode = DecodeWav;

	pWavDecCompPrivate->uiAudioProcessMode = AUDIO_NORMAL_MODE; /* decoded pcm mode */
	pWavDecCompPrivate->uiCodecType = AUDIO_ID_WAV;
	pWavDecCompPrivate->iMinStreamSize = 0;

	LOGD("constructor of wav decoder component is completed!");

EXIT:
	return eError;
}
