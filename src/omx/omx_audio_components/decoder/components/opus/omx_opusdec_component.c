/**

  @file omx_opusdec_component.c

  This component implement OPUS decoder.

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

#include <omx_opusdec_component.h>
#include <OMX_TCC_Index.h>
#include "TCCFile.h"
#include "TCCMemory.h"

#ifdef HAVE_ANDROID_OS
#define USE_EXTERNAL_BUFFER 0
#define LOG_TAG	"OMX_TCC_OPUSDEC"
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
			tcc_printf(T_DEFAULT "[OMXOPUSDEC:V]" __VA_ARGS__);\
			tcc_printf(" - [%s - %d]\n"T_DEFAULT,__func__,__LINE__);\
		}\
	}
#define LOGD(...)	{\
		if (gs_iDbgmsgLevel_Adec >= 4) {\
			tcc_printf(TC_CYAN "[OMXOPUSDEC:D]" __VA_ARGS__);\
			tcc_printf(" - [%s - %d]\n"T_DEFAULT,__func__,__LINE__);\
		}\
	}
#define LOGI(...)	{\
		if (gs_iDbgmsgLevel_Adec >= 3) {\
			tcc_printf(TC_GREEN "[OMXOPUSDEC:I]" __VA_ARGS__);\
			tcc_printf(" - [%s - %d]\n"T_DEFAULT,__func__,__LINE__);\
		}\
	}
#define LOGW(...)	{\
		if (gs_iDbgmsgLevel_Adec >= 2) {\
			tcc_printf(TC_MAGENTA "[OMXOPUSDEC:W]" __VA_ARGS__);\
			tcc_printf(" - [%s - %d]\n"T_DEFAULT,__func__,__LINE__);\
		}\
	}
#define LOGE(...)	{\
		if (gs_iDbgmsgLevel_Adec >= 1) {\
			tcc_printf(TC_RED "[OMXOPUSDEC:E]" __VA_ARGS__);\
			tcc_printf(" - [%s - %d]\n"T_DEFAULT,__func__,__LINE__);\
		}\
	}
#endif /* HAVE_ANDROID_OS */

static OMX_S32 InitOpusDec(omx_opusdec_component_PrivateType* pOpusDecCompPrivate, OMX_BUFFERHEADERTYPE* pInputBuffer)
{	
	LOGD(" %s start",__func__);

	OMX_S32 nRetValue = 0;

	/* setting callback functions */
	// set already
	
	/* set pcm-info struct */
	// set already
	
	/* set stream-info struct */
	// set already

	// setting extra info
	pOpusDecCompPrivate->stADecInit.m_pucExtraData = pInputBuffer->pBuffer;
	pOpusDecCompPrivate->stADecInit.m_iExtraDataLen = pInputBuffer->nFilledLen;

	nRetValue = pOpusDecCompPrivate->pfCdkAudioDec(AUDIO_INIT, &pOpusDecCompPrivate->hDecoderHandle, &pOpusDecCompPrivate->stADecInit, NULL);

	LOGD(" %s result %ld",__func__, nRetValue);
	
	return nRetValue;
}

static OMX_S32 SetInternalParam(omx_opusdec_component_PrivateType* pOpusDecCompPrivate)
{
	LOGD(" %s Start",__func__);		
	/* reset OMX_AUDIO_PARAM_PCMMODETYPE Info */
	pOpusDecCompPrivate->stOutPcmMode.nChannels = pOpusDecCompPrivate->stPcmInfo.m_uiNumberOfChannel;
	pOpusDecCompPrivate->stOutPcmMode.nSamplingRate = pOpusDecCompPrivate->stPcmInfo.m_eSampleRate;

	/* reset OMX_AUDIO_PARAM_OPUSTYPE Info */
	pOpusDecCompPrivate->stAudioOpus.nSampleRate = pOpusDecCompPrivate->stPcmInfo.m_eSampleRate;
	pOpusDecCompPrivate->stAudioOpus.nChannels = pOpusDecCompPrivate->stPcmInfo.m_uiNumberOfChannel;
	LOGD(" %s End",__func__);	

	return 0;
}

/** this function sets the parameter values regarding audio format & index */
static OMX_ERRORTYPE omx_opusdec_component_SetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_IN  OMX_PTR ComponentParameterStructure)
{
	LOGD(" %s Start",__func__);	
	OMX_ERRORTYPE eError = OMX_ErrorNone;
	OMX_PARAM_COMPONENTROLETYPE * pComponentRole;
	OMX_U32 portIndex;

	/* Check which structure we are being fed and make control its header */
	OMX_COMPONENTTYPE *pOpenmaxStandComp = (OMX_COMPONENTTYPE *)hComponent;
	omx_opusdec_component_PrivateType* pOpusDecCompPrivate = pOpenmaxStandComp->pComponentPrivate;
	
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
			
			if (pAudioSrcPcmMode->nSamplingRate > pOpusDecCompPrivate->nMaxSupportSamplerate)
			{
				LOGE(" This system doesn't support over %luHz sampling rate!", pOpusDecCompPrivate->nMaxSupportSamplerate);
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
				LOGD(" In %s:: nCh = %lu, %lu bit Pcm, SampleRate = %lu"
				, __func__, pAudioSrcPcmMode->nChannels, pAudioSrcPcmMode->nBitPerSample, pAudioSrcPcmMode->nSamplingRate);
				(void)memcpy(&pOpusDecCompPrivate->stOutPcmMode, pAudioSrcPcmMode, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));          
			}
			break;			
		}

		case OMX_IndexParamStandardComponentRole:
		{
			pComponentRole = (OMX_PARAM_COMPONENTROLETYPE*)ComponentParameterStructure;
			if (strncmp( (char*) pComponentRole->cRole, TCC_OMX_COMPONENT_OPUSDEC_ROLE, sizeof(TCC_OMX_COMPONENT_OPUSDEC_ROLE))) 
			{
				eError = OMX_ErrorBadParameter;
			}
			break;
		}
		
		// codec specific parameters -------------------------------------------------------
		case OMX_IndexParamAudioOPUS:	
		{
			OMX_AUDIO_PARAM_PCMMODETYPE *pAudioParamPCM = &pOpusDecCompPrivate->stOutPcmMode;
			portIndex = pOpusDecCompPrivate->stAudioOpus.nPortIndex;
			eError = omx_base_component_ParameterSanityCheck(hComponent,portIndex,ComponentParameterStructure,sizeof(OMX_AUDIO_PARAM_OPUSTYPE));
			if(eError!=OMX_ErrorNone)
			{ 
				LOGD("In %s Parameter Check Error=%x",__func__,eError); 
				break;
			}			
			
			if (((OMX_AUDIO_PARAM_OPUSTYPE *)(ComponentParameterStructure))->nPortIndex != portIndex)
			{
				eError = OMX_ErrorBadPortIndex;
				break;
			}
			
			(void)memcpy(&pOpusDecCompPrivate->stAudioOpus, ComponentParameterStructure, sizeof(OMX_AUDIO_PARAM_OPUSTYPE));

			pOpusDecCompPrivate->stStreamInfo.m_eSampleRate = (TCAS_U32)pOpusDecCompPrivate->stAudioOpus.nSampleRate;
			pOpusDecCompPrivate->stStreamInfo.m_uiNumberOfChannel = pOpusDecCompPrivate->stAudioOpus.nChannels;
			
			if(pAudioParamPCM->nSamplingRate != pOpusDecCompPrivate->stAudioOpus.nSampleRate)
			{
				pAudioParamPCM->nSamplingRate = pOpusDecCompPrivate->stAudioOpus.nSampleRate;
			}
			
			if (pAudioParamPCM->nChannels != pOpusDecCompPrivate->stAudioOpus.nChannels) 
			{
				pAudioParamPCM->nChannels = pOpusDecCompPrivate->stAudioOpus.nChannels;
			}
			LOGD(" %s Param :: nCh = %lu, SampleRate = %lu", __func__, pOpusDecCompPrivate->stAudioOpus.nChannels, pOpusDecCompPrivate->stAudioOpus.nSampleRate);

			if (pAudioParamPCM->nChannels > 2) // Opus Decoder only Supports mono and stereo. 
			{
				LOGD("Error Opus :: Opus Only Supports mono and stereo. request %lu", pAudioParamPCM->nChannels);
				return -1;
			}			

			break;
		}
		
		default: /*Call the base component function*/
			return omx_audiodec_component_SetParameter(hComponent, nParamIndex, ComponentParameterStructure);

	}
	LOGD(" %s End",__func__);	
	return eError;
}


/** this function gets the parameters regarding audio formats and index */
static OMX_ERRORTYPE omx_opusdec_component_GetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_INOUT OMX_PTR ComponentParameterStructure)
{
	LOGD(" %s Start", __func__ );	
	OMX_ERRORTYPE eError = OMX_ErrorNone;
	OMX_COMPONENTTYPE *pOpenmaxStandComp = (OMX_COMPONENTTYPE *)hComponent;
	omx_opusdec_component_PrivateType* pOpusDecCompPrivate = pOpenmaxStandComp->pComponentPrivate;
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
				LOGD(" %s Pcm:: nCh = %lu, %lu bit Pcm, SampleRate = %lu", 
						__func__, 
						pOpusDecCompPrivate->stOutPcmMode.nChannels, 
						pOpusDecCompPrivate->stOutPcmMode.nBitPerSample, 
						pOpusDecCompPrivate->stOutPcmMode.nSamplingRate);				
				(void)memcpy(ComponentParameterStructure, &pOpusDecCompPrivate->stOutPcmMode, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));
			}
			else
			{
				eError = OMX_ErrorBadPortIndex;
			}

			break;
		}
			
		case OMX_IndexParamAudioOPUS:			
		{
			if ((eError = checkHeader(ComponentParameterStructure, sizeof(OMX_AUDIO_PARAM_OPUSTYPE))) != OMX_ErrorNone) 
			{ 
				break;
			}
			
			if (((OMX_AUDIO_PARAM_OPUSTYPE *)(ComponentParameterStructure))->nPortIndex != pOpusDecCompPrivate->stAudioOpus.nPortIndex)
			{
				return OMX_ErrorBadPortIndex;
			}
				// output is PCM
			LOGD("@@@ Opus:  %s nCh = %lu, SampleRate = %lu", 
						__func__, 
						pOpusDecCompPrivate->stAudioOpus.nChannels, 
						pOpusDecCompPrivate->stAudioOpus.nSampleRate);

			(void)memcpy(ComponentParameterStructure, &pOpusDecCompPrivate->stAudioOpus, sizeof(OMX_AUDIO_PARAM_OPUSTYPE));
			break;
		}
		
		case OMX_IndexParamStandardComponentRole:
		{
			pComponentRole = (OMX_PARAM_COMPONENTROLETYPE*)ComponentParameterStructure;
			if ((eError = checkHeader(ComponentParameterStructure, sizeof(OMX_PARAM_COMPONENTROLETYPE))) != OMX_ErrorNone) 
			{ 
				break;
			}
			
			(void)strncpy( (char*) pComponentRole->cRole, TCC_OMX_COMPONENT_OPUSDEC_ROLE, sizeof(TCC_OMX_COMPONENT_OPUSDEC_ROLE));
			break;
		}
		default: /*Call the audiodec component function*/
		{
			return omx_audiodec_component_GetParameter(hComponent, nParamIndex, ComponentParameterStructure);
		}
	}
	LOGD(" %s End",__func__);
	return eError;

}

OMX_ERRORTYPE OMX_ComponentInit(OMX_COMPONENTTYPE *pOpenmaxStandComp, OMX_STRING cComponentName)
{
	OMX_ERRORTYPE eError = OMX_ErrorNone;
	omx_base_audio_PortType *inPort;
	omx_opusdec_component_PrivateType* pOpusDecCompPrivate;
	
	if(strncmp(cComponentName, TCC_OMX_COMPONENT_OPUSDEC_NAME, sizeof(TCC_OMX_COMPONENT_OPUSDEC_NAME)))
	{   
		// IL client specified an invalid component name
		LOGE(" In %s OMX_ErrorInvalidComponentName %s",__func__, cComponentName);
		eError = OMX_ErrorInvalidComponentName;
		goto EXIT;
	}
	
	eError = omx_audiodec_component_Init(pOpenmaxStandComp, cComponentName, sizeof(omx_opusdec_component_PrivateType));
	if(eError != OMX_ErrorNone)
	{
		goto EXIT;
	}

	pOpusDecCompPrivate = pOpenmaxStandComp->pComponentPrivate;  
	
	/** parameters related to output port */
	// use default setting (in omx_audiodec_component_Init)

	/** parameters related to output audio param port */
	// use default setting (in omx_audiodec_component_Init)
	
	/** parameters related to input port */
	inPort = (omx_base_audio_PortType *) pOpusDecCompPrivate->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];
	inPort->sPortParam.nBufferSize = DEFAULT_IN_BUFFER_SIZE*4; // CHECK (by.Claire Lee)
	(void)strncpy(inPort->sPortParam.format.audio.cMIMEType, TCC_OMX_COMPONENT_OPUSDEC_MIME, sizeof(TCC_OMX_COMPONENT_OPUSDEC_MIME));
	inPort->sPortParam.format.audio.eEncoding = OMX_AUDIO_CodingOPUS;
	inPort->sAudioParam.eEncoding = OMX_AUDIO_CodingOPUS;
	
    /** parameters related to input audio param port */
	setHeader(&pOpusDecCompPrivate->stAudioOpus, sizeof(OMX_AUDIO_PARAM_OPUSTYPE));
	pOpusDecCompPrivate->stAudioOpus.nPortIndex = 0;
	pOpusDecCompPrivate->stAudioOpus.nChannels = 2;
	pOpusDecCompPrivate->stAudioOpus.nSampleRate = 48000;

	/** load audio decoder library */
	if (omx_audiodec_component_library_Load(pOpusDecCompPrivate, TCC_OPUSDEC_LIB_NAME) != OMX_ErrorNone)
	{
		eError = OMX_ErrorInsufficientResources;
		goto EXIT;
	}
	
	/** overwrite omx component functions */
	pOpenmaxStandComp->SetParameter = omx_opusdec_component_SetParameter;;
	pOpenmaxStandComp->GetParameter = omx_opusdec_component_GetParameter;
	pOpusDecCompPrivate->pfSetInternalParam = SetInternalParam;

	/** overwrite audio decoder functions */
	pOpusDecCompPrivate->pfCodecInit = InitOpusDec; // use own init function
	//pOpusDecCompPrivate->pfCodecDecode = DecodeOpus;

	pOpusDecCompPrivate->uiAudioProcessMode = AUDIO_NORMAL_MODE; /* decoded pcm mode */
	pOpusDecCompPrivate->uiCodecType = AUDIO_ID_OPUS;
	pOpusDecCompPrivate->iMinStreamSize = 0;
	
	LOGD("constructor of opus decoder component is completed!");		
EXIT:
	return eError;

}
