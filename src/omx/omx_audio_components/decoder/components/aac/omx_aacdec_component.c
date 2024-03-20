/**

  @file omx_aacdec_component.c

  This component implement AAC decoder.

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

#include <omx_aacdec_component.h>
#include <OMX_TCC_Index.h>
#include "TCCFile.h"
#include "TCCMemory.h"

#ifdef HAVE_ANDROID_OS
#define USE_EXTERNAL_BUFFER 0
#define LOG_TAG	"OMX_TCC_AACDEC"
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
			tcc_printf(T_DEFAULT "[OMXAACDEC:V]" __VA_ARGS__);\
			tcc_printf(" - [%s - %d]\n"T_DEFAULT,__func__,__LINE__);\
		}\
	}
#define LOGD(...)	{\
		if (gs_iDbgmsgLevel_Adec >= 4) {\
			tcc_printf(TC_CYAN "[OMXAACDEC:D]" __VA_ARGS__);\
			tcc_printf(" - [%s - %d]\n"T_DEFAULT,__func__,__LINE__);\
		}\
	}
#define LOGI(...)	{\
		if (gs_iDbgmsgLevel_Adec >= 3) {\
			tcc_printf(TC_GREEN "[OMXAACDEC:I]" __VA_ARGS__);\
			tcc_printf(" - [%s - %d]\n"T_DEFAULT,__func__,__LINE__);\
		}\
	}
#define LOGW(...)	{\
		if (gs_iDbgmsgLevel_Adec >= 2) {\
			tcc_printf(TC_MAGENTA "[OMXAACDEC:W]" __VA_ARGS__);\
			tcc_printf(" - [%s - %d]\n"T_DEFAULT,__func__,__LINE__);\
		}\
	}
#define LOGE(...)	{\
		if (gs_iDbgmsgLevel_Adec >= 1) {\
			tcc_printf(TC_RED "[OMXAACDEC:E]" __VA_ARGS__);\
			tcc_printf(" - [%s - %d]\n"T_DEFAULT,__func__,__LINE__);\
		}\
	}
#endif /* HAVE_ANDROID_OS */

#ifdef DUMP_INPUT
static FILE *fdump;
#endif

static OMX_S32 OpenLatmDemuxer(omx_aacdec_component_PrivateType* pAacDecCompPrivate, unsigned char* pucLatmstream, int iStreamSize, int iNeedStreamInfo)
{
	OMX_S32 ret = 0;
	// setting callback functions for latm demuxer
	pAacDecCompPrivate->stCallbackFunc.m_pfMalloc = (void* (*) ( size_t ))malloc;
	pAacDecCompPrivate->stCallbackFunc.m_pfRealloc  = (void* (*) ( void*, size_t ))realloc;
	pAacDecCompPrivate->stCallbackFunc.m_pfFree= (void  (*) ( void* ))free;
	pAacDecCompPrivate->stCallbackFunc.m_pfMemcpy= (void* (*) ( void*, const void*, size_t ))memcpy;
	pAacDecCompPrivate->stCallbackFunc.m_pfMemmove= (void* (*) ( void*, const void*, size_t ))memmove;
	pAacDecCompPrivate->stCallbackFunc.m_pfMemset= (void  (*) ( void*, int, size_t ))memset;

	if(iNeedStreamInfo && pucLatmstream && iStreamSize )
	{
		pAacDecCompPrivate->pvSubParser = pAacDecCompPrivate->pfLatmInit( pucLatmstream, iStreamSize, (int *)&pAacDecCompPrivate->stStreamInfo.m_eSampleRate, (int *)&pAacDecCompPrivate->stStreamInfo.m_uiNumberOfChannel, (void*)&pAacDecCompPrivate->stCallbackFunc, TF_AAC_LOAS);
	}
	else
	{
		pAacDecCompPrivate->pvSubParser = pAacDecCompPrivate->pfLatmInit( NULL, 0, NULL, NULL, (void*)&pAacDecCompPrivate->stCallbackFunc, TF_AAC_LOAS);
	}

	if( pAacDecCompPrivate->pvSubParser == NULL )
	{
		ret = (-1);
	}

	return ret;
}

static OMX_S32 InitAacDec(omx_aacdec_component_PrivateType* pAacDecCompPrivate, OMX_BUFFERHEADERTYPE* pInputBuffer)
{
	AacDecoderPrivate *pAacDecPrivate = (AacDecoderPrivate *)pAacDecCompPrivate->stADecInit.m_unAudioCodecParams;
	OMX_S32 nRetValue = 0;
	
	/* setting callback functions */
	// set already
	

	/* set pcm-info struct */
	// set already
	
	/* set stream-info struct */
	// set already
	
	LOGI("In AAC: ch %lu, sf %lu", pAacDecCompPrivate->stAudioAac.nChannels, pAacDecCompPrivate->stAudioAac.nSampleRate);
	
	pAacDecPrivate->m_iAACForceUpmix = 1;
	//pAacDecPrivate->m_iAACForceUpsampling = 1;

	switch(pAacDecCompPrivate->stAudioAac.eAACProfile)
	{
	case OMX_AUDIO_AACObjectMain:
	case OMX_AUDIO_AACObjectLTP:
		LOGE("not supported profile");
		nRetValue = -1;
		break;
	case OMX_AUDIO_AACObjectLC:
	case OMX_AUDIO_AACObjectHE:
	case OMX_AUDIO_AACObjectHE_PS:
		pAacDecPrivate->m_iAACObjectType = OMX_AUDIO_AACObjectLC;
		break;
	default:
		LOGI("not support --> set default");
		pAacDecPrivate->m_iAACObjectType = OMX_AUDIO_AACObjectLC;
		break;
	}

	switch(pAacDecCompPrivate->stAudioAac.eAACStreamFormat)
	{
	case OMX_AUDIO_AACStreamFormatMP2ADTS:
	case OMX_AUDIO_AACStreamFormatMP4ADTS:
		pAacDecPrivate->m_iAACHeaderType = 1; // ADTS
		pAacDecCompPrivate->iMinStreamSize = 0;
		break;
	case OMX_AUDIO_AACStreamFormatMP4LOAS:
	case OMX_AUDIO_AACStreamFormatMP4LATM:
		pAacDecCompPrivate->iMinStreamSize = 0;
#if 0
		pAacDecCompPrivate->pfLatmInit = pAacDecCompPrivate->pDecFunctionList->pfCodecSpecific[0];
		pAacDecCompPrivate->pfLatmGetFrame = pAacDecCompPrivate->pDecFunctionList->pfCodecSpecific[1];
		pAacDecCompPrivate->pfLatmGetHeaderType = pAacDecCompPrivate->pDecFunctionList->pfCodecSpecific[2];
		pAacDecCompPrivate->pfLatmClose = pAacDecCompPrivate->pDecFunctionList->pfCodecSpecific[3];
		pAacDecPrivate->m_iAACHeaderType = 0; // RAW
		if (pAacDecCompPrivate->pfLatmInit == NULL || pAacDecCompPrivate->pfLatmGetFrame == NULL || pAacDecCompPrivate->pfLatmClose == NULL)
		{
			LOGE("No latm functions!");
			return -1;
		}
		pAacDecCompPrivate->uiNeedLatmParsing = 1;
#endif
		pAacDecPrivate->m_iAACHeaderType = 3; // LATM
		LOGI(" AAC Stream Type: LATM/LOAS");
		break;

	case OMX_AUDIO_AACStreamFormatRAW:
		pAacDecPrivate->m_iAACHeaderType = 0; // RAW
		break;
	case OMX_AUDIO_AACStreamFormatADIF:
		pAacDecPrivate->m_iAACHeaderType = 2; // ADIF
		break;
	default:
		pAacDecPrivate->m_iAACHeaderType = 0; // RAW
		break;
	}

#ifdef DUMP_INPUT
	fdump = fopen("/mnt/SD2p1/dump.bin", "wb");
	if (fdump == NULL) {
		LOGE("file open error");
	}
#endif

	if(pInputBuffer->nFlags & OMX_BUFFERFLAG_CODECCONFIG) {
		LOGI("%s config-data exist",__func__);
#ifdef DUMP_INPUT
		if (fdump) {
			fwrite(&pInputBuffer->nFilledLen, sizeof(int), 1, fdump);
			fwrite(pInputBuffer->pBuffer, sizeof(char), pInputBuffer->nFilledLen, fdump);
		}
#endif
	} else {
		if (pAacDecPrivate->m_iAACHeaderType == 0 && pInputBuffer->pBuffer && pInputBuffer->nFilledLen > 2) {
			if (((pInputBuffer->pBuffer[0] & 0xff) == 0xff) && ((pInputBuffer->pBuffer[1] & 0xf0) == 0xf0)) {
				pAacDecPrivate->m_iAACHeaderType = 1; // ADTS
			}
		}
	}

	if (nRetValue == 0) {
		nRetValue = pAacDecCompPrivate->pfCdkAudioDec(AUDIO_INIT, &pAacDecCompPrivate->hDecoderHandle, &pAacDecCompPrivate->stADecInit, NULL);

		/* reset OMX_AUDIO_PARAM_PCMMODETYPE Info */
		//pAacDecCompPrivate->stOutPcmMode.nChannels = pAacDecCompPrivate->stPcmInfo.m_uiNumberOfChannel;
		//pAacDecCompPrivate->stOutPcmMode.nSamplingRate = pAacDecCompPrivate->stPcmInfo.m_eSampleRate;

		/* reset OMX_AUDIO_PARAM_AACPROFILETYPE Info */
		pAacDecCompPrivate->stAudioAac.nSampleRate = pAacDecCompPrivate->stPcmInfo.m_eSampleRate;
		pAacDecCompPrivate->stAudioAac.nChannels = pAacDecCompPrivate->stPcmInfo.m_uiNumberOfChannel;

		LOGI("Out AAC: ch %lu, sf %lu, result %lu", pAacDecCompPrivate->stAudioAac.nChannels, pAacDecCompPrivate->stAudioAac.nSampleRate,nRetValue);
	}

	return nRetValue;
}


static OMX_S32 DecodeAAC(omx_aacdec_component_PrivateType* pAacDecCompPrivate, OMX_BUFFERHEADERTYPE* pInputBuffer, OMX_BUFFERHEADERTYPE* pOutputBuffer)
{
	OMX_S32 iRet = TCAS_SUCCESS;

	int iOutFrameSize = 0;
	pAacDecCompPrivate->iDecodedSamplePerCh = 0;

	if (pAacDecCompPrivate->uiNeedLatmParsing && pAacDecCompPrivate->pvSubParser == NULL)
	{
		if (OpenLatmDemuxer(pAacDecCompPrivate, pAacDecCompPrivate->stStreamInfo.m_pcStream, pAacDecCompPrivate->stStreamInfo.m_iStreamLength, 0))
		{
			LOGE("Error OpenLatmDemuxer");
			return -1;
		}
	}

	if(pAacDecCompPrivate->pvSubParser != NULL)
	{
		//if(pAacDecCompPrivate->isNewInputBuffer) // new buffer ?
		{
			iRet = pAacDecCompPrivate->pfLatmGetFrame( pAacDecCompPrivate->pvSubParser, pAacDecCompPrivate->stStreamInfo.m_pcStream, pAacDecCompPrivate->stStreamInfo.m_iStreamLength, &pAacDecCompPrivate->pAACRawData, &pAacDecCompPrivate->iAACRawDataSize, 0 );
			if( iRet < 0 )
			{
				LOGE("[AAC DEC] latm_parser_get_frame: Fatal error %ld!", iRet);
				pAacDecCompPrivate->stStreamInfo.m_iStreamLength = 0;
				return iRet;
			}

			if(pAacDecCompPrivate->iAACRawDataSize <= 0)
			{
				LOGD("[AAC DEC] latm_parser_get_frame: Need more data!");
				pAacDecCompPrivate->stStreamInfo.m_iStreamLength = 0;
				return TCAS_ERROR_MORE_DATA;
			}
			
			pAacDecCompPrivate->stStreamInfo.m_pcStream = pAacDecCompPrivate->pAACRawData;
			pAacDecCompPrivate->stStreamInfo.m_iStreamLength = pAacDecCompPrivate->iAACRawDataSize;
		}
	}

	//LOGD(" %s: input length %d", __func__, pAacDecCompPrivate->stStreamInfo.m_iStreamLength);
#ifdef DUMP_INPUT
	if (fdump) {
		fwrite(&pAacDecCompPrivate->stStreamInfo.m_iStreamLength, sizeof(int), 1, fdump);
		fwrite(pAacDecCompPrivate->stStreamInfo.m_pcStream, sizeof(char), pAacDecCompPrivate->stStreamInfo.m_iStreamLength, fdump);
	}
#endif

	while((pAacDecCompPrivate->stStreamInfo.m_iStreamLength > pAacDecCompPrivate->iMinStreamSize ) || ((pAacDecCompPrivate->stStreamInfo.m_iStreamLength > 0) && (pAacDecCompPrivate->isEndOfFile == 1)))
	{
		pAacDecCompPrivate->stPcmInfo.m_pvChannel[0]  = (void *)(pOutputBuffer->pBuffer + iOutFrameSize);

		iRet = pAacDecCompPrivate->pfCdkAudioDec(AUDIO_DECODE, &pAacDecCompPrivate->hDecoderHandle, &pAacDecCompPrivate->stStreamInfo, &pAacDecCompPrivate->stPcmInfo);

		if(iRet == TCAS_SUCCESS || iRet == TCAS_ERROR_CONCEALMENT_APPLIED)
		{
			iOutFrameSize += pAacDecCompPrivate->stPcmInfo.m_uiNumberOfChannel * pAacDecCompPrivate->stPcmInfo.m_uiSamplesPerChannel * sizeof(short);
			pAacDecCompPrivate->iDecodedSamplePerCh += pAacDecCompPrivate->stPcmInfo.m_uiSamplesPerChannel;
			//break;
		}
		else
		{
			if((int)iRet == TCAS_ERROR_MORE_DATA)
			{
				break;
			}

			if (((AacDecoderPrivate *)pAacDecCompPrivate->stADecInit.m_unAudioCodecParams)->m_iAACHeaderType != 1) // 1 = ADTS
			{
				pAacDecCompPrivate->stStreamInfo.m_iStreamLength = 0;
				break;
			}
		}
	}

	return iRet;
}

static OMX_S32 CloseAACDec(omx_aacdec_component_PrivateType* pAacDecCompPrivate)
{
	LOGD(" %s",__func__);

	if(pAacDecCompPrivate->pvSubParser != NULL)
	{
		pAacDecCompPrivate->pfLatmClose(pAacDecCompPrivate->pvSubParser);
	}

	pAacDecCompPrivate->pfCdkAudioDec(AUDIO_CLOSE, &pAacDecCompPrivate->hDecoderHandle, NULL, NULL);

	pAacDecCompPrivate->hDecoderHandle = 0;
#ifdef DUMP_INPUT
	if (fdump) {
		fclose(fdump);
		LOGE("dump file closed");
	}
#endif
	return 0;
}

static OMX_S32 SetInternalParam(omx_aacdec_component_PrivateType* pAacDecCompPrivate)
{
	/* reset OMX_AUDIO_PARAM_AACPROFILETYPE Info */
	pAacDecCompPrivate->stAudioAac.nSampleRate = pAacDecCompPrivate->stPcmInfo.m_eSampleRate;
	pAacDecCompPrivate->stAudioAac.nChannels = pAacDecCompPrivate->stPcmInfo.m_uiNumberOfChannel;
	return 0;
}

/** this function sets the parameter values regarding audio format & index */
static OMX_ERRORTYPE omx_aacdec_component_SetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_IN  OMX_PTR ComponentParameterStructure)
{

	OMX_ERRORTYPE eError = OMX_ErrorNone;
	OMX_PARAM_COMPONENTROLETYPE * pComponentRole;
	OMX_U32 portIndex;

	/* Check which structure we are being fed and make control its header */
	OMX_COMPONENTTYPE *pOpenmaxStandComp = (OMX_COMPONENTTYPE *)hComponent;
	omx_aacdec_component_PrivateType* pAacDecCompPrivate = pOpenmaxStandComp->pComponentPrivate;
	
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
			
			if (pAudioSrcPcmMode->nSamplingRate > pAacDecCompPrivate->nMaxSupportSamplerate)
			{
				LOGE(" This system doesn't support over %luHz sampling rate!", pAacDecCompPrivate->nMaxSupportSamplerate);
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
				LOGD(" %s Pcm:: nCh = %lu, %lu bit Pcm, SampleRate = %lu", __func__,
				    pAudioSrcPcmMode->nChannels, pAudioSrcPcmMode->nBitPerSample, pAudioSrcPcmMode->nSamplingRate);
				(void)memcpy(&pAacDecCompPrivate->stOutPcmMode, pAudioSrcPcmMode, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));          
			}
			break;			
		}

		case OMX_IndexParamStandardComponentRole:
		{
			pComponentRole = (OMX_PARAM_COMPONENTROLETYPE*)ComponentParameterStructure;
			if (strncmp( (char*) pComponentRole->cRole, TCC_OMX_COMPONENT_AACDEC_ROLE, sizeof(TCC_OMX_COMPONENT_AACDEC_ROLE))) 
			{
				eError = OMX_ErrorBadParameter;
			}
			break;
		}
		
		// codec specific parameters -------------------------------------------------------
		case OMX_IndexParamAudioAac:	
		{
			OMX_AUDIO_PARAM_PCMMODETYPE *pAudioParamPCM = &pAacDecCompPrivate->stOutPcmMode;
			portIndex = pAacDecCompPrivate->stAudioAac.nPortIndex;
			eError = omx_base_component_ParameterSanityCheck(hComponent,portIndex,ComponentParameterStructure,sizeof(OMX_AUDIO_PARAM_AACPROFILETYPE));
			if(eError!=OMX_ErrorNone)
			{ 
				LOGD("In %s Parameter Check Error=%x",__func__,eError); 
				break;
			}			
			
			if (((OMX_AUDIO_PARAM_AACPROFILETYPE *)(ComponentParameterStructure))->nPortIndex != portIndex)
			{
				eError = OMX_ErrorBadPortIndex;
				break;
			}
			
			(void)memcpy(&pAacDecCompPrivate->stAudioAac, ComponentParameterStructure, sizeof(OMX_AUDIO_PARAM_AACPROFILETYPE));

			pAacDecCompPrivate->stStreamInfo.m_eSampleRate = (TCAS_U32)pAacDecCompPrivate->stAudioAac.nSampleRate;
			pAacDecCompPrivate->stStreamInfo.m_uiNumberOfChannel = pAacDecCompPrivate->stAudioAac.nChannels;

			pAudioParamPCM->nSamplingRate = pAacDecCompPrivate->stAudioAac.nSampleRate;
			pAudioParamPCM->nChannels = pAacDecCompPrivate->stAudioAac.nChannels;
			/*if(pAacDecCompPrivate->stAudioAac.nSampleRate < 32000)
			{
				pAudioParamPCM->nSamplingRate = pAacDecCompPrivate->stAudioAac.nSampleRate * 2;
			}
			else
			{
				pAudioParamPCM->nSamplingRate = pAacDecCompPrivate->stAudioAac.nSampleRate;
			}*/
			
			if (pAacDecCompPrivate->stAudioAac.nChannels == 1)
			{
				pAudioParamPCM->nChannels = 2;
			}

			if (pAudioParamPCM->nChannels > 2)
			{
				if (pAacDecCompPrivate->stADecInit.m_iDownMixMode == 1)
				{
					pAudioParamPCM->nChannels = 2;
				}
				else
				{
					LOGI("multichannel out");
				}
			}
			
			LOGD(" %s AAC:: nCh = %lu, SampleRate = %lu", __func__, pAacDecCompPrivate->stAudioAac.nChannels, pAacDecCompPrivate->stAudioAac.nSampleRate);
			break;
		}
		
		default: /*Call the base component function*/
			eError = omx_audiodec_component_SetParameter(hComponent, nParamIndex, ComponentParameterStructure);
			break;
	}

	return eError;
}  

/** this function gets the parameters regarding audio formats and index */
static OMX_ERRORTYPE omx_aacdec_component_GetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_INOUT OMX_PTR ComponentParameterStructure)
{

	OMX_ERRORTYPE eError = OMX_ErrorNone;
	OMX_COMPONENTTYPE *pOpenmaxStandComp = (OMX_COMPONENTTYPE *)hComponent;
	omx_aacdec_component_PrivateType* pAacDecCompPrivate = pOpenmaxStandComp->pComponentPrivate;
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
						pAacDecCompPrivate->stOutPcmMode.nChannels, 
						pAacDecCompPrivate->stOutPcmMode.nBitPerSample, 
						pAacDecCompPrivate->stOutPcmMode.nSamplingRate);
				
				(void)memcpy(ComponentParameterStructure, &pAacDecCompPrivate->stOutPcmMode, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));
			}
			else
			{
				eError = OMX_ErrorBadPortIndex;
			}

			break;
		}
			
		case OMX_IndexParamAudioAac:
		{
			if ((eError = checkHeader(ComponentParameterStructure, sizeof(OMX_AUDIO_PARAM_AACPROFILETYPE))) != OMX_ErrorNone) 
			{ 
				break;
			}
			
			if (((OMX_AUDIO_PARAM_AACPROFILETYPE *)(ComponentParameterStructure))->nPortIndex != pAacDecCompPrivate->stAudioAac.nPortIndex)
			{
				return OMX_ErrorBadPortIndex;
			}

			LOGD(" %s AAC:: nCh = %lu, SampleRate = %lu", __func__,
			     pAacDecCompPrivate->stAudioAac.nChannels, pAacDecCompPrivate->stAudioAac.nSampleRate);
			(void)memcpy(ComponentParameterStructure, &pAacDecCompPrivate->stAudioAac, sizeof(OMX_AUDIO_PARAM_AACPROFILETYPE));
			break;
		}
		
		case OMX_IndexParamStandardComponentRole:
		{
			pComponentRole = (OMX_PARAM_COMPONENTROLETYPE*)ComponentParameterStructure;
			if ((eError = checkHeader(ComponentParameterStructure, sizeof(OMX_PARAM_COMPONENTROLETYPE))) != OMX_ErrorNone) 
			{ 
				break;
			}
			
			(void)strncpy( (char*) pComponentRole->cRole, TCC_OMX_COMPONENT_AACDEC_ROLE, sizeof(TCC_OMX_COMPONENT_AACDEC_ROLE));
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
	omx_aacdec_component_PrivateType* pAacDecCompPrivate;  

	if(strncmp(cComponentName, TCC_OMX_COMPONENT_AACDEC_NAME, sizeof(TCC_OMX_COMPONENT_AACDEC_NAME)))
	{
		// IL client specified an invalid component name
		LOGE("OMX_ErrorInvalidComponentName %s", cComponentName);
		eError = OMX_ErrorInvalidComponentName;
		goto EXIT;
	}
	
	eError = omx_audiodec_component_Init(pOpenmaxStandComp, cComponentName, sizeof(omx_aacdec_component_PrivateType));
	if(eError != OMX_ErrorNone)
	{
		goto EXIT;
	}

	pAacDecCompPrivate = pOpenmaxStandComp->pComponentPrivate;  
	
	/** parameters related to output port */
	// use default setting (see omx_audiodec_component_Init)

	/** parameters related to output audio param port */
	// use default setting (see omx_audiodec_component_Init)
	
	/** parameters related to input port */
	inPort = (omx_base_audio_PortType *) pAacDecCompPrivate->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];
	inPort->sPortParam.nBufferSize = DEFAULT_IN_BUFFER_SIZE*2;
	(void)strncpy(inPort->sPortParam.format.audio.cMIMEType, TCC_OMX_COMPONENT_AACDEC_MIME, sizeof(TCC_OMX_COMPONENT_AACDEC_MIME));
	inPort->sPortParam.format.audio.eEncoding = OMX_AUDIO_CodingAAC;
	inPort->sAudioParam.eEncoding = OMX_AUDIO_CodingAAC;
	
	/** parameters related to input audio param port */
	setHeader(&pAacDecCompPrivate->stAudioAac, sizeof(OMX_AUDIO_PARAM_AACPROFILETYPE));
	pAacDecCompPrivate->stAudioAac.nPortIndex = 0;
	pAacDecCompPrivate->stAudioAac.nChannels = 2;
	pAacDecCompPrivate->stAudioAac.nBitRate = 0;
	pAacDecCompPrivate->stAudioAac.nSampleRate = 44100;
	pAacDecCompPrivate->stAudioAac.nAudioBandWidth = 0;
	pAacDecCompPrivate->stAudioAac.nFrameLength = 2048; // use HE_PS frame size as default
	pAacDecCompPrivate->stAudioAac.eChannelMode = OMX_AUDIO_ChannelModeStereo;
	pAacDecCompPrivate->stAudioAac.eAACProfile = OMX_AUDIO_AACObjectHE_PS;    //OMX_AUDIO_AACObjectLC;
	pAacDecCompPrivate->stAudioAac.eAACStreamFormat = OMX_AUDIO_AACStreamFormatRAW;

	/** load audio decoder library */
	if (omx_audiodec_component_library_Load(pAacDecCompPrivate, TCC_AACDEC_LIB_NAME) != OMX_ErrorNone)
	{
		eError = OMX_ErrorInsufficientResources;
		goto EXIT;
	}
	
	/** overwrite omx component functions */
	pOpenmaxStandComp->SetParameter = omx_aacdec_component_SetParameter;
	pOpenmaxStandComp->GetParameter = omx_aacdec_component_GetParameter;
	pAacDecCompPrivate->pfSetInternalParam = SetInternalParam;

	/** overwrite audio decoder functions */
	pAacDecCompPrivate->pfCodecInit = InitAacDec; // use own init function
	pAacDecCompPrivate->pfCodecDecode = DecodeAAC;
	//pAacDecCompPrivate->pfCodecFlush = FlushCodec;
	pAacDecCompPrivate->pfCodecClose = CloseAACDec;


	pAacDecCompPrivate->isSilenceInsertionEnable = OMX_FALSE;

	pAacDecCompPrivate->uiAudioProcessMode = AUDIO_NORMAL_MODE; /* decoded pcm mode */
	pAacDecCompPrivate->uiCodecType = AUDIO_ID_AAC;
	pAacDecCompPrivate->iMinStreamSize = 0;
	
	LOGD("constructor of aac decoder component is completed!");	
	
EXIT:
	return eError;

}

#if 0
#define LATM_PARSER_LIB_NAME "libTCC.latmdmx.so"

static OMX_S32 OpenLatmDemuxer(omx_aacdec_component_PrivateType* pAacDecCompPrivate, unsigned char* pucLatmstream, int iStreamSize, int iNeedStreamInfo)
{
	// setting callback functions for latm demuxer
	pAacDecCompPrivate->stCallbackFunc.m_pfMalloc = (void* (*) ( size_t ))malloc;
	pAacDecCompPrivate->stCallbackFunc.m_pfRealloc  = (void* (*) ( void*, size_t ))realloc;
	pAacDecCompPrivate->stCallbackFunc.m_pfFree= (void  (*) ( void* ))free;
	pAacDecCompPrivate->stCallbackFunc.m_pfMemcpy= (void* (*) ( void*, const void*, size_t ))memcpy;
	pAacDecCompPrivate->stCallbackFunc.m_pfMemmove= (void* (*) ( void*, const void*, size_t ))memmove;
	pAacDecCompPrivate->stCallbackFunc.m_pfMemset= (void  (*) ( void*, int, size_t ))memset;

	pAacDecCompPrivate->pfLatmDLLModule = dlopen(LATM_PARSER_LIB_NAME, RTLD_LAZY | RTLD_GLOBAL);
    if( pAacDecCompPrivate->pfLatmDLLModule == NULL ) 
	{
        LOGE("Load library '%s' failed: %s", LATM_PARSER_LIB_NAME, dlerror());
        return -1;
    } 
	else 
    {
		LOGI("Library '%s' Loaded", LATM_PARSER_LIB_NAME);
    }

	pAacDecCompPrivate->pfLatmInit = dlsym(pAacDecCompPrivate->pfLatmDLLModule, "Latm_Init");
    if( pAacDecCompPrivate->pfLatmInit == NULL ) 
	{
		LOGE("Load symbol Latm_Init failed");
        return -1;
    }

	pAacDecCompPrivate->pfLatmClose = dlsym(pAacDecCompPrivate->pfLatmDLLModule, "Latm_Close");
    if( pAacDecCompPrivate->pfLatmClose == NULL ) 
	{
		LOGE("Load symbol Latm_Close failed");
        return -1;
    }

	pAacDecCompPrivate->pfLatmGetFrame = dlsym(pAacDecCompPrivate->pfLatmDLLModule, "Latm_GetFrame");
    if( pAacDecCompPrivate->pfLatmGetFrame == NULL ) 
	{
		LOGE("Load symbol Latm_GetFrame failed");
        return -1;
    }

	//pAacDecCompPrivate->pfLatmGetHeaderType = dlsym(pAacDecCompPrivate->pfLatmDLLModule, "Latm_GetHeaderType");
    //if( pAacDecCompPrivate->pfLatmGetHeaderType == NULL ) {
	//	LOGE("Load symbol Latm_GetHeaderType failed");
    //   return -1;
    //}

	if(iNeedStreamInfo && pucLatmstream && iStreamSize )
	{
		pAacDecCompPrivate->pvSubParser = pAacDecCompPrivate->pfLatmInit( pucLatmstream, iStreamSize, (int *)&pAacDecCompPrivate->stStreamInfo.m_eSampleRate, (int *)&pAacDecCompPrivate->stStreamInfo.m_uiNumberOfChannel, (void*)&pAacDecCompPrivate->stCallbackFunc, TF_AAC_LOAS);
	}
	else
	{
		pAacDecCompPrivate->pvSubParser = pAacDecCompPrivate->pfLatmInit( NULL, 0, NULL, NULL, (void*)&pAacDecCompPrivate->stCallbackFunc, TF_AAC_LOAS);	
	}

	if( pAacDecCompPrivate->pvSubParser == NULL )
	{
		if( pAacDecCompPrivate->pfLatmDLLModule != NULL )
		{
			dlclose(pAacDecCompPrivate->pfLatmDLLModule);
			pAacDecCompPrivate->pfLatmDLLModule = NULL;
		}
		return -1;				
	}

	return 0;
}

static OMX_S32 DecodeAAC(OMX_COMPONENTTYPE* openmaxStandComp, OMX_BUFFERHEADERTYPE* pInputBuffer, OMX_BUFFERHEADERTYPE* pOutputBuffer)
{
	OMX_S32 iOutFrameSize, iRet = TCAS_SUCCESS;
	omx_aacdec_component_PrivateType* pAacDecCompPrivate = openmaxStandComp->pComponentPrivate;

	iOutFrameSize = 0;
	pAacDecCompPrivate->iDecodedSamplePerCh = 0;

	if(pAacDecCompPrivate->pvSubParser != NULL)
	{	
		if(pInputBuffer->nOffset == 0) // new buffer ?
		{
			pAacDecCompPrivate->iCurrInputBufferOrgSize = pAacDecCompPrivate->stStreamInfo.m_iStreamLength;
			iRet = pAacDecCompPrivate->pfLatmGetFrame( pAacDecCompPrivate->pvSubParser, pAacDecCompPrivate->stStreamInfo.m_pcStream, pAacDecCompPrivate->stStreamInfo.m_iStreamLength, &pAacDecCompPrivate->pAACRawData, &pAacDecCompPrivate->iAACRawDataSize, 0 );
			if( iRet < 0 )
			{
				LOGD("[AAC DEC] latm_parser_get_frame: Fatal error %ld!", iRet);
				pAacDecCompPrivate->stStreamInfo.m_iStreamLength = 0;
				return iRet;
			}

			if(pAacDecCompPrivate->iAACRawDataSize <= 0)
			{
				LOGD("[AAC DEC] latm_parser_get_frame: Need more data!");
				pAacDecCompPrivate->stStreamInfo.m_iStreamLength = 0;
				return TCAS_ERROR_MORE_DATA;
			}

			pAacDecCompPrivate->stStreamInfo.m_pcStream = pAacDecCompPrivate->pAACRawData;
			pAacDecCompPrivate->stStreamInfo.m_iStreamLength = pAacDecCompPrivate->iAACRawDataSize;
		}
	}

	LOGD("%s: input length %d", __func__, pAacDecCompPrivate->stStreamInfo.m_iStreamLength);

	while((pAacDecCompPrivate->stStreamInfo.m_iStreamLength > pAacDecCompPrivate->iMinStreamSize ) || ((pAacDecCompPrivate->stStreamInfo.m_iStreamLength > 0) && (pAacDecCompPrivate->iEndOfFile == 1)))
	{
		pAacDecCompPrivate->stPcmInfo.m_pvChannel[0]  = (void *)(pOutputBuffer->pBuffer + iOutFrameSize);

		iRet = pAacDecCompPrivate->pExternalDec(AUDIO_DECODE, &pAacDecCompPrivate->iDecoderHandle, &pAacDecCompPrivate->stStreamInfo, &pAacDecCompPrivate->stPcmInfo);
		
		if(iRet == TCAS_SUCCESS || iRet == TCAS_ERROR_CONCEALMENT_APPLIED)
		{
			iOutFrameSize += pAacDecCompPrivate->stPcmInfo.m_uiNumberOfChannel * pAacDecCompPrivate->stPcmInfo.m_uiSamplesPerChannel * sizeof(short);
			pAacDecCompPrivate->iDecodedSamplePerCh += pAacDecCompPrivate->stPcmInfo.m_uiSamplesPerChannel;
			break;
		}
		else
		{
			if(iRet != TCAS_ERROR_SKIP_FRAME)
			{
				break;
			}
		}
	}

	return iRet;
}

static OMX_S32 CloseAACDec(OMX_COMPONENTTYPE* openmaxStandComp)
{
	omx_aacdec_component_PrivateType* pAacDecCompPrivate = openmaxStandComp->pComponentPrivate;

	LOGD("%s",__func__);

	if(pAacDecCompPrivate->pvSubParser != NULL)
	{
		pAacDecCompPrivate->pfLatmClose(pAacDecCompPrivate->pvSubParser);
		if( pAacDecCompPrivate->pfLatmDLLModule != NULL)
		{
			dlclose(pAacDecCompPrivate->pfLatmDLLModule);
			pAacDecCompPrivate->pfLatmDLLModule = NULL;
		}
	}

	pAacDecCompPrivate->pExternalDec(AUDIO_CLOSE, &pAacDecCompPrivate->iDecoderHandle, NULL, NULL);

	pAacDecCompPrivate->iDecoderHandle = 0;

	return 0;
}
#endif


