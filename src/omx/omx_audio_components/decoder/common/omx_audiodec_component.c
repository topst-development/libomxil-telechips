/**

  @file omx_audiodec_component.c

  This component implement audio decoder's common part.

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

#include <omx_audiodec_component.h>
#include <OMX_TCC_Index.h>
#include <OMX_IndexExt.h>
#include <dlfcn.h>
#include "TCCFile.h"
#include "TCCMemory.h"

#include "omx_comp_debug_levels.h"

#define LOGV(...) {\
  if (gs_iDbgmsgLevel_Audio >= 5) {\
    tcc_printf(T_DEFAULT "[OMXAUDIODEC:V]" __VA_ARGS__);\
    tcc_printf(" - [%s - %d]\n"T_DEFAULT,__func__,__LINE__);\
  }\
  }
#define LOGD(...) {\
  if (gs_iDbgmsgLevel_Audio >= 4) {\
    tcc_printf(TC_CYAN "[OMXAUDIODEC:D]" __VA_ARGS__);\
    tcc_printf(" - [%s - %d]\n"T_DEFAULT,__func__,__LINE__);\
  }\
  }
#define LOGI(...) {\
  if (gs_iDbgmsgLevel_Audio >= 3) {\
    tcc_printf(TC_GREEN "[OMXAUDIODEC:I]" __VA_ARGS__);\
    tcc_printf(" - [%s - %d]\n"T_DEFAULT,__func__,__LINE__);\
  }\
  }
#define LOGW(...) {\
  if (gs_iDbgmsgLevel_Audio >= 2) {\
    tcc_printf(TC_MAGENTA "[OMXAUDIODEC:W]" __VA_ARGS__);\
    tcc_printf(" - [%s - %d]\n"T_DEFAULT,__func__,__LINE__);\
  }\
  }
#define LOGE(...) {\
  if (gs_iDbgmsgLevel_Audio >= 1) {\
    tcc_printf(TC_RED "[OMXAUDIODEC:E]" __VA_ARGS__);\
    tcc_printf(" - [%s - %d]\n"T_DEFAULT,__func__,__LINE__);\
  }\
  }

#if defined(TCC_LLOG_ENABLE)
#define LOG_TAG "OMX_TCC_AUDIODEC"
#include <log/log.h>
#define OMX_LOGD ALOGD
#else
#define OMX_LOGD(fmt, ...)
#endif
OMX_ERRORTYPE omx_audiodec_component_library_Load(omx_audiodec_component_PrivateType* pADecCompPrivate, OMX_STRING cAudioLibName)
{
  OMX_ERRORTYPE ret = OMX_ErrorNone;
  if(cAudioLibName == NULL){
    ret = OMX_ErrorInsufficientResources;
  }
  else  {
    pADecCompPrivate->pAudioDLLModule = dlopen(cAudioLibName , RTLD_LAZY | RTLD_GLOBAL);
    if( pADecCompPrivate->pAudioDLLModule == NULL )
    {
      LOGE("Error AudioDecoder library %s Not Found", cAudioLibName);
      ret = OMX_ErrorInsufficientResources;
    }
    else
    {
      LOGI(" Audio Library '%s' loaded!", cAudioLibName);
      pADecCompPrivate->pDecFunctionList = dlsym(pADecCompPrivate->pAudioDLLModule, "FucntionList");
      if(pADecCompPrivate->pDecFunctionList == NULL)
      {
        LOGE("No decoder function table!");
        dlclose(pADecCompPrivate->pAudioDLLModule);
        pADecCompPrivate->pAudioDLLModule = NULL;
        ret = OMX_ErrorInsufficientResources;
      }

      if (ret == OMX_ErrorNone) {
        pADecCompPrivate->pfCdkAudioDec = pADecCompPrivate->pDecFunctionList->pfMainFunction;
        if(pADecCompPrivate->pfCdkAudioDec == NULL)
        {
          LOGE("No decoder main function!");
          dlclose(pADecCompPrivate->pAudioDLLModule);
          pADecCompPrivate->pAudioDLLModule = NULL;
          ret = OMX_ErrorInsufficientResources;
        }
      }
    }
  }
  return ret;
}

OMX_ERRORTYPE omx_audiodec_component_Init(OMX_COMPONENTTYPE *pOpenmaxStandComp, OMX_STRING cComponentName, OMX_U32 nComponentPrivateSize)
{

  OMX_ERRORTYPE eError = OMX_ErrorNone;
  omx_audiodec_component_PrivateType* pADecCompPrivate;
  omx_base_audio_PortType *outPort;
  OMX_U32 i;

  CHECK_AUDIOLOG_LEVEL();
  CHECK_ADECLOG_LEVEL();

  LOGD("In %s", __func__);

  if (!pOpenmaxStandComp->pComponentPrivate)
  {
    pOpenmaxStandComp->pComponentPrivate = TCC_calloc(1, nComponentPrivateSize);

    if (pOpenmaxStandComp->pComponentPrivate==NULL)
    {
      eError = OMX_ErrorInsufficientResources;
      goto EXIT;
    }
  }
  else
  {
    LOGE("In %s, Error Component %x Already Allocated",
        __func__, (int)pOpenmaxStandComp->pComponentPrivate);
  }

  pADecCompPrivate = pOpenmaxStandComp->pComponentPrivate;
  pADecCompPrivate->ports = NULL;

  /** we could create our own port structures here
  * fixme maybe the base class could use a "port factory" function pointer?
  */
  eError = omx_base_filter_Constructor(pOpenmaxStandComp, cComponentName);
  if (eError)
  {
    goto EXIT;
  }

  /* Domain specific section for the ports. */
  /* first we set the parameter common to both formats */
  /* parameters related to input port which does not depend upon input audio format  */
  /* Allocate Ports and call port constructor. */

  pADecCompPrivate->sPortTypesParam[OMX_PortDomainAudio].nStartPortNumber = 0;
  pADecCompPrivate->sPortTypesParam[OMX_PortDomainAudio].nPorts = 2;

  if (/*pADecCompPrivate->sPortTypesParam[OMX_PortDomainAudio].nPorts &&*/ !pADecCompPrivate->ports)
  {
    pADecCompPrivate->ports = TCC_calloc(pADecCompPrivate->sPortTypesParam[OMX_PortDomainAudio].nPorts, sizeof(omx_base_PortType *));
    if (!pADecCompPrivate->ports)
    {
      eError = OMX_ErrorInsufficientResources;
      goto EXIT;
    }
    for (i=0; i < pADecCompPrivate->sPortTypesParam[OMX_PortDomainAudio].nPorts; i++)
    {
      pADecCompPrivate->ports[i] = TCC_calloc(1, sizeof(omx_base_audio_PortType));
      if (!pADecCompPrivate->ports[i])
      {
        eError = OMX_ErrorInsufficientResources;
        goto EXIT;
      }
    }
 
    base_audio_port_Constructor(pOpenmaxStandComp, &pADecCompPrivate->ports[0], 0, OMX_TRUE); // input
    base_audio_port_Constructor(pOpenmaxStandComp, &pADecCompPrivate->ports[1], 1, OMX_FALSE); // output
  }

  /** parameters related to output port */
  /** output is pcm mode for all decoders - so generalise it */
  outPort = (omx_base_audio_PortType *) pADecCompPrivate->ports[OMX_BASE_FILTER_OUTPUTPORT_INDEX];
  outPort->sPortParam.format.audio.eEncoding = OMX_AUDIO_CodingPCM;
  outPort->sPortParam.nBufferSize = AUDIO_DEC_OUT_BUFFER_SIZE;
  outPort->sAudioParam.eEncoding = OMX_AUDIO_CodingPCM;

  /** parameters related to output audio param port */
  setHeader(&pADecCompPrivate->stOutPcmMode,sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));
  pADecCompPrivate->stOutPcmMode.nPortIndex = 1;
  pADecCompPrivate->stOutPcmMode.nChannels = 2;
  pADecCompPrivate->stOutPcmMode.eNumData = OMX_NumericalDataSigned;
  pADecCompPrivate->stOutPcmMode.eEndian = OMX_EndianLittle;
  pADecCompPrivate->stOutPcmMode.bInterleaved = OMX_TRUE;
  pADecCompPrivate->stOutPcmMode.nBitPerSample = 16;
  pADecCompPrivate->stOutPcmMode.nSamplingRate = 44100;
  pADecCompPrivate->stOutPcmMode.ePCMMode = OMX_AUDIO_PCMModeLinear;
  pADecCompPrivate->stOutPcmMode.eChannelMapping[0] = OMX_AUDIO_ChannelLF;
  pADecCompPrivate->stOutPcmMode.eChannelMapping[1] = OMX_AUDIO_ChannelRF;

  /* set default omx-component functions */
  pADecCompPrivate->BufferMgmtCallback = omx_audiodec_component_BufferMgmtCallback;
  pADecCompPrivate->messageHandler = omx_audiodec_component_MessageHandler;
  pADecCompPrivate->destructor = omx_audiodec_component_Destructor;
  pOpenmaxStandComp->SetParameter = omx_audiodec_component_SetParameter;
  pOpenmaxStandComp->GetParameter = omx_audiodec_component_GetParameter;
  pOpenmaxStandComp->GetExtensionIndex = omx_audiodec_component_GetExtensionIndex;
  //pOpenmaxStandComp->SetConfig = omx_audiodec_component_SetConfig;
  //pOpenmaxStandComp->ComponentRoleEnum = omx_audiodec_component_ComponentRoleEnum;

  /* set default audio decoder functions */
  pADecCompPrivate->pfCodecInit = InitDecoder;
  pADecCompPrivate->pfCodecDecode = DecodeFrame;
  pADecCompPrivate->pfCodecFlush = FlushCodec;
  pADecCompPrivate->pfCodecClose = CloseCodec;

  pADecCompPrivate->isDecoderReady = OMX_FALSE;
  pADecCompPrivate->isPcmSplitEnabled = OMX_FALSE;
  pADecCompPrivate->isSilenceInsertionEnable = OMX_FALSE;
  pADecCompPrivate->nMaxSupportSamplerate = 0x7FFFFFFF;

  eError = omx_audiodec_component_LibInit(pADecCompPrivate);

  memset(&pADecCompPrivate->stStreamInfo, 0x00, sizeof(audio_streaminfo_t));
  memset(&pADecCompPrivate->stPcmInfo, 0x00, sizeof(audio_pcminfo_t));
  memset(&pADecCompPrivate->stADecInit, 0x00, sizeof(adec_init_t));

  pADecCompPrivate->stADecInit.m_iDownMixMode = 1; //if(ch > 2) downmix to stereo

EXIT:
  LOGD("Out %s ret = %ld", __func__, (OMX_S32)eError);
  return eError;

}


// library init function
OMX_ERRORTYPE omx_audiodec_component_LibInit(omx_audiodec_component_PrivateType* pADecCompPrivate)
{
  pADecCompPrivate->isPrevDecFail = OMX_FALSE;
  pADecCompPrivate->iNumOfSeek = 0;
  pADecCompPrivate->iPrevTS = -1;
  pADecCompPrivate->iMuteSamples = 0;
  pADecCompPrivate->uiNumSamplesOutput = 0;
  pADecCompPrivate->iPrevOriginalTS = -1;
  pADecCompPrivate->pRest = NULL;
  pADecCompPrivate->uiSplitBufferSampleCapacity = 0;
  pADecCompPrivate->uiInputBufferCount = 0;
  LOGD("%s", __func__);

  return OMX_ErrorNone;
}


// library de-init function
OMX_ERRORTYPE omx_audiodec_component_LibDeinit(omx_audiodec_component_PrivateType* pADecCompPrivate)
{
  LOGD("In %s", __func__);
  pADecCompPrivate->isDecoderReady = OMX_FALSE;
  pADecCompPrivate->pfCodecClose(pADecCompPrivate);
  return OMX_ErrorNone;
}

OMX_ERRORTYPE omx_audiodec_component_MessageHandler(OMX_COMPONENTTYPE* pOpenmaxStandComp, internalRequestMessageType *message)
{
  omx_audiodec_component_PrivateType* pADecCompPrivate = (omx_audiodec_component_PrivateType*)pOpenmaxStandComp->pComponentPrivate;
  OMX_ERRORTYPE eError;
  OMX_STATETYPE eCurrentState = pADecCompPrivate->state;
  LOGD("In %s", __func__);

  /** Execute the base message handling */
  eError = omx_base_component_MessageHandler(pOpenmaxStandComp, message);

  if (message->messageType == (int)OMX_CommandStateSet){
    if ((message->messageParam == (int)OMX_StateIdle) &&
		(eCurrentState == OMX_StateExecuting || eCurrentState == OMX_StatePause))
    {
      if(pADecCompPrivate->pucStreamBuffer != NULL) {
        free(pADecCompPrivate->pucStreamBuffer);
      }

      if (pADecCompPrivate->isDecoderReady == OMX_TRUE) {
        pADecCompPrivate->isDecoderReady = OMX_FALSE;
        (void)pADecCompPrivate->pfCodecClose(pADecCompPrivate);
        if(eError!=OMX_ErrorNone) {
          LOGE("In %s Audio decoder library Deinit Failed Error=%lx", __func__,(OMX_U32)eError);
          //return eError;
        }
      }
    }
  }
  return eError;
}

OMX_ERRORTYPE omx_audiodec_component_GetExtensionIndex(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_STRING cParameterName,
  OMX_OUT OMX_INDEXTYPE* pIndexType)
{

  LOGD("In  %s",__func__);
  if(strncmp(cParameterName,TCC_AUDIO_FILE_OPEN_STRING,sizeof(TCC_AUDIO_FILE_OPEN_STRING)) == 0) {
    *pIndexType = OMX_IndexVendorParamFileOpen;
  } else if(strncmp(cParameterName,TCC_AUDIO_POWERSPECTUM_STRING,sizeof(TCC_AUDIO_POWERSPECTUM_STRING)) == 0){
    *pIndexType = OMX_IndexVendorConfigPowerSpectrum;
  } else if(strncmp(cParameterName,TCC_AUDIO_MEDIA_INFO_STRING,sizeof(TCC_AUDIO_MEDIA_INFO_STRING)) == 0){
    *pIndexType = OMX_IndexVendorParamMediaInfo;
  }else if(strncmp(cParameterName,TCC_AUDIO_ENERGYVOLUME_STRING,sizeof(TCC_AUDIO_ENERGYVOLUME_STRING)) == 0){
    *pIndexType = OMX_IndexVendorConfigEnergyVolume;
  }else{
    return OMX_ErrorBadParameter;
  }

  return OMX_ErrorNone;
}

/** The destructor */
OMX_ERRORTYPE omx_audiodec_component_Destructor(OMX_COMPONENTTYPE *pOpenmaxStandComp)
{
  omx_audiodec_component_PrivateType* pADecCompPrivate = pOpenmaxStandComp->pComponentPrivate;
  OMX_U32 i;

  LOGD("In %s", __func__);

  if (pADecCompPrivate->pRest != NULL)
  {
    free(pADecCompPrivate->pRest);
    pADecCompPrivate->pRest = NULL;
  }

  /* frees port/s */
  if (pADecCompPrivate->ports)
  {
    for (i=0; i < pADecCompPrivate->sPortTypesParam[OMX_PortDomainAudio].nPorts; i++)
    {
      if(pADecCompPrivate->ports[i]){
        pADecCompPrivate->ports[i]->PortDestructor(pADecCompPrivate->ports[i]);
      }
    }

    TCC_free(pADecCompPrivate->ports);
    pADecCompPrivate->ports=NULL;
  }

  if( pADecCompPrivate->pAudioDLLModule != NULL)
  {
    dlclose(pADecCompPrivate->pAudioDLLModule);
    pADecCompPrivate->pAudioDLLModule = NULL;
  }

  LOGI("%s", __func__);
  omx_base_filter_Destructor(pOpenmaxStandComp);

  return OMX_ErrorNone;
}

/** this function sets the parameter values regarding audio format & index */
OMX_ERRORTYPE omx_audiodec_component_SetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_IN  OMX_PTR ComponentParameterStructure)
{

  OMX_ERRORTYPE eError = OMX_ErrorNone;
  OMX_AUDIO_PARAM_PORTFORMATTYPE *pAudioPortFormat;
  omx_base_audio_PortType *port;
  OMX_U32 portIndex;

  omx_audiodec_component_PrivateType* pADecCompPrivate = ((OMX_COMPONENTTYPE *)(hComponent))->pComponentPrivate;

  LOGD(" %s parameter 0x%x", __func__, nParamIndex);

  switch(nParamIndex)
  {
    case OMX_IndexParamAudioPortFormat:
    {
      pAudioPortFormat = (OMX_AUDIO_PARAM_PORTFORMATTYPE*)ComponentParameterStructure;
      portIndex = pAudioPortFormat->nPortIndex;
      /*Check Structure Header and verify component state*/
      eError = omx_base_component_ParameterSanityCheck(hComponent, portIndex, pAudioPortFormat, sizeof(OMX_AUDIO_PARAM_PORTFORMATTYPE));
      if(eError!=OMX_ErrorNone)
      {
        LOGE("In %s Parameter Check Error=%lx",__func__,eError);
        break;
      }
      if (portIndex <= 1)
      {
        port = (omx_base_audio_PortType *) pADecCompPrivate->ports[portIndex];
        (void)memcpy(&port->sAudioParam, pAudioPortFormat, sizeof(OMX_AUDIO_PARAM_PORTFORMATTYPE));
      }
      else
      {
        eError = OMX_ErrorBadPortIndex;
      }
    }
    break;

    case OMX_IndexVendorAudioExtraData:
    {
      OMX_VENDOR_EXTRADATATYPE* pExtradata;
      pExtradata = (OMX_VENDOR_EXTRADATATYPE*)ComponentParameterStructure;

      if (pExtradata->nPortIndex <= 1) {
        /** copy the extradata in the codec context private structure */
        //(void)memcpy(&omx_mp3dec_component_Private->audioinfo, pExtradata->pData, sizeof(rm_audio_info));
      } else {
        eError = OMX_ErrorBadPortIndex;
      }
    }
    break;

    case OMX_IndexVendorParamMediaInfo:
    {
      //OMX_AUDIO_CONFIG_INFOTYPE *info = (OMX_AUDIO_CONFIG_INFOTYPE*) ComponentParameterStructure;
    }
    break;

    default: /*Call the base component function*/
      eError = omx_base_component_SetParameter(hComponent, nParamIndex, ComponentParameterStructure);
    break;  
  }

  if(eError != OMX_ErrorNone){
    LOGE("Error %s :: nParamIndex = 0x%lx, error(0x%lx)", __func__, (OMX_U32)nParamIndex, (OMX_U32)eError);
  }

  return eError;
}

/** this function gets the parameters regarding audio formats and index */
/* coverity[HIS_metric_violation : FALSE] */
OMX_ERRORTYPE omx_audiodec_component_GetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_INOUT OMX_PTR ComponentParameterStructure)
{

  OMX_AUDIO_PARAM_PORTFORMATTYPE *pAudioPortFormat;
  omx_base_audio_PortType *port;
  OMX_ERRORTYPE eError = OMX_ErrorNone;

  omx_audiodec_component_PrivateType* pADecCompPrivate = ((OMX_COMPONENTTYPE *)(hComponent))->pComponentPrivate;

  LOGD(" %s parameter 0x%lx", __func__, (OMX_U32)nParamIndex);

  switch(nParamIndex)
  {
    case OMX_IndexParamAudioInit:
      if ((eError = checkHeader(ComponentParameterStructure, sizeof(OMX_PORT_PARAM_TYPE))) != OMX_ErrorNone)
      {
        break;
      }
      (void)memcpy(ComponentParameterStructure, &pADecCompPrivate->sPortTypesParam, sizeof(OMX_PORT_PARAM_TYPE));
      break;

    case OMX_IndexParamAudioPortFormat:
      pAudioPortFormat = (OMX_AUDIO_PARAM_PORTFORMATTYPE*)ComponentParameterStructure;
      if ((eError = checkHeader(ComponentParameterStructure, sizeof(OMX_AUDIO_PARAM_PORTFORMATTYPE))) != OMX_ErrorNone)
      {
        break;
      }
      if (pAudioPortFormat->nPortIndex <= 1)
      {
        port = (omx_base_audio_PortType *)pADecCompPrivate->ports[pAudioPortFormat->nPortIndex];
        (void)memcpy(pAudioPortFormat, &port->sAudioParam, sizeof(OMX_AUDIO_PARAM_PORTFORMATTYPE));
      }
      else
      {
        eError = OMX_ErrorBadPortIndex;
      }
      break;

    default: /*Call the base component function*/
      eError = omx_base_component_GetParameter(hComponent, nParamIndex, ComponentParameterStructure);
  }
  return eError;
}


OMX_ERRORTYPE SetMultiChannelOrder(OMX_COMPONENTTYPE *openmaxStandComp, OMX_U32 nChannels )
{
  omx_audiodec_component_PrivateType* pADecCompPrivate = openmaxStandComp->pComponentPrivate;
  OMX_U32 i;

  switch (nChannels) {
  case 1:
    pADecCompPrivate->stOutPcmMode.eChannelMapping[0] = OMX_AUDIO_ChannelCF;
    break;
  case 2:
    pADecCompPrivate->stOutPcmMode.eChannelMapping[0] = OMX_AUDIO_ChannelLF;
    pADecCompPrivate->stOutPcmMode.eChannelMapping[1] = OMX_AUDIO_ChannelRF;
    break;
  case 3:
    pADecCompPrivate->stOutPcmMode.eChannelMapping[0] = OMX_AUDIO_ChannelLF;
    pADecCompPrivate->stOutPcmMode.eChannelMapping[1] = OMX_AUDIO_ChannelRF;
    pADecCompPrivate->stOutPcmMode.eChannelMapping[2] = OMX_AUDIO_ChannelCF;
    break;
  case 4:
    pADecCompPrivate->stOutPcmMode.eChannelMapping[0] = OMX_AUDIO_ChannelLF;
    pADecCompPrivate->stOutPcmMode.eChannelMapping[1] = OMX_AUDIO_ChannelRF;
    pADecCompPrivate->stOutPcmMode.eChannelMapping[2] = OMX_AUDIO_ChannelCF;
    pADecCompPrivate->stOutPcmMode.eChannelMapping[3] = OMX_AUDIO_ChannelLFE;
    break;
  case 5:
    pADecCompPrivate->stOutPcmMode.eChannelMapping[0] = OMX_AUDIO_ChannelLF;
    pADecCompPrivate->stOutPcmMode.eChannelMapping[1] = OMX_AUDIO_ChannelRF;
    pADecCompPrivate->stOutPcmMode.eChannelMapping[2] = OMX_AUDIO_ChannelCF;
    pADecCompPrivate->stOutPcmMode.eChannelMapping[3] = OMX_AUDIO_ChannelLR;
    pADecCompPrivate->stOutPcmMode.eChannelMapping[4] = OMX_AUDIO_ChannelRR;
    break;
  case 6:
    pADecCompPrivate->stOutPcmMode.eChannelMapping[0] = OMX_AUDIO_ChannelLF;
    pADecCompPrivate->stOutPcmMode.eChannelMapping[1] = OMX_AUDIO_ChannelRF;
    pADecCompPrivate->stOutPcmMode.eChannelMapping[2] = OMX_AUDIO_ChannelCF;
    pADecCompPrivate->stOutPcmMode.eChannelMapping[3] = OMX_AUDIO_ChannelLFE;
    pADecCompPrivate->stOutPcmMode.eChannelMapping[4] = OMX_AUDIO_ChannelLR;
    pADecCompPrivate->stOutPcmMode.eChannelMapping[5] = OMX_AUDIO_ChannelRR;
    break;
  case 7:
    pADecCompPrivate->stOutPcmMode.eChannelMapping[0] = OMX_AUDIO_ChannelLF;
    pADecCompPrivate->stOutPcmMode.eChannelMapping[1] = OMX_AUDIO_ChannelRF;
    pADecCompPrivate->stOutPcmMode.eChannelMapping[2] = OMX_AUDIO_ChannelCF;
    pADecCompPrivate->stOutPcmMode.eChannelMapping[3] = OMX_AUDIO_ChannelLR;
    pADecCompPrivate->stOutPcmMode.eChannelMapping[4] = OMX_AUDIO_ChannelRR;
    pADecCompPrivate->stOutPcmMode.eChannelMapping[5] = OMX_AUDIO_ChannelLS;
    pADecCompPrivate->stOutPcmMode.eChannelMapping[6] = OMX_AUDIO_ChannelRS;
    break;
  case 8:
    pADecCompPrivate->stOutPcmMode.eChannelMapping[0] = OMX_AUDIO_ChannelLF;
    pADecCompPrivate->stOutPcmMode.eChannelMapping[1] = OMX_AUDIO_ChannelRF;
    pADecCompPrivate->stOutPcmMode.eChannelMapping[2] = OMX_AUDIO_ChannelCF;
    pADecCompPrivate->stOutPcmMode.eChannelMapping[3] = OMX_AUDIO_ChannelLFE;
    pADecCompPrivate->stOutPcmMode.eChannelMapping[4] = OMX_AUDIO_ChannelLR;
    pADecCompPrivate->stOutPcmMode.eChannelMapping[5] = OMX_AUDIO_ChannelRR;
    pADecCompPrivate->stOutPcmMode.eChannelMapping[6] = OMX_AUDIO_ChannelLS;
    pADecCompPrivate->stOutPcmMode.eChannelMapping[7] = OMX_AUDIO_ChannelRS;
    break;
  default:
    break;
  }

  for(i = 0; i < OMX_AUDIO_MAXCHANNELS; i++)
  {
    LOGD("ch %lu, type %ld :: ", i, (OMX_S32)pADecCompPrivate->stOutPcmMode.eChannelMapping[i]);
    switch(pADecCompPrivate->stOutPcmMode.eChannelMapping[i])
    {
    case OMX_AUDIO_ChannelNone: /**< Unused or empty */
      LOGD("%lu th Channel is Unused!", i);
      break;
    case OMX_AUDIO_ChannelLF: /**< Left front */
      LOGD("%lu th Channel is Left front!", i);
      pADecCompPrivate->stPcmInfo.m_iNchannelOrder[CH_LEFT_FRONT] = i+1;
      break;
    case OMX_AUDIO_ChannelRF: /**< Right front */
      LOGD("%lu th Channel is Right front!", i);
      pADecCompPrivate->stPcmInfo.m_iNchannelOrder[CH_RIGHT_FRONT] = i+1;
      break;
    case OMX_AUDIO_ChannelCF: /**< Center front */
      LOGD("%lu th Channel is Center front!", i);
      pADecCompPrivate->stPcmInfo.m_iNchannelOrder[CH_CENTER] = i+1;
      break;
    case OMX_AUDIO_ChannelLS: /**< Left surround */
      LOGD("%lu th Channel is Left surround!", i);
      pADecCompPrivate->stPcmInfo.m_iNchannelOrder[CH_LEFT_SIDE] = i+1;
      break;
    case OMX_AUDIO_ChannelRS: /**< Right surround */
      LOGD("%lu th Channel is Right surround!", i);
      pADecCompPrivate->stPcmInfo.m_iNchannelOrder[CH_RIGHT_SIDE] = i+1;
      break;
    case OMX_AUDIO_ChannelLFE:  /**< Low frequency effects */
      LOGD("%lu th Channel is Low frequency effects!", i);
      pADecCompPrivate->stPcmInfo.m_iNchannelOrder[CH_LFE] = i+1;
      break;
    case OMX_AUDIO_ChannelCS: /**< Back surround */
      LOGD("%lu th Channel is Back surround -> not support!", i);
      break;
    case OMX_AUDIO_ChannelLR: /**< Left rear. */
      LOGD("%lu th Channel is Left rear!", i);
      pADecCompPrivate->stPcmInfo.m_iNchannelOrder[CH_LEFT_REAR] = i+1;
      break;
    case OMX_AUDIO_ChannelRR: /**< Right rear. */
      LOGD("%lu th Channel is Right rear!", i);
      pADecCompPrivate->stPcmInfo.m_iNchannelOrder[CH_RIGHT_REAR] = i+1;
      break;
    default:
      //LOGD("not support!!", i);
      break;
    }
  }

  return OMX_ErrorNone;
}

static OMX_ERRORTYPE OpenAudioDecoder(OMX_COMPONENTTYPE *pOpenmaxStandComp, OMX_BUFFERHEADERTYPE* pInputBuffer, OMX_BUFFERHEADERTYPE* pOutputBuffer)
{
  omx_audiodec_component_PrivateType* pADecCompPrivate = pOpenmaxStandComp->pComponentPrivate;
  OMX_ERRORTYPE iError = OMX_ErrorNone;

  /* setting callback functions */
  pADecCompPrivate->stADecInit.m_pfMalloc = (void* (*) ( size_t ))malloc;
  pADecCompPrivate->stADecInit.m_pfRealloc = (void* (*) ( void*, size_t ))realloc;
  pADecCompPrivate->stADecInit.m_pfFree = (void  (*) ( void* ))free;
  pADecCompPrivate->stADecInit.m_pfMemcpy = (void* (*) ( void*, const void*, size_t ))memcpy;
  pADecCompPrivate->stADecInit.m_pfMemmove = (void* (*) ( void*, const void*, size_t ))memmove;
  pADecCompPrivate->stADecInit.m_pfMemset = (void  (*) ( void*, int, size_t ))memset;

  /* set pcm-info struct */
  pADecCompPrivate->stPcmInfo.m_uiBitsPerSample = pADecCompPrivate->stOutPcmMode.nBitPerSample;
  pADecCompPrivate->stPcmInfo.m_ePcmInterLeaved = (TCAS_U32)pADecCompPrivate->stOutPcmMode.bInterleaved;
  pADecCompPrivate->stPcmInfo.m_iNchannelOrder[CH_LEFT_FRONT] = 1;  //first channel
  pADecCompPrivate->stPcmInfo.m_iNchannelOrder[CH_RIGHT_FRONT] = 2; //second channel

  if (pADecCompPrivate->stOutPcmMode.nChannels > 2)
  {
    LOGI("SetMultiChannelOrder. ch: %lu", pADecCompPrivate->stOutPcmMode.nChannels);
    (void)SetMultiChannelOrder (pOpenmaxStandComp, pADecCompPrivate->stOutPcmMode.nChannels);
    pADecCompPrivate->stADecInit.m_iDownMixMode = 0;
  }

  /* set stream-info struct */
  if (!pADecCompPrivate->stStreamInfo.m_eSampleRate)
  {
    pADecCompPrivate->stStreamInfo.m_eSampleRate = (TCAS_U32)pADecCompPrivate->stOutPcmMode.nSamplingRate;
  }
  if (!pADecCompPrivate->stStreamInfo.m_uiNumberOfChannel)
  {
    pADecCompPrivate->stStreamInfo.m_uiNumberOfChannel = pADecCompPrivate->stOutPcmMode.nChannels;
  }

  if(pInputBuffer->nFlags & OMX_BUFFERFLAG_CODECCONFIG) {
    pADecCompPrivate->stADecInit.m_pucExtraData = pInputBuffer->pBuffer;
    pADecCompPrivate->stADecInit.m_iExtraDataLen = pInputBuffer->nFilledLen;
  }

  pADecCompPrivate->stADecInit.m_psAudiodecOutput = &pADecCompPrivate->stPcmInfo;
  pADecCompPrivate->stADecInit.m_psAudiodecInput = &pADecCompPrivate->stStreamInfo;

  if (pADecCompPrivate->pfCodecInit(pADecCompPrivate, pInputBuffer) == 0)
  {
    OMX_U64 temp_time;

    LOGI("Audio DEC initialized.");
    pADecCompPrivate->isDecoderReady = OMX_TRUE;
    if(pInputBuffer->nFlags & OMX_BUFFERFLAG_CODECCONFIG)
    {
      pInputBuffer->nFilledLen = 0;
    }

    if((pADecCompPrivate->stOutPcmMode.nSamplingRate != pADecCompPrivate->stPcmInfo.m_eSampleRate) ||
       (pADecCompPrivate->stOutPcmMode.nChannels!= pADecCompPrivate->stPcmInfo.m_uiNumberOfChannel))
    {
      if (pADecCompPrivate->stPcmInfo.m_eSampleRate && pADecCompPrivate->stPcmInfo.m_uiNumberOfChannel)
      {
        LOGI(" Sending Port Settings Change Event");
        LOGI(" OutPcmMode[%lu:%lu] PcmInfo[%u:%u]",
            pADecCompPrivate->stOutPcmMode.nSamplingRate,pADecCompPrivate->stOutPcmMode.nChannels,
            pADecCompPrivate->stPcmInfo.m_eSampleRate,pADecCompPrivate->stPcmInfo.m_uiNumberOfChannel);
        pADecCompPrivate->stOutPcmMode.nSamplingRate = pADecCompPrivate->stPcmInfo.m_eSampleRate;
        pADecCompPrivate->stOutPcmMode.nChannels = pADecCompPrivate->stPcmInfo.m_uiNumberOfChannel;

        pADecCompPrivate->pfSetInternalParam(pADecCompPrivate);

        /*Send Port Settings changed call back*/
        (*(pADecCompPrivate->callbacks->EventHandler))
          (pOpenmaxStandComp,
          pADecCompPrivate->callbackData,
          OMX_EventPortSettingsChanged, /* The command was completed */
          0,
          1, /* This is the output port index */
          NULL);
      }
    }

    /* set input buffer control */
    pADecCompPrivate->iStreamBufferSize = 0;
    pADecCompPrivate->pucStreamBuffer = NULL;
    pADecCompPrivate->iEmptyBufferDone = OMX_FALSE;

    /* output buffer split */
    if (pADecCompPrivate->isPcmSplitEnabled)
    {
      pADecCompPrivate->iGuardSamples = pADecCompPrivate->stOutPcmMode.nSamplingRate >> 3; // number of guard samples are corresponding to 125ms
      temp_time = (OMX_U64)OUTPUT_SPLIT_TIME_LIMIT * pADecCompPrivate->stOutPcmMode.nSamplingRate - (pADecCompPrivate->stOutPcmMode.nSamplingRate >> 1);
      pADecCompPrivate->iSplitLength = temp_time / 1000000;
      LOGD("num of guard samples %lld, split length %lu", pADecCompPrivate->iGuardSamples, pADecCompPrivate->iSplitLength);
    }
    pADecCompPrivate->iDebugDuration = pADecCompPrivate->stOutPcmMode.nSamplingRate;
  }
  else
  {
    LOGE("Audio DEC init error!");
    iError = OMX_ErrorUndefined;
    pADecCompPrivate->pfCodecClose(pADecCompPrivate);
  }

  return iError;
}

static OMX_S32 MergeStreamBuffer(omx_audiodec_component_PrivateType* pADecCompPrivate, OMX_BUFFERHEADERTYPE* pInputBuffer)
{
  OMX_S32 err = 0;
  if (pADecCompPrivate->stStreamInfo.m_iStreamLength < 0) {
    pADecCompPrivate->stStreamInfo.m_iStreamLength = 0;
  }

  if((OMX_S32)(pADecCompPrivate->stStreamInfo.m_iStreamLength + pInputBuffer->nFilledLen) > pADecCompPrivate->iStreamBufferSize)
  {
    LOGI("[%d] realloc buffer %ld -> %ld!", __LINE__,pADecCompPrivate->iStreamBufferSize, pADecCompPrivate->stStreamInfo.m_iStreamLength + pInputBuffer->nFilledLen);
    pADecCompPrivate->iStreamBufferSize = pADecCompPrivate->stStreamInfo.m_iStreamLength + pInputBuffer->nFilledLen;
    pADecCompPrivate->pucStreamBuffer = realloc(pADecCompPrivate->pucStreamBuffer, pADecCompPrivate->iStreamBufferSize);
    if( pADecCompPrivate->pucStreamBuffer == NULL )
    {
      LOGE("pucStreamBuffer re-allocation-0 fail");
      err = -1;
    }
  }

  if (err == 0) {
    (void)memcpy(pADecCompPrivate->pucStreamBuffer + pADecCompPrivate->stStreamInfo.m_iStreamLength, pInputBuffer->pBuffer + pInputBuffer->nOffset, pInputBuffer->nFilledLen);
    pADecCompPrivate->stStreamInfo.m_pcStream = pADecCompPrivate->pucStreamBuffer;
    pADecCompPrivate->stStreamInfo.m_iStreamLength += pInputBuffer->nFilledLen;
  }
  return err;
}

static OMX_S32 KeepRemainingStream(omx_audiodec_component_PrivateType* pADecCompPrivate, OMX_BUFFERHEADERTYPE* pInputBuffer)
{
  OMX_S32 ret = 0;
  if(pADecCompPrivate->pucStreamBuffer == NULL)
  {
    if(pADecCompPrivate->iStreamBufferSize == 0)
    {
      pADecCompPrivate->iStreamBufferSize = pADecCompPrivate->stStreamInfo.m_iStreamLength + (pInputBuffer->nFilledLen*2);
    }
    pADecCompPrivate->pucStreamBuffer = malloc(pADecCompPrivate->iStreamBufferSize);
    if( pADecCompPrivate->pucStreamBuffer == NULL )
    {
      LOGE("pucStreamBuffer allocation fail");
      ret = (-1);
    }
  }
  else {
    if((OMX_S32)(pADecCompPrivate->stStreamInfo.m_iStreamLength) > pADecCompPrivate->iStreamBufferSize)
    {
      LOGI("[%d] realloc buffer %ld -> %d!\n", __LINE__,pADecCompPrivate->iStreamBufferSize, pADecCompPrivate->stStreamInfo.m_iStreamLength*2);
      pADecCompPrivate->iStreamBufferSize = pADecCompPrivate->stStreamInfo.m_iStreamLength*2;
      pADecCompPrivate->pucStreamBuffer = realloc(pADecCompPrivate->pucStreamBuffer, pADecCompPrivate->iStreamBufferSize);
      if( pADecCompPrivate->pucStreamBuffer == NULL )
      {
        LOGE("pucStreamBuffer re-allocation-1 fail\n");
        ret = (-1);
      }
    }
  }

  if (ret == 0) {
    (void)memcpy(pADecCompPrivate->pucStreamBuffer, pADecCompPrivate->stStreamInfo.m_pcStream, pADecCompPrivate->stStreamInfo.m_iStreamLength);
  }
  return ret;
}

static OMX_S32 SetStreamBuff(omx_audiodec_component_PrivateType* pADecCompPrivate, OMX_BUFFERHEADERTYPE* pInputBuffer)
{
  OMX_S32 ret = 0;
  if(pADecCompPrivate->isNewInputBuffer == OMX_TRUE) // New pInputBuffer has arrived
  {
    pADecCompPrivate->uiInputBufferCount++;
    pADecCompPrivate->iEmptyBufferDone = OMX_FALSE;
    pADecCompPrivate->isNewInputBuffer = OMX_FALSE;
    if(pADecCompPrivate->stStreamInfo.m_iStreamLength > 0) // Previous pInputBuffer is still being used.
    {
      LOGV("New buffer!, merge with previous buffer, prev %d, curr %lu", pADecCompPrivate->stStreamInfo.m_iStreamLength, pInputBuffer->nFilledLen);
      if(MergeStreamBuffer(pADecCompPrivate, pInputBuffer) < 0)
      {
        ret = (-1);
      }
    }
    else
    {
      LOGV("New buffer, offset %lu, length %lu", pInputBuffer->nOffset, pInputBuffer->nFilledLen);
      pADecCompPrivate->stStreamInfo.m_pcStream = pInputBuffer->pBuffer + pInputBuffer->nOffset;
      pADecCompPrivate->stStreamInfo.m_iStreamLength = pInputBuffer->nFilledLen;
    }
  }
  else
  {
    LOGV("Now using previous pInputBuffer, length %d", pADecCompPrivate->stStreamInfo.m_iStreamLength);
  }

  if (ret == 0) {
    LOGV("START: stream length %d, current pInputBuffer offset %lu ", pADecCompPrivate->stStreamInfo.m_iStreamLength, pInputBuffer->nOffset);
  }
  return ret;
}

static OMX_S32 UpdateStreamBuff(omx_audiodec_component_PrivateType* pADecCompPrivate, OMX_BUFFERHEADERTYPE* pInputBuffer, OMX_S32 iRet)
{
  OMX_S32 ret = 0;
  LOGV("END: bytes left %d, ret %ld, min_size %ld", pADecCompPrivate->stStreamInfo.m_iStreamLength, iRet, pADecCompPrivate->iMinStreamSize);

  // in order to prevent recognizing this pInputBuffer as new buffer at the next time processing,
  // if pInputBuffer->nOffset is 0, the pInputBuffer is regarded as new pInputBuffer.
  //pInputBuffer->nOffset++;

  if(((int)iRet == TCAS_ERROR_MORE_DATA) || (pADecCompPrivate->stStreamInfo.m_iStreamLength <= 0) || (pADecCompPrivate->stStreamInfo.m_iStreamLength < pADecCompPrivate->iMinStreamSize))
  {
    LOGV("current pInputBuffer is empty, need new input buffer!");
    pADecCompPrivate->iEmptyBufferDone = OMX_TRUE;
    pInputBuffer->nFilledLen = 0; // return this buffer

    if(pADecCompPrivate->stStreamInfo.m_iStreamLength > 0)
    {
      LOGV("keep remaining stream, length %d", pADecCompPrivate->stStreamInfo.m_iStreamLength);
      if(KeepRemainingStream(pADecCompPrivate, pInputBuffer) < 0)
      {
        ret = (-1);
      }
    }
  }
  else
  {
    LOGV("input buffer is not empty");
  }

  return ret;
}

// This is not necessary in most cases.
static OMX_S32 CopyRestSample(omx_audiodec_component_PrivateType* pADecCompPrivate, OMX_U8 *pRestSample)
{
  OMX_S32 ret = 0;
  int ByteofPCMSample = (pADecCompPrivate->stOutPcmMode.nBitPerSample <= 16) ? 2 : 4;

  LOGV("Split size %lu, buffer size %lu", pADecCompPrivate->iRestSamples, pADecCompPrivate->uiSplitBufferSampleCapacity);
  if((pADecCompPrivate->pRest == NULL) || (pADecCompPrivate->iRestSamples > pADecCompPrivate->uiSplitBufferSampleCapacity))
  {
    LOGV("Split-Buffer allocation, size %lu", pADecCompPrivate->iRestSamples);
    // For the saving of memory.
    if(pADecCompPrivate->pRest)
    {
      free(pADecCompPrivate->pRest);
    }

    pADecCompPrivate->uiSplitBufferSampleCapacity = pADecCompPrivate->iRestSamples * 2;
    pADecCompPrivate->pRest = (OMX_U8*)malloc(pADecCompPrivate->uiSplitBufferSampleCapacity * pADecCompPrivate->stOutPcmMode.nChannels * ByteofPCMSample );
    if(pADecCompPrivate->pRest == NULL)
    {
      LOGE("Split-Buffer allocation fail");
      ret = (-1);
    }
  }

  if (ret == 0)
  {
    (void)memcpy(pADecCompPrivate->pRest, pRestSample,
                 pADecCompPrivate->iRestSamples * pADecCompPrivate->stOutPcmMode.nChannels * ByteofPCMSample);
  }
  return ret;
}

static void CalcTimeStamp
(omx_audiodec_component_PrivateType* pADecCompPrivate, OMX_BUFFERHEADERTYPE* pOutputBuffer, OMX_S32 iCurrSamples, OMX_TICKS dmxTs)
{
  pADecCompPrivate->iDuration = (pADecCompPrivate->uiNumSamplesOutput * 1000000ll) / pADecCompPrivate->stOutPcmMode.nSamplingRate;
  pOutputBuffer->nTimeStamp = pADecCompPrivate->iPrevOriginalTS + pADecCompPrivate->iDuration;

  if( dmxTs != -1 )
  {
    // if Difference between demuxer and decoder is more than 10 seconds,
    // replace the output timestamp with the value of the demuxer timestamp.
    OMX_TICKS diffTs = pOutputBuffer->nTimeStamp > dmxTs ? pOutputBuffer->nTimeStamp - dmxTs : dmxTs - pOutputBuffer->nTimeStamp;
    OMX_TICKS max_diff = (pADecCompPrivate->uiCodecType == AUDIO_ID_WMA) ? MAX_DIFF_TIMESTAMP * 10 : MAX_DIFF_TIMESTAMP;
    if ( diffTs > max_diff ) {
      LOGI("CalcTimeStamp iCurrSamples = %ld, nTimeStamp (%lld) - InTime (%lld) = diff (%lld)", iCurrSamples, pOutputBuffer->nTimeStamp, dmxTs,diffTs);
      pADecCompPrivate->iPrevOriginalTS = pOutputBuffer->nTimeStamp = dmxTs;
      pADecCompPrivate->uiNumSamplesOutput = 0;
    }
  }

  pADecCompPrivate->uiNumSamplesOutput += iCurrSamples;
  pADecCompPrivate->iPrevTS = pOutputBuffer->nTimeStamp;
}


OMX_S32 InitDecoder(omx_audiodec_component_PrivateType* pADecCompPrivate)
{
  OMX_S32 nReturn = 0;
  nReturn = pADecCompPrivate->pfCdkAudioDec(AUDIO_INIT, &pADecCompPrivate->hDecoderHandle, &pADecCompPrivate->stADecInit, NULL);
  pADecCompPrivate->pfSetInternalParam(pADecCompPrivate);
  LOGD("%s result %ld",__func__, nReturn);
  return nReturn;
}

OMX_S32 DecodeFrame(omx_audiodec_component_PrivateType* pADecCompPrivate, OMX_BUFFERHEADERTYPE* pInputBuffer, OMX_BUFFERHEADERTYPE* pOutputBuffer)
{
  OMX_S32 nReturn = 0;
  OMX_U8* pOutBufBase;
  int iOutFrameSize;
  int ByteofPCMSample = (pADecCompPrivate->stOutPcmMode.nBitPerSample <= 16) ? 2 : 4;

  iOutFrameSize = 0;
  pADecCompPrivate->iDecodedSamplePerCh = 0;

  LOGD("start: input length %d", pADecCompPrivate->stStreamInfo.m_iStreamLength);

  pOutBufBase = pOutputBuffer->pBuffer;

  while((pADecCompPrivate->stStreamInfo.m_iStreamLength > pADecCompPrivate->iMinStreamSize ) ||
       ((pADecCompPrivate->stStreamInfo.m_iStreamLength > 0) && (pADecCompPrivate->isEndOfFile == 1)))
  {
    pADecCompPrivate->stPcmInfo.m_pvChannel[0]  = (void *)(pOutBufBase + iOutFrameSize);

    if(iOutFrameSize + (pADecCompPrivate->stPcmInfo.m_uiNumberOfChannel * pADecCompPrivate->stPcmInfo.m_uiSamplesPerChannel * ByteofPCMSample) > pOutputBuffer->nAllocLen)
    {
      LOGE("too many sample!");
      break;
    }

    // Using the opus decoder, pass the header packet
    if(pADecCompPrivate->uiCodecType == AUDIO_ID_OPUS)
    {
      if(!memcmp(pInputBuffer->pBuffer, "OpusHead", 8))
      {
        LOGD("In %s This packet is OpusHeader.",__func__);
        pADecCompPrivate->stStreamInfo.m_iStreamLength = 0;
        return nReturn;
      }
      if(!memcmp(pInputBuffer->pBuffer, "OpusTags", 8))
      {
        LOGD("In %s This packet is OpusTags.",__func__);
        pADecCompPrivate->stStreamInfo.m_iStreamLength = 0;
        return nReturn;
      }
    }

    nReturn = pADecCompPrivate->pfCdkAudioDec(AUDIO_DECODE, &pADecCompPrivate->hDecoderHandle, &pADecCompPrivate->stStreamInfo, &pADecCompPrivate->stPcmInfo);

    if((nReturn == TCAS_SUCCESS) || ((int)nReturn == TCAS_ERROR_CONCEALMENT_APPLIED))
    {
      iOutFrameSize += pADecCompPrivate->stPcmInfo.m_uiNumberOfChannel * pADecCompPrivate->stPcmInfo.m_uiSamplesPerChannel * ByteofPCMSample;
      pADecCompPrivate->iDecodedSamplePerCh += pADecCompPrivate->stPcmInfo.m_uiSamplesPerChannel;
      break;
    }
    else
    {
      if((int)nReturn != TCAS_ERROR_MORE_DATA)
      LOGE("In %s Decoding Error [%lu], StreamLength [%d]", __func__, nReturn, pADecCompPrivate->stStreamInfo.m_iStreamLength);
      if((int)nReturn != TCAS_ERROR_SKIP_FRAME)
      {
        break;
      }
    }
  }

  return nReturn;
}

OMX_S32 FlushCodec(omx_audiodec_component_PrivateType* pADecCompPrivate, OMX_BUFFERHEADERTYPE* pInputBuffer)
{
  LOGD("%s",__func__);
  if (pADecCompPrivate->hDecoderHandle != NULL){
    pADecCompPrivate->pfCdkAudioDec(AUDIO_FLUSH, &pADecCompPrivate->hDecoderHandle, NULL, NULL);
  }
  return 0;
}

OMX_S32 CloseCodec(omx_audiodec_component_PrivateType* pADecCompPrivate)
{
  LOGI("%s",__func__);
  if (pADecCompPrivate->hDecoderHandle != NULL) {
    pADecCompPrivate->pfCdkAudioDec(AUDIO_CLOSE, &pADecCompPrivate->hDecoderHandle, NULL, NULL);
    pADecCompPrivate->hDecoderHandle = 0;
  }
  return 0;
}

void omx_audiodec_component_BufferMgmtCallback(OMX_COMPONENTTYPE *pOpenmaxStandComp, OMX_BUFFERHEADERTYPE* pInputBuffer, OMX_BUFFERHEADERTYPE* pOutputBuffer)
{
  omx_audiodec_component_PrivateType* pADecCompPrivate = pOpenmaxStandComp->pComponentPrivate;

  OMX_S32 nRetValue = -1;
  pOutputBuffer->nFilledLen = 0;
  pOutputBuffer->nOffset = 0;
  pADecCompPrivate->pvCodecSpecific = NULL;

  LOGV("BufferMgmtCallback IN inLen = %lu, Flags = 0x%lx, Timestamp = %lld", pInputBuffer->nFilledLen, pInputBuffer->nFlags, pInputBuffer->nTimeStamp);

  if (pOutputBuffer->pBuffer == NULL){
    return;
  }

  if(pADecCompPrivate->isDecoderReady == OMX_FALSE)
  {
    if (OpenAudioDecoder(pOpenmaxStandComp, pInputBuffer, pOutputBuffer) != OMX_ErrorNone)
    {
      LOGE(" Audio Decoder not Initialized!!");
      (*(pADecCompPrivate->callbacks->EventHandler))(pOpenmaxStandComp, pADecCompPrivate->callbackData,
        OMX_EventError, OMX_ErrorNotReady,
        0, NULL);
      pInputBuffer->nFilledLen = 0; // to skip all audio data
      return;
    }

    if (pInputBuffer->nFilledLen == 0)
    {
      return;
    }
  }

  if (pADecCompPrivate->isPcmSplitEnabled == OMX_TRUE)
  {
    // remaining data exist
    // but if pInputBuffer has SYNCFRAME flag, it means that the remaining data is useless because it also means seeking was done.
    if((pADecCompPrivate->iRestSamples > 0) &&
    	!((pADecCompPrivate->isNewInputBuffer == OMX_TRUE) && (pInputBuffer->nFlags & OMX_BUFFERFLAG_STARTTIME)))  /*OMX_BUFFERFLAG_SYNCFRAME*/
    {
      OMX_U32 iCurrentSample = (pADecCompPrivate->iRestSamples >= pADecCompPrivate->iSplitLength)
                           ? pADecCompPrivate->iSplitLength : pADecCompPrivate->iRestSamples;
      int ByteofPCMSample = (pADecCompPrivate->stOutPcmMode.nBitPerSample <= 16) ? 2 : 4;

      pADecCompPrivate->iRestSamples -= iCurrentSample;
      pOutputBuffer->nFilledLen = iCurrentSample * pADecCompPrivate->stOutPcmMode.nChannels * ByteofPCMSample;
      (void)memcpy(pOutputBuffer->pBuffer, pADecCompPrivate->pRest+pADecCompPrivate->iSplitPosition, pOutputBuffer->nFilledLen);
      pADecCompPrivate->iSplitPosition += pOutputBuffer->nFilledLen;

      if(pADecCompPrivate->iRestSamples == 0 && pADecCompPrivate->iEmptyBufferDone == OMX_TRUE)
      {
        pInputBuffer->nFilledLen = 0;
      }

      CalcTimeStamp(pADecCompPrivate, pOutputBuffer, iCurrentSample, -1);

      LOGV("Consume remaining data, nTimeStamp = %lld, output size = %lu", pOutputBuffer->nTimeStamp, pOutputBuffer->nFilledLen);
      return;
    }
  }

  // if previous decoding failed, silence should be inserted
  if (pADecCompPrivate->isSilenceInsertionEnable == OMX_TRUE)
  {
    if (pADecCompPrivate->isPrevDecFail == OMX_TRUE)
    {
      OMX_TICKS time_diff = pOutputBuffer->nTimeStamp - pADecCompPrivate->iPrevTS;

      if((time_diff > 0) && (time_diff < OUTPUT_SILENT_TIME_LIMIT))
      {
        OMX_TICKS samples = (pOutputBuffer->nTimeStamp - pADecCompPrivate->iPrevTS) * pADecCompPrivate->stOutPcmMode.nSamplingRate / 1000000;
        int ByteofPCMSample = (pADecCompPrivate->stOutPcmMode.nBitPerSample <= 16U) ? 2 : 4;

        pOutputBuffer->nFilledLen = pADecCompPrivate->stOutPcmMode.nChannels * samples * ByteofPCMSample;

        if(pOutputBuffer->nFilledLen > pOutputBuffer->nAllocLen){
          pOutputBuffer->nFilledLen = pOutputBuffer->nAllocLen;
        }

        memset(pOutputBuffer->pBuffer, 0, pOutputBuffer->nFilledLen);
        pOutputBuffer->nTimeStamp = pADecCompPrivate->iPrevTS;
        pADecCompPrivate->isPrevDecFail = OMX_FALSE;

        return;
      }
    }
  }

  if(pADecCompPrivate->isNewInputBuffer == OMX_TRUE && pInputBuffer->nFlags & OMX_BUFFERFLAG_STARTTIME)   //OMX_BUFFERFLAG_SYNCFRAME
  {
    pADecCompPrivate->iStartTS = pInputBuffer->nTimeStamp;
    pADecCompPrivate->iMuteSamples = pADecCompPrivate->iGuardSamples;
    pADecCompPrivate->iNumOfSeek++;

    pADecCompPrivate->iPrevOriginalTS = pInputBuffer->nTimeStamp;
    pADecCompPrivate->uiNumSamplesOutput = 0;

    // eliminate remaining pcm samples
    pADecCompPrivate->iRestSamples = 0;

    // eliminate remaining stream data
    pADecCompPrivate->stStreamInfo.m_iStreamLength = 0;
    pADecCompPrivate->pfCodecFlush(pADecCompPrivate, pInputBuffer);

    pADecCompPrivate->isPrevDecFail = OMX_FALSE;
  }

  if (pADecCompPrivate->iPrevOriginalTS == -1)
  {
    pADecCompPrivate->iPrevOriginalTS = pInputBuffer->nTimeStamp;
  }

  /* Decode the block */
  if (SetStreamBuff(pADecCompPrivate, pInputBuffer))
  {
    pInputBuffer->nFilledLen = 0;
    return;
  }

  if (pADecCompPrivate->stStreamInfo.m_iStreamLength > pADecCompPrivate->iMinStreamSize)
  {
    // decode one frame of audio data
    nRetValue = pADecCompPrivate->pfCodecDecode(pADecCompPrivate, pInputBuffer, pOutputBuffer);
  }
  else
  {
    nRetValue = TCAS_ERROR_MORE_DATA;
    pADecCompPrivate->iDecodedSamplePerCh = 0;
  }

  if (UpdateStreamBuff(pADecCompPrivate, pInputBuffer, nRetValue))
  {
    pInputBuffer->nFilledLen = 0;
    return;
  }

  pOutputBuffer->nFilledLen = pADecCompPrivate->stOutPcmMode.nChannels * pADecCompPrivate->iDecodedSamplePerCh * \
                                      ((pADecCompPrivate->stOutPcmMode.nBitPerSample <= 16) ? 2 : 4);

  if (pOutputBuffer->nFilledLen)
  {
    OMX_S32 iCurrentSamples = pADecCompPrivate->iDecodedSamplePerCh;

    if (pADecCompPrivate->isPcmSplitEnabled == OMX_TRUE)
    {
      if (pADecCompPrivate->iDecodedSamplePerCh > pADecCompPrivate->iSplitLength)
      {
        int ByteofPCMSample = (pADecCompPrivate->stOutPcmMode.nBitPerSample <= 16) ? 2 : 4;

        pADecCompPrivate->iRestSamples = pADecCompPrivate->iDecodedSamplePerCh - pADecCompPrivate->iSplitLength;
        pOutputBuffer->nFilledLen = pADecCompPrivate->stOutPcmMode.nChannels * pADecCompPrivate->iSplitLength * ByteofPCMSample;

        LOGV( "PCM Split %ld sample, Rest %lu", pADecCompPrivate->iDecodedSamplePerCh, pADecCompPrivate->iRestSamples);

        if(CopyRestSample(pADecCompPrivate, pOutputBuffer->pBuffer + pOutputBuffer->nFilledLen) == 0)
        {
          iCurrentSamples = pADecCompPrivate->iSplitLength;
          pADecCompPrivate->iSplitPosition = 0;
        }
        else
        {
          pADecCompPrivate->iRestSamples = 0;
          pOutputBuffer->nFilledLen = pADecCompPrivate->stOutPcmMode.nChannels * pADecCompPrivate->iDecodedSamplePerCh * ByteofPCMSample;
        }
      }
    }

    CalcTimeStamp(pADecCompPrivate, pOutputBuffer, iCurrentSamples, pInputBuffer->nTimeStamp);
    pOutputBuffer->nTickCount = (iCurrentSamples * 1000000ll) / pADecCompPrivate->stOutPcmMode.nSamplingRate; //duration

    if(pADecCompPrivate->isPrevDecFail == OMX_TRUE)
    {
      pADecCompPrivate->isPrevDecFail = OMX_FALSE;
    }

#ifndef HAVE_ANDROID_OS
    if((pADecCompPrivate->stOutPcmMode.nSamplingRate != pADecCompPrivate->stPcmInfo.m_eSampleRate) ||
       (pADecCompPrivate->stOutPcmMode.nChannels!= pADecCompPrivate->stPcmInfo.m_uiNumberOfChannel))
    {
      LOGI(" Sending Port Settings Change Event");

      pADecCompPrivate->stOutPcmMode.nSamplingRate = pADecCompPrivate->stPcmInfo.m_eSampleRate;
      pADecCompPrivate->stOutPcmMode.nChannels = pADecCompPrivate->stPcmInfo.m_uiNumberOfChannel;

      pADecCompPrivate->pfSetInternalParam(pADecCompPrivate);

      /*Send Port Settings changed call back*/
      (*(pADecCompPrivate->callbacks->EventHandler))
      (pOpenmaxStandComp,
      pADecCompPrivate->callbackData,
      OMX_EventPortSettingsChanged, /* The command was completed */
      0,
      1, /* This is the output port index */
      NULL);
    }
#endif
    LOGD("Audio DEC Success. nTimeStamp = %lld(intime %lld), output size = %lu", pOutputBuffer->nTimeStamp, pInputBuffer->nTimeStamp, pOutputBuffer->nFilledLen);
    pADecCompPrivate->iDebugDuration -= pADecCompPrivate->iDecodedSamplePerCh;
    if (pADecCompPrivate->iDebugDuration <= 0) {
      OMX_LOGD("[OUT] [PTS: %8d]\n",(OMX_S32)(pOutputBuffer->nTimeStamp/1000));
      pADecCompPrivate->iDebugDuration = pADecCompPrivate->stOutPcmMode.nSamplingRate;
    }
  }
  else
  {
    OMX_LOGD("[OUT] No decoded sample, ret: %ld\n", nRetValue);
    if(((int)nRetValue != TCAS_ERROR_MORE_DATA) && (nRetValue != TCAS_SUCCESS))
    {
      if(pADecCompPrivate->isNewInputBuffer == OMX_FALSE)
      {
        pADecCompPrivate->iPrevTS = pOutputBuffer->nTimeStamp; // no pcm samples were generated from the current pInputBuffer.
      }
      else
      {
        pADecCompPrivate->iDuration = (pADecCompPrivate->uiNumSamplesOutput * 1000000ll) / pADecCompPrivate->stOutPcmMode.nSamplingRate;
        pADecCompPrivate->iPrevTS = pADecCompPrivate->iPrevOriginalTS + pADecCompPrivate->iDuration; // time-stamp of the last sample generated.
      }
      LOGD( "audio dec error: %ld, TimeStamp of the last pcm-sample generated %lld", nRetValue , pADecCompPrivate->iPrevTS);
      pADecCompPrivate->isPrevDecFail = OMX_TRUE;

      pOutputBuffer->nFilledLen = 2048;
      memset(pOutputBuffer->pBuffer,0,pOutputBuffer->nFilledLen);
      if (pADecCompPrivate->iPrevOriginalTS != pInputBuffer->nTimeStamp)
      {
        pADecCompPrivate->iPrevOriginalTS = pInputBuffer->nTimeStamp;
        pADecCompPrivate->uiNumSamplesOutput = 2048;
        pADecCompPrivate->iDuration = (pADecCompPrivate->uiNumSamplesOutput * 1000000ll) / pADecCompPrivate->stOutPcmMode.nSamplingRate;
        pADecCompPrivate->iPrevTS = pADecCompPrivate->iPrevOriginalTS + pADecCompPrivate->iDuration; // time-stamp of the last sample generated.
      }
    //  (void)printf("[ :A][ERROR][OUTPTS: %8d] [SIZE: %8d]\n", (OMX_S32)(pOutputBuffer->nTimeStamp/1000), pOutputBuffer->nFilledLen);
    }
  }

  if (pADecCompPrivate->iRestSamples > 0)
  {
    // do not set nFilledLen to 0, in order to transfer all the split data to output at once
    pInputBuffer->nFlags = OMX_BUFFERFLAG_ENDOFFRAME;
    if(pADecCompPrivate->iEmptyBufferDone == OMX_TRUE)
    {
      pInputBuffer->nFilledLen = 1;
    }
  }
}
