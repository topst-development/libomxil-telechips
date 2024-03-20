/**
  @file omx_videoenc_component.c

  This component implements tcc video encoder.
  The tcc video encoder is based on the MPEG-4 SP Video encoder.
  The MPEG-4 SP Video Encoder is based on the FFmpeg software library.

  Copyright (C) 2007-2008 STMicroelectronics
  Copyright (C) 2007-2008 Nokia Corporation and/or its subsidiary(-ies)
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

  Date: 2009/04/6
  Author : ZzaU, MMT
*/

#define LOG_TAG "OMX_VIDEO_ENC"

#include <stdio.h>
#include <glib.h>
#include <glib/gprintf.h>

#include <omxcore.h>
#include <OMX_Types.h>
#include <OMX_Component.h>
#include <OMX_Core.h>
#include <OMX_Video.h>
#include <OMX_CoreExt.h>
#include <OMX_IndexExt.h>
#include "omx_comp_debug_levels.h"

#include <omx_base_video_port.h>
#include <omx_videoenc_component.h>
#include <OMX_Video.h>

#include <tcc_vpu_encode_interface.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
//#include <linux/fb.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <tcc_video_private.h>
#ifdef MOVE_HW_OPERATION
#include <mach/tcc_mem_ioctl.h>
#endif

#include <sys/poll.h>
#include <OMX_IndexExt.h>
//#include <MetadataBufferType.h>
#include <errno.h>

#define COPY_DEVICE "/dev/wmixer1"

/////////////////////////////////////////////////////////////////////////////////////////
// Macro conditional compilation features
//
#define FOR_V2IP

//#define IN_BUFFER_REAL_SIZE
//#define USE_VPU_OUTPUT_BUFFER

/* test only */
//#define CHANGE_BITRATE_TEST  //to change bitrate.
//#define CHANGE_QP_TEST  //to change QP.
//#define CHANGE_FPS_TEST //to change framerate.
//#define REQUEST_INTRAR_EFRESH_TEST  //to request I-Frame.

/////////////////////////////////////////////////////////////////////////////////////////
// Macro functions
//
#define ALOGD(...)	//{tcc_printf(TC_CYAN "[%4d]", __LINE__);tcc_printf(__VA_ARGS__);tcc_printf(T_DEFAULT "\n");}
#define ALOGI(...)	{tcc_printf(TC_GREEN "[%4d]", __LINE__);tcc_printf(__VA_ARGS__);tcc_printf(T_DEFAULT "\n");}
#define ALOGW(...)	{tcc_printf(TC_MAGENTA "[%4d]", __LINE__);tcc_printf(__VA_ARGS__);tcc_printf(T_DEFAULT "\n");}
#define ALOGE(...)	{tcc_printf(TC_RED "[%4d]", __LINE__);tcc_printf(__VA_ARGS__);tcc_printf(T_DEFAULT "\n");}

static int DEBUG_ON  = 0;
#define DBUG_MSG(msg...)   { if (DEBUG_ON == 1) { ALOGD( ": " msg);/* sleep(1);*/}}

/////////////////////////////////////////////////////////////////////////////////////////
// Macro constants
//
/** Maximum Number of Video Component Instance*/
#define MAX_COMPONENT_VIDEOENC 4

/** The output decoded color format */
#define INPUT_ENCODED_COLOR_FMT OMX_COLOR_FormatYUV420Planar

/** In/Output buffer count */
#define NUM_IN_BUFFERS    4 // Input Buffers (camera buffer - 2)

#ifdef PASS_BUFFER_TO_UPPER_LAYER
#define NUM_OUT_BUFFERS   VIDEO_ENC_BUFFER_COUNT // Output Buffers
#else
#define NUM_OUT_BUFFERS   10 // Output Buffers
#endif

#define IN_META_MIN_BUFFER_SIZE 8//VideoNativeMetadata: 12, VideoGrallocMetadata and Others: 8

#define OUT_BUFFER_SIZE   (1 * 1024 * 1024) // (LARGE_STREAM_BUF_SIZE)


/////////////////////////////////////////////////////////////////////////////////////////
/* static global variables */
static OMX_U32 total_count = 0;

/** Counter of Video Component Instance*/
static OMX_U32 gVEncInstanceCount = 0;

/* OMX private abbreviation */
typedef omx_videoenc_component_PrivateType  videoenc_private_t;
#if 0
/* From media_plugin_headers */
struct VideoNativeHandleMetadata {
	MetadataBufferType eType;               // must be kMetadataBufferTypeNativeHandleSource
	native_handle_t *pHandle;
};
#endif
/////////////////////////////////////////////////////////////////////////////////////////
// Codec profile & level
//
typedef struct VIDEO_PROFILE_LEVEL {
    OMX_S32  nProfile;
    OMX_S32  nLevel;
} VIDEO_PROFILE_LEVEL_TYPE;

/* H.263 Supported Levels & profiles */
VIDEO_PROFILE_LEVEL_TYPE SupportedH263ProfileLevels[] = {
    {OMX_VIDEO_H263ProfileBaseline, OMX_VIDEO_H263Level10},
    {OMX_VIDEO_H263ProfileBaseline, OMX_VIDEO_H263Level20},
    {OMX_VIDEO_H263ProfileBaseline, OMX_VIDEO_H263Level30},
    {OMX_VIDEO_H263ProfileBaseline, OMX_VIDEO_H263Level40},
    {OMX_VIDEO_H263ProfileBaseline, OMX_VIDEO_H263Level45},
    {OMX_VIDEO_H263ProfileBaseline, OMX_VIDEO_H263Level50},
    {OMX_VIDEO_H263ProfileBaseline, OMX_VIDEO_H263Level60},
    {OMX_VIDEO_H263ProfileBaseline, OMX_VIDEO_H263Level70},
    {-1, -1}
};

/* MPEG4 Supported Levels & profiles */
VIDEO_PROFILE_LEVEL_TYPE SupportedMPEG4ProfileLevels[] = {
    {OMX_VIDEO_MPEG4ProfileSimple, OMX_VIDEO_MPEG4Level0},
    {OMX_VIDEO_MPEG4ProfileSimple, OMX_VIDEO_MPEG4Level0b},
    {OMX_VIDEO_MPEG4ProfileSimple, OMX_VIDEO_MPEG4Level1},
    {OMX_VIDEO_MPEG4ProfileSimple, OMX_VIDEO_MPEG4Level2},
    {OMX_VIDEO_MPEG4ProfileSimple, OMX_VIDEO_MPEG4Level3},
    {OMX_VIDEO_MPEG4ProfileSimple, OMX_VIDEO_MPEG4Level4},
    {OMX_VIDEO_MPEG4ProfileSimple, OMX_VIDEO_MPEG4Level4a},
    {OMX_VIDEO_MPEG4ProfileSimple, OMX_VIDEO_MPEG4Level5},
#if 0
    {OMX_VIDEO_MPEG4ProfileAdvancedSimple, OMX_VIDEO_MPEG4Level0},
    {OMX_VIDEO_MPEG4ProfileAdvancedSimple, OMX_VIDEO_MPEG4Level0b},
    {OMX_VIDEO_MPEG4ProfileAdvancedSimple, OMX_VIDEO_MPEG4Level1},
    {OMX_VIDEO_MPEG4ProfileAdvancedSimple, OMX_VIDEO_MPEG4Level2},
    {OMX_VIDEO_MPEG4ProfileAdvancedSimple, OMX_VIDEO_MPEG4Level3},
    {OMX_VIDEO_MPEG4ProfileAdvancedSimple, OMX_VIDEO_MPEG4Level4},
    {OMX_VIDEO_MPEG4ProfileAdvancedSimple, OMX_VIDEO_MPEG4Level5},
#endif
    {-1,-1}
};

/* AVC Supported Levels & profiles */
VIDEO_PROFILE_LEVEL_TYPE SupportedAVCProfileLevels[] = {
    {OMX_VIDEO_AVCProfileBaseline, OMX_VIDEO_AVCLevel1},
    {OMX_VIDEO_AVCProfileBaseline, OMX_VIDEO_AVCLevel1b},
    {OMX_VIDEO_AVCProfileBaseline, OMX_VIDEO_AVCLevel11},
    {OMX_VIDEO_AVCProfileBaseline, OMX_VIDEO_AVCLevel12},
    {OMX_VIDEO_AVCProfileBaseline, OMX_VIDEO_AVCLevel13},
    {OMX_VIDEO_AVCProfileBaseline, OMX_VIDEO_AVCLevel2},
    {OMX_VIDEO_AVCProfileBaseline, OMX_VIDEO_AVCLevel21},
    {OMX_VIDEO_AVCProfileBaseline, OMX_VIDEO_AVCLevel22},
    {OMX_VIDEO_AVCProfileBaseline, OMX_VIDEO_AVCLevel3},
    {OMX_VIDEO_AVCProfileBaseline, OMX_VIDEO_AVCLevel31},
    {-1,-1}
};

/*
 * Initializes a data structure using a pointer to the structure.
 * The initialization of OMX structures always sets up the nSize and nVersion fields
 * of the structure.
 */
#define OMX_CONF_INIT_STRUCT_PTR(_s_, _name_)   \
    memset((_s_), 0x0, sizeof(_name_));         \
    (_s_)->nSize = sizeof(_name_);              \
    (_s_)->nVersion.s.nVersionMajor = 0x1;      \
    (_s_)->nVersion.s.nVersionMinor = 0x0;      \
    (_s_)->nVersion.s.nRevision = 0x0;          \
    (_s_)->nVersion.s.nStep = 0x0


static OMX_ERRORTYPE omx_videoenc_component_AllocateBuffer(
    OMX_IN OMX_HANDLETYPE hComponent,
    OMX_INOUT OMX_BUFFERHEADERTYPE** pBuffer,
    OMX_IN OMX_U32 nPortIndex,
    OMX_IN OMX_PTR pAppPrivate,
    OMX_IN OMX_U32 nSizeBytes);

static OMX_ERRORTYPE omx_videoenc_component_UseBuffer(
    OMX_IN OMX_HANDLETYPE hComponent,
    OMX_INOUT OMX_BUFFERHEADERTYPE** ppBufferHdr,
    OMX_IN OMX_U32 nPortIndex,
    OMX_IN OMX_PTR pAppPrivate,
    OMX_IN OMX_U32 nSizeBytes,
    OMX_IN OMX_U8* pBuffer);

static OMX_ERRORTYPE omx_videoenc_component_FreeBuffer(
    OMX_IN  OMX_HANDLETYPE hComponent,
    OMX_IN  OMX_U32 nPortIndex,
    OMX_IN  OMX_BUFFERHEADERTYPE* pBuffer);

/** internal function to set codec related parameters in the private type structure
  */
#ifdef USE_VPU_OUTPUT_BUFFER
static void mem_prepare(videoenc_private_t* p_private) {
    p_private->mTmem_fd = -1;
    p_private->mMapInfo_video = MAP_FAILED;

    p_private->mTmem_fd = open("/dev/tmem", O_RDWR | O_NDELAY);
    if (p_private->mTmem_fd < 0) {
        memset(&p_private->mVideomap, 0, sizeof(pmap_t));
        return;
    }

#ifdef USE_VPU_OUTPUT_BUFFER
    memset(&p_private->mVideomap, 0, sizeof(pmap_t));
    pmap_get_info("video", &p_private->mVideomap);

    p_private->mMapInfo_video = (void *)mmap(0, p_private->mVideomap.size,
                                               PROT_READ | PROT_WRITE,
                                               MAP_SHARED,
                                               p_private->mTmem_fd,
                                               p_private->mVideomap.base);

    if (MAP_FAILED == p_private->mMapInfo_video) {
        ALOGE("[VENC-%d] mmap failed. fd(%d), base addr(0x%x), size(%d)", p_private->nVpuInstance,
                p_private->mTmem_fd, p_private->mVideomap.base, p_private->mVideomap.size);

        memset(&p_private->mVideomap, 0, sizeof(pmap_t));
        close(p_private->mTmem_fd);
        p_private->mTmem_fd = -1;
        return;
    }
#endif

}

static void mem_destory(videoenc_private_t* p_private) {
#ifdef USE_VPU_OUTPUT_BUFFER
    if (p_private->mMapInfo_video != MAP_FAILED) {
        munmap((void *)p_private->mMapInfo_video, p_private->mVideomap.size);
        p_private->mMapInfo_video = MAP_FAILED;
    }
	if(p_private->mVideomap.base){
	    pmap_release_info("video");
        p_private->mVideomap.base = 0x00;
    }
#endif

    if (p_private->mTmem_fd >= 0) {
        close(p_private->mTmem_fd);
        p_private->mTmem_fd = -1;
    }

    memset(&p_private->mVideomap, 0, sizeof(pmap_t));
}
#endif

#ifdef USE_VPU_OUTPUT_BUFFER
static OMX_BOOL mem_get_addr_video(
    videoenc_private_t *p_private,
    OMX_U32            *out_virtaddr,
    OMX_U32             in_phyaddr)
{
    if (p_private->mMapInfo_video != MAP_FAILED) {
        *out_virtaddr = (OMX_U32)((OMX_U32)p_private->mMapInfo_video
                      + (in_phyaddr - p_private->mVideomap.base));
        return OMX_TRUE;
    }

    return OMX_FALSE;
}
#endif

static void SetInternalVideoParameters(OMX_COMPONENTTYPE *openmaxStandComp) {
    videoenc_private_t* p_private = openmaxStandComp->pComponentPrivate;
    omx_base_video_PortType *inPort;

    switch(p_private->video_coding_type){
      case OMX_VIDEO_CodingMPEG4:
        (void)strncpy(p_private->ports[OMX_OUTPORT_INDEX]->sPortParam.format.video.cMIMEType,"video/mpeg4", sizeof("video/mpeg4"));
        p_private->ports[OMX_OUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingMPEG4;

        setHeader(&p_private->pVideoMpeg4, sizeof(OMX_VIDEO_PARAM_MPEG4TYPE));
        p_private->pVideoMpeg4.nPortIndex = 0x1;
        p_private->pVideoMpeg4.nSliceHeaderSpacing = 0;
        p_private->pVideoMpeg4.bSVH = OMX_FALSE;
        p_private->pVideoMpeg4.bGov = OMX_FALSE;
        p_private->pVideoMpeg4.nPFrames = 10;
        p_private->pVideoMpeg4.nBFrames = 0;
        p_private->pVideoMpeg4.nIDCVLCThreshold = 0;
        p_private->pVideoMpeg4.bACPred = OMX_FALSE;
        p_private->pVideoMpeg4.nMaxPacketSize = 256;
        p_private->pVideoMpeg4.nTimeIncRes = 0;
        p_private->pVideoMpeg4.eProfile = OMX_VIDEO_MPEG4ProfileSimple; //OMX_VIDEO_MPEG4ProfileCore
        p_private->pVideoMpeg4.eLevel = OMX_VIDEO_MPEG4Level0; //OMX_VIDEO_MPEG4Level2
        p_private->pVideoMpeg4.nAllowedPictureTypes = OMX_VIDEO_PictureTypeI | OMX_VIDEO_PictureTypeP;
        p_private->pVideoMpeg4.nHeaderExtension = 0;
        p_private->pVideoMpeg4.bReversibleVLC = OMX_FALSE;

        inPort = (omx_base_video_PortType *)p_private->ports[OMX_INPORT_INDEX];
        inPort->sVideoParam.eCompressionFormat = OMX_VIDEO_CodingUnused;
        break;
      case OMX_VIDEO_CodingAVC:
        (void)strncpy(p_private->ports[OMX_OUTPORT_INDEX]->sPortParam.format.video.cMIMEType,"video/avc", sizeof("video/avc"));
        p_private->ports[OMX_OUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingAVC;

        setHeader(&p_private->pVideoAvc, sizeof(OMX_VIDEO_PARAM_AVCTYPE));
        p_private->pVideoAvc.nPortIndex = 0x1;
        p_private->pVideoAvc.nSliceHeaderSpacing = 0;
        p_private->pVideoAvc.bUseHadamard = OMX_FALSE;
        p_private->pVideoAvc.nRefFrames = 2;
        p_private->pVideoAvc.nPFrames = 0;
        p_private->pVideoAvc.nBFrames = 0;
        p_private->pVideoAvc.eProfile = OMX_VIDEO_AVCProfileBaseline;
        p_private->pVideoAvc.eLevel = OMX_VIDEO_AVCLevel1;
        p_private->pVideoAvc.nAllowedPictureTypes = OMX_VIDEO_PictureTypeI | OMX_VIDEO_PictureTypeP;
        p_private->pVideoAvc.bFrameMBsOnly = OMX_FALSE;
        p_private->pVideoAvc.nRefIdx10ActiveMinus1 = 0;
        p_private->pVideoAvc.nRefIdx11ActiveMinus1 = 0;
        p_private->pVideoAvc.bEnableUEP = OMX_FALSE;
        p_private->pVideoAvc.bEnableFMO = OMX_FALSE;
        p_private->pVideoAvc.bEnableASO = OMX_FALSE;
        p_private->pVideoAvc.bEnableRS = OMX_FALSE;

        p_private->pVideoAvc.bMBAFF = OMX_FALSE;
        p_private->pVideoAvc.bEntropyCodingCABAC = OMX_FALSE;
        p_private->pVideoAvc.bWeightedPPrediction = OMX_FALSE;
        p_private->pVideoAvc.nWeightedBipredicitonMode = 0;
        p_private->pVideoAvc.bconstIpred = OMX_FALSE;
        p_private->pVideoAvc.bDirect8x8Inference = OMX_FALSE;
        p_private->pVideoAvc.bDirectSpatialTemporal = OMX_FALSE;
        p_private->pVideoAvc.nCabacInitIdc = 0;
        p_private->pVideoAvc.eLoopFilterMode = OMX_VIDEO_AVCLoopFilterDisable;

        inPort = (omx_base_video_PortType *)p_private->ports[OMX_INPORT_INDEX];
        inPort->sVideoParam.eCompressionFormat = OMX_VIDEO_CodingUnused;
        break;
      case OMX_VIDEO_CodingH263:
        strncpy(p_private->ports[OMX_OUTPORT_INDEX]->sPortParam.format.video.cMIMEType,"video/h263", sizeof("video/h263"));
        p_private->ports[OMX_OUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingH263;

        setHeader(&p_private->pVideoH263, sizeof(OMX_VIDEO_PARAM_H263TYPE));
        p_private->pVideoH263.nPortIndex = 0x1;
        p_private->pVideoH263.eProfile = OMX_VIDEO_H263ProfileBaseline; //OMX_VIDEO_MPEG4ProfileCore
        p_private->pVideoH263.eLevel = OMX_VIDEO_H263Level10;
        p_private->pVideoH263.bPLUSPTYPEAllowed = OMX_FALSE;
        p_private->pVideoH263.nAllowedPictureTypes = OMX_VIDEO_PictureTypeI | OMX_VIDEO_PictureTypeP;
        p_private->pVideoH263.bForceRoundingTypeToZero = OMX_TRUE;
        p_private->pVideoH263.nPictureHeaderRepetition = 0;
        p_private->pVideoH263.nGOBHeaderInterval = 0;
        p_private->pVideoH263.nPFrames = 10;
        p_private->pVideoH263.nBFrames = 0;

        inPort = (omx_base_video_PortType *)p_private->ports[OMX_INPORT_INDEX];
        inPort->sVideoParam.eCompressionFormat = OMX_VIDEO_CodingUnused;
        break;
      default:
        ALOGE("%s Unsupported video encoder codec type %lu",__func__, p_private->video_coding_type);
        break;
    }
}

OMX_ERRORTYPE omx_videoenc_component_Init(OMX_COMPONENTTYPE *openmaxStandComp) {
    videoenc_private_t* p_private = openmaxStandComp->pComponentPrivate;;
    OMX_ERRORTYPE err = OMX_ErrorNone;

    ALOGE("omx_videoenc_component_Init");

    if (p_private->video_coding_type == OMX_VIDEO_CodingMPEG4) {
        err = omx_videoenc_component_Constructor(openmaxStandComp, VIDEO_ENC_MPEG4_NAME);
    }else if (p_private->video_coding_type == OMX_VIDEO_CodingAVC) {
        err = omx_videoenc_component_Constructor(openmaxStandComp, VIDEO_ENC_H264_NAME);
    }else if (p_private->video_coding_type == OMX_VIDEO_CodingH263) {
        err = omx_videoenc_component_Constructor(openmaxStandComp, VIDEO_ENC_H263_NAME);
    }

    return err;
}
/** The Constructor of the video decoder component
  * @param openmaxStandComp the component handle to be constructed
  * @param cComponentName is the name of the constructed component
  */
OMX_ERRORTYPE omx_videoenc_component_Constructor(OMX_COMPONENTTYPE *openmaxStandComp,OMX_STRING cComponentName) {
  OMX_ERRORTYPE eError = OMX_ErrorNone;
  videoenc_private_t* p_private;
  omx_base_video_PortType *inPort,*outPort;

    DBUG_MSG( "[In %s, allocating component\n", __func__);
    openmaxStandComp->pComponentPrivate = TCC_calloc(1, sizeof(videoenc_private_t));
    if (openmaxStandComp->pComponentPrivate == NULL) {
        return OMX_ErrorInsufficientResources;
    }

    p_private = openmaxStandComp->pComponentPrivate;
    p_private->ports = NULL;

    eError = omx_base_filter_Constructor(openmaxStandComp, cComponentName);

    p_private->sPortTypesParam[OMX_PortDomainVideo].nStartPortNumber = 0;
    p_private->sPortTypesParam[OMX_PortDomainVideo].nPorts = 2;

    /** Allocate Ports and call port constructor. */
    if (/*p_private->sPortTypesParam[OMX_PortDomainVideo].nPorts &&*/ !p_private->ports) {
        p_private->ports = TCC_calloc(p_private->sPortTypesParam[OMX_PortDomainVideo].nPorts,
                                      sizeof(omx_base_PortType *));
        if (!p_private->ports) {
            ALOGE("%s, Port allocating error", __func__);
            return OMX_ErrorInsufficientResources;
        }

        OMX_U32 i;
        for (i = 0; i < p_private->sPortTypesParam[OMX_PortDomainVideo].nPorts; i++) {
            p_private->ports[i] = TCC_calloc(1, sizeof(omx_base_video_PortType));
            if (!p_private->ports[i]) {
                ALOGE("%s, Port allocating error1", __func__);
                return OMX_ErrorInsufficientResources;
            }
        }
        base_video_port_Constructor(openmaxStandComp, &p_private->ports[0], 0, OMX_TRUE);
        base_video_port_Constructor(openmaxStandComp, &p_private->ports[1], 1, OMX_FALSE);
    }

    /** now it's time to know the video coding type of the component */
    if (!strcmp(cComponentName, VIDEO_ENC_MPEG4_NAME)) {
        p_private->video_coding_type = OMX_VIDEO_CodingMPEG4;
    } else if (!strcmp(cComponentName, VIDEO_ENC_H264_NAME)) {
        p_private->video_coding_type = OMX_VIDEO_CodingAVC;
    } else if (!strcmp(cComponentName, VIDEO_ENC_H263_NAME)) {
        p_private->video_coding_type = OMX_VIDEO_CodingH263;
    } else if (!strcmp(cComponentName, VIDEO_ENC_BASE_NAME)) {
        p_private->video_coding_type = OMX_VIDEO_CodingUnused;
    } else {
        // IL client specified an invalid component name
        ALOGE("%s, OMX_ErrorInvalidComponentName", __func__);
        return OMX_ErrorInvalidComponentName;
    }

    /** here we can override whatever defaults the base_component constructor set
    * e.g. we can override the function pointers in the private struct
    */

    /** Domain specific section for the ports.
    * first we set the parameter common to both formats
    */
    //common parameters related to input port.
    inPort = (omx_base_video_PortType *)p_private->ports[OMX_INPORT_INDEX];
    inPort->sPortParam.nBufferSize = DEFAULT_IN_BUFFER_SIZE;
    inPort->sPortParam.format.video.xFramerate = (30 << 16);
    inPort->sPortParam.nBufferCountMin = NUM_IN_BUFFERS;
    inPort->sPortParam.nBufferCountActual = NUM_IN_BUFFERS;
    inPort->sPortParam.format.video.eColorFormat = INPUT_ENCODED_COLOR_FMT;
    inPort->sVideoParam.eColorFormat = INPUT_ENCODED_COLOR_FMT;

    //common parameters related to output port
    outPort = (omx_base_video_PortType *)p_private->ports[OMX_OUTPORT_INDEX];
    outPort->sPortParam.nBufferSize =  OUT_BUFFER_SIZE;
    outPort->sPortParam.format.video.xFramerate = (30 << 16);
    outPort->sPortParam.nBufferCountMin = NUM_OUT_BUFFERS;
    outPort->sPortParam.nBufferCountActual = NUM_OUT_BUFFERS;

    if (p_private->video_coding_type == OMX_VIDEO_CodingMPEG4) {
        outPort->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingMPEG4;
        outPort->sVideoParam.eCompressionFormat = OMX_VIDEO_CodingMPEG4;
        p_private->mBitstreamFormat = STD_MPEG4;
    } else if (p_private->video_coding_type == OMX_VIDEO_CodingAVC) {
        outPort->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingAVC;
        outPort->sVideoParam.eCompressionFormat = OMX_VIDEO_CodingAVC;
        p_private->mBitstreamFormat = STD_AVC;
    } else if (p_private->video_coding_type == OMX_VIDEO_CodingH263) {
        outPort->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingH263;
        outPort->sVideoParam.eCompressionFormat = OMX_VIDEO_CodingH263;
        p_private->mBitstreamFormat = STD_H263;
    }

    p_private->gspfVEnc = gspfVEncList[p_private->mBitstreamFormat];
    if (p_private->gspfVEnc == 0) {
        return OMX_ErrorComponentNotFound;
    }

    outPort->sPortParam.format.video.nBitrate = 64000;
    outPort->sPortParam.format.video.xFramerate = (30 << 16);

    /** settings of output port parameter definition */
    outPort->sVideoParam.xFramerate = (30 << 16);

    if (!p_private->avCodecSyncSem) {
        p_private->avCodecSyncSem = TCC_malloc(sizeof(tsem_t));
        if (p_private->avCodecSyncSem == NULL) {
            ALOGE("%s, avCodecSyncSem  - OMX_ErrorInsufficientResources", __func__);
            return OMX_ErrorInsufficientResources;
        }
        tsem_init(p_private->avCodecSyncSem, 0);
    }

    SetInternalVideoParameters(openmaxStandComp);

    p_private->eOutFramePixFmt = PIX_FMT_YUV420P;

    /** general configuration irrespective of any video formats
    * setting other parameters of omx_videodec_component_private
    */
    //p_private->avCodec = NULL;
    //p_private->avCodecContext= NULL;
    p_private->avcodecReady = OMX_FALSE;
    p_private->extradata = NULL;
    p_private->extradata_size = 0;
    p_private->BufferMgmtCallback = omx_videoenc_component_BufferMgmtCallback;
    p_private->isVPUClosed = OMX_TRUE;
    p_private->isEncError = OMX_FALSE;
    p_private->qp_value = 0;

    /** initializing the codec context etc that was done earlier by ffmpeglibinit function */
    p_private->messageHandler = omx_videoenc_component_MessageHandler;
    p_private->destructor = omx_videoenc_component_Destructor;

    openmaxStandComp->SetParameter = omx_videoenc_component_SetParameter;
    openmaxStandComp->GetParameter = omx_videoenc_component_GetParameter;
    openmaxStandComp->SetConfig    = omx_videoenc_component_SetConfig;
    openmaxStandComp->GetConfig    = omx_videoenc_component_GetConfig;
    openmaxStandComp->ComponentRoleEnum = omx_videoenc_component_ComponentRoleEnum;
    openmaxStandComp->GetExtensionIndex = omx_videoenc_component_GetExtensionIndex;

    //For reducing needless memory copy.
    openmaxStandComp->AllocateBuffer = omx_videoenc_component_AllocateBuffer;
    openmaxStandComp->FreeBuffer = omx_videoenc_component_FreeBuffer;
    openmaxStandComp->UseBuffer = omx_videoenc_component_UseBuffer;

    if (p_private->video_coding_type == OMX_VIDEO_CodingH263)
        p_private->iConfigDataFlag = OMX_TRUE;
    else
        p_private->iConfigDataFlag = OMX_FALSE;

    gVEncInstanceCount++;

    if(gVEncInstanceCount > MAX_COMPONENT_VIDEOENC) {
        ALOGE( "%s, gVEncInstanceCount > MAX_COMPONENT_VIDEOENC \n", __func__);
        return OMX_ErrorInsufficientResources;
    }


    //OMX_VIDEO_PARAM_PROFILELEVELTYPE structure
    OMX_CONF_INIT_STRUCT_PTR(&p_private->sProfileLevel, OMX_VIDEO_PARAM_PROFILELEVELTYPE);
    p_private->sProfileLevel.nPortIndex = 0x1;
    p_private->sProfileLevel.nProfileIndex = 0;
    if (p_private->video_coding_type == OMX_VIDEO_CodingMPEG4) {
        p_private->sProfileLevel.eProfile = OMX_VIDEO_MPEG4ProfileSimple;
        p_private->sProfileLevel.eLevel = OMX_VIDEO_MPEG4Level2;
    }else if (p_private->video_coding_type == OMX_VIDEO_CodingAVC) {
        p_private->sProfileLevel.eProfile = OMX_VIDEO_AVCProfileBaseline;
        p_private->sProfileLevel.eLevel = OMX_VIDEO_AVCLevel1;
    }else if (p_private->video_coding_type == OMX_VIDEO_CodingH263) {
        p_private->sProfileLevel.eProfile = OMX_VIDEO_H263ProfileBaseline;
        p_private->sProfileLevel.eLevel = OMX_VIDEO_H263Level45;
    }


    //OMX_CONFIG_ROTATIONTYPE SETTINGS ON INPUT PORT
    OMX_CONF_INIT_STRUCT_PTR(&p_private->stRotationType, OMX_CONFIG_ROTATIONTYPE);
    p_private->stRotationType.nPortIndex = OMX_DirInput;
    p_private->stRotationType.nRotation = -1;  //For all the YUV formats that are other than RGB


    //OMX_VIDEO_PARAM_ERRORCORRECTIONTYPE settings of output port
    memset(&p_private->stErrCorrectionType, 0, sizeof(OMX_VIDEO_PARAM_ERRORCORRECTIONTYPE));
    OMX_CONF_INIT_STRUCT_PTR(&p_private->stErrCorrectionType, OMX_VIDEO_PARAM_ERRORCORRECTIONTYPE);
    p_private->stErrCorrectionType.nPortIndex = OMX_DirOutput;
    p_private->stErrCorrectionType.bEnableDataPartitioning = OMX_FALSE;  //As in node default is h263


    //OMX_VIDEO_PARAM_BITRATETYPE settings of output port
    OMX_CONF_INIT_STRUCT_PTR(&p_private->stBitrateType, OMX_VIDEO_PARAM_BITRATETYPE);
    p_private->stBitrateType.nPortIndex = OMX_DirOutput;
    p_private->stBitrateType.eControlRate = OMX_Video_ControlRateVariable;
    p_private->stBitrateType.nTargetBitrate = 64000;


    //OMX_CONFIG_FRAMERATETYPE default seetings (specified in khronos conformance test)
    OMX_CONF_INIT_STRUCT_PTR(&p_private->stFrameRateType, OMX_CONFIG_FRAMERATETYPE);
    p_private->stFrameRateType.nPortIndex = OMX_DirOutput;
    p_private->stFrameRateType.xEncodeFramerate = 0;

    //OMX_VIDEO_CONFIG_BITRATETYPE default settings (specified in khronos conformance test)
    OMX_CONF_INIT_STRUCT_PTR(&p_private->stBitRateType, OMX_VIDEO_CONFIG_BITRATETYPE);
    p_private->stBitRateType.nPortIndex = OMX_DirOutput;
    p_private->stBitRateType.nEncodeBitrate = 0;

    //OMX_VIDEO_PARAM_QUANTIZATIONTYPE settings of output port
    OMX_CONF_INIT_STRUCT_PTR(&p_private->stQuantType, OMX_VIDEO_PARAM_QUANTIZATIONTYPE);
    p_private->stQuantType.nPortIndex = OMX_DirOutput;
    p_private->stQuantType.nQpI = 15;
    p_private->stQuantType.nQpP = 12;
    p_private->stQuantType.nQpB = 12;


    //OMX_VIDEO_PARAM_VBSMCTYPE settings of output port
    memset(&p_private->stVBSMCType, 0, sizeof(OMX_VIDEO_PARAM_VBSMCTYPE));
    OMX_CONF_INIT_STRUCT_PTR(&p_private->stVBSMCType, OMX_VIDEO_PARAM_VBSMCTYPE);
    p_private->stVBSMCType.nPortIndex = OMX_DirOutput;
    p_private->stVBSMCType.b16x16 = OMX_TRUE;


    //OMX_VIDEO_PARAM_MOTIONVECTORTYPE settings of output port
    memset(&p_private->stMotionVectorType, 0, sizeof(OMX_VIDEO_PARAM_MOTIONVECTORTYPE));
    OMX_CONF_INIT_STRUCT_PTR(&p_private->stMotionVectorType, OMX_VIDEO_PARAM_MOTIONVECTORTYPE);
    p_private->stMotionVectorType.nPortIndex = OMX_DirOutput;
    p_private->stMotionVectorType.eAccuracy = OMX_Video_MotionVectorHalfPel;
    p_private->stMotionVectorType.bUnrestrictedMVs = OMX_TRUE;
    p_private->stMotionVectorType.sXSearchRange = 16;
    p_private->stMotionVectorType.sYSearchRange = 16;


    //OMX_VIDEO_PARAM_INTRAREFRESHTYPE settings of output port
    memset(&p_private->stIntraRefreshType, 0, sizeof(OMX_VIDEO_PARAM_INTRAREFRESHTYPE));
    OMX_CONF_INIT_STRUCT_PTR(&p_private->stIntraRefreshType, OMX_VIDEO_PARAM_INTRAREFRESHTYPE);
    p_private->stIntraRefreshType.nPortIndex = OMX_DirOutput;
    p_private->stIntraRefreshType.eRefreshMode = OMX_VIDEO_IntraRefreshCyclic;
    p_private->stIntraRefreshType.nCirMBs = 0;


    //OMX_CONFIG_INTRAREFRESHVOPTYPE settings of output port
    memset(&p_private->stIntraRefreshVop, 0, sizeof(OMX_CONFIG_INTRAREFRESHVOPTYPE));
    OMX_CONF_INIT_STRUCT_PTR(&p_private->stIntraRefreshVop, OMX_CONFIG_INTRAREFRESHVOPTYPE);
    p_private->stIntraRefreshVop.nPortIndex = OMX_DirOutput;
    p_private->stIntraRefreshVop.IntraRefreshVOP = OMX_FALSE;

#if defined(USE_VPU_OUTPUT_BUFFER)
    mem_prepare(p_private);
#endif
    p_private->mCopy_fd = -1;
    p_private->pVpuInstance = NULL;
    p_private->nVpuInstance = 0;

    return eError;
}

static OMX_ERRORTYPE _encoder_close(videoenc_private_t* p_private, OMX_BOOL real_close) {
    OMX_S32 ret = 0;

    ALOGI("_encoder_close: isVPUClosed(%d), real_close(%d)", p_private->isVPUClosed, real_close);

    if (p_private->isVPUClosed == OMX_FALSE) {
        ret = p_private->gspfVEnc(VENC_CLOSE, NULL, NULL, NULL, p_private->pVpuInstance);
        p_private->isVPUClosed = OMX_TRUE;
        if (ret < 0) {
            ALOGE("[VENC-%d][Err:%ld] VENC_CLOSE failed - %d", p_private->nVpuInstance, ret, __LINE__);
        }

        if (p_private->pVpuInstance) {
            venc_release_instance(p_private->pVpuInstance, p_private->gsVEncInit.m_iBitstreamFormat);
            p_private->pVpuInstance = NULL;
        }

        if (!real_close) {
            p_private->pVpuInstance = venc_alloc_instance(p_private->gsVEncInit.m_iBitstreamFormat);
            p_private->nVpuInstance = venc_get_instance_index(p_private->pVpuInstance);
        }
    }

    return (ret < 0) ? OMX_ErrorHardware : OMX_ErrorNone;
}

/** The destructor of the video decoder component
  */
OMX_ERRORTYPE omx_videoenc_component_Destructor(OMX_COMPONENTTYPE *openmaxStandComp) {
    videoenc_private_t* p_private = openmaxStandComp->pComponentPrivate;

    _encoder_close(p_private, OMX_TRUE);

    if (p_private->pVpuInstance) {
        venc_release_instance(p_private->pVpuInstance, p_private->gsVEncInit.m_iBitstreamFormat);
        p_private->pVpuInstance = NULL;
    }

    if (p_private->extradata) {
        TCC_free(p_private->extradata);
        p_private->extradata=NULL;
    }

    if (p_private->avCodecSyncSem) {
        tsem_deinit(p_private->avCodecSyncSem);
        TCC_free(p_private->avCodecSyncSem);
        p_private->avCodecSyncSem = NULL;
    }

    /* frees port(s) */
    if (p_private->ports) {
        OMX_U32 i;
        for (i=0; i < p_private->sPortTypesParam[OMX_PortDomainVideo].nPorts; i++) {
            if (p_private->ports[i]) {
                p_private->ports[i]->PortDestructor(p_private->ports[i]);
            }
        }
        TCC_free(p_private->ports);
        p_private->ports = NULL;
    }

    ALOGD("[VENC-%d] Destructor of video encoder component is called", p_private->nVpuInstance);

#if defined(USE_VPU_OUTPUT_BUFFER)
    mem_destory(p_private);
#endif

    if (p_private->mCopy_fd > 0) {
        close(p_private->mCopy_fd);
    }

    omx_base_filter_Destructor(openmaxStandComp);

    gVEncInstanceCount--;

    return OMX_ErrorNone;
}

#ifdef MAX_MIN_BPS_LIMITATION
static OMX_U32 get_bitrate(OMX_U32  uiWidth, OMX_U32  uiHeight, OMX_U32  quality) {
    OMX_U32 baseBitrateKbps;   // 320x240 base
    OMX_U32 bitrateKbps;
    OMX_U32 basePixel;
    OMX_U32 comparePixel;
    OMX_U32 rate;

    if (uiWidth * uiHeight <= 0) {
        return 0;
    }

    switch (quality) {
        case 0:
            baseBitrateKbps = 768 * 1024; // QQVGA/QVGA/VGA/HD = 192/768/3072/9216
            break;
        case 1:
            baseBitrateKbps = 512 * 1024; // QQVGA/QVGA/VGA/HD = 128/512/2048/7192
            break;

        case 2:
            baseBitrateKbps = 384 * 1024; // QQVGA/QVGA/VGA/HD = 96/384/1536/4608
            break;
        case 3:
            baseBitrateKbps = 256 * 1024; // QQVGA/QVGA/VGA/HD = 64/256/1024/3096
            break;
        case 4:
            baseBitrateKbps = 192 * 1024; // QQVGA/QVGA/VGA/HD = 48/192/768/2304
            break;
        default:
            baseBitrateKbps =  96 * 1024; // QQVGA/QVGA/VGA/HD = 24/96/384/1152
            break;
    }

    basePixel = 320 * 240;
    comparePixel = uiWidth * uiHeight;

    rate = (comparePixel * 100) / basePixel;

    bitrateKbps = rate * baseBitrateKbps / 100;

    bitrateKbps = ((bitrateKbps + 127) / 128) * 128;

    if (bitrateKbps < 128) bitrateKbps = 128;

    return bitrateKbps;
}
#endif

/** It initializates the FFmpeg framework, and opens an FFmpeg videodecoder of type specified by IL client
  */
OMX_ERRORTYPE omx_videoenc_component_LibInit(videoenc_private_t* p_private) {
    OMX_S32 fps, keyInterval, kbps;

    omx_base_video_PortType *outPort = (omx_base_video_PortType *)p_private->ports[OMX_OUTPORT_INDEX];
    omx_base_video_PortType *inPort = (omx_base_video_PortType *)p_private->ports[OMX_INPORT_INDEX];

    if (!p_private->pVpuInstance) {
        p_private->pVpuInstance = venc_alloc_instance(p_private->gsVEncInit.m_iBitstreamFormat);
    }
    p_private->nVpuInstance = venc_get_instance_index(p_private->pVpuInstance);

    fps = inPort->sPortParam.format.video.xFramerate / (1 << 16);
    kbps = outPort->sPortParam.format.video.nBitrate / 1024;

    if (p_private->stBitRateType.nEncodeBitrate != 0 &&
        p_private->stBitRateType.nEncodeBitrate != outPort->sPortParam.format.video.nBitrate)
    {
        outPort->sPortParam.format.video.nBitrate = p_private->stBitRateType.nEncodeBitrate;
        kbps = outPort->sPortParam.format.video.nBitrate / 1024;
    }

    if (p_private->stFrameRateType.xEncodeFramerate != 0 &&
        p_private->stFrameRateType.xEncodeFramerate != inPort->sPortParam.format.video.xFramerate)
    {
        inPort->sPortParam.format.video.xFramerate = p_private->stFrameRateType.xEncodeFramerate;
        fps = inPort->sPortParam.format.video.xFramerate / (1 << 16);
    }

    if (p_private->video_coding_type == OMX_VIDEO_CodingMPEG4) {
        keyInterval = p_private->pVideoMpeg4.nPFrames;
    } else if (p_private->video_coding_type == OMX_VIDEO_CodingAVC) {
        keyInterval = p_private->pVideoAvc.nPFrames;
    } else if (p_private->video_coding_type == OMX_VIDEO_CodingH263) {
        keyInterval = p_private->pVideoH263.nPFrames;

        if (fps < 15)
            fps = 15;
        if (keyInterval < 15)
            keyInterval = 15;
    } else {
        keyInterval = 15;
    }

    if (keyInterval == 0)
        keyInterval = fps * 2;

    total_count = 0;
    memset(&p_private->gsVEncInit, 0x00, sizeof(venc_init_t));
    memset(&p_private->gsVEncSeqHeader, 0x00, sizeof(venc_seq_header_t));
    memset(&p_private->gsVEncInput, 0x00, sizeof(venc_input_t));
    memset(&p_private->gsVEncOutput, 0x00, sizeof(venc_output_t));
    memset(&p_private->stFrameRateType, 0x00, sizeof(OMX_CONFIG_FRAMERATETYPE));
    memset(&p_private->stBitRateType, 0x00, sizeof(OMX_VIDEO_CONFIG_BITRATETYPE));

    p_private->gsVEncInit.m_iBitstreamFormat  = p_private->mBitstreamFormat;
    p_private->gsVEncInit.m_iPicWidth         = outPort->sPortParam.format.video.nFrameWidth;
    p_private->gsVEncInit.m_iPicHeight        = outPort->sPortParam.format.video.nFrameHeight;
    p_private->gsVEncInit.m_iFrameRate        = fps;
    p_private->gsVEncInit.m_iTargetKbps       = kbps;

    if (keyInterval > 32000)
        p_private->gsVEncInit.m_iKeyInterval  = 32000; // fps * 2; // only first picture is I
    else
        p_private->gsVEncInit.m_iKeyInterval  = keyInterval; // only first picture is I

#ifdef MAX_MIN_BPS_LIMITATION
    OMX_U32 max_Bps, min_Bps;
    OMX_U32 res_region = outPort->sPortParam.format.video.nFrameWidth * outPort->sPortParam.format.video.nFrameHeight;

    max_Bps = get_bitrate(p_private->gsVEncInit.m_iPicWidth, p_private->gsVEncInit.m_iPicHeight, 0);
    min_Bps = get_bitrate(p_private->gsVEncInit.m_iPicWidth, p_private->gsVEncInit.m_iPicHeight, 5);

    //temp
    if (res_region <= 160 * 120) {
        min_Bps = 128 * 1024;
    } else if (res_region <= 176 * 144) {
        min_Bps = 128 * 1024;
    } else if (res_region <= 320 * 240) {
        min_Bps = 192 * 1024;
    }

    if (fps > 20) {
        max_Bps *= 2;
    }

    if (max_Bps/1024 < p_private->gsVEncInit.m_iTargetKbps) {
        p_private->gsVEncInit.m_iTargetKbps = max_Bps/1024;
    } else if (min_Bps/1024 > p_private->gsVEncInit.m_iTargetKbps) {
        p_private->gsVEncInit.m_iTargetKbps = min_Bps/1024;
    }
    ALOGE("[VENC-%d] ENC Bitrate Info :: %d fmt, %dx%d, %d fps, %d interval, %d(%d ~ %d) Kbps",
            p_private->nVpuInstance, p_private->gsVEncInit.m_iBitstreamFormat,
            p_private->gsVEncInit.m_iPicWidth, p_private->gsVEncInit.m_iPicHeight,
            p_private->gsVEncInit.m_iFrameRate, p_private->gsVEncInit.m_iKeyInterval,
            p_private->gsVEncInit.m_iTargetKbps, max_Bps/1024, min_Bps/1024);
#endif

    p_private->gsVEncInit.m_iAvcFastEncoding  = 0;
    p_private->gsVEncInit.m_iSliceMode        = 0; // 0:Off, 1: On
    p_private->gsVEncInit.m_iSliceSizeMode    = 0; // 0:bits, 1:MBs
    p_private->gsVEncInit.m_iSliceSize        = 0; // 32Kb

    ALOGI("[VENC-%d] ENC Info :: %d fmt, %dx%d, %d fps, %d interval, %d Kbps",
            p_private->nVpuInstance, p_private->gsVEncInit.m_iBitstreamFormat,
            p_private->gsVEncInit.m_iPicWidth, p_private->gsVEncInit.m_iPicHeight,
            p_private->gsVEncInit.m_iFrameRate, p_private->gsVEncInit.m_iKeyInterval,
            p_private->gsVEncInit.m_iTargetKbps);

    if (p_private->bSwap_input == OMX_TRUE) {
        p_private->gsVEncInit.m_bNeedSwap = 1;
    }

    OMX_S32 nal_start = 1;
    p_private->bSeenEOS = OMX_FALSE;

    OMX_S32 ret = p_private->gspfVEnc(VENC_INIT, NULL,
                                      &p_private->gsVEncInit,
                                      (void *)&nal_start, p_private->pVpuInstance);
    if (ret < 0) {
        ALOGE("[VENC-%d][Err:%ld] VENC_INIT failed", p_private->nVpuInstance, ret);
        if (ret != -VPU_ENV_INIT_ERROR) {
            _encoder_close(p_private, OMX_FALSE);
        }
        return OMX_ErrorHardware;
    }

    p_private->isVPUClosed = OMX_FALSE;

    ALOGI("[VENC-%d] VENC_INIT Success!! size = %lu x %lu", p_private->nVpuInstance,
            outPort->sPortParam.format.video.nFrameWidth, outPort->sPortParam.format.video.nFrameHeight);

    if (p_private->isEncError == OMX_FALSE)
        tsem_up(p_private->avCodecSyncSem);

    return OMX_ErrorNone;
}

/** It Deinitializates the ffmpeg framework, and close the ffmpeg video decoder of selected coding type
  */
OMX_ERRORTYPE omx_videoenc_component_LibDeinit(videoenc_private_t* p_private) {
    return  _encoder_close(p_private, OMX_TRUE);
}

/** The Initialization function of the video decoder
  */
OMX_ERRORTYPE omx_videoenc_component_Initialize(OMX_COMPONENTTYPE *openmaxStandComp) {
    videoenc_private_t* p_private = openmaxStandComp->pComponentPrivate;
    OMX_ERRORTYPE eError = OMX_ErrorNone;

    /** Temporary First Output buffer size */
    p_private->inputCurrBuffer = NULL;
    p_private->inputCurrLength = 0;
    p_private->isFirstBuffer = 1;
    p_private->isNewBuffer = 1;

    return eError;
}

/** The Deinitialization function of the video decoder
  */
OMX_ERRORTYPE omx_videoenc_component_Deinit(OMX_COMPONENTTYPE *openmaxStandComp) {
    videoenc_private_t *p_private = openmaxStandComp->pComponentPrivate;
    OMX_ERRORTYPE eError = OMX_ErrorNone;

    if (p_private->avcodecReady) {
        omx_videoenc_component_LibDeinit(p_private);
        p_private->avcodecReady = OMX_FALSE;
    }

    return eError;
}

OMX_ERRORTYPE omx_videoenc_component_AllocateBuffer(
    OMX_IN OMX_HANDLETYPE hComponent,
    OMX_INOUT OMX_BUFFERHEADERTYPE** pBuffer,
    OMX_IN OMX_U32 nPortIndex,
    OMX_IN OMX_PTR pAppPrivate,
    OMX_IN OMX_U32 nSizeBytes)
{
    OMX_ERRORTYPE eError;

    eError = omx_base_component_AllocateBuffer(hComponent, pBuffer, nPortIndex, pAppPrivate, nSizeBytes);
#ifdef FOR_V2IP
    if (eError == OMX_ErrorNone && nPortIndex == OMX_DirOutput) {
        (*pBuffer)->pOutputPortPrivate = TCC_calloc(1, sizeof(TCC_PLATFORM_PRIVATE_PMEM_INFO));
        DBUG_MSG("Allocate OutputPortPrivate 0x%x - 0x%x",
                (OMX_U32)(*pBuffer)->pOutputPortPrivate, sizeof(TCC_PLATFORM_PRIVATE_PMEM_INFO));
    }
#endif

    return eError;
}

OMX_ERRORTYPE omx_videoenc_component_UseBuffer(
    OMX_IN OMX_HANDLETYPE hComponent,
    OMX_INOUT OMX_BUFFERHEADERTYPE** ppBufferHdr,
    OMX_IN OMX_U32 nPortIndex,
    OMX_IN OMX_PTR pAppPrivate,
    OMX_IN OMX_U32 nSizeBytes,
    OMX_IN OMX_U8* pBuffer)
{
#ifdef  FOR_V2IP
    OMX_COMPONENTTYPE *openmaxStandComp = (OMX_COMPONENTTYPE *)hComponent;
    videoenc_private_t* p_private = (videoenc_private_t *)openmaxStandComp->pComponentPrivate;
#endif
    OMX_ERRORTYPE eError = omx_base_component_UseBuffer(
            hComponent, ppBufferHdr, nPortIndex, pAppPrivate, nSizeBytes, pBuffer);
#ifdef  FOR_V2IP
    if (eError == OMX_ErrorNone && nPortIndex == OMX_DirOutput) {
        (*ppBufferHdr)->pOutputPortPrivate = TCC_calloc(1, sizeof(TCC_PLATFORM_PRIVATE_PMEM_INFO));
        DBUG_MSG("[VENC-%d] Use OutputPortPrivate 0x%x - 0x%x", p_private->nVpuInstance,
                (OMX_U32)(*ppBufferHdr)->pOutputPortPrivate, sizeof(TCC_PLATFORM_PRIVATE_PMEM_INFO));
    }
#endif

    return eError;
}

OMX_ERRORTYPE omx_videoenc_component_FreeBuffer(
    OMX_IN  OMX_HANDLETYPE hComponent,
    OMX_IN  OMX_U32 nPortIndex,
    OMX_IN  OMX_BUFFERHEADERTYPE* pBuffer)
{
    if ((pBuffer->pOutputPortPrivate) && (nPortIndex == OMX_DirOutput)) {
        TCC_free(pBuffer->pOutputPortPrivate);
    }
    return omx_base_component_FreeBuffer(hComponent, nPortIndex, pBuffer);
}

/** Executes all the required steps after an output buffer frame-size has changed.
*/
static inline void UpdateFrameSize(OMX_COMPONENTTYPE *openmaxStandComp) {
    videoenc_private_t* p_private = openmaxStandComp->pComponentPrivate;

    omx_base_video_PortType *outPort = (omx_base_video_PortType *)p_private->ports[OMX_OUTPORT_INDEX];
    omx_base_video_PortType *inPort = (omx_base_video_PortType *)p_private->ports[OMX_INPORT_INDEX];

    outPort->sPortParam.format.video.nFrameWidth  = inPort->sPortParam.format.video.nFrameWidth;
    outPort->sPortParam.format.video.nFrameHeight = inPort->sPortParam.format.video.nFrameHeight;

    switch(inPort->sVideoParam.eColorFormat) {
    case OMX_COLOR_FormatYUV420Planar:
        if (inPort->sPortParam.format.video.nFrameWidth && inPort->sPortParam.format.video.nFrameHeight) {
            if (p_private->BufferType == EncoderMetadataPtr) {
            #ifdef IN_BUFFER_REAL_SIZE
                inPort->sPortParam.nBufferSize =
                    inPort->sPortParam.format.video.nFrameWidth * inPort->sPortParam.format.video.nFrameHeight * 3/2;
            #else
                inPort->sPortParam.nBufferSize = IN_META_MIN_BUFFER_SIZE;
            #endif
            } else {
                inPort->sPortParam.nBufferSize =
                    inPort->sPortParam.format.video.nFrameWidth * inPort->sPortParam.format.video.nFrameHeight * 3/2;
            }
        }
		break;

#if 0
	case OMX_COLOR_FormatAndroidOpaque:
		inPort->sPortParam.nBufferSize = IN_META_MIN_BUFFER_SIZE;
		break;
#endif
    default:
        if (inPort->sPortParam.format.video.nFrameWidth && inPort->sPortParam.format.video.nFrameHeight) {
            inPort->sPortParam.nBufferSize =
                inPort->sPortParam.format.video.nFrameWidth * inPort->sPortParam.format.video.nFrameHeight * 3;
        }
        break;
    }
}

static void GetInputYuv(videoenc_private_t *pPrivate, OMX_BUFFERHEADERTYPE* pInputBuffer)
{
    unsigned char *pAddr_YuvBuf_VA = vpu_get_YuvBufAddr(VA, pPrivate->pVpuInstance);
    if (pAddr_YuvBuf_VA != NULL) {
        (void)memcpy(pAddr_YuvBuf_VA, pInputBuffer->pBuffer, pInputBuffer->nFilledLen);

        // Set Input YUV
        pPrivate->gsVEncInput.m_pInputY = vpu_get_YuvBufAddr(PA, pPrivate->pVpuInstance);
        pPrivate->gsVEncInput.m_pInputCbCr[0] = pPrivate->gsVEncInput.m_pInputY
                + (pPrivate->gsVEncInit.m_iPicWidth * pPrivate->gsVEncInit.m_iPicHeight);
        pPrivate->gsVEncInput.m_pInputCbCr[1] = pPrivate->gsVEncInput.m_pInputCbCr[0]
                + (pPrivate->gsVEncInit.m_iPicWidth * pPrivate->gsVEncInit.m_iPicHeight / 4);
    } else {
        ALOGE("[%d] Fail to get YuvBufAddr", __LINE__);
    }
}

/** This function is used to process the input buffer and provide one output buffer
  */
void omx_videoenc_component_BufferMgmtCallback(
    OMX_COMPONENTTYPE       *openmaxStandComp,
    OMX_BUFFERHEADERTYPE    *pInputBuffer,
    OMX_BUFFERHEADERTYPE    *pOutputBuffer)
{
    videoenc_private_t* p_private = openmaxStandComp->pComponentPrivate;
    OMX_S32 ret;
    OMX_S32 nOutputFilled = 0;
    OMX_S32 nLen = 0;
    int internalOutputFilled = 0;
    OMX_U32 output_len;
    OMX_BOOL bCodecConfigData = OMX_FALSE;
    OMX_U32 nEncOutBuffAddr = 0x00;

    omx_base_video_PortType *outPort = (omx_base_video_PortType *)p_private->ports[OMX_OUTPORT_INDEX];
    omx_base_video_PortType *inPort = (omx_base_video_PortType *)p_private->ports[OMX_INPORT_INDEX];

    /** Fill up the current input buffer when a new buffer has arrived */
    if(p_private->isNewBuffer) {
        p_private->inputCurrBuffer = pInputBuffer->pBuffer;
        p_private->inputCurrLength = pInputBuffer->nFilledLen;
        p_private->isNewBuffer = 0;
        DBUG_MSG("[VENC-%d] New Buffer (FilledLen: %u)", p_private->nVpuInstance, pInputBuffer->nFilledLen);
    }

    if (pInputBuffer->nFlags & OMX_BUFFERFLAG_EOS) {
        p_private->bSeenEOS = OMX_TRUE;
    }

    DBUG_MSG("[VENC-%d] BufferMgmtCallback IN inLen = %u, Flags = 0x%x, Timestamp = %lld", p_private->nVpuInstance,pInputBuffer->nFilledLen, pInputBuffer->nFlags, pInputBuffer->nTimeStamp);
    pOutputBuffer->nFilledLen = 0;
    pOutputBuffer->nOffset = 0;

    if (p_private->isEncError == OMX_TRUE) {
        ALOGI("[VENC-%d] Restore Encode-Error", p_private->nVpuInstance);

        if (OMX_ErrorNone != omx_videoenc_component_LibInit(p_private))
            goto ERR_PROCESS;

        p_private->isEncError = OMX_FALSE;
        //p_private->iConfigDataFlag = OMX_FALSE;
    }

    if (p_private->isVPUClosed == OMX_TRUE) {
        pInputBuffer->nFilledLen = 0;
        ALOGE("[VENC-%d] Vpu already is closed because of Error!!", p_private->nVpuInstance);
        return;
    }

    while (!nOutputFilled) {
        if (p_private->isFirstBuffer) {
            tsem_down(p_private->avCodecSyncSem);
            p_private->isFirstBuffer = 0;
        }

        /* send the first output buffer as vol header in case of m4v format */
        if (p_private->iConfigDataFlag == OMX_FALSE) {
            p_private->gsVEncSeqHeader.m_SeqHeaderBuffer[PA] = (codec_addr_t)NULL;
            p_private->gsVEncSeqHeader.m_SeqHeaderBuffer[VA] = (codec_addr_t)NULL;
            p_private->gsVEncSeqHeader.m_iSeqHeaderBufferSize = 0;

            if (p_private->video_coding_type == OMX_VIDEO_CodingMPEG4) {
                ret = p_private->gspfVEnc(VENC_SEQ_HEADER, NULL,
                                          &p_private->gsVEncSeqHeader,
                                          NULL, p_private->pVpuInstance);
                if (ret < 0) {
                    ALOGE("[VENC-%d][Err:%ld] VENC_SEQ_HEADER failed !!", p_private->nVpuInstance, -ret);

                    p_private->isEncError = OMX_TRUE;
                    goto ERR_PROCESS;
                }

                nEncOutBuffAddr = (OMX_U32)((uintptr_t)p_private->gsVEncSeqHeader.m_pSeqHeaderOut);
                pOutputBuffer->nFilledLen = p_private->gsVEncSeqHeader.m_iSeqHeaderOutSize;
                (void)memcpy(pOutputBuffer->pBuffer, p_private->gsVEncSeqHeader.m_pSeqHeaderOut, pOutputBuffer->nFilledLen);
                pOutputBuffer->nTimeStamp = 0;//pInputBuffer->nTimeStamp;
                pOutputBuffer->nFlags |= OMX_BUFFERFLAG_CODECCONFIG;
                pOutputBuffer->nFlags |= OMX_BUFFERFLAG_ENDOFFRAME;

                DBUG_MSG("[VENC-%d]  VOL Header length = %u", p_private->nVpuInstance, pOutputBuffer->nFilledLen);
            } else if (p_private->video_coding_type == OMX_VIDEO_CodingAVC) {
                ret = p_private->gspfVEnc(VENC_SEQ_HEADER, NULL,
                                          &p_private->gsVEncSeqHeader,
                                          NULL, p_private->pVpuInstance);
                if (ret < 0) {
                    ALOGE("[VENC-%d][Err:%ld] VENC_SEQ_HEADER failed !!", p_private->nVpuInstance, (-ret));
                    p_private->isEncError = OMX_TRUE;
                    goto ERR_PROCESS;
                }
                DBUG_MSG("[VENC-%d] SPS - %d !!", p_private->nVpuInstance, p_private->gsVEncSeqHeader.m_iSeqHeaderOutSize);

                nEncOutBuffAddr = (OMX_U32)((uintptr_t)p_private->gsVEncSeqHeader.m_pSeqHeaderOut);
                pOutputBuffer->nFilledLen = p_private->gsVEncSeqHeader.m_iSeqHeaderOutSize;
                pOutputBuffer->nTimeStamp = 0; //pInputBuffer->nTimeStamp;
                pOutputBuffer->nFlags |= OMX_BUFFERFLAG_CODECCONFIG;
                pOutputBuffer->nFlags |= OMX_BUFFERFLAG_ENDOFFRAME;

                ret = p_private->gspfVEnc(VENC_SEQ_HEADER, NULL,
                                          &p_private->gsVEncSeqHeader,
                                          NULL, p_private->pVpuInstance);
                if (ret < 0) {
                    ALOGE("[VENC-%d][Err:%ld] VENC_SEQ_HEADER failed !!", p_private->nVpuInstance, (-ret));
                    p_private->isEncError = OMX_TRUE;
                    goto ERR_PROCESS;
                }

                DBUG_MSG("[VENC-%d] sequence header length (%d)",
                         p_private->nVpuInstance, p_private->gsVEncSeqHeader.m_iSeqHeaderOutSize);

                pOutputBuffer->nFilledLen += p_private->gsVEncSeqHeader.m_iSeqHeaderOutSize;
                (void)memcpy(pOutputBuffer->pBuffer, p_private->gsVEncSeqHeader.m_pSeqHeaderOut, pOutputBuffer->nFilledLen);

                DBUG_MSG("[VENC-%d] CodecConfig size =  %u", p_private->nVpuInstance, pOutputBuffer->nFilledLen);
            }

            bCodecConfigData = p_private->iConfigDataFlag = OMX_TRUE;
            p_private->nFrameCount = 0;
        } else {
            if (inPort->sVideoParam.eColorFormat == OMX_COLOR_FormatYUV420SemiPlanar)
                p_private->gsVEncInput.m_bCbCrInterleaved = 1;
            else
                p_private->gsVEncInput.m_bCbCrInterleaved = 0;

            // gsVEncInput.m_pInputY
            GetInputYuv (p_private, pInputBuffer);

        #ifdef CHANGE_BITRATE_TEST
            OMX_U32 change_period = 100;
            if (total_count == 5000) {
                total_count = 0;
            }

            switch (total_count % 1000) {
            case change_period:
                p_private->stBitRateType.nEncodeBitrate = 512 * 1024;
                break;
            case change_period * 2:
                p_private->stBitRateType.nEncodeBitrate = 384 * 1024;
                break;
            case change_period * 3:
                p_private->stBitRateType.nEncodeBitrate = 256 * 1024;
                break;
            case change_period * 4:
                p_private->stBitRateType.nEncodeBitrate = 192 * 1024;
                break;
            case change_period * 5:
                p_private->stBitRateType.nEncodeBitrate = 162 * 1024;
                break;
            case change_period * 6:
                p_private->stBitRateType.nEncodeBitrate = 128 * 1024;
                break;
            case change_period * 7:
                p_private->stBitRateType.nEncodeBitrate = 96 * 1024;
                break;
            case change_period * 8:
                p_private->stBitRateType.nEncodeBitrate = 64 * 1024;
                break;
            case change_period * 9:
                p_private->stBitRateType.nEncodeBitrate = 32 * 1024;
                break;
            default:
                break;
            }
        #endif

        #ifdef CHANGE_FPS_TEST
            OMX_U32 change_fps_period = 1000;
            if (total_count == 5000)
                total_count = 0;

            switch (total_count % 10000) {
            case change_fps_period:
                p_private->stFrameRateType.xEncodeFramerate = 1 * (1 << 16);
                break;
            case change_fps_period * 2:
                p_private->stFrameRateType.xEncodeFramerate = 5 * (1 << 16);
                break;
            case change_fps_period * 3:
                p_private->stFrameRateType.xEncodeFramerate = 10 * (1 << 16);
                break;
            case change_fps_period * 4:
                p_private->stFrameRateType.xEncodeFramerate = 15 * (1 << 16);
                break;
            default:
                break;
            }
        #endif

        #ifdef REQUEST_INTRAR_EFRESH_TEST
            if (total_count % 27 == 0) {
                p_private->stIntraRefreshVop.IntraRefreshVOP = OMX_TRUE;
            }
        #endif
            total_count++;

        #ifdef ENABLE_RATE_CONTROL
          #ifdef CHANGE_QP_TEST
            OMX_U32 mQp;

            OMX_U32 change_fps_period = 25;
            if (total_count == 5000)
                total_count = 0;

            if (total_count % 100 <= change_fps_period) {
                mQp = 0;
            } else if (total_count % 100 <= change_fps_period * 2) {
                mQp = 10;
            } else if (total_count % 100 <= change_fps_period * 3) {
                mQp = 35;
            } else {
                mQp = 50;
            }

            p_private->gsVEncInput.m_iChangeRcParamFlag = 0;
            p_private->gsVEncInput.m_iQuantParam = mQp;
            if (mQp == 0)
                p_private->gsVEncInput.m_iChangeRcParamFlag = (0x20 | 0x1);
            else
                p_private->stIntraRefreshVop.IntraRefreshVOP = OMX_TRUE;

            if (mQp != 0 && p_private->qp_value != mQp) {
                p_private->gsVEncInput.m_iChangeRcParamFlag = (0x10 | 0x1);
                p_private->qp_value = mQp;
                ALOGD("QP- Change(%d)", p_private->qp_value);
            }
          #else
            #ifdef REMOVE_RC_AUTO_SKIP
            p_private->gsVEncInput.m_iChangeRcParamFlag = (0x20 | 0x1);
            #endif
          #endif
            if (p_private->stBitRateType.nEncodeBitrate != 0 &&
                p_private->stBitRateType.nEncodeBitrate != outPort->sPortParam.format.video.nBitrate)
            {
                outPort->sPortParam.format.video.nBitrate = p_private->stBitRateType.nEncodeBitrate;

                p_private->gsVEncInput.m_iChangeRcParamFlag |= (0x2 | 0x1);
                p_private->gsVEncInput.m_iChangeTargetKbps = (outPort->sPortParam.format.video.nBitrate / 1024);
                ALOGD("[VENC-%d] Bitrate- Change(%d)", p_private->nVpuInstance, p_private->gsVEncInput.m_iChangeTargetKbps);
            }
            else if (p_private->stFrameRateType.xEncodeFramerate != 0 &&
                     p_private->stFrameRateType.xEncodeFramerate != inPort->sPortParam.format.video.xFramerate)
            {
                inPort->sPortParam.format.video.xFramerate = p_private->stFrameRateType.xEncodeFramerate;

                p_private->gsVEncInput.m_iChangeRcParamFlag |= (0x4 | 0x1);
                p_private->gsVEncInput.m_iChangeFrameRate = (inPort->sPortParam.format.video.xFramerate/(1 << 16));
                ALOGD("[VENC-%d] FrameRate- Change(%d)", p_private->nVpuInstance, p_private->gsVEncInput.m_iChangeFrameRate);
            }
        #endif

            if (p_private->stIntraRefreshVop.IntraRefreshVOP) {
                p_private->gsVEncInput.request_IntraFrame = 1;
                p_private->stIntraRefreshVop.IntraRefreshVOP = OMX_FALSE;
                ALOGD("[VENC-%d] IntraRefreshVOP", p_private->nVpuInstance);
            } else {
                p_private->gsVEncInput.request_IntraFrame = 0;
            }

            ret = p_private->gspfVEnc(VENC_ENCODE, NULL,
                                      &p_private->gsVEncInput,
                                      &p_private->gsVEncOutput,
                                      p_private->pVpuInstance);
            if (ret < 0) {
                pInputBuffer->nFilledLen = 0;
                p_private->isNewBuffer = 1;
                p_private->isEncError = OMX_TRUE;

                ALOGE("[VENC-%d][Err:%ld] VENC_ENCODE failed !!", p_private->nVpuInstance, (-ret));
                goto ERR_PROCESS;
            }

            DBUG_MSG("[VENC-%d] Enc Done %d => 0x%x", p_private->nVpuInstance,
                    p_private->gsVEncOutput.m_iBitstreamOutSize, (OMX_U32)p_private->gsVEncInput.m_pInputY);

            nEncOutBuffAddr = (OMX_U32)((uintptr_t)p_private->gsVEncOutput.m_pBitstreamOut);
            (void)memcpy(pOutputBuffer->pBuffer, p_private->gsVEncOutput.m_pBitstreamOut,
                   p_private->gsVEncOutput.m_iBitstreamOutSize);
            output_len = p_private->gsVEncOutput.m_iBitstreamOutSize;

            /* I-Frame (sync.frame) */
            if (p_private->gsVEncOutput.m_iPicType == VENC_PIC_TYPE_I && output_len > 0) {
                DBUG_MSG("[VENC-%d] I-Frame for Sync (Frm_size: %u)", p_private->nVpuInstance, output_len);

                // This flag is set when the buffer content contains a coded sync frame
                pOutputBuffer->nFlags |= OMX_BUFFERFLAG_SYNCFRAME;
            }
            /* Attach the end of frame flag while sending out the last piece of output buffer */
            pOutputBuffer->nFlags |= OMX_BUFFERFLAG_ENDOFFRAME;


            internalOutputFilled = 1;
            nLen += p_private->inputCurrLength;

            if (nLen >= 0 && internalOutputFilled) {
                p_private->inputCurrBuffer += nLen;
                p_private->inputCurrLength -= nLen;
                pInputBuffer->nFilledLen -= nLen;

                /* Buffer is fully consumed. Request for new Input Buffer */
                if (pInputBuffer->nFilledLen == 0) {
                    p_private->isNewBuffer = 1;
                }

                pOutputBuffer->nFilledLen = output_len;
                pOutputBuffer->nTimeStamp = pInputBuffer->nTimeStamp;
            } else {
                pInputBuffer->nFilledLen = 0;
                p_private->isNewBuffer = 1;
            }
        }

        if ((pOutputBuffer->nFilledLen > 0)  && (DEBUG_ON == 1)){
            OMX_U8 *p = (OMX_U8 *)pOutputBuffer->pBuffer;
            p_private->nFrameCount++;
            DBUG_MSG("[VENC-%d][frame-count:%u] e-out frame flags = 0x%08x, TS = %8d, "
                     "Data[%d] = %#x, %#x, %#x, %#x, %#x", p_private->nVpuInstance,
                     p_private->nFrameCount, pOutputBuffer->nFlags,
                     (OMX_S32)(pOutputBuffer->nTimeStamp / 1000), pOutputBuffer->nFilledLen,
                     p[0], p[1], p[2], p[3], p[4]);
        }

    #ifdef FOR_V2IP
        if (pOutputBuffer->pOutputPortPrivate != NULL) {
            TCC_PLATFORM_PRIVATE_PMEM_INFO *pmemInfoPtr =
                (TCC_PLATFORM_PRIVATE_PMEM_INFO *) pOutputBuffer->pOutputPortPrivate;

            if (bCodecConfigData) {
                pmemInfoPtr->offset[0] = (OMX_U32)((uintptr_t)vpu_getSeqHeaderPhyAddr(
                                (OMX_U8 *)((uintptr_t)nEncOutBuffAddr), VA, p_private->pVpuInstance));
            } else {
                pmemInfoPtr->offset[0] = (OMX_U32)((uintptr_t)vpu_getStreamOutPhyAddr(
                                (OMX_U8 *)((uintptr_t)nEncOutBuffAddr), VA, p_private->pVpuInstance));
            }
            pmemInfoPtr->offset[1] = pmemInfoPtr->offset[2] = 0;
        }
    #endif

        if (p_private->bSeenEOS) {
            pOutputBuffer->nFlags |= OMX_BUFFERFLAG_EOS;
        }
        nOutputFilled = 1;
    }

    return;

ERR_PROCESS:
    pInputBuffer->nFilledLen = 0;
    pOutputBuffer->nFilledLen = 0;

    if (p_private->isVPUClosed == OMX_FALSE) {
        ret = _encoder_close(p_private, OMX_FALSE);
        if (0) //p_private->isEncError != OMX_TRUE)
        {
            (*(p_private->callbacks->EventHandler))(openmaxStandComp, p_private->callbackData,
                                                    OMX_EventError, OMX_ErrorHardware, 0, NULL);
        }
    }
    return;
}

OMX_ERRORTYPE omx_videoenc_component_SetParameter(
    OMX_IN  OMX_HANDLETYPE hComponent,
    OMX_IN  OMX_INDEXTYPE nParamIndex,
    OMX_IN  OMX_PTR params)
{
    OMX_ERRORTYPE eError = OMX_ErrorNone;

    /* Check which structure we are being fed and make control its header */
    OMX_COMPONENTTYPE *openmaxStandComp = hComponent;
    videoenc_private_t* p_private = openmaxStandComp->pComponentPrivate;
    omx_base_video_PortType *port;
    if (params == NULL) {
        return OMX_ErrorBadParameter;
    }
    DBUG_MSG("[VENC-%d]  IN  Setting parameter %#x", p_private->nVpuInstance, nParamIndex);
    OMX_U64 param_index = (OMX_U64)nParamIndex; // remove enum type mismatch warning

    switch (param_index) {
        case OMX_IndexParamPortDefinition:
        {
            eError = omx_base_component_SetParameter(hComponent, nParamIndex, params);
            if(eError == OMX_ErrorNone) {
                OMX_PARAM_PORTDEFINITIONTYPE *pPortDef = (OMX_PARAM_PORTDEFINITIONTYPE*)params;
                UpdateFrameSize (openmaxStandComp);
                port = (omx_base_video_PortType *)p_private->ports[pPortDef->nPortIndex];
                port->sVideoParam.eColorFormat = port->sPortParam.format.video.eColorFormat;
            }
        } break;

        case OMX_IndexParamVideoPortFormat:
        {
            OMX_VIDEO_PARAM_PORTFORMATTYPE *pVideoPortFormat =
                (OMX_VIDEO_PARAM_PORTFORMATTYPE *)params;

            /* Check Structure Header and verify component state */
            eError = omx_base_component_ParameterSanityCheck(
                    hComponent, pVideoPortFormat->nPortIndex, pVideoPortFormat,
                    sizeof(OMX_VIDEO_PARAM_PORTFORMATTYPE));

            if (eError != OMX_ErrorNone) {
                ALOGE("[VENC-%d][SetParam] Parameter Check Error=%x", p_private->nVpuInstance, eError);
                break;
            }
            if (pVideoPortFormat->nPortIndex > 1) {
                return OMX_ErrorUndefined;
            }
            if (pVideoPortFormat->nIndex > 2) {
                return OMX_ErrorNoMore;
            }

            if (pVideoPortFormat->nPortIndex == 0) {
                ALOGD("[VENC-%d][SetParam] color-format: %d",
                        p_private->nVpuInstance, pVideoPortFormat->eColorFormat);
                if (pVideoPortFormat->eCompressionFormat != OMX_VIDEO_CodingUnused ||
                    ((pVideoPortFormat->nIndex == 0 &&
                      pVideoPortFormat->eColorFormat != OMX_COLOR_FormatYUV420Planar) ||
                     (pVideoPortFormat->nIndex == 1 &&
                      pVideoPortFormat->eColorFormat != OMX_COLOR_FormatYUV420SemiPlanar)
                )) {
                    return OMX_ErrorUndefined;
                }
            }

            port = (omx_base_video_PortType *)p_private->ports[pVideoPortFormat->nPortIndex];
            (void)memcpy(&port->sVideoParam, pVideoPortFormat, sizeof(OMX_VIDEO_PARAM_PORTFORMATTYPE));
            p_private->ports[pVideoPortFormat->nPortIndex]->sPortParam.format.video.eColorFormat =
                port->sVideoParam.eColorFormat;
        } break;

        case OMX_IndexParamStandardComponentRole:
        {
            OMX_PARAM_COMPONENTROLETYPE *pComponentRole = (OMX_PARAM_COMPONENTROLETYPE *)params;
            if (!strcmp((char *)pComponentRole->cRole, VIDEO_ENC_MPEG4_ROLE)) {
                p_private->video_coding_type = OMX_VIDEO_CodingMPEG4;
                SetInternalVideoParameters(openmaxStandComp);
            }else if (!strcmp((char *)pComponentRole->cRole, VIDEO_ENC_H264_ROLE)) {
                p_private->video_coding_type = OMX_VIDEO_CodingAVC;
                SetInternalVideoParameters(openmaxStandComp);
            }else if (!strcmp((char *)pComponentRole->cRole, VIDEO_ENC_H263_ROLE)) {
                p_private->video_coding_type = OMX_VIDEO_CodingH263;
                SetInternalVideoParameters(openmaxStandComp);
            } else {
                eError =  OMX_ErrorBadParameter;
            }
        } break;

        case OMX_IndexParamVideoMpeg4:
        {
            OMX_VIDEO_PARAM_MPEG4TYPE *pVideoMpeg4 = (OMX_VIDEO_PARAM_MPEG4TYPE *)params;
            (void)memcpy(&p_private->pVideoMpeg4, pVideoMpeg4, sizeof(OMX_VIDEO_PARAM_MPEG4TYPE));
            ALOGI("[VENC-%d][SetParam][MPEG4] profile %d, level %d", p_private->nVpuInstance,
                    p_private->pVideoMpeg4.eProfile, p_private->pVideoMpeg4.eLevel);
        } break;

        case OMX_IndexParamVideoAvc:
        {
            OMX_VIDEO_PARAM_AVCTYPE *pVideoAvc = (OMX_VIDEO_PARAM_AVCTYPE *)params;
            (void)memcpy(&p_private->pVideoAvc, pVideoAvc, sizeof(OMX_VIDEO_PARAM_AVCTYPE));
            ALOGI("[VENC-%d][SetParam][H264] profile %d, level %d", p_private->nVpuInstance,
                    p_private->pVideoAvc.eProfile, p_private->pVideoAvc.eLevel);
        } break;

        case OMX_IndexParamVideoH263:
        {
            OMX_VIDEO_PARAM_H263TYPE *pVideoH263 = (OMX_VIDEO_PARAM_H263TYPE *)params;
            (void)memcpy(&p_private->pVideoH263, pVideoH263, sizeof(OMX_VIDEO_PARAM_H263TYPE));
            ALOGI("[VENC-%d][SetParam][H263] profile %d, level %d", p_private->nVpuInstance,
                    p_private->pVideoH263.eProfile, p_private->pVideoH263.eLevel);
        } break;

        case OMX_IndexParamVideoProfileLevelCurrent:
        {
            (void)memcpy(&p_private->sProfileLevel, params, sizeof(OMX_VIDEO_PARAM_PROFILELEVELTYPE));
        } break;

        case OMX_IndexConfigVideoFramerate:
        {
            (void)memcpy(&p_private->stFrameRateType, params, sizeof(OMX_CONFIG_FRAMERATETYPE));
            ALOGI("[VENC-%d][SetParam] OMX_IndexConfigVideoFramerate = %lu [%lu]", p_private->nVpuInstance,
                    p_private->stFrameRateType.xEncodeFramerate,
                    (p_private->stFrameRateType.xEncodeFramerate / (1 << 16)));
        } break;

        case OMX_IndexConfigVideoBitrate:
        {
            (void)memcpy(&p_private->stBitRateType, params, sizeof(OMX_VIDEO_CONFIG_BITRATETYPE));
            ALOGI("[VENC-%d][SetParam] OMX_IndexConfigVideoBitrate = %lu", p_private->nVpuInstance,
                    p_private->stBitRateType.nEncodeBitrate);
        } break;

        case OMX_IndexConfigCommonRotate:
        {
            OMX_CONFIG_ROTATIONTYPE *pVideoRotation = (OMX_CONFIG_ROTATIONTYPE *)params;
            (void)memcpy(&p_private->stRotationType, pVideoRotation, sizeof(OMX_CONFIG_ROTATIONTYPE));
        } break;

        case OMX_IndexParamVideoErrorCorrection:
        {
            OMX_VIDEO_PARAM_ERRORCORRECTIONTYPE *pVideoErrCorr = (OMX_VIDEO_PARAM_ERRORCORRECTIONTYPE *)params;
            (void)memcpy(&p_private->stErrCorrectionType, pVideoErrCorr, sizeof(OMX_VIDEO_PARAM_ERRORCORRECTIONTYPE));
        } break;

        case OMX_IndexParamVideoBitrate:
            break;

        case OMX_IndexParamVideoQuantization:
        {
            OMX_VIDEO_PARAM_QUANTIZATIONTYPE *pVideoQuant = (OMX_VIDEO_PARAM_QUANTIZATIONTYPE *)params;
            (void)memcpy(&p_private->stQuantType, pVideoQuant, sizeof(OMX_VIDEO_PARAM_QUANTIZATIONTYPE));
        } break;

        case OMX_IndexParamVideoVBSMC:
        {
            OMX_VIDEO_PARAM_VBSMCTYPE *pVideoBlock = (OMX_VIDEO_PARAM_VBSMCTYPE *)params;
            (void)memcpy(&p_private->stVBSMCType, pVideoBlock, sizeof(OMX_VIDEO_PARAM_VBSMCTYPE));
        } break;

        case OMX_IndexParamVideoMotionVector:
        {
            OMX_VIDEO_PARAM_MOTIONVECTORTYPE *pstMotionVectorType = (OMX_VIDEO_PARAM_MOTIONVECTORTYPE *)params;
            (void)memcpy(&p_private->stMotionVectorType, pstMotionVectorType, sizeof(OMX_VIDEO_PARAM_MOTIONVECTORTYPE));
        } break;

        case OMX_IndexParamVideoIntraRefresh:
        {
            OMX_VIDEO_PARAM_INTRAREFRESHTYPE *pstIntraRefreshType = (OMX_VIDEO_PARAM_INTRAREFRESHTYPE *)params;
            (void)memcpy(&p_private->stIntraRefreshType, pstIntraRefreshType, sizeof(OMX_VIDEO_PARAM_INTRAREFRESHTYPE));
        } break;

        case OMX_IndexConfigVideoIntraVOPRefresh:
        {
            OMX_CONFIG_INTRAREFRESHVOPTYPE *pstIntraRefreshVOPType = (OMX_CONFIG_INTRAREFRESHVOPTYPE *)params;
            (void)memcpy(&p_private->stIntraRefreshVop, pstIntraRefreshVOPType, sizeof(OMX_CONFIG_INTRAREFRESHVOPTYPE));
        } break;

        case OMX_IndexParamQFactor:
            break;

        default: /*Call the base component function*/
            eError = omx_base_component_SetParameter(hComponent, nParamIndex, params);
            break;
    }

    if (eError != OMX_ErrorNone) {
        ALOGE("[VENC-%d][SetParam] ERROR: nParamIndex = 0x%x, error(%#lx)",
                p_private->nVpuInstance, nParamIndex, (OMX_S32)eError);
    }

    return eError;
}

OMX_ERRORTYPE omx_videoenc_component_GetParameter(
    OMX_IN    OMX_HANDLETYPE  hComponent,
    OMX_IN    OMX_INDEXTYPE   nParamIndex,
    OMX_INOUT OMX_PTR         params)
{
    omx_base_video_PortType *port;
    OMX_ERRORTYPE eError = OMX_ErrorNone;

    OMX_COMPONENTTYPE *openmaxStandComp = hComponent;
    videoenc_private_t* p_private = openmaxStandComp->pComponentPrivate;
    if (params == NULL) {
        return OMX_ErrorBadParameter;
    }

    DBUG_MSG("[VENC-%d] IN  Getting parameter 0x%x", p_private->nVpuInstance, nParamIndex);
    OMX_U64 param_index = (OMX_U64)nParamIndex; // remove enum type mismatch warning

    /* Check which structure we are being fed and fill its header */
    switch (param_index) {
    case OMX_IndexParamVideoInit:
        eError = checkHeader(params, sizeof(OMX_PORT_PARAM_TYPE));
        if (eError != OMX_ErrorNone) {
            break;
        }
        (void)memcpy(params, &p_private->sPortTypesParam[OMX_PortDomainVideo], sizeof(OMX_PORT_PARAM_TYPE));
        break;

    case OMX_IndexParamVideoPortFormat:
    {
        OMX_VIDEO_PARAM_PORTFORMATTYPE *pVideoPortFormat;
        pVideoPortFormat = params;

        eError = checkHeader(params, sizeof(OMX_VIDEO_PARAM_PORTFORMATTYPE));
        if (eError != OMX_ErrorNone) {
            break;
        }
        if (pVideoPortFormat->nPortIndex > 1) {
            return OMX_ErrorUndefined;
        }
        if (pVideoPortFormat->nIndex > 2) {
            return OMX_ErrorNoMore;
        }

        if (pVideoPortFormat->nPortIndex == 0) {
            pVideoPortFormat->eCompressionFormat = OMX_VIDEO_CodingUnused;
            if (pVideoPortFormat->nIndex == 0) {
                pVideoPortFormat->eColorFormat = OMX_COLOR_FormatYUV420Planar;
                ALOGD("[VENC-%d][GetParameter] OMX_COLOR_FormatYUV420Planar", p_private->nVpuInstance);
            } else if (pVideoPortFormat->nIndex == 1) {
                pVideoPortFormat->eColorFormat = OMX_COLOR_FormatYUV420SemiPlanar;
                ALOGD("[VENC-%d][GetParameter] OMX_COLOR_FormatYUV420SemiPlanar", p_private->nVpuInstance);
            } else {
            }
        } else {
            port = (omx_base_video_PortType *)p_private->ports[pVideoPortFormat->nPortIndex];
            (void)memcpy(pVideoPortFormat, &port->sVideoParam, sizeof(OMX_VIDEO_PARAM_PORTFORMATTYPE));
        }
    } break;

    case OMX_IndexParamVideoMpeg4:
    {
        OMX_VIDEO_PARAM_MPEG4TYPE *pVideoMpeg4 = (OMX_VIDEO_PARAM_MPEG4TYPE *)params;
        (void)memcpy(pVideoMpeg4, &p_private->pVideoMpeg4, sizeof(OMX_VIDEO_PARAM_MPEG4TYPE));
        ALOGI("[VENC-%d][GetParameter][MPEG4] profile %d, level %d",
                p_private->nVpuInstance, pVideoMpeg4->eProfile, pVideoMpeg4->eLevel);
    } break;

    case OMX_IndexParamVideoAvc:
    {
        OMX_VIDEO_PARAM_AVCTYPE *pVideoAvc = (OMX_VIDEO_PARAM_AVCTYPE *)params;
        (void)memcpy(pVideoAvc, &p_private->pVideoAvc, sizeof(OMX_VIDEO_PARAM_AVCTYPE));
        ALOGI("[VENC-%d][GetParameter][H264] profile %d, level %d",
                p_private->nVpuInstance, pVideoAvc->eProfile, pVideoAvc->eLevel);
    } break;

    case OMX_IndexParamVideoH263:
    {
        OMX_VIDEO_PARAM_H263TYPE *pVideoH263 = (OMX_VIDEO_PARAM_H263TYPE *)params;
        (void)memcpy(pVideoH263, &p_private->pVideoH263, sizeof(OMX_VIDEO_PARAM_H263TYPE));
        ALOGI("[VENC-%d][GetParameter][H263] profile %d, level %d",
                p_private->nVpuInstance, pVideoH263->eProfile, pVideoH263->eLevel);
    } break;

    case OMX_IndexParamStandardComponentRole:
    {
        OMX_PARAM_COMPONENTROLETYPE *pComponentRole = (OMX_PARAM_COMPONENTROLETYPE *)params;
        eError = checkHeader(params, sizeof(OMX_PARAM_COMPONENTROLETYPE));
        if (eError != OMX_ErrorNone) {
            break;
        }
        if (p_private->video_coding_type == OMX_VIDEO_CodingMPEG4) {
            (void)strncpy((char *)pComponentRole->cRole, VIDEO_ENC_MPEG4_ROLE, sizeof(VIDEO_ENC_MPEG4_ROLE));
        } else if (p_private->video_coding_type == OMX_VIDEO_CodingAVC) {
            (void)strncpy((char *)pComponentRole->cRole, VIDEO_ENC_H264_ROLE, sizeof(VIDEO_ENC_H264_ROLE));
        } else if (p_private->video_coding_type == OMX_VIDEO_CodingH263) {
            (void)strncpy((char *)pComponentRole->cRole, VIDEO_ENC_H263_ROLE, sizeof(VIDEO_ENC_H263_ROLE));
        } else {
            (void)strncpy((char *)pComponentRole->cRole,"\0", sizeof("\0"));
        }
    } break;

    case OMX_IndexParamVideoProfileLevelCurrent:
    {
        (void)memcpy(params, &p_private->sProfileLevel, sizeof(OMX_VIDEO_PARAM_PROFILELEVELTYPE));
    } break;

    case OMX_IndexConfigVideoFramerate:
    {
        (void)memcpy(params, &p_private->stFrameRateType, sizeof(OMX_CONFIG_FRAMERATETYPE));
    } break;

    case OMX_IndexConfigCommonRotate:
    {
        OMX_CONFIG_ROTATIONTYPE *pVideoRotation = (OMX_CONFIG_ROTATIONTYPE *)params;
        (void)memcpy(pVideoRotation, &p_private->stRotationType, sizeof(OMX_CONFIG_ROTATIONTYPE));
    } break;

    case OMX_IndexParamVideoErrorCorrection:
    {
        OMX_VIDEO_PARAM_ERRORCORRECTIONTYPE* pVideoErrCorr = (OMX_VIDEO_PARAM_ERRORCORRECTIONTYPE *)params;
        (void)memcpy(pVideoErrCorr, &p_private->stErrCorrectionType, sizeof(OMX_VIDEO_PARAM_ERRORCORRECTIONTYPE));
    } break;

    case OMX_IndexParamVideoBitrate:
    {
        OMX_VIDEO_PARAM_BITRATETYPE *pVideoRateControl = (OMX_VIDEO_PARAM_BITRATETYPE *)params;
        (void)memcpy(pVideoRateControl, &p_private->stBitrateType, sizeof(OMX_VIDEO_PARAM_BITRATETYPE));
    } break;

    case OMX_IndexParamVideoQuantization:
    {
        OMX_VIDEO_PARAM_QUANTIZATIONTYPE *pVideoQuant = (OMX_VIDEO_PARAM_QUANTIZATIONTYPE *)params;
        (void)memcpy(pVideoQuant, &p_private->stQuantType, sizeof(OMX_VIDEO_PARAM_QUANTIZATIONTYPE));
    } break;

    case OMX_IndexParamVideoVBSMC:
    {
        OMX_VIDEO_PARAM_VBSMCTYPE *pVideoBlock = (OMX_VIDEO_PARAM_VBSMCTYPE *)params;
        (void)memcpy(pVideoBlock, &p_private->stVBSMCType, sizeof(OMX_VIDEO_PARAM_VBSMCTYPE));
    } break;

    case OMX_IndexParamVideoMotionVector:
    {
        OMX_VIDEO_PARAM_MOTIONVECTORTYPE *pstMVType = (OMX_VIDEO_PARAM_MOTIONVECTORTYPE *)params;
        (void)memcpy(pstMVType, &p_private->stMotionVectorType, sizeof(OMX_VIDEO_PARAM_MOTIONVECTORTYPE));
    }
        break;

    case OMX_IndexParamVideoIntraRefresh:
    {
        OMX_VIDEO_PARAM_INTRAREFRESHTYPE *pstIntraRefreshType = (OMX_VIDEO_PARAM_INTRAREFRESHTYPE *)params;
        (void)memcpy(pstIntraRefreshType, &p_private->stIntraRefreshType, sizeof(OMX_VIDEO_PARAM_INTRAREFRESHTYPE));
    } break;

    case OMX_IndexConfigVideoIntraVOPRefresh:
    {
        OMX_CONFIG_INTRAREFRESHVOPTYPE *pstIntraRefreshVOPType = (OMX_CONFIG_INTRAREFRESHVOPTYPE *)params;
        (void)memcpy(pstIntraRefreshVOPType, &p_private->stIntraRefreshVop, sizeof(OMX_CONFIG_INTRAREFRESHVOPTYPE));
    } break;

    case OMX_IndexParamVideoProfileLevelQuerySupported:
    {
        OMX_VIDEO_PARAM_PROFILELEVELTYPE *profileLevel = (OMX_VIDEO_PARAM_PROFILELEVELTYPE *)params;
        VIDEO_PROFILE_LEVEL_TYPE *pProfileLevel = NULL;
        OMX_U32 nNumberOfProfiles = 0;

        if (profileLevel->nPortIndex != OMX_OUTPORT_INDEX) {
            ALOGE("[VENC-%d] Invalid port index: %lu", p_private->nVpuInstance, profileLevel->nPortIndex);
            return OMX_ErrorUnsupportedIndex;
        }

        ALOGI("[%s] [%d] format: %d", __func__, __LINE__,
                p_private->ports[OMX_OUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat);

        /* Choose table based on compression format */
        switch (p_private->ports[OMX_OUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat) {
            case OMX_VIDEO_CodingH263:
                pProfileLevel = SupportedH263ProfileLevels;
                nNumberOfProfiles = sizeof(SupportedH263ProfileLevels) / sizeof (VIDEO_PROFILE_LEVEL_TYPE);
                break;
            case OMX_VIDEO_CodingMPEG4:
                pProfileLevel = SupportedMPEG4ProfileLevels;
                nNumberOfProfiles = sizeof(SupportedMPEG4ProfileLevels) / sizeof (VIDEO_PROFILE_LEVEL_TYPE);
                break;
            case OMX_VIDEO_CodingAVC:
                pProfileLevel = SupportedAVCProfileLevels;
                nNumberOfProfiles = sizeof(SupportedAVCProfileLevels) / sizeof (VIDEO_PROFILE_LEVEL_TYPE);
                break;
            default:
                ALOGE("[VENC-%d] Invalid Format: %d", p_private->nVpuInstance,
                        p_private->ports[OMX_OUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat);
                return OMX_ErrorBadParameter;
        }

        if (profileLevel->nProfileIndex >= nNumberOfProfiles - 1) {
            ALOGI("[%s] [%d]", __func__, __LINE__);
            return OMX_ErrorNoMore;
        }

        profileLevel->eProfile = pProfileLevel[profileLevel->nProfileIndex].nProfile;
        profileLevel->eLevel = pProfileLevel[profileLevel->nProfileIndex].nLevel;

        ALOGI("profile:%lu, level:%lu", profileLevel->eProfile, profileLevel->eLevel);

        return OMX_ErrorNone;
    } break;

    default: /* Call the base component function */
        eError  = omx_base_component_GetParameter(hComponent, nParamIndex, params);
        break;
    }

    if (eError != OMX_ErrorNone) {
        ALOGE("[VENC-%d] ERROR %s :: nParamIndex = 0x%x, error(0x%lx)",
                p_private->nVpuInstance, __func__, nParamIndex, (OMX_S32)eError);
    }

    return OMX_ErrorNone;
}

OMX_ERRORTYPE omx_videoenc_component_MessageHandler(
    OMX_COMPONENTTYPE          *openmaxStandComp,
    internalRequestMessageType *message)
{
    videoenc_private_t* p_private = (videoenc_private_t*)openmaxStandComp->pComponentPrivate;
    OMX_ERRORTYPE err;

    DBUG_MSG("[VENC-%d] In %s", p_private->nVpuInstance, __func__);

    if (message->messageType == OMX_CommandStateSet) {
        if (message->messageParam == OMX_StateExecuting && p_private->state == OMX_StateIdle) {
            if (!p_private->avcodecReady) {
                err = omx_videoenc_component_LibInit(p_private);
                if (err != OMX_ErrorNone) {
                    return OMX_ErrorNotReady;
                }
                p_private->avcodecReady = OMX_TRUE;
            }
        } else if (message->messageParam == OMX_StateIdle && p_private->state == OMX_StateLoaded) {
            err = omx_videoenc_component_Initialize(openmaxStandComp);
            if (err != OMX_ErrorNone) {
                ALOGE("[VENC-%d][MessageHandler] comp. init. failed Error=%x", p_private->nVpuInstance, err);
                return err;
            }
        } else if (message->messageParam == OMX_StateLoaded && p_private->state == OMX_StateIdle) {
            err = omx_videoenc_component_Deinit(openmaxStandComp);
            if (err != OMX_ErrorNone) {
                ALOGE("[VENC-%d][MessageHandler] comp. deinit failed Error=%x", p_private->nVpuInstance, err);
                return err;
            }
        }
    }

    /* Execute the base message handling */
    err = omx_base_component_MessageHandler(openmaxStandComp, message);
    return err;
}

OMX_ERRORTYPE omx_videoenc_component_ComponentRoleEnum(
    OMX_IN OMX_HANDLETYPE hComponent, OMX_OUT OMX_U8 *cRole, OMX_IN OMX_U32 nIndex)
{
    if (nIndex == 0) {
        return OMX_ErrorUnsupportedIndex;
    }

    OMX_COMPONENTTYPE *openmaxStandComp = (OMX_COMPONENTTYPE *)hComponent;
    videoenc_private_t *p_private = (videoenc_private_t*)openmaxStandComp->pComponentPrivate;

    switch(p_private->video_coding_type){
        case OMX_VIDEO_CodingMPEG4:
          (void)strncpy((char *)cRole, VIDEO_ENC_MPEG4_ROLE, sizeof(VIDEO_ENC_MPEG4_ROLE));
          break;
        case OMX_VIDEO_CodingAVC:
          (void)strncpy((char *)cRole, VIDEO_ENC_H264_ROLE, sizeof(VIDEO_ENC_H264_ROLE));
          break;
        case OMX_VIDEO_CodingH263:
          (void)strncpy((char *)cRole, VIDEO_ENC_H263_ROLE, sizeof(VIDEO_ENC_H263_ROLE));
          break;
        default:
          ALOGW("%s: video codeing type(%lu) is not designated !!", __func__, p_private->video_coding_type);
          break;
	}
    return OMX_ErrorNone;
}

OMX_ERRORTYPE omx_videoenc_component_SetConfig(
    OMX_HANDLETYPE  hComponent,
    OMX_INDEXTYPE   nIndex,
    OMX_PTR         params)
{
    if (params == NULL) {
        return OMX_ErrorBadParameter;
    }

    OMX_COMPONENTTYPE *openmaxStandComp = (OMX_COMPONENTTYPE *)hComponent;
    videoenc_private_t *p_private = (videoenc_private_t*)openmaxStandComp->pComponentPrivate;
    omx_base_video_PortType *outPort = (omx_base_video_PortType *)p_private->ports[OMX_OUTPORT_INDEX];
    omx_base_video_PortType *inPort = (omx_base_video_PortType *)p_private->ports[OMX_INPORT_INDEX];

    ALOGT("Setting configuration %i", nIndex);
    OMX_U64 config_index = (OMX_U64)nIndex; // remove enum type mismatch warning

    /* Check which structure we are being fed and fill its header */
    switch (config_index) {
    case OMX_IndexVendorVideoExtraData:
    {
        OMX_VENDOR_EXTRADATATYPE* pExtradata = (OMX_VENDOR_EXTRADATATYPE *)params;
        if (pExtradata->nPortIndex > 1) {
            return OMX_ErrorBadPortIndex;
        }

        /** copy the extradata in the codec context private structure */
        p_private->extradata_size = (OMX_U32)pExtradata->nDataSize;
        if (p_private->extradata_size > 0) {
            if (p_private->extradata) {
                TCC_free(p_private->extradata);
            }
            p_private->extradata = (OMX_U8 *)TCC_malloc(pExtradata->nDataSize * sizeof(OMX_U8));
            //(void)memcpy(p_private->extradata, (OMX_U8 *)pExtradata->pData, pExtradata->nDataSize);
        } else {
            DBUG_MSG("[VENC-%d] extradata size is 0 !!", p_private->nVpuInstance);
        }
    } break;

    case OMX_IndexConfigVideoFramerate:
        (void)memcpy(&p_private->stFrameRateType, params, sizeof(OMX_CONFIG_FRAMERATETYPE));
        ALOGI("[VENC-%d] Framerate = %lu (%lu x %lu, %lu kbps, %lu fps)", p_private->nVpuInstance,
                p_private->stFrameRateType.xEncodeFramerate,
                outPort->sPortParam.format.video.nFrameWidth,
                outPort->sPortParam.format.video.nFrameHeight,
                outPort->sPortParam.format.video.nBitrate / 1024,
                inPort->sPortParam.format.video.xFramerate / (1 << 16));
        break;

    case OMX_IndexConfigVideoBitrate:
        (void)memcpy(&p_private->stBitRateType, params, sizeof(OMX_VIDEO_CONFIG_BITRATETYPE));
    #ifdef MAX_MIN_BPS_LIMITATION
        OMX_U32 max_Bps, min_Bps;
        OMX_U32 res_region = outPort->sPortParam.format.video.nFrameWidth \
                           * outPort->sPortParam.format.video.nFrameHeight;
        OMX_U32 curr_fps = inPort->sPortParam.format.video.xFramerate / (1 << 16);

        max_Bps = get_bitrate(outPort->sPortParam.format.video.nFrameWidth,
                              outPort->sPortParam.format.video.nFrameHeight, 0);
        min_Bps = get_bitrate(outPort->sPortParam.format.video.nFrameWidth,
                              outPort->sPortParam.format.video.nFrameHeight, 5);

        // temp
        if (res_region <= 160 * 120) {
            min_Bps = 128 * 1024;
        } else if (res_region <= 176 * 144) {
            min_Bps = 128 * 1024;
        } else if (res_region <= 320 * 240) {
            min_Bps = 192 * 1024;
        }

        if (curr_fps > 20) {
            max_Bps *= 2;
        }
        DBUG_MSG("[VENC-%d] OMX_IndexConfigVideoBitrate: rev = %d -> chg(%d ~ %d) = %d (orig: %d)",
                p_private->nVpuInstance,
                ((OMX_VIDEO_CONFIG_BITRATETYPE*)params)->nEncodeBitrate / 1024,
                min_Bps / 1024, max_Bps / 1024,
                p_private->stBitRateType.nEncodeBitrate / 1024,
                outPort->sPortParam.format.video.nBitrate / 1024);

        if (max_Bps < p_private->stBitRateType.nEncodeBitrate) {
            p_private->stBitRateType.nEncodeBitrate = max_Bps;
        } else if (min_Bps > p_private->stBitRateType.nEncodeBitrate) {
            p_private->stBitRateType.nEncodeBitrate = min_Bps;
        }
    #endif

        ALOGI("[VENC-%d] Bitrate = %lu kbps (%lu x %lu, %lu kbps, %lu fps)",
                p_private->nVpuInstance,
                p_private->stBitRateType.nEncodeBitrate / 1024,
                outPort->sPortParam.format.video.nFrameWidth,
                outPort->sPortParam.format.video.nFrameHeight,
                outPort->sPortParam.format.video.nBitrate / 1024,
                inPort->sPortParam.format.video.xFramerate / (1 << 16));
        break;

    case OMX_IndexConfigVideoIntraVOPRefresh:
    {
        OMX_CONFIG_INTRAREFRESHVOPTYPE *pConfig = (OMX_CONFIG_INTRAREFRESHVOPTYPE *)params;
        (void)memcpy(&p_private->stIntraRefreshVop, pConfig, sizeof(OMX_CONFIG_INTRAREFRESHVOPTYPE));
    } break;

#if 0
    case OMX_IndexConfigAndroidIntraRefresh:
    {
        const OMX_VIDEO_CONFIG_ANDROID_INTRAREFRESHTYPE *pConfig =
            (const OMX_VIDEO_CONFIG_ANDROID_INTRAREFRESHTYPE *)params;

        if (pConfig->nPortIndex != OMX_OUTPORT_INDEX) {
            return OMX_ErrorUndefined;
        }

    #if 0
        if (pConfig->nRefreshPeriod == 0) {
            mAIRMode = IVE_AIR_MODE_NONE;
            mAIRRefreshPeriod = 0;
        } else if (pConfig->nRefreshPeriod > 0) {
            mAIRMode = IVE_AIR_MODE_CYCLIC;
            mAIRRefreshPeriod = pConfig->nRefreshPeriod;
        }
    #endif

        return OMX_ErrorNone;
    } break;
#endif
    default: // delegate to superclass
        return omx_base_component_SetConfig(hComponent, nIndex, params);
    }

    return OMX_ErrorNone;
}

OMX_ERRORTYPE omx_videoenc_component_GetConfig(
    OMX_HANDLETYPE  hComponent,
    OMX_INDEXTYPE   nIndex,
    OMX_PTR         params)
{
    OMX_ERRORTYPE ret = OMX_ErrorNone;
    if (params != NULL) {
        DBUG_MSG(" Setting configuration %i", nIndex);
        /* Check which structure we are being fed and fill its header */
        ret = omx_base_component_GetConfig(hComponent, nIndex, params);
    } else {
        ret = OMX_ErrorBadParameter;
    }

    return ret;
}

OMX_ERRORTYPE omx_videoenc_component_GetExtensionIndex(
    OMX_IN  OMX_HANDLETYPE hComponent,
    OMX_IN  OMX_STRING cParameterName,
    OMX_OUT OMX_INDEXTYPE* pIndexType)
{
    (void)hComponent;
    DBUG_MSG("In  %s", __func__);

    if (strncmp(cParameterName, "OMX.tcc.index.config.videoextradata", sizeof("OMX.tcc.index.config.videoextradata")) == 0) {
        *pIndexType = (OMX_INDEXTYPE)OMX_IndexVendorVideoExtraData;
    } else {
        return OMX_ErrorBadParameter;
    }
    return OMX_ErrorNone;
}

OMX_ERRORTYPE OMX_ComponentInit(
    OMX_HANDLETYPE  openmaxStandComp,
    OMX_STRING      cCompontName)
{
    DBUG_MSG("OMX_ComponentInit");

    OMX_ERRORTYPE err = omx_videoenc_component_Constructor(openmaxStandComp, cCompontName);
    return err;
}
