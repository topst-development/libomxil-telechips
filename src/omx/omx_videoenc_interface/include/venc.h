// SPDX-License-Identifier: LGPL-2.1-or later
/****************************************************************************
 *   FileName    : vdec.h
 *   Description :
 ****************************************************************************
 *
 *   TCC Version 1.0
 *   Copyright (c) Telechips Inc.
 *   All rights reserved

This source code contains confidential information of Telechips.
Any unauthorized use without a written permission of Telechips including not limited to re-distribution in source or binary form is strictly prohibited.
This source code is provided "AS IS" and nothing contained in this source code shall constitute any express or implied warranty of any kind, including without limitation, any warranty of merchantability, fitness for a particular purpose or non-infringement of any patent, copyright or other third party intellectual property right. No warranty is made, express or implied, regarding the information's accuracy, completeness, or performance.
In no event shall Telechips be liable for any claim, damages or other liability arising from, out of or in connection with this source code or the use in the source code.
This source code is provided subject to the terms of a Mutual Non-Disclosure Agreement between Telechips and Company.
*
****************************************************************************/

#ifndef VENC_H_
#define VENC_H_

#include <stdint.h>
#include <glib/gprintf.h>

#define TCC_VPU_INCLUDE
#include <TCC_VPU_C7_CODEC.h>

#include <TCC_JPU_C6.h>
#define TCC_JPU_INCLUDE

#include <tcc_video_private.h>
#include <tcc_video_common.h>

//#include <cdk.h>
#define VCODEC_MAX      20

//#define PASS_BUFFER_TO_UPPER_LAYER
#ifdef PASS_BUFFER_TO_UPPER_LAYER
//Caution: Don't exceed the number of queue allocated in the queue.h [MAX_QUEUE_ELEMENTS 100].
#define VIDEO_ENC_BUFFER_COUNT 10 //20//150
#endif

#ifndef K_VA
#define K_VA 2
#endif

#define ENABLE_RATE_CONTROL
#define MULTI_SLICES_AVC
#define REMOVE_RC_AUTO_SKIP

#ifndef VPU_ENV_INIT_ERROR
#define VPU_ENV_INIT_ERROR      10000
#endif
#ifndef VPU_NOT_ENOUGH_MEM
#define VPU_NOT_ENOUGH_MEM      20000
#endif

#define VENC_PIC_TYPE_I 0
#define VENC_PIC_TYPE_P 1

#ifndef RETCODE_MULTI_CODEC_EXIT_TIMEOUT
#define RETCODE_MULTI_CODEC_EXIT_TIMEOUT  (99)
#endif

typedef struct venc_init_t
{
    // Encoding Info
    int m_iBitstreamFormat;
    int m_iPicWidth;                    //!< Width  : multiple of 16
    int m_iPicHeight;                   //!< Height : multiple of 16
    int m_iFrameRate;                   //!< Frame rate
    int m_iTargetKbps;                  //!< Target bit rate in Kbps. if 0, there will be no rate control,
                                        //!< and pictures will be encoded with a quantization parameter equal to quantParam

    int m_iKeyInterval;                 //!< Key interval : max 32767
    int m_iAvcFastEncoding;             //!< fast encoding for AVC( 0: default, 1: encode intra 16x16 only )

    int m_iBitstreamBufferSize;         //!< bitstream buffer size : multiple of 1024

    //! Options
    int m_iSliceMode;
    int m_iSliceSizeMode;
    int m_iSliceSize;
    int m_iWFDTranscoding;              //!< Transcoding for WFD Source Device

    int m_bNeedSwap;

    //!MJPEG Options
    int m_iMjpg_sourceFormat;               //!< input YUV format for mjpeg encoding
    int m_iEncQuality;                  //!< jpeg encoding quality
} venc_init_t;

typedef struct venc_seq_header_t
{
    codec_addr_t m_SeqHeaderBuffer[2];  //!< [in]  Seqence header buffer
    int m_iSeqHeaderBufferSize;         //!< [in]  Seqence header buffer size
    unsigned char* m_pSeqHeaderOut;     //!< [out] Seqence header pointer
    int m_iSeqHeaderOutSize;            //!< [out] Seqence header size
} venc_seq_header_t;

typedef struct venc_input_t
{
    int m_bCbCrInterleaved;
    unsigned char* m_pInputY;
    unsigned char* m_pInputCbCr[2];

    int m_iChangeRcParamFlag;   //0: disable, 3:enable(change a bitrate), 5: enable(change a framerate), 7:enable(change bitrate and framerate)
    int m_iChangeTargetKbps;
    int m_iChangeFrameRate;
    int m_iQuantParam;

    codec_addr_t m_BitstreamBufferPA;   //!< [in] physical address
    int m_iBitstreamBufferSize;
    unsigned char request_IntraFrame;
} venc_input_t;

typedef struct venc_output_t
{
    unsigned char* m_pBitstreamOut;
//----> MJPEG only
    int m_iHeaderOutSize;
// <---- MJPEG only
    int m_iBitstreamOutSize;
    int m_iSliceCount;
    unsigned int* m_pSliceInfo;
    int m_iPicType;
} venc_output_t;

#define VENC_INIT       0
#define VENC_CLOSE      1
#define VENC_SEQ_HEADER 2
#define VENC_ENCODE     3


int venc_vpu( int iOpCode, unsigned long* pHandle, void* pParam1, void* pParam2, void* pParam3 );
#ifdef TCC_JPU_INCLUDE
int venc_mjpeg_jpu( int iOpCode, unsigned long* pHandle, void* pParam1, void* pParam2, void* pParam3 );
#endif
typedef int32_t (cdk_func_t) ( int32_t iOpCode, unsigned long* pHandle, unsigned long* pParam1, unsigned long* pParam2, unsigned long* pParam3);
static cdk_func_t* gspfVEncList[VCODEC_MAX] =
{
    venc_vpu //STD_AVC
    ,0 //STD_VC1
    ,0 //STD_MPEG2
    ,0 //STD_MPEG4
    ,0 //STD_H263
    ,0 //STD_DIV3
    ,0 //STD_RV
    ,0 //STD_AVS
    ,0 //STD_WMV78
    ,0 //STD_SH263
#ifdef TCC_JPU_INCLUDE
    ,venc_mjpeg_jpu //STD_MJPG
#else
    ,0 //STD_MJPG
#endif
    ,0 //STD_VP8
    ,0 //STD_THEORA
    ,0 //
    ,0 //STD_MVC
    ,0 //STD_HEVC
    ,0
    ,0
    ,0
    ,0
};

unsigned char *vpu_get_YuvBufAddr(unsigned int index, void *pVenc);
unsigned char *vpu_getStreamOutPhyAddr(unsigned char *convert_addr, unsigned int base_index, void *pVenc);
unsigned char *vpu_getSeqHeaderPhyAddr(unsigned char *convert_addr, unsigned int base_index, void *pVenc);

void * venc_alloc_instance(int codec_format);
void venc_release_instance(void * pInst, int codec_format);
int venc_get_instance_index(void * pInst);

#endif //VENC_H_
