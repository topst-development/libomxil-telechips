// SPDX-License-Identifier: LGPL-2.1-or later
/****************************************************************************
 *   FileName    : vdec_common.h
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
#ifndef VDEC_COMMON_H_
#define VDEC_COMMON_H_

#include <stdint.h>
#include <sys/poll.h>
#include "omx_comp_debug_levels.h"

#define TCC_VPU_INCLUDE

#if defined(USE_COMMON_KERNEL_LOCATION)

#include <TCC_VPU_C7_CODEC.h>
#if defined(TCC_HEVC_INCLUDE)
#include <TCC_HEVC_CODEC.h>
#endif
#if defined(TCC_JPU_C6_INCLUDE)
#include <TCC_JPU_C6.h>
#endif
#if defined(TCC_VPU_4K_D2_INCLUDE)
#include <TCC_VPU_4K_D2_CODEC.h>
#endif
#include <tcc_video_private.h>

#else //use chipset folder

#if defined(TCC_897X_INCLUDE)
#include <mach/TCC_VPU_D6.h>
#else
#include <mach/TCC_VPU_C7_CODEC.h>
#endif
#if defined(TCC_HEVC_INCLUDE)
#include <mach/TCC_HEVC_CODEC.h>
#endif
#if defined(TCC_JPU_C6_INCLUDE)
#include <mach/TCC_JPU_C6.h>
#endif
#include <mach/tcc_video_private.h>

#endif

#include <tcc_video_common.h>

//#define SUPPORT_SORENSON_SPARK_H_263
//#define SUPPORT_VCACHE_CTRL

#if defined(TCC_JPU_C6_INCLUDE)
#define TCC_JPU_INCLUDE
#endif

#define SUPPORT_VP8_DECODER
#define SUPPORT_MVC_DECODER

#if !defined(TCC_VPU_D6_INCLUDE)
#define SUPPORT_EXT_DECODER
#endif

//#define SUPPORT_ASPECT_RATIO

#if defined(USE_COMMON_KERNEL_LOCATION)
#include <tcc_vpu_ioctl.h>
#if defined(TCC_JPU_INCLUDE)
#include <tcc_jpu_ioctl.h>
#endif
#if defined(TCC_HEVC_INCLUDE)
#include <tcc_hevc_ioctl.h>
#endif
#if defined(TCC_VPU_4K_D2_INCLUDE)
#include <tcc_4k_d2_ioctl.h>
#endif
#else //use chipset folder
#include <mach/tcc_vpu_ioctl.h>
#if defined(TCC_JPU_INCLUDE)
#include <mach/tcc_jpu_ioctl.h>
#endif
#if defined(TCC_HEVC_INCLUDE)
#include <mach/tcc_hevc_ioctl.h>
#endif
#endif

#ifdef INCLUDE_WMV78_DEC
#include "wmv78dec/TCC_WMV78_DEC.h"
#endif
#ifdef INCLUDE_SORENSON263_DEC
#include "th263dec.h"
#endif

#ifndef STD_EXT
#    define STD_EXT (6)
#endif

//#define ERROR_TEST
#ifdef ERROR_TEST
int err_test = 0;
#endif
//#define DEBUG_TIME_LOG
#ifdef DEBUG_TIME_LOG
#include "time.h"
unsigned int dec_time[30] = {0,};
unsigned int time_cnt = 0;
unsigned int total_dec_time = 0;

#endif
/********************************************************************************************/
/* TEST and Debugging       =======================> End                *********************/
/********************************************************************************************/

#define REMOVE_SECURE_COPY (0)

#define TCC_VPU_INPUT_BUF_SIZE 		(1024 * 1024)
#define POLL_RETRY_COUNT (2)
#define CMD_RETRY_COUNT (3)

#define CDMX_PTS_MODE		(0)	//! Presentation Timestamp (Display order)
#define CDMX_DTS_MODE		(1)	//! Decode Timestamp (Decode order)

#define CVDEC_DISP_INFO_INIT	(0)
#define CVDEC_DISP_INFO_UPDATE	(1)
#define CVDEC_DISP_INFO_GET		(2)
#define CVDEC_DISP_INFO_RESET	(3)

#define TMP_SKIP_OPT_SKIP_INTERVAL  (5)

#define SEQ_HEADER_INIT_ERROR_COUNT (20000) //sync with REPEAT_COUNT_FOR_THUMBNAIL in tcc_mediaffparser_node.cpp
#define DISPLAYING_FAIL_IN_THUMBNAIL_MODE_ERROR_COUNT (20)
#define MAX_CONSECUTIVE_VPU_FAIL_COUNT (20000)
#define MAX_CONSECUTIVE_VPU_FAIL_COUNT_FOR_IDR (20000)
#define MAX_CONSECUTIVE_VPU_BUFFER_FULL_COUNT (30)
#define MAX_CONSECUTIVE_VPU_FAIL_TO_RESTORE_COUNT (20000)

#define MPEG4_VOL_STARTCODE_MIN		(0x00000120)	// MPEG-4
#define MPEG4_VOL_STARTCODE_MAX		(0x0000012F)	// MPEG-4
#define MPEG4_VOP_STARTCODE				(0x000001B6)	// MPEG-4
#define MAX_SEQ_HEADER_ALLOC_SIZE	(0x0007D000)  // 500KB

#ifndef K_VA
#define K_VA (2)
#endif

//#define MOVE_HW_OPERATION
#define MAX_FRAME_BUFFER_COUNT		(31)

#define AVC_IDR_PICTURE_SEARCH_MODE      (0x001)
#define AVC_NONIDR_PICTURE_SEARCH_MODE   (0x201)

#ifndef VPU_ENV_INIT_ERROR
#define VPU_ENV_INIT_ERROR		(10000)
#endif
#ifndef VPU_NOT_ENOUGH_MEM
#define VPU_NOT_ENOUGH_MEM		(20000)
#endif
//
#define AVAILABLE_MAX_WIDTH    (1920)
#define AVAILABLE_MAX_HEIGHT   (1088)
#define AVAILABLE_MAX_REGION   (AVAILABLE_MAX_WIDTH * AVAILABLE_MAX_HEIGHT)

#if defined(SUPPORT_4K_VIDEO)
# define AVAILABLE_4K_MAX_WIDTH    (4096)
# define AVAILABLE_4K_MAX_HEIGHT   (2160)
#else
# define AVAILABLE_4K_MAX_WIDTH    (1920)
# define AVAILABLE_4K_MAX_HEIGHT   (1088)
#endif
#define AVAILABLE_4K_MAX_REGION     (AVAILABLE_4K_MAX_WIDTH   * AVAILABLE_4K_MAX_HEIGHT)

#define AVAILABLE_HEVC_MAX_WIDTH   AVAILABLE_4K_MAX_WIDTH
#define AVAILABLE_HEVC_MAX_HEIGHT  AVAILABLE_4K_MAX_HEIGHT
#define AVAILABLE_HEVC_MAX_REGION  AVAILABLE_4K_MAX_REGION

#define AVAILABLE_VP9_MAX_WIDTH    (4096)
#define AVAILABLE_VP9_MAX_HEIGHT   (2160)
#define AVAILABLE_VP9_MAX_REGION   (AVAILABLE_VP9_MAX_WIDTH * AVAILABLE_VP9_MAX_HEIGHT)

#define AVAILABLE_MIN_WIDTH    (16)
#define AVAILABLE_MIN_HEIGHT   (16)

#define VDEC_INIT						(0)	//!< init
#define VDEC_DEC_SEQ_HEADER	(1)	//!< seq
#define VDEC_DECODE					(2)	//!< decode
#define VDEC_BUF_FLAG_CLEAR	(3)	//!< display buffer flag clear
#define VDEC_CLOSE					(4)	//!< close
#define VDEC_FINISH					(5)	//!< the end of decoding
#define VDEC_DEC_FLUSH_OUTPUT	(6)	//!< flush delayed output frame
#define VDEC_GET_RING_BUFFER_STATUS		(7)
#define VDEC_FILL_RING_BUFFER					(8) //!< Fill the ring buffer
#define VDEC_UPDATE_WRITE_BUFFER_PTR	(9)
#define VDEC_GET_INITIAL_INFO_FOR_STREAMING_MODE_ONLY	(10)
#define VDEC_GET_INTERMEDIATE_BUF_INFO	(11)
#define VDEC_SW_RESET					(12)
#define VDEC_GET_OUTPUT_INFO	(13)

#define VDEC_SKIP_FRAME_DISABLE		(0)	//!< Skip disable
#define VDEC_SKIP_FRAME_EXCEPT_I	(1)	//!< Skip except I(IDR) picture
#define VDEC_SKIP_FRAME_ONLY_B		(2)	//!< Skip B picture(skip if nal_ref_idc==0 in H.264)
#define VDEC_SKIP_FRAME_UNCOND		(3)	//!< Unconditionally Skip one picture

#ifdef INCLUDE_WMV78_DEC
	#define STD_WMV78	(8)
#endif
#ifdef INCLUDE_SORENSON263_DEC
	#define	STD_SORENSON263	(9)
#endif

#define VCODEC_MAX		(20)
#define VPU_BUFF_COUNT  (VPU_BUFFER_MANAGE_COUNT)

#define REINIT_VPU_FOR_RESOLUTION_CHANGING // Should enable in order to work not in case of framebuffer_max mode!!
/******************* START :: To support adaptive or smooth streaming or something like those *******************************/
//#define SET_FRAMEBUFFER_INTO_MAX
#ifdef SET_FRAMEBUFFER_INTO_MAX
#define VPU_FRAMEBUFFER_MAX_WIDTH		(1920)
#define VPU_FRAMEBUFFER_MAX_HEIGHT	(1080)
#define ONLY_SUPPORT_STREAMING_CASE // In other word, will not support it in case of local playback. Instead of that, local playback will support to re-init vpu.
#endif
/******************* END :: To support adaptive or smooth streaming  ********************************************************/

// The errors should be in sync with "frameworks\av\include\media\stagefright\OMXCodec.h"
#define ERR_PACKED_BITSTREAM    (1) // PB detection: mm008

#if defined(TCC_HEVC_INCLUDE) || defined(TCC_VPU_4K_D2_INCLUDE) //temp!!
#   define USE_MAP_CONVERTER
#   define MIN_RESOLUTION_FOR_MAP_CONVERTER (1920*1088)
#   define USE_PREV_STREAMBUFF_DECODING
#endif

//#define DISPLAY_1ST_DECODED_IDX
#ifdef DISPLAY_1ST_DECODED_IDX
#define MAX_INDEX (33)
#else
#define MAX_INDEX (32)
#endif

#define EXT_FUNC_MEM_PROTECTION_WITH_INSTREAM (0x0001)  // for read protection for VPU memory!! specially WFD-SINK!!
#define EXT_FUNC_MEM_PROTECTION_ONLY					(0x0002)  // for read protection for VPU memory!! specially WFD-SOURCE!!
#define EXT_FUNC_NO_BUFFER_DELAY							(0x0004)  // To ignore buffer-delay on VPU!!
#ifdef USE_MAP_CONVERTER
#define EXT_FUNC_USE_MAP_CONVERTER						(0x0008)  // To enable Map-Converter function for HEVC.
#endif
#define EXT_FUNC_MAX_FRAMEBUFFER							(0x0010)  // To configure reference frame buffer into 1080p for smooth-streamming(adaptive-streamming).
#define EXT_FUNC_DISABLE_10BIT_OUTPUT					(0x0080)  // disable 10 bit output (chip dependency, thumnail, etc.)
#define EXT_FUNC_HIDE_SUPERFRAME_OUTPUT				(0x0100)  // hide VP9 super-frame output to user-space
#define EXT_FUNC_4KD2_USE_CQ_LEVEL_1					(0x0200)  // To enable CQ Level 1 for VPU-4KD2

typedef struct {
    uint32_t    fullRange;
    int32_t primaries;
    int32_t matrix_coeffs;
    int32_t transfer;
} vpu_color_aspects_t;

typedef struct vpu_hdr_color_t {
    // Static_Metadata_Descriptor_ID
    uint8_t mID;

    uint16_t mR[2]; // display primary 0
    uint16_t mG[2]; // display primary 1
    uint16_t mB[2]; // display primary 2
    uint16_t mW[2]; // white point

    uint16_t mMaxDisplayLuminance; // in cd/m^2
    uint16_t mMinDisplayLuminance; // in 0.0001 cd/m^2
    uint16_t mMaxContentLightLevel; // in cd/m^2
    uint16_t mMaxFrameAverageLightLevel; // in cd/m^2
} vpu_hdr_color_t;

typedef struct vdec_user_info_t
{
    uint32_t  bitrate_mbps;             //!< video BitRate (Mbps)
    uint32_t  frame_rate;               //!< video FrameRate (fps)
    uint32_t  m_bStillJpeg;             //!< If this is set, jpeg is not M-JPEG but Still-image (ex. **.jpg)
    uint32_t  jpg_ScaleRatio;           ////!< 0 ( Original Size )
                                        //!< 1 ( 1/2 Scaling Down )
                                        //!< 2 ( 1/4 Scaling Down )
                                        //!< 3 ( 1/8 Scaling Down )
    uint32_t extFunction;
} vdec_option_t, vdec_user_info_t;

typedef struct vdec_init_t
{
    int32_t m_iBitstreamFormat;         //!< bitstream format
    int32_t m_iBitstreamBufSize;        //!< bitstream buf size    : multiple of 1024
    int32_t m_iPicWidth;                //!< frame width from demuxer or etc
    int32_t m_iPicHeight;               //!< frame height from demuxer or etc
    uint32_t m_bEnableUserData;         //!< If this is set, VPU returns userdata.
    uint32_t m_bEnableVideoCache;       //!< video cache
    uint32_t m_bCbCrInterleaveMode;     //!< 0 (chroma separate mode   : CbCr data is written in separate frame memories) \n
                                        //!< 1 (chroma interleave mode : CbCr data is interleaved in chroma memory)
    uint32_t m_bFilePlayEnable;         //!< 1:file play mode(default), 0:streaming mode
    uint32_t m_uiMaxResolution;         //!< 0:full-HD, 1:HD, 2:SD

    unsigned char *m_pExtraData;        //!< codec specific data of asf ( WMV78 Video Decoder only )
    int32_t m_iFourCC;                  //!< FourCC info.               ( WMV78 Video Decoder only )
    int32_t m_iExtraDataLen;            //!< codec specific data size   ( WMV78 Video Decoder only )
    uint32_t m_Reserved;
} vdec_init_t;

typedef struct vdec_info_t
{
    int32_t m_iPicWidth;                //!< {(PicX+15)/16} * 16  (this width  will be used while allocating decoder frame buffers. picWidth  is a multiple of 16)
    int32_t m_iPicHeight;               //!< {(PicY+15)/16} * 16  (this height will be used while allocating decoder frame buffers. picHeight is a multiple of 16)
    uint32_t m_uiFrameRateRes;          //!< decoded picture frame rate residual(number of time units of a clock operating at the frequency[m_iFrameRateDiv] Hz, frameRateInfo(frame per second) = m_uiFrameRateRes/m_uiFrameRateDiv
    uint32_t m_uiFrameRateDiv;          //!< decoded picture frame rate unit number in Hz
    int32_t m_iMinFrameBufferCount;     //!< the minimum number of frame buffers that are required for decoding. application must allocate at least this number of frame buffers.
    int32_t m_iMinFrameBufferSize;      //!< minimum frame buffer size
    int32_t m_iFrameBufferFormat;       //!< frame buffer format
    int32_t m_iNormalSliceSize;         //!< recommended size of to save slice. this value is determined as a quater of the memory size for one raw YUV image in KB unit.
    int32_t m_iWorstSliceSize;          //!< recommended size of to save slice in worst case. this value is determined as a half of the memory size for one raw YUV image in KB unit.

    // H264/AVC only param
    pic_crop_t m_iAvcPicCrop;           //!< represents rectangular window information in a frame
    int32_t m_iAvcConstraintSetFlag[4]; //!< syntax element which is used to make level in H.264. used only in H.264 case.
    int32_t m_iAvcParamerSetFlag;       //!< These are H.264 SPS or PPS syntax element
                                        //!< [bit 0  ] direct_8x8_inference_flag in H.264 SPS
    int32_t m_iFrameBufDelay;           //!< maximum display frame buffer delay to process reordering in case of H.264

    // MPEG-4 only param
    int32_t m_iM4vDataPartitionEnable;  //!< ( 0: disable   1: enable )
    int32_t m_iM4vReversibleVlcEnable;  //!< ( 0: disable   1: enable )
    int32_t m_iM4vShortVideoHeader;     //!< ( 0: disable   1: enable )
    int32_t m_iM4vH263AnnexJEnable;     //!< ( 0: disable   1: enable )

    // VC-1 only param
    int32_t m_iVc1Psf;                  //!< this is only available in VC1 and indicates the value of "Progressive Segmented Frame"

    //! Additional Info
    int32_t m_iProfile;                 //!< profile of the decoded stream
    int32_t m_iLevel;                   //!< level of the decoded stream
    int32_t m_iInterlace;               //!< when this value is 1, decoded stream will be decoded into both progressive or interlace frame.
                                        //!< otherwise, all the frames will be progressive.
    int32_t m_iAspectRateInfo;          //!< aspect rate information. if this value is 0, then aspect ratio information is not present
    int32_t m_iReportErrorReason;       //!< reports reason of 'RETCODE_CODEC_SPECOUT' or 'RETCODE_INVALID_STRIDE' error

} vdec_info_t;

typedef struct vdec_input_t
{
    unsigned char* m_pInp[2];               //!< input data
    int32_t m_iInpLen;                      //!< input data len
    unsigned char* m_UserDataAddr[2];       //!< Picture Layer User-data address
    int32_t m_iUserDataBufferSize;          //!< Picture Layer User-data Size

    int32_t m_iFrameSearchEnable;           //!< I-frame Search Mode. \n
                                            //!< If this option is enabled, the decoder skips the frame decoding until decoder meet IDR(and/or I)-picture for H.264 or I-frame.\n
                                            //!< If this option is enabled, m_iSkipFrameMode option is ignored.\n
                                            //!< 0 ( Disable ) \n
                                            //!< 1 ( Enable : search IDR-picture for H.264 or I-frame ) \n
                                            //!< 513 ( Enable : search I(or IDR)-picture for H.264 or I-frame )

    int32_t m_iSkipFrameMode;               //!< Skip Frame Mode \n
                                            //!< 0 ( Skip disable ) \n
                                            //!< 1 ( Skip except I(IDR) picture ) \n
                                            //!< 2 ( Skip B picture : skip if nal_ref_idc==0 in H.264 ) \n
                                            //!< 3 ( Unconditionally Skip one picture )

    int32_t m_iSkipFrameNum;                //!< Number of skip frames (for I-frame Search Mode or Skip Frame Mode) \n
                                            //!< When this number is 0, m_iSkipFrameMode option is disabled.

    int32_t m_iIsThumbnail;                 //!< For checking size of memory while thumbnail extracting on sequence header init
} vdec_input_t;

#ifndef COMP_Y
#define COMP_Y (0)
#endif
#ifndef COMP_U
#define COMP_U (1)
#endif
#ifndef COMP_V
#define COMP_V (2)
#endif

typedef struct vdec_initial_info_t
{
    int32_t m_iPicWidth;                //!< {(PicX+15)/16} * 16  (this width  will be used while allocating decoder frame buffers. picWidth  is a multiple of 16)
    int32_t m_iPicHeight;               //!< {(PicY+15)/16} * 16  (this height will be used while allocating decoder frame buffers. picHeight is a multiple of 16)
    pic_crop_t m_iPicCrop;              //!< represents rectangular window information in a frame

    int32_t m_iInterlace;               //!< when this value is 1, decoded stream will be decoded into both progressive or interlace frame. otherwise, all the frames will be progressive. In H.264, this is frame_mbs_only_flag.
    int32_t m_iMjpg_sourceFormat;       //!< MJPEG source chroma format(0 - 4:2:0, 1 - 4:2:2, 2 - 4:2:2 vertical, 3 - 4:4:4, 4 - 4:0:0 )(only for TCC891x/88xx/93XX)

    int32_t m_iFrameBufferFormat;       // HEVC out Bit Depth info(0 : 8 Bit, 6 : 10 Bit)
    int32_t m_iSourceFormat;            // HEVC Source Bit Depth info(0 : 8 Bit, 6 : 10 Bit)
    int32_t m_iErrCode;                 // mm008
} vdec_initial_info_t;

typedef struct vdec_output_t
{
	dec_output_info_t m_DecOutInfo;
	unsigned char* m_pDispOut[2][3];	//!< physical[0] and virtual[1] display  address of Y, Cb, Cr component
	unsigned char* m_pCurrOut[2][3];	//!< physical[0] and virtual[1] current  address of Y, Cb, Cr component
	unsigned char* m_pPrevOut[2][3];	//!< physical[0] and virtual[1] previous address of Y, Cb, Cr component
	vdec_initial_info_t* m_pInitialInfo;
#ifdef USE_MAP_CONVERTER
  #if defined(TCC_VPU_4K_D2_INCLUDE)
    hevc_MapConv_info_t m_MapConvInfo;
  #else
	hevc_dec_MapConv_info_t	m_MapConvInfo;
  #endif
#endif

#ifdef USE_DTRC_CONVERTER
	vp9_compressed_info_t m_DtrcConvInfo;
#endif
#ifdef TCC_HEVC_INCLUDE
    hevc_dec_initial_info_t *m_pInitialInfo_Hevc;
    //hevc user data
    uint32_t m_uiUserData;
	hevc_userdata_output_t m_UserDataInfo;
#endif
#ifdef TCC_VPU_4K_D2_INCLUDE
    int32_t m_iIsSuperFrame;
    vpu_4K_D2_dec_initial_info_t *m_pInitialInfo_4KD2;
    //hevc user data
    uint32_t m_IsHdrPlus;
	hdr10p_userdata_info_t m_VsifData;
	hevc_userdata_output_t m_UserDataInfo;
#endif 
    uint32_t m_uiBitDepthLuma;
#if defined(USE_PREV_STREAMBUFF_DECODING)
    uint32_t m_NumDisplayPending;
#endif
} vdec_output_t;

typedef struct vdec_ring_buffer_set_t
{
    unsigned char *m_pbyBuffer;
    uint32_t m_uiBufferSize;
} vdec_ring_buffer_set_t;

typedef struct vdec_ring_buffer_out_t
{
    uint64_t m_ulAvailableSpaceInRingBuffer;
    uint64_t m_ptrReadAddr_PA;
    uint64_t m_ptrWriteAddr_PA;
} vdec_ring_buffer_out_t;

#define EXT_V_DECODER_TR_TEST
#define TIMESTAMP_CORRECTION
#ifdef TIMESTAMP_CORRECTION
typedef struct pts_ctrl
{
    int64_t m_lLatestPTS;//[usec]
    int32_t m_iPTSInterval;
    int32_t m_iRamainingDuration;
} pts_ctrl;
#endif

#ifdef EXT_V_DECODER_TR_TEST
typedef struct EXT_F_frame_t
{
    int32_t Current_TR;
    int32_t Previous_TR;
    int32_t Current_time_stamp;
    int32_t Previous_time_stamp;
} EXT_F_frame_t;

typedef struct EXT_F_frame_time_t
{
    EXT_F_frame_t  ref_frame;
    EXT_F_frame_t  frame_P1;
    EXT_F_frame_t  frame_P2;
} EXT_F_frame_time_t;
#endif

typedef struct dec_disp_info_ctrl_t
{
    int32_t       m_iTimeStampType;                            //! TS(Timestamp) type (0: Presentation TS(default), 1:Decode TS)
    int32_t       m_iStdType;                                  //! STD type
    int32_t       m_iFmtType;                                  //! Formater Type

    int32_t       m_iUsedIdxPTS;                               //! total number of decoded index for PTS
    int32_t       m_iPrevIdx;                                   //! Index of previous frame
    int32_t       m_iRegIdxPTS[MAX_INDEX];                             //! decoded index for PTS
    void          *m_pRegInfoPTS[MAX_INDEX];                        //! side information of the decoded index for PTS

    int32_t       m_iDecodeIdxDTS;                             //! stored DTS index of decoded frame
    int32_t       m_iDispIdxDTS;                               //! display DTS index of DTS array
    int64_t       m_lDTS[MAX_INDEX];//[usec]                        //! Decode Timestamp (decoding order)

    int32_t       m_iReserved;
} dec_disp_info_ctrl_t;

typedef struct dec_disp_info_t
{
    int32_t       m_iFrameType;                                //! Frame Type

    int64_t       m_lTimeStamp;   //[usec]                     //! Time Stamp
    int64_t       m_lextTimeStamp;//[usec]                     //! TR(Ext)


    int32_t       m_iPicStructure;                             //! PictureStructure
    int32_t       m_iM2vFieldSequence;                         //! Field sequence(MPEG2)
    int32_t       m_iFrameDuration;                            //! MPEG2 Frame Duration

    int32_t       m_iFrameSize;                                //! Frame size

    int32_t       m_iTopFieldFirst;                            //! Top Field First
    int32_t       m_iIsProgressive;                            //! Interlace information :: 0:interlace, 1:progressive

    int32_t       m_bIsMvcDependent;                           //! MVC Dependent frame
    int32_t       m_iNumMBError;
    int32_t       m_iPicWidth;
    int32_t       m_iPicHeight;
} dec_disp_info_t;

typedef struct dec_disp_info_input_t
{
    int32_t       m_iFrameIdx;                                 //! Display frame buffer index for CVDEC_DISP_INFO_UPDATE command
    //! Decoded frame buffer index for CVDEC_DISP_INFO_GET command
    int32_t       m_iStdType;                                  //! STD type for CVDEC_DISP_INFO_INIT
    int32_t       m_iTimeStampType;                            //! TS(Timestamp) type (0: Presentation TS(default), 1:Decode TS) for CVDEC_DISP_INFO_INIT
    int32_t       m_iFmtType;                                  //! Formater Type specification
    int32_t       m_iFrameRate;
} dec_disp_info_input_t;

typedef struct mpeg2_pts_ctrl
{
//  int32_t       m_iLatestPTS;
    int64_t       m_lLatestPTS;//[usec]
    int32_t       m_iPTSInterval;
    int32_t       m_iRamainingDuration;
} mpeg2_pts_ctrl;

/********************************************************************************************/
/* TEST and Debugging       =======================> Start              *********************/
/********************************************************************************************/
#define LOGV(...)	{\
		if (gs_iDbgmsgLevel_Vdec >= 5) {\
			tcc_printf(T_DEFAULT "[OMXVDEC:V]" __VA_ARGS__);\
			tcc_printf(" - [%s - %d]\n"T_DEFAULT,__func__,__LINE__);\
		}\
	}
#define LOGD(...)	{\
		if (gs_iDbgmsgLevel_Vdec >= 4) {\
			tcc_printf(TC_CYAN "[OMXVDEC:D]" __VA_ARGS__);tcc_printf(" - [%s - %d]\n"T_DEFAULT,__func__,__LINE__);\
		}\
	}
#define LOGI(...)	{\
		if (gs_iDbgmsgLevel_Vdec >= 3) {\
			tcc_printf(TC_GREEN "[OMXVDEC:I]" __VA_ARGS__);tcc_printf(" - [%s - %d]\n"T_DEFAULT,__func__,__LINE__);\
		}\
	}
#define LOGW(...)	{\
		if (gs_iDbgmsgLevel_Vdec >= 2) {\
			tcc_printf(TC_MAGENTA "[OMXVDEC:W]" __VA_ARGS__);tcc_printf(" - [%s - %d]\n"T_DEFAULT,__func__,__LINE__);\
		}\
	}
#define LOGE(...)	{\
		if (gs_iDbgmsgLevel_Vdec >= 1) {\
			tcc_printf(TC_RED "[OMXVDEC:E]" __VA_ARGS__);tcc_printf(" - [%s - %d]\n"T_DEFAULT,__func__,__LINE__);\
		}\
	}

static int DEBUG_ON	= 0;
#define DPRINTF(msg...)	  {LOGE(msg);}
#define DSTATUS(msg...)	  {LOGD(msg);}
#define DPRINTF_FRAME(msg...) {LOGV(msg);}
#define DISPLAY_BUFFER(msg...) {LOGV(msg);}
#define CLEAR_BUFFER(msg...) {LOGV(msg);}
#define SEQ_EXTRACTOR(msg...) {LOGD(msg);}

//#define VPU_IN_FRAME_DUMP
#ifdef VPU_IN_FRAME_DUMP
//#define VPU_ALL_FRAME_DUMP
#define TARGET_STORAGE "/data"
#define MAX_DUMP_LEN (10*1024*1024) //10MB
#define MAX_DUMP_CNT (10)
#endif
//#define VPU_OUT_FRAME_DUMP // Change the output format into YUV420 seperated to play on PC.
//#define HEVC_OUT_FRAME_DUMP //HEVC Data save
//#define CHANGE_INPUT_STREAM // to change frame-data to test stream from RTP etc.
#ifdef CHANGE_INPUT_STREAM
#define STREAM_NAME "data/changed_stream.dat"
#endif

#define ALIGN_LEN (4*1024)

typedef int32_t (*DEC_FUNC)( int32_t iOpCode, unsigned long* pHandle, unsigned long* pParam1, unsigned long* pParam2 );

typedef struct _vdec_ {
  int32_t vdec_instance_index;
  int32_t vpu_closed[VPU_MAX*2];
  int32_t renderered;
  uint32_t total_frm;

  unsigned char vdec_env_opened;
  unsigned char vdec_codec_opened;
  unsigned char prev_codec;

  int32_t mgr_fd; // default -1
  int32_t dec_fd; // default -1

#if defined(TCC_HEVC_INCLUDE) || defined(TCC_VPU_4K_D2_INCLUDE)
  int32_t hHDMIDevice; // default: -1
#endif

  int32_t codec_format;
  uint32_t aspect_ratio;

#if defined(VPU_OUT_FRAME_DUMP) || defined(CHANGE_INPUT_STREAM)
  FILE *pFs;
  uint32_t is1st_dec;
  unsigned char* backup_data;
  uint32_t backup_len;
#endif

  int32_t gsBitstreamBufSize;
  int32_t  gsIntermediateBufSize;
  int32_t gsUserdataBufSize;
  int32_t gsMaxBitstreamSize;
  int32_t gsAdditionalFrameCount;
  int32_t gsTotalFrameCount;

  codec_addr_t gsBitstreamBufAddr[3];
  codec_addr_t gsIntermediateBufAddr[3];
  codec_addr_t gsUserdataBufAddr[3];

  dec_user_info_t gsUserInfo;

  codec_addr_t gsRegBaseVideoBusAddr;
  codec_addr_t gsRegBaseClkCtrlAddr;

  int32_t gsBitWorkBufSize;
  int32_t gsSliceBufSize;
  int32_t gsSpsPpsSize;
  int32_t gsFrameBufSize;
  int32_t gsFrameBufCount;
#if defined(SUPPORT_VP8_DECODER)
  int32_t gsMbSaveSize;
#endif

  codec_addr_t gsBitWorkBufAddr[3];
  codec_addr_t gsSliceBufAddr;
  codec_addr_t gsSpsPpsAddr;
  codec_addr_t gsFrameBufAddr[3];
#if defined(SUPPORT_VP8_DECODER)
  codec_addr_t gsMbSaveAddr;
#endif

  VDEC_INIT_t gsVpuDecInit_Info;
  VDEC_SEQ_HEADER_t gsVpuDecSeqHeader_Info;
  VDEC_SET_BUFFER_t gsVpuDecBuffer_Info;
  VDEC_DECODE_t gsVpuDecInOut_Info;
  VDEC_RINGBUF_GETINFO_t gsVpuDecBufStatus;
  VDEC_RINGBUF_SETBUF_t gsVpuDecBufFill;
  VDEC_RINGBUF_SETBUF_PTRONLY_t gsVpuDecUpdateWP;
  VDEC_GET_VERSION_t gsVpuDecVersion;

#ifdef TCC_HEVC_INCLUDE
#ifdef USE_PREV_STREAMBUFF_DECODING
  int gsCurrStreamBuffer_index;
#endif
  HEVC_INIT_t gsHevcDecInit_Info;
  HEVC_SEQ_HEADER_t gsHevcDecSeqHeader_Info;
  HEVC_SET_BUFFER_t gsHevcDecBuffer_Info;
  HEVC_DECODE_t gsHevcDecInOut_Info;
  HEVC_RINGBUF_GETINFO_t gsHevcDecBufStatus;
  HEVC_RINGBUF_SETBUF_t gsHevcDecBufFill;
  HEVC_RINGBUF_SETBUF_PTRONLY_t gsHevcDecUpdateWP;
  HEVC_GET_VERSION_t gsHevcDecVersion;

  uint32_t m_uiUserData;
  hevc_userdata_output_t gsHevcUserDataInfo;
  int32_t gsHEVCUserDataEOTF;
  int32_t gsHLGFLAG;
#endif

#if defined (TCC_VPU_4K_D2_INCLUDE)
#ifdef USE_PREV_STREAMBUFF_DECODING
  int gsCurrStreamBuffer_index;
  uint32_t bBitstreamSplitIndexPinned;
#endif
  VPU_4K_D2_INIT_t                   gsVpu4KD2InitInfo;
  VPU_4K_D2_SEQ_HEADER_t             gsVpu4KD2SeqHeaderInfo;
  VPU_4K_D2_SET_BUFFER_t             gsV4kd2DecBufferInfo;
  VPU_4K_D2_DECODE_t                 gsVpu4KD2InOutInfo;
  VPU_4K_D2_RINGBUF_GETINFO_t        gsVpu4KD2BufStatus;
  VPU_4K_D2_RINGBUF_SETBUF_t         gsVpu4KD2BufFill;
  VPU_4K_D2_RINGBUF_SETBUF_PTRONLY_t gsVpu4KD2UpdateWP;
  VPU_4K_D2_GET_VERSION_t            gsVpu4KD2Version;
  int32_t bSeqChanged;
  int32_t prev_buffer_full;
  int32_t gsHEVCUserDataEOTF;
  int32_t gsHLGFLAG;
#endif

#ifdef TCC_JPU_INCLUDE
  JDEC_INIT_t gsJpuDecInit_Info;
  JDEC_SEQ_HEADER_t gsJpuDecSeqHeader_Info;
  JPU_SET_BUFFER_t gsJpuDecBuffer_Info;
  JPU_DECODE_t gsJpuDecInOut_Info;
  JPU_GET_VERSION_t gsJpuDecVersion;
#endif
  int32_t gsbHasSeqHeader;
#ifdef INCLUDE_WMV78_DEC	// for WMV78 video decoder
  unsigned char* gsWMV78CurYFrameAddress;
  unsigned char* gsWMV78CurUFrameAddress;
  unsigned char* gsWMV78CurVFrameAddress;
  unsigned char* gsWMV78Ref0YFrameAddress;
  unsigned char* gsWMV78Ref0UFrameAddress;
  unsigned char* gsWMV78Ref0VFrameAddress;

  int32_t             gsWMV78FrameSize;
  int32_t             gsWMV78NCFrameSize;
  WMV78_HANDLE		gsWMV78DecHandle;
  WMV78_INIT			gsWMV78DecInit;
  WMV78_INPUT			gsWMV78DecInput;
  WMV78_OUTPUT		gsWMV78DecOutput;
  dec_initial_info_t	gsWMV78DecInitialInfo;
#endif

#ifdef INCLUDE_SORENSON263_DEC
  codec_handle_t 		gsS263DecHandle;
  dec_init_t 			gsS263DecInit;
  dec_initial_info_t 	gsS263DecInitialInfo;
  h263dec_stats_t 	gsS263DecInitialStats;
  unsigned char 		*gsS263DecHeapAddr;
#endif

#if defined(INCLUDE_WMV78_DEC) || defined(INCLUDE_SORENSON263_DEC) || !defined(SUPPORT_MANAGE_MJPEG_BUFFER)
  int32_t gsFirstFrame;
  int32_t gsIsINITdone;

  dec_init_t gsVpuDecInit;

  codec_addr_t decoded_phyAddr[10];
  codec_addr_t decoded_virtAddr[10];
  uint32_t decoded_buf_maxcnt;
  uint32_t decoded_buf_size;
  uint32_t decoded_buf_curIdx;
#endif

  struct pollfd tcc_event[1];

  int32_t bMaxfb_Mode;

  int32_t mMaxAdaptiveWidth;
  int32_t mMaxAdaptiveHeight;
  int32_t mRealPicWidth;
  int32_t mRealPicHeight;
  int32_t extFunction;

  unsigned long* pExtDLLModule;
  DEC_FUNC gExtDecFunc;

  vdec_initial_info_t gsCommDecInfo;

  pts_ctrl gsPtsInfo;
  int32_t gsextTRDelta;
  int32_t gsextP_frame_cnt;
  int32_t gsextReference_Flag;
  EXT_F_frame_time_t gsEXT_F_frame_time;

#ifdef DISPLAY_1ST_DECODED_IDX
  int32_t mdisplayed_1st_IFrm;
#endif
#ifdef VPU_IN_FRAME_DUMP
  uint32_t lenDump;
  int32_t IdxDump;
  int32_t DelDump;
#endif
  int32_t AvcSeq_status;
  uint32_t m_bCbCrInterleaveMode;

  vpu_color_aspects_t stColorAspects;
  uint32_t bHasColorAspects;
  vpu_hdr_color_t  stHdrColor;
  uint32_t bHasHdrInfo;
  uint32_t bUseUserDataBearer;
} _vdec_;

void vpu_set_additional_refframe_count(int32_t count, unsigned long* pInst);
int vpu_get_refframe_count(unsigned long* pInst);
void vpu_update_sizeinfo(uint32_t format, uint32_t bps, uint32_t fps, uint32_t image_width, uint32_t image_height, unsigned long* pVdec);
unsigned char *vpu_getBitstreamBufAddr(uint32_t index, unsigned long* pVdec);
int32_t  vpu_getBitstreamBufSize(unsigned long *pVdec);
#if defined(TCC_HEVC_INCLUDE) || defined(TCC_VPU_4K_D2_INCLUDE)
void notify_hdmi_drm_config(_vdec_ *pInst, hevc_userdata_output_t *dst);
#endif
/* VDEC hevc prescan/decode buffer management */
#if defined(USE_PREV_STREAMBUFF_DECODING)
int32_t VDEC_GetCurrBitstreamSplitIndex(void *pVdec);
void VDEC_ResetBitstreamSplitIndex(void *pVdec);
void VDEC_PinBitstreamSplitIndex(void *pVdec, uint32_t on_off);
void VDEC_ToggleBitstreamSplitIndex(void *pVdec);
#endif

int32_t vdec_vpu( int32_t iOpCode, unsigned long* pHandle, unsigned long* pParam1, unsigned long* pParam2, unsigned long* pParam3 );

#ifdef INCLUDE_WMV78_DEC
int32_t vdec_WMV78( int32_t iOpCode, unsigned long* pHandle, unsigned long* pParam1, unsigned long* pParam2, unsigned long* pParam3 );
#endif

#ifdef TCC_JPU_INCLUDE
int32_t vdec_mjpeg_jpu( int32_t iOpCode, unsigned long* pHandle, unsigned long* pParam1, unsigned long* pParam2, unsigned long* pParam3 ); // For TCC892XS / TCC893XS, use JPU as a MJPEG H/W decoder
#endif

#ifdef TCC_HEVC_INCLUDE
int32_t vdec_hevc( int32_t iOpCode, unsigned long* pHandle, unsigned long* pParam1, unsigned long* pParam2, unsigned long* pParam3 );
#endif

#ifdef TCC_VPU_4K_D2_INCLUDE
int32_t vdec_vpu_4k_d2( int32_t iOpCode, unsigned long* pHandle, unsigned long* pParam1, unsigned long* pParam2, unsigned long* pParam3 );
#endif

#ifdef INCLUDE_SORENSON263_DEC
int32_t vdec_sorensonH263dec( int32_t iOpCode, unsigned long* pHandle, unsigned long* pParam1, unsigned long* pParam2, unsigned long* pParam3 );
#endif

typedef int32_t (cdk_func_t) ( int32_t iOpCode, unsigned long* pHandle, unsigned long* pParam1, unsigned long* pParam2, unsigned long* pParam3);

cdk_func_t* gspfVDecList[VCODEC_MAX];
uint32_t cdk_sys_remain_memory_size( _vdec_ * pVdec );
uint32_t cdk_sys_final_free_mem( _vdec_ * pVdec );
unsigned long * cdk_sys_malloc_physical_addr(unsigned long *remap_addr, uint32_t uiSize, Buffer_Type type, _vdec_ *pVdec);
unsigned long * cdk_sys_malloc_virtual_addr(unsigned long * pPtr, uint32_t uiSize, _vdec_ *pVdec);
void cdk_sys_free_virtual_addr(unsigned long* pPtr, uint32_t uiSize);
unsigned char *vpu_getFrameBufVirtAddr(unsigned char *convert_addr, uint32_t base_index, unsigned long* pVdec);
void set_max_adaptive_resolution(int max_width, int max_height, void* pVdec);
void set_dec_common_info(int w, int h, pic_crop_t* crop, int interlace, int mjpgFmt, _vdec_ *pVdec);
int vpu_env_open(uint32_t format, uint32_t bps, uint32_t fps, uint32_t image_width, uint32_t image_height, _vdec_ *pVdec);
void vpu_env_close(_vdec_ *pVdec);
void save_input_stream(char* name, int32_t size, _vdec_ * pVdec, int32_t check_size);
#ifdef VPU_OUT_FRAME_DUMP
void save_decoded_frame(unsigned char* Y, unsigned char* U, unsigned char *V, int width, int height, _vdec_ *pVdec);
#endif
#ifdef CHANGE_INPUT_STREAM
void change_input_stream(unsigned char* out, int* len, int type, _vdec_ *pVdec);
#endif
void print_dec_initial_info( dec_init_t* pDecInit, dec_initial_info_t* pInitialInfo, _vdec_ *pVdec);
unsigned long* vdec_alloc_instance(int32_t codec_format, int32_t refer_instance);
void vdec_release_instance(unsigned long* pInst);
int32_t vdec_get_instance_index(unsigned long* pInst);
#endif //VDEC_COMMON_H_

