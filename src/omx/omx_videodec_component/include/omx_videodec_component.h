/**
  @file omx_videodec_component.h

  This component implements various video decoder.
  (MPEG4/AVC/H.263/WMV/Ext)

  Copyright (C) 2007-2008 STMicroelectronics
  Copyright (C) 2007-2008 Nokia Corporation and/or its subsidiary(-ies).
  Copyright (C) 2009-2014 Telechips Inc.

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

  $Date: 2009/03/10 13:33:29 $
  Revision $Rev: 554 $
  Author $Author: Audio/Video Group of Telechipa Inc $
*/

#ifndef OMX_VIDEODEC_COMPONENT_H_
#define OMX_VIDEODEC_COMPONENT_H_

#include <OMX_Types.h>
#include <OMX_Component.h>
#include <OMX_Core.h>
#include <OMX_Video.h>
#include <OMX_TCC_Index.h>
#include <omx_base_filter.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>

#include <tcc_video_common.h>
#include <vdec.h>


#ifndef TRUE
#define TRUE (1)
#endif
#ifndef FALSE
#define FALSE (0)
#endif
#ifndef prtdiff_t
#define ptrdiff_t size_t
#endif

/*
   frame skip (or search) mode definitions
*/
#define SKIPMODE_NONE               (0)
#define SKIPMODE_I_SEARCH           (1)
#define SKIPMODE_B_SKIP             (2)
#define SKIPMODE_EXCEPT_I_SKIP      (3)
#define SKIPMODE_NEXT_B_SKIP        (4)
#define SKIPMODE_B_WAIT             (5)
#define SKIPMODE_NEXT_STEP          (10)

#define SKIPMODE_IDR_SEARCH         (21)	// (SKIPMODE_I_SEARCH)
#define SKIPMODE_NONIDR_SEARCH      (22)	// (SKIPMODE_I_SEARCH)

/*
   decoder ring-buffer state structures
*/
typedef struct ringbuff_state_t {
	void             *pstVDecPrivate;
	OMX_U8           *pRingBuffBase[2];
	OMX_U8           *pRingBuffEnd[2];
	OMX_U8           *pReadPtr;
	OMX_U8           *pWritePtr;
	OMX_U8           *pPrevReadPtr;
	OMX_U8           *pPrevWritePtr;

	OMX_U8           *pBackupBuffer;  // for resolution change
	OMX_U8           *pPrevRingBuffBase;
	OMX_U8           *pPrevRingBuffEnd;
	OMX_S32           lBackupStartOffset;
	OMX_S32           lBackupLength;

	OMX_S32           lRingBuffSize;
	OMX_S32           lWrittenBytes;
	OMX_S32           lEmptySpace;
	OMX_S32           lUsedByte;
	OMX_BOOL          bFlushRing;
} ringbuff_state_t;

/*
   Input buffer information queue structures
*/
typedef struct input_info_t {
	OMX_TICKS  llTimestamp;
	OMX_S32    lFilledLen;
	OMX_U8    *pStartPtr;
	OMX_U8    *pEndPtr;
} input_info_t;

typedef struct input_queue_t {
	input_info_t  *pstInfoRing;
	OMX_S32        lQueSize;
	OMX_S32        lQueCount;
	OMX_S32        lStartPos;
	OMX_S32        lEndPos;
	OMX_U8        *pBasePtr; //for debugging
} input_queue_t;

/*
   Output buffer index queue structures for vsync mode
*/
#define REFERENCE_BUFFER_MAX (32)
typedef struct dispidx_queue_t {
	OMX_U32  aulDispIdxRing[REFERENCE_BUFFER_MAX];
	OMX_U32  aulDispIdxRing_DV[REFERENCE_BUFFER_MAX];
	OMX_S32  lQueCount;
	OMX_S32  lStartPos;
	OMX_S32  lEndPos;
	OMX_S32  needClearCount;
} dispidx_queue_t;

/*
   Output buffer index list structures for non-vsync mode
*/
typedef struct dispbuff_state_t {
	OMX_PTR  pOmxOutBuff;
	OMX_BOOL bIsCleared;
	OMX_S32  alKeepedIndex[2];
	OMX_S32  alDisplayIndex[2];
} dispbuff_state_t;

typedef struct dispidx_list_t {
	dispbuff_state_t  astDispBuffState[REFERENCE_BUFFER_MAX];
	OMX_S32           lUsedCount;
	OMX_S32           lMaxCount;
	OMX_PTR           pVpuInstance;
	OMX_S32           hFBDevice;
} dispidx_list_t;

/*
   Display information manager structures
*/
typedef struct disp_info_t {
	OMX_S32      lFrameType;           // Frame type
	OMX_TICKS    llTimestamp;          // Timestamp (us)
	OMX_TICKS    llPrevTimestamp;      // Timestamp (us) for error control
	OMX_S32      lExtTimestamp;         // TR fo RealVideo timestamp
	OMX_S32      lPicStructure;        // PictureStructure
	OMX_S32      lM2vFieldSequence;    // Field sequence(MPEG2)
	OMX_S32      lFrameDurationFactor; // MPEG2 frame duration factor
	OMX_S32      lFrameSize;           // Frame size
	OMX_S32      lCropLeft;
	OMX_S32      lCropTop;
	OMX_S32      lCropWidth;
	OMX_S32      lCropHeight;
	OMX_S32      lErrorMB;             // Number of error macroblock

	#define DISPINFO_FLAG_INTERLACED        (0x00000001)
	#define DISPINFO_FLAG_ODDFIELD_FIRST    (0x00000002)
	#define DISPINFO_FLAG_DEPENDENT_VIEW    (0x00000004)
	#define DISPINFO_FLAG_FIELD_DECODED     (0x00000008)
	OMX_U32      ulFlags;              // Output buffer flags
	OMX_BOOL     bPtsNotExist;
	OMX_BOOL     bIsValid;

	OMX_U8      *pbyUserDataBuff;
	OMX_S32      lUserDataBuffSize;
	OMX_S32      lUserDataLength;
} disp_info_t;

typedef struct dispinfo_manager_t {
	disp_info_t  astDispInfoList[REFERENCE_BUFFER_MAX];
	OMX_TICKS    allTimestampRing[REFERENCE_BUFFER_MAX];
	OMX_S32      lQueCount;
	OMX_S32      lStartPos;
	OMX_S32      lEndPos;
	OMX_S32      lStreamType;          // stream type (STD_XXX)
	OMX_S32      lFrameRate;           // frame-rate (fps*1000)
	OMX_S32      lFrameDurationFactor; // MPEG2 frame duration factor
	OMX_TICKS    llFrameDuration;      // frame-duration (us, 1000000/fps)
	OMX_TICKS    llLastBaseTimestamp;  // Last base timestamp
	OMX_TICKS    llLastOutTimestamp;   // Last output timestamp
	OMX_TICKS    llLastDecTimestamp;   // Last decoded timestamp
	OMX_TICKS    llShowedTimestamp;    // Last showed timestamp

	#define DISPMGR_BACKWARD_PLAYBACK     (0x00000001)
	#define DISPMGR_TIMESTAMP_SORTING     (0x00000002)
	#define DISPMGR_TIMESTAMP_GENERATING  (0x00000004)
	#define DISPMGR_NON_B_FRAME_UPDATED   (0x00000010)
	#define DISPMGR_PTS_MODE_DETERMINED   (0x00000020)
	OMX_U32      ulFlags;              // state flags

	OMX_TICKS    lExtTimestamp[2];
	OMX_S32      lExtTimeReference[2];
	OMX_S32      lExtReferenceIdx;
} dispinfo_manager_t;


/*
   Structures etc
*/
typedef struct output_info_t {
	OMX_S32      lFrameType;           // Frame type
	OMX_S32      lPicStructure;        // PictureStructure
	OMX_TICKS    llTimestamp;          // Timestamp (us)
	OMX_U32      ulFlags;              // Output buffer flags
	OMX_S32      lErrorMB;
	OMX_S32      lCropTop;
	OMX_S32      lCropLeft;
	OMX_S32      lCropWidth;
	OMX_S32      lCropHeight;
	OMX_U8      *pbyUserDataBuff;
	OMX_S32      lUserDataLength;
} output_info_t;

typedef struct dv_info_t {
  //for build
} dv_info_t;

typedef struct tag_VideoDec_Pmap_T
{
	int iAddress;
	int iSize;
} VideoDec_Pmap_T;

/** Video Decoder component private structure. */
DERIVEDCLASS(omx_videodec_component_PrivateType, omx_base_filter_PrivateType)
#define omx_videodec_component_PrivateType_FIELDS omx_base_filter_PrivateType_FIELDS \
  /** @param pointer of core function of decoder */ \
  cdk_func_t         *pfVDecFunc; \
  /** @param Decoder input/output parameters */ \
  OMX_PTR             pVDecInstance; \
  vdec_init_t         stVDecInit; \
  vdec_input_t        stVDecInput; \
  vdec_output_t       stVDecOutput; \
  vdec_user_info_t    stVDecUserInfo; \
  /** @param helper for VPU ring-buffer operation */ \
  ringbuff_state_t    stRingBuffState; \
  input_queue_t       stInputInfoQueue; \
  dispidx_queue_t     stDispIdxQueue; \
  dispidx_list_t      stDispBuffList; \
  dispinfo_manager_t  stDispInfoMgr; \
  /** @param profile / level */ \
  OMX_VIDEO_PARAM_PROFILELEVELTYPE stProfileLevel; \
  /** @param resolution change */ \
  OMX_CONFIG_RECTTYPE stCropRect; \
  /** @param Aspect Ratio change */ \
  OMX_CONFIG_SCALEFACTORTYPE stScaleFactor; \
  /** @param video_coding_type Field that indicate the supported video format of video decoder */\
  OMX_VIDEO_CODINGTYPE 	enVideoCodingType;\
  OMX_U32             eCodecVersion; \
  /** @param Reference to OMX_VIDEO_PARAM_{xxx}TYPE structure */ \
  OMX_PTR	         *pVideoParam; \
  /** @param extra data */ \
  OMX_U8             *pbyExtraDataBuff; \
  OMX_S32             lExtraDataLength; \
  /** @param private flags */ \
  OMX_U32             ulStateFlags; \
  OMX_U32             ulErrorFlags; \
  OMX_U32             ulModeFlags; \
  /** @param Semaphore for waiting LibInit() */ \
  tsem_t             *pstCodecSyncSem; \
  /** @param output frame count */ \
  OMX_S32             lAdditionalBuffCount; \
  OMX_S32             lFrameBuffCount; \
  OMX_S32             lUsedBuffCount; \
  /** @param input buffer counting */ \
  OMX_S32             lInputCount; \
  /** @param decoding success counting */ \
  OMX_S32             lDecodedCount; \
  OMX_S32             lPreviousKeyCount; \
  OMX_S32             lKeyFrameDistance; \
  /** @param seqinit/decoding/output fail counting */ \
  OMX_S32             lSeqInitFailCount; \
  OMX_S32             lSeqInitFailMax; \
  OMX_S32             lDecodingFailCount; \
  OMX_S32             lDecodingFailMax; \
  OMX_S32             lSkippedCount; \
  OMX_S32             lSkipMax; \
  /** @param frame skip(or search) mode */ \
  OMX_S32             lFrameSkipMode; \
  /** @param about input data feeding (buffering) */ \
  OMX_TICKS           llQueuedStartTime; \
  OMX_TICKS           llFeedMaxTimeDiff; \
  OMX_TICKS           llFeedMinTimeDiff; \
  /** @param frame-rate update */ \
  OMX_S32             lFrameSuccessCount; \
  OMX_S32             lNewFrameRate; \
  /** @param sequence header backup */ \
  OMX_U8             *pbySequenceHeader; \
  OMX_S32             lSeqHeaderLength; \
  /** @param thumbnail output buffer */\
  OMX_U8             *pbyThumbnailBuff; \
  OMX_S32             lThumbnailLength; \
  /** @param tmem device handle for system buffers etc */ \
  OMX_S32             hTMemDevice; \
  /** @param Sequence header buffer for hdcp source mode */ \
  OMX_S32             lSeqBuffMapSize; \
  OMX_PTR             pSeqBuffBase[2]; \
  /** @param MVC output */ \
  OMX_S32             lMVCBaseViewIndex; \
  OMX_PTR             pMVCBaseView[3]; \
  /** @param AvcC stream info */ \
  OMX_S32             lNalLengthSize; \
  /** @param temporary input buffer for stream converting */ \
  OMX_U8             *pbyTempInputBuff; \
  OMX_S32             lTempInputOffset; \
  OMX_S32             lTempInputLength; \
  /** @param pointer to the decoder input buffer */ \
  OMX_U8             *pbyVDecInputPtr; \
  OMX_S32             lVDecInputLength; \
  /** @param H.264 slice counting/merging */ \
  OMX_S32             lSliceCount;\
  OMX_S32             lNumberOfSlice;\
  OMX_S32             lNumberOfErrorFrame;\
  /** @param provide for HEVC prescan */ \
  OMX_TICKS           llPrevInputPts;\
  OMX_TICKS           llPrevPrevInputPts;\
  /** @param etc */ \
  OMX_S32             lKeyframeIndex;\
  OMX_S32             lPicWidthMax;\
  OMX_S32             lPicHeightMax;\
  OMX_BOOL            bDecStarted;\
  /** @param managing display index by gobject  */ \
  OMX_BOOL            bClearDisplayIndex; \
  OMX_S32             lMaxFifoCount; \
  OMX_BOOL            bEventFlush; \
  OMX_S32             lDispPushIndex; \
  OMX_S32             lDispPopIndex; \
  OMX_S32             lDispNbIndex; \
  pthread_mutex_t     dispMutex; \
  /** @param using tmem **/ \
  OMX_S32             mTmem_fd; \
  /** @param Secured SeqBackup Buffer **/ \
  OMX_BOOL            bTEEEnable; \
  VideoDec_Pmap_T              mSeqBackupPmap; \
  void*               mSeqBackupMapInfo; \
  /** FPS **/ \
  OMX_S32 mDecodingCount; \
  OMX_S32 mOutputCount; \
  OMX_S32 mFPS;  \
  OMX_TICKS llFPS_time_diff; \
  OMX_TICKS llFPS_time_prev; \
  OMX_TICKS llFPS_time_sum; \
  /** @param JPU image decoding **/ \
  OMX_U8             *pAddedInputBuffer; \
  OMX_U32             lAddedInputBufferLen; \
  /** @param Dolby-Vision Data **/ \
  dv_info_t mDolbyInfo; \
  OMX_BOOL bPrevHdmiPlugStatus; \
  OMX_S32 mInputFrameCount; \
  /** @param RingMode Used Inpuf size **/ \
  OMX_S32 mInBufUsedSize; \

ENDCLASS(omx_videodec_component_PrivateType)

#endif //OMX_VIDEODEC_COMPONENT_H_
