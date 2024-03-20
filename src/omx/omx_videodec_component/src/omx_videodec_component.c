/**
  @file omx_videodec_component.c

  This component implements various video decoder.
  (MPEG4/AVC/H.263/WMV/RV)

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

#include <stdio.h>

#include <omxcore.h>
#include <omx_base_video_port.h>
#include <omx_videodec_component.h>
#include <OMX_Video.h>
#include <OMX_CoreExt.h>
#include <OMX_IndexExt.h>

#include <lcd_resolution.h>
#include "omx_comp_debug_levels.h"

#include <sys/syscall.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
//#include <linux/fb.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <errno.h>

#if defined(USE_COMMON_KERNEL_LOCATION)
#include <tcc_video_private.h>
#else
#include <mach/tcc_video_private.h>
#endif

#ifdef MOVE_HW_OPERATION
#include <mach/tcc_mem_ioctl.h>
#endif

#include "TCCMemory.h"

/* OMX private */
typedef omx_videodec_component_PrivateType	vdec_private_t;

#define IS_NULL_PARAM(_param) \
        (_param == NULL ? OMX_TRUE : OMX_FALSE)

#define GET_VIDEODEC_PRIVATE(__p_omx_component) \
		((omx_videodec_component_PrivateType*)((OMX_COMPONENTTYPE *)__p_omx_component)->pComponentPrivate)

#define VDEC_FUNC(__ptr_vdec_private, __opcode, __ptr_handle, __ptr_param1, __ptr_param2) \
		((vdec_private_t*)__ptr_vdec_private)->pfVDecFunc(__opcode, __ptr_handle, __ptr_param1, __ptr_param2, ((vdec_private_t*)__ptr_vdec_private)->pVDecInstance)
#define K2Q16(val) (OMX_U32)((double)(val) / 1000 * 0x8000)


////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//  Options
//
//
/*----------------------------------------------------------*/
/*        THESE ARE MUST BE '0' WHEN IT IS RELEASED         */
/*----------------------------------------------------------*/
#define DEBUG_MESSAGE_ENABLE            (1)  //
#define VDEC_INPUT_DUMP_ENABLE          (0)  // decoder input dump on/off
#define VDEC_PROFILER_ENABLE            (0)  // decoder profiler on/off
#define VDEC_LOGGER_ENABLE              (0)  // decoder call logger on/off
#define CHECK_RING_STATUS_AFTER_UPDATE  (0)  // re-checking after ring-buffer status update
#define USE_MAX_VPU_CLOCK               (0)  // MAX VPU clock
#define TEST_CONDITION_ENABLE           (0)  // test condition setting
#define ES_DUMP                         (0)  // no decoding, es stream dump only.
/*------------------------------------------------------------*/

/* build options */
#define DECODER_RINGBUFFER_MODE               (1)
#define FLUSH_RING_BEFORE_UPDATE_TIME         (1)
#define UPDATE_WRITE_PTR_WITH_ALIGNED_LENGTH  (1)
#define WRITE_PTR_ALIGN_BYTES                 (512)
#define USE_AVAILABLE_SIZE_FROM_VDEC          (0)
#define MVC_PROCESS_ENABLE                    (0)
#define CHECK_ERROR_MACROBLOCK                (0)

/* test condition setting */
#if TEST_CONDITION_ENABLE
	#define AVC_CROP_ENABLE                   (1) //
	#define AVC_CROP_APPLIED_TO_PORT          (1) //
	#define SINGLE_FRAME_INPUT_ENABLE         (0) //
	#define RESOLUTION_CHANGE_WITH_CROP       (0) //
	#define RESOLUTION_CHANGE_WITHOUT_EVENT   (1) //
	#define SET_FULLHD_TO_PORT                (1) //
#else //TEST_CONDITION_ENABLE
	#define AVC_CROP_ENABLE                   (1) //
	#define AVC_CROP_APPLIED_TO_PORT          (1) //
	#define SINGLE_FRAME_INPUT_ENABLE         (0) //
	#define RESOLUTION_CHANGE_WITH_CROP       (1) //
	#define RESOLUTION_CHANGE_WITHOUT_EVENT   (1) //
	#define SET_FULLHD_TO_PORT                (0) // 0 <--- fix omx buffer pool problem
#endif //TEST_CONDITION_ENABLE


////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//  Debugging tools
//
//

#define CONDITIONAL_LOG_ENABLE        (1)  // property conditional debug message

#define LOG_OMX_STEP_INFO             (1)
#define LOG_INPUT_BUFF_INFO           (1)  // input buffer information log - LOG_INPUT()
#define LOG_FRAME_SKIP_MODE           (1)  // skip mode information log - LOG_SKIP()
#define LOG_DECODE_STATUS             (1)  // decode status log - LOG_DEC()
#define LOG_OUTPUT_STATUS             (1)  // output status log - LOG_OUT()
#define LOG_INPUT_INFO_QUEUE          (1)  // input information queue internal log - LOG_IIQUE()
#define LOG_INPUT_INFO_MANAGEMENT     (1)  // input information management log - LOG_IIMGE()
#define LOG_RING_BUFFER_STATUS        (1)  // decoder ring-buffer helper internal log - LOG_RING()
#define LOG_FEEDING_STATUS            (1)  // decoder feeder internal log - LOG_FEED()
#define LOG_DISPLAY_INFO_MANAGER      (1)  // display information manager log - LOG_DIMGR()
#define LOG_DISPLAY_INDEX_QUEUE       (1)  // display index queue internal log - LOG_IDXQUE()
#define LOG_BUFFER_CLEAR_STATUS       (1)  // buffer clear log - LOG_BUFCLR()

#ifdef OMX_DEBUG_MODE
#define LINUX_PLATFORM
#endif

#if 0
#if defined(LINUX_PLATFORM)	// linux
#define LOGV(...)	{tcc_printf(T_DEFAULT "[%4d]", __LINE__);tcc_printf(__VA_ARGS__);tcc_printf("\n" T_DEFAULT);}
#define LOGD(...)	{tcc_printf(TC_CYAN "[%4d]", __LINE__);tcc_printf(__VA_ARGS__);tcc_printf("\n" T_DEFAULT);}
#define LOGI(...)	{tcc_printf(TC_GREEN "[%4d]", __LINE__);tcc_printf(__VA_ARGS__);tcc_printf("\n" T_DEFAULT);}
#define LOGW(...)	{tcc_printf(TC_MAGENTA "[%4d]", __LINE__);tcc_printf(__VA_ARGS__);tcc_printf("\n" T_DEFAULT);}
#define LOGE(...)	{tcc_printf(TC_RED "[%4d]", __LINE__);tcc_printf(__VA_ARGS__);tcc_printf("\n" T_DEFAULT);}
#else
#define LOGV(...)
#define LOGD(...)
#define LOGI(...)
#define LOGW(...)
#define LOGE(...)
#endif
#endif
#if defined(TCC_LLOG_ENABLE)
#define LOG_TAG	"OMX_TCC_VIDEODEC"
#include <log/log.h>
#define OMX_LOGD ALOGD
#else
#define OMX_LOGD(fmt, ...)
#endif

#define INFO(...)  LOGI("[INFO] "__VA_ARGS__)
#define WARN(...)
#define ERROR(...)  LOGE("[ERROR] "__VA_ARGS__)
#define ASSERT(cond)  // assertion
#define DBGM(...)     // debug message
#define DBGL()        // line printing
#define DBGV(v)       // value printing
#define DBGT()        // line/time printing
#define DBGB(bin, len) // binary printing
#define CLOG(TAG, ...)

#define LOG_STEP(...)
#define LOG_INPUT(...)
#define LOG_SKIP(...)
#define LOG_DEC(...)
#define LOG_DECE(...)
#define LOG_OUT(...)
#define LOG_IIQUE(...)
#define LOG_IIMGE(...)
#define LOG_RING(...)
#define LOG_FEED(...)
#define LOG_DIMGR(...)
#define LOG_IDXQUE(...)
#define LOG_BUFCLR(...)

#if DEBUG_MESSAGE_ENABLE
#define __VDEC_LOGV(...)    LOGV(TC_BOLD __VA_ARGS__)
#define __VDEC_LOGD(...)    LOGD(TC_BOLD __VA_ARGS__)
#define __VDEC_LOGI(...)    LOGI(TC_BOLD __VA_ARGS__)
#define __VDEC_LOGW(...)    LOGW(TC_BOLD __VA_ARGS__)
#define __VDEC_LOGE(...)    LOGE(TC_BOLD __VA_ARGS__)

#undef INFO
#define INFO(...)	__VDEC_LOGI("[INFO] "__VA_ARGS__)

#undef WARN
#define WARN(...)  __VDEC_LOGW("[WARN] "__VA_ARGS__)

#undef ERROR
#define ERROR(...)  __VDEC_LOGE("[ERROR] "__VA_ARGS__)

#undef DBGM
#define DBGM(...)   __VDEC_LOGD("[VDEC_DBG] "__VA_ARGS__)

#undef DBGL
#define DBGL()      __VDEC_LOGE("[DBG_LINE] %d line", __LINE__)

#undef DBGV
#define DBGV(v)     __VDEC_LOGE("[DBG_VAL][%4d line] 0x%08X (%d)", __LINE__, v, v)

#undef DBGB
#if defined(LINUX_PLATFORM)	// linux
	#define DBGB(bin, len)   {\
		OMX_S32 __idx;\
		OMX_S32 __len = len;\
		tcc_printf(TC_CYAN "[DBG_BIN][LINE: %4d]", __LINE__);\
		for(__idx = 0; __idx < __len; __idx++) {\
			if( (__idx % 32) == 0 ) {tcc_printf("\n");}\
			if( (__idx % 4) == 0 ) {tcc_printf("  ");}\
			tcc_printf("%02X ", ((OMX_U8*)bin)[__idx]);\
		}\
		tcc_printf(T_DEFAULT "\n");\
	}
#else
	#define DBGB(bin, len)
#endif

#if CONDITIONAL_LOG_ENABLE

#if defined(LINUX_PLATFORM)	// linux
static const char *gs_szDbgMsgColor[] = {T_DEFAULT, TC_YELLOW, TC_CYAN, TC_GREEN, TC_MAGENTA, TC_RED};
#endif

#if defined(LINUX_PLATFORM)	// linux
	#define VDEC_CLOG(CLOG_TAG, level, ...) {\
		if( gs_iDbgmsgEnable ){\
			char *env = getenv("CLOG_VPU_"#CLOG_TAG);\
			int idx = 0;\
			if( env ) {idx = (int)(env[0] - '0');}\
			if( idx > 5 ) {idx = 5;}\
			if( level ) {idx = level;}\
			if( (idx > 0) && (idx <= 5) ) {\
				unsigned int uidx = (unsigned int)idx;\
				tcc_printf(TC_BOLD "%s", gs_szDbgMsgColor[uidx]);\
				tcc_printf("[LOG_"#CLOG_TAG"] "__VA_ARGS__);\
				tcc_printf(T_DEFAULT "\n");\
			}\
		}\
	}
#else
	#define VDEC_CLOG(CLOG_TAG, leve, ...)
#endif

#undef CLOG
#if defined(LINUX_PLATFORM)	// linux
	#define CLOG(TAG, ...) {\
		char *env = getenv("CLOG_GP_"#TAG);\
		int idx = 0;\
		if( env ) {idx = (int)(env[0] - '0');}\
		if( idx > 5 ) {idx = 5;}\
		if( (idx > 0) && (idx <= 5) ) {\
			unsigned int uidx = (unsigned int)idx;\
			tcc_printf(TC_BOLD "%s", gs_szDbgMsgColor[idx]);\
			tcc_printf("[LOG_"#TAG"] "__VA_ARGS__);\
			tcc_printf(T_DEFAULT "\n");\
		}\
	}
#else
	#define CLOG(TAG, ...)
#endif

#define VDEC_LOGV    VDEC_CLOG
#define VDEC_LOGD    VDEC_CLOG
#define VDEC_LOGI    VDEC_CLOG
#define VDEC_LOGW    VDEC_CLOG
#define VDEC_LOGE    VDEC_CLOG

#else //CONDITIONAL_LOG_ENABLE

#define VDEC_LOGV(CLOG_TAG, cat, ...)	__VDEC_LOGV("[LOG_"#CLOG_TAG"] "__VA_ARGS__)
#define VDEC_LOGD(CLOG_TAG, cat, ...)	__VDEC_LOGD("[LOG_"#CLOG_TAG"] "__VA_ARGS__)
#define VDEC_LOGI(CLOG_TAG, cat, ...)	__VDEC_LOGI("[LOG_"#CLOG_TAG"] "__VA_ARGS__)
#define VDEC_LOGW(CLOG_TAG, cat, ...)	__VDEC_LOGW("[LOG_"#CLOG_TAG"] "__VA_ARGS__)
#define VDEC_LOGE(CLOG_TAG, cat, ...)	__VDEC_LOGE("[LOG_"#CLOG_TAG"] "__VA_ARGS__)
#define CLOG(TAG, ...)                  __VDEC_LOGD("[LOG_"#TAG"] "__VA_ARGS__)

#endif


static OMX_S64 g_llPrevLogTime;
#undef DBGT
#define DBGT() {\
	struct timeval time;\
	OMX_S64 curr_time;\
	gettimeofday(&time, NULL);\
	curr_time = (OMX_S64)1000000*time.tv_sec + time.tv_usec;\
	VDEC_LOGD("[DBG_TIME][%4d line] latency: %8lld us", __LINE__, curr_time-g_llPrevLogTime);\
	g_llPrevLogTime = curr_time;\
}

#if LOG_OMX_STEP_INFO
#undef LOG_STEP
#define LOG_STEP(...)   VDEC_LOGD(STEP, 0, __VA_ARGS__)
#endif

#if LOG_INPUT_BUFF_INFO
#undef LOG_INPUT
#define LOG_INPUT(...)   VDEC_LOGV(INPUT, 0, __VA_ARGS__)
#endif

#if LOG_FRAME_SKIP_MODE
#undef LOG_SKIP
#define LOG_SKIP(...)    VDEC_LOGI(SKIP, 0, __VA_ARGS__)
#endif

#if LOG_DECODE_STATUS
#undef LOG_DEC
#define LOG_DEC(...)     VDEC_LOGI(DEC, 0, __VA_ARGS__)
#undef LOG_DECE
#define LOG_DECE(...)    VDEC_LOGE(DEC, 4, __VA_ARGS__)
#endif

#if LOG_OUTPUT_STATUS
#undef LOG_OUT
#define LOG_OUT(...)     VDEC_LOGD(OUT, 0, __VA_ARGS__)
#endif

#if LOG_INPUT_INFO_QUEUE
#undef LOG_IIQUE
#define LOG_IIQUE(...)   VDEC_LOGD(IIQUE, 0, __VA_ARGS__)
#endif

#if LOG_INPUT_INFO_MANAGEMENT
#undef LOG_IIMGE
#define LOG_IIMGE(...)   VDEC_LOGI(IIMGE, 0, __VA_ARGS__)
#endif

#if LOG_RING_BUFFER_STATUS
#undef LOG_RING
#define LOG_RING(...)    VDEC_LOGI(RING, 0, __VA_ARGS__)
#endif

#if LOG_FEEDING_STATUS
#undef LOG_FEED
#define LOG_FEED(...)    VDEC_LOGD(FEED, 0, __VA_ARGS__)
#endif

#if LOG_DISPLAY_INFO_MANAGER
#undef LOG_DIMGR
#define LOG_DIMGR(...)   VDEC_LOGD(DIMGR, 0, __VA_ARGS__)
#endif

#if LOG_DISPLAY_INDEX_QUEUE
#undef LOG_IDXQUE
#define LOG_IDXQUE(...)  VDEC_LOGI(IDXQUE, 0, __VA_ARGS__)
#endif

#if LOG_BUFFER_CLEAR_STATUS
#undef LOG_BUFCLR
#define LOG_BUFCLR(...)  VDEC_LOGI(BUFCLR, 0, __VA_ARGS__)
#endif

#undef ASSERT
#define ASSERT(cond) \
	if(!(cond)){\
		LOGE("ASSERTION FAILED!"); \
		LOGE("  - Position: %s (%d line) : %s()", __FILE__, __LINE__, __FUNCTION__); \
		LOGE("  - Expression: " #cond); \
	}

#if VDEC_INPUT_DUMP_ENABLE
#undef VDEC_PROFILER_ENABLE
#define VDEC_PROFILER_ENABLE 0
#undef VDEC_LOGGER_ENABLE
#define VDEC_LOGGER_ENABLE 0

int
indump_vdec_func(
	vdec_private_t *pstVDecPrivate,
    int    nOpCode, int*   pHandle, void*  pParam1, void*  pParam2,
    char*  pszProfileTag, int    nCallLine
	)
{
	OMX_S32 ret = 0;

	if (nOpCode == VDEC_INIT) {
		FILE *fp = fopen("/sd/vdec_dump.es", "wb");
		if (fp){
			fclose(fp);
		}
	}
	else if (nOpCode == VDEC_DECODE) {
		FILE *fp = fopen("/sd/vdec_dump.es", "a+b");
		if (fp) {
			fwrite(&pstVDecPrivate->stVDecInput.m_iInpLen, 1, 4, fp);
			fwrite(pstVDecPrivate->stVDecInput.m_pInp[VA], 1, pstVDecPrivate->stVDecInput.m_iInpLen, fp);
			fclose(fp);
		}
	}

	ret = pstVDecPrivate->pfVDecFunc(nOpCode, pHandle, pParam1, pParam2, pstVDecPrivate->pVDecInstance);

	return ret;
}

#undef VDEC_FUNC
#define VDEC_FUNC(__ptr_vdec_private, __opcode, __ptr_handle, __ptr_param1, __ptr_param2) \
		indump_vdec_func(__ptr_vdec_private, __opcode, __ptr_handle, __ptr_param1, __ptr_param2, #__opcode, __LINE__)
#endif //VDEC_INPUT_DUMP_ENABLE

#if VDEC_PROFILER_ENABLE
#undef VDEC_LOGGER_ENABLE
#define VDEC_LOGGER_ENABLE 0

int32_t
profile_vdec_func(
    vdec_private_t *pstVDecPrivate,
    int32_t nOpCode, int32_t*   pHandle, unsigned long*  pParam1, unsigned long*  pParam2,
    char*  pszProfileTag, int32_t    nCallLine
    )
{
	OMX_S32 ret = 0;
	struct timeval start;
	struct timeval end;

	gettimeofday(&start, NULL);
	ret = pstVDecPrivate->pfVDecFunc(nOpCode, pHandle, pParam1, pParam2, pstVDecPrivate->pVDecInstance);
	gettimeofday(&end, NULL);

	INFO("[PROFILE][%3d][%s] time: %lld us",
		nCallLine, pszProfileTag,
		((OMX_S64)1000000*end.tv_sec + end.tv_usec)-((OMX_S64)1000000*start.tv_sec + start.tv_usec));

	return ret;
}

#undef VDEC_FUNC
#define VDEC_FUNC(__ptr_vdec_private, __opcode, __ptr_handle, __ptr_param1, __ptr_param2) \
		profile_vdec_func(__ptr_vdec_private, __opcode, __ptr_handle, __ptr_param1, __ptr_param2, #__opcode, __LINE__)
#endif //VDEC_PROFILER_ENABLE

#if VDEC_LOGGER_ENABLE
#undef VDEC_PROFILER_ENABLE
#define VDEC_PROFILER_ENABLE 0
int32_t
logger_vdec_func(
    vdec_private_t *pstVDecPrivate,
    int32_t    nOpCode, int32_t*   pHandle, unsigned long*  pParam1, unsigned long*  pParam2,
    char*  pszOpCode, int32_t    nCallLine
    )
{
	OMX_S32 ret = 0;
	struct timeval start;
	struct timeval end;

	LOGE("[VDEC_CALL][%3d][%s] - IN", nCallLine, pszOpCode);
	ret = pstVDecPrivate->pfVDecFunc(nOpCode, pHandle, pParam1, pParam2, pstVDecPrivate->pVDecInstance);
	LOGE("[VDEC_CALL][%3d][%s] - OUT (ret: %ld)", nCallLine, pszOpCode, ret);

	return ret;
}

#undef VDEC_FUNC
#define VDEC_FUNC(__ptr_vdec_private, __opcode, __ptr_handle, __ptr_param1, __ptr_param2) \
		logger_vdec_func(__ptr_vdec_private, __opcode, __ptr_handle, __ptr_param1, __ptr_param2, #__opcode, __LINE__)

#endif

#endif //DEBUG_MESSAGE_ENABLE

////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//  Definitions
//
//
#if defined(TCC_VPU_C7_INCLUDE) || defined(TCC_VPU_C5_INCLUDE)
//#define TCC_EXT_INCLUDED
#define TCC_AVS_INCLUDED
#define TCC_THEORA_INCLUDED
#endif

/* input buffer setup */
#ifdef TCC_VPU_4K_D2_INCLUDE
#  define HEVC_INPUT_BUFFER_SIZE        (OMX_U32)(1048576*3)
#  define HEVC_INPUT_BUFFER_COUNT_MIN   (3)
#  define VP9_INPUT_BUFFER_SIZE         (OMX_U32)(1048576*3)
#  define VP9_INPUT_BUFFER_COUNT_MIN    (3)
#else
# ifdef TCC_HEVC_INCLUDE
#  define HEVC_INPUT_BUFFER_SIZE        (OMX_U32)(1048576*3)
#  define HEVC_INPUT_BUFFER_COUNT_MIN   (3)
# endif
#endif
#define INPUT_BUFFER_SIZE              (OMX_U32)(1048576*2)
#define INPUT_BUFFER_COUNT_MIN         (4)

/* output buffer setup */
#define OUTPUT_BUFFER_SIZE          (OMX_U32)(1024)	// for physical address mode
#ifdef TCC_VPU_4K_D2_INCLUDE
# define HEVC_OUTPUT_WIDTH_MAX      (4096)
# define HEVC_OUTPUT_HEIGHT_MAX     (2160)
# define VP9_OUTPUT_WIDTH_MAX       (4096)
# define VP9_OUTPUT_HEIGHT_MAX      (2160)
#else
# ifdef TCC_HEVC_INCLUDE
#  if defined(TCC_803X_INCLUDE)
#   define HEVC_OUTPUT_WIDTH_MAX       (4096)
#   define HEVC_OUTPUT_HEIGHT_MAX      (2160)
#  elif defined(TCC_897X_INCLUDE)
#   define HEVC_OUTPUT_WIDTH_MAX       (3840)
#   define HEVC_OUTPUT_HEIGHT_MAX      (2160)
#  else
#   define HEVC_OUTPUT_WIDTH_MAX       (3840)
#   define HEVC_OUTPUT_HEIGHT_MAX      (2160)
#  endif
# endif
#endif

#define OUTPUT_WIDTH_MAX	        (1920)
#define OUTPUT_HEIGHT_MAX	        (1088)

#define OUTPUT_BUFFER_COUNT_MIN     (4)  // for delayed buffer flushing
#define OUTPUT_FRAME_RATE_DEFAULT   (30000)
#define OUTPUT_FORMAT_DEFAULT       (OMX_COLOR_FormatYUV420SemiPlanarTc)
#define OUTPUT_BUFFER_ACTUAL_MIN    (4)  // buffering max between vdec and sink

/* decoding process setup */
#define SEQ_INIT_FAIL_MAX_INIT					(300)
#define SEQ_HEADER_SEARCH_MAX						(300)
#define DECODING_FAIL_MAX_INIT					(30)
#define VSYNC_DISP_CHECK_COUNT_MAX			(100)
#define INPUT_INFO_QUEUE_INIT_SIZE			(64)
#define THUMBNAIL_DECODING_FAIL_MAX			(20)
#define SEQHEAD_LENGTH_MAX							(1024*500)
#define EMPTY_SIZE_MIN									(1024*2)   // decoder ring-buffer margine between read ptr and write ptr
#define FEED_LIMIT_TIMEDIFF_INIT				(300000) // init as 300ms
#define FRAME_RATE_FOR_FEED_LIMIT_MIN		(15000)  // 15 fps

/* I-frame search */
#define VDEC_FRAME_SKIP_MAX							(350)	   // after I frame search
#define VDEC_OUTPUT_IDR_FAIL_MAX				(20000)  // after I frame search

/* AVC decode only mode */
#define DECODE_ONLY_SKIP_MAX			(VPU_BUFF_COUNT*3)  // decoded frame skip max
#define DECODE_ONLY_ERROR_MAX			(60)                // decoded frame skip max
#define AVC_IDR_SLICE_SCAN_MAX		(20)

/* VPU specific header */
#define VPU_FEED_LIMIT_FACTOR		(5)

/* state flags */
#define STATE_RESOURCE_INITIALIZED               (1u<<0)
#define STATE_WAIT_RESOURCE_INIT                 (1u<<1)
#define STATE_DECODER_OPENED                     (1u<<2)
#define STATE_VDEC_INITIALIZED                   (1u<<3)
#define STATE_WAIT_DECODED_KEYFRAME              (1u<<4)
#define STATE_WAIT_DISPLAY_KEYFRAME              (1u<<5)
#define STATE_IN_DEC_RESET_PROCESS               (1u<<6)
#define STATE_CLEAR_DISPLAY_BUFFER               (1u<<7)
#define STATE_SEQUENCE_HEADER_FOUND              (0x100u)
#define STATE_FRAME_RATE_UPDATE                  (0x100u<<1)
#define STATE_MVC_OUTPUT_ENABLE                  (0x100u<<2) // currently not used
#define STATE_SKIP_OUTPUT_B_FRAME                (0x100u<<3)
#define STATE_VDEC_BUFFER_PROTECTION             (0x100u<<4)
#define STATE_OUTPUT_PORT_FLUSHED                (0x100u<<5) // error handling
#define STATE_RESOLUTION_CHANGING_WITH_REINIT    (0x100u<<6) // resolution change
#define STATE_RESOLUTION_CHANGING_WITH_CROP      (0x100u<<7) // resolution change
#define STATE_NOT_DEFINED01                      (0x10000u) // not used
#define STATE_READY_TO_RESET                     (0x10000u<<1) // stream conversion
#define STATE_STREAM_CONVERTION_NEEDED           (0x10000u<<2) // not used with STATE_SEQHEAD_ATTACHMENT_NEEDED at the same time
#define STATE_SEQHEAD_ATTACHMENT_NEEDED          (0x10000u<<3) // not used with STATE_STREAM_CONVERTION_NEEDED at the same time
#define STATE_SLICE_COUNTING                     (0x10000u<<4) // slice counting/merging (MODE_SLICE_MERGING)
#define STATE_SLICE_COUNTED                      (0x10000u<<5) // slice counting/merging (MODE_SLICE_MERGING)
#define STATE_SLICE_MERGING                      (0x10000u<<6) // slice counting/merging (MODE_SLICE_MERGING)
#define STATE_FLUSHING_DELAYED_FRAME             (0x10000u<<7)
#define STATE_WAIT_CHANGED_OUTPUT                (0x1000000u)
#define STATE_FRAME_DISCONTINUITY                (0x1000000u<<1)
#define STATE_DETECT_ERROR_MACROBLOCK            (0x1000000u<<2)

#define SET_STATE(__p_vdec_private, __flag)    (((__p_vdec_private)->ulStateFlags) |= (__flag))
#define CLEAR_STATE(__p_vdec_private, __flag)  (((__p_vdec_private)->ulStateFlags) &= ~(__flag))
#define CHECK_STATE(__p_vdec_private, __flag)  (((__p_vdec_private)->ulStateFlags) & (__flag))
#define SHIFT_AND_MERGE(value1,value2) ((value1*2)<<7 |( value2))

/* mode flags */
#define MODE_DECODED_KEYFRAME_OUTPUT             (1u<<0 )
#define MODE_DECODED_FRAME_OUTPUT                (1u<<1 )
#define MODE_PHYSICAL_ADDRESS_OUTPUT             (1u<<2 )
#define MODE_COMPRESSED_OUTPUT                   (1u<<3 )
#define MODE_SLICE_MERGING                       (1u<<4 ) // slice counting/merging
#define MODE_MB_ERROR_REPORTING                  (1u<<5 ) // error handling
#define MODE_DECODING_ERROR_REPORTING            (1u<<6 ) // error handling
#define MODE_DECODING_ERROR_REPORTING_REPEAT     (1u<<7 ) // error handling
#define MODE_USERDATA_HANDLING                   (0x100u )
#define MODE_RINGBUFFER_MODE	                   (0x100u<<1)
#define MODE_I_FRAME_ONLY_OUTPUT                 (0x100u<<2)
#define MODE_EXT_FUNC_NO_BUFFER_DELAY            (0x100u<<3) // 170724.1.no-buffer-delay
#define MODE_EXT_FUNC_USE_CQ_1                   (0x100u<<4) // for 4kd2 debug

#define SET_MODE(__p_vdec_private, __flag)    (((__p_vdec_private)->ulModeFlags) |= (__flag))
#define CLEAR_MODE(__p_vdec_private, __flag)  (((__p_vdec_private)->ulModeFlags) &= ~(__flag))
#define CHECK_MODE(__p_vdec_private, __flag)  (((__p_vdec_private)->ulModeFlags) & (__flag))

/* error flags */
#define ERROR_UNDEFINED                          (1u<<0 )
#define ERROR_SPEC_OUT                           (1u<<1 )
#define ERROR_OUT_OF_MEMORY                      (1u<<2 )
#define ERROR_INVALID_STRIDE                     (1u<<3 )
#define ERROR_INPUT_BUFFER_REMAINNING            (1u<<4 )
#define ERROR_INSUFFICIENT_BITSTREAM             (1u<<5 ) //

#define SET_ERROR(__p_vdec_private, __flag)    (((__p_vdec_private)->ulErrorFlags) |= (__flag))
#define CLEAR_ERROR(__p_vdec_private, __flag)  (((__p_vdec_private)->ulErrorFlags) &= ~(__flag))
#define CHECK_ERROR(__p_vdec_private, __flag)  (((__p_vdec_private)->ulErrorFlags) & (__flag))

/* decoding result flags */
#define DECODING_SUCCESS_FRAME    (0x0001u)
#define DECODING_SUCCESS_FIELD    (0x0002u)
#define DECODING_SUCCESS          (0x0003u)
#define DECODING_SUCCESS_SKIPPED  (0x0004u)
#define DISPLAY_OUTPUT_SUCCESS    (0x0010u)
#define DECODED_OUTPUT_SUCCESS    (0x0020u)
#define OUTPUT_SUCCESS            (0x0030u)
#define DECODING_BUFFER_FULL      (0x0100u)
#define RESOLUTION_CHANGED        (0x1000u)

/* Resolution Change */
#define ERROR_INVALID_BUFFER_STATE      (-1001)
#define ERROR_INVALID_OUTPUT_FRAME      (-1002)
#define ERROR_SKIP_OUTPUT_FRAME         (-1003)
#define SUCCESS_RESOLUTION_CHANGED      (1001)


#if defined(TC_SECURE_MEMORY_COPY)
#define TMEM_DEVICE		"/dev/tmem"

extern int
TC_SecureMemoryCopy(
	unsigned int paTarget,
	unsigned int paSource,
	unsigned int nSize
);

extern int
TC_SecureScanFilterSpace(
	unsigned int paRingBase,   //[IN] physical address
	int           nRingSize,   //[IN] offset
	int           nScanStart,  //[IN] offset
	int           nScanEnd,    //[IN] offset
	int           bIsAVC,      //[IN] boolean
	int          *plSpaceEnd   //[OUT] offset
);
#endif



////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//  Codec / Decoder tables
//
//
typedef struct VIDEO_PROFILE_LEVEL
{
    OMX_S32  nProfile;
    OMX_S32  nLevel;
} VIDEO_PROFILE_LEVEL_TYPE;

/* H.263 Supported Levels & profiles */
static const VIDEO_PROFILE_LEVEL_TYPE SupportedH263ProfileLevels[] = {
  {OMX_VIDEO_H263ProfileBaseline, OMX_VIDEO_H263Level10},
  {OMX_VIDEO_H263ProfileBaseline, OMX_VIDEO_H263Level20},
  {OMX_VIDEO_H263ProfileBaseline, OMX_VIDEO_H263Level30},
  {OMX_VIDEO_H263ProfileBaseline, OMX_VIDEO_H263Level40},
  {OMX_VIDEO_H263ProfileBaseline, OMX_VIDEO_H263Level45},
  {OMX_VIDEO_H263ProfileBaseline, OMX_VIDEO_H263Level50},
  {OMX_VIDEO_H263ProfileBaseline, OMX_VIDEO_H263Level60},
  {OMX_VIDEO_H263ProfileBaseline, OMX_VIDEO_H263Level70},
  {OMX_VIDEO_H263ProfileISWV2, OMX_VIDEO_H263Level10},
  {OMX_VIDEO_H263ProfileISWV2, OMX_VIDEO_H263Level20},
  {OMX_VIDEO_H263ProfileISWV2, OMX_VIDEO_H263Level30},
  {OMX_VIDEO_H263ProfileISWV2, OMX_VIDEO_H263Level40},
  {OMX_VIDEO_H263ProfileISWV2, OMX_VIDEO_H263Level50},
  {OMX_VIDEO_H263ProfileISWV2, OMX_VIDEO_H263Level60},
  {OMX_VIDEO_H263ProfileISWV2, OMX_VIDEO_H263Level70},
  {-1, -1}};

/* MPEG4 Supported Levels & profiles */
static const VIDEO_PROFILE_LEVEL_TYPE SupportedMPEG4ProfileLevels[] ={
  {OMX_VIDEO_MPEG4ProfileSimple, OMX_VIDEO_MPEG4Level0},
  {OMX_VIDEO_MPEG4ProfileSimple, OMX_VIDEO_MPEG4Level0b},
  {OMX_VIDEO_MPEG4ProfileSimple, OMX_VIDEO_MPEG4Level1},
  {OMX_VIDEO_MPEG4ProfileSimple, OMX_VIDEO_MPEG4Level2},
  {OMX_VIDEO_MPEG4ProfileSimple, OMX_VIDEO_MPEG4Level3},
  {OMX_VIDEO_MPEG4ProfileSimple, OMX_VIDEO_MPEG4Level4},
  {OMX_VIDEO_MPEG4ProfileSimple, OMX_VIDEO_MPEG4Level4a},
  {OMX_VIDEO_MPEG4ProfileSimple, OMX_VIDEO_MPEG4Level5},
  {OMX_VIDEO_MPEG4ProfileAdvancedSimple, OMX_VIDEO_MPEG4Level0},
  {OMX_VIDEO_MPEG4ProfileAdvancedSimple, OMX_VIDEO_MPEG4Level0b},
  {OMX_VIDEO_MPEG4ProfileAdvancedSimple, OMX_VIDEO_MPEG4Level1},
  {OMX_VIDEO_MPEG4ProfileAdvancedSimple, OMX_VIDEO_MPEG4Level2},
  {OMX_VIDEO_MPEG4ProfileAdvancedSimple, OMX_VIDEO_MPEG4Level3},
  {OMX_VIDEO_MPEG4ProfileAdvancedSimple, OMX_VIDEO_MPEG4Level4},
  {OMX_VIDEO_MPEG4ProfileAdvancedSimple, OMX_VIDEO_MPEG4Level5},
  {-1,-1}};

/* AVC Supported Levels & profiles */
static const VIDEO_PROFILE_LEVEL_TYPE SupportedAVCProfileLevels[] ={
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
  {OMX_VIDEO_AVCProfileBaseline, OMX_VIDEO_AVCLevel32},
  {OMX_VIDEO_AVCProfileBaseline, OMX_VIDEO_AVCLevel4},
  {OMX_VIDEO_AVCProfileBaseline, OMX_VIDEO_AVCLevel41},
  {OMX_VIDEO_AVCProfileBaseline, OMX_VIDEO_AVCLevel42},
  {OMX_VIDEO_AVCProfileBaseline, OMX_VIDEO_AVCLevel5},
  {OMX_VIDEO_AVCProfileBaseline, OMX_VIDEO_AVCLevel51},
  {OMX_VIDEO_AVCProfileMain, OMX_VIDEO_AVCLevel1},
  {OMX_VIDEO_AVCProfileMain, OMX_VIDEO_AVCLevel1b},
  {OMX_VIDEO_AVCProfileMain, OMX_VIDEO_AVCLevel11},
  {OMX_VIDEO_AVCProfileMain, OMX_VIDEO_AVCLevel12},
  {OMX_VIDEO_AVCProfileMain, OMX_VIDEO_AVCLevel13},
  {OMX_VIDEO_AVCProfileMain, OMX_VIDEO_AVCLevel2},
  {OMX_VIDEO_AVCProfileMain, OMX_VIDEO_AVCLevel21},
  {OMX_VIDEO_AVCProfileMain, OMX_VIDEO_AVCLevel22},
  {OMX_VIDEO_AVCProfileMain, OMX_VIDEO_AVCLevel3},
  {OMX_VIDEO_AVCProfileMain, OMX_VIDEO_AVCLevel31},
  {OMX_VIDEO_AVCProfileMain, OMX_VIDEO_AVCLevel32},
  {OMX_VIDEO_AVCProfileMain, OMX_VIDEO_AVCLevel4},
  {OMX_VIDEO_AVCProfileMain, OMX_VIDEO_AVCLevel41},
  {OMX_VIDEO_AVCProfileMain, OMX_VIDEO_AVCLevel42},
  {OMX_VIDEO_AVCProfileMain, OMX_VIDEO_AVCLevel5},
  {OMX_VIDEO_AVCProfileMain, OMX_VIDEO_AVCLevel51},
  {OMX_VIDEO_AVCProfileHigh, OMX_VIDEO_AVCLevel1},
  {OMX_VIDEO_AVCProfileHigh, OMX_VIDEO_AVCLevel1b},
  {OMX_VIDEO_AVCProfileHigh, OMX_VIDEO_AVCLevel11},
  {OMX_VIDEO_AVCProfileHigh, OMX_VIDEO_AVCLevel12},
  {OMX_VIDEO_AVCProfileHigh, OMX_VIDEO_AVCLevel13},
  {OMX_VIDEO_AVCProfileHigh, OMX_VIDEO_AVCLevel2},
  {OMX_VIDEO_AVCProfileHigh, OMX_VIDEO_AVCLevel21},
  {OMX_VIDEO_AVCProfileHigh, OMX_VIDEO_AVCLevel22},
  {OMX_VIDEO_AVCProfileHigh, OMX_VIDEO_AVCLevel3},
  {OMX_VIDEO_AVCProfileHigh, OMX_VIDEO_AVCLevel31},
  {OMX_VIDEO_AVCProfileHigh, OMX_VIDEO_AVCLevel32},
  {OMX_VIDEO_AVCProfileHigh, OMX_VIDEO_AVCLevel4},
  {OMX_VIDEO_AVCProfileHigh, OMX_VIDEO_AVCLevel41},
  {OMX_VIDEO_AVCProfileHigh, OMX_VIDEO_AVCLevel42},
  {OMX_VIDEO_AVCProfileHigh, OMX_VIDEO_AVCLevel5},
  {OMX_VIDEO_AVCProfileHigh, OMX_VIDEO_AVCLevel51},
  {-1,-1}};

/* MPEG2 Supported Levels & profiles */
static const VIDEO_PROFILE_LEVEL_TYPE SupportedMPEG2ProfileLevels[] = {
  {OMX_VIDEO_MPEG2ProfileSimple, OMX_VIDEO_MPEG2LevelLL},
  {OMX_VIDEO_MPEG2ProfileSimple, OMX_VIDEO_MPEG2LevelML},
  {OMX_VIDEO_MPEG2ProfileSimple, OMX_VIDEO_MPEG2LevelH14},
  {OMX_VIDEO_MPEG2ProfileSimple, OMX_VIDEO_MPEG2LevelHL},
  {OMX_VIDEO_MPEG2ProfileMain, OMX_VIDEO_MPEG2LevelLL},
  {OMX_VIDEO_MPEG2ProfileMain, OMX_VIDEO_MPEG2LevelML},
  {OMX_VIDEO_MPEG2ProfileMain, OMX_VIDEO_MPEG2LevelHL},
  {OMX_VIDEO_MPEG2ProfileMain, OMX_VIDEO_MPEG2LevelHL},
  {-1, -1}};

/* HEVC Supported Levels & profiles */
VIDEO_PROFILE_LEVEL_TYPE SupportedHEVCProfileLevels[] = {
  {OMX_VIDEO_HEVCProfileMain, OMX_VIDEO_HEVCMainTierLevel5 },
  {OMX_VIDEO_HEVCProfileMain, OMX_VIDEO_HEVCHighTierLevel5 },
  {OMX_VIDEO_HEVCProfileMain10, OMX_VIDEO_HEVCMainTierLevel51 },
  {OMX_VIDEO_HEVCProfileMain10, OMX_VIDEO_HEVCHighTierLevel51 },
//#if !defined(CODE_FOR_CTS)
  {OMX_VIDEO_HEVCProfileMain10HDR10, OMX_VIDEO_HEVCMainTierLevel51 },
//#endif
  {-1, -1}};

/* VP8 Supported Levels & profiles */
static const VIDEO_PROFILE_LEVEL_TYPE SupportedVP8ProfileLevels[] = {
  {-1, -1}};

#if defined(SUPPORT_HDR_COLOR_CHANGE)
/* VP9 Supported Levels & profiles */
VIDEO_PROFILE_LEVEL_TYPE SupportedVP9ProfileLevels[] = {
  {OMX_VIDEO_VP9Profile0, OMX_VIDEO_VP9Level5},
  {OMX_VIDEO_VP9Profile2, OMX_VIDEO_VP9Level5},
  {OMX_VIDEO_VP9Profile2HDR, OMX_VIDEO_VP9Level5},
  //{OMX_VIDEO_VP9Profile3HDR, OMX_VIDEO_VP9Level6},
  {-1, -1} };
#endif

#if defined(TC_SECURE_MEMORY_COPY)
static int32_t st_func_get_pmap_info(const char *pstrName, VideoDec_Pmap_T *ptPmap)
{
	int32_t hFd;
	int32_t iReadByte;
	int32_t iMatches;
	uint32_t iAddress;
	uint32_t iSize;

	const char *p;
	char strTemp[128];

	unsigned char ucBuffer[2048];

	hFd = open("/proc/pmap", O_RDONLY);

	if(hFd < 0)
	{
		LOGE("[%s:%d] open fail - /proc/pmap\n", __func__, __LINE__);
		return -1;
	}

	read(hFd, ucBuffer, 2048);

	close(hFd);

	p = (char *)ucBuffer;

	while(1)
	{
		iMatches = sscanf_s(p, "0x%x 0x%x %s", &iAddress, &iSize, strTemp,sizeof(strTemp));

		if(iMatches == 3 && !strncmp(pstrName, strTemp, sizeof(strTemp)))
		{
			LOGE("PMAP '%s' (addr=0x%x size=0x%x)\n", pstrName, iAddress, iSize);

			ptPmap->iAddress 	= iAddress;
			ptPmap->iSize 		= iSize;
			return 0;
		}

		p = strchr(p, '\n');

		if(p == NULL){
			break;
		}

		p++;
	}

	return -1;
}
#endif
//===============================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//  Debugging helper
//
//
static
const char*
GetFrameTypeString(
	OMX_S32 lCodecType,
	OMX_S32 lPictureType,
	OMX_S32 lPictureStructure
	)
{
	const char *sType;
	switch ( lCodecType )
	{
	case STD_MPEG2 :
		switch( lPictureType ) {
#ifdef PIC_TYPE_IDR
		case PIC_TYPE_IDR:
#endif
		case PIC_TYPE_I: sType = "I"; break;
		case PIC_TYPE_P: sType = "P"; break;
		case PIC_TYPE_B: sType = "B"; break;
		default:         sType = "D"; break;//D_TYPE
		}
		break;

	case STD_MPEG4 :
		switch( lPictureType ) {
#ifdef PIC_TYPE_IDR
		case PIC_TYPE_IDR:
#endif
		case PIC_TYPE_I:     sType = "I "; break;
		case PIC_TYPE_P:     sType = "P "; break;
		case PIC_TYPE_B:     sType = "B "; break;
		case PIC_TYPE_B_PB:  sType = "pB"; break; //MPEG-4 Packed PB-frame
		default:             sType = "S "; break; //S_TYPE
		}
		break;

	case STD_VC1 :
		if( lPictureStructure == 3 ) {
			switch( lPictureType & 0x7 ) { // FIELD_INTERLACED(BOTTOM FIELD)
#ifdef PIC_TYPE_IDR
			case PIC_TYPE_IDR:
#endif
			case PIC_TYPE_I: sType = "BF_I   "; break; //TOP_FIELD = I
			case PIC_TYPE_P: sType = "BF_P   "; break; //TOP_FIELD = P
			case 2:          sType = "BF_BI  "; break; //TOP_FIELD = BI_TYPE
			case 3:          sType = "BF_B   "; break; //TOP_FIELD = B_TYPE
			case 4:          sType = "BF_SKIP"; break; //TOP_FIELD = SKIP_TYPE
			default:         sType = "BF_FORB"; break;//TOP_FIELD = FORBIDDEN
			}
		}
		else {
			#if !defined(TCC_892X_INCLUDE) && !defined(TCC_893X_INCLUDE)
			lPictureType >>= 3;
			#endif
			switch( lPictureType ) { //Frame or // FIELD_INTERLACED(TOP FIELD)
			case PIC_TYPE_I:  sType = "I   "; break;
			case PIC_TYPE_P:  sType = "P   "; break;
			case 2:           sType = "BI  "; break;
			case 3:           sType = "B   "; break;
			case 4:           sType = "SKIP"; break;
			default:          sType = "FORB"; break;//FORBIDDEN
			}
		}
		break;
	default:
		if( lPictureStructure ) {
			switch( lPictureType ) {
			case 0:  sType = "I/I"; break;
			case 1:  sType = "I/P"; break;
			case 2:  sType = "I/B"; break;
			case 8:  sType = "P/I"; break;
			case 9:  sType = "P/P"; break;
			case 10: sType = "P/B"; break;
			case 16: sType = "B/I"; break;
			case 17: sType = "B/P"; break;
			case 18: sType = "B/B"; break;
			default: sType = "U/U"; break;
			}
		}
		else
		{
			switch( lPictureType ) {
#ifdef PIC_TYPE_IDR
			case PIC_TYPE_IDR:  sType = "IDR"; break;
#endif
			case PIC_TYPE_I:    sType = "I  "; break;
			case PIC_TYPE_P:    sType = "P  "; break;
			case PIC_TYPE_B:    sType = "B  "; break;
			default:            sType = "U  "; break;//Unknown
			}
		}
		break;
	}
	return sType;
}

////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//	Input buffer information queue (dynamic size queue)
//
//
#if DECODER_RINGBUFFER_MODE

static OMX_BOOL InitInputQueue(input_queue_t *pstQue, OMX_S32 lInitSize);
static void DeinitInputQueue(input_queue_t *pstQue);
static void ClearInputQueue(input_queue_t *pstQue);
static OMX_S32 PushInputInfo(input_queue_t *pstQue, const input_info_t *pstInput);
static OMX_S32 GetInputInfo(input_queue_t *pstQue, input_info_t *pstInput);
static OMX_S32 ShowInputInfo(input_queue_t *pstQue, input_info_t *pstInput);
static OMX_S32 ShowLastInputInfo(input_queue_t *pstQue, input_info_t *pstInput); // for debugging
static OMX_S32 ClearFirstInputInfo(input_queue_t *pstQue);

static
OMX_BOOL
InitInputQueue(
	input_queue_t  *pstQue,
	OMX_S32         lInitSize
	)
{
	OMX_BOOL ret;
	if( pstQue->pstInfoRing ){
		TCC_free(pstQue->pstInfoRing);
	}

	memset(pstQue, 0, sizeof(input_queue_t));

	pstQue->pstInfoRing = TCC_malloc(sizeof(input_info_t) * lInitSize);
	if( pstQue->pstInfoRing == NULL )
	{
		DeinitInputQueue(pstQue);
		ERROR("CreatePTSQueue() - out of memory");
		ret = OMX_FALSE;
	}
	else
	{
		pstQue->lQueSize = lInitSize;
		ret = OMX_TRUE;
	}

	return ret;
}

static
void
DeinitInputQueue(
	input_queue_t  *pstQue
	)
{
	if( pstQue->pstInfoRing ){
		TCC_free( pstQue->pstInfoRing );
	}

	pstQue->pstInfoRing = NULL;
}

static
void
ClearInputQueue(
	input_queue_t  *pstQue
	)
{
	pstQue->lStartPos = 0;
	pstQue->lEndPos   = 0;
	pstQue->lQueCount = 0;
}

static
OMX_S32
PushInputInfo(
	input_queue_t      *pstQue,
	const input_info_t *pstInput
	)
{
	input_info_t  *p_ring = pstQue->pstInfoRing;
	OMX_S32        size   = pstQue->lQueSize;
	OMX_S32        count  = pstQue->lQueCount;
	OMX_S32        end    = pstQue->lEndPos;

#if 0	// This part will be modify in the near future.
	if( count ) {
		OMX_S32 last = end-1;

		if( last < 0 ){
			last = size-1;
		}

		if( p_ring[last].llTimestamp == pstInput->llTimestamp )
		{
			p_ring[last].lFilledLen += pstInput->lFilledLen;
			p_ring[last].pEndPtr = pstInput->pEndPtr;

			LOG_IIQUE("[PUSH ][QUECNT: %3ld] [PTS: %8ld] [REGION: %7ld ~ %7ld] [LENGTH: %7ld byte]"
					  , pstQue->lQueCount
					  , (OMX_S32)(p_ring[last].llTimestamp/1000)
					  , p_ring[last].pStartPtr - pstQue->pBasePtr
					  , p_ring[last].pEndPtr - pstQue->pBasePtr
					  , p_ring[last].lFilledLen
					  );

			return count;
		}
	}
#endif

	if( ++count > size )
	{
		input_info_t *p_tmp_ring;
		OMX_S32       cpy = pstQue->lStartPos;
		OMX_S32       new_size = size * 2;
		OMX_S32       i;

		INFO("Input information queue resizing");

		p_tmp_ring = TCC_malloc(sizeof(input_info_t) * new_size);
		if( p_tmp_ring == NULL ) {
			ERROR("PushInputInfo() - out of memory");
			return -1;
		}

		for(i = 0; i < count-1; i++, cpy++)
		{
			if(cpy >= size){
				cpy -= size;
			}
			p_tmp_ring[i] = p_ring[cpy];
		}

		size = new_size;
		end	= i;

		TCC_free(p_ring);
		p_ring = p_tmp_ring;
		pstQue->pstInfoRing = p_ring;
		pstQue->lQueSize = new_size;
		pstQue->lStartPos = 0;
		pstQue->lEndPos = end;
	}

	if( count == 1 ) {
		p_ring[pstQue->lEndPos++] = *pstInput;
		pstQue->lQueCount = 1;
	}
	else {
		p_ring[end] = *pstInput;

		if( ++pstQue->lEndPos >= size ){
			pstQue->lEndPos -= size;
		}
		pstQue->lQueCount = count;
	}

	LOG_IIQUE("[PUSH ][QUECNT: %3ld] [PTS: %8ld] [REGION: %7ld ~ %7ld] [LENGTH: %7ld byte]"
			  , pstQue->lQueCount
			  , (OMX_S32)(pstInput->llTimestamp/1000)
			  , pstInput->pStartPtr - pstQue->pBasePtr
			  , pstInput->pEndPtr - pstQue->pBasePtr
			  , pstInput->lFilledLen
			  );

	return count;
}

static
OMX_S32
GetInputInfo(
	input_queue_t  *pstQue,
	input_info_t   *pstInput
	)
{
	input_info_t  *p_ring = pstQue->pstInfoRing;
	OMX_S32        start  = pstQue->lStartPos;
	OMX_S32        size   = pstQue->lQueSize;
	OMX_S32        count  = pstQue->lQueCount;

	if( count <= 0 ) {
		ERROR("GetInputInfo() - internal error");
		return -1;
	}

	*pstInput = p_ring[start];

	start++;
	if( start >= size ){
		start -= size;
	}

	count--;
	if( count ){
		pstQue->lStartPos = start;
	}
	else {
		pstQue->lStartPos	= 0;
		pstQue->lEndPos		= 0;
	}

	pstQue->lQueCount = count;

	LOG_IIQUE("[GET  ][QUECNT: %3ld] [PTS: %8ld] [REGION: %7ld ~ %7ld] [LENGTH: %7ld byte]"
			  , pstQue->lQueCount
			  , (OMX_S32)(pstInput->llTimestamp/1000)
			  , pstInput->pStartPtr - pstQue->pBasePtr
			  , pstInput->pEndPtr - pstQue->pBasePtr
			  , pstInput->lFilledLen
			  );

	return count;
}

static
OMX_S32
ShowInputInfo(
	input_queue_t  *pstQue,
	input_info_t   *pstInput
	)
{
	OMX_S32 count = pstQue->lQueCount;

	if( count == 0 ) {
		ERROR("ShowInputInfo() - info queue is empty");
		count = -1;
	} else {

	*pstInput = pstQue->pstInfoRing[pstQue->lStartPos];

	LOG_IIQUE("[SHOW ][QUECNT: %3ld] [PTS: %8ld] [REGION: %7ld ~ %7ld] [LENGTH: %7ld byte]"
			  , pstQue->lQueCount
			  , (OMX_S32)(pstInput->llTimestamp/1000)
			  , pstInput->pStartPtr - pstQue->pBasePtr
			  , pstInput->pEndPtr - pstQue->pBasePtr
			  , pstInput->lFilledLen
			  );
	}
	return count;
}

static
OMX_S32
ShowLastInputInfo(
	input_queue_t  *pstQue,
	input_info_t   *pstInput
	)
{
	OMX_S32 count = pstQue->lQueCount;
	OMX_S32 end   = pstQue->lEndPos-1;

	if( count == 0 ) {
		ERROR("ShowLastInputInfo() - info queue is empty");
		count = -1;
	} else {

		if( end < 0 ){
			end = pstQue->lQueSize - 1;
		}

		*pstInput = pstQue->pstInfoRing[end];

		LOG_IIQUE("[SHOWL][QUECNT: %3ld] [PTS: %8ld] [REGION: %7ld ~ %7ld] [LENGTH: %7ld byte]"
				  , pstQue->lQueCount
				  , (OMX_S32)(pstInput->llTimestamp/1000)
				  , pstInput->pStartPtr - pstQue->pBasePtr
				  , pstInput->pEndPtr - pstQue->pBasePtr
				  , pstInput->lFilledLen
				  );
	}
	return count;
}

static
OMX_S32
ClearFirstInputInfo(
	input_queue_t  *pstQue
	)
{
	OMX_S32        start  = pstQue->lStartPos;
	OMX_S32        size   = pstQue->lQueSize;
	OMX_S32        count  = pstQue->lQueCount;

	if( count == 0 ) {
		ERROR("ClearFirstInputInfo() - internal error");
		count = -1;
	} else {

		start++;
		if( start >= size ){
			start -= size;
		}

		count--;
		if( count ){
			pstQue->lStartPos = start;
		}
		else {
			pstQue->lStartPos	= 0;
			pstQue->lEndPos		= 0;
		}

		pstQue->lQueCount = count;

		LOG_IIQUE("[CLEAR][QUECNT: %3ld]", pstQue->lQueCount);
	}
	return count;
}
#endif //DECODER_RINGBUFFER_MODE


////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//  Display index / VSync buffer ID queue (static size queue)
//
//
static void ClearDispIdxQueue(dispidx_queue_t *pstQue);
static OMX_S32 PushDispIdx(dispidx_queue_t *pstQue, OMX_U32 ulDispIdx, OMX_U32 ulElDispIdx);
static OMX_S32 GetDispIdx(dispidx_queue_t *pstQue, OMX_U32 *pulDispIdx, OMX_U32 *pulElDispIdx);
static OMX_S32 ShowDispIdx(dispidx_queue_t *pstQue, OMX_U32 *pulDispIdx, OMX_U32 *pulElDispIdx);
static OMX_S32 ClearFirstDispIdx(dispidx_queue_t *pstQue);
static OMX_S32 GetDispIdxCount(dispidx_queue_t *pstQue);

static
void
ClearDispIdxQueue(
	dispidx_queue_t  *pstQue
	)
{
	memset(pstQue, 0, sizeof(dispidx_queue_t));

	LOG_IDXQUE("[CLEAR] [QUECNT: %3d]", pstQue->lQueCount);
}

static
OMX_S32
PushDispIdx(
	dispidx_queue_t  *pstQue,
	OMX_U32	         ulDispIdx,
	OMX_U32          ulElDispIdx
	)
{
	OMX_S32   count	= pstQue->lQueCount;
	OMX_S32   end	= pstQue->lEndPos;

	if( ++count > REFERENCE_BUFFER_MAX )
	{
		ERROR("PushDispIdx() - queue overflow");
		return -1;
	}

	if( count == 1 ) {
		pstQue->aulDispIdxRing[end] = ulDispIdx;
		pstQue->aulDispIdxRing_DV[end] = ulElDispIdx;
		pstQue->lQueCount = 1;
		pstQue->lEndPos++;
	}
	else {
		pstQue->aulDispIdxRing[end] = ulDispIdx;
		pstQue->aulDispIdxRing_DV[end] = ulElDispIdx;

		if( ++end >= REFERENCE_BUFFER_MAX ){
			end = 0;
		}
		pstQue->lEndPos = end;
		pstQue->lQueCount = count;
	}

	LOG_IDXQUE("[PUSH ] [QUECNT: %3d] [DISP_IDX: %3d] [DV_DISP_IDX: %3d]"
			   , pstQue->lQueCount
			   , ulDispIdx
			   , ulElDispIdx
			   );

	return count;
}

static
OMX_S32
GetDispIdx(
	dispidx_queue_t  *pstQue,
	OMX_U32          *pulDispIdx,
	OMX_U32          *pulElDispIdx
	)
{
	OMX_S32   start = pstQue->lStartPos;
	OMX_S32   count = pstQue->lQueCount;

	if( count == 0 ){
		return -1;
	}

	*pulDispIdx = pstQue->aulDispIdxRing[start];
	*pulElDispIdx = pstQue->aulDispIdxRing_DV[start];

	if( ++start >= REFERENCE_BUFFER_MAX ){
		start = 0;
	}

	if( --count ){
		pstQue->lStartPos = start;
	} else {
		pstQue->lStartPos	= 0;
		pstQue->lEndPos		= 0;
	}

	pstQue->lQueCount = count;

	LOG_IDXQUE("[GET  ] [QUECNT: %3ld] [DISP_IDX: %3lu]"
			   , pstQue->lQueCount
			   , *pulDispIdx
			   );

	return count;
}

void IncDispBufCount(dispidx_queue_t  *pstQue,OMX_U32 dispIdx)
{
	if(pstQue)
	{
		pstQue->needClearCount++;
	}
}

void DecDispBufCount(dispidx_queue_t  *pstQue)
{
	if(pstQue)
	{
		if (pstQue->needClearCount > 0) {
			pstQue->needClearCount--;
		}
	}
}

OMX_S32 GetDispBufCount(dispidx_queue_t  *pstQue)
{
	OMX_S32 ret = -1;
	if(pstQue)
	{
		ret = pstQue->needClearCount;
	}
	return ret;
}

static OMX_S32 GetDispIdxByRealIdx(dispidx_queue_t  *pstQue,OMX_S32 realDispIdx,OMX_U32       *pulDispIdx)
{
	OMX_S32   start = pstQue->lStartPos;
	OMX_S32   count = pstQue->lQueCount;

	if(( count > 0 ) && (pstQue->aulDispIdxRing[start] <= realDispIdx))
	{
		*pulDispIdx = pstQue->aulDispIdxRing[start];

		LOG_IDXQUE("[SHOW ] [QUECNT: %3d] [DISP_IDX: %3d] by realDispIdx %3d"
				   , pstQue->lQueCount
				   , *pulDispIdx
				   ,realDispIdx
				   );
	}
	else
	{
		count = -1;
	}

	return count;
}

static
OMX_S32
ShowDispIdx(
	dispidx_queue_t  *pstQue,
	OMX_U32          *pulDispIdx,
	OMX_U32          *pulElDispIdx
	)
{
	OMX_S32   start = pstQue->lStartPos;
	OMX_S32   count = pstQue->lQueCount;

	if( count == 0 ){
		count = -1;
	} else {

		*pulDispIdx = pstQue->aulDispIdxRing[start];
		*pulElDispIdx = pstQue->aulDispIdxRing_DV[start];

		LOG_IDXQUE("[SHOW ] [QUECNT: %3d] [DISP_IDX: %3d] [DV_EL_IDX: %3d]"
			   , pstQue->lQueCount
			   , *pulDispIdx
			   , *pulElDispIdx
			   );
	}
	return count;
}

static
OMX_S32
ClearFirstDispIdx(
	dispidx_queue_t  *pstQue
	)
{
	OMX_S32   start = pstQue->lStartPos;
	OMX_S32   count = pstQue->lQueCount;

	if( count == 0 ){
		count = -1;
	} else {

		if( ++start >= REFERENCE_BUFFER_MAX ){
			start = 0;
		}

		if( --count ){
			pstQue->lStartPos = start;
		} else {
			pstQue->lStartPos	= 0;
			pstQue->lEndPos		= 0;
		}

		pstQue->lQueCount = count;

		LOG_IDXQUE("[CLEAR] [QUECNT: %3d]", pstQue->lQueCount);
	}
	return count;
}

static
OMX_S32
GetDispIdxCount(
	dispidx_queue_t *pstQue
	)
{
	return pstQue->lQueCount;
}

////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//  Display information manager
//
//
static OMX_S32 GetFrameType(OMX_S32 lCodecType, OMX_S32 lPictureType, OMX_S32 lPictureStructure);
static void InitDispInfoManager(dispinfo_manager_t *pstMgr, OMX_S32 lStreamType, OMX_S32 lFrameRate);
static void DeinitDispInfoManager(dispinfo_manager_t *pstMgr);
static void ResetDispInfoManager(dispinfo_manager_t *pstMgr);
static disp_info_t* GetDispInfoSlot(dispinfo_manager_t *pstMgr, OMX_S32 lBuffIdx);
static OMX_BOOL UpdateDispInfo(dispinfo_manager_t *pstMgr, OMX_S32 lBuffIdx, OMX_S32 lFrameRate);
static OMX_BOOL ShowOutputDispInfo(dispinfo_manager_t *pstMgr, OMX_S32 lBuffIdx, disp_info_t **ppstInfo, OMX_TICKS *pllOutPTS);
static OMX_BOOL ShowNonQueuedDispInfo(dispinfo_manager_t *pstMgr, OMX_S32 lBuffIdx, disp_info_t **ppstInfo, OMX_TICKS *pllOutPTS);
static OMX_BOOL ClearDispInfo(dispinfo_manager_t *pstMgr, OMX_S32 lBuffIdx);
static OMX_BOOL ClearNonQueuedDispInfo(dispinfo_manager_t *pstMgr, OMX_S32 lBuffIdx);
static OMX_BOOL SetUserData(disp_info_t *pstInfo, OMX_U8 *pbyUserData, OMX_S32 lDataLength);

static
void
InitDispInfoManager(
	dispinfo_manager_t  *pstMgr,
	OMX_S32              lStreamType,
	OMX_S32              lFrameRate
	)
{
	ResetDispInfoManager(pstMgr);
	pstMgr->lStreamType = lStreamType;
	pstMgr->lFrameRate = lFrameRate;
	if( lFrameRate > 0 ){
		pstMgr->llFrameDuration = 1000000000 / lFrameRate;
	} else {
		pstMgr->llFrameDuration = 0;
	}

	pstMgr->ulFlags = 0;

	LOG_DIMGR("FRAME-RATE INIT - [FRAME-RATE: %ld] [FRAME-DUE: %lld us]"
			  , pstMgr->lFrameRate
			  , pstMgr->llFrameDuration
			  );
}

static
void
DeinitDispInfoManager(
	dispinfo_manager_t  *pstMgr
	)
{
	OMX_S32 i;

	if(IS_NULL_PARAM(pstMgr))
	{
		LOGE("[%s:%d] Input value is NULL!!", __func__, __LINE__);
	} else {
		for(i = 0; i < REFERENCE_BUFFER_MAX; i++) {
			if (pstMgr->astDispInfoList[i].pbyUserDataBuff) {
				TCC_free (pstMgr->astDispInfoList[i].pbyUserDataBuff);
				pstMgr->astDispInfoList[i].pbyUserDataBuff = NULL;
			}
		}
	}
}

static
void
ResetDispInfoManager(
	dispinfo_manager_t  *pstMgr
	)
{
	OMX_S32 i;

	if(IS_NULL_PARAM(pstMgr))
	{
		LOGE("[%s:%d] Input value is NULL!!", __func__, __LINE__);
	} else {
		for(i = 0; i < REFERENCE_BUFFER_MAX; i++) {
			pstMgr->astDispInfoList[i].bIsValid = OMX_FALSE;
			pstMgr->astDispInfoList[i].lUserDataLength = 0;
		}
		memset(pstMgr->allTimestampRing, 0, sizeof(OMX_TICKS) * REFERENCE_BUFFER_MAX);
		pstMgr->lQueCount = 0;
		pstMgr->lStartPos = 0;
		pstMgr->lEndPos = 0;
		pstMgr->lFrameDurationFactor = 0;
		pstMgr->llLastBaseTimestamp = -1;
		pstMgr->llLastOutTimestamp = -1;
		pstMgr->llLastDecTimestamp = -1;
		pstMgr->llShowedTimestamp = 0;
		pstMgr->lExtReferenceIdx = -1;
	}
}

static
disp_info_t*
GetDispInfoSlot(
	dispinfo_manager_t  *pstMgr,
	OMX_S32              lBuffIdx
	)
{
	disp_info_t  *p_info;
	
	if(IS_NULL_PARAM(pstMgr))
	{
		LOGE("[%s:%d] Input value is NULL!!", __func__, __LINE__);
		return NULL;
	}
	
	if(lBuffIdx >= REFERENCE_BUFFER_MAX || lBuffIdx < 0)
	{
		ERROR("GetDispInfoSlot() - invalid buffer index (buff_idx %ld)", lBuffIdx);
		return NULL;
	}
	
	p_info = pstMgr->astDispInfoList + lBuffIdx;
	if( p_info->bIsValid == OMX_TRUE ){
		LOG_STEP("GetDispInfoSlot() - already valid state: (buff_idx %ld)", lBuffIdx);

		//It is abnormal event caused by same decoded buffer index as previous decoding result.
		//But, it is ignored and renew buffer state from current decoding result.
		//return NULL;
	}

	return pstMgr->astDispInfoList + lBuffIdx;
}

static
OMX_BOOL
UpdateDispInfo(
	dispinfo_manager_t  *pstMgr,
	OMX_S32	             lBuffIdx,
	OMX_S32              lFrameRate
	)
{
	disp_info_t  *p_info = pstMgr->astDispInfoList + lBuffIdx;
	OMX_S32       count	 = pstMgr->lQueCount;
	OMX_TICKS     timestamp;
	OMX_S32       i;

	if( lBuffIdx >= REFERENCE_BUFFER_MAX ) {
		ERROR("UpdateDispInfo() - invalid buffer index: (buff_idx %ld)", lBuffIdx);
		return OMX_FALSE;
	}

//	It is abnormal event caused by same decoded buffer index as previous decoding result.
//  But, it is ignored and renew buffer state from current decoding result.
//
//	if( p_info->bIsValid == OMX_TRUE ){
//		WARN("UpdateDispInfo() - already valid state: (buff_idx %ld)", lBuffIdx);
//		return OMX_FALSE;
//	}

	if( p_info->bIsValid == OMX_TRUE )
	{
		OMX_TICKS *p_ring      = pstMgr->allTimestampRing;
		OMX_TICKS  pts_finding = p_info->llPrevTimestamp;
		OMX_S32    current     = pstMgr->lEndPos;
		OMX_S32    lremove      = -1;

		for(i = 0; i < count-1; i++)
		{
			if ( current <= 0 ) {
				current = REFERENCE_BUFFER_MAX-1;
			} else {
				current--;
			}

			if( p_ring[current] == pts_finding ) {
				lremove = current;
				break;
			}
		}

		if( lremove < 0 ) {
			LOG_STEP("UpdateDispInfo() - already valid state: (buff_idx %ld / lremove: non)", lBuffIdx);
		}
		else
		{
			OMX_S32    end    = pstMgr->lEndPos;
			OMX_S32    next;

			do
			{
				next = current + 1;
				if( next >= REFERENCE_BUFFER_MAX ){
					next = 0;
				}

				p_ring[current] = p_ring[next];

				current++;
				if( current >= REFERENCE_BUFFER_MAX ){
					current = 0;
				}

			} while(next != end);

			WARN("UpdateDispInfo() - already valid state: (buff_idx %ld / lremove: %ld)", lBuffIdx, lremove);
		}

		if( pstMgr->lEndPos == 0 ){
			pstMgr->lEndPos = REFERENCE_BUFFER_MAX-1;
		} else {
			pstMgr->lEndPos --;
		}
		
	}
	else
	{
		count ++;
		if( count >= REFERENCE_BUFFER_MAX ) {
			ERROR("UpdateDispInfo() - timestamp queue overflow");
			return -1;
		}
	}

	/* update frame-rate */
	if( lFrameRate && pstMgr->lFrameRate != lFrameRate ) {
		pstMgr->lFrameRate = lFrameRate;
		pstMgr->llFrameDuration = 1000000000 / lFrameRate;

		LOG_DIMGR("FRAME-RATE UPDATED - [FRAME-RATE: %ld] [FRAME-DUE: %lld us]"
				  , pstMgr->lFrameRate
				  , pstMgr->llFrameDuration
				  );
	}

#ifdef TCC_EXT_INCLUDED
	/* RealVideo timestamp calculation */
	if( pstMgr->lStreamType == STD_EXT )
	{
		OMX_S32    rv_timestamp   = p_info->lExtTimestamp;
		OMX_TICKS  curr_timestamp = p_info->llTimestamp;
		OMX_S32    ref_idx        = pstMgr->lExtReferenceIdx;

		if( ref_idx < 0 )
		{
			ref_idx = 0;
			pstMgr->lExtTimestamp[0] = curr_timestamp;
			pstMgr->lExtTimestamp[1] = curr_timestamp;
			pstMgr->lExtTimeReference[0] = rv_timestamp;
			pstMgr->lExtTimeReference[1] = rv_timestamp;
		}
		else
		{
			OMX_S32 tr_delta = rv_timestamp - pstMgr->lExtTimeReference[ref_idx];

			if( tr_delta < 0 ){
				tr_delta += 8192;
			}

			if( p_info->lFrameType == 2 ) {//B-frame
				curr_timestamp = pstMgr->lExtTimestamp[ref_idx] + ((OMX_TICKS)tr_delta * 1000);
			} else {
				ref_idx++;
			}

			if( ref_idx == 1 ) {
				pstMgr->lExtTimestamp[0] = curr_timestamp;
				pstMgr->lExtTimeReference[0] = rv_timestamp;

			}
			else if( ref_idx == 2 ) {
				ref_idx = 0;
				pstMgr->lExtTimestamp[1] = curr_timestamp;
				pstMgr->lExtTimeReference[1] = rv_timestamp;
			}
		}

		pstMgr->lExtReferenceIdx = ref_idx;
		p_info->llTimestamp = curr_timestamp;
	}
#endif //TCC_EXT_INCLUDED

	p_info->bIsValid = OMX_TRUE;

	/* update timestamp queue */
	timestamp = p_info->llTimestamp;
	p_info->llPrevTimestamp = timestamp;

	if ((pstMgr->ulFlags & DISPMGR_PTS_MODE_DETERMINED) == 0) {
		if (pstMgr->ulFlags & DISPMGR_NON_B_FRAME_UPDATED) {
			if (GetFrameType(pstMgr->lStreamType, p_info->lFrameType, p_info->lPicStructure) == PIC_TYPE_B) {
				pstMgr->ulFlags |= DISPMGR_PTS_MODE_DETERMINED;
				if (pstMgr->llLastDecTimestamp < timestamp) {
					pstMgr->ulFlags |= DISPMGR_TIMESTAMP_SORTING;
					INFO("Timestamp sorting enabled.")
				}
			}
		}
		else {
			if (GetFrameType(pstMgr->lStreamType, p_info->lFrameType, p_info->lPicStructure) != PIC_TYPE_B) {
				pstMgr->ulFlags |= DISPMGR_NON_B_FRAME_UPDATED;
			}
		}
	}

	if (pstMgr->ulFlags & DISPMGR_TIMESTAMP_GENERATING) {
		if (pstMgr->llLastDecTimestamp < timestamp) {
			p_info->bPtsNotExist = OMX_FALSE;
			pstMgr->llLastDecTimestamp = timestamp;
		} else {
			p_info->bPtsNotExist = OMX_TRUE;
		}
	}
	else {
		if (pstMgr->llLastDecTimestamp != timestamp) {
			p_info->bPtsNotExist = OMX_FALSE;
			pstMgr->llLastDecTimestamp = timestamp;
		} else {
			p_info->bPtsNotExist = OMX_TRUE;
			pstMgr->ulFlags |= DISPMGR_TIMESTAMP_GENERATING;
		}
	}

	if (pstMgr->ulFlags & DISPMGR_TIMESTAMP_SORTING && pstMgr->ulFlags & DISPMGR_TIMESTAMP_GENERATING) {
		pstMgr->ulFlags &= ~DISPMGR_TIMESTAMP_SORTING;
	}

	if (count == 1) {
		pstMgr->allTimestampRing[pstMgr->lEndPos] = timestamp;
		pstMgr->lQueCount = 1;
	}
	else {
		if (pstMgr->ulFlags & DISPMGR_BACKWARD_PLAYBACK) {
			pstMgr->allTimestampRing[pstMgr->lEndPos] = timestamp;
		}
		else {
			OMX_TICKS *p_ring = pstMgr->allTimestampRing;
			OMX_S32    end    = pstMgr->lEndPos;
			OMX_S32    comp   = end-1;

			for(i = 0; i < count-1; i++)
			{
				if(comp < 0){
					comp = REFERENCE_BUFFER_MAX-1;
				}

				if( p_ring[comp] <= timestamp ) {
					p_ring[end] = timestamp;
					break;
				}
				else {
					p_ring[end] = p_ring[comp];
					end = comp--;
				}
			}

			if( i == count-1 ){
				p_ring[end] = timestamp;
			}
		}

		pstMgr->lQueCount = count;
	}

	if( ++pstMgr->lEndPos >= REFERENCE_BUFFER_MAX ){
		pstMgr->lEndPos = 0;
	}

	LOG_DIMGR("[UPDATE] [BUFF_IDX: %2ld] {PTS} [INPUT   : %8ld]"
			  , lBuffIdx
			  , (OMX_S32)(timestamp/1000)
			  );

	return OMX_TRUE;
}

static
OMX_BOOL
ShowOutputDispInfo(
	dispinfo_manager_t  *pstMgr,
	OMX_S32	             lBuffIdx,
	disp_info_t        **ppstInfo,
	OMX_TICKS           *pllOutPTS
	)
{
	disp_info_t  *p_info = pstMgr->astDispInfoList + lBuffIdx;
	OMX_S32       start  = pstMgr->lStartPos;
	OMX_S32       count  = pstMgr->lQueCount;
	OMX_TICKS     out_timestamp;

	if (count == 0) {
		WARN("ShowOutputDispInfo() - decoded buffer is not registered: (buff_idx %ld)", lBuffIdx);
		return OMX_FALSE;
	}

	if (lBuffIdx >= REFERENCE_BUFFER_MAX) {
		WARN("ShowOutputDispInfo() - invalid buffer index: (buff_idx %ld)", lBuffIdx);
		return OMX_FALSE;
	}

	if (p_info->bIsValid == OMX_FALSE){
		WARN("ShowOutputDispInfo() - invalid buffer state: (buff_idx %ld)", lBuffIdx);
		return OMX_FALSE;
	}

	if (pstMgr->ulFlags & DISPMGR_TIMESTAMP_SORTING){
		out_timestamp = pstMgr->allTimestampRing[start];
	} else {
		out_timestamp = p_info->llTimestamp;
	}

	if (p_info->ulFlags & DISPINFO_FLAG_DEPENDENT_VIEW) {
		out_timestamp = pstMgr->llLastOutTimestamp;
	} else {
		if ((pstMgr->ulFlags & DISPMGR_BACKWARD_PLAYBACK) == 0 && p_info->bPtsNotExist == OMX_TRUE)
		{
			if (pstMgr->lFrameDurationFactor){
				out_timestamp = pstMgr->llLastOutTimestamp + ((pstMgr->llFrameDuration * pstMgr->lFrameDurationFactor) >> 1);
			} else {
				out_timestamp = pstMgr->llLastOutTimestamp + pstMgr->llFrameDuration;
			}

			LOG_DIMGR("[SHOW  ] [BUFF_IDX: %2ld] {PTS} [LAST_OUT: %8ld] [ORG: %8ld / %8ld (%ld)][MOD: %8ld] [FACTOR: %ld]"
					  , lBuffIdx
					  , (OMX_S32)(pstMgr->llLastOutTimestamp/1000)
					  , (OMX_S32)(p_info->llTimestamp/1000)
					  , (OMX_S32)(pstMgr->allTimestampRing[start]/1000)
					  , (OMX_S32)p_info->bPtsNotExist
					  , (OMX_S32)(out_timestamp/1000)
					  , pstMgr->lFrameDurationFactor
					  );
		}
		else {
			LOG_DIMGR("[SHOW  ] [BUFF_IDX: %2ld] {PTS} [LAST_OUT: %8ld] [ORG: %8ld / %8ld (%ld)]"
					  , lBuffIdx
					  , (OMX_S32)(pstMgr->llLastOutTimestamp/1000)
					  , (OMX_S32)(p_info->llTimestamp/1000)
					  , (OMX_S32)(pstMgr->allTimestampRing[start]/1000)
					  , (OMX_S32)p_info->bPtsNotExist
					  );
		}
	}

	pstMgr->llShowedTimestamp = out_timestamp;

	*ppstInfo = p_info;
	*pllOutPTS = out_timestamp;

	return OMX_TRUE;
}

static
OMX_BOOL
ShowNonQueuedDispInfo(
	dispinfo_manager_t  *pstMgr,
	OMX_S32	             lBuffIdx,
	disp_info_t        **ppstInfo,
	OMX_TICKS           *pllOutPTS
	)
{
	disp_info_t  *p_info = pstMgr->astDispInfoList + lBuffIdx;
	OMX_TICKS     out_timestamp;

	if (lBuffIdx >= REFERENCE_BUFFER_MAX) {
		WARN("ShowNonQueuedDispInfo() - invalid buffer index: (buff_idx %ld)", lBuffIdx);
		return OMX_FALSE;
	}

	if (p_info->ulFlags & DISPINFO_FLAG_DEPENDENT_VIEW) {
		out_timestamp = pstMgr->llLastOutTimestamp;
	}
	else
	{
		if ((pstMgr->ulFlags & DISPMGR_BACKWARD_PLAYBACK) == 0)
		{
			if (pstMgr->lFrameDurationFactor){
				out_timestamp = pstMgr->llLastOutTimestamp + ((pstMgr->llFrameDuration * pstMgr->lFrameDurationFactor) >> 1);
			} else {
				out_timestamp = pstMgr->llLastOutTimestamp + pstMgr->llFrameDuration;
			}

			LOG_DIMGR("[SHOW  ] [BUFF_IDX: %2ld] {PTS} [LAST_OUT: %8ld] [ORG: %8ld / -------- (%ld)][MOD: %8ld] [FACTOR: %ld]"
					  , lBuffIdx
					  , (OMX_S32)(pstMgr->llLastOutTimestamp/1000)
					  , (OMX_S32)(p_info->llTimestamp/1000)
					  , (OMX_S32)p_info->bPtsNotExist
					  , (OMX_S32)(out_timestamp/1000)
					  , pstMgr->lFrameDurationFactor
					  );
		}
		else
		{
			out_timestamp = pstMgr->llLastBaseTimestamp;

			LOG_DIMGR("[SHOW  ] [BUFF_IDX: %2ld] {PTS} [LAST_OUT: %8ld] [ORG: %8ld / -------- (%ld)][MOD: %8ld]"
					  , lBuffIdx
					  , (OMX_S32)(pstMgr->llLastOutTimestamp/1000)
					  , (OMX_S32)(p_info->llTimestamp/1000)
					  , (OMX_S32)p_info->bPtsNotExist
					  , (OMX_S32)(out_timestamp/1000)
					  );
		}
	}

	pstMgr->llShowedTimestamp = out_timestamp;

	*ppstInfo = p_info;
	*pllOutPTS = out_timestamp;

	return OMX_TRUE;
}

static
OMX_BOOL
ClearDispInfo(
	dispinfo_manager_t  *pstMgr,
	OMX_S32	             lBuffIdx
	)
{
	disp_info_t  *p_info = pstMgr->astDispInfoList + lBuffIdx;
	OMX_S32       start  = pstMgr->lStartPos;
	OMX_S32       count  = pstMgr->lQueCount;

	if (count == 0){
		return -1;
	}

	if (lBuffIdx >= REFERENCE_BUFFER_MAX) {
		ERROR("ClearDispInfo() - invalid buffer index: (buff_idx %ld)", lBuffIdx);
		return OMX_FALSE;
	}

	if (p_info->bIsValid == OMX_FALSE){
		ERROR("ClearDispInfo() - already invalid state: (buff_idx %ld)", lBuffIdx);
		return OMX_FALSE;
	}

	p_info->bIsValid = OMX_FALSE;
	p_info->lUserDataLength = 0;

	/* last timestamp */
	if (pstMgr->ulFlags & DISPMGR_TIMESTAMP_SORTING) {
		if (pstMgr->llShowedTimestamp >= 0){
			pstMgr->llLastOutTimestamp = pstMgr->llShowedTimestamp;
		} else {
			pstMgr->llLastOutTimestamp = pstMgr->allTimestampRing[start];
		}
		pstMgr->llLastBaseTimestamp = pstMgr->allTimestampRing[start];
		pstMgr->llShowedTimestamp = -1;
	}
	else {
		if (pstMgr->llShowedTimestamp >= 0){
			pstMgr->llLastOutTimestamp = pstMgr->llShowedTimestamp;
		} else {
			pstMgr->llLastOutTimestamp = p_info->llTimestamp;
		}
		pstMgr->llLastBaseTimestamp = p_info->llTimestamp;
		pstMgr->llShowedTimestamp = -1;
	}

	/* duration factor */
	pstMgr->lFrameDurationFactor = p_info->lFrameDurationFactor;

	if (++start >= REFERENCE_BUFFER_MAX){
		start = 0;
	}

	if (--count){
		pstMgr->lStartPos = start;
	} else {
		pstMgr->lStartPos = 0;
		pstMgr->lEndPos   = 0;
	}

	pstMgr->lQueCount = count;

	LOG_DIMGR("[CLEAR ] [BUFF_IDX: %2ld] {PTS} [LAST_OUT: %8ld] [ORG: %8ld / %8ld (%ld)]"
			  , lBuffIdx
			  , (OMX_S32)(pstMgr->llLastOutTimestamp/1000)
			  , (OMX_S32)(p_info->llTimestamp/1000)
			  , (OMX_S32)(pstMgr->llLastBaseTimestamp/1000)
			  , (OMX_S32)p_info->bPtsNotExist
			  );

	return OMX_TRUE;
}

static
OMX_BOOL
ClearNonQueuedDispInfo(
	dispinfo_manager_t  *pstMgr,
	OMX_S32	             lBuffIdx
	)
{
	disp_info_t  *p_info = pstMgr->astDispInfoList + lBuffIdx;
	OMX_S32       start  = pstMgr->lStartPos;

	if (lBuffIdx >= REFERENCE_BUFFER_MAX) {
		ERROR("ClearDispInfo() - invalid buffer index: (buff_idx %ld)", lBuffIdx);
		return OMX_FALSE;
	}

	p_info->bIsValid = OMX_FALSE;
	p_info->lUserDataLength = 0;

	/* last timestamp */
	if (pstMgr->ulFlags & DISPMGR_TIMESTAMP_SORTING) {
		if (pstMgr->llShowedTimestamp >= 0){
			pstMgr->llLastOutTimestamp = pstMgr->llShowedTimestamp;
		} else {
			pstMgr->llLastOutTimestamp = pstMgr->allTimestampRing[start];
		}
		pstMgr->llLastBaseTimestamp = pstMgr->allTimestampRing[start];
		pstMgr->llShowedTimestamp = -1;
	}
	else {
		if (pstMgr->llShowedTimestamp >= 0){
			pstMgr->llLastOutTimestamp = pstMgr->llShowedTimestamp;
		} else {
			pstMgr->llLastOutTimestamp = p_info->llTimestamp;
		}
		pstMgr->llLastBaseTimestamp = p_info->llTimestamp;
		pstMgr->llShowedTimestamp = -1;
	}

	/* duration factor */
	pstMgr->lFrameDurationFactor = p_info->lFrameDurationFactor;

	LOG_DIMGR("[CLEAR ] [BUFF_IDX: %2ld] {PTS} [LAST_OUT: %8ld] [ORG: %8ld / %8ld (%ld)] -- non-ququed"
			  , lBuffIdx
			  , (OMX_S32)(pstMgr->llLastOutTimestamp/1000)
			  , (OMX_S32)(p_info->llTimestamp/1000)
			  , (OMX_S32)(pstMgr->llLastBaseTimestamp/1000)
			  , (OMX_S32)p_info->bPtsNotExist
			  );

	return OMX_TRUE;
}

static
OMX_BOOL
SetUserData(
	disp_info_t *pstInfo,
	OMX_U8 *pbyUserData,
	OMX_S32 lDataLength
	)
{
	if( lDataLength <= 0 ){
		return OMX_FALSE;
	}

	if (pstInfo->lUserDataBuffSize < lDataLength)
	{
		OMX_S32 buffsize = pstInfo->lUserDataBuffSize == 0 ? 1024 : pstInfo->lUserDataBuffSize;
		while (buffsize < lDataLength){
			buffsize <<= 1;
		}

		if (pstInfo->pbyUserDataBuff){
			TCC_free(pstInfo->pbyUserDataBuff);
		}

		pstInfo->pbyUserDataBuff = TCC_malloc(buffsize);
		if (pstInfo->pbyUserDataBuff == NULL ) {
			ERROR("SetUserData() - Out of memory");
			return FALSE;
		}

		pstInfo->lUserDataBuffSize = buffsize;
	}

	(void)memcpy((void*)pstInfo->pbyUserDataBuff, (void*)pbyUserData, lDataLength);
	pstInfo->lUserDataLength = lDataLength;

	return TRUE;
}

////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//	Configuration Helper
//
//
static void DecideLimitTimeDiffForFeeding(vdec_private_t *pstVDecPrivate, OMX_U32 ulFrameRate);
static OMX_S32 GetAvccToAnnexbSpsPpsLength(const OMX_U8 *pbyAvcc, OMX_S32 lAvccLength, OMX_S32 *plNalLengthSize);
static OMX_BOOL ConvertAvccToAnnexbSpaPps(const OMX_U8 *pbyAvcc, OMX_S32 lAvccLength, OMX_U8 *pbyAnnexb, OMX_S32 *plAnnexbLength);
static OMX_BOOL ConvertAvccToAnnexbStream(vdec_private_t *pstVDecPrivate, const OMX_U8 *pbyAvcc, const OMX_S32 lAvccLength, const OMX_BOOL bAttachSeqHeader);
static OMX_S32 GetHvccToAnnexbSpsPpsLength(const OMX_U8 *pbyHvcc, OMX_S32 lHvccLength, OMX_S32 *plNalLengthSize);
static OMX_BOOL ConvertHvccToAnnexbSpaPps(const OMX_U8 *pbyHvcc, OMX_S32 lHvccLength, OMX_U8 *pbyAnnexb, OMX_S32 *plAnnexbLength);
static OMX_BOOL ConvertHvccToAnnexbStream(vdec_private_t *pstVDecPrivate, const OMX_U8 *pbyHvcc, const OMX_S32 lHvccLength, const OMX_BOOL bAttachSeqHeader);
static OMX_BOOL AttachVpuSpecificHeader(vdec_private_t *pstVDecPrivate, const OMX_U8 *pbyInputStream, const OMX_S32 lInputLength, const OMX_BOOL bAttachSeqHeader);
static OMX_BOOL AttachWmvVpuSpecificHeader(vdec_private_t *pstVDecPrivate, const OMX_U8 *pbyInputStream, const OMX_S32 lInputLength, const OMX_BOOL bAttachSeqHeader);
static OMX_BOOL AttachVc1Syncword(vdec_private_t *pstVDecPrivate, const OMX_U8 *pbyInputStream, const OMX_S32 lInputLength, const OMX_BOOL bAttachSeqHeader);
static OMX_BOOL ConvertBitstream(vdec_private_t *pstVDecPrivate, const OMX_U8 *pbyInputStream, const OMX_S32 lInputLength, const OMX_BOOL bAttachSeqHeader);
static OMX_S32 ScanSequenceHeader(vdec_private_t *pstVDecPrivate, OMX_BYTE *pbyStreamBuffer, OMX_S32 lStreamLength, OMX_S32 lStreamType);
static OMX_BOOL StoreSequenceHeader(vdec_private_t *pstVDecPrivate, OMX_U8 *pbySequenceHeader, OMX_S32 nHeaderLength);
static OMX_BOOL ParseConfigData(vdec_private_t *pstVDecPrivate, OMX_BUFFERHEADERTYPE *pInputBuffer);
static OMX_S32 GetDecodedFrameType(vdec_private_t *pstVDecPrivate);
static void IFrameSearchEnable(vdec_private_t *pstVDecPrivate);
static void SetFrameSkipMode(vdec_private_t *pstVDecPrivate, OMX_S32 lSkipMode);
static OMX_S32 ScanAvcIdrSlice(OMX_BUFFERHEADERTYPE	*pInputBuffer);
static OMX_BOOL IsAvcFrameStart(OMX_U8 *pbyAnnexb, OMX_S32 *plAnnexbLength);

static
void
DecideLimitTimeDiffForFeeding(
	vdec_private_t        *pstVDecPrivate,
	OMX_U32                ulFrameRate
	)
{
	OMX_S64 numerator = VPU_FEED_LIMIT_FACTOR * (OMX_TICKS)1000000000;	//frame-duration * VPU_FEED_LIMIT_FACTOR

	// frame-duration(us) * factor
	if( ulFrameRate < FRAME_RATE_FOR_FEED_LIMIT_MIN	){
		pstVDecPrivate->llFeedMinTimeDiff = numerator / FRAME_RATE_FOR_FEED_LIMIT_MIN;
	} else {
		pstVDecPrivate->llFeedMinTimeDiff = numerator / ulFrameRate;
	}

	pstVDecPrivate->llFeedMaxTimeDiff = pstVDecPrivate->llFeedMinTimeDiff * 2;

	INFO("Feed limit time difference: %lld us (%.3f fps)",
			pstVDecPrivate->llFeedMinTimeDiff,
			(double)ulFrameRate/1000);
}


static
OMX_S32
GetAvccToAnnexbSpsPpsLength(
	const OMX_U8  *pbyAvcc,
	OMX_S32        lAvccLength,
	OMX_S32       *plNalLengthSize
	)
{
	OMX_S32	nal_len;
	OMX_S32 spspps_len = 0;
	OMX_S32	count;
	OMX_S32	i;

	const OMX_U8  *p_avcc = pbyAvcc+5;

	if (( lAvccLength < 7 ) || ( *pbyAvcc != 1 )) {
		if( lAvccLength < 7 ) {
			ERROR("GetAvccToAnnexbSpsPpsLength() - avcC is too short (len: %ld)", lAvccLength);
		}
		// configurationVersion
		if( *pbyAvcc != 1 ) {
			ERROR("GetAvccToAnnexbSpsPpsLength() - Unknown avcC version %d", (int)(*p_avcc));
		}
	} else {
		// lengthSizeMinusOne
		*plNalLengthSize = (pbyAvcc[4] & 0x03)+1;

		// numOfSequenceParameterSets
		count = (*p_avcc++) & 0x1F;

		for(i = 0; i < count; i++)
		{
			// sequenceParameterSetLength
			nal_len = (OMX_S32)(((OMX_U32)p_avcc[0] << 8) | p_avcc[1]);
			spspps_len += 4+nal_len;
			p_avcc += nal_len+2;
		}

		// numOfPictureParameterSets
		count = *p_avcc++;

		for(i = 0; i < count; i++)
		{
			// pictureParameterSetLength
			nal_len = (OMX_S32)(((OMX_U32)p_avcc[0] << 8) | p_avcc[1]);
			spspps_len += 4+nal_len;
			p_avcc += nal_len+2;
		}
	}

	return spspps_len;
}

static
OMX_BOOL
ConvertAvccToAnnexbSpaPps(
	const OMX_U8  *pbyAvcc,
	OMX_S32        lAvccLength,
	OMX_U8        *pbyAnnexb,
	OMX_S32       *plAnnexbLength
	)
{
	OMX_S32	nal_len;
	OMX_S32	count;
	OMX_S32	i;
	OMX_BOOL ret = OMX_TRUE;
	const OMX_U8  *p_avcc = pbyAvcc+5;
	OMX_U8        *p_annexb = pbyAnnexb;

	if( lAvccLength < 7 ) {
		ERROR("ConvertAvccToAnnexbSpaPps() - avcC is too short (len: %ld)", lAvccLength);
		ret = OMX_FALSE;
	}

	// configurationVersion
	if( *pbyAvcc != 1 ) {
		ERROR("ConvertAvccToAnnexbSpaPps() - Unknown avcC version %d", (int)(*p_avcc));
		ret = OMX_FALSE;
	}

	if (ret == OMX_TRUE) {
		// lengthSizeMinusOne
		//*plNalLenSize = (pbyAvcc[4] & 0x03)+1;

		// numOfSequenceParameterSets
		count = (*p_avcc++) & 0x1F;

		for(i = 0; i < count; i++)
		{
			// sequenceParameterSetLength
			nal_len = (OMX_S32)(((OMX_U32)p_avcc[0] << 8) | p_avcc[1]);

			// insert start_code
			*p_annexb++ = 0;
			*p_annexb++ = 0;
			*p_annexb++ = 0;
			*p_annexb++ = 1;

			//sequenceParameterSetNALUnit
			(void)memcpy(p_annexb, p_avcc+2, nal_len);

			p_annexb += nal_len;
			p_avcc += nal_len+2;
		}

		// numOfPictureParameterSets
		count = *p_avcc++;

		for(i = 0; i < count; i++)
		{
			// pictureParameterSetLength
			nal_len = (OMX_S32)(((OMX_U32)p_avcc[0] << 8) | p_avcc[1]);

			// insert start_code
			*p_annexb++ = 0;
			*p_annexb++ = 0;
			*p_annexb++ = 0;
			*p_annexb++ = 1;

			// pictureParameterSetNALUnit
			(void)memcpy(p_annexb, p_avcc+2, nal_len);

			p_annexb += nal_len;
			p_avcc += nal_len+2;
		}

		*plAnnexbLength = (OMX_S32)(p_annexb-pbyAnnexb);
	}
	return ret;
}

static
OMX_BOOL
ConvertAvccToAnnexbStream(
	vdec_private_t  *pstVDecPrivate,
	const OMX_U8    *pbyAvcc,
	const OMX_S32    lAvccLength,
	const OMX_BOOL   bAttachSeqHeader
	)
{
	OMX_S32 in_len = lAvccLength;
	OMX_U8 *p_inptr = pbyAvcc;
	OMX_U8 *p_outptr = pstVDecPrivate->pbyTempInputBuff + pstVDecPrivate->lTempInputOffset;
	OMX_S32 len_size = pstVDecPrivate->lNalLengthSize;
	OMX_S32 nal_len = 0;
	OMX_S32 i;
	OMX_BOOL ret = OMX_TRUE;

	if (bAttachSeqHeader == OMX_TRUE) {
		if (pstVDecPrivate->pbySequenceHeader && pstVDecPrivate->lSeqHeaderLength ) {
			(void)memcpy(p_outptr, pstVDecPrivate->pbySequenceHeader, pstVDecPrivate->lSeqHeaderLength);
			p_outptr += pstVDecPrivate->lSeqHeaderLength;
		}
	}

	while (1)
	{
		if (in_len <= len_size){
			break;
		}

		// get a length of nal
		nal_len = 0;
		for (i = 0; i < len_size; i++){
			nal_len = (nal_len<<8) | *p_inptr++;
		}

		// invalid length
		if (in_len < nal_len) {
			ERROR("ConvertAvcc/HvccToAnnexbStream() - invalid nal length: %ld", nal_len);
			ret = OMX_FALSE;
			break;
		}

		// insert startcode
		*p_outptr++ = 0;
		*p_outptr++ = 0;
		*p_outptr++ = 0;
		*p_outptr++ = 1;

		(void)memcpy(p_outptr, p_inptr, nal_len);

		in_len -= (len_size+nal_len);
		p_inptr += nal_len;
		p_outptr += nal_len;
	}

	// set annexb length
	pstVDecPrivate->lTempInputLength = (OMX_S32)(p_outptr - pstVDecPrivate->pbyTempInputBuff);
	return ret;
}


static
OMX_S32
GetHvccToAnnexbSpsPpsLength(
	const OMX_U8  *pbyHvcc,
	OMX_S32        lHvccLength,
	OMX_S32       *plNalLengthSize
	)
{
	OMX_S32	nal_len;
	OMX_S32 spspps_len = 0;
	OMX_S32	ary_count;
	OMX_S32	nal_count;
	OMX_S32	i, j;

	const OMX_U8  *p_hvcc = pbyHvcc;

	if( lHvccLength < 7 ) {
		ERROR("GetHvccToAnnexbSpsPpsLength() - hvcC is too short (len: %ld)", lHvccLength);
		spspps_len = 0;//return OMX_FALSE;
	} else {

		// lengthSizeMinusOne
		*plNalLengthSize = (pbyHvcc[21] & 0x03)+1;

		// numOfArrays
		ary_count = p_hvcc[22];

		p_hvcc += 23;

		for(i = 0; i < ary_count; i++)
		{
			// array_completeness
			// NAL_unit_type
			// numNalus
			nal_count = p_hvcc[1] << 8 | p_hvcc[2];
			p_hvcc += 3;

			for(j = 0; j < nal_count; j++) {
				nal_len = (OMX_S32)SHIFT_AND_MERGE(p_hvcc[0],p_hvcc[1]); //(((OMX_U32)p_hvcc[0] << 8) | p_hvcc[1]);
				spspps_len += 4+nal_len;
				p_hvcc += nal_len+2;
			}
		}

	}

	return spspps_len;
}


static
OMX_BOOL
ConvertHvccToAnnexbSpaPps(
	const OMX_U8  *pbyHvcc,
	OMX_S32        lHvccLength,
	OMX_U8        *pbyAnnexb,
	OMX_S32       *plAnnexbLength
	)
{
	OMX_S32	nal_len;
	OMX_S32	ary_count;
	OMX_S32	nal_count;
	OMX_S32	i, j;
	OMX_BOOL ret;
	const OMX_U8  *p_hvcc = pbyHvcc;
	OMX_U8        *p_annexb = pbyAnnexb;

	if( lHvccLength < 7 ) {
		ERROR("ConvertHvccToAnnexbSpaPps() - hvcC is too short (len: %ld)", lHvccLength);
		ret = OMX_FALSE;
	} 
	else {

		// lengthSizeMinusOne
		//*plNalLengthSize = (pbyHvcc[21] & 0x03)+1;

		// numOfArrays
		ary_count = p_hvcc[22];

		p_hvcc += 23;

		for(i = 0; i < ary_count; i++)
		{
			// array_completeness
			// NAL_unit_type
			// numNalus
			nal_count = SHIFT_AND_MERGE(p_hvcc[1],p_hvcc[2]); //p_hvcc[1] << 8 | p_hvcc[2];
			p_hvcc += 3;

			for(j = 0; j < nal_count; j++) {
				nal_len = (OMX_S32)SHIFT_AND_MERGE(p_hvcc[0],p_hvcc[1]); //(((OMX_U32)p_hvcc[0] << 8) | p_hvcc[1]);

				// insert start_code
				*p_annexb++ = 0;
				*p_annexb++ = 0;
				*p_annexb++ = 0;
				*p_annexb++ = 1;

				//sequenceParameterSetNALUnit
				(void)memcpy(p_annexb, p_hvcc+2, nal_len);

				p_annexb += nal_len;
				p_hvcc += nal_len+2;
			}
		}

		*plAnnexbLength = (OMX_S32)(p_annexb-pbyAnnexb);
		ret = OMX_TRUE;
	}

	return ret;
}

#if 0
static
OMX_BOOL
ConvertHvccToAnnexbStream(
	vdec_private_t  *pstVDecPrivate,
	const OMX_U8    *pbyHvcc,
	const OMX_S32    lHvccLength,
	const OMX_BOOL   bAttachSeqHeader
	)
{
	OMX_S32 in_len = lHvccLength;
	OMX_U8 *p_inptr = pbyHvcc;
	OMX_U8 *p_outptr = pstVDecPrivate->pbyTempInputBuff + pstVDecPrivate->lTempInputOffset;
	OMX_S32 len_size = pstVDecPrivate->lNalLengthSize;
	OMX_S32 nal_len = 0;
	OMX_S32 i;

#if defined(TC_SECURE_MEMORY_COPY)
	if(CHECK_STATE(pstVDecPrivate, STATE_VDEC_BUFFER_PROTECTION))
	{
		return OMX_TRUE;
	}
#endif

	if (bAttachSeqHeader == OMX_TRUE) {
		if (pstVDecPrivate->pbySequenceHeader && pstVDecPrivate->lSeqHeaderLength ) {
			(void)memcpy(p_outptr, pstVDecPrivate->pbySequenceHeader, pstVDecPrivate->lSeqHeaderLength);
			p_outptr += pstVDecPrivate->lSeqHeaderLength;
		}
	}

	while (1)
	{
		if (in_len <= len_size)
			break;

		// get a length of nal
		nal_len = 0;
		for (i = 0; i < len_size; i++)
			nal_len = (nal_len<<8) | *p_inptr++;

		// invalid length
		if (in_len < nal_len) {
			ERROR("ConvertHvccToAnnexbStream() - invalid nal length: %ld", nal_len);
			return OMX_FALSE;
		}

		// insert startcode
		*p_outptr++ = 0;
		*p_outptr++ = 0;
		*p_outptr++ = 0;
		*p_outptr++ = 1;

		(void)memcpy(p_outptr, p_inptr, nal_len);

		in_len -= len_size+nal_len;
		p_inptr += nal_len;
		p_outptr += nal_len;
	}

	// set annexb length
	pstVDecPrivate->lTempInputLength = (OMX_S32)(p_outptr - pstVDecPrivate->pbyTempInputBuff);

	return OMX_TRUE;
}
#endif

static
OMX_BOOL
AttachVpuSpecificHeader(
	vdec_private_t  *pstVDecPrivate,
	const OMX_U8    *pbyInputStream,
	const OMX_S32    lInputLength,
	const OMX_BOOL   bAttachSeqHeader
	)
{
	OMX_U32 unused = 0;
	OMX_U8 *p_buffer = pstVDecPrivate->pbyTempInputBuff + pstVDecPrivate->lTempInputOffset;
	OMX_BOOL ret;

	if(p_buffer == NULL) //  dambee7 modify
	{
		ERROR("%s %d p_buffer is NULL!!!!!!!!!!!!!!!\n",__func__,__LINE__);
		ret = OMX_FALSE;
	}
	else {
		if (bAttachSeqHeader)
		{
			omx_base_video_PortType *p_inport = (omx_base_video_PortType *)pstVDecPrivate->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];
			OMX_S32 width = pstVDecPrivate->stCropRect.nWidth;
			OMX_S32 height = pstVDecPrivate->stCropRect.nHeight;
			OMX_U16 headerlength = 32;
			OMX_U32 signature = 0;
			OMX_U32 codecfourcc = 0;

			switch (pstVDecPrivate->stVDecInit.m_iBitstreamFormat) {
			case STD_MPEG4:
			case STD_DIV3:
				signature   = 'VMNC';
				codecfourcc = '3VID';
				break;
			case STD_VP8:
				signature   = 'FIKD';
				codecfourcc = '08PV';
				break;
//FIXME - can't access below code (2022-01-10 Helena)
#ifdef TCC_VP9_INCLUDE
			case STD_VP9:
				signature   = 'FIKD';
				codecfourcc = '09PV';
				break;
#endif
			}

			(void)memcpy(p_buffer,     &signature,     sizeof(OMX_U32) );
			(void)memcpy(p_buffer+4,   &unused,        sizeof(OMX_U16) ); //version
			(void)memcpy(p_buffer+6,   &headerlength,  sizeof(OMX_U16) );
			(void)memcpy(p_buffer+8,   &codecfourcc,   sizeof(OMX_U32) );
			(void)memcpy(p_buffer+12,  &width,         sizeof(OMX_U16) );
			(void)memcpy(p_buffer+14,  &height,        sizeof(OMX_U16) );
			(void)memcpy(p_buffer+16,  &unused,        sizeof(OMX_U32) ); //framerate
			(void)memcpy(p_buffer+20,  &unused,        sizeof(OMX_U32) ); //timescale
			(void)memcpy(p_buffer+24,  &unused,        sizeof(OMX_U32) ); //indexentry
			(void)memcpy(p_buffer+28,  &unused,        sizeof(OMX_U32) ); //unused

			p_buffer = pstVDecPrivate->pbyTempInputBuff + pstVDecPrivate->lTempInputOffset + 32;
		}

		(void)memcpy(p_buffer,     &lInputLength,   sizeof(OMX_U32) );
		(void)memcpy(p_buffer+4,   &unused,         sizeof(OMX_U32) );
		(void)memcpy(p_buffer+8,   &unused,         sizeof(OMX_U32) );
		(void)memcpy(p_buffer+12,  pbyInputStream,  lInputLength    );
		p_buffer += 12+lInputLength;

		// vpu specific stream
		pstVDecPrivate->lTempInputLength = (OMX_S32)(p_buffer - pstVDecPrivate->pbyTempInputBuff);
		ret = OMX_TRUE;
	}
	return ret;
}

static
OMX_BOOL
AttachWmvVpuSpecificHeader(
	vdec_private_t  *pstVDecPrivate,
	const OMX_U8    *pbyInputStream,
	const OMX_S32    lInputLength,
	const OMX_BOOL   bAttachSeqHeader
	)
{
	OMX_U32 unused = 0;
	OMX_U8 *p_buffer = pstVDecPrivate->pbyTempInputBuff + pstVDecPrivate->lTempInputOffset;
	OMX_S32 input_len = lInputLength;
	OMX_BOOL ret;

	if(p_buffer == NULL) //  dambee7 modify
	{
		ERROR("%s %d p_buffer is NULL!!!!!!!!!!!!!!!\n",__func__,__LINE__);
		ret = OMX_FALSE;
	} 
	else {

		if (bAttachSeqHeader)
		{
			omx_base_video_PortType *p_inport = (omx_base_video_PortType *)pstVDecPrivate->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];
			OMX_S32 width = pstVDecPrivate->stCropRect.nWidth;
			OMX_S32 height = pstVDecPrivate->stCropRect.nHeight;
			OMX_U32 pre_roll = 0xf0000000;
			OMX_U32 rcv_version = 0xc5000000;
			OMX_U32 rcv_header_size = 12;

			OMX_U8 *p_seqhead = pstVDecPrivate->pbySequenceHeader;
			OMX_S32 seqhead_len = pstVDecPrivate->lSeqHeaderLength;

			(void)memcpy((void*)p_buffer,      (void*)&rcv_version,      sizeof(OMX_U32));
			(void)memcpy((void*)(p_buffer+4),  (void*)&seqhead_len,      sizeof(OMX_U32));
			(void)memcpy((void*)(p_buffer+8),  (void*)p_seqhead,         seqhead_len);
			p_buffer += 8+seqhead_len;

			(void)memcpy((void*)p_buffer,      (void*)&height,           sizeof(OMX_U32));
			(void)memcpy((void*)(p_buffer+4),  (void*)&width,            sizeof(OMX_U32));
			(void)memcpy((void*)(p_buffer+8),  (void*)&rcv_header_size,  sizeof(OMX_U32));
			(void)memcpy((void*)(p_buffer+12), (void*)&pre_roll,         sizeof(OMX_U32));
			(void)memcpy((void*)(p_buffer+16), (void*)&unused,           sizeof(OMX_U32));  //bitrate
			(void)memcpy((void*)(p_buffer+20), (void*)&unused,           sizeof(OMX_U32));  //framerate
			p_buffer += 24;

			input_len |= 0x80000000;
		}

		(void)memcpy((void*)p_buffer,     (void*)&input_len,         sizeof(OMX_U32));
		(void)memcpy((void*)(p_buffer+4), (void*)&unused,            sizeof(OMX_U32));  //timestamp
		(void)memcpy((void*)(p_buffer+8), (void*)pbyInputStream,     lInputLength);
		p_buffer += 8+lInputLength;

		// vpu specific stream
		pstVDecPrivate->lTempInputLength = (OMX_S32)(p_buffer - pstVDecPrivate->pbyTempInputBuff);

		ret = OMX_TRUE;
	}
	return ret;
}

static
OMX_BOOL
AttachVc1Syncword(
	vdec_private_t  *pstVDecPrivate,
	const OMX_U8    *pbyInputStream,
	const OMX_S32    lInputLength,
	const OMX_BOOL   bAttachSeqHeader
	)
{
	OMX_U8 *p_buffer = pstVDecPrivate->pbyTempInputBuff + pstVDecPrivate->lTempInputOffset;
	OMX_BOOL ret;

	if(p_buffer == NULL) //  dambee7 modify
	{
		ERROR("%s %d p_buffer is NULL!!!!!!!!!!!!!!!\n",__func__,__LINE__);
		ret = OMX_FALSE;
	}
	else {
		if (bAttachSeqHeader == OMX_TRUE) {
			if ((pstVDecPrivate->pbySequenceHeader) && (pstVDecPrivate->lSeqHeaderLength > 0)) {
				(void)memcpy((void*)p_buffer, (void*)pstVDecPrivate->pbySequenceHeader, pstVDecPrivate->lSeqHeaderLength);
				p_buffer += pstVDecPrivate->lSeqHeaderLength;
			}
		}

		if (pbyInputStream[0] != 0 || pbyInputStream[1] != 0 || pbyInputStream[2] != 1) {
			p_buffer[0] = 0x00;
			p_buffer[1] = 0x00;
			p_buffer[2] = 0x01;
			p_buffer[3] = 0x0D;
			(void)memcpy((void*)(p_buffer+4), (void*)pbyInputStream, lInputLength);
			p_buffer += lInputLength+4;
			pstVDecPrivate->lTempInputLength = (OMX_S32)(p_buffer - pstVDecPrivate->pbyTempInputBuff);
			ret = OMX_TRUE;
		}
		else {
			if (bAttachSeqHeader == OMX_TRUE) {
				(void)memcpy((void*)p_buffer, (void*)pbyInputStream, lInputLength);
				p_buffer += lInputLength;
				pstVDecPrivate->lTempInputLength = (OMX_S32)(p_buffer - pstVDecPrivate->pbyTempInputBuff);
				ret = OMX_TRUE;
			}
			else {
				ret = OMX_FALSE;
			}
		}
	}
	return ret;
}

static
OMX_BOOL
ConvertBitstream(
	vdec_private_t  *pstVDecPrivate,
	const OMX_U8    *pbyInputStream,
	const OMX_S32    lInputLength,
	const OMX_BOOL   bAttachSeqHeader
	)
{
	OMX_BOOL ret;
#if defined(TC_SECURE_MEMORY_COPY)
	if ( CHECK_STATE(pstVDecPrivate, STATE_VDEC_BUFFER_PROTECTION) )
	{
		ret = OMX_TRUE;
	} else
#endif
	{
		if( pstVDecPrivate->lNalLengthSize ) {
			switch (pstVDecPrivate->stVDecInit.m_iBitstreamFormat) {
			case STD_MVC:
			case STD_AVC:
#if defined (TCC_HEVC_INCLUDE) || defined (TCC_VPU_4K_D2_INCLUDE)
			case STD_HEVC:
#endif
				ret = ConvertAvccToAnnexbStream(pstVDecPrivate, pbyInputStream, lInputLength, bAttachSeqHeader);
				break;
			default:
				ret = OMX_TRUE;
				break;					
			}
		}
		else {
			switch (pstVDecPrivate->stVDecInit.m_iBitstreamFormat) {
			case STD_MPEG4:
			case STD_DIV3:
			case STD_VP8:
				ret = AttachVpuSpecificHeader(pstVDecPrivate, pbyInputStream, lInputLength, bAttachSeqHeader);
				break;
			case STD_VC1:
				if ((pstVDecPrivate->stProfileLevel.eProfile == OMX_VIDEO_WMV9ProfileSimple) ||
				     (pstVDecPrivate->stProfileLevel.eProfile == OMX_VIDEO_WMV9ProfileMain)) {
					ret = AttachWmvVpuSpecificHeader(pstVDecPrivate, pbyInputStream, lInputLength, bAttachSeqHeader);
				} else if (pstVDecPrivate->stProfileLevel.eProfile == OMX_VIDEO_WMV9ProfileAdvanced) {
					ret = AttachVc1Syncword(pstVDecPrivate, pbyInputStream, lInputLength, bAttachSeqHeader);
				} else {
					ret = OMX_TRUE;
				}
				break;
			default:
				ret = OMX_TRUE;
				break;
			}
		}
	}

	return ret;
}

static
OMX_S32
ScanSequenceHeader(
	vdec_private_t           *pstVDecPrivate,
	OMX_BYTE                 *pbyStreamBuffer,
	OMX_S32                  lStreamLength,
	OMX_S32                  lStreamType
	)
{
	OMX_U8 *p_buff = pbyStreamBuffer;
	OMX_U8 *p_buff_start = p_buff;
	OMX_U8 *p_buff_end = p_buff + lStreamLength - 4;
	OMX_U32 syncword = 0xFFFFFFFF;
	OMX_S32 SeqHead_pos = -1;

#if defined(TC_SECURE_MEMORY_COPY)
	if ( CHECK_STATE(pstVDecPrivate, STATE_VDEC_BUFFER_PROTECTION) )
	{
		SeqHead_pos = 0;
	}
	else
#endif
	{
		syncword = ((OMX_U32)p_buff[0] << 16) | ((OMX_U32)p_buff[1] << 8) | (OMX_U32)p_buff[2];

		switch(lStreamType) {
			case STD_AVC:
			case STD_MVC:
				while( p_buff < p_buff_end ) {
					syncword <<= 8;
					syncword |= p_buff[3];

					if( (((syncword >> 8LL) & 0x00000000ffffffffLL )== 0x000001LL) && ((syncword & 0x1F) == 7) ){ // nal_type: SPS
						SeqHead_pos = (OMX_S32)(p_buff - p_buff_start);
						break;
					}
					p_buff++;
				}
				break;
#if defined (TCC_HEVC_INCLUDE) || defined (TCC_VPU_4K_D2_INCLUDE)
			case STD_HEVC:
				while( p_buff < p_buff_end ) {
					syncword <<= 8;
					syncword |= p_buff[3];
					if( ((syncword >> 8LL) & 0x00000000ffffffffLL) == 0x000001 ) {
						OMX_U32 nal_type = (syncword >> 1) & 0x3F;
						if( nal_type == 32 || nal_type == 33 ){ // nal_type: VPS/SPS
							SeqHead_pos = (OMX_S32)(p_buff - p_buff_start);
							break;
						}
					}
					p_buff++;
				}
				break;
#endif
			case STD_MPEG4:
				while( p_buff < p_buff_end ) {
					syncword <<= 8;
					syncword |= p_buff[3];
					if( (((syncword >> 8LL) & 0x00000000ffffffffLL) == 0x000001) && (syncword >= 0x00000120) && (syncword <= 0x0000012F) ){ // video_object_layer_start_code
						SeqHead_pos = (OMX_S32)(p_buff - p_buff_start);
						break;
					}
					p_buff++;
				}
				break;
			case STD_VC1:
			case STD_MPEG2:
#ifdef TCC_AVS_INCLUDED
			case STD_AVS:
#endif
			{
				OMX_U32 seqhead_sig = 0;
				if (lStreamType == STD_VC1) {
					seqhead_sig = 0x0000010F; //VC-1 sequence start code
				}
				if (lStreamType == STD_MPEG2) {
					seqhead_sig = 0x000001B3; //MPEG video sequence start code
				}
#ifdef TCC_AVS_INCLUDED
				if (lStreamType == STD_AVS) {
					seqhead_sig = 0x000001B0; //AVS sequence start code
				}
#endif
				while( p_buff < p_buff_end ) {
					syncword <<= 8;
					syncword |= p_buff[3];
					if( syncword == seqhead_sig ){
						SeqHead_pos = (OMX_S32)(p_buff - p_buff_start);
						break;
					}
					p_buff++;
				}
				break;
			}
			default:
				INFO("ScanSequenceHeader() - do not need to find the header (type: %ld)", lStreamType);
				SeqHead_pos = 0;
				break;				
		}
	}

	return SeqHead_pos;
}

static
OMX_BOOL
StoreSequenceHeader(
	vdec_private_t      *pstVDecPrivate,
	OMX_U8              *pbySequenceHeader,
	OMX_S32              nHeaderLength
	)
{
	OMX_BOOL ret = OMX_TRUE;
#if DECODER_RINGBUFFER_MODE && defined(TC_SECURE_MEMORY_COPY)
	if( CHECK_MODE(pstVDecPrivate, MODE_RINGBUFFER_MODE)
		&& CHECK_STATE(pstVDecPrivate, STATE_VDEC_BUFFER_PROTECTION) )
	{
		ringbuff_state_t *p_rb_state = &pstVDecPrivate->stRingBuffState;

		if( pstVDecPrivate->lSeqBuffMapSize <= 0 ) {
			ERROR("StoreSequenceHeader() - invalid map size of the sequence header buffer");
			ret = OMX_FALSE;
		}
		else {
			if( p_rb_state->pRingBuffBase[VA] <= pbySequenceHeader && pbySequenceHeader <= p_rb_state->pRingBuffEnd[VA] )
			{
				TC_SecureMemoryCopy((unsigned int)(pstVDecPrivate->pSeqBuffBase[PA]), (unsigned int)(p_rb_state->pRingBuffBase[PA] + (pbySequenceHeader - p_rb_state->pRingBuffBase[VA])), (unsigned int)(nHeaderLength));
			}
			else {
				(void)memcpy((void*)pstVDecPrivate->pSeqBuffBase[VA], (void*)pbySequenceHeader, nHeaderLength);
			}

			pstVDecPrivate->lSeqHeaderLength = nHeaderLength;
		}
	}
	else
#endif
	{
#if defined(TC_SECURE_MEMORY_COPY)
		if( CHECK_STATE(pstVDecPrivate, STATE_VDEC_BUFFER_PROTECTION) )
		{
			if( pstVDecPrivate->mSeqBackupPmap.iSize != 0 )
			{
				int ret_value = 0;
				ret_value = TC_SecureMemoryCopy(pstVDecPrivate->mSeqBackupPmap.iAddress, pstVDecPrivate->stVDecInput.m_pInp[PA], nHeaderLength);
				//(void)memcpy(pstVDecPrivate->pbySequenceHeader, pstVDecPrivate->stVDecInput.m_pInp[VA], nHeaderLength);
			}
			pstVDecPrivate->lSeqHeaderLength = nHeaderLength;
		}
		else
#endif
		{
			if( nHeaderLength > (1024*100L) ) {
				OMX_S32 new_size;
				OMX_U8 *p_ptr = (OMX_U8 *)((ptrdiff_t)pbySequenceHeader + (ptrdiff_t)nHeaderLength);
				p_ptr--;
				while((p_ptr > pbySequenceHeader) && (*p_ptr == 0)){
					p_ptr--;
				}

				new_size = p_ptr - pbySequenceHeader;
				if(nHeaderLength < (new_size + 4)){
					nHeaderLength = new_size;
				}
			}
			if( nHeaderLength < 4 ) {
				INFO("Sequence header not stored - zero data");
				ret = OMX_TRUE;
			}
			else {
				if( pstVDecPrivate->pbySequenceHeader != NULL ){
					TCC_free(pstVDecPrivate->pbySequenceHeader);
					pstVDecPrivate->lSeqHeaderLength = 0;
				}

				pstVDecPrivate->pbySequenceHeader = (OMX_U8*)TCC_malloc((unsigned int)nHeaderLength);

				if( pstVDecPrivate->pbySequenceHeader == NULL ) {
					ERROR("StoreSequenceHeader() - out of memory");
					ret = OMX_FALSE;
				}
				else {
					(void)memcpy(pstVDecPrivate->pbySequenceHeader, pbySequenceHeader, nHeaderLength);
					pstVDecPrivate->lSeqHeaderLength = nHeaderLength;
					INFO("Sequence header stored");
				}
			}
		}
	}
	return ret;
}

static
OMX_BOOL
ParseConfigData(
	vdec_private_t        *pstVDecPrivate,
	OMX_BUFFERHEADERTYPE  *pInputBuffer
	)
{
	OMX_U8 *p_buff = pInputBuffer->pBuffer + pInputBuffer->nOffset;
	OMX_U32 data_len = pInputBuffer->nFilledLen;

#if defined(TC_SECURE_MEMORY_COPY)
	if(CHECK_STATE(pstVDecPrivate, STATE_VDEC_BUFFER_PROTECTION))
	{
		return OMX_TRUE;
	}
#endif

	switch (pstVDecPrivate->stVDecInit.m_iBitstreamFormat) {
	case STD_MVC:
	case STD_AVC:
		{
			OMX_BOOL is_avcc = OMX_TRUE;
			OMX_U32 syncword;
			OMX_U8 *p_tmp = p_buff;

			// check AvcC box format
			syncword = *p_tmp++;
			syncword <<= 8; syncword |= *p_tmp++;
			syncword <<= 8; syncword |= *p_tmp++;
			syncword <<= 8; syncword |= *p_tmp++;

			if (((syncword >> 8) & 0x00000000ffffffffLL ) == 0x000001) {
				is_avcc = OMX_FALSE;
			}
			else {
				syncword <<= 8;
				syncword |= *p_tmp++;
				if (((syncword >> 8) & 0x00000000ffffffffLL ) == 0x000001) {
					is_avcc = OMX_FALSE;
				}
			}

			if (data_len < 7) {
				is_avcc = OMX_FALSE;
			}

			if (is_avcc) {
				pstVDecPrivate->lSeqHeaderLength = GetAvccToAnnexbSpsPpsLength(p_buff, data_len, &pstVDecPrivate->lNalLengthSize);

				if (pstVDecPrivate->lNalLengthSize > 4) {
					ERROR("ParseConfigData() - invalid nal length size: %ld", pstVDecPrivate->lNalLengthSize);
					return OMX_FALSE;
				}

				if (pstVDecPrivate->lSeqHeaderLength)
				{
					if (pstVDecPrivate->pbySequenceHeader){
						TCC_free(pstVDecPrivate->pbySequenceHeader);
						pstVDecPrivate->lSeqHeaderLength = 0;
					}

					pstVDecPrivate->pbySequenceHeader = (OMX_U8*)TCC_malloc(pstVDecPrivate->lSeqHeaderLength);
					if (pstVDecPrivate->pbySequenceHeader == NULL) {
						ERROR("ParseConfigData() - out of memory");
						return OMX_FALSE;
					}
					if (ConvertAvccToAnnexbSpaPps(p_buff, data_len, pstVDecPrivate->pbySequenceHeader, &pstVDecPrivate->lSeqHeaderLength) == OMX_TRUE) {
						SET_STATE(pstVDecPrivate, STATE_STREAM_CONVERTION_NEEDED);
					}
				}
			}
			else {
				if (StoreSequenceHeader(pstVDecPrivate, p_buff, data_len) == OMX_FALSE) {
					ERROR("StoreSequenceHeader() - failed");
					return OMX_FALSE;
				}
			}
		}
		break;

#if defined (TCC_HEVC_INCLUDE) || defined (TCC_VPU_4K_D2_INCLUDE)
	case STD_HEVC:
		{
			OMX_BOOL is_hvcc = OMX_TRUE;
			OMX_U32 syncword;
			OMX_U8 *p_tmp = p_buff;

			// check AvcC box format
			syncword = *p_tmp++;
			syncword <<= 8; syncword |= *p_tmp++;
			syncword <<= 8; syncword |= *p_tmp++;
			syncword <<= 8; syncword |= *p_tmp++;
			if ((syncword >> 8) == 0x000001) {
				is_hvcc = OMX_FALSE;
			}
			else {
				syncword <<= 8;
				syncword |= *p_tmp++;
				if ((syncword >> 8) == 0x000001) {
					is_hvcc = OMX_FALSE;
				}
			}

			if (data_len < 7) {
				is_hvcc = OMX_FALSE;
			}

			if (is_hvcc) {
				pstVDecPrivate->lSeqHeaderLength = GetHvccToAnnexbSpsPpsLength(p_buff, data_len, &pstVDecPrivate->lNalLengthSize);

				if (pstVDecPrivate->lNalLengthSize > 4) {
					ERROR("ParseConfigData() - invalid nal length size: %ld", pstVDecPrivate->lNalLengthSize);
					return OMX_FALSE;
				}

				if (pstVDecPrivate->lSeqHeaderLength)
				{
#if defined(TC_SECURE_MEMORY_COPY)
					if (CHECK_STATE(pstVDecPrivate, STATE_VDEC_BUFFER_PROTECTION))
					{
						if (pstVDecPrivate->lSeqBuffMapSize <= 0) {
							ERROR("ParseConfigData() - invalid map size of the sequence header buffer");
							return OMX_FALSE;
						}

						if (ConvertHvccToAnnexbSpaPps(p_buff, data_len, pstVDecPrivate->pSeqBuffBase[VA], &pstVDecPrivate->lSeqHeaderLength) == OMX_TRUE) {
							SET_STATE(pstVDecPrivate, STATE_STREAM_CONVERTION_NEEDED);
						}
					}
					else
#endif
					{
						if (pstVDecPrivate->pbySequenceHeader){
							TCC_free(pstVDecPrivate->pbySequenceHeader);
							pstVDecPrivate->lSeqHeaderLength = 0;
						}

						pstVDecPrivate->pbySequenceHeader = (OMX_U8*)TCC_malloc(pstVDecPrivate->lSeqHeaderLength);
						if (pstVDecPrivate->pbySequenceHeader == NULL) {
							ERROR("ParseConfigData() - out of memory");
							return OMX_FALSE;
						}
						if (ConvertHvccToAnnexbSpaPps(p_buff, data_len, pstVDecPrivate->pbySequenceHeader, &pstVDecPrivate->lSeqHeaderLength) == OMX_TRUE) {
							SET_STATE(pstVDecPrivate, STATE_STREAM_CONVERTION_NEEDED);
						}
					}
				}
			}
			else {
				if (StoreSequenceHeader(pstVDecPrivate, p_buff, data_len) == OMX_FALSE) {
					ERROR("StoreSequenceHeader() - failed");
					return OMX_FALSE;
				}
			}
		}
		break;
#endif

#ifdef INCLUDE_WMV78_DEC
	case STD_WMV78:
		if (pstVDecPrivate->pbyExtraDataBuff) {
			TCC_free(pstVDecPrivate->pbyExtraDataBuff);
			pstVDecPrivate->pbyExtraDataBuff = NULL;
		}
		pstVDecPrivate->pbyExtraDataBuff = TCC_malloc(data_len);
		if (pstVDecPrivate->pbyExtraDataBuff == NULL) {
			ERROR("ParseConfigData() - out of memory");
			return OMX_FALSE;
		}
		(void)memcpy(pstVDecPrivate->pbyExtraDataBuff, p_buff, data_len);
		pstVDecPrivate->lExtraDataLength = data_len;
		break;
#endif

	case STD_MPEG4:
	case STD_VC1:
	case STD_MPEG2:
	case STD_H263:
	case STD_DIV3:
	case STD_SH263:
	case STD_MJPG:
	case STD_VP8:
#ifdef TCC_VPU_4K_D2_INCLUDE
   	case STD_VP9:
#endif
#ifdef TCC_EXT_INCLUDED
	case STD_EXT:
#endif
#ifdef TCC_AVS_INCLUDED
	case STD_AVS:
#endif
#ifdef TCC_THEORA_INCLUDED
	case STD_THEORA:
#endif
	default:
		if (StoreSequenceHeader(pstVDecPrivate, p_buff, data_len) == OMX_FALSE) {
			ERROR("StoreSequenceHeader() - failed");
			return OMX_FALSE;
		}

		SET_STATE(pstVDecPrivate, STATE_SEQHEAD_ATTACHMENT_NEEDED);
	}

	return OMX_TRUE;
}

static
OMX_S32
GetDecodedFrameType(
	vdec_private_t     *pstVDecPrivate
	)
{
	return GetFrameType(  pstVDecPrivate->stVDecInit.m_iBitstreamFormat
						, pstVDecPrivate->stVDecOutput.m_DecOutInfo.m_iPicType
						, pstVDecPrivate->stVDecOutput.m_DecOutInfo.m_iPictureStructure);
}

static void IFrameSearchEnable(vdec_private_t *pstVDecPrivate)
{
	if(pstVDecPrivate->stVDecInit.m_iBitstreamFormat == STD_AVC)
	{
		SetFrameSkipMode(pstVDecPrivate, SKIPMODE_NONIDR_SEARCH);
	}
	else
	{
		SetFrameSkipMode(pstVDecPrivate, SKIPMODE_I_SEARCH);
	}
}

static void SetFrameSkipMode(vdec_private_t *pstVDecPrivate, OMX_S32 lSkipMode)
{
	switch(lSkipMode)
	{
	case SKIPMODE_NONE:
	case SKIPMODE_NEXT_B_SKIP:
	case SKIPMODE_B_WAIT:
		{
			pstVDecPrivate->lFrameSkipMode = lSkipMode;

			pstVDecPrivate->stVDecInput.m_iSkipFrameNum      = 0;
			pstVDecPrivate->stVDecInput.m_iFrameSearchEnable = 0;
			pstVDecPrivate->stVDecInput.m_iSkipFrameMode     = 0;

			LOG_SKIP("Disabled");

//			INFO("Skipped Frames: %ld", pstVDecPrivate->lSkippedCount);
		}
		break;
	case SKIPMODE_IDR_SEARCH:
		{
			pstVDecPrivate->lFrameSkipMode = SKIPMODE_I_SEARCH;

			pstVDecPrivate->stVDecInput.m_iSkipFrameNum      = 0;
			pstVDecPrivate->stVDecInput.m_iSkipFrameMode     = 0;
			pstVDecPrivate->lSkippedCount                    = 0;

			if(CHECK_MODE(pstVDecPrivate, MODE_DECODED_FRAME_OUTPUT))
			{
				pstVDecPrivate->stVDecInput.m_iFrameSearchEnable = AVC_NONIDR_PICTURE_SEARCH_MODE;
				LOG_SKIP("I-frame Search (non-IDR slice search)");
			}
			else
			{
				pstVDecPrivate->stVDecInput.m_iFrameSearchEnable = AVC_IDR_PICTURE_SEARCH_MODE;
				LOG_SKIP("I-frame Search (IDR slice search)");
			}
		}
		break;
	case SKIPMODE_NONIDR_SEARCH:
		{
			pstVDecPrivate->lFrameSkipMode = SKIPMODE_I_SEARCH;

			pstVDecPrivate->stVDecInput.m_iSkipFrameNum      = 0;
			pstVDecPrivate->stVDecInput.m_iSkipFrameMode     = 0;
			pstVDecPrivate->lSkippedCount                    = 0;

			pstVDecPrivate->stVDecInput.m_iFrameSearchEnable = AVC_NONIDR_PICTURE_SEARCH_MODE;

			LOG_SKIP("I-frame Search (non-IDR slice search)");
		}
		break;
	case SKIPMODE_I_SEARCH:
		{
			pstVDecPrivate->lFrameSkipMode = SKIPMODE_I_SEARCH;

			pstVDecPrivate->stVDecInput.m_iSkipFrameNum      = 0;
			pstVDecPrivate->stVDecInput.m_iSkipFrameMode     = 0;
			pstVDecPrivate->lSkippedCount                    = 0;

			if(pstVDecPrivate->stVDecInit.m_iBitstreamFormat == STD_AVC)
			{
				if(CHECK_MODE(pstVDecPrivate, MODE_DECODED_FRAME_OUTPUT))
				{
					pstVDecPrivate->stVDecInput.m_iFrameSearchEnable = AVC_NONIDR_PICTURE_SEARCH_MODE;
					LOG_SKIP("I-frame Search (non-IDR slice search)");
				}
				else
				{
					pstVDecPrivate->stVDecInput.m_iFrameSearchEnable = AVC_NONIDR_PICTURE_SEARCH_MODE;
					LOG_SKIP("I-frame Search (non-IDR slice search)");
				}
			}
			else
			{
				pstVDecPrivate->stVDecInput.m_iFrameSearchEnable = AVC_IDR_PICTURE_SEARCH_MODE;
				LOG_SKIP("I-frame Search");
			}
		}
		break;
	case SKIPMODE_B_SKIP:
		{
			pstVDecPrivate->lFrameSkipMode = lSkipMode;

			pstVDecPrivate->stVDecInput.m_iSkipFrameNum      = 1;
			pstVDecPrivate->stVDecInput.m_iFrameSearchEnable = 0;
			pstVDecPrivate->stVDecInput.m_iSkipFrameMode     = VDEC_SKIP_FRAME_ONLY_B;

			LOG_SKIP("B-frame Skip");
		}
		break;
	case SKIPMODE_EXCEPT_I_SKIP:
		{
			pstVDecPrivate->lFrameSkipMode = lSkipMode;

			pstVDecPrivate->stVDecInput.m_iSkipFrameNum      = 1;
			pstVDecPrivate->stVDecInput.m_iFrameSearchEnable = 0;
			pstVDecPrivate->stVDecInput.m_iSkipFrameMode     = VDEC_SKIP_FRAME_EXCEPT_I;

			LOG_SKIP("Except-I-frame Skip");
		}
		break;
	default:
		break;
	}
}

static void SetNextFrameSkipMode(vdec_private_t *pstVDecPrivate, OMX_S32 lSkipMode)
{
//	case SKIPMODE_NEXT_STEP:
	switch(lSkipMode)
	{
		case SKIPMODE_I_SEARCH:
			INFO("If Current FrameType(%ld) is not PIC_TYPE_I(%d), skip this frame.",GetDecodedFrameType(pstVDecPrivate),PIC_TYPE_I);
			if(pstVDecPrivate->stVDecInit.m_iBitstreamFormat == STD_HEVC)
			{
				SetFrameSkipMode(pstVDecPrivate, SKIPMODE_B_WAIT);
			}
			else
			{
				if(pstVDecPrivate->stVDecOutput.m_DecOutInfo.m_iDecodingStatus == VPU_DEC_SUCCESS_FIELD_PICTURE)
				{
					SetFrameSkipMode(pstVDecPrivate, SKIPMODE_NEXT_B_SKIP);
				}
				else
				{
					SetFrameSkipMode(pstVDecPrivate, SKIPMODE_B_SKIP);
				}
			}
			break;
		case SKIPMODE_NEXT_B_SKIP:
			SetFrameSkipMode(pstVDecPrivate, SKIPMODE_B_SKIP);
			break;
		case SKIPMODE_B_SKIP:
			INFO("If Current FrameType(%ld) is PIC_TYPE_B(%d), skip this frame. ",GetDecodedFrameType(pstVDecPrivate),PIC_TYPE_B);
			SetFrameSkipMode(pstVDecPrivate, SKIPMODE_NONE);
			break;
		case SKIPMODE_EXCEPT_I_SKIP:
			INFO("If Current FrameType(%ld) is not PIC_TYPE_I(%d), skip this frame.",GetDecodedFrameType(pstVDecPrivate),PIC_TYPE_I);
			SetFrameSkipMode(pstVDecPrivate, SKIPMODE_NONE);
			break;
		case SKIPMODE_B_WAIT:
			if(GetDecodedFrameType(pstVDecPrivate) == PIC_TYPE_B)
			{
				SetFrameSkipMode(pstVDecPrivate, SKIPMODE_NONE);
			}
			break;
		default:
			SetFrameSkipMode(pstVDecPrivate, SKIPMODE_NONE);
			break;
	}
}
static
OMX_S32
ScanAvcIdrSlice(
	OMX_BUFFERHEADERTYPE	*pInputBuffer
	)
{
	OMX_U8 *p_buff = pInputBuffer->pBuffer + pInputBuffer->nOffset;
	OMX_U32 data_len = pInputBuffer->nFilledLen;
	OMX_U8 *p_buff_start = p_buff;
	OMX_U8 *p_buff_end = p_buff + data_len - 4;
	OMX_U32 syncword = 0xFFFFFFFF;

	syncword = ((OMX_U32)p_buff[0] << 16) | ((OMX_U32)p_buff[1] << 8) | (OMX_U32)p_buff[2];
	while( p_buff < p_buff_end ) {
		if( syncword == 0x000001 && (p_buff[3] & 0x1F) == 5 ) {//IDR slice
			return (OMX_S32)(p_buff - p_buff_start);
		}
		p_buff++;
	}
	return -1;
}

static
OMX_BOOL
IsAvcFrameStart(
	OMX_U8 *pbyAnnexb,
	OMX_S32 *plAnnexbLength
	)
{
	OMX_U8 *p_buff = pbyAnnexb;
	OMX_U32 data_len = plAnnexbLength;
	OMX_U8 *p_buff_end = p_buff + data_len - 4;
	OMX_U32 syncword = 0xFFFFFFFF;
	OMX_S32 nal_type;

	syncword = ((OMX_U32)p_buff[0] << 16) | ((OMX_U32)p_buff[1] << 8) | (OMX_U32)p_buff[2];
	while( p_buff < p_buff_end ) {
		nal_type = p_buff[3] & 0x1F;
		if( syncword == 0x000001 && (nal_type == 1 || nal_type == 5) ) { //slice or IDR-slice
			if( p_buff[4] >> 7 ){
				return OMX_TRUE;
			} else {
				return OMX_FALSE;
			}
		}

		p_buff++;
	}
	return FALSE;
}


////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//	Decoder Ring-buffer operators
//
//
#if DECODER_RINGBUFFER_MODE

static OMX_BOOL InitRingBuffer(ringbuff_state_t *pstRingbuff, vdec_private_t *pstVDecPrivate);
static OMX_BOOL FillRingBuffer(ringbuff_state_t *pstRingbuff, OMX_U8 *pInputBuff, OMX_S32 lInputLength);
static OMX_S32 UpdateRingBuffer(ringbuff_state_t *pstRingbuff, OMX_BOOL bFromDecoder);
static OMX_S32 ResetRingBuffer(ringbuff_state_t *pstRingbuff, OMX_BOOL bFlush);
static OMX_BOOL CheckRemainRingBufferSize(ringbuff_state_t *pstRingbuff, vdec_private_t *pstVDecPrivate, OMX_S32 *validsize);

static
OMX_BOOL
InitRingBuffer(
	ringbuff_state_t   *pstRingbuff,
	vdec_private_t     *pstVDecPrivate
	)
{
	vdec_ring_buffer_out_t ring_stat;
	OMX_S32 ret = 0;

	INFO("decoder ring-buffer initialize");

	pstRingbuff->pstVDecPrivate = pstVDecPrivate;
	pstRingbuff->pRingBuffBase[PA] = vpu_getBitstreamBufAddr(PA, pstVDecPrivate->pVDecInstance);
	pstRingbuff->pRingBuffBase[VA] = vpu_getBitstreamBufAddr(VA, pstVDecPrivate->pVDecInstance);
	pstRingbuff->lRingBuffSize = vpu_getBitstreamBufSize(pstVDecPrivate->pVDecInstance);
	pstRingbuff->pRingBuffEnd[PA] = pstRingbuff->pRingBuffBase[PA] + pstRingbuff->lRingBuffSize;
	pstRingbuff->pRingBuffEnd[VA] = pstRingbuff->pRingBuffBase[VA] + pstRingbuff->lRingBuffSize;

	ResetRingBuffer(pstRingbuff, FALSE);

	if( (ret = VDEC_FUNC(pstVDecPrivate, VDEC_GET_RING_BUFFER_STATUS, NULL, NULL, &ring_stat)) < 0 ) {
		LOGE("[VDEC_ERROR] [OP: VDEC_GET_RING_BUFFER_STATUS] [RET_CODE: %ld]", ret);
		return OMX_FALSE;
	}

	LOG_RING("[INIT     ] - RING-BUFFER Environment - PA: 0x%08X / VA: 0x%08X / size: %ld"
			 , pstRingbuff->pRingBuffBase[PA]
			 , pstRingbuff->pRingBuffBase[VA]
			 , pstRingbuff->lRingBuffSize);

	LOG_RING("[INIT     ] - [RP: %7ld][WP: %7ld] [EMPTY: %7ld bytes] [BUFFERED: %7ld bytes]"
			 , ring_stat.m_ptrReadAddr_PA-(OMX_U32)pstRingbuff->pRingBuffBase[PA]
			 , ring_stat.m_ptrWriteAddr_PA-(OMX_U32)pstRingbuff->pRingBuffBase[PA]
			 , pstRingbuff->lEmptySpace
			 , pstRingbuff->lWrittenBytes);

	return OMX_TRUE;
}

static
OMX_BOOL
FillRingBuffer(
	ringbuff_state_t        *pstRingbuff,
	OMX_U8                  *pInputBuff,
	OMX_S32                  lInputLength
	)
{
	OMX_BOOL ret = OMX_FALSE;
	vdec_private_t *pstVDecPrivate = pstRingbuff->pstVDecPrivate;
	OMX_S32 copy_size;

	if( pstRingbuff->lEmptySpace > (lInputLength + EMPTY_SIZE_MIN) ) {
		copy_size = lInputLength;
	} else {
		copy_size = ( pstRingbuff->lEmptySpace > EMPTY_SIZE_MIN ) ?
					( pstRingbuff->lEmptySpace - EMPTY_SIZE_MIN ) : (0);
	}
	pstVDecPrivate->mInBufUsedSize = copy_size;

	if( copy_size > 0 )
	{
		OMX_U8  *p_wp        = pstRingbuff->pWritePtr;
		OMX_U8  *p_data      = pInputBuff;
		OMX_S32  data_size   = copy_size;
		OMX_S32  tmp_size    = (OMX_S32)(pstRingbuff->pRingBuffEnd[VA] - p_wp);

		pstRingbuff->pPrevWritePtr = p_wp;

		if( tmp_size < data_size ) {
			OMX_U8  *p_ring_base = pstRingbuff->pRingBuffBase[VA];

			(void)memcpy(p_wp, p_data, tmp_size);
			data_size -= tmp_size;
			p_data += tmp_size;

			(void)memcpy(p_ring_base, p_data, data_size);
			p_wp = p_ring_base + data_size;
		}
		else {
			(void)memcpy(p_wp, p_data, data_size);
			p_wp += data_size;
		}

		pstRingbuff->pWritePtr = p_wp;
		pstRingbuff->lWrittenBytes += copy_size;
		pstRingbuff->lEmptySpace -= copy_size;

		ret = OMX_TRUE;
	}

	LOG_RING("[FILL     ] - [RP: %7ld][WP: %7ld] [EMPTY: %7ld bytes] [BUFFERED: %7ld bytes] [INPUT: %7ld bytes] %s"
			 , pstRingbuff->pReadPtr-(OMX_U32)pstRingbuff->pRingBuffBase[VA]
			 , pstRingbuff->pWritePtr-(OMX_U32)pstRingbuff->pRingBuffBase[VA]
			 , pstRingbuff->lEmptySpace
			 , pstRingbuff->lWrittenBytes
			 , copy_size
			 , ((ret == OMX_FALSE) ? ("- [NOT ENOUGH]") : (" "))
			 );

	return ret;
}

#if defined(TC_SECURE_MEMORY_COPY)
static
OMX_BOOL
FillRingBuffer_secure(
	ringbuff_state_t        *pstRingbuff,
	OMX_U8                  *pPhyInputBuff,	// physical address
	OMX_S32                  lInputLength
	)
{
	OMX_BOOL ret = OMX_FALSE;

	if( pstRingbuff->lEmptySpace > (lInputLength + EMPTY_SIZE_MIN) )
	{
		OMX_U8  *p_wp        = pstRingbuff->pRingBuffBase[PA] + (pstRingbuff->pWritePtr - pstRingbuff->pRingBuffBase[VA]);
		OMX_U8  *p_data      = pPhyInputBuff;
		OMX_S32  data_size   = lInputLength;
		OMX_S32  tmp_size    = (OMX_S32)(pstRingbuff->pRingBuffEnd[PA]-p_wp);

		pstRingbuff->pPrevWritePtr = pstRingbuff->pWritePtr;

		if( tmp_size < data_size ) {
			OMX_U8  *p_ring_base = pstRingbuff->pRingBuffBase[PA];

			TC_SecureMemoryCopy((unsigned int)p_wp, (unsigned int)p_data, (unsigned int)tmp_size);
			data_size -= tmp_size;
			p_data += tmp_size;

			TC_SecureMemoryCopy((unsigned int)p_ring_base, (unsigned int)p_data, (unsigned int)data_size);
			p_wp = p_ring_base + data_size;
		}
		else {
			TC_SecureMemoryCopy((unsigned int)p_wp, (unsigned int)p_data, (unsigned int)data_size);
			p_wp += data_size;
		}

		pstRingbuff->pWritePtr = pstRingbuff->pRingBuffBase[VA] + (p_wp - pstRingbuff->pRingBuffBase[PA]);
		pstRingbuff->lWrittenBytes += lInputLength;
		pstRingbuff->lEmptySpace -= lInputLength;

		ret = OMX_TRUE;
	}

	LOG_RING("[FILL     ] - [RP: %7ld][WP: %7ld] [EMPTY: %7ld bytes] [BUFFERED: %7ld bytes] [INPUT: %7ld bytes] %s"
			 , pstRingbuff->pReadPtr-(OMX_U32)pstRingbuff->pRingBuffBase[VA]
			 , pstRingbuff->pWritePtr-(OMX_U32)pstRingbuff->pRingBuffBase[VA]
			 , pstRingbuff->lEmptySpace
			 , pstRingbuff->lWrittenBytes
			 , lInputLength
			 , ((ret == OMX_FALSE) ? ("- [NOT ENOUGH]") : (" "))
			 );

	return ret;
}
#endif

static
OMX_S32
UpdateRingBuffer(
	ringbuff_state_t        *pstRingbuff,
	OMX_BOOL                 bFromDecoder
	)
{
	OMX_S32 ret = 0;

	if (bFromDecoder)
	{
		vdec_ring_buffer_out_t ring_stat;
		OMX_U8 *p_ring_base_pa = pstRingbuff->pRingBuffBase[PA];
		OMX_U8 *p_ring_base_va = pstRingbuff->pRingBuffBase[VA];
		OMX_U8 *p_prev_rp;
		OMX_U8 *p_curr_rp;
		OMX_S32 ret = 0;

		if( (ret = VDEC_FUNC(pstRingbuff->pstVDecPrivate, VDEC_GET_RING_BUFFER_STATUS, NULL, NULL, &ring_stat)) < 0 ) {
			LOGE("[VDEC_ERROR] [OP: VDEC_GET_RING_BUFFER_STATUS] [RET_CODE: %ld]", ret);
			return ret;
		}

		if (ring_stat.m_ptrReadAddr_PA > (pstRingbuff->pRingBuffBase[PA] + pstRingbuff->lRingBuffSize))
		{
			LOGD("read ptr not valid address.");
			return -1;
		}

		ASSERT( ((pstRingbuff->pReadPtr <= pstRingbuff->pWritePtr) && (ring_stat.m_ptrReadAddr_PA <= ring_stat.m_ptrWriteAddr_PA))
					|| ((pstRingbuff->pReadPtr > pstRingbuff->pWritePtr)) );

		#if USE_AVAILABLE_SIZE_FROM_VDEC
		ASSERT( (pstRingbuff->lEmptySpace + pstRingbuff->lWrittenBytes) <= (ring_stat.m_ulAvailableSpaceInRingBuffer) );
		#endif

		/* prev read pointer */
		p_prev_rp = pstRingbuff->pReadPtr;
		pstRingbuff->pPrevReadPtr = p_prev_rp;

		/* read pointer */
		p_curr_rp = p_ring_base_va + ((OMX_U8*)ring_stat.m_ptrReadAddr_PA - p_ring_base_pa);
		pstRingbuff->pReadPtr = p_curr_rp;

		/* used bytes */
		//tcc_printf("UpdateRingBuffer: ReadAddr_PA 0x%8x \n", (OMX_U8*)ring_stat.m_ptrReadAddr_PA);
		//tcc_printf("UpdateRingBuffer: prev_rp 0x%8x curr_rp 0x%8x \n", p_prev_rp, p_curr_rp);

		if( p_curr_rp < p_prev_rp ){
			pstRingbuff->lUsedByte = (p_curr_rp - p_ring_base_va) + (pstRingbuff->pRingBuffEnd[VA] - p_prev_rp);
		} else {
			pstRingbuff->lUsedByte = p_curr_rp - p_prev_rp;
		}
		ASSERT( pstRingbuff->lUsedByte <= (pstRingbuff->lRingBuffSize - (pstRingbuff->lEmptySpace + pstRingbuff->lWrittenBytes)) );

		/* empty size */
		#if USE_AVAILABLE_SIZE_FROM_VDEC
		pstRingbuff->lEmptySpace = ring_stat.m_ulAvailableSpaceInRingBuffer - pstRingbuff->lWrittenBytes;
		#else
		{
			OMX_S32 temp_size;
			if( ring_stat.m_ptrReadAddr_PA <= ring_stat.m_ptrWriteAddr_PA ){
				temp_size = pstRingbuff->lRingBuffSize - (OMX_S32)(ring_stat.m_ptrWriteAddr_PA - ring_stat.m_ptrReadAddr_PA);
			} else {
				temp_size = (OMX_S32)(ring_stat.m_ptrReadAddr_PA - ring_stat.m_ptrWriteAddr_PA);
			}
			temp_size -= pstRingbuff->lWrittenBytes;
			pstRingbuff->lEmptySpace = temp_size;
		}
		#endif

		LOG_RING("[UPDATE-DN] - [RP: %7ld][WP: %7ld] [EMPTY: %7ld bytes] [BUFFERED: %7ld bytes] [USED : %7ld bytes]"
				 , ring_stat.m_ptrReadAddr_PA-(OMX_U32)pstRingbuff->pRingBuffBase[PA]
				 , ring_stat.m_ptrWriteAddr_PA-(OMX_U32)pstRingbuff->pRingBuffBase[PA]
				 , pstRingbuff->lEmptySpace
				 , pstRingbuff->lWrittenBytes
				 , pstRingbuff->lUsedByte
				 );
	}
	else
	{
#if UPDATE_WRITE_PTR_WITH_ALIGNED_LENGTH
		if( pstRingbuff->lWrittenBytes < WRITE_PTR_ALIGN_BYTES ){
			return 0;
		} else {
			OMX_S32 unaligned_byte = pstRingbuff->lWrittenBytes % WRITE_PTR_ALIGN_BYTES;

			ret = VDEC_FUNC(pstRingbuff->pstVDecPrivate, VDEC_UPDATE_WRITE_BUFFER_PTR, NULL,
										(unsigned long*)(pstRingbuff->lWrittenBytes - unaligned_byte), (unsigned long*)pstRingbuff->bFlushRing);
			if( ret < 0 ) {
				LOGE( "[VDEC_ERROR] [OP: VDEC_UPDATE_WRITE_BUFFER_PTR] [RET_CODE: %ld]", ret);
				return ret;
			}
			pstRingbuff->lWrittenBytes = unaligned_byte;
			pstRingbuff->bFlushRing = OMX_FALSE;
		}
#else
		ret = VDEC_FUNC(pstRingbuff->pstVDecPrivate, VDEC_UPDATE_WRITE_BUFFER_PTR, NULL,
										(unsigned long*)pstRingbuff->lWrittenBytes, (unsigned long*)pstRingbuff->bFlushRing);
		if( ret < 0 ) {
			LOGE( "[VDEC_ERROR] [OP: VDEC_UPDATE_WRITE_BUFFER_PTR] [RET_CODE: %ld]", ret);
			return ret;
		}
		pstRingbuff->lWrittenBytes = 0;
		pstRingbuff->bFlushRing = OMX_FALSE;
#endif

		#if CHECK_RING_STATUS_AFTER_UPDATE
		{
			vdec_ring_buffer_out_t ring_stat;
			if( (ret = VDEC_FUNC(pstRingbuff->pstVDecPrivate, VDEC_GET_RING_BUFFER_STATUS, NULL, NULL, &ring_stat)) < 0 ) {
				LOGE("[VDEC_ERROR] [OP: VDEC_GET_RING_BUFFER_STATUS] [RET_CODE: %ld]", ret);
				return ret;
			}

			if( (pstRingbuff->pWritePtr-pstRingbuff->lWrittenBytes) != pstRingbuff->pRingBuffBase[VA] + (ring_stat.m_ptrWriteAddr_PA-(OMX_U32)pstRingbuff->pRingBuffBase[PA]) ) {
				LOGE("[LOG_RING] [RP: %d / %d] [WP: %d / %d] "
					 , pstRingbuff->pReadPtr-(OMX_U32)pstRingbuff->pRingBuffBase[VA]		//rp offset (VA)
					 , ring_stat.m_ptrReadAddr_PA-(OMX_U32)pstRingbuff->pRingBuffBase[PA]	//rp offset (PA)
					 , pstRingbuff->pWritePtr-(OMX_U32)pstRingbuff->pRingBuffBase[VA]		//wp offset (VA)
					 , ring_stat.m_ptrWriteAddr_PA-(OMX_U32)pstRingbuff->pRingBuffBase[PA]	//wp offset (PA)
					 );
			}

			#if USE_AVAILABLE_SIZE_FROM_VDEC
			if( pstRingbuff->lEmptySpace != ring_stat.m_ulAvailableSpaceInRingBuffer ) {
				LOGE("[LOG_RING] EMPTY: %ld / %lu", pstRingbuff->lEmptySpace, ring_stat.m_ulAvailableSpaceInRingBuffer);
			}
			#else
			{
				long temp_size;
				if( ring_stat.m_ptrReadAddr_PA <= ring_stat.m_ptrWriteAddr_PA ){
					temp_size = pstRingbuff->lRingBuffSize - (OMX_S32)(ring_stat.m_ptrWriteAddr_PA - ring_stat.m_ptrReadAddr_PA);
				} else {
					temp_size = (OMX_S32)(ring_stat.m_ptrReadAddr_PA - ring_stat.m_ptrWriteAddr_PA);
				}
				if( pstRingbuff->lEmptySpace != temp_size ) {
					LOGE("[LOG_RING] EMPTY: %ld / %ld", pstRingbuff->lEmptySpace, temp_size);
				}
			}
			#endif
		}
		#endif

		LOG_RING("[UPDATE-UP] - [RP: %7ld][WP: %7ld] [EMPTY: %7ld bytes] [BUFFERED: %7ld bytes]"
				 , pstRingbuff->pReadPtr-(OMX_U32)pstRingbuff->pRingBuffBase[VA]
				 , pstRingbuff->pWritePtr-(OMX_U32)pstRingbuff->pRingBuffBase[VA]
				 , pstRingbuff->lEmptySpace
				 , pstRingbuff->lWrittenBytes
				 );
	}
	return 0;
}

static
OMX_S32
ResetRingBuffer(
	ringbuff_state_t	*pstRingbuff ,
	OMX_BOOL					bFlush
	)
{
	OMX_S32 ret = 0;

	pstRingbuff->pWritePtr     = pstRingbuff->pRingBuffBase[VA];
	pstRingbuff->pPrevWritePtr = pstRingbuff->pRingBuffBase[VA];
	pstRingbuff->pReadPtr      = pstRingbuff->pRingBuffBase[VA];
	pstRingbuff->pPrevReadPtr  = pstRingbuff->pRingBuffBase[VA];
	pstRingbuff->lWrittenBytes = 0;
	pstRingbuff->lEmptySpace   = pstRingbuff->lRingBuffSize;
	pstRingbuff->bFlushRing    = bFlush;

#if FLUSH_RING_BEFORE_UPDATE_TIME
	ret = VDEC_FUNC(pstRingbuff->pstVDecPrivate, VDEC_UPDATE_WRITE_BUFFER_PTR, NULL,
									(unsigned long*)0UL, (unsigned long*)pstRingbuff->bFlushRing);
	if( ret < 0 ){
		LOGE( "[VDEC_ERROR] [OP: VDEC_UPDATE_WRITE_BUFFER_PTR] [RET_CODE: %ld]", ret);
	}
	pstRingbuff->bFlushRing = OMX_FALSE;

	if( ret == 0 ) {
		vdec_ring_buffer_out_t ring_stat;

		ret = VDEC_FUNC(pstRingbuff->pstVDecPrivate, VDEC_GET_RING_BUFFER_STATUS, NULL, NULL, &ring_stat);
		if( ret < 0 )
		{
			LOGE("[VDEC_ERROR] [OP: VDEC_GET_RING_BUFFER_STATUS] [RET_CODE: %ld]", ret);
		}
		else
		{
			pstRingbuff->pReadPtr += ((OMX_U8*)ring_stat.m_ptrReadAddr_PA - pstRingbuff->pRingBuffBase[PA]);
			pstRingbuff->pWritePtr += ((OMX_U8*)ring_stat.m_ptrWriteAddr_PA - pstRingbuff->pRingBuffBase[PA]);
			pstRingbuff->pPrevWritePtr = pstRingbuff->pWritePtr;
			#if USE_AVAILABLE_SIZE_FROM_VDEC
			pstRingbuff->lEmptySpace = ring_stat.m_ulAvailableSpaceInRingBuffer;
			#endif
		}
	}
#else
#endif

	LOG_RING("[RESET    ] - [RP: %7ld][WP: %7ld] [EMPTY: %7ld bytes] [BUFFERED: %7ld bytes]"
			 , pstRingbuff->pReadPtr-(OMX_U32)pstRingbuff->pRingBuffBase[VA]
			 , pstRingbuff->pWritePtr-(OMX_U32)pstRingbuff->pRingBuffBase[VA]
			 , pstRingbuff->lEmptySpace
			 , pstRingbuff->lWrittenBytes
			 );

	return ret;
}

static OMX_BOOL CheckRemainRingBufferSize(ringbuff_state_t *pstRingbuff, vdec_private_t *pstVDecPrivate, OMX_S32 *validsize)
{
	OMX_S32 remainbuffersize;
	OMX_S32 ret = 0;
	OMX_BOOL need_more_data = OMX_FALSE;
	vdec_ring_buffer_out_t ring_stat;

	if (pstRingbuff) {
		OMX_S32 temp_size;
		if( (ret = VDEC_FUNC(pstRingbuff->pstVDecPrivate, VDEC_GET_RING_BUFFER_STATUS, NULL, NULL, &ring_stat)) < 0 ) {
			LOGE("[VDEC_ERROR] [OP: VDEC_GET_RING_BUFFER_STATUS] [RET_CODE: %ld] - %s", ret ,__func__);
		}
		else {
			if( ring_stat.m_ptrReadAddr_PA <= ring_stat.m_ptrWriteAddr_PA ){
				temp_size = pstRingbuff->lRingBuffSize - (OMX_S32)(ring_stat.m_ptrWriteAddr_PA - ring_stat.m_ptrReadAddr_PA);
			} else {
				temp_size = (OMX_S32)(ring_stat.m_ptrReadAddr_PA - ring_stat.m_ptrWriteAddr_PA);
			}

			remainbuffersize = (OMX_S32)pstRingbuff->lRingBuffSize - (OMX_S32)temp_size;
		}
	}
	else {
		remainbuffersize = 0;
	}

	if (ret >= 0) {
		if ((pstVDecPrivate->stVDecInit.m_iBitstreamFormat == STD_HEVC) &&
		   ((pstVDecPrivate->stVDecInit.m_iPicWidth*pstVDecPrivate->stVDecInit.m_iPicHeight) > MIN_RESOLUTION_FOR_MAP_CONVERTER) &&
		   (remainbuffersize < 2000000) ) {
		   need_more_data = OMX_TRUE;
		}

		if (validsize) {
			*validsize = remainbuffersize;
		}
	}
	return need_more_data;
}
#endif //DECODER_RINGBUFFER_MODE


////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//	Decoder operation helper
//
//
static OMX_S32 GetFrameType(OMX_S32 lCodecType, OMX_S32 lPictureType, OMX_S32 lPictureStructure);
static OMX_BOOL IsSupportMaxFramebufferMode(vdec_private_t *pstVDecPrivate);
static void SendEventToClient(vdec_private_t *pstVDecPrivate, OMX_EVENTTYPE eEvent, OMX_U32 nData1, OMX_U32 nData2);
static void SendEventToClient2(vdec_private_t *pstVDecPrivate, OMX_EVENTTYPE eEvent, OMX_U32 nData1, OMX_U32 nData2, void *pData3);
static OMX_S32 CheckPortConfigChange(vdec_private_t *pstVDecPrivate);
static OMX_BOOL IsResolutionChanged(vdec_private_t *pstVDecPrivate);
static OMX_U8* CopyOutputFrame(vdec_private_t *pstVDecPrivate, OMX_U8 *pDestAddress, OMX_U32 *pulCopyedBytes, OMX_S32 lSrcIndex);
static OMX_S32 GetOutputBufferInfo(vdec_private_t *pstVDecPrivate, OMX_S32 lBuffIdx, output_info_t *pstOutputInfo);
static OMX_S32 ClearAllDisplayBuffers(vdec_private_t *pstVDecPrivate);
static OMX_S32 ClearDisplayedBuffer(vdec_private_t *pstVDecPrivate, OMX_BOOL bForced);

#if DECODER_RINGBUFFER_MODE
static OMX_S32 ScanFillerSpace(vdec_private_t *pstVDecPrivate, OMX_U8 *pScanStart, OMX_U8 *pScanEnd, OMX_U8 **ppSpaceEnd);
static OMX_TICKS GetCurrTimestamp(vdec_private_t *pstVDecPrivate, OMX_TICKS *pllTimestamp, OMX_BOOL bDecodeSuccess, OMX_BOOL bInterlaced);
static OMX_BOOL BackupRingBuffer(vdec_private_t *pstVDecPrivate);
static OMX_BOOL RestoreRingBuffer(vdec_private_t *pstVDecPrivate);
static OMX_S32 FeedDecoder(vdec_private_t *pstVDecPrivate, OMX_BUFFERHEADERTYPE *pInputBuffer);
#endif

static OMX_S32 DecoderInit(vdec_private_t *pstVDecPrivate);
static void DecoderDeinit(vdec_private_t *pstVDecPrivate);
static OMX_S32 DecoderSeqInit(vdec_private_t *pstVDecPrivate);
static OMX_S32 OMXVideoDecode_Decode(vdec_private_t *pstVDecPrivate,OMX_U32 *pulResultFlags);
static OMX_S32 DecSuccessProcess(vdec_private_t *pstVDecPrivate, OMX_TICKS llDecTimestamp, OMX_S32 lBuffIdx, OMX_BOOL bFieldSuccess);
static OMX_S32 OutputProcess(vdec_private_t *pstVDecPrivate, OMX_BUFFERHEADERTYPE *pOutputBuffer, OMX_BOOL bDecBuffOutput, OMX_S32 *plMbError);
static OMX_BOOL ErrorProcess(vdec_private_t *pstVDecPrivate, OMX_BUFFERHEADERTYPE *pInputBuffer, OMX_BUFFERHEADERTYPE *pOutputBuffer, OMX_S32 ret, OMX_S32 lProcessStatus);
static void PrepareResolutionChange(vdec_private_t *pstVDecPrivate);

static inline OMX_S32 UpdateFrameSize(vdec_private_t *pstVDecPrivate);

static OMX_S32 GetUserDataLength(OMX_U8 *pbyUserData);
static void SendUserData(vdec_private_t *pstVDecPrivate, OMX_U8 *pbyUserData, OMX_S32 lDataLength, OMX_TICKS llTimestamp);

static
OMX_S32
GetFrameType(
	OMX_S32 lCodecType,
	OMX_S32 lPictureType,
	OMX_S32 lPictureStructure
	)
{
	OMX_S32 frameType = -1; //unknown

	switch ( lCodecType )
	{
	case STD_VC1 :
		if( lPictureStructure == 3) {
			switch( (lPictureType&0x7) ) {// FIELD_INTERLACED(BOTTOM FIELD)
			case PIC_TYPE_I:
				frameType = PIC_TYPE_I;
				break;//I
			case PIC_TYPE_P:
				frameType = PIC_TYPE_P;
				break;//P
			case 2:
				frameType = PIC_TYPE_B;
				break;//B //DSTATUS( "BI  :" );
			case 3:
				frameType = PIC_TYPE_B;
				break;//B //DSTATUS( "B   :" );
			case 4:
				frameType = PIC_TYPE_B;
				break;//B //DSTATUS( "SKIP:" );
			}
		}
		else {
			#if !defined(TCC_892X_INCLUDE) && !defined(TCC_893X_INCLUDE)
			lPictureType >>= 3;
			#endif
			switch( lPictureType ) {//Frame or // FIELD_INTERLACED(TOP FIELD)
			case PIC_TYPE_I:
				frameType = PIC_TYPE_I;
				break;//I
			case PIC_TYPE_P:
				frameType = PIC_TYPE_P;
				break;//P
			case 2:
				frameType = PIC_TYPE_B;
				break;//B //DSTATUS( "BI  :" );
			case 3:
				frameType = PIC_TYPE_B;
				break;//B //DSTATUS( "B   :" );
			case 4:
				frameType = PIC_TYPE_B;
				break;//B //DSTATUS( "SKIP:" );
			}
		}
		break;
	case STD_MPEG4 :
		switch( lPictureType ) {
		case PIC_TYPE_I:
			frameType = PIC_TYPE_I;break;//I
		case PIC_TYPE_P:
			frameType = PIC_TYPE_P;
			break;//P
		case PIC_TYPE_B:
			frameType = PIC_TYPE_B;
			break;//B
		case PIC_TYPE_B_PB:
			frameType = PIC_TYPE_B;
			break;//B of Packed PB-frame
		}
		break;
	case STD_MPEG2 :
	default:
		switch( lPictureType & 0xF ) {
#ifdef PIC_TYPE_IDR
		case PIC_TYPE_IDR:
#endif
		case PIC_TYPE_I:
			frameType = PIC_TYPE_I;
			break;//I
		case PIC_TYPE_P:
			frameType = PIC_TYPE_P;
			break;//P
		case PIC_TYPE_B:
			frameType = PIC_TYPE_B;
			break;//B
		}
	}
	return frameType;
}

static
OMX_BOOL
IsSupportMaxFramebufferMode(
	vdec_private_t *pstVDecPrivate
	)
{
    OMX_U32  video_coding_type = pstVDecPrivate->enVideoCodingType;
    return (video_coding_type == OMX_VIDEO_CodingAVC || video_coding_type == OMX_VIDEO_CodingMPEG2
#if defined (TCC_VPU_4K_D2_INCLUDE)
                    || video_coding_type == OMX_VIDEO_CodingVP9
#endif
           );
}

static
void
SendEventToClient(
	vdec_private_t  *pstVDecPrivate,
	OMX_EVENTTYPE    eEvent,
	OMX_U32          nData1,
	OMX_U32          nData2
	)
{
	SendEventToClient2(pstVDecPrivate, eEvent, nData1, nData2, NULL);
}

static
void
SendEventToClient2(
	vdec_private_t  *pstVDecPrivate,
	OMX_EVENTTYPE    eEvent,
	OMX_U32          nData1,
	OMX_U32          nData2,
	void			*pData3
	)
{
	(*(pstVDecPrivate->callbacks->EventHandler))(
			pstVDecPrivate->openmaxStandComp,
			pstVDecPrivate->callbackData,
			eEvent,
			nData1,
			nData2,
			pData3);
}

static
OMX_S32
CheckPortConfigChange(
	vdec_private_t   *pstVDecPrivate
	)
{
#define MAX_RESOLUTION_EXCEEDED         (-1)
#define PORT_RECONFIGURATION_NEEDED     (0x1)
#define PORT_CROP_CHANGE_NEEDED         (0x2)
#define PORT_AR_CHANGE_NEEDED           (0x4)
#define PORT_CONFIGURATION_DONE         (0)

	omx_base_video_PortType *p_outport = (omx_base_video_PortType *)pstVDecPrivate->ports[OMX_BASE_FILTER_OUTPUTPORT_INDEX];
	vdec_initial_info_t *p_init_info = pstVDecPrivate->stVDecOutput.m_pInitialInfo;
#if defined(JPEG_DECODE_FOR_MJPEG)
	OMX_COLOR_FORMATTYPE colorformat;
#endif
	OMX_CONFIG_RECTTYPE st_crop;
	OMX_U32 width;
	OMX_U32 height;
	OMX_U32 port_width = p_outport->sPortParam.format.video.nFrameWidth;
	OMX_U32 port_height = p_outport->sPortParam.format.video.nFrameHeight;

	OMX_S32 ret = PORT_CONFIGURATION_DONE;

#if AVC_CROP_ENABLE
	if(    pstVDecPrivate->stVDecInit.m_iBitstreamFormat == STD_AVC
		|| pstVDecPrivate->stVDecInit.m_iBitstreamFormat == STD_MVC
#if defined (TCC_HEVC_INCLUDE) || defined (TCC_VPU_4K_D2_INCLUDE)
		|| pstVDecPrivate->stVDecInit.m_iBitstreamFormat == STD_HEVC
#endif
#if defined (TCC_VPU_4K_D2_INCLUDE)
		|| pstVDecPrivate->stVDecInit.m_iBitstreamFormat == STD_VP9
#endif
		)
	{
		pic_crop_t avc_crop = p_init_info->m_iPicCrop;

		st_crop.nLeft    = avc_crop.m_iCropLeft;
		st_crop.nTop     = avc_crop.m_iCropTop;
		st_crop.nWidth   = (p_init_info->m_iPicWidth - avc_crop.m_iCropLeft - avc_crop.m_iCropRight);
		st_crop.nHeight  = (p_init_info->m_iPicHeight - avc_crop.m_iCropBottom - avc_crop.m_iCropTop);

#if AVC_CROP_APPLIED_TO_PORT
		width  = st_crop.nWidth;
		height = st_crop.nHeight;
		width  += width & 1;
		height += height & 1;
#else
		width  = p_init_info->m_iPicWidth;
		height = p_init_info->m_iPicHeight;
#endif

#if RESOLUTION_CHANGE_WITH_CROP && SET_FULLHD_TO_PORT
		if( pstVDecPrivate->stVDecUserInfo.extFunction & EXT_FUNC_MAX_FRAMEBUFFER ) {
			if( (port_width != pstVDecPrivate->lPicWidthMax) || (port_height != pstVDecPrivate->lPicHeightMax)){
				ret |= PORT_RECONFIGURATION_NEEDED;
			}
		}
		else
#endif
		{
			if( port_width != width || port_height != height )
			{
				SET_STATE(pstVDecPrivate, STATE_RESOLUTION_CHANGING_WITH_REINIT);
				ret |= PORT_RECONFIGURATION_NEEDED;
			}
		}
	}
	else
#endif
	{
		width  = p_init_info->m_iPicWidth;
		height = p_init_info->m_iPicHeight;
		width  += width & 1;
		height += height & 1;

#if RESOLUTION_CHANGE_WITH_CROP && SET_FULLHD_TO_PORT
		if( pstVDecPrivate->stVDecUserInfo.extFunction & EXT_FUNC_MAX_FRAMEBUFFER ) {
			if( (port_width != pstVDecPrivate->lPicWidthMax) || (port_height != pstVDecPrivate->lPicHeightMax)){
				ret |= PORT_RECONFIGURATION_NEEDED;
			}
		}
		else
#endif
		{
			if( (port_width != width) || (port_height != height) )
			{
				SET_STATE(pstVDecPrivate, STATE_RESOLUTION_CHANGING_WITH_REINIT);
				ret |= PORT_RECONFIGURATION_NEEDED;
			}
		}

		st_crop.nLeft    = 0;
		st_crop.nTop     = 0;
		st_crop.nWidth   = width;
		st_crop.nHeight  = height;
	}

	if( st_crop.nLeft   != pstVDecPrivate->stCropRect.nLeft ||
			st_crop.nTop    != pstVDecPrivate->stCropRect.nTop ||
			st_crop.nWidth  != pstVDecPrivate->stCropRect.nWidth ||
			st_crop.nHeight != pstVDecPrivate->stCropRect.nHeight)
	{
		pstVDecPrivate->stCropRect.nLeft    = st_crop.nLeft;
		pstVDecPrivate->stCropRect.nTop     = st_crop.nTop;
		pstVDecPrivate->stCropRect.nWidth   = st_crop.nWidth;
		pstVDecPrivate->stCropRect.nHeight  = st_crop.nHeight;
		//ret |= PORT_CROP_CHANGE_NEEDED;
	}

#if 0	//TAG:MOD - aspect ratio changing
	if( vdec_getAspectRatio(pstVDecPrivate->pVDecInstance,
							&pstVDecPrivate->stScaleFactor.xWidth,
							&pstVDecPrivate->stScaleFactor.xHeight,
							width, height,
							*pstVDecPrivate->pstCdmxInfo,
							pstVDecPrivate->ulContainerType,
							pstVDecPrivate->ulExtractorType & OMX_BUFFERFLAG_EXTRACTORTYPE_TCC) )
	{
		ret |= PORT_AR_CHANGE_NEEDED;
	}
#endif

	if( CHECK_STATE(pstVDecPrivate, STATE_RESOLUTION_CHANGING_WITH_REINIT) ) {
		/* resolution with re-init step7 - port reconfiguration */
#if RESOLUTION_CHANGE_WITHOUT_EVENT
		ret = 0;
#else
		ret |= PORT_RECONFIGURATION_NEEDED;
		ret |= PORT_CROP_CHANGE_NEEDED;
#endif
		/* resolution with re-init step8 - complete */
		INFO("Resolution Change Complete");
		CLEAR_STATE(pstVDecPrivate, STATE_RESOLUTION_CHANGING_WITH_REINIT);
		CLEAR_STATE(pstVDecPrivate, STATE_READY_TO_RESET);
		SET_STATE(pstVDecPrivate, STATE_WAIT_CHANGED_OUTPUT);
	}

#if defined (TCC_HEVC_INCLUDE) || defined (TCC_VPU_4K_D2_INCLUDE)
	if( pstVDecPrivate->stVDecInit.m_iBitstreamFormat == STD_HEVC ) {
		if(width > AVAILABLE_HEVC_MAX_WIDTH || ((width *height) > AVAILABLE_HEVC_MAX_REGION)) {
			ERROR("%ld x %ld ==> MAX-Resolution(%d x %d) over!!", width, height, AVAILABLE_HEVC_MAX_WIDTH, AVAILABLE_HEVC_MAX_HEIGHT);
			ret = MAX_RESOLUTION_EXCEEDED;
		}
	}
	else
#endif
#if defined (TCC_VPU_4K_D2_INCLUDE)
	if( pstVDecPrivate->stVDecInit.m_iBitstreamFormat == STD_VP9 ) {
		if(width > AVAILABLE_VP9_MAX_WIDTH || ((width *height) > AVAILABLE_VP9_MAX_REGION)) {
			ERROR("%ld x %ld ==> MAX-Resolution(%d x %d) over!!", width, height, AVAILABLE_VP9_MAX_WIDTH, AVAILABLE_VP9_MAX_HEIGHT);
			ret = MAX_RESOLUTION_EXCEEDED;
		}
	}
	else
#endif
	{
		if(width > AVAILABLE_MAX_WIDTH || ((width *height) > AVAILABLE_MAX_REGION)) {
			ERROR("%ld x %ld ==> MAX-Resolution(%d x %d) over!!", width, height, AVAILABLE_MAX_WIDTH, AVAILABLE_MAX_HEIGHT);
			ret = MAX_RESOLUTION_EXCEEDED;
		}
	}

	if(width < AVAILABLE_MIN_WIDTH || height < AVAILABLE_MIN_HEIGHT) {
		ERROR("%ld x %ld ==> MIN-Resolution(%d x %d) less!!", width, height, AVAILABLE_MIN_WIDTH, AVAILABLE_MIN_HEIGHT);
		ret = MAX_RESOLUTION_EXCEEDED;
	}

#if defined(JPEG_DECODE_FOR_MJPEG)
	if( pstVDecPrivate->stVDecInit.m_iBitstreamFormat == STD_MJPG )
	{
		if( pstVDecPrivate->stVDecInit.m_bCbCrInterleaveMode != 1 )
		{
			//!< MJPEG source chroma format(0 - 4:2:0, 1 - 4:2:2, 2 - 4:2:2 vertical, 3 - 4:4:4, 4 - 4:0:0 )

			if( CHECK_MODE(pstVDecPrivate, MODE_PHYSICAL_ADDRESS_OUTPUT) ) {
				colorformat = OMX_COLOR_FormatYUV420PlanarTc;
			} else {
				colorformat = OMX_COLOR_FormatYUV420Planar;
			}

			if( p_outport->sPortParam.format.video.eColorFormat != colorformat )
			{
				INFO("Change ColorFormat! (%ld --> %ld)", p_outport->sPortParam.format.video.eColorFormat, colorformat);

				p_outport->sVideoParam.eColorFormat = colorformat;
				p_outport->sPortParam.format.video.eColorFormat = colorformat;
				ret = PORT_RECONFIGURATION_NEEDED;
			}
		}
	}
#endif

	if( ret & PORT_RECONFIGURATION_NEEDED ) {
		INFO("Port reconfiguration is needed! (%lu x %lu --> %lu x %lu)", port_width, port_height, width, height);
		UpdateFrameSize(pstVDecPrivate);
	}

	if( ret & PORT_RECONFIGURATION_NEEDED ) {
		vpu_update_sizeinfo(pstVDecPrivate->stVDecInit.m_iBitstreamFormat,
							pstVDecPrivate->stVDecUserInfo.bitrate_mbps,
							pstVDecPrivate->stVDecUserInfo.frame_rate,
							p_init_info->m_iPicWidth,
							p_init_info->m_iPicHeight,
							pstVDecPrivate->pVDecInstance);
	}

#if USE_MAX_VPU_CLOCK
	vpu_update_sizeinfo(STD_AVC,
						1000,
						60,
						pstVDecPrivate->lPicWidthMax, pstVDecPrivate->lPicHeightMax,
						pstVDecPrivate->pVDecInstance);
#endif

	return ret;
}

static OMX_BOOL
IsCropChange(
	vdec_private_t      *pstVDecPrivate
	)
{
#ifdef SET_FRAMEBUFFER_INTO_MAX
	OMX_S32 dec_width = pstVDecPrivate->stVDecOutput.m_DecOutInfo.m_iWidth;
	OMX_S32 dec_height = pstVDecPrivate->stVDecOutput.m_DecOutInfo.m_iHeight;
	pic_crop_t avc_crop = pstVDecPrivate->stVDecOutput.m_DecOutInfo.m_CropInfo;

	if ((dec_width == 0) || (dec_height == 0)){	//Buffer full case
		return OMX_FALSE;
	}

	dec_width  -= avc_crop.m_iCropLeft;
	dec_width  -= avc_crop.m_iCropRight;
	dec_height -= avc_crop.m_iCropBottom;
	dec_height -= avc_crop.m_iCropTop;

	if( avc_crop.m_iCropLeft   != pstVDecPrivate->stCropRect.nLeft ||
		avc_crop.m_iCropTop    != pstVDecPrivate->stCropRect.nTop ||
		dec_width              != pstVDecPrivate->stCropRect.nWidth ||
		dec_height             != pstVDecPrivate->stCropRect.nHeight)
	{
		INFO("Resolution changed (%ld x %ld --> %ld x %ld)",
				pstVDecPrivate->stCropRect.nWidth, pstVDecPrivate->stCropRect.nHeight,
				dec_width, dec_height);

		pstVDecPrivate->stCropRect.nLeft    = avc_crop.m_iCropLeft;
		pstVDecPrivate->stCropRect.nTop     = avc_crop.m_iCropTop;
		pstVDecPrivate->stCropRect.nWidth   = dec_width;
		pstVDecPrivate->stCropRect.nHeight  = dec_height;

		return OMX_TRUE;
	}
#endif

	return OMX_FALSE;
}

static
OMX_BOOL
IsResolutionChanged(
	vdec_private_t      *pstVDecPrivate
	)
{
	OMX_S32 dec_width = pstVDecPrivate->stVDecOutput.m_DecOutInfo.m_iWidth;
	OMX_S32 dec_height = pstVDecPrivate->stVDecOutput.m_DecOutInfo.m_iHeight;

	if( (dec_width > 0) && (dec_height > 0) )
	{
		OMX_S32 old_width = pstVDecPrivate->stCropRect.nWidth;
		OMX_S32 old_height = pstVDecPrivate->stCropRect.nHeight;

#if AVC_CROP_ENABLE && AVC_CROP_APPLIED_TO_PORT
		if(    pstVDecPrivate->stVDecInit.m_iBitstreamFormat == STD_AVC
			|| pstVDecPrivate->stVDecInit.m_iBitstreamFormat == STD_MVC
#if defined (TCC_HEVC_INCLUDE) || defined (TCC_VPU_4K_D2_INCLUDE)
			|| pstVDecPrivate->stVDecInit.m_iBitstreamFormat == STD_HEVC
#endif
#if defined (TCC_VPU_4K_D2_INCLUDE)
			|| pstVDecPrivate->stVDecInit.m_iBitstreamFormat == STD_VP9
#endif
		  )
		{
			pic_crop_t avc_crop = pstVDecPrivate->stVDecOutput.m_DecOutInfo.m_CropInfo;
			dec_width  -= avc_crop.m_iCropLeft;
			dec_width  -= avc_crop.m_iCropRight;
			dec_height -= avc_crop.m_iCropBottom;
			dec_height -= avc_crop.m_iCropTop;
		}
#endif

		if( (old_width != dec_width) || (old_height != dec_height) ) {
			INFO("Resolution changed (%ld x %ld --> %ld x %ld)",
					old_width, old_height,
					dec_width, dec_height);
			pstVDecPrivate->stCropRect.nWidth   = dec_width;
			pstVDecPrivate->stCropRect.nHeight  = dec_height;

			return OMX_TRUE;
		}
	}

	return OMX_FALSE;
}


#define COPY_FROM_DECODED_BUFFER	(1)
#define COPY_FROM_DISPLAY_BUFFER	(2)
#define COPY_FROM_BLACK_FRAME			(3)

static
OMX_U8*
CopyOutputFrame(
	vdec_private_t   *pstVDecPrivate,
	OMX_U8           *pDestAddress,
	OMX_U32          *pulCopyedBytes,
	OMX_S32           lSrcIndex
	)
{
	OMX_S32 frame_size = 0;
	OMX_S32	dst_width, dst_height;
	OMX_S32	y_stride, cbcr_stride;
	OMX_S32 h_off = 0, v_off = 0;
	OMX_U8 *p_src_y = NULL;
	OMX_U8 *p_src_cb = NULL;
	OMX_U8 *p_src_cr = NULL;
	OMX_S32 i;
#if defined(TC_SECURE_MEMORY_COPY)
	OMX_PTR* p_thumb_buff = NULL;
	OMX_S32 thumb_buff_size = NULL;
#endif
	if(    (pstVDecPrivate->stVDecInit.m_iBitstreamFormat == STD_AVC)
	    || (pstVDecPrivate->stVDecInit.m_iBitstreamFormat == STD_MVC)
#if defined (TCC_VPU_4K_D2_INCLUDE)
	    || (pstVDecPrivate->stVDecInit.m_iBitstreamFormat == STD_VP9)
#endif
#if defined (TCC_HEVC_INCLUDE) || defined (TCC_VPU_4K_D2_INCLUDE)
		|| (pstVDecPrivate->stVDecInit.m_iBitstreamFormat == STD_HEVC)
#endif
	  )
	{
		vdec_initial_info_t *p_init_info = pstVDecPrivate->stVDecOutput.m_pInitialInfo;
		pic_crop_t avc_crop = p_init_info->m_iPicCrop;

		y_stride  = p_init_info->m_iPicWidth;
		dst_width = p_init_info->m_iPicWidth;
		dst_width -= avc_crop.m_iCropLeft;
		dst_width -= avc_crop.m_iCropRight;

		dst_height = p_init_info->m_iPicHeight;
		dst_height -= avc_crop.m_iCropBottom;
		dst_height -= avc_crop.m_iCropTop;

		h_off = avc_crop.m_iCropLeft;
		h_off -= h_off & 1;
		v_off = avc_crop.m_iCropTop;
		v_off -= v_off & 1;
	}
	else
	{
		vdec_initial_info_t *p_init_info = pstVDecPrivate->stVDecOutput.m_pInitialInfo;

		y_stride   = p_init_info->m_iPicWidth;
		dst_width  = p_init_info->m_iPicWidth;
		dst_height = p_init_info->m_iPicHeight;
	}

	dst_width  += dst_width & 1;
	dst_height += dst_height & 1;

	// src size setup
#if defined(TCC_HEVC_INCLUDE) || defined (TCC_VPU_4K_D2_INCLUDE)
	if (pstVDecPrivate->stVDecInit.m_iBitstreamFormat == STD_HEVC) {
		y_stride    = ((y_stride + 31) >> 5) << 5;
		cbcr_stride = y_stride >> 1;
	}
	else
#endif
#if defined (TCC_VPU_4K_D2_INCLUDE)
	if (pstVDecPrivate->stVDecInit.m_iBitstreamFormat == STD_VP9) {
		y_stride    = ((y_stride + 31) >> 5) << 5;
		cbcr_stride = y_stride >> 1;
	}
	else
#endif
	{
		y_stride    = ((y_stride + 15) >> 4) << 4;
		cbcr_stride = y_stride >> 1;
	}

	if( lSrcIndex != COPY_FROM_BLACK_FRAME )
	{
#if defined(TC_SECURE_MEMORY_COPY)
		if( CHECK_STATE(pstVDecPrivate, STATE_VDEC_BUFFER_PROTECTION) )
		{
			OMX_S32 inst_idx = vdec_get_instance_index(pstVDecPrivate->pVDecInstance);
			int ret = 0;
			VideoDec_Pmap_T pmap_thumb;

			if( inst_idx <= 1 ) {	// instance idx 0 or 1
				st_func_get_pmap_info("video_thumb", &pmap_thumb);
				if( pmap_thumb.iSize > 0 ) {
					if( ( p_thumb_buff = (unsigned long*)mmap(0, pmap_thumb.iSize, PROT_READ|PROT_WRITE, MAP_SHARED, pstVDecPrivate->hTMemDevice, pmap_thumb.iAddress) ) == MAP_FAILED ) {
						ERROR("Secured thumbnail buffer mmap failed. %s", TMEM_DEVICE);
						p_thumb_buff = NULL;
						ret = -1;
					}
					thumb_buff_size = pmap_thumb.iSize;
				}
			}

			if(pmap_thumb.iSize != 0 && p_thumb_buff != NULL)
			{
				#define MAX_FRAME_SIZE 	(3*1024*1024) // MAX: Full HD
				OMX_U32 base_offset = inst_idx * MAX_FRAME_SIZE;

				if( lSrcIndex == COPY_FROM_DECODED_BUFFER ) {
					p_src_y  = pstVDecPrivate->stVDecOutput.m_pCurrOut[PA][0];
					p_src_cb = pstVDecPrivate->stVDecOutput.m_pCurrOut[PA][1];
					p_src_cr = pstVDecPrivate->stVDecOutput.m_pCurrOut[PA][2];
				}
				else if( lSrcIndex == COPY_FROM_DISPLAY_BUFFER ) {
					p_src_y  = pstVDecPrivate->stVDecOutput.m_pDispOut[PA][0];
					p_src_cb = pstVDecPrivate->stVDecOutput.m_pDispOut[PA][1];
					p_src_cr = pstVDecPrivate->stVDecOutput.m_pDispOut[PA][2];
				}

				OMX_U8 *pDestY = pmap_thumb.iAddress + base_offset;
				OMX_U8 *pDestU = pDestY + (p_src_cb - p_src_y);
				OMX_U8 *pDestV = pDestU + (p_src_cr - p_src_cb);

	/*
				GBUG_MSG("Thumb copy(%d) :: 0x%x-0x%x-0x%x -> 0x%x-0x%x-0x%x", pstVDecPrivate->pVideoDecodInstance.gsVDecInit.m_bCbCrInterleaveMode, pSrcY, pSrcCb, pSrcCr, pDestY, pDestU, pDestV);
				ret = _gralloc_copy_data( openmaxStandComp, OMX_TRUE,
														pstVDecPrivate->pVideoDecodInstance.gsVDecOutput.m_pInitialInfo->m_iPicWidth,
														pstVDecPrivate->pVideoDecodInstance.gsVDecOutput.m_pInitialInfo->m_iPicHeight,
														p_src_y, p_src_cb, p_src_cr, pstVDecPrivate->pVideoDecodInstance.gsVDecInit.m_bCbCrInterleaveMode,
														pDestY, pDestU, pDestV, SEND_CMD | WAIT_RESPOND, OMX_TRUE);
	*/
				p_src_y = (OMX_U8*)p_thumb_buff + base_offset;
				p_src_cb = p_src_y + (pDestU - pDestY);
				p_src_cr = p_src_cb + (pDestV - pDestU);

			}

			if( ret < 0 ){
				lSrcIndex = COPY_FROM_BLACK_FRAME;
			}
		}
		else
#endif
		{
			if( lSrcIndex == COPY_FROM_DECODED_BUFFER ) {
				p_src_y  = pstVDecPrivate->stVDecOutput.m_pCurrOut[VA][0];
				p_src_cb = pstVDecPrivate->stVDecOutput.m_pCurrOut[VA][1];
				p_src_cr = pstVDecPrivate->stVDecOutput.m_pCurrOut[VA][2];
			}
			else if( lSrcIndex == COPY_FROM_DISPLAY_BUFFER ) {
				p_src_y  = pstVDecPrivate->stVDecOutput.m_pDispOut[VA][0];
				p_src_cb = pstVDecPrivate->stVDecOutput.m_pDispOut[VA][1];
				p_src_cr = pstVDecPrivate->stVDecOutput.m_pDispOut[VA][2];
			}
		}
	}

	// create buffer
	if( pDestAddress == NULL )
	{
		if( pstVDecPrivate->pbyThumbnailBuff == NULL )
		{
			frame_size = (dst_width*dst_height*3) >> 1;

			// thumbnail buffer allocation
			pDestAddress = TCC_malloc(frame_size);
			if( pDestAddress == 0 )
			{
				*pulCopyedBytes = 0;
				ERROR("CreateThumbFrame() - out of memory (%ld)", frame_size);
				return 0;
			}
			*pulCopyedBytes = frame_size;
			pstVDecPrivate->pbyThumbnailBuff = pDestAddress;
		}

		pDestAddress = pstVDecPrivate->pbyThumbnailBuff;
	}

	if( pstVDecPrivate->stVDecInit.m_bCbCrInterleaveMode )
	{
		// dst size setup
		OMX_S32 y_size     = dst_width * dst_height;
		OMX_S32 cbcr_size  = y_size >> 1;

		// dst pointer setup
		OMX_U8 *p_dst_y    = pDestAddress;
		OMX_U8 *p_dst_cbcr = p_dst_y + y_size;

		if( lSrcIndex == COPY_FROM_BLACK_FRAME )
		{
			memset(p_dst_y, 0x10, y_size);
			memset(p_dst_cbcr, 0x80, cbcr_size);
		}
		else
		{
			OMX_U8 *p_src_cbcr = p_src_cb;
			OMX_S32 dst_height_cbcr = dst_height >> 1;

			frame_size = y_size + cbcr_size;

			// cropping
			p_src_y += h_off + y_stride * v_off;
			p_src_cbcr += ((h_off >> 1) << 1) + (y_stride * (v_off >> 1));

			// luminance
			for(i = 0; i < dst_height; i++)
			{
				(void)memcpy(p_dst_y, p_src_y, dst_width);
				p_dst_y += dst_width;
				p_src_y += y_stride;
			}

			// chrominance
			for(i = 0; i < dst_height_cbcr; i++)
			{
				(void)memcpy(p_dst_cbcr, p_src_cbcr, dst_width);
				p_dst_cbcr += dst_width;
				p_src_cbcr += y_stride;
			}
		}
	}
	else
	{
		OMX_S32 dst_width_cbcr = dst_width >> 1;

		// dst size setup
		OMX_S32 y_size     = dst_width * dst_height;
		OMX_S32 cb_size    = y_size >> 2;
		OMX_S32 cr_size    = cb_size;

		// dst pointer setup
		OMX_U8 *p_dst_y    = pDestAddress;
		OMX_U8 *p_dst_cb   = p_dst_y + y_size;
		OMX_U8 *p_dst_cr   = p_dst_cb + cb_size;

		frame_size = y_size + cb_size + cr_size;

		if( lSrcIndex == COPY_FROM_BLACK_FRAME )
		{
			memset(p_dst_y, 0x10, y_size);
			memset(p_dst_cb, 0x80, cb_size);
			memset(p_dst_cr, 0x80, cr_size);
		}
		else
		{
			//!< MJPEG source chroma format(0 - 4:2:0, 1 - 4:2:2, 2 - 4:2:2 vertical, 3 - 4:4:4, 4 - 4:0:0 )
			if( pstVDecPrivate->stVDecInit.m_iBitstreamFormat == STD_MJPG &&
				pstVDecPrivate->stVDecOutput.m_pInitialInfo->m_iMjpg_sourceFormat == 1 )
			{
				// cropping
				p_src_y += h_off + y_stride * v_off;
				h_off >>= 1;
				p_src_cb += h_off + (cbcr_stride * v_off);
				p_src_cr += h_off + (cbcr_stride * v_off);

				for(i = 0; i < dst_height; i++)
				{
					// luminance
					(void)memcpy(p_dst_y, p_src_y, dst_width);
					p_dst_y += dst_width;
					p_src_y += y_stride;

					// chrominance (4:2:2 to 4:2:0)
					if( i & 1 ) {
						(void)memcpy(p_dst_cr, p_src_cr, dst_width_cbcr);
						p_dst_cr += dst_width_cbcr;
					}
					else {
						(void)memcpy(p_dst_cb, p_src_cb, dst_width_cbcr);
						p_dst_cb += dst_width_cbcr;
					}
					p_src_cb += cbcr_stride;
					p_src_cr += cbcr_stride;
				}
			}
			else
			{
				OMX_S32 dst_height_cbcr = dst_height >> 1;

				// cropping
				p_src_y += h_off + (y_stride * v_off);
				h_off >>= 1;
				v_off >>= 1;
				p_src_cb += h_off + (cbcr_stride * v_off);
				p_src_cr += h_off + (cbcr_stride * v_off);

				// luminance
				for(i = 0; i < dst_height; i++)
				{
					// luminance
					(void)memcpy(p_dst_y, p_src_y, dst_width);
					p_dst_y += dst_width;
					p_src_y += y_stride;
				}

				// chrominance
				for(i = 0; i < dst_height_cbcr; i++)
				{
					(void)memcpy(p_dst_cb, p_src_cb, dst_width_cbcr);
					p_dst_cb += dst_width_cbcr;
					p_src_cb += cbcr_stride;
					(void)memcpy(p_dst_cr, p_src_cr, dst_width_cbcr);
					p_dst_cr += dst_width_cbcr;
					p_src_cr += cbcr_stride;
				}
			}
		}
	}

#if 0
	{
		FILE *fp;
		OMX_S8 path[256];
		ERROR("Thumbnail frame: %ld x %ld", dst_width, dst_height);
		sprintf(path, "/sdcard/tflash/thumb_%ldx%ld.yuv", dst_width, dst_height);
		if( fp = fopen(path, "wb") ) {
			fwrite( pThumbBuff, 1, frame_size, fp);
			fclose(fp);
		}
	}
#endif

#if defined(TC_SECURE_MEMORY_COPY)
	if( p_thumb_buff ){
		munmap((unsigned long*)p_thumb_buff, thumb_buff_size);
	}
#endif
	*pulCopyedBytes = frame_size;

	return pDestAddress;
}


static
OMX_S32
GetOutputBufferInfo(
	vdec_private_t         *pstVDecPrivate,
	OMX_S32                 lBufferIndex,
	output_info_t          *pstOutputInfo
	)
{
	disp_info_t *p_info;
	OMX_TICKS timestamp;
	OMX_S32 ret;
	char char_m_iDecodedIdx,char_m_iDispOutIdx;

	if( ShowOutputDispInfo(&pstVDecPrivate->stDispInfoMgr, lBufferIndex, &p_info, &timestamp) == OMX_FALSE )
	{
		/* resolution with re-init step4 - complete flushing */
		if( CHECK_STATE(pstVDecPrivate, STATE_RESOLUTION_CHANGING_WITH_REINIT) ) {
			SET_STATE(pstVDecPrivate, STATE_READY_TO_RESET);
			return ERROR_INVALID_BUFFER_STATE;
		}

		if( ShowNonQueuedDispInfo(&pstVDecPrivate->stDispInfoMgr, lBufferIndex, &p_info, &timestamp) == OMX_FALSE )
		{
			return -1;
		}
	}

	if( pstOutputInfo ) {
		pstOutputInfo->lFrameType      = p_info->lFrameType;
		pstOutputInfo->lPicStructure   = p_info->lPicStructure;
		pstOutputInfo->llTimestamp     = timestamp;
		pstOutputInfo->lCropLeft       = p_info->lCropLeft;
		pstOutputInfo->lCropTop        = p_info->lCropTop;
		pstOutputInfo->lCropWidth      = p_info->lCropWidth;
		pstOutputInfo->lCropHeight     = p_info->lCropHeight;
		pstOutputInfo->ulFlags         = p_info->ulFlags;
		pstOutputInfo->lErrorMB        = p_info->lErrorMB;
		pstOutputInfo->lUserDataLength = p_info->lUserDataLength;
		pstOutputInfo->pbyUserDataBuff = p_info->pbyUserDataBuff;
	}
	if ((pstVDecPrivate->stVDecOutput.m_DecOutInfo.m_iDispOutIdx == 3) || (p_info->lFrameType == 0) || (p_info->lFrameType == 5)){
		OMX_LOGD("[OUT][TYPE: %s ][PTS: %8d (diff: %4d)] [State: %2d/%2d][BuffIdx: %2d/%2d]\n"
			, GetFrameTypeString( pstVDecPrivate->stVDecInit.m_iBitstreamFormat
								  , p_info->lFrameType
								  , p_info->lPicStructure)
			, (OMX_S32)(timestamp/1000)
			, (OMX_S32)((timestamp - pstVDecPrivate->stDispInfoMgr.llLastOutTimestamp)/1000)
			, pstVDecPrivate->stVDecOutput.m_DecOutInfo.m_iDecodingStatus
			, pstVDecPrivate->stVDecOutput.m_DecOutInfo.m_iOutputStatus
			, pstVDecPrivate->stVDecOutput.m_DecOutInfo.m_iDecodedIdx
			, pstVDecPrivate->stVDecOutput.m_DecOutInfo.m_iDispOutIdx
			);
	}

#if defined(TCC_VPU_C7_INCLUDE)
	char_m_iDecodedIdx = (pstVDecPrivate->stVDecOutput.m_DecOutInfo.m_iDecodedIdx < 0) ? '-' : (pstVDecPrivate->stVDecOutput.m_DecOutInfo.m_MvcPicInfo.m_iViewIdxDecoded ? 'D' : 'B');
	char_m_iDispOutIdx = (pstVDecPrivate->stVDecOutput.m_DecOutInfo.m_iDispOutIdx < 0) ? '-' : (pstVDecPrivate->stVDecOutput.m_DecOutInfo.m_MvcPicInfo.m_iViewIdxDisplay ? 'D' : 'B');
#else
	char_m_iDecodedIdx = '-';
	char_m_iDispOutIdx = '-';
#endif
	LOG_OUT("[TYPE: %s (%3ld/%ld)][PTS: %8ld (diff: %4ld)] [State: %2d/%2d][BuffIdx: %2d(%c)/%2d(%c)] [FieldSeq: %ld][Interlaced: %d (%d)][TR: %8ld] [MBerr: %ld] "
			, GetFrameTypeString( pstVDecPrivate->stVDecInit.m_iBitstreamFormat
			, p_info->lFrameType
			, p_info->lPicStructure)
			, p_info->lFrameType
			, p_info->lPicStructure
			, (OMX_S32)(timestamp/1000)
			, (OMX_S32)((timestamp - pstVDecPrivate->stDispInfoMgr.llLastOutTimestamp)/1000)
			, pstVDecPrivate->stVDecOutput.m_DecOutInfo.m_iDecodingStatus
			, pstVDecPrivate->stVDecOutput.m_DecOutInfo.m_iOutputStatus
			, pstVDecPrivate->stVDecOutput.m_DecOutInfo.m_iDecodedIdx
			, char_m_iDecodedIdx
			, pstVDecPrivate->stVDecOutput.m_DecOutInfo.m_iDispOutIdx
			, char_m_iDispOutIdx
			, p_info->lM2vFieldSequence
			, (p_info->ulFlags & DISPINFO_FLAG_INTERLACED) ? 1 : 0
			, (p_info->ulFlags & DISPINFO_FLAG_ODDFIELD_FIRST) ? 0 : 1
			, p_info->lExtTimestamp
			, p_info->lErrorMB
			);
	if( p_info->bIsValid == OMX_FALSE ){
		(void)ClearNonQueuedDispInfo(&pstVDecPrivate->stDispInfoMgr, lBufferIndex);
	}	else {
		(void)ClearDispInfo(&pstVDecPrivate->stDispInfoMgr, lBufferIndex);
	}

	return 0;
}

static
OMX_S32
ClearAllDisplayBuffers(
	vdec_private_t         *pstVDecPrivate
	)
{
	OMX_S32 index;
	OMX_S32 disp_idx, dv_el_disp_idx;
	OMX_S32 ret = 0;

	while( GetDispIdx(&pstVDecPrivate->stDispIdxQueue, &disp_idx, &dv_el_disp_idx) >= 0 )
	{
		LOG_BUFCLR("[LINE: %4d] [DISP_IDX: %2ld] [USED_COUNT: %2ld]", __LINE__, disp_idx, pstVDecPrivate->lUsedBuffCount);
		pstVDecPrivate->lUsedBuffCount--;
		if( (ret = VDEC_FUNC(pstVDecPrivate, VDEC_BUF_FLAG_CLEAR, NULL, &disp_idx, NULL)) < 0 ) {
			LOGE( "[VDEC_ERROR] [OP: VDEC_BUF_FLAG_CLEAR] [DISP_IDX: %ld / RET_CODE: %ld]", disp_idx, ret);
			break;//return ret;
		}
	}

	if (ret >= 0) {
		pstVDecPrivate->lUsedBuffCount = 0;
	}

	return 0;
}

static OMX_S32 doClearDisplayedBuffer(vdec_private_t        *pstVDecPrivate)
{
	OMX_S32 ret = -1;
	OMX_S32 disp_idx, dv_el_disp_idx;
	if(pstVDecPrivate)
	{
		if( ShowDispIdx(&pstVDecPrivate->stDispIdxQueue, &disp_idx, &dv_el_disp_idx) >= 0 )
		{
			OMX_S32 buff_idx;
			buff_idx = disp_idx;

			if(buff_idx >= 0)
			{
				OMX_S32 ret;

				LOG_BUFCLR("[LINE: %4d] [DISP_IDX: %2ld] [USED_COUNT: %2ld]", __LINE__, disp_idx, pstVDecPrivate->lUsedBuffCount);

				pstVDecPrivate->lUsedBuffCount--;

				if( (ret = VDEC_FUNC(pstVDecPrivate, VDEC_BUF_FLAG_CLEAR, NULL, &disp_idx, NULL)) < 0 ) {
					LOGE( "[VDEC_ERROR] [OP: VDEC_BUF_FLAG_CLEAR] [DISP_IDX: %ld / RET_CODE: %ld]", disp_idx, ret);
					return ret;
				}

				if( buff_idx != disp_idx ) {
					LOGE( "[VDEC_ERROR] [OP: VDEC_BUF_FLAG_CLEAR] [DISP_IDX: %ld / RET_CODE: %ld] - [NOT MATCHED: %ld]", disp_idx, ret, buff_idx);
				}
				ClearFirstDispIdx(&pstVDecPrivate->stDispIdxQueue);
				ret = 0;
			}
		}
	}
	return ret;
}

static
OMX_S32
ClearDisplayedBuffer(
	vdec_private_t        *pstVDecPrivate,
	OMX_BOOL               bForced
	)
{
	/* Clear displayed buffer */
	// additional buffer count means real buffering count between omx and renderer.
	if( bForced || (GetDispIdxCount(&pstVDecPrivate->stDispIdxQueue) >= pstVDecPrivate->lMaxFifoCount) )
	{
		OMX_S32 disp_idx;
		OMX_S32 dv_el_disp_idx;

		if( ShowDispIdx(&pstVDecPrivate->stDispIdxQueue, &disp_idx, &dv_el_disp_idx) >= 0 )
		{
			OMX_S32 buff_idx;
			buff_idx = disp_idx;

			if(buff_idx >= 0)
			{
				OMX_S32 ret;
				LOG_BUFCLR("[LINE: %4d] [DISP_IDX: %2ld] [USED_COUNT: %2ld]", __LINE__, disp_idx, pstVDecPrivate->lUsedBuffCount);
				pstVDecPrivate->lUsedBuffCount--;

				ret = VDEC_FUNC(pstVDecPrivate, VDEC_BUF_FLAG_CLEAR, NULL, &disp_idx, NULL);
				if( ret < 0 ) {
					LOGE( "[VDEC_ERROR] [OP: VDEC_BUF_FLAG_CLEAR] [DISP_IDX: %ld / RET_CODE: %ld]", disp_idx, ret);
					return ret;
				}

				if( buff_idx != disp_idx ){
					LOGE( "[VDEC_ERROR] [OP: VDEC_BUF_FLAG_CLEAR] [DISP_IDX: %ld / RET_CODE: %ld] - [NOT MATCHED: %ld]", disp_idx, ret, buff_idx);
				}

				ClearFirstDispIdx(&pstVDecPrivate->stDispIdxQueue);
			}
		}
	}

	return TRUE;
}

#define NEED_MORE_DATA	(1)
#define FEED_COMPLETE		(0)
#define FEED_FAILED			(-1)

#if DECODER_RINGBUFFER_MODE

static OMX_S32 ScanFillerSpace_internal(
			OMX_U8      *pRingBase,
			OMX_U8      *pRingEnd,
			OMX_U8      *pScanStart,
			OMX_U8      *pScanEnd,
			OMX_BOOL     bIsAVC,
			OMX_U8     **ppSpaceEnd
)
{
#define PRESCAN_MAX		(32)

	OMX_U8  *p_curr_rp = pScanStart;
	OMX_U32 *p_rp32 = (OMX_U32 *)((((uintptr_t)pScanStart + 3) >> 2) << 2);
	OMX_U8  *p_filler_start = 0;
	OMX_U8  *p_filler_end = 0;
	OMX_S32 filler_size = 0;
	OMX_U32 syncword = 0xFFFFFFFF;
	OMX_U32 filler = 0;
	
	if (pScanStart == pScanEnd){
		return 0;
	}
	
	if (pScanStart < pScanEnd)  {
		OMX_U8 *p_temp_end = pScanStart + PRESCAN_MAX;
		
		if( p_temp_end > pScanEnd ){
			p_temp_end = pScanEnd;
		}
		
		// search zero start to PRESCAN_MAX bytes
		while (p_curr_rp < p_temp_end) {
			if (*p_curr_rp){
				p_filler_start = NULL;
			} else if (p_filler_start == NULL){
				p_filler_start = p_curr_rp;
			}
			
			if (bIsAVC) {
				syncword <<= 8;
				syncword |= *p_curr_rp;
				if( (syncword & 0x1F) == 0x0C ) {
					filler = 0xFF;
					p_filler_start = p_curr_rp+1;
					break;
				}
			}
			p_curr_rp++;
		}
		
		if (p_filler_start) {
			p_curr_rp = p_filler_start;
			p_rp32 = (OMX_U32 *)((((uintptr_t)p_curr_rp + 3) >> 2) << 2);
			
			while ((uintptr_t)p_curr_rp < (uintptr_t)p_rp32 && p_curr_rp < pScanEnd) {
				if( *p_curr_rp != filler ) {
					p_filler_end = p_curr_rp;
					break;
				}
				p_curr_rp++;
			}
		
			if (p_filler_end == NULL && p_curr_rp < pScanEnd) {
				if (filler == 0xFF){
					filler = 0xFFFFFFFF;
				}
				
				while ((ptrdiff_t)p_rp32 < (ptrdiff_t)pScanEnd - (sizeof(long) - 1)) {
					if (*p_rp32 != filler){
						break;
					}
					p_rp32++;
				}
				
				filler &= 0xFF;
				
				p_curr_rp = (OMX_U8*)p_rp32;
				while (p_curr_rp < pScanEnd) {
					if (*p_curr_rp != filler) {
						p_filler_end = p_curr_rp;
						break;
					}
					p_curr_rp++;
				}
			}
			
			if (p_filler_end == NULL){
				p_filler_end = pScanEnd;
			}
			
			filler_size = (OMX_S32)(p_filler_end - p_filler_start);			
			ASSERT( pScanStart <= p_filler_end && p_filler_end <= pScanEnd );
		}
	} else {
		OMX_S32 count;
		
		count  = (OMX_S32)(pRingEnd - pScanStart);
		count += (OMX_S32)(pScanEnd - pRingBase);
		
		if (count > PRESCAN_MAX){
			count = PRESCAN_MAX;
		}
		
		while (count--) {
			if (*p_curr_rp){
				p_filler_start = NULL;
			} else if( p_filler_start == NULL ){
				p_filler_start = p_curr_rp;
			}
			
			if (bIsAVC) {
				syncword <<= 8;
				syncword |= *p_curr_rp;
				if( (syncword & 0x1F) == 0x0C ) {
					filler = 0xFF;
					p_filler_start = p_curr_rp+1;
					if( p_filler_start >= pRingEnd ){
						p_filler_start = pRingBase;
					}
					break;
				}
			}
			
			if (++p_curr_rp >= pRingEnd){
				p_curr_rp = pRingBase;
			}
		}
		
		if (p_filler_start) {
			OMX_U8 *p_scan_end = (OMX_U8 *)((((uintptr_t)p_filler_start + 3) >> 2) << 2);
			
			p_rp32 = (OMX_U32 *)p_scan_end;
			p_curr_rp = p_filler_start;
			
			if (pScanEnd < p_scan_end && p_scan_end < pScanStart){
				p_scan_end = pScanEnd;
			}
			
			while (p_curr_rp < p_scan_end) {
				if (*p_curr_rp != filler) {
					p_filler_end = p_curr_rp;
					break;
				}
				p_curr_rp++;
			}
			
			if (p_filler_end == NULL && p_curr_rp != pScanEnd) {
				if ((uintptr_t)pScanStart < (uintptr_t)p_curr_rp) {
					p_scan_end = pRingEnd;
					
					if( filler == 0xFF ){
						filler = 0xFFFFFFFF;
					}
					
					// rear
					while ((ptrdiff_t)p_rp32 < (ptrdiff_t)p_scan_end - (sizeof(OMX_U32) - 1)) {
						if (*p_rp32 != filler){
							break;
						}
						p_rp32++;
					}
					
					// front
					if ((ptrdiff_t)p_rp32 >= (ptrdiff_t)p_scan_end) {
						p_rp32 = (OMX_U32 *)((uintptr_t)pRingBase);
						p_scan_end = (OMX_U8 *)(((uintptr_t)pScanEnd >> 2) << 2);
						
						while ((ptrdiff_t)p_rp32 < (ptrdiff_t)p_scan_end - (sizeof(OMX_U32) - 1)) {
							if (*p_rp32 != filler){
								break;
							}
							p_rp32++;
						}
						
						p_scan_end = pScanEnd;
					}
					
					filler &= 0xFF;
					
					p_curr_rp = (OMX_U8 *)((uintptr_t)p_rp32);
					while (p_curr_rp < p_scan_end) {
						if (*p_curr_rp != filler) {
							p_filler_end = p_curr_rp;
							break;
						}
						p_curr_rp++;
					}
				} else {
					// front
					p_rp32 = (OMX_U32 *)p_scan_end;
					p_scan_end = (OMX_U8 *)(((uintptr_t)pScanEnd >> 2) << 2);
					
					if (filler == 0xFF){
						filler = 0xFFFFFFFF;
					}
					
					while ((ptrdiff_t)p_rp32 < (ptrdiff_t)p_scan_end - (sizeof(OMX_U32) - 1)) {
						if (*p_rp32 != filler){
							break;
						}
						p_rp32++;
					}
					
					filler &= 0xFF;
					
					p_curr_rp = (OMX_U8 *)p_rp32;
					while (p_curr_rp < pScanEnd) {
						if (*p_curr_rp != filler) {
							p_filler_end = p_curr_rp;
							break;
						}
						p_curr_rp++;
					}
				}
			}
			
			if (p_filler_end == NULL){
				p_filler_end = pScanEnd;
			}
			
			if (p_filler_start < p_filler_end) {
				filler_size  = (OMX_S32)(p_filler_end - p_filler_start);
			} else {
				filler_size  = (OMX_S32)(pRingEnd - p_filler_start);
				filler_size += (OMX_S32)(p_filler_end - pRingBase);
			}
			
			ASSERT(p_filler_end <= pScanEnd || pScanStart <= p_filler_end);
		}
	}
	
	if (filler_size && ppSpaceEnd){
		*ppSpaceEnd = p_filler_end;
	}
	
	return filler_size;
}

static OMX_S32 ScanFillerSpace(
	vdec_private_t     *pstVDecPrivate,
	OMX_U8             *pScanStart,
	OMX_U8             *pScanEnd,
	OMX_U8            **ppSpaceEnd
	)
{
	OMX_S32 size = 0;
#if defined(TCC_HEVC_INCLUDE) || defined (TCC_VPU_4K_D2_INCLUDE)
	/* it doesn't support HEVC properly */
	if(pstVDecPrivate->stVDecInit.m_iBitstreamFormat == STD_HEVC
		#if defined (TCC_VPU_4K_D2_INCLUDE)
		|| pstVDecPrivate->stVDecInit.m_iBitstreamFormat == STD_VP9
		#endif
		)
	{
		return size;
	}
#endif

#if defined(TC_SECURE_MEMORY_COPY)
	if( CHECK_STATE(pstVDecPrivate, STATE_VDEC_BUFFER_PROTECTION) )
	{
		ringbuff_state_t *p_rb_state = &pstVDecPrivate->stRingBuffState;
		OMX_S32 size;
		OMX_S32 end_pos;

		size = TC_SecureScanFilterSpace((unsigned int)(p_rb_state->pRingBuffBase[PA]),
									  (int)(p_rb_state->lRingBuffSize),
									  (int)(pScanStart-p_rb_state->pRingBuffBase[VA]),
									  (int)(pScanEnd-p_rb_state->pRingBuffBase[VA]),
									  (int)(pstVDecPrivate->stVDecInit.m_iBitstreamFormat == STD_AVC),
									  (int *)&end_pos);

		if( size > 0 ){
			*ppSpaceEnd = p_rb_state->pRingBuffBase[VA] + end_pos;
		}
	}
	else
#endif
	{
		size = ScanFillerSpace_internal(pstVDecPrivate->stRingBuffState.pRingBuffBase[VA],
										pstVDecPrivate->stRingBuffState.pRingBuffEnd[VA],
										pScanStart,
										pScanEnd,
										pstVDecPrivate->stVDecInit.m_iBitstreamFormat == STD_AVC,
										ppSpaceEnd);
	}

#if 0
	{
		#define DEBUG_BINARY(__p_start, __p_end)\
		{\
			OMX_S32 idx0 = 0, idx1 = 0;\
			OMX_S32 count;\
			OMX_U8 start_bytes[8] = {0,0,0,0,0,0,0,0};\
			OMX_U8 end_bytes[8] = {0,0,0,0,0,0,0,0};\
			OMX_U8 *p_bytes;\
			\
			if( !CHECK_STATE(pstVDecPrivate, STATE_VDEC_BUFFER_PROTECTION) ) \
			{\
				if( __p_start < __p_end )\
				{\
					idx0 = 0;\
					count = 8;\
					p_bytes = __p_start;\
					while( idx0 < count && p_bytes < __p_end )\
						start_bytes[idx0++] = *p_bytes++;\
					\
					if( idx0 >= count ) {\
						idx1 = 0;\
						p_bytes = __p_end-8;\
						while( idx1 < count && p_bytes < __p_end )\
							end_bytes[idx1++] = *p_bytes++;\
					}\
				}\
				else\
				{\
					idx0 = 0;\
					count = 8;\
					p_bytes = __p_start;\
					while( idx0 < count && p_bytes < pRingEnd )\
						start_bytes[idx0++] = *p_bytes++;\
					\
					if( idx0 < count ) {\
						p_bytes = pRingBase;\
						while( idx0 < count && p_bytes < __p_end )\
							start_bytes[idx0++] = *p_bytes++;\
					\
						idx1 = 0;\
						p_bytes = __p_end-8;\
						if( p_bytes < pRingBase ) {\
							p_bytes = pRingBase;\
							count = (OMX_S32)(__p_end - pRingBase);\
							while( idx1 < count )\
								end_bytes[idx1++] = *p_bytes++;\
						}\
						else  {\
							while( idx1 < count && p_bytes < __p_end )\
								end_bytes[idx1++] = *p_bytes++;\
						}\
					}\
					else {\
						idx1 = 0;\
						p_bytes = __p_end-8;\
						if( p_bytes < pRingBase )\
							p_bytes = pRingEnd - (pRingBase - p_bytes);\
						\
						while( idx1 < count && p_bytes < pRingEnd )\
							end_bytes[idx1++] = *p_bytes++;\
						\
						if( idx1 < count ) {\
							p_bytes = pRingBase;\
							while( idx1 < count && p_bytes < __p_end )\
								end_bytes[idx1++] = *p_bytes++;\
						}\
					}\
				}\
				\
				CLOG(B, "[MGE][FILLER] %02X %02X %02X %02X %02X %02X %02X %02X (%2d) ~ %02X %02X %02X %02X %02X %02X %02X %02X (%2d)"\
					 , start_bytes[0] ,start_bytes[1] ,start_bytes[2] ,start_bytes[3]\
					 , start_bytes[4], start_bytes[5], start_bytes[6], start_bytes[7]\
					 , idx0\
					 , end_bytes[0], end_bytes[1], end_bytes[2], end_bytes[3]\
					 , end_bytes[4], end_bytes[5], end_bytes[6], end_bytes[7]\
					 , idx1\
					 );\
			}\
			else {\
				CLOG(B, "[MGE][FILLER] XX XX XX XX XX XX XX XX (--) ~ XX XX XX XX XX XX XX XX (--)");\
			}\
		}

		OMX_S32 rgn_size;
		OMX_U8 *pRingBase = pstVDecPrivate->stRingBuffState.pRingBuffBase[VA];
		OMX_U8 *pRingEnd = pstVDecPrivate->stRingBuffState.pRingBuffEnd[VA];
		OMX_U8 *p_filler_start = *ppSpaceEnd - size;
		OMX_U8 *p_filler_end = *ppSpaceEnd;
		OMX_S32 filler_size = size;

		if( p_filler_start < pRingBase )
			p_filler_start = pRingEnd - (pRingBase - p_filler_start);

		if( pScanStart < pScanEnd )
			rgn_size = (OMX_S32)(pScanEnd - pScanStart);
		else {
			rgn_size  = (OMX_S32)(pRingEnd - pScanStart);
			rgn_size += (OMX_S32)(pScanEnd - pRingBase);
		}

		if( filler_size )
		{
			CLOG(A, "[MGE][FILLER] [SCANRGN: %8d ~ %8d (LEN: %8d)] [PADRGN: %8d ~ %8d (LEN: %8d)] %s"
				  , pScanStart - pRingBase
				  , pScanEnd - pRingBase
				  , rgn_size
				  , filler_size ? p_filler_start - pRingBase : 0
				  , filler_size ? p_filler_end - pRingBase : 0
				  , filler_size ? filler_size : 0
				  , CHECK_STATE(pstVDecPrivate, STATE_VDEC_BUFFER_PROTECTION) ? "- secured" : ""
				  );

			if( filler_size ) {
				DEBUG_BINARY(p_filler_start, p_filler_end);
			} else {
				DEBUG_BINARY(pScanStart, pRingEnd);
			}
		}
	}
#endif

	return size;
}


static
OMX_TICKS
GetCurrTimestamp(
	vdec_private_t   *pstVDecPrivate,
	OMX_TICKS        *pllTimestamp,
	OMX_BOOL          bDecodeSuccess,
	OMX_BOOL          bInterlaced
	)
{
	if( UpdateRingBuffer(&pstVDecPrivate->stRingBuffState, OMX_TRUE) < 0 )
		return -1;

#if SINGLE_FRAME_INPUT_ENABLE
	if( CHECK_STATE(pstVDecPrivate, STATE_FRAME_UNIT_INPUT) )
	{
		input_info_t   info;

		if( GetInputInfo(&pstVDecPrivate->stInputInfoQueue, &info) < 0 )
			return -1;
		*pllTimestamp = info.llTimestamp;

		if( ShowInputInfo(&pstVDecPrivate->stInputInfoQueue, &info) < 0 )
			info.llTimestamp = -1;
		pstVDecPrivate->llQueuedStartTime = info.llTimestamp;

		LOG_IIMGE("[RESULT] --------- [CURRENT_BASE_PTS: %8d / NEXT_BASE_PTS: %8d] ---------"
				  , (OMX_S32)(*pllTimestamp/1000)
				  , (OMX_S32)(pstVDecPrivate->llQueuedStartTime/1000)
				  );
	}
	else
#endif
	{
		input_info_t   info;
		OMX_U8        *p_curr_rp = pstVDecPrivate->stRingBuffState.pReadPtr;
		OMX_U8        *p_prev_rp = pstVDecPrivate->stRingBuffState.pPrevReadPtr;
		OMX_TICKS      candidate_pts[4];
		OMX_S32        candidate_byte[4];
		OMX_S32        candidate_count = 0;
		OMX_S32        index = 0;
		OMX_TICKS      result_pts = -1;
		OMX_U8        *p_org_rp = p_curr_rp;	// for debug

		if( ShowInputInfo(&pstVDecPrivate->stInputInfoQueue, &info) < 0 ){
			return -1;
		}

		/* resolution with re-init step3 - flush delayed output frame (ring-buffer mode)*/
		if( CHECK_STATE(pstVDecPrivate, STATE_RESOLUTION_CHANGING_WITH_REINIT) ) {
			*pllTimestamp = result_pts;
			return 0;
		}

		if( p_curr_rp == pstVDecPrivate->stRingBuffState.pRingBuffEnd[VA] ){
			p_curr_rp = 0;
		}

		LOG_IIMGE("[SEARCH] [PTS: %8ld / REGION: %7ld ~ %7ld (%5ld Byte)] - [PRP: %7ld ~ RP: %7ld]"
				  , (OMX_S32)(info.llTimestamp/1000)
				  , info.pStartPtr - pstVDecPrivate->stRingBuffState.pRingBuffBase[VA]
				  , info.pEndPtr - pstVDecPrivate->stRingBuffState.pRingBuffBase[VA]
				  , info.lFilledLen
				  , (p_prev_rp >= pstVDecPrivate->stRingBuffState.pRingBuffBase[VA]) ? (p_prev_rp - pstVDecPrivate->stRingBuffState.pRingBuffBase[VA]) : 0
				  , (p_curr_rp >= pstVDecPrivate->stRingBuffState.pRingBuffBase[VA]) ? (p_curr_rp - pstVDecPrivate->stRingBuffState.pRingBuffBase[VA]) : 0
				  );

		if( info.pStartPtr < info.pEndPtr ) {
			if( (info.pStartPtr <= p_curr_rp) && (p_curr_rp <= info.pEndPtr) ) {
				if( bDecodeSuccess && ScanFillerSpace(pstVDecPrivate, p_curr_rp, info.pEndPtr, &p_curr_rp) > 0 ){
					pstVDecPrivate->stRingBuffState.pReadPtr = p_curr_rp;
				}
				result_pts = info.llTimestamp;
			}
		}
		else {
			if( (p_curr_rp <= info.pEndPtr) || (info.pStartPtr <= p_curr_rp) ) {
				if( bDecodeSuccess && ScanFillerSpace(pstVDecPrivate, p_curr_rp, info.pEndPtr, &p_curr_rp) > 0 ){
					pstVDecPrivate->stRingBuffState.pReadPtr = p_curr_rp;
				}
				result_pts = info.llTimestamp;
			}
		}

		if( result_pts < 0 )
		{
			if( info.pStartPtr < info.pEndPtr ) {
				candidate_byte[index] = (OMX_S32)(info.pEndPtr - p_prev_rp);
			}
			else {
				if( p_prev_rp < info.pEndPtr ){
					candidate_byte[index]  = (OMX_S32)(info.pEndPtr - p_prev_rp);
				} else {
					candidate_byte[index]  = (OMX_S32)(pstVDecPrivate->stRingBuffState.pRingBuffEnd[VA] - p_prev_rp);
					candidate_byte[index] += (OMX_S32)(info.pEndPtr - pstVDecPrivate->stRingBuffState.pRingBuffBase[VA]);
				}
			}
			candidate_pts[index] = info.llTimestamp;
			candidate_count++;

			while( 1 )
			{
				ClearFirstInputInfo(&pstVDecPrivate->stInputInfoQueue);
				if( ShowInputInfo(&pstVDecPrivate->stInputInfoQueue, &info) < 0 ) {
					info.pStartPtr   = 0;
					info.pEndPtr     = 0;
					info.llTimestamp = -1;	// queued input is not exist.
					break;
				}

				LOG_IIMGE("[SEARCH] [PTS: %8ld / REGION: %7ld ~ %7ld (%5ld Byte)] - [PRP: %7ld ~ RP: %7ld]"
						  , (OMX_S32)(info.llTimestamp/1000)
						  , info.pStartPtr - pstVDecPrivate->stRingBuffState.pRingBuffBase[VA]
						  , info.pEndPtr - pstVDecPrivate->stRingBuffState.pRingBuffBase[VA]
						  , info.lFilledLen
						  , p_prev_rp - pstVDecPrivate->stRingBuffState.pRingBuffBase[VA]
						  , p_curr_rp - pstVDecPrivate->stRingBuffState.pRingBuffBase[VA]
						  );

				if( ++index >= 4 ){
					index = 0;
				}
				candidate_pts[index] = info.llTimestamp;
				candidate_byte[index] = info.lFilledLen;
				candidate_count++;

				if( info.pStartPtr < info.pEndPtr ) {
					if( (info.pStartPtr <= p_curr_rp) && (p_curr_rp <= info.pEndPtr) ) {
						if( bDecodeSuccess && ScanFillerSpace(pstVDecPrivate, p_curr_rp, info.pEndPtr, &p_curr_rp) > 0 ){
							pstVDecPrivate->stRingBuffState.pReadPtr = p_curr_rp;
						}
						candidate_byte[index]  = (OMX_S32)(p_curr_rp - info.pStartPtr);
						break;
					}
				}
				else {
					if( (p_curr_rp <= info.pEndPtr) || (info.pStartPtr <= p_curr_rp) ) {
						if( bDecodeSuccess && ScanFillerSpace(pstVDecPrivate, p_curr_rp, info.pEndPtr, &p_curr_rp) > 0 ){
							pstVDecPrivate->stRingBuffState.pReadPtr = p_curr_rp;
						}

						if( p_curr_rp <= info.pEndPtr ) {
							candidate_byte[index]  = (OMX_S32)(pstVDecPrivate->stRingBuffState.pRingBuffEnd[VA] - info.pStartPtr);
							candidate_byte[index] += (OMX_S32)(p_curr_rp - pstVDecPrivate->stRingBuffState.pRingBuffBase[VA]);
						} else {
							candidate_byte[index]  = (OMX_S32)(p_curr_rp - info.pStartPtr);
						}
						break;
					}
				}
			}
		}

		LOG_IIMGE("[SEARCH][CANDIDATE: %ld] [%6lld (%6ld Byte)] [%6lld (%6ld Byte)] [%6lld (%6ld Byte)] [%6lld (%6ld Byte)]"
				  , candidate_count
				  , (candidate_count > 0) ? (candidate_pts[0]/1000) : -1
				  , (candidate_count > 0) ? (candidate_byte[0]) : -1
				  , (candidate_count > 1) ? (candidate_pts[1]/1000) : -1
				  , (candidate_count > 1) ? (candidate_byte[1]) : -1
				  , (candidate_count > 2) ? (candidate_pts[2]/1000) : -1
				  , (candidate_count > 2) ? (candidate_byte[2]) : -1
				  , (candidate_count > 3) ? (candidate_pts[3]/1000) : -1
				  , (candidate_count > 3) ? (candidate_byte[3]) : -1
				  );

		if( bDecodeSuccess )
		{
			if( bInterlaced )
			{
				if( candidate_count == 2 ) {	// candidate count == 2
					result_pts = candidate_pts[1];
				}
				else if( candidate_count == 3 ) {
					if( candidate_byte[0] < candidate_byte[2] ){
						result_pts = candidate_pts[2];
					} else {
						result_pts = candidate_pts[1];
					}
				}
				else if( candidate_count == 4 ) {
					result_pts = candidate_pts[2];
				}
				else if( candidate_count > 4 ) {
					if( ((candidate_byte[index] * 100) / info.lFilledLen) < 1 ) {// < 1 percent
						result_pts = candidate_pts[index-1 < 0 ? 3 : index-1];
					} else {
						result_pts = candidate_pts[index];
					}
				}
			}
			else
			{
				if( candidate_count == 2 ) {	// candidate count == 2
					if( candidate_byte[0] <= candidate_byte[1] ){
						result_pts = candidate_pts[1];
					} else {
						result_pts = candidate_pts[0];
					}
				}
				else if( candidate_count == 3 ) {
					result_pts = candidate_pts[1];	// second (mid) pts
				}
				else if( candidate_count >= 4 ) {
					if( (candidate_byte[index] * 100) / info.lFilledLen < 1 ) {// < 1 percent
						result_pts = candidate_pts[index-1 < 0 ? 3 : index-1];
					} else {
						result_pts = candidate_pts[index];
					}
				}
			}

			*pllTimestamp = result_pts;
		}
		else
		{
			*pllTimestamp = info.llTimestamp;
		}

		if( p_curr_rp == info.pEndPtr ) {
			ClearFirstInputInfo(&pstVDecPrivate->stInputInfoQueue);
			ShowInputInfo(&pstVDecPrivate->stInputInfoQueue, &info);
		}

		if( pstVDecPrivate->llQueuedStartTime != info.llTimestamp ){
			pstVDecPrivate->llQueuedStartTime = info.llTimestamp;
		}

		LOG_IIMGE("[RESULT] --------- [INPUT_RP: %8d / BASE_PTS: %8d] ---------"
				  , p_org_rp - pstVDecPrivate->stRingBuffState.pRingBuffBase[VA]
				  , (OMX_S32)(*pllTimestamp/1000)
				  );
	}

	return 0;
}

static
OMX_BOOL
BackupRingBuffer(
	vdec_private_t        *pstVDecPrivate
	)
{
	ringbuff_state_t *p_rb_state = &pstVDecPrivate->stRingBuffState;
	OMX_S32 start_pos = (OMX_S32)(p_rb_state->pReadPtr - p_rb_state->pRingBuffBase[VA]);
	OMX_S32 length = 0;

	INFO("Backup Ring-Buffer");

#if defined(TC_SECURE_MEMORY_COPY)
	if( CHECK_STATE(pstVDecPrivate, STATE_VDEC_BUFFER_PROTECTION) )
	{
		if( pstVDecPrivate->lSeqBuffMapSize <= 0 ) {
			ERROR("BackupRingBuffer() - invalid map size of the backup buffer");
			return OMX_FALSE;
		}

		if( p_rb_state->pReadPtr < p_rb_state->pWritePtr )
		{
			length = (OMX_S32)(p_rb_state->pWritePtr - p_rb_state->pReadPtr);

			TC_SecureMemoryCopy((unsigned int)(pstVDecPrivate->pSeqBuffBase[PA]),
								(unsigned int)(p_rb_state->pRingBuffBase[PA] + (p_rb_state->pReadPtr - p_rb_state->pRingBuffBase[VA])),
								(unsigned int)length);
		}
		else
		{
			OMX_S32 len0 = p_rb_state->pRingBuffEnd[VA] - p_rb_state->pReadPtr;
			OMX_S32 len1 = p_rb_state->pWritePtr - p_rb_state->pRingBuffBase[VA];

			length = len0 + len1;

			TC_SecureMemoryCopy((unsigned int)(pstVDecPrivate->pSeqBuffBase[PA]),
								(unsigned int)(p_rb_state->pRingBuffBase[PA] + (p_rb_state->pReadPtr - p_rb_state->pRingBuffBase[VA])),
								(unsigned int)len0);

			TC_SecureMemoryCopy((unsigned int)(pstVDecPrivate->pSeqBuffBase[PA] + len0),
								(unsigned int)(p_rb_state->pRingBuffBase[PA]),
								(unsigned int)len1);
		}
	}
	else
#endif
	{
		if( p_rb_state->pBackupBuffer == NULL ) {
			p_rb_state->pBackupBuffer = TCC_calloc(1, p_rb_state->lRingBuffSize);
			if( p_rb_state->pBackupBuffer == NULL ) {
				ERROR("BackupAndUpdateInputState() - out of memory");
				return OMX_FALSE;
			}
		}

		if( p_rb_state->pReadPtr < p_rb_state->pWritePtr )
		{
			length = (OMX_S32)(p_rb_state->pWritePtr - p_rb_state->pReadPtr);

			(void)memcpy((void*)(p_rb_state->pBackupBuffer),
						(void*)p_rb_state->pReadPtr, length);
		}
		else
		{
			OMX_S32 len0 = p_rb_state->pRingBuffEnd[VA] - p_rb_state->pReadPtr;
			OMX_S32 len1 = p_rb_state->pWritePtr - p_rb_state->pRingBuffBase[VA];

			length = len0 + len1;

			(void)memcpy((void*)p_rb_state->pBackupBuffer,
						(void*)p_rb_state->pReadPtr, len0);

			(void)memcpy((void*)(p_rb_state->pBackupBuffer + len0),
						(void*)p_rb_state->pRingBuffBase[VA], len1);
		}
	}

	p_rb_state->pPrevRingBuffBase = p_rb_state->pRingBuffBase[VA];
	p_rb_state->pPrevRingBuffEnd = p_rb_state->pRingBuffEnd[VA];
	p_rb_state->lBackupStartOffset = start_pos;
	p_rb_state->lBackupLength = length;

	return OMX_TRUE;
}

static
OMX_BOOL
RestoreRingBuffer(
	vdec_private_t        *pstVDecPrivate
	)
{
	input_info_t info;
	OMX_S32 offset;
	OMX_S32 count;
	OMX_S32 length;

	ringbuff_state_t *p_rb_state = &pstVDecPrivate->stRingBuffState;
	OMX_U8 *p_start;
	OMX_U8 *p_curr_base = p_rb_state->pRingBuffBase[VA];

	INFO("Restore Ring-Buffer");

	ResetRingBuffer(p_rb_state, OMX_FALSE);
	offset = 0;

	if( (count = GetInputInfo(&pstVDecPrivate->stInputInfoQueue, &info)) < 0 ){
		return OMX_FALSE;
	}

	LOG_IIMGE("[GET   ] [PTS: %8ld / REGION: %7ld ~ %7ld / LEN: %6ld]"
			  , (OMX_S32)(info.llTimestamp/1000)
			  , info.pStartPtr - p_rb_state->pPrevRingBuffBase
			  , info.pEndPtr - p_rb_state->pPrevRingBuffBase
			  , info.lFilledLen
			  );

	info.pStartPtr = p_rb_state->pPrevRingBuffBase + p_rb_state->lBackupStartOffset;

	if( info.pStartPtr < info.pEndPtr ){
		info.lFilledLen = (OMX_S32)(info.pEndPtr-info.pStartPtr);
	} else {
		info.lFilledLen  = (OMX_S32)(p_rb_state->pPrevRingBuffEnd-info.pStartPtr);
		info.lFilledLen += (OMX_S32)(info.pEndPtr-p_rb_state->pPrevRingBuffBase);
	}

	LOG_IIMGE("[GET   ] [PTS: %8ld / REGION: %7ld ~ %7ld / LEN: %6ld]"
			  , (OMX_S32)(info.llTimestamp/1000)
			  , info.pStartPtr - p_rb_state->pPrevRingBuffBase
			  , info.pEndPtr - p_rb_state->pPrevRingBuffBase
			  , info.lFilledLen
			  );

	info.pStartPtr = p_curr_base + offset;
	info.pEndPtr   = info.pStartPtr + info.lFilledLen;
	offset += info.lFilledLen;

	LOG_IIMGE("[PUSH  ] [PTS: %8ld / REGION: %7ld ~ %7ld / LEN: %6ld]"
			  , (OMX_S32)(info.llTimestamp/1000)
			  , info.pStartPtr - p_curr_base
			  , info.pEndPtr - p_curr_base
			  , info.lFilledLen
			  );

	if( PushInputInfo(&pstVDecPrivate->stInputInfoQueue, &info) < 0 ){
		return OMX_FALSE;
	}

	p_rb_state->pPrevWritePtr = p_rb_state->pWritePtr;
	p_rb_state->pWritePtr     += info.lFilledLen;
	p_rb_state->lWrittenBytes += info.lFilledLen;
	p_rb_state->lEmptySpace   -= info.lFilledLen;

	pstVDecPrivate->llQueuedStartTime = info.llTimestamp;

	while( count-- )
	{
		if( GetInputInfo(&pstVDecPrivate->stInputInfoQueue, &info) < 0 ){
			break;
		}

		LOG_IIMGE("[GET   ] [PTS: %8ld / REGION: %7ld ~ %7ld / LEN: %6ld]"
				  , (OMX_S32)(info.llTimestamp/1000)
				  , info.pStartPtr - p_rb_state->pPrevRingBuffBase
				  , info.pEndPtr - p_rb_state->pPrevRingBuffBase
				  , info.lFilledLen
				  );

		info.pStartPtr = p_curr_base + offset;
		info.pEndPtr   = info.pStartPtr + info.lFilledLen;
		offset += info.lFilledLen;

		LOG_IIMGE("[PUSH  ] [PTS: %8ld / REGION: %7ld ~ %7ld / LEN: %6ld]"
				  , (OMX_S32)(info.llTimestamp/1000)
				  , info.pStartPtr - p_curr_base
				  , info.pEndPtr - p_curr_base
				  , info.lFilledLen
				  );

		if( PushInputInfo(&pstVDecPrivate->stInputInfoQueue, &info) < 0 ){
			return OMX_FALSE;
		}

		p_rb_state->pPrevWritePtr = p_rb_state->pWritePtr;
		p_rb_state->pWritePtr     += info.lFilledLen;
		p_rb_state->lWrittenBytes += info.lFilledLen;
		p_rb_state->lEmptySpace   -= info.lFilledLen;
	}

#if defined(TC_SECURE_MEMORY_COPY)
	if( CHECK_STATE(pstVDecPrivate, STATE_VDEC_BUFFER_PROTECTION) )
	{
		TC_SecureMemoryCopy((unsigned int)(p_rb_state->pRingBuffBase[PA]),
							(unsigned int)(pstVDecPrivate->pSeqBuffBase[PA]),
							(unsigned int)(p_rb_state->lBackupLength));
	}
	else
#endif
	{
		(void)memcpy((void*)p_curr_base, (void*)(p_rb_state->pBackupBuffer), p_rb_state->lBackupLength);
	}

	if( UpdateRingBuffer(p_rb_state, OMX_FALSE) < 0 ){
		return OMX_FALSE;
	}

	p_rb_state->lBackupStartOffset = -1;

	return OMX_TRUE;
}

static
OMX_S32
FeedDecoder(
	vdec_private_t          *pstVDecPrivate,
	OMX_BUFFERHEADERTYPE	*pInputBuffer
	)
{
	/* sequence header first */
	if( pInputBuffer == NULL )
	{
		if( pstVDecPrivate->lSeqHeaderLength > 0 )
		{
#if defined(TC_SECURE_MEMORY_COPY)
			if( CHECK_STATE(pstVDecPrivate, STATE_VDEC_BUFFER_PROTECTION) )
			{
				if( FillRingBuffer_secure(&pstVDecPrivate->stRingBuffState,
										  pstVDecPrivate->pSeqBuffBase[PA],
										  pstVDecPrivate->lSeqHeaderLength) == OMX_FALSE )
				{
					return FEED_FAILED;
				}
			}
			else
#endif
			{
				if( FillRingBuffer(&pstVDecPrivate->stRingBuffState,
								   pstVDecPrivate->pbySequenceHeader,
								   pstVDecPrivate->lSeqHeaderLength) == OMX_FALSE )
				{
					return FEED_FAILED;
				}
			}
		}

		if( UpdateRingBuffer(&pstVDecPrivate->stRingBuffState, OMX_FALSE) < 0 ){
			return FEED_FAILED;
		}

		return NEED_MORE_DATA;
	}

	if( CHECK_STATE(pstVDecPrivate, STATE_VDEC_INITIALIZED) ) {
		if( CHECK_ERROR(pstVDecPrivate, ERROR_INSUFFICIENT_BITSTREAM) ){
			CLEAR_ERROR(pstVDecPrivate, ERROR_INSUFFICIENT_BITSTREAM);
		} else {
			if( pstVDecPrivate->stInputInfoQueue.lQueCount > 1 &&
				(pstVDecPrivate->stRingBuffState.lRingBuffSize - pstVDecPrivate->stRingBuffState.lEmptySpace) > WRITE_PTR_ALIGN_BYTES &&
				(pInputBuffer->nTimeStamp - pstVDecPrivate->llQueuedStartTime) > pstVDecPrivate->llFeedMaxTimeDiff
				)
			{
				LOG_FEED("[INPUT_CNT: %7ld] [QUEUED TIME: %8ld ~ %8ld ms] [DIFF:%10lld us] [LIMIT:%10lld us] - [OVER LIMIT]"
						 , pstVDecPrivate->lInputCount
						 , (OMX_S32)(pstVDecPrivate->llQueuedStartTime/1000)
						 , (OMX_S32)(pInputBuffer->nTimeStamp/1000)
						 , pInputBuffer->nTimeStamp - pstVDecPrivate->llQueuedStartTime
						 , pstVDecPrivate->llFeedMinTimeDiff
						 );

				if( UpdateRingBuffer(&pstVDecPrivate->stRingBuffState, OMX_FALSE) < 0 ){
					return FEED_FAILED;
				}

				if (!(pInputBuffer->nFlags & OMX_BUFFERFLAG_EOS)){
					OMX_S32 validsize = 0;
					OMX_BOOL need_more_data;
					need_more_data = CheckRemainRingBufferSize(&pstVDecPrivate->stRingBuffState, pstVDecPrivate, &validsize);
					if (need_more_data){
						LOG_RING("[%s %d] Need more data. CurrentRemainSize: %ld)", __func__,__LINE__, validsize);
						SET_ERROR(pstVDecPrivate, ERROR_INSUFFICIENT_BITSTREAM);
						return NEED_MORE_DATA;
					}
				}
				else {
					return FEED_COMPLETE;
				}
			}
		}
	}

	if( FillRingBuffer(&pstVDecPrivate->stRingBuffState,
					   pInputBuffer->pBuffer + pInputBuffer->nOffset,
					   pInputBuffer->nFilledLen) == OMX_TRUE )
	{
		input_info_t info;

		info.llTimestamp = pInputBuffer->nTimeStamp;
		info.lFilledLen  = pstVDecPrivate->mInBufUsedSize;
		info.pStartPtr   = pstVDecPrivate->stRingBuffState.pPrevWritePtr;
		info.pEndPtr     = pstVDecPrivate->stRingBuffState.pWritePtr;

		LOG_IIMGE("[PUSH  ] [PTS: %8ld / REGION: %7ld ~ %7ld]"
				  , (OMX_S32)(info.llTimestamp/1000)
				  , info.pStartPtr - pstVDecPrivate->stRingBuffState.pRingBuffBase[VA]
				  , info.pEndPtr - pstVDecPrivate->stRingBuffState.pRingBuffBase[VA]
				  );

		if( PushInputInfo(&pstVDecPrivate->stInputInfoQueue, &info) < 0 ){
			return FEED_FAILED;
		}

		pInputBuffer->nFilledLen -= pstVDecPrivate->mInBufUsedSize;
		pInputBuffer->nOffset += pstVDecPrivate->mInBufUsedSize;
		if (pInputBuffer->nFilledLen == 0)
			pstVDecPrivate->lInputCount++;

		LOG_FEED("[INPUT_CNT: %7ld] [QUEUED TIME: %8ld ~ %8ld ms] [DIFF:%10lld us] [LIMIT:%10lld us]"
				 , pstVDecPrivate->lInputCount
				 , (OMX_S32)(pstVDecPrivate->llQueuedStartTime/1000)
				 , (OMX_S32)(info.llTimestamp/1000)
				 , info.llTimestamp - pstVDecPrivate->llQueuedStartTime
				 , pstVDecPrivate->llFeedMinTimeDiff
				 );
		if( pstVDecPrivate->llQueuedStartTime < 0 ) {
			SET_ERROR(pstVDecPrivate, ERROR_INSUFFICIENT_BITSTREAM);
			pstVDecPrivate->llQueuedStartTime = info.llTimestamp;
			return NEED_MORE_DATA;
		}

		// check criteria amount of time
		if( (info.llTimestamp - pstVDecPrivate->llQueuedStartTime) < pstVDecPrivate->llFeedMinTimeDiff ){
			SET_ERROR(pstVDecPrivate, ERROR_INSUFFICIENT_BITSTREAM);
			return NEED_MORE_DATA;
		}

		#if UPDATE_WRITE_PTR_WITH_ALIGNED_LENGTH
		if( (pstVDecPrivate->stRingBuffState.lRingBuffSize - pstVDecPrivate->stRingBuffState.lEmptySpace) < WRITE_PTR_ALIGN_BYTES*4 ) {//FIXME
			SET_ERROR(pstVDecPrivate, ERROR_INSUFFICIENT_BITSTREAM);
			return NEED_MORE_DATA;
		}
		#endif
	}
	else
	{
		LOG_FEED("[INPUT_CNT: %7ld] [QUEUED TIME: %8ld ~ %8ld ms] [DIFF:%10lld us] [LIMIT:%10lld us] - [BUFFER FULL]"
				 , pstVDecPrivate->lInputCount
				 , (OMX_S32)(pstVDecPrivate->llQueuedStartTime/1000)
				 , (OMX_S32)(pInputBuffer->nTimeStamp/1000)
				 , pInputBuffer->nTimeStamp - pstVDecPrivate->llQueuedStartTime
				 , pstVDecPrivate->llFeedMinTimeDiff
				 );
	}

	if( UpdateRingBuffer(&pstVDecPrivate->stRingBuffState, OMX_FALSE) < 0 ){
		return FEED_FAILED;
	}

	if (!(pInputBuffer->nFlags & OMX_BUFFERFLAG_EOS)){
		OMX_S32 validsize = 0;
		OMX_BOOL need_more_data;
		need_more_data = CheckRemainRingBufferSize(&pstVDecPrivate->stRingBuffState, pstVDecPrivate, &validsize);
		if (need_more_data){
			LOG_RING("[%s %d] Need more data. CurrentRemainSize: %ld)", __func__,__LINE__, validsize);
			SET_ERROR(pstVDecPrivate, ERROR_INSUFFICIENT_BITSTREAM);
			return NEED_MORE_DATA;
		}
	}

	return FEED_COMPLETE;
}

#endif //DECODER_RINGBUFFER_MODE

static
OMX_S32
DecoderInit(
	vdec_private_t      *pstVDecPrivate
	)
{
	omx_base_video_PortType *p_outport = (omx_base_video_PortType *)pstVDecPrivate->ports[OMX_BASE_FILTER_OUTPUTPORT_INDEX];
	vdec_init_t      *p_vdec_init = &pstVDecPrivate->stVDecInit;
	vdec_user_info_t *p_userinfo = &pstVDecPrivate->stVDecUserInfo;
	OMX_S32 ret;

	INFO("decoder initialize");

	/* decoder initialize */
	p_vdec_init->m_iPicWidth           = pstVDecPrivate->stCropRect.nWidth;
	p_vdec_init->m_iPicHeight          = pstVDecPrivate->stCropRect.nHeight;
	p_vdec_init->m_bEnableVideoCache   = 0;
	p_vdec_init->m_bEnableUserData     = 0;
	p_vdec_init->m_pExtraData          = pstVDecPrivate->pbyExtraDataBuff;
	p_vdec_init->m_iExtraDataLen       = pstVDecPrivate->lExtraDataLength;
//	p_vdec_init->m_bM4vDeblk           = 0;
//	p_vdec_init->m_uiDecOptFlags       = 0;
	p_vdec_init->m_uiMaxResolution     = 0;

	if( CHECK_MODE(pstVDecPrivate, MODE_USERDATA_HANDLING) ){
		p_vdec_init->m_bEnableUserData     = 1;
	}

#ifdef INCLUDE_WMV78_DEC
	if (p_vdec_init->m_iBitstreamFormat == STD_WMV78) {
		if (pstVDecPrivate->eCodecVersion == OMX_VIDEO_WMVFormat7){
			p_vdec_init->m_iFourCC = 0x31564D57;  //WMV1
		} else if (pstVDecPrivate->eCodecVersion == OMX_VIDEO_WMVFormat8) {
			p_vdec_init->m_iFourCC = 0x32564D57;  //WMV2
		} else {
			ERROR("WMV format error (0x%08lX)", pstVDecPrivate->eCodecVersion);
			return -1;
		}

		if ((p_vdec_init->m_iPicWidth > 640) || (p_vdec_init->m_iPicHeight > 480))
		{
			WARN("WMV78 decoder library can't decode over 640 x 480 well, and this video is %d x %d. It can be played abnormally.",p_vdec_init->m_iPicWidth,p_vdec_init->m_iPicHeight);
		}
	}
#endif

	if( CHECK_MODE(pstVDecPrivate, MODE_RINGBUFFER_MODE) ) {
		INFO("RingBuffer Mode Enable!");
		p_vdec_init->m_bFilePlayEnable     = 0;
	}	else {
		p_vdec_init->m_bFilePlayEnable     = 1;
	}

	/* CbCr interleaved */
	if( p_outport->sVideoParam.eColorFormat == OMX_COLOR_FormatYUV420Planar ||
		p_outport->sVideoParam.eColorFormat == OMX_COLOR_FormatYUV420PlanarTc ){
		p_vdec_init->m_bCbCrInterleaveMode = 0;
	} else if( p_outport->sVideoParam.eColorFormat == OMX_COLOR_FormatYUV420SemiPlanar ||
			 p_outport->sVideoParam.eColorFormat == OMX_COLOR_FormatYUV420SemiPlanarTc ) {
		p_vdec_init->m_bCbCrInterleaveMode = 1;
	} else {
		ERROR("Invalid output format");
		return -1;
	}

	if (p_vdec_init->m_iBitstreamFormat == STD_MJPG) {
		p_vdec_init->m_bCbCrInterleaveMode = 0;
	}

	INFO("InterleaveMode %d ColorFormat 0x%X", p_vdec_init->m_bCbCrInterleaveMode, p_outport->sVideoParam.eColorFormat);
	//TAG:MOD - temp code (vdec_user_info_t)
	p_userinfo->bitrate_mbps = 10;
	p_userinfo->frame_rate = OUTPUT_FRAME_RATE_DEFAULT;
	p_userinfo->extFunction = 0;

#if defined(USE_MAP_CONVERTER) && (defined(TCC_HEVC_INCLUDE) || defined (TCC_VPU_4K_D2_INCLUDE)) && !defined(FORCE_DISABLE_MAP_CONVERTER)
	if ( CHECK_MODE(pstVDecPrivate, MODE_PHYSICAL_ADDRESS_OUTPUT) &&
		((p_vdec_init->m_iBitstreamFormat == STD_HEVC)
#if defined(TCC_VPU_4K_D2_INCLUDE)
		 || (p_vdec_init->m_iBitstreamFormat == STD_VP9)
#endif
		 )
	)
	{
		OMX_BOOL comp_map_forced = FALSE;
#if defined(OMX_DEBUG_MODE)
		char *env = getenv("OMX_VDEC_COMPRESSED_MAP_FORCED");

		if( env ){
			comp_map_forced = atoi(env);
		}
#endif
		if( comp_map_forced || ((p_vdec_init->m_iPicWidth*p_vdec_init->m_iPicHeight) > MIN_RESOLUTION_FOR_MAP_CONVERTER) ) {
			INFO("Compressed Map Enabled (comp_map_forced:%d)",comp_map_forced);
			p_userinfo->extFunction |= EXT_FUNC_USE_MAP_CONVERTER;
			SET_MODE(pstVDecPrivate, MODE_COMPRESSED_OUTPUT);
		}
	}
#endif

#if defined (TCC_VPU_4K_D2_INCLUDE) && defined(OMX_DEBUG_MODE)
	if (p_vdec_init->m_iBitstreamFormat == STD_HEVC)
	{
		char *env = getenv("OMX_VDEC_CQ");
		int32_t force_cq = 0;
		if( env ){
			force_cq = atoi(env);
		}
		if (force_cq) {
			if (force_cq == 1){
				p_userinfo->extFunction |= EXT_FUNC_4KD2_USE_CQ_LEVEL_1;
				SET_MODE(pstVDecPrivate, MODE_EXT_FUNC_USE_CQ_1);
				INFO("force set : COMMAND_QUEUE_DEPTH = 1");
			} else {
				p_userinfo->extFunction &= ~EXT_FUNC_4KD2_USE_CQ_LEVEL_1;
				INFO("force set : COMMAND_QUEUE_DEPTH = 2");
			}
		}
	}
#endif

#if RESOLUTION_CHANGE_WITH_CROP
#ifdef SET_FRAMEBUFFER_INTO_MAX
	if( IsSupportMaxFramebufferMode(pstVDecPrivate) ) {
		p_userinfo->extFunction |= EXT_FUNC_MAX_FRAMEBUFFER;
	}
#endif
#endif

#if defined(TC_SECURE_MEMORY_COPY)
	if( CHECK_STATE(pstVDecPrivate, STATE_VDEC_BUFFER_PROTECTION) ){
		p_userinfo->extFunction |= EXT_FUNC_MEM_PROTECTION_WITH_INSTREAM;
	}
#endif

	// 170724.1.no-buffer-delay
	if (CHECK_MODE(pstVDecPrivate, MODE_EXT_FUNC_NO_BUFFER_DELAY) )
	{
		INFO("No-Buffer-Delay enable forced!!");
		p_userinfo->extFunction |= EXT_FUNC_NO_BUFFER_DELAY;
	}

#if defined (TCC_VPU_4K_D2_INCLUDE)
	if (p_vdec_init->m_iBitstreamFormat == STD_VP9) {
		INFO("HIDE_SUPERFRAME_OUTPUT for VP9 ");
		p_userinfo->extFunction |= EXT_FUNC_HIDE_SUPERFRAME_OUTPUT;
	}
#endif

	if( pstVDecPrivate->pVDecInstance == NULL ){
		pstVDecPrivate->pVDecInstance = vdec_alloc_instance(pstVDecPrivate->stVDecInit.m_iBitstreamFormat, 0);
	}

	if( (ret = VDEC_FUNC(pstVDecPrivate, VDEC_INIT, NULL, p_vdec_init, p_userinfo)) < 0 )
	{
		LOGE("[VDEC_ERROR] [OP: VDEC_INIT] [RET_CODE: %ld]", ret);
		if(ret != -VPU_ENV_INIT_ERROR){
			SET_STATE(pstVDecPrivate, STATE_DECODER_OPENED); //to close decoder!!
		}
		return ret;
	}

	SET_STATE(pstVDecPrivate, STATE_DECODER_OPENED);

	/* vpu clock change with resolution and etc. */
	vpu_update_sizeinfo(p_vdec_init->m_iBitstreamFormat,
						p_userinfo->bitrate_mbps,
						p_userinfo->frame_rate,
						p_vdec_init->m_iPicWidth,
						p_vdec_init->m_iPicHeight,
						pstVDecPrivate->pVDecInstance);

	/* decoder ring-buffer initialize */
	if( CHECK_MODE(pstVDecPrivate, MODE_RINGBUFFER_MODE) ) {
		if( InitRingBuffer(&pstVDecPrivate->stRingBuffState, pstVDecPrivate) == OMX_FALSE ){
			return -1;
		}

		// for debugging
		pstVDecPrivate->stInputInfoQueue.pBasePtr = pstVDecPrivate->stRingBuffState.pRingBuffBase[VA];
	}

	/* display information manager initialize */
	if( !CHECK_STATE(pstVDecPrivate, STATE_IN_DEC_RESET_PROCESS) )
	{
		OMX_S32 frame_rate = 0;

		/* generate component instance ID */
#if 0	//TAG:MOD - component ID is not used
		//if( !CHECK_STATE(pstVDecPrivate, STATE_RESOLUTION_CHANGING) )
			//pstVDecPrivate->ulComponentUID = (OMX_U32)(systemTime(SYSTEM_TIME_MONOTONIC)/1000000);
#endif

		if( CHECK_MODE(pstVDecPrivate, MODE_RINGBUFFER_MODE) ) {
			/* input buffer information queue init. */
			if( InitInputQueue(&pstVDecPrivate->stInputInfoQueue, INPUT_INFO_QUEUE_INIT_SIZE) == OMX_FALSE ){
				return -1;
			}
		}

		/* display buffer index/id queue init. */
		ClearDispIdxQueue(&pstVDecPrivate->stDispIdxQueue);

		/* display information manager init. */
		//TAG:MOD - frame-rate
		if( p_userinfo->frame_rate ){
			frame_rate = p_userinfo->frame_rate;
		}

		InitDispInfoManager(&pstVDecPrivate->stDispInfoMgr,
							p_vdec_init->m_iBitstreamFormat,
							frame_rate);

		if( CHECK_MODE(pstVDecPrivate, MODE_RINGBUFFER_MODE) == 0) {
			INFO("Enable Timestamp sorting");
			pstVDecPrivate->stDispInfoMgr.ulFlags |= DISPMGR_TIMESTAMP_SORTING;
		}
	}

	/* I-frame search enable */
	IFrameSearchEnable(pstVDecPrivate);

	SET_STATE(pstVDecPrivate, STATE_SKIP_OUTPUT_B_FRAME);

	return 0;
}

static
void
DecoderDeinit(
	vdec_private_t *pstVDecPrivate
	)
{
	if( CHECK_STATE(pstVDecPrivate, STATE_DECODER_OPENED) ) {
		(void)VDEC_FUNC(pstVDecPrivate, VDEC_CLOSE, NULL, NULL, &pstVDecPrivate->stVDecOutput);
		CLEAR_STATE(pstVDecPrivate, STATE_DECODER_OPENED);

#if defined(TCC_HEVC_INCLUDE) || defined (TCC_VPU_4K_D2_INCLUDE)
		/** close decoder instance (close decoder driver) */
		if( pstVDecPrivate->stVDecInit.m_iBitstreamFormat == STD_HEVC
			#if defined (TCC_VPU_4K_D2_INCLUDE)
			|| pstVDecPrivate->stVDecInit.m_iBitstreamFormat == STD_VP9
			#endif
			) {
			if( pstVDecPrivate->pVDecInstance != NULL ){
				vdec_release_instance(pstVDecPrivate->pVDecInstance);
			}
			pstVDecPrivate->pVDecInstance = NULL;
		}
#endif
	}
}

static
OMX_S32
DecoderSeqInit(
	vdec_private_t      *pstVDecPrivate
	)
{
#define RETRY_WITH_MORE_DATA	1

	OMX_S32 ret;
	OMX_S32 additional_buff_count;
#if MVC_PROCESS_ENABLE && defined(TCC_VPU_C7_INCLUDE)
	omx_base_video_PortType *p_outport = (omx_base_video_PortType *)pstVDecPrivate->ports[OMX_BASE_FILTER_OUTPUTPORT_INDEX];
#endif
	/* Sequence header initialize */
	INFO("Sequence header initialize");

	pstVDecPrivate->stVDecInput.m_iIsThumbnail = 0;

	/* get reference frame buffer count */
#if MVC_PROCESS_ENABLE && defined(TCC_VPU_C7_INCLUDE)

	if( pstVDecPrivate->stVDecInit.m_iBitstreamFormat == STD_MVC ) {
		//TAG:MOD - MVC check
		#if 0
		VideoDec_Pmap_T pmap;
		if( st_func_get_pmap_info("video", &pmap) && pmap.iSize > 0x07000000 ) {
			INFO("MVC Output Enabled");
			SET_STATE(pstVDecPrivate, STATE_MVC_OUTPUT_ENABLE);
		}
		#endif
	}

	if( CHECK_STATE(pstVDecPrivate, STATE_MVC_OUTPUT_ENABLE) ) {
		if(p_outport->sPortParam.format.video.nFrameHeight == 720){
			additional_buff_count = OUTPUT_BUFFER_ACTUAL_MIN+4;
		} else {
			additional_buff_count = OUTPUT_BUFFER_ACTUAL_MIN+6;
		}
	} else {
		additional_buff_count = OUTPUT_BUFFER_ACTUAL_MIN;
	}

#else
	{
		additional_buff_count = OUTPUT_BUFFER_ACTUAL_MIN+7;
	#if defined(TCC_VP9_INCLUDE) || defined (TCC_VPU_4K_D2_INCLUDE)
		if (pstVDecPrivate->stVDecInit.m_iBitstreamFormat == STD_VP9)
		{
			if (additional_buff_count > 10){
				additional_buff_count = 10;
			}
		}
	#endif
	}
#endif

	/* set additional reference frame buffer count */
	vpu_set_additional_refframe_count(additional_buff_count, pstVDecPrivate->pVDecInstance);

	if( (ret = VDEC_FUNC(pstVDecPrivate,
						 VDEC_DEC_SEQ_HEADER,
						 NULL,
						 &pstVDecPrivate->stVDecInput,
						 &pstVDecPrivate->stVDecOutput)) < 0 )
	{
		LOGE("[VDEC_ERROR] [OP: VDEC_DEC_SEQ_HEADER] [RET_CODE: %ld]", ret);

		if ( ++pstVDecPrivate->lSeqInitFailCount >= pstVDecPrivate->lSeqInitFailMax ||
			 ret == -VPU_NOT_ENOUGH_MEM ||
			 ret == -RETCODE_INVALID_STRIDE ||
			 ret == -RETCODE_CODEC_SPECOUT ||
			 ret == -RETCODE_CODEC_EXIT )
		{
			ERROR("Sequence header init failed (INIT-CNT: %ld / %ld)",
					pstVDecPrivate->lSeqInitFailCount,
					pstVDecPrivate->lSeqInitFailMax);

			return ret;
		}

		if( CHECK_MODE(pstVDecPrivate, MODE_RINGBUFFER_MODE) ) {
			if( UpdateRingBuffer(&pstVDecPrivate->stRingBuffState, OMX_TRUE) < 0 ){
				return -1;
			}
		}

		INFO("Retry sequence header initialization with more data");
		return RETRY_WITH_MORE_DATA;
	}

	/* Store sequence header */
	INFO("Sequence header initialization success");
	if( pstVDecPrivate->lSeqHeaderLength <= 0 ) {
		//FIXME - sequence header backup
		if( CHECK_MODE(pstVDecPrivate, MODE_RINGBUFFER_MODE) ) {
			OMX_U8 *p_seqhead = pstVDecPrivate->stRingBuffState.pReadPtr;
			OMX_S32 seqhead_length = pstVDecPrivate->stRingBuffState.lRingBuffSize - pstVDecPrivate->stRingBuffState.lEmptySpace;
			seqhead_length = seqhead_length < SEQHEAD_LENGTH_MAX ? seqhead_length : SEQHEAD_LENGTH_MAX;
			if( StoreSequenceHeader(
						pstVDecPrivate,
						p_seqhead,
						seqhead_length) == OMX_FALSE )
			{
				ERROR("StoreSequenceHeader() - returns OMX_FALSE");
				return -1;
			}
		}
		else {
			OMX_U8 *p_seqhead = pstVDecPrivate->stVDecInput.m_pInp[VA];
			OMX_S32 seqhead_length = pstVDecPrivate->stVDecInput.m_iInpLen;

			if( (pstVDecPrivate->pbySequenceHeader != p_seqhead)
				&& (pstVDecPrivate->lSeqHeaderLength != seqhead_length) )
			{
				seqhead_length = (seqhead_length < SEQHEAD_LENGTH_MAX) ? seqhead_length : SEQHEAD_LENGTH_MAX;

				if( StoreSequenceHeader(
							pstVDecPrivate,
							p_seqhead,
							seqhead_length) == OMX_FALSE )
				{
					ERROR("StoreSequenceHeader() - returns OMX_FALSE");
					return -1;
				}
			}
		}
	}

	if( CHECK_MODE(pstVDecPrivate, MODE_RINGBUFFER_MODE) ) {
		if( UpdateRingBuffer(&pstVDecPrivate->stRingBuffState, OMX_TRUE) < 0 ){
			return -1;
		}
	}

	/* decoder ring-buffer reset */
	if( CHECK_STATE(pstVDecPrivate, STATE_IN_DEC_RESET_PROCESS) ) {
		if( CHECK_MODE(pstVDecPrivate, MODE_RINGBUFFER_MODE) ) {
			/* reset decoder ring-buffer state (resolution change: restored ring-buffer state)*/
			if( !CHECK_STATE(pstVDecPrivate, STATE_RESOLUTION_CHANGING_WITH_REINIT) ){
				ResetRingBuffer(&pstVDecPrivate->stRingBuffState, OMX_FALSE);
			}
			CLEAR_STATE(pstVDecPrivate, STATE_IN_DEC_RESET_PROCESS);
		}
	}

	pstVDecPrivate->lAdditionalBuffCount = additional_buff_count;
	pstVDecPrivate->lFrameBuffCount = vpu_get_refframe_count(pstVDecPrivate->pVDecInstance);

	if (pstVDecPrivate->lFrameBuffCount <= 0)
	{
		pstVDecPrivate->lFrameBuffCount = additional_buff_count;
	}

#ifdef TCC_VSYNC_INCLUDE
	{
		pstVDecPrivate->lMaxFifoCount = pstVDecPrivate->lFrameBuffCount - 2;
	}
#else
	pstVDecPrivate->lMaxFifoCount = pstVDecPrivate->lAdditionalBuffCount;
#endif
	INFO("Frame Buffer Count: %ld (additional: %ld, maxfifo: %ld)", pstVDecPrivate->lFrameBuffCount, pstVDecPrivate->lAdditionalBuffCount, pstVDecPrivate->lMaxFifoCount);

	return 0;
}

static
OMX_S32
OMXVideoDecode_Decode(
	vdec_private_t      *pstVDecPrivate,
	OMX_U32             *pulResultFlags
	)
{
	OMX_U32 dec_result = 0;
	OMX_S32 ret;

	/* resolution with re-init step3 - flush delayed output frame */
	if( CHECK_STATE(pstVDecPrivate, STATE_RESOLUTION_CHANGING_WITH_REINIT|STATE_FLUSHING_DELAYED_FRAME) )
	{
		if( (ret = VDEC_FUNC(pstVDecPrivate, VDEC_DEC_FLUSH_OUTPUT, NULL, &pstVDecPrivate->stVDecInput, &pstVDecPrivate->stVDecOutput)) < 0 ) {
			((omx_base_filter_PrivateType*)pstVDecPrivate)->delayedOutputBufferCount = 0;
			CLEAR_STATE(pstVDecPrivate, STATE_FLUSHING_DELAYED_FRAME);
			LOGE("[VDEC_ERROR] [OP: VDEC_DEC_FLUSH_OUTPUT] [RET_CODE: %ld]", ret);
			return -RETCODE_CODEC_FINISH;
		}
	}
	else
	{
		if( (ret = VDEC_FUNC(pstVDecPrivate, VDEC_DECODE, NULL, &pstVDecPrivate->stVDecInput, &pstVDecPrivate->stVDecOutput)) < 0 ) {
			LOGE("[VDEC_ERROR] [OP: VDEC_DECODE] [RET_CODE: %ld]", ret);
			return ret;
		}
	}

	/* set decoding status */
	switch( pstVDecPrivate->stVDecOutput.m_DecOutInfo.m_iDecodingStatus )
	{
	case VPU_DEC_SUCCESS:
		if( pstVDecPrivate->stVDecOutput.m_DecOutInfo.m_iDecodedIdx >= 0 ) {
			((omx_base_filter_PrivateType*)pstVDecPrivate)->delayedOutputBufferCount++;
			dec_result |= DECODING_SUCCESS_FRAME;
		}
		else if( pstVDecPrivate->stVDecOutput.m_DecOutInfo.m_iDecodedIdx == -2 ) {
			dec_result |= DECODING_SUCCESS_SKIPPED;
		}
		break;
	case VPU_DEC_SUCCESS_FIELD_PICTURE:
		dec_result |= DECODING_SUCCESS_FIELD;
		break;
	case VPU_DEC_BUF_FULL:
		dec_result |= DECODING_BUFFER_FULL;
		break;
	case 6: //FIXME - resolution change (not defined value)
		if( pstVDecPrivate->stVDecOutput.m_DecOutInfo.m_iDecodedIdx >= 0 ) {
			((omx_base_filter_PrivateType*)pstVDecPrivate)->delayedOutputBufferCount++;
			dec_result |= DECODING_SUCCESS_FRAME;
		}
		dec_result |= RESOLUTION_CHANGED;
		break;
	}

	/* set output status */
	if( pstVDecPrivate->stVDecOutput.m_DecOutInfo.m_iOutputStatus == VPU_DEC_OUTPUT_SUCCESS &&
		(pstVDecPrivate->lFrameSkipMode != SKIPMODE_I_SEARCH ||
		 pstVDecPrivate->stVDecOutput.m_DecOutInfo.m_iDecodedIdx == pstVDecPrivate->stVDecOutput.m_DecOutInfo.m_iDispOutIdx // check display index after I-frame searching (VPU issue)
		))
	{
		if( pstVDecPrivate->stVDecOutput.m_DecOutInfo.m_iDispOutIdx >= 0 ){
			((omx_base_filter_PrivateType*)pstVDecPrivate)->delayedOutputBufferCount--;
		}
		dec_result |= DISPLAY_OUTPUT_SUCCESS;
	}

	*pulResultFlags = dec_result;

	return 0;
}

static OMX_S32 DecSuccessProcess(vdec_private_t *pstVDecPrivate, OMX_TICKS llDecTimestamp, OMX_S32 lBufferIndex, OMX_BOOL bFieldSuccess)
{
	dec_output_info_t *p_output = &pstVDecPrivate->stVDecOutput.m_DecOutInfo;
	disp_info_t *p_info = GetDispInfoSlot(&pstVDecPrivate->stDispInfoMgr, lBufferIndex);
	OMX_S32 frame_rate = 0;

	if( p_info == NULL ) {
		ERROR("GetDispInfoSlot() - returns NULL");
		return -1;
	}

	if( bFieldSuccess )
	{
		p_info->llTimestamp    = llDecTimestamp;
		p_info->ulFlags        = DISPINFO_FLAG_FIELD_DECODED;
	}
	else
	{
		p_info->lFrameType           = p_output->m_iPicType;
		if( (p_info->ulFlags & DISPINFO_FLAG_FIELD_DECODED) == 0 || p_info->llTimestamp < llDecTimestamp ){
			p_info->llTimestamp          = llDecTimestamp;
		}
		p_info->lExtTimestamp         = 0;
		p_info->lPicStructure        = p_output->m_iPictureStructure;
		p_info->lM2vFieldSequence    = 0;
		p_info->lFrameDurationFactor = 0;
		p_info->lFrameSize           = 0; //FIXME - not used
		p_info->lErrorMB             = p_output->m_iNumOfErrMBs;
		p_info->lCropLeft            = p_output->m_CropInfo.m_iCropLeft;
		p_info->lCropTop             = p_output->m_CropInfo.m_iCropTop;
		p_info->lCropWidth           = p_output->m_iWidth - p_output->m_CropInfo.m_iCropLeft - p_output->m_CropInfo.m_iCropRight;
		p_info->lCropHeight          = p_output->m_iHeight - p_output->m_CropInfo.m_iCropTop - p_output->m_CropInfo.m_iCropBottom;
		p_info->ulFlags              = 0;

		switch( pstVDecPrivate->stVDecInit.m_iBitstreamFormat )
		{
#ifdef TCC_EXT_INCLUDED
		case STD_EXT:
			p_info->lExtTimestamp = p_output->m_iExtTimestamp;
			break;
#endif
		case STD_MPEG4:
		case STD_MVC:
		case STD_AVC:
			#if defined(TCC_VPU_C7_INCLUDE)
			if( CHECK_STATE(pstVDecPrivate, STATE_MVC_OUTPUT_ENABLE) && pstVDecPrivate->stVDecOutput.m_DecOutInfo.m_MvcPicInfo.m_iViewIdxDecoded )
				p_info->ulFlags |= DISPINFO_FLAG_DEPENDENT_VIEW;
			#endif

			if( (p_output->m_iM2vProgressiveFrame == 0 && p_output->m_iPictureStructure == 3)
				|| p_output->m_iInterlacedFrame
				|| ((p_output->m_iPictureStructure == 1) && (pstVDecPrivate->stVDecOutput.m_pInitialInfo->m_iInterlace == 0)) ){
				p_info->ulFlags |= DISPINFO_FLAG_INTERLACED;
			}

			p_info->lM2vFieldSequence = 0;
			break;

		case STD_VC1:
			if( (p_output->m_iM2vProgressiveFrame == 0 && p_output->m_iPictureStructure == 3)
				|| p_output->m_iInterlacedFrame
				|| ((p_output->m_iPictureStructure == 1) && (pstVDecPrivate->stVDecOutput.m_pInitialInfo->m_iInterlace == 0)) ){
				p_info->ulFlags |= DISPINFO_FLAG_INTERLACED;
			}

			p_info->lM2vFieldSequence = 0;
			break;

#if defined (TCC_HEVC_INCLUDE) || defined (TCC_VPU_4K_D2_INCLUDE)
		case STD_HEVC:
			//TODO: interlaced
			if( (p_output->m_iM2vProgressiveFrame == 0 && p_output->m_iPictureStructure == 3)
				|| p_output->m_iInterlacedFrame
				|| ((p_output->m_iPictureStructure == 1) && (pstVDecPrivate->stVDecOutput.m_pInitialInfo->m_iInterlace == 0)) ){
				p_info->ulFlags |= DISPINFO_FLAG_INTERLACED;
			}

			p_info->lM2vFieldSequence = 0;
			break;
#endif

		case STD_MPEG2:
			if ( (p_output->m_iM2vProgressiveFrame == 0 && p_output->m_iPictureStructure == 3 )
				  || (p_output->m_iInterlacedFrame
				    && (pstVDecPrivate->stVDecOutput.m_pInitialInfo->m_iInterlace)) ){
				p_info->ulFlags |= DISPINFO_FLAG_INTERLACED;
			}

			if( p_output->m_iTopFieldFirst == 0 ){
				p_info->ulFlags |= DISPINFO_FLAG_ODDFIELD_FIRST;
			}

			p_info->lFrameDurationFactor = 2;

			if( p_output->m_iPictureStructure != 3 )
			{
				p_info->lFrameDurationFactor = 2; //FIXME - MPEG-2 PTS calculation (origin: 1)
			}
			else if(p_output->m_iM2vProgressiveSequence == 0 )
			{
				if( p_output->m_iM2vProgressiveFrame == 0 )
				{
					p_info->lFrameDurationFactor = 2;
				}
				else
				{
					if(p_output->m_iRepeatFirstField == 0)
					{
						p_info->lFrameDurationFactor =2 ;
					}
					else
					{
						p_info->lFrameDurationFactor = 3 ;
					}
				}
			}
			else
			{
				if( p_output->m_iRepeatFirstField == 0 )
				{
					p_info->lFrameDurationFactor = 2;
				}
				else
				{
					if(p_output->m_iTopFieldFirst == 0)
					{
						p_info->lFrameDurationFactor = 4;
					}
					else
					{
						p_info->lFrameDurationFactor = 6;
					}
				}
			}

			p_info->lM2vFieldSequence = p_output->m_iM2vFieldSequence;
			frame_rate = p_output->m_iM2vFrameRate;
			frame_rate = ((frame_rate & 0xffff) * 1000) / ((frame_rate >> 16) + 1);

			break;

		default:
			p_info->lM2vFieldSequence = 0;
			break;
		}

		if( pstVDecPrivate->lNewFrameRate && frame_rate == 0 )  {
			frame_rate = pstVDecPrivate->lNewFrameRate;
			pstVDecPrivate->lNewFrameRate = 0;
		}

		if( !CHECK_MODE(pstVDecPrivate, MODE_RINGBUFFER_MODE) )
		{
			pstVDecPrivate->llQueuedStartTime = pstVDecPrivate->stDispInfoMgr.llLastDecTimestamp;
		}
		else {
			pstVDecPrivate->stDispInfoMgr.ulFlags &= ~DISPMGR_TIMESTAMP_GENERATING;
		}

		if( UpdateDispInfo(&pstVDecPrivate->stDispInfoMgr, lBufferIndex, frame_rate) == OMX_FALSE ){
			return -1;
		}

		if( (pstVDecPrivate->lDecodedCount > pstVDecPrivate->lPreviousKeyCount+1) &&
			GetFrameType(pstVDecPrivate->stVDecInit.m_iBitstreamFormat
						 , pstVDecPrivate->stVDecOutput.m_DecOutInfo.m_iPicType
						 , pstVDecPrivate->stVDecOutput.m_DecOutInfo.m_iPictureStructure) == PIC_TYPE_I ) {
			pstVDecPrivate->lKeyFrameDistance = pstVDecPrivate->lDecodedCount - pstVDecPrivate->lPreviousKeyCount;
			pstVDecPrivate->lPreviousKeyCount = pstVDecPrivate->lDecodedCount;
		}

		pstVDecPrivate->lDecodedCount++;

		if( !CHECK_MODE(pstVDecPrivate, MODE_RINGBUFFER_MODE) ) {
			/* move offset of temp input buffer */
			if( CHECK_STATE(pstVDecPrivate, STATE_SLICE_COUNTING) ) {
				pstVDecPrivate->lSliceCount++;
				if( p_output->m_iNumOfErrMBs > 0 ) {
					pstVDecPrivate->lTempInputOffset = pstVDecPrivate->stVDecInput.m_iInpLen;
					pstVDecPrivate->lNumberOfErrorFrame++;
					pstVDecPrivate->pbyVDecInputPtr = NULL;
				}
				else {
					pstVDecPrivate->lTempInputOffset = 0;
					pstVDecPrivate->lNumberOfSlice = pstVDecPrivate->lSliceCount;
					pstVDecPrivate->lSliceCount = 0;

					CLEAR_STATE(pstVDecPrivate, STATE_SLICE_COUNTING);
					SET_STATE(pstVDecPrivate, STATE_SLICE_MERGING);
					SET_STATE(pstVDecPrivate, STATE_SLICE_COUNTED);
				}
			}
		}
	}

	/* set next skip mode */
	if( pstVDecPrivate->lFrameSkipMode ){
		//SetFrameSkipMode(pstVDecPrivate, SKIPMODE_NEXT_STEP);
		SetNextFrameSkipMode(pstVDecPrivate,pstVDecPrivate->lFrameSkipMode);
	}

	/* user data storing */
	if( pstVDecPrivate->stVDecInit.m_bEnableUserData ) {
#ifdef TCC_VPU_4K_D2_INCLUDE
		OMX_S32 userdata_buf_size = WAVE5_USERDATA_BUF_SIZE;
#else
		OMX_S32 userdata_buf_size = WAVE4_USERDATA_BUF_SIZE;
#endif
		OMX_S32 length = GetUserDataLength(pstVDecPrivate->stVDecOutput.m_DecOutInfo.m_UserDataAddress[VA]);
		OMX_S32 max_length =(pstVDecPrivate->stVDecInit.m_iBitstreamFormat == STD_HEVC) ? 50*1024 : userdata_buf_size;
		if((length > 0) && (length <= max_length)){
			SetUserData(p_info, pstVDecPrivate->stVDecOutput.m_DecOutInfo.m_UserDataAddress[VA], length);
		}
	}

	pstVDecPrivate->lUsedBuffCount++;

	return 0;
}

static
OMX_S32
OutputProcess(
	vdec_private_t         *pstVDecPrivate,
	OMX_BUFFERHEADERTYPE   *pOutputBuffer,
	OMX_BOOL                bDecBuffOutput,
	OMX_S32                *plMbError
	)
{
	omx_base_video_PortType *p_outport = (omx_base_video_PortType *)pstVDecPrivate->ports[OMX_BASE_FILTER_OUTPUTPORT_INDEX];
	tcc_video_out_info *p_output = (tcc_video_out_info *)pOutputBuffer->pBuffer;
	OMX_S32 disp_idx, dv_el_disp_idx = -1;
	OMX_S32 is_dependent = 0;
	OMX_S32 ret;

	pOutputBuffer->nFilledLen = 0;
	pOutputBuffer->nOffset = 0;

	if( bDecBuffOutput )
	{
		disp_idx = pstVDecPrivate->stVDecOutput.m_DecOutInfo.m_iDecodedIdx;
	}
	else
	{
		disp_idx = pstVDecPrivate->stVDecOutput.m_DecOutInfo.m_iDispOutIdx;

		if( CHECK_STATE(pstVDecPrivate, STATE_WAIT_DISPLAY_KEYFRAME) ) {
			if( pstVDecPrivate->lKeyframeIndex == disp_idx ) {
				CLEAR_STATE(pstVDecPrivate, STATE_WAIT_DISPLAY_KEYFRAME);
				pOutputBuffer->nFilledLen = 0;
				return ERROR_SKIP_OUTPUT_FRAME;
			}
		}
	}

	#if MVC_PROCESS_ENABLE && defined(TCC_VPU_C7_INCLUDE)
	if( CHECK_STATE(pstVDecPrivate, STATE_MVC_OUTPUT_ENABLE) )
	{
		if( is_dependent )	// dependent
		{
			if( pstVDecPrivate->lMVCBaseViewIndex < 0 ) {
				ERROR("Dependent frame is decoded before its base frame");

				GetOutputBufferInfo(pstVDecPrivate, disp_idx, NULL);

				LOG_BUFCLR("[LINE: %4d] [DISP_IDX: %2d] [USED_COUNT: %2d]", __LINE__, disp_idx, pstVDecPrivate->lUsedBuffCount);

				pstVDecPrivate->lUsedBuffCount--;

				VDEC_FUNC(pstVDecPrivate, VDEC_BUF_FLAG_CLEAR, NULL, &disp_idx, NULL);

				return ERROR_INVALID_OUTPUT_FRAME;
			}

			pstVDecPrivate->lMVCBaseViewIndex = -1;
			pstVDecPrivate->pMVCBaseView[0] = NULL;
			pstVDecPrivate->pMVCBaseView[1] = NULL;
			pstVDecPrivate->pMVCBaseView[2] = NULL;

			pOutputBuffer->nFilledLen = 0; //FIXME
		}
		else
		{
			pstVDecPrivate->lMVCBaseViewIndex = disp_idx;
			pstVDecPrivate->pMVCBaseView[0] = NULL;
			pstVDecPrivate->pMVCBaseView[1] = NULL;
			pstVDecPrivate->pMVCBaseView[2] = NULL;

			pOutputBuffer->nFilledLen = 0;
		}
	}
	else
	#endif
	{
		if( CHECK_MODE(pstVDecPrivate, MODE_PHYSICAL_ADDRESS_OUTPUT) ){
			pOutputBuffer->nFilledLen = sizeof(tcc_video_out_info);
		} else {
			CopyOutputFrame(pstVDecPrivate, p_output, &pOutputBuffer->nFilledLen,
					(bDecBuffOutput) ? (COPY_FROM_DECODED_BUFFER) : (COPY_FROM_DISPLAY_BUFFER));
		}
	}

	if( pOutputBuffer->nFilledLen )
	{
		vdec_initial_info_t *p_init_info = pstVDecPrivate->stVDecOutput.m_pInitialInfo;
		dec_output_info_t *p_output_info = &pstVDecPrivate->stVDecOutput.m_DecOutInfo;
		output_info_t out_info;
		OMX_BOOL resolution_chagned = OMX_FALSE;

		/* get timestamp */
		if( (ret = GetOutputBufferInfo(pstVDecPrivate, disp_idx, &out_info)) < 0 ) {
			ERROR("GetOutputBufferInfo() - return: %ld", ret);
			return ret;
		}

        if( !CHECK_MODE(pstVDecPrivate, MODE_RINGBUFFER_MODE) ) {
            if( CHECK_STATE(pstVDecPrivate, STATE_SLICE_COUNTING|STATE_SLICE_COUNTED) ) {
                if( out_info.lErrorMB ) {
                    pOutputBuffer->nFilledLen = 0;

                    LOG_BUFCLR("[LINE: %4d] [DISP_IDX: %2ld] [EL_DISP_IDX: %2ld] [USED_COUNT: %2ld]", __LINE__, disp_idx, dv_el_disp_idx, pstVDecPrivate->lUsedBuffCount--);

                    pstVDecPrivate->lUsedBuffCount--;

                    if( (ret = VDEC_FUNC(pstVDecPrivate, VDEC_BUF_FLAG_CLEAR, NULL, &disp_idx, NULL)) < 0 ) {
                        LOGE("[VDEC_BUF_FLAG_CLEAR] Idx = %ld, ret = %ld", disp_idx, ret );
                        return ret;
                    }
                }

                if( --pstVDecPrivate->lNumberOfErrorFrame <= 0 && CHECK_STATE(pstVDecPrivate, STATE_SLICE_COUNTED) )
                    CLEAR_STATE(pstVDecPrivate, STATE_SLICE_COUNTED);
            }
        }

#if defined(TCC_VSYNC_INCLUDE)
        if(pstVDecPrivate->mFPS == -1 && (pstVDecPrivate->mDecodingCount++ > 1 && pstVDecPrivate->mDecodingCount < 30))
        {
            pstVDecPrivate->llFPS_time_sum += (out_info.llTimestamp - pstVDecPrivate->llFPS_time_prev);

            if(pstVDecPrivate->mOutputCount++ == 15)
            {
                OMX_U32 fps = 0;

                pstVDecPrivate->llFPS_time_diff = pstVDecPrivate->llFPS_time_sum / (pstVDecPrivate->mOutputCount);

                if(pstVDecPrivate->llFPS_time_diff > 1000)
                {
                    fps = (OMX_U32)(1000000 / (pstVDecPrivate->llFPS_time_diff / 1000 * 1000));
                }
                else
                {
                    LOGI("[%s:%d] llFPS_time_diff is less than 1000 - %lld\n", __func__, __LINE__, pstVDecPrivate->llFPS_time_diff);
                }

                if(fps > 0 && fps < 65)
                {
                    if(fps > 55)
                    {
                        fps = 60;
                    }
                    else if(fps > 45)
                    {
                        fps = 0; // 50
                    }
                    else if(fps > 27)
                    {
                        fps = 30;
                    }
                    else
                    {
                        fps = 0; // 24
                    }

                    pstVDecPrivate->mFPS = fps;
                }
                else
                {
                    pstVDecPrivate->mFPS = -1;
                    pstVDecPrivate->mOutputCount = 0;
                    pstVDecPrivate->mDecodingCount = 0;
                }
            }
        }

        p_output->mFPS = pstVDecPrivate->mFPS;
        pstVDecPrivate->llFPS_time_prev = out_info.llTimestamp;

        if (p_output->mFPS < 0) {
            p_output->mFPS = 0;
        }
#endif
        /* skip B frame before I frame */
        if( CHECK_STATE(pstVDecPrivate, STATE_SKIP_OUTPUT_B_FRAME) )
        {
            OMX_S32 pic_type = GetFrameType(pstVDecPrivate->stVDecInit.m_iBitstreamFormat,
                    out_info.lFrameType,
                    out_info.lPicStructure);

            if( ((pstVDecPrivate->stVDecInit.m_iBitstreamFormat != STD_MVC) || ((out_info.ulFlags & DISPINFO_FLAG_DEPENDENT_VIEW) == 0)) &&
				((pic_type == PIC_TYPE_I) || (pic_type == PIC_TYPE_P)) /*&& pstVDecPrivate->stVDecOutput.m_DecOutInfo.m_iDecodedIdx >= 0*/)
			{
                CLEAR_STATE(pstVDecPrivate, STATE_SKIP_OUTPUT_B_FRAME);
            }
            else {
                //Clear VPU Buffer Flag
                LOG_BUFCLR("[LINE: %4d] [DISP_IDX: %2d] [EL_DISP_IDX: %2d]  [USED_COUNT: %2d]", __LINE__, disp_idx, dv_el_disp_idx, pstVDecPrivate->lUsedBuffCount--);
                VDEC_FUNC(pstVDecPrivate, VDEC_BUF_FLAG_CLEAR, NULL, &disp_idx, NULL);
                pOutputBuffer->nFilledLen = 0;
                LOG_OUT("[SKIP][BUFF_IDX: %2d]", disp_idx);
                return ERROR_SKIP_OUTPUT_FRAME;
            }
        }

        /* resolution with crop step3 - wait output changed */
        if( CHECK_STATE(pstVDecPrivate, STATE_WAIT_CHANGED_OUTPUT) ) {
            if( pstVDecPrivate->stCropRect.nLeft    == out_info.lCropLeft &&
                    pstVDecPrivate->stCropRect.nTop     == out_info.lCropTop &&
                    pstVDecPrivate->stCropRect.nWidth   == out_info.lCropWidth &&
                    pstVDecPrivate->stCropRect.nHeight  == out_info.lCropHeight )
            {
                resolution_chagned = OMX_TRUE;
                INFO("Start of resolution changed output");
                CLEAR_STATE(pstVDecPrivate, STATE_WAIT_CHANGED_OUTPUT);
            }
        }

        if( CHECK_MODE(pstVDecPrivate, MODE_PHYSICAL_ADDRESS_OUTPUT) )
        {
            p_output->mType = TYPE_VIDEO;

            if( bDecBuffOutput ) {
                p_output->pCurrOut[PA][0] = pstVDecPrivate->stVDecOutput.m_pCurrOut[PA][0];
                p_output->pCurrOut[PA][1] = pstVDecPrivate->stVDecOutput.m_pCurrOut[PA][1];
                p_output->pCurrOut[PA][2] = pstVDecPrivate->stVDecOutput.m_pCurrOut[PA][2];
                p_output->pCurrOut[VA][0] = pstVDecPrivate->stVDecOutput.m_pCurrOut[VA][0];
                p_output->pCurrOut[VA][1] = pstVDecPrivate->stVDecOutput.m_pCurrOut[VA][1];
                p_output->pCurrOut[VA][2] = pstVDecPrivate->stVDecOutput.m_pCurrOut[VA][2];
            } else {
                p_output->pCurrOut[PA][0] = pstVDecPrivate->stVDecOutput.m_pDispOut[PA][0];
                p_output->pCurrOut[PA][1] = pstVDecPrivate->stVDecOutput.m_pDispOut[PA][1];
                p_output->pCurrOut[PA][2] = pstVDecPrivate->stVDecOutput.m_pDispOut[PA][2];
                p_output->pCurrOut[VA][0] = pstVDecPrivate->stVDecOutput.m_pDispOut[VA][0];
                p_output->pCurrOut[VA][1] = pstVDecPrivate->stVDecOutput.m_pDispOut[VA][1];
                p_output->pCurrOut[VA][2] = pstVDecPrivate->stVDecOutput.m_pDispOut[VA][2];
            }

            // MVC dependent view
            p_output->pMVCInfo[PA][0] = 0;	// physical address for Y (dependent view)
            p_output->pMVCInfo[PA][1] = 0;	// physical address for Cb (dependent view)
            p_output->pMVCInfo[PA][2] = 0;	// physical address for Cr (dependent view)
            p_output->pMVCInfo[VA][0] = 0;	// virtual address for Y (dependent view)
            p_output->pMVCInfo[VA][1] = 0;	// virtual address for Cb (dependent view)
            p_output->pMVCInfo[VA][2] = 0;	// virtual address for Cr (dependent view)

            if((p_output_info->m_iWidth == 1920) && (p_output_info->m_iHeight > 1080)){
                //HDMI :: to prevent scaling in case of 1920x1088 contents, from Android SDK tc_hwc_overlay.cpp
                p_output_info->m_iHeight = 1080;
            }

            // VPU output width/height
#if defined(USE_MAP_CONVERTER) && (defined(TCC_HEVC_INCLUDE) || defined (TCC_VPU_4K_D2_INCLUDE))
            if(CHECK_MODE(pstVDecPrivate, MODE_COMPRESSED_OUTPUT)
                    && (pstVDecPrivate->stVDecInit.m_iBitstreamFormat == STD_HEVC))
            {
                p_output->mWidth  = p_output_info->m_iWidth;
                p_output->mHeight = p_output_info->m_iHeight;

                // Crop Info.
                p_output->stCropInfo.iCropLeft   = p_output_info->m_CropInfo.m_iCropLeft;
                p_output->stCropInfo.iCropTop    = p_output_info->m_CropInfo.m_iCropTop;
                p_output->stCropInfo.iCropWidth  = p_output_info->m_iWidth - p_output_info->m_CropInfo.m_iCropLeft -p_output_info->m_CropInfo.m_iCropRight;
                p_output->stCropInfo.iCropHeight = p_output_info->m_iHeight - p_output_info->m_CropInfo.m_iCropTop - p_output_info->m_CropInfo.m_iCropBottom;
            }
            else
#endif
#if defined (TCC_VPU_4K_D2_INCLUDE)
                if(!CHECK_MODE(pstVDecPrivate, MODE_COMPRESSED_OUTPUT)
                        && (pstVDecPrivate->stVDecInit.m_iBitstreamFormat == STD_VP9))
                {
                    p_output->mWidth  = p_output_info->m_iWidth;
                    p_output->mHeight = p_output_info->m_iHeight;

                    // Crop Info.
                    p_output->stCropInfo.iCropLeft   = out_info.lCropLeft;
                    p_output->stCropInfo.iCropTop    = out_info.lCropTop;
                    p_output->stCropInfo.iCropWidth  = out_info.lCropWidth;
                    p_output->stCropInfo.iCropHeight = out_info.lCropHeight;
                }
                else
#endif
                {
                    p_output->mWidth  = p_init_info->m_iPicWidth;
                    p_output->mHeight = p_init_info->m_iPicHeight;

                    // Crop Info.
                    p_output->stCropInfo.iCropLeft   = out_info.lCropLeft;
                    p_output->stCropInfo.iCropTop    = out_info.lCropTop;
                    p_output->stCropInfo.iCropWidth  = out_info.lCropWidth;
                    p_output->stCropInfo.iCropHeight = out_info.lCropHeight;
                }
            // Stride
#if defined(TCC_HEVC_INCLUDE) || defined (TCC_VPU_4K_D2_INCLUDE)
            if(pstVDecPrivate->stVDecInit.m_iBitstreamFormat == STD_HEVC) {
                if(p_init_info->m_iFrameBufferFormat != 0) {
                    p_output->stStride.iY    = (((p_output_info->m_iWidth+31) >>5) << 5) * 2;
                    p_output->stStride.iCbCr = (((p_output_info->m_iWidth)+31) >> 5) << 5;
                }
                else
                {
                    p_output->stStride.iY = ((p_output_info->m_iWidth+31) >> 5) << 5;
                    if( p_outport->sVideoParam.eColorFormat == OMX_COLOR_FormatYUV420PlanarTc )
                        p_output->stStride.iCbCr = (((p_output_info->m_iWidth/2)+15) >> 4) << 4;
                    else if( p_outport->sVideoParam.eColorFormat == OMX_COLOR_FormatYUV420SemiPlanarTc )
                        p_output->stStride.iCbCr = ((p_output_info->m_iWidth+15) >> 4) << 4;
                }
            }
            else
#endif
#if defined (TCC_VPU_4K_D2_INCLUDE)
                if(pstVDecPrivate->stVDecInit.m_iBitstreamFormat == STD_VP9) {
                    if(p_init_info->m_iFrameBufferFormat != 0) {
                        p_output->stStride.iY = (((p_init_info->m_iPicWidth+31) >>5) << 5) * 2;
                        p_output->stStride.iCbCr = (((p_init_info->m_iPicWidth)+31) >> 5) << 5;
                    }
                    else
                    {
                        p_output->stStride.iY = ((p_init_info->m_iPicWidth+31) >> 5) << 5;
                        if( p_outport->sVideoParam.eColorFormat == OMX_COLOR_FormatYUV420PlanarTc )
                            p_output->stStride.iCbCr = (((p_init_info->m_iPicWidth/2)+15) >> 4) << 4;
                        else if( p_outport->sVideoParam.eColorFormat == OMX_COLOR_FormatYUV420SemiPlanarTc )
                            p_output->stStride.iCbCr = ((p_init_info->m_iPicWidth+15) >> 4) << 4;
                    }
                }
                else
#endif
                {
                    p_output->stStride.iY = ((p_init_info->m_iPicWidth+15) >> 4) << 4;
                    if( p_outport->sVideoParam.eColorFormat == OMX_COLOR_FormatYUV420PlanarTc )
                        p_output->stStride.iCbCr = (((p_init_info->m_iPicWidth/2)+7) >> 3) << 3;
                    else if( p_outport->sVideoParam.eColorFormat == OMX_COLOR_FormatYUV420SemiPlanarTc )
                        p_output->stStride.iCbCr = ((p_init_info->m_iPicWidth+7) >> 3) << 3;
                }

            // Interlaced
            p_output->mFlags = 0;
            if(out_info.ulFlags & DISPINFO_FLAG_INTERLACED) {
                p_output->mFlags |= DEC_FLAGS_INTERLACED;	// interlaced
                if(out_info.ulFlags & DISPINFO_FLAG_ODDFIELD_FIRST)
                    p_output->mFlags |= DEC_FLAGS_INTERLACED_ODD_FIRST;	// odd field first
            }

#if defined (TCC_VPU_4K_D2_INCLUDE)
            if(pstVDecPrivate->stVDecInit.m_iBitstreamFormat == STD_VP9
                    && !CHECK_MODE(pstVDecPrivate, MODE_COMPRESSED_OUTPUT))
            {
                p_output->mWidth = ((p_output->stCropInfo.iCropWidth +15) >>4) <<4;
                p_output->mHeight =((p_output->stCropInfo.iCropHeight + 7)>>3)<<3;

                p_output->stStride.iCbCr = p_output->stStride.iY = ((p_output->stCropInfo.iCropWidth +15) >>4) <<4;
            }
#endif

#if 0
            DBGM("[PIC: %dx%d] [RECT: (%d, %d) %dx%d)] [STRIDE: %d, %d] [INTERLACED: %d]"
                    , p_output->mWidth, p_output->mHeight
                    , p_output->stCropInfo.iCropTop, p_output->stCropInfo.iCropLeft, p_output->stCropInfo.iCropWidth, p_output->stCropInfo.iCropHeight
                    , p_output->stStride.iY, p_output->stStride.iCbCr
                    , (p_output->mFlags & DEC_FLAGS_INTERLACED ? 1 : 0)
                );
#endif

            /* resolution with crop step3 - wait output changed */
            if( resolution_chagned == OMX_TRUE )
                p_output->mFlags |= DEC_FLAGS_RESOLUTION_CHANGE;	// resolution changed

            if( CHECK_STATE(pstVDecPrivate, STATE_FRAME_DISCONTINUITY) ) {
                p_output->mFlags |= DEC_FLAGS_DISCONTINUITY;
                CLEAR_STATE(pstVDecPrivate, STATE_FRAME_DISCONTINUITY);
            }

            p_output->mDispIdx = NULL;
            p_output->mErrMB = out_info.lErrorMB;

            if( CHECK_MODE(pstVDecPrivate, MODE_COMPRESSED_OUTPUT)) {
#if defined(USE_MAP_CONVERTER)
                if ((pstVDecPrivate->stVDecInit.m_iBitstreamFormat == STD_HEVC)
#if defined (TCC_VPU_4K_D2_INCLUDE)
                 || (pstVDecPrivate->stVDecInit.m_iBitstreamFormat == STD_VP9)
#endif
                ) {
                    p_output->mFlags |= DEC_FLAGS_USE_MAP_CONV; // compressed map enabled
#if defined(TCC_HEVC_INCLUDE)
					(void)memcpy((void*)&p_output->stHEVCMapConv, (void*)&pstVDecPrivate->stVDecOutput.m_MapConvInfo, sizeof(hevc_dec_MapConv_info_t)); // Don't copy reseved region
#elif defined (TCC_VPU_4K_D2_INCLUDE)
					(void)memcpy((void*)&p_output->stHEVCMapConv, (void*)&pstVDecPrivate->stVDecOutput.m_MapConvInfo, sizeof(hevc_MapConv_info_t)); // Don't copy reseved region
#endif
					p_output->stHEVCMapConv.m_Reserved[0] = pstVDecPrivate->stVDecInit.m_iBitstreamFormat;
				}
#endif
            }

#if !defined(TCC_VSYNC_INCLUDE)
            p_output->mDispIdx = disp_idx;
#endif

            if( pstVDecPrivate->stVDecInit.m_iBitstreamFormat == STD_MJPG ) {
                //!< MJPEG source chroma format(0 - 4:2:0, 1 - 4:2:2, 2 - 4:2:2 vertical, 3 - 4:4:4, 4 - 4:0:0 )
                p_output->mColorFormat = pstVDecPrivate->stVDecOutput.m_pInitialInfo->m_iMjpg_sourceFormat;
                if (p_output->mColorFormat == 0)
                    p_output->mColorFormat = 5; // 4:2:0 sep.
            } else {
                p_output->mColorFormat = (p_outport->sVideoParam.eColorFormat == OMX_COLOR_FormatYUV420SemiPlanarTc)
                    ? 0 : 5; // 4:2:0 => 0 : sp / 5 : p; // 4:2:0
            }

            // Subtitle info.
            p_output->stSubTitileInfo.iSubEnable   = 0; //sutitle on/off flag
            p_output->stSubTitileInfo.iSubAddr     = 0; //subtitle base addr
            p_output->stSubTitileInfo.iSubIdx      = 0; //subtitle buf index
            p_output->stSubTitileInfo.iSubWidth    = 0; //width
            p_output->stSubTitileInfo.iSubHeight   = 0; //height
            p_output->stSubTitileInfo.iSuboffset_x = 0; //x offset
            p_output->stSubTitileInfo.iSuboffset_y = 0; //y offset

        }

		PushDispIdx(&pstVDecPrivate->stDispIdxQueue, disp_idx, dv_el_disp_idx);

#if defined(TCC_VSYNC_INCLUDE)
		pstVDecPrivate->bClearDisplayIndex = OMX_TRUE;
#else
		ClearDisplayedBuffer(pstVDecPrivate, OMX_FALSE);
#endif

		pOutputBuffer->nTimeStamp = out_info.llTimestamp;

		if((out_info.lErrorMB > 0) && plMbError) {
			*plMbError = out_info.lErrorMB;
		}

		if( resolution_chagned == OMX_TRUE ) {
			return SUCCESS_RESOLUTION_CHANGED;
		}

		if( out_info.lUserDataLength > 0 ) {
			SendUserData(pstVDecPrivate, out_info.pbyUserDataBuff, out_info.lUserDataLength, out_info.llTimestamp);
		}
	}

	return 0;
}

#define PROCESS_NONE            0
#define PROCESS_OPEN            1
#define PROCESS_SEQUENCEINIT    2
#define PROCESS_DECODE			3

static
OMX_BOOL // OMX_TRUE: return immediately / OMX_FALSE: try next line
ErrorProcess(
	vdec_private_t        *pstVDecPrivate,
	OMX_BUFFERHEADERTYPE  *pInputBuffer,
	OMX_BUFFERHEADERTYPE  *pOutputBuffer,
	OMX_S32 lErrorCode,
	OMX_S32 lProcessStatus
	)
{
	if( pOutputBuffer ) {
		pOutputBuffer->nFilledLen = 0;
	}

	switch( lErrorCode ) {
	case -RETCODE_CODEC_EXIT:
	case -RETCODE_MULTI_CODEC_EXIT_TIMEOUT:
		if( !CHECK_STATE(pstVDecPrivate, STATE_IN_DEC_RESET_PROCESS) && pstVDecPrivate->lSeqHeaderLength > 0 )
		{
			ERROR("ErrorProcess() - [VDEC_ERROR] Codec Exit (code: %ld)", lErrorCode);

			pstVDecPrivate->lSeqInitFailCount = 0;
			pstVDecPrivate->lInputCount       = 0;
			pstVDecPrivate->lDecodedCount     = 0;
			pstVDecPrivate->lPreviousKeyCount = 0;
			pstVDecPrivate->lKeyFrameDistance = 0;
			pstVDecPrivate->lDecodingFailCount= 0;
			pstVDecPrivate->lSkippedCount     = 0;
			pstVDecPrivate->llQueuedStartTime = -1;

			SET_STATE(pstVDecPrivate, STATE_IN_DEC_RESET_PROCESS);
			CLEAR_STATE(pstVDecPrivate, STATE_VDEC_INITIALIZED);

			if( CHECK_MODE(pstVDecPrivate, MODE_RINGBUFFER_MODE) ) {
				/* reset ring-buffer state */
				if(ResetRingBuffer(&pstVDecPrivate->stRingBuffState, OMX_FALSE) == 0) {
					/* reset input/output information managers */
					ClearInputQueue(&pstVDecPrivate->stInputInfoQueue);
				}
			}

			ClearDispIdxQueue(&pstVDecPrivate->stDispIdxQueue);
			ResetDispInfoManager(&pstVDecPrivate->stDispInfoMgr);
			CLEAR_STATE(pstVDecPrivate, STATE_CLEAR_DISPLAY_BUFFER);

			/* decoder close */
			DecoderDeinit(pstVDecPrivate);
		}
		break;

	case -RETCODE_INSUFFICIENT_BITSTREAM:
		{
			if( CHECK_MODE(pstVDecPrivate, MODE_RINGBUFFER_MODE) ) {
				ringbuff_state_t *p_ring_state = &pstVDecPrivate->stRingBuffState;
				input_info_t info_first;
				input_info_t info_last;
				OMX_TICKS temp_timestamp;

				LOG_RING("ErrorProcess() - [VDEC_ERROR] Insufficient bitstream (code: %ld)", lErrorCode);

				if( ShowInputInfo(&pstVDecPrivate->stInputInfoQueue, &info_first) >= 0 &&
					ShowLastInputInfo(&pstVDecPrivate->stInputInfoQueue, &info_last) >= 0 )
				{
					OMX_U8 *p_ring_base = p_ring_state->pRingBuffBase[VA];
					OMX_S32 start_pos = (OMX_S32)(info_first.pStartPtr - p_ring_base);
					OMX_S32 end_pos = (OMX_S32)(info_last.pEndPtr - p_ring_base);
					OMX_S32 data_size;

					if( start_pos < end_pos ) {
						data_size = end_pos - start_pos;
					} else {
						data_size = end_pos + (p_ring_state->lRingBuffSize - start_pos);
					}

					LOG_RING("[INPUTQUE_STATE] [PTS: %8ld ~ %8ld] [PTR: %ld ~ %ld] [SIZE: %ld]",
						 (OMX_S32)(info_first.llTimestamp/1000),
						 (OMX_S32)(info_last.llTimestamp/1000),
						 start_pos,
						 end_pos,
						 data_size);
				}

				LOG_RING("[RINGBUFF_STATE] [OMX_EMPTY: %ld][OMX_WRITTEN: %ld] [VPU_FILLED: %ld]",
					 p_ring_state->lEmptySpace,
					 p_ring_state->lWrittenBytes,
					 p_ring_state->lRingBuffSize - p_ring_state->lEmptySpace + p_ring_state->lWrittenBytes);

				// update ring-buffer and input queue status;
				GetCurrTimestamp(pstVDecPrivate, &temp_timestamp, OMX_FALSE, OMX_FALSE);
			}
			SET_ERROR(pstVDecPrivate, ERROR_INSUFFICIENT_BITSTREAM);
		}
		break;

	case -RETCODE_CODEC_FINISH:
		/* resolution with re-init step4 - complete flushing */
		INFO("ErrorProcess() - Codec Finish (code: %ld)", lErrorCode);
		if( CHECK_STATE(pstVDecPrivate, STATE_RESOLUTION_CHANGING_WITH_REINIT) ) {
			SET_STATE(pstVDecPrivate, STATE_READY_TO_RESET);
		}
		break;

	case ERROR_INVALID_BUFFER_STATE:
		ERROR("ErrorProcess() - Invalid Buffer State");
		if( CHECK_STATE(pstVDecPrivate, STATE_READY_TO_RESET) ) {
			SET_STATE(pstVDecPrivate, STATE_READY_TO_RESET);
		}
		break;

	case ERROR_INVALID_OUTPUT_FRAME:
		ERROR("ErrorProcess() - Invalid Output Frame");
		break;

	case ERROR_SKIP_OUTPUT_FRAME:
		break;

	default:
		if( lErrorCode < 0 ) {
			switch(lErrorCode) {
			case -RETCODE_CODEC_SPECOUT:
				ERROR("ErrorProcess() - [VDEC_ERROR] Spec out (code: %ld)", lErrorCode);
				if( !CHECK_ERROR(pstVDecPrivate, ERROR_SPEC_OUT) )
				{
					OMX_ERRORTYPE specout_type = OMX_ErrorHardware;
					SET_ERROR(pstVDecPrivate, ERROR_SPEC_OUT);

					ERROR("ErrorProcess() - [VDEC_ERROR] ProcessStatus: %ld Codec: %d\n",lProcessStatus,pstVDecPrivate->stVDecInit.m_iBitstreamFormat);
					if (lProcessStatus == PROCESS_DECODE) {
						specout_type = OMX_ErrorUnSupportedDivxHD; //OMX_ErrorUnSupportedDivxPlusHD
					}
					else if (lProcessStatus == PROCESS_OPEN) {
						if (pstVDecPrivate->stVDecInit.m_iBitstreamFormat == STD_DIV3) {
							specout_type = OMX_ErrorUnSupportedDivxHD;
						} else {
							specout_type = OMX_ErrorUnSupportedCodec;	//Sorenson H.263, Theora..
						}
					}
					else if (lProcessStatus == PROCESS_SEQUENCEINIT) {
						if (pstVDecPrivate->stVDecOutput.m_pInitialInfo->m_iErrCode == 147) { //RETCODE_MPEG4ERR_PACKEDPB
							specout_type = OMX_ErrorUnSupportedDivxHD;
						} else {
							specout_type = OMX_ErrorHardware;
						}
					}

					ERROR("ErrorProcess() - [VDEC_ERROR] Detail Reason (0x%x) ProcessStatus: %ld Codec: %d", specout_type,lProcessStatus,pstVDecPrivate->stVDecInit.m_iBitstreamFormat);
					SendEventToClient(pstVDecPrivate,
									  OMX_EventError,
									  specout_type,
									  0
									  );
				}
				break;
			case -VPU_NOT_ENOUGH_MEM:
				ERROR("ErrorProcess() - [VDEC_ERROR] Not enough memory (code: %ld)", lErrorCode);
				if( !CHECK_ERROR(pstVDecPrivate, ERROR_OUT_OF_MEMORY) ) {
					SET_ERROR(pstVDecPrivate, ERROR_OUT_OF_MEMORY);
					SendEventToClient(pstVDecPrivate,
									  OMX_EventError,
									  OMX_ErrorInsufficientResources,
									  0
									  );
				}
				break;
			case -RETCODE_INVALID_STRIDE:
				ERROR("ErrorProcess() - [VDEC_ERROR] Invalid stride (code: %ld)", lErrorCode);
				if( !CHECK_ERROR(pstVDecPrivate, ERROR_INVALID_STRIDE) ) {
					SET_ERROR(pstVDecPrivate, ERROR_INVALID_STRIDE);
					SendEventToClient(pstVDecPrivate,
									  OMX_EventError,
									  OMX_ErrorHardware,
									  0
									  );
				}
				break;
			default:
				ERROR("ErrorProcess() - [VDEC_ERROR] unknown (code: %ld)", lErrorCode);
				SendEventToClient(pstVDecPrivate,
								  OMX_EventError,
								  OMX_ErrorHardware,
								  0
								  );
				break;
			}

		}
		else {
			ERROR("ErrorProcess() - error line: %ld", lErrorCode);
			SendEventToClient(pstVDecPrivate,
							  OMX_EventError,
							  OMX_ErrorHardware,	//OMX_ErrorUndefined,
							  0
							  );
		}

		if( pInputBuffer ){
			pInputBuffer->nFilledLen = 0;
		}
		DecoderDeinit(pstVDecPrivate);

		break;
	}

	return OMX_TRUE;
}

static
void
PrepareResolutionChange(
	vdec_private_t *pstVDecPrivate
	)
{
	INFO("PREPARE RESOLUTION CHANGE");

	pstVDecPrivate->lSeqInitFailCount = 0;
	pstVDecPrivate->lInputCount       = 0;
	pstVDecPrivate->lDecodedCount     = 0;
	pstVDecPrivate->lPreviousKeyCount = 0;
	pstVDecPrivate->lKeyFrameDistance = 0;
	pstVDecPrivate->lDecodingFailCount= 0;
	pstVDecPrivate->lSkippedCount     = 0;
	pstVDecPrivate->llQueuedStartTime = -1;

	SET_STATE(pstVDecPrivate, STATE_IN_DEC_RESET_PROCESS);
	CLEAR_STATE(pstVDecPrivate, STATE_VDEC_INITIALIZED);

	if( CHECK_MODE(pstVDecPrivate, MODE_RINGBUFFER_MODE) ) {
		/* reset VPU ring-buffer state */
		ResetRingBuffer(&pstVDecPrivate->stRingBuffState, OMX_FALSE);
	}

	/* reset input/output information managers */
//	ClearInputQueue(&pstVDecPrivate->stInputInfoQueue);
	ClearDispIdxQueue(&pstVDecPrivate->stDispIdxQueue);
	ResetDispInfoManager(&pstVDecPrivate->stDispInfoMgr);
	CLEAR_STATE(pstVDecPrivate, STATE_CLEAR_DISPLAY_BUFFER);

	/* VPU close */
	DecoderDeinit(pstVDecPrivate);

	if( pstVDecPrivate->pbySequenceHeader ) {
		TCC_free(pstVDecPrivate->pbySequenceHeader);
		pstVDecPrivate->pbySequenceHeader = NULL;
	}
	pstVDecPrivate->lSeqHeaderLength = 0;
}


////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//	Component internal functions
//
//
static OMX_S32 GetBitstreamFormat(OMX_VIDEO_CODINGTYPE enCodingType, OMX_S32 lVersion);

/** internal function to set codec related parameters in the private type structure
  */
static
OMX_BOOL
SetInternalVideoParameters(
	vdec_private_t        *pstVDecPrivate
	)
{
	omx_base_video_PortType *p_inport = (omx_base_video_PortType *)pstVDecPrivate->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];
	OMX_VIDEO_CODINGTYPE coding_type = pstVDecPrivate->enVideoCodingType;

    LOG_STEP("In %s ", __func__);

	p_inport->sPortParam.format.video.eCompressionFormat = coding_type;
	p_inport->sVideoParam.eCompressionFormat = coding_type;

	switch (coding_type)
	{
	case OMX_VIDEO_CodingMPEG2:
		{
			OMX_VIDEO_PARAM_MPEG2TYPE* pst_param = pstVDecPrivate->pVideoParam;
			INFO("video coding type: OMX_VIDEO_CodingMPEG2");

			(void)strncpy(p_inport->sPortParam.format.video.cMIMEType, "video/mpeg2", sizeof("video/mpeg2"));

			if (pst_param != NULL){
				TCC_free(pst_param);
			}
			pst_param = calloc(1, sizeof(OMX_VIDEO_PARAM_MPEG2TYPE));
			pstVDecPrivate->pVideoParam = pst_param;

			if (pst_param != NULL) {
				setHeader(pst_param, sizeof(OMX_VIDEO_PARAM_MPEG2TYPE));
				pst_param->nPortIndex           = 0;
				pst_param->nPFrames             = 0;
				pst_param->nBFrames             = 0;
				pst_param->eProfile             = OMX_VIDEO_MPEG2ProfileSimple;
				pst_param->eLevel               = OMX_VIDEO_MPEG2LevelLL;
			}
		}
		break;

	case OMX_VIDEO_CodingH263:
		{
			OMX_VIDEO_PARAM_H263TYPE* pst_param = pstVDecPrivate->pVideoParam;
			INFO("video coding type: OMX_VIDEO_CodingH263");

			(void)strncpy(p_inport->sPortParam.format.video.cMIMEType, "video/h263", sizeof("video/h263"));

			if (pst_param != NULL) {
				TCC_free(pst_param);
			}
			pst_param = calloc(1, sizeof(OMX_VIDEO_PARAM_H263TYPE));
			pstVDecPrivate->pVideoParam = pst_param;

			if (pst_param != NULL)
			{
				setHeader(pst_param, sizeof(OMX_VIDEO_PARAM_H263TYPE));
				pst_param->nPortIndex             = 0;
				pst_param->nPFrames               = 0;
				pst_param->nBFrames               = 0;
				pst_param->eProfile               = OMX_VIDEO_H263ProfileBaseline;
				pst_param->eLevel                 = OMX_VIDEO_H263Level10;
				pst_param->bPLUSPTYPEAllowed      = OMX_FALSE;
				pst_param->nAllowedPictureTypes   = 0;
				pst_param->bForceRoundingTypeToZero = OMX_FALSE;
				pst_param->nPictureHeaderRepetition = 0;
				pst_param->nGOBHeaderInterval     = 0;
			}
		}
		break;

	case OMX_VIDEO_CodingMPEG4:
		{
			OMX_VIDEO_PARAM_MPEG4TYPE* pst_param = pstVDecPrivate->pVideoParam;
			INFO("video coding type: OMX_VIDEO_CodingMPEG4");

			(void)strncpy(p_inport->sPortParam.format.video.cMIMEType, "video/mpeg4", sizeof("video/mpeg4"));

			if (pst_param != NULL) {
				TCC_free(pst_param);
			}
			pst_param = calloc(1, sizeof(OMX_VIDEO_PARAM_MPEG4TYPE));
			pstVDecPrivate->pVideoParam = pst_param;

			if (pst_param != NULL)
			{
				setHeader(pst_param, sizeof(OMX_VIDEO_PARAM_MPEG4TYPE));
				pst_param->nPortIndex            = 0;
				pst_param->nSliceHeaderSpacing   = 0;
				pst_param->bSVH                  = OMX_FALSE;
				pst_param->bGov                  = OMX_FALSE;
				pst_param->nPFrames              = 0;
				pst_param->nBFrames              = 0;
				pst_param->nIDCVLCThreshold      = 0;
				pst_param->bACPred               = OMX_FALSE;
				pst_param->nMaxPacketSize        = 0;
				pst_param->nTimeIncRes           = 0;
				pst_param->eProfile              = OMX_VIDEO_MPEG4ProfileSimple;
				pst_param->eLevel                = OMX_VIDEO_MPEG4Level0;
				pst_param->nAllowedPictureTypes  = 0;
				pst_param->nHeaderExtension      = 0;
				pst_param->bReversibleVLC        = OMX_FALSE;
			}
		}
		break;

	case OMX_VIDEO_CodingMSMPEG4:
		{
			OMX_VIDEO_PARAM_MPEG4TYPE* pst_param = pstVDecPrivate->pVideoParam;
			INFO("video coding type: OMX_VIDEO_CodingMSMPEG4");

			(void)strncpy(p_inport->sPortParam.format.video.cMIMEType, "video/mpeg4", sizeof("video/mpeg4"));

			if (pst_param != NULL) {
				TCC_free(pst_param);
			}
			pst_param = calloc(1, sizeof(OMX_VIDEO_PARAM_MPEG4TYPE));
			pstVDecPrivate->pVideoParam = pst_param;

			if (pst_param != NULL)
			{
				setHeader(pst_param, sizeof(OMX_VIDEO_PARAM_MPEG4TYPE));
				pst_param->nPortIndex            = 0;
				pst_param->nSliceHeaderSpacing   = 0;
				pst_param->bSVH                  = OMX_FALSE;
				pst_param->bGov                  = OMX_FALSE;
				pst_param->nPFrames              = 0;
				pst_param->nBFrames              = 0;
				pst_param->nIDCVLCThreshold      = 0;
				pst_param->bACPred               = OMX_FALSE;
				pst_param->nMaxPacketSize        = 0;
				pst_param->nTimeIncRes           = 0;
				pst_param->eProfile              = OMX_VIDEO_MPEG4ProfileSimple;
				pst_param->eLevel                = OMX_VIDEO_MPEG4Level0;
				pst_param->nAllowedPictureTypes  = 0;
				pst_param->nHeaderExtension      = 0;
				pst_param->bReversibleVLC        = OMX_FALSE;
			}
		}
		break;

	case OMX_VIDEO_CodingWMV:
		{
			OMX_VIDEO_PARAM_WMVTYPE* pst_param = pstVDecPrivate->pVideoParam;
			INFO("video coding type: OMX_VIDEO_CodingWMV");

			(void)strncpy(p_inport->sPortParam.format.video.cMIMEType, "video/wmv", sizeof("video/wmv"));

			if (pst_param != NULL) {
				TCC_free(pst_param);
			}
			pst_param = calloc(1, sizeof(OMX_VIDEO_PARAM_WMVTYPE));
			pstVDecPrivate->pVideoParam = pst_param;

			if (pst_param != NULL) {
				setHeader(pst_param, sizeof(OMX_VIDEO_PARAM_WMVTYPE));
				pst_param->nPortIndex           = 0;
				pst_param->eFormat              = OMX_VIDEO_WMVFormat9;
			}
		}
		break;

	case OMX_VIDEO_CodingAVC:
		{
			OMX_VIDEO_PARAM_AVCTYPE* pst_param = pstVDecPrivate->pVideoParam;
			INFO("video coding type: OMX_VIDEO_CodingAVC");

			(void)strncpy(p_inport->sPortParam.format.video.cMIMEType, "video/avc", sizeof("video/avc"));

			if (pst_param != NULL) {
				TCC_free(pst_param);
			}
			pst_param = calloc(1, sizeof(OMX_VIDEO_PARAM_AVCTYPE));
			pstVDecPrivate->pVideoParam = pst_param;

			if (pst_param != NULL)
			{
				setHeader(pst_param, sizeof(OMX_VIDEO_PARAM_AVCTYPE));
				pst_param->nPortIndex             = 0;
				pst_param->nSliceHeaderSpacing    = 0;
				pst_param->bUseHadamard           = OMX_FALSE;
				pst_param->nRefFrames             = 2;
				pst_param->nPFrames               = 0;
				pst_param->nBFrames               = 0;
				pst_param->bUseHadamard           = OMX_FALSE;
				pst_param->nRefFrames             = 2;
				pst_param->eProfile               = OMX_VIDEO_AVCProfileBaseline;
				pst_param->eLevel                 = OMX_VIDEO_AVCLevel1;
				pst_param->nAllowedPictureTypes   = 0;
				pst_param->bFrameMBsOnly          = OMX_FALSE;
				pst_param->nRefIdx10ActiveMinus1  = 0;
				pst_param->nRefIdx11ActiveMinus1  = 0;
				pst_param->bEnableUEP             = OMX_FALSE;
				pst_param->bEnableFMO             = OMX_FALSE;
				pst_param->bEnableASO             = OMX_FALSE;
				pst_param->bEnableRS              = OMX_FALSE;

				pst_param->bMBAFF                 = OMX_FALSE;
				pst_param->bEntropyCodingCABAC    = OMX_FALSE;
				pst_param->bWeightedPPrediction   = OMX_FALSE;
				pst_param->nWeightedBipredicitonMode = 0;
				pst_param->bconstIpred            = OMX_FALSE;
				pst_param->bDirect8x8Inference    = OMX_FALSE;
				pst_param->bDirectSpatialTemporal = OMX_FALSE;
				pst_param->nCabacInitIdc          = 0;
				pst_param->eLoopFilterMode        = OMX_VIDEO_AVCLoopFilterDisable;
			}
		}
		break;

	case OMX_VIDEO_CodingMJPEG:
		{
			INFO("video coding type: OMX_VIDEO_CodingMJPEG");
			(void)strncpy(p_inport->sPortParam.format.video.cMIMEType, "video/x-jpeg", sizeof("video/x-jpeg"));

			if (pstVDecPrivate->pVideoParam != NULL) {
				TCC_free(pstVDecPrivate->pVideoParam);
			}
			pstVDecPrivate->pVideoParam = NULL;
		}
		break;

	case OMX_VIDEO_CodingDIVX:
		{
			INFO("video coding type: OMX_VIDEO_CodingDIVX");
			(void)strncpy(p_inport->sPortParam.format.video.cMIMEType, "video/divx", sizeof("video/divx"));

			if (pstVDecPrivate->pVideoParam != NULL) {
				TCC_free(pstVDecPrivate->pVideoParam);
			}
			pstVDecPrivate->pVideoParam = NULL;
		}
		break;

	case OMX_VIDEO_CodingFLV1:
		{
			OMX_VIDEO_PARAM_H263TYPE* pst_param = pstVDecPrivate->pVideoParam;
			INFO("video coding type: OMX_VIDEO_CodingFLV1");

			(void)strncpy(p_inport->sPortParam.format.video.cMIMEType, "video/x-flv", sizeof("video/x-flv"));

			if (pst_param != NULL) {
				TCC_free(pst_param);
			}
			pst_param = calloc(1, sizeof(OMX_VIDEO_PARAM_H263TYPE));
			pstVDecPrivate->pVideoParam = pst_param;

			if (pst_param != NULL)
			{
				setHeader(pst_param, sizeof(OMX_VIDEO_PARAM_H263TYPE));
				pst_param->nPortIndex             = 0;
				pst_param->nPFrames               = 0;
				pst_param->nBFrames               = 0;
				pst_param->eProfile               = OMX_VIDEO_H263ProfileBaseline;
				pst_param->eLevel                 = OMX_VIDEO_H263Level10;
				pst_param->bPLUSPTYPEAllowed      = OMX_FALSE;
				pst_param->nAllowedPictureTypes   = 0;
				pst_param->bForceRoundingTypeToZero = OMX_FALSE;
				pst_param->nPictureHeaderRepetition = 0;
				pst_param->nGOBHeaderInterval     = 0;
			}
		}
		break;

	case OMX_VIDEO_CodingVP8:
		{
			INFO("video coding type: OMX_VIDEO_CodingVP8");
			(void)strncpy(p_inport->sPortParam.format.video.cMIMEType, "video/x-vnd.on2.vp8", sizeof("video/x-vnd.on2.vp8"));

			if (pstVDecPrivate->pVideoParam != NULL) {
				TCC_free(pstVDecPrivate->pVideoParam);
			}
			pstVDecPrivate->pVideoParam = NULL;
		}
		break;
#if defined (TCC_VPU_4K_D2_INCLUDE)
	case OMX_VIDEO_CodingVP9:
		{
			INFO("video coding type: OMX_VIDEO_CodingVP9 ( %d )", OMX_VIDEO_CodingVP9);
			(void)strncpy(p_inport->sPortParam.format.video.cMIMEType, "video/x-vnd.on2.vp9", sizeof("video/x-vnd.on2.vp9"));

			if (pstVDecPrivate->pVideoParam != NULL) {
				TCC_free(pstVDecPrivate->pVideoParam);
			}
			pstVDecPrivate->pVideoParam = NULL;
		}
		break;
#endif
	case OMX_VIDEO_CodingMVC:
		{
			OMX_VIDEO_PARAM_AVCTYPE* pst_param = pstVDecPrivate->pVideoParam;
			INFO("video coding type: OMX_VIDEO_CodingMVC");

			(void)strncpy(p_inport->sPortParam.format.video.cMIMEType, "video/x-mvc", sizeof("video/x-mvc"));

			if (pst_param != NULL) {
				TCC_free(pst_param);
			}
			pst_param = calloc(1, sizeof(OMX_VIDEO_PARAM_AVCTYPE));
			pstVDecPrivate->pVideoParam = pst_param;

			if (pst_param != NULL)
			{
				setHeader(pst_param, sizeof(OMX_VIDEO_PARAM_AVCTYPE));
				pst_param->nPortIndex             = 0;
				pst_param->nSliceHeaderSpacing    = 0;
				pst_param->bUseHadamard           = OMX_FALSE;
				pst_param->nRefFrames             = 2;
				pst_param->nPFrames               = 0;
				pst_param->nBFrames               = 0;
				pst_param->bUseHadamard           = OMX_FALSE;
				pst_param->nRefFrames             = 2;
				pst_param->eProfile               = OMX_VIDEO_AVCProfileBaseline;
				pst_param->eLevel                 = OMX_VIDEO_AVCLevel1;
				pst_param->nAllowedPictureTypes   = 0;
				pst_param->bFrameMBsOnly          = OMX_FALSE;
				pst_param->nRefIdx10ActiveMinus1  = 0;
				pst_param->nRefIdx11ActiveMinus1  = 0;
				pst_param->bEnableUEP             = OMX_FALSE;
				pst_param->bEnableFMO             = OMX_FALSE;
				pst_param->bEnableASO             = OMX_FALSE;
				pst_param->bEnableRS              = OMX_FALSE;

				pst_param->bMBAFF                 = OMX_FALSE;
				pst_param->bEntropyCodingCABAC    = OMX_FALSE;
				pst_param->bWeightedPPrediction   = OMX_FALSE;
				pst_param->nWeightedBipredicitonMode = 0;
				pst_param->bconstIpred            = OMX_FALSE;
				pst_param->bDirect8x8Inference    = OMX_FALSE;
				pst_param->bDirectSpatialTemporal = OMX_FALSE;
				pst_param->nCabacInitIdc          = 0;
				pst_param->eLoopFilterMode        = OMX_VIDEO_AVCLoopFilterDisable;
			}
		}
		break;

#ifdef TCC_EXT_INCLUDED
	case OMX_VIDEO_CodingRV:
		{
			OMX_VIDEO_PARAM_RVTYPE* pst_param = pstVDecPrivate->pVideoParam;
			INFO("video coding type: OMX_VIDEO_CodingRV");

			(void)strncpy(p_inport->sPortParam.format.video.cMIMEType, "video/rv", sizeof("video/rv"));

			if (pst_param != NULL) {
				TCC_free(pst_param);
			}
			pst_param = calloc(1, sizeof(OMX_VIDEO_PARAM_RVTYPE));
			pstVDecPrivate->pVideoParam = pst_param;

			if (pst_param != NULL)
			{
				setHeader(pst_param, sizeof(OMX_VIDEO_PARAM_RVTYPE));

				pst_param->nPortIndex             = 0;
				pst_param->eFormat                = OMX_VIDEO_RVFormatUnused;
				pst_param->nBitsPerPixel          = 24;
				pst_param->nPaddedWidth           = 0;
				pst_param->nPaddedHeight          = 0;
				pst_param->nFrameRate             = K2Q16(OUTPUT_FRAME_RATE_DEFAULT);
				pst_param->nBitstreamFlags        = 0;
				pst_param->nBitstreamVersion      = 0;
				pst_param->nMaxEncodeFrameSize    = 0;
				pst_param->bEnablePostFilter      = OMX_FALSE;
				pst_param->bEnableTemporalInterpolation = OMX_FALSE;
				pst_param->bEnableLatencyMode     = OMX_FALSE;
			}
		}
		break;
#endif //TCC_EXT_INCLUDED

#ifdef TCC_AVS_INCLUDED
	case OMX_VIDEO_CodingAVS:
		{
			INFO("video coding type: OMX_VIDEO_CodingAVS");
			(void)strncpy(p_inport->sPortParam.format.video.cMIMEType, "video/avs-video", sizeof("video/avs-video"));

			if (pstVDecPrivate->pVideoParam != NULL) {
				TCC_free(pstVDecPrivate->pVideoParam);
			}
			pstVDecPrivate->pVideoParam = NULL;
		}
		break;
#endif //TCC_AVS_INCLUDED

#if defined (TCC_HEVC_INCLUDE) || defined (TCC_VPU_4K_D2_INCLUDE)
	case OMX_VIDEO_CodingHEVC:
		{
			INFO("video coding type: OMX_VIDEO_CodingHEVC");
			(void)strncpy(p_inport->sPortParam.format.video.cMIMEType, "video/hevc", sizeof("video/hevc"));

			if (pstVDecPrivate->pVideoParam != NULL) {
				TCC_free(pstVDecPrivate->pVideoParam);
			}
			pstVDecPrivate->pVideoParam = NULL;

#if 0
			//TODO: SetParam
			OMX_VIDEO_PARAM_HEVCTYPE* pst_param = pstVDecPrivate->pVideoParam;
			INFO("video coding type: OMX_VIDEO_CodingHEVC");

			if (pst_param) TCC_free(pst_param);
			pst_param = calloc(1, sizeof(OMX_VIDEO_PARAM_HEVCTYPE));
			pstVDecPrivate->pVideoParam = pst_param;

			if (pst_param)
			{
				setHeader(pst_param, sizeof(OMX_VIDEO_PARAM_HEVCTYPE));
				pst_param->nPortIndex             = 0;
				pst_param->nSliceHeaderSpacing    = 0;
				pst_param->bUseHadamard           = OMX_FALSE;
				pst_param->nRefFrames             = 2;
				pst_param->nPFrames               = 0;
				pst_param->nBFrames               = 0;
				pst_param->bUseHadamard           = OMX_FALSE;
				pst_param->nRefFrames             = 2;
				pst_param->eProfile               = OMX_VIDEO_HEVCProfileBaseline;
				pst_param->eLevel                 = OMX_VIDEO_HEVCLevel1;
				pst_param->nAllowedPictureTypes   = 0;
				pst_param->bFrameMBsOnly          = OMX_FALSE;
				pst_param->nRefIdx10ActiveMinus1  = 0;
				pst_param->nRefIdx11ActiveMinus1  = 0;
				pst_param->bEnableUEP             = OMX_FALSE;
				pst_param->bEnableFMO             = OMX_FALSE;
				pst_param->bEnableASO             = OMX_FALSE;
				pst_param->bEnableRS              = OMX_FALSE;

				pst_param->bMBAFF                 = OMX_FALSE;
				pst_param->bEntropyCodingCABAC    = OMX_FALSE;
				pst_param->bWeightedPPrediction   = OMX_FALSE;
				pst_param->nWeightedBipredicitonMode = 0;
				pst_param->bconstIpred            = OMX_FALSE;
				pst_param->bDirect8x8Inference    = OMX_FALSE;
				pst_param->bDirectSpatialTemporal = OMX_FALSE;
				pst_param->nCabacInitIdc          = 0;
				pst_param->eLoopFilterMode        = OMX_VIDEO_HEVCLoopFilterDisable;
			}
#endif
		}
		break;
#endif

	default:
		ERROR("Invalid video coding type: %ld", (OMX_S32)coding_type);
		return FALSE;
	}

	pstVDecPrivate->stVDecInit.m_iBitstreamFormat = GetBitstreamFormat(coding_type, pstVDecPrivate->eCodecVersion);
	if (pstVDecPrivate->stVDecInit.m_iBitstreamFormat < 0) {
		ERROR("Bitstream format not found - coding type: %ld, eCodecVersion: %ld", (OMX_S32)coding_type, (OMX_S32)pstVDecPrivate->eCodecVersion);
		return FALSE;
	}

	pstVDecPrivate->pfVDecFunc = gspfVDecList[pstVDecPrivate->stVDecInit.m_iBitstreamFormat];
	if (pstVDecPrivate->pfVDecFunc == NULL) {
		ERROR("Decoder is not exist - bitstream format: %d", pstVDecPrivate->stVDecInit.m_iBitstreamFormat);
		return FALSE;
	}

	switch (pstVDecPrivate->stVDecInit.m_iBitstreamFormat) {
#ifdef INCLUDE_WMV78_DEC
	case STD_WMV78:
		CLEAR_STATE(pstVDecPrivate, STATE_STREAM_CONVERTION_NEEDED);
		SET_STATE(pstVDecPrivate, STATE_SEQUENCE_HEADER_FOUND);
		break;
#endif

	case STD_MPEG4:
		if ((pstVDecPrivate->enVideoCodingType == OMX_VIDEO_CodingMSMPEG4)
				&& (pstVDecPrivate->eCodecVersion == OMX_VIDEO_MSMPEG4Version3)) {
			SET_STATE(pstVDecPrivate, STATE_STREAM_CONVERTION_NEEDED);
		} else {
			CLEAR_STATE(pstVDecPrivate, STATE_STREAM_CONVERTION_NEEDED);
		}
		break;

	case STD_DIV3:
	case STD_VP8:
	case STD_VC1:
		SET_STATE(pstVDecPrivate, STATE_STREAM_CONVERTION_NEEDED);
		break;
#if defined (TCC_VPU_4K_D2_INCLUDE)
	case STD_VP9:
		break;
#endif
	default:
		CLEAR_STATE(pstVDecPrivate, STATE_STREAM_CONVERTION_NEEDED);
		CLEAR_STATE(pstVDecPrivate, STATE_SEQUENCE_HEADER_FOUND);
		break;
	}

	return TRUE;
}

#define SET_IF_MATCHED(pszComponent, component_name, coding_type, ret_val) \
	{if(!strncmp((pszComponent), (component_name), sizeof((component_name)))) {(ret_val) = (coding_type);}}

static
OMX_VIDEO_CODINGTYPE
GetCodingTypeFromName(
   OMX_STRING			 pszComponentName
   )
{
	OMX_VIDEO_CODINGTYPE ret = OMX_VIDEO_CodingUnused;
	SET_IF_MATCHED(pszComponentName, VIDEO_DEC_H263_NAME,          OMX_VIDEO_CodingH263, ret);
	SET_IF_MATCHED(pszComponentName, VIDEO_DEC_SORENSON_H263_NAME, OMX_VIDEO_CodingFLV1, ret);
	SET_IF_MATCHED(pszComponentName, VIDEO_DEC_MPEG2_NAME,         OMX_VIDEO_CodingMPEG2, ret);
	SET_IF_MATCHED(pszComponentName, VIDEO_DEC_MPEG4_NAME,         OMX_VIDEO_CodingMPEG4, ret);
	SET_IF_MATCHED(pszComponentName, VIDEO_DEC_MSMPEG4_NAME,       OMX_VIDEO_CodingMSMPEG4, ret);
	SET_IF_MATCHED(pszComponentName, VIDEO_DEC_H264_NAME,          OMX_VIDEO_CodingAVC, ret);
	SET_IF_MATCHED(pszComponentName, VIDEO_DEC_DIVX_NAME,          OMX_VIDEO_CodingDIVX, ret);
	SET_IF_MATCHED(pszComponentName, VIDEO_DEC_MJPEG_NAME,         OMX_VIDEO_CodingMJPEG, ret);
	SET_IF_MATCHED(pszComponentName, VIDEO_DEC_VP8_NAME,           OMX_VIDEO_CodingVP8, ret);
#if defined (TCC_VPU_4K_D2_INCLUDE)
	SET_IF_MATCHED(pszComponentName, VIDEO_DEC_VP9_NAME,           OMX_VIDEO_CodingVP9, ret);
#endif
	SET_IF_MATCHED(pszComponentName, VIDEO_DEC_MVC_NAME,           OMX_VIDEO_CodingMVC, ret);

#ifdef TCC_EXT_INCLUDED
	SET_IF_MATCHED(pszComponentName, VIDEO_DEC_RV_NAME,            OMX_VIDEO_CodingRV, ret);
#endif
#ifdef TCC_AVS_INCLUDED
	SET_IF_MATCHED(pszComponentName, VIDEO_DEC_AVS_NAME,           OMX_VIDEO_CodingAVS, ret);
#endif
#if defined (TCC_HEVC_INCLUDE) || defined (TCC_VPU_4K_D2_INCLUDE)
	SET_IF_MATCHED(pszComponentName, VIDEO_DEC_H265_NAME,          OMX_VIDEO_CodingHEVC, ret);
#endif
#if defined(INCLUDE_WMV78_DEC) || defined(INCLUDE_WMV9_DEC)
	SET_IF_MATCHED(pszComponentName, VIDEO_DEC_WMV_NAME,           OMX_VIDEO_CodingWMV, ret);
#endif

	return ret;
}

static
OMX_VIDEO_CODINGTYPE
GetCodingTypeFromRole(
   OMX_STRING			 pszRoleName
   )
{
	OMX_VIDEO_CODINGTYPE ret = OMX_VIDEO_CodingUnused;
	SET_IF_MATCHED(pszRoleName, VIDEO_DEC_H263_ROLE,          OMX_VIDEO_CodingH263, ret);
	SET_IF_MATCHED(pszRoleName, VIDEO_DEC_SORENSON_H263_ROLE, OMX_VIDEO_CodingFLV1, ret);
	SET_IF_MATCHED(pszRoleName, VIDEO_DEC_MPEG2_ROLE,         OMX_VIDEO_CodingMPEG2, ret);
	SET_IF_MATCHED(pszRoleName, VIDEO_DEC_MPEG4_ROLE,         OMX_VIDEO_CodingMPEG4, ret);
	SET_IF_MATCHED(pszRoleName, VIDEO_DEC_MSMPEG4_ROLE,       OMX_VIDEO_CodingMSMPEG4, ret);
	SET_IF_MATCHED(pszRoleName, VIDEO_DEC_H264_ROLE,          OMX_VIDEO_CodingAVC, ret);
	SET_IF_MATCHED(pszRoleName, VIDEO_DEC_DIVX_ROLE,          OMX_VIDEO_CodingDIVX, ret);
	SET_IF_MATCHED(pszRoleName, VIDEO_DEC_MJPEG_ROLE,         OMX_VIDEO_CodingMJPEG, ret);
	SET_IF_MATCHED(pszRoleName, VIDEO_DEC_VP8_ROLE,           OMX_VIDEO_CodingVP8, ret);
#if defined (TCC_VPU_4K_D2_INCLUDE)
	SET_IF_MATCHED(pszRoleName, VIDEO_DEC_VP9_ROLE,           OMX_VIDEO_CodingVP9, ret);
#endif
	SET_IF_MATCHED(pszRoleName, VIDEO_DEC_MVC_ROLE,           OMX_VIDEO_CodingMVC, ret);

#ifdef TCC_EXT_INCLUDED
	SET_IF_MATCHED(pszRoleName, VIDEO_DEC_RV_ROLE,            OMX_VIDEO_CodingRV, ret);
#endif
#ifdef TCC_AVS_INCLUDED
	SET_IF_MATCHED(pszRoleName, VIDEO_DEC_AVS_ROLE,           OMX_VIDEO_CodingAVS, ret);
#endif
#if defined (TCC_HEVC_INCLUDE) || defined (TCC_VPU_4K_D2_INCLUDE)
	SET_IF_MATCHED(pszRoleName, VIDEO_DEC_H265_ROLE,          OMX_VIDEO_CodingHEVC, ret);
#endif
#if defined(INCLUDE_WMV78_DEC) || defined(INCLUDE_WMV9_DEC)
	SET_IF_MATCHED(pszRoleName, VIDEO_DEC_WMV_ROLE,           OMX_VIDEO_CodingWMV, ret);
#endif

	return ret;
}

static
OMX_S32
GetBitstreamFormat(
	OMX_VIDEO_CODINGTYPE  enCodingType,
	OMX_S32               lVersion
	)
{
	OMX_S32 lBitstreamFormat = -1;
	switch (enCodingType) {
	case OMX_VIDEO_CodingAVC:
		lBitstreamFormat = STD_AVC;
		break;
	case OMX_VIDEO_CodingMPEG2:
		lBitstreamFormat = STD_MPEG2;
		break;
	case OMX_VIDEO_CodingMJPEG:
		lBitstreamFormat = STD_MJPG;
		break;
	case OMX_VIDEO_CodingVP8:
		lBitstreamFormat = STD_VP8;
		break;
#if defined (TCC_VPU_4K_D2_INCLUDE)
	case OMX_VIDEO_CodingVP9:
		lBitstreamFormat = STD_VP9;
		break;
#endif
	case OMX_VIDEO_CodingMPEG4:
		lBitstreamFormat = STD_MPEG4;
		break;
	case OMX_VIDEO_CodingMSMPEG4:
		if (lVersion == OMX_VIDEO_MSMPEG4Version3) {
			lBitstreamFormat = STD_DIV3;
		} else {
			lBitstreamFormat = STD_MPEG4;
		}
		break;
	case OMX_VIDEO_CodingDIVX:
		if (lVersion == OMX_VIDEO_DIVXVersion3) {
			lBitstreamFormat = STD_DIV3;
		} else {
			lBitstreamFormat = STD_MPEG4;
		}

		break;
#ifdef TCC_EXT_INCLUDED
	case OMX_VIDEO_CodingRV:
		lBitstreamFormat = STD_EXT;
		break;
#endif

#ifdef TCC_AVS_INCLUDED
	case OMX_VIDEO_CodingAVS:
		lBitstreamFormat = STD_AVS;
		break;
#endif

#if defined (TCC_HEVC_INCLUDE) || defined (TCC_VPU_4K_D2_INCLUDE)
	case OMX_VIDEO_CodingHEVC:
		lBitstreamFormat = STD_HEVC;
		break;
#endif

	case OMX_VIDEO_CodingWMV:
		{
			OMX_S32 default_format;
			if ((lVersion == OMX_VIDEO_WMVFormat7) || (lVersion == OMX_VIDEO_WMVFormat8)) {
			#ifdef INCLUDE_WMV78_DEC
				default_format = STD_WMV78;
			#else
				default_format =  -1;
			#endif
			} else if (lVersion == OMX_VIDEO_WMVFormat9) {
			#ifdef INCLUDE_WMV9_DEC
				default_format =  STD_VC1;
			#else
				default_format =  -1;
			#endif
			} else {
			#if defined (INCLUDE_WMV78_DEC) || defined (INCLUDE_WMV9_DEC)
				default_format =  STD_VC1;
			#else
				default_format =  -1;
			#endif
			}
			lBitstreamFormat = default_format;
		}
		break;
	case OMX_VIDEO_CodingH263:
		lBitstreamFormat = STD_H263;
		break;
	case OMX_VIDEO_CodingFLV1:
		lBitstreamFormat = STD_SH263;
		break;
	case OMX_VIDEO_CodingMVC:
		lBitstreamFormat = STD_MVC;
		break;
	default:
		ERROR("BitstreamFormat Not found");
		break;
	}

	return lBitstreamFormat;
}


/** Executes all the required steps after an output buffer frame-size has changed.
*/
static inline
OMX_S32
UpdateFrameSize(
	vdec_private_t      *pstVDecPrivate
	)
{
	omx_base_video_PortType *outPort = (omx_base_video_PortType *)pstVDecPrivate->ports[OMX_BASE_FILTER_OUTPUTPORT_INDEX];
	omx_base_video_PortType *inPort = (omx_base_video_PortType *)pstVDecPrivate->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];
	vdec_initial_info_t *p_init_info = pstVDecPrivate->stVDecOutput.m_pInitialInfo;

	LOG_STEP("In %s ", __func__);

	OMX_S32 width;
	OMX_S32 height;
	OMX_S32 stride;
	OMX_S32 ret_val = 0;

#if RESOLUTION_CHANGE_WITH_CROP && SET_FULLHD_TO_PORT
#ifdef SET_FRAMEBUFFER_INTO_MAX
	if( IsSupportMaxFramebufferMode(pstVDecPrivate) ) {
		width = pstVDecPrivate->lPicWidthMax;
		height = pstVDecPrivate->lPicHeightMax;
		stride = pstVDecPrivate->lPicWidthMax;
	}
	else
#endif
#endif
	{
		if( p_init_info != NULL ) {
#if AVC_CROP_APPLIED_TO_PORT
			pic_crop_t avc_crop = p_init_info->m_iPicCrop;
			width  = (p_init_info->m_iPicWidth - avc_crop.m_iCropLeft - avc_crop.m_iCropRight);
			height = (p_init_info->m_iPicHeight - avc_crop.m_iCropBottom - avc_crop.m_iCropTop);
#else
			width  = p_init_info->m_iPicWidth;
			height = p_init_info->m_iPicHeight;
#endif

#if defined(TCC_HEVC_INCLUDE) || defined (TCC_VPU_4K_D2_INCLUDE)
			if (pstVDecPrivate->stVDecInit.m_iBitstreamFormat == STD_HEVC) {
				stride = ((p_init_info->m_iPicWidth+31) >> 5) << 5;
			} else
#endif
#if defined (TCC_VPU_4K_D2_INCLUDE)
			if (pstVDecPrivate->stVDecInit.m_iBitstreamFormat == STD_VP9) {
				stride = ((p_init_info->m_iPicWidth+31) >> 5) << 5;
			} else
#endif
			{
				stride = ((p_init_info->m_iPicWidth+15) >> 4) << 4;
			}
		}
		else {
			width = inPort->sPortParam.format.video.nFrameWidth;
			height = inPort->sPortParam.format.video.nFrameHeight;
			stride = width;
		}

		width += width&1;
		height += height&1;
	}

	outPort->sPortParam.format.video.nFrameWidth = width;
	outPort->sPortParam.format.video.nFrameHeight = height;
	outPort->sPortParam.format.video.nStride = stride;

	switch(outPort->sVideoParam.eColorFormat) {
	case OMX_COLOR_FormatYUV420Planar:
	case OMX_COLOR_FormatYUV420SemiPlanar:
		if ((width != 0) && (height != 0)) {
			outPort->sPortParam.nBufferSize = width * height * 3/2;
		}
		break;
	case OMX_COLOR_FormatYUV420PlanarTc:
	case OMX_COLOR_FormatYUV420SemiPlanarTc:
			outPort->sPortParam.nBufferSize = sizeof(tcc_video_out_info);
		break;
	default:
		ERROR("Output format is not supported (eColorFormat: 0x%08X", outPort->sVideoParam.eColorFormat);
		ret_val = -1;
	}

	return ret_val;
}

#define USERDATA_MAX	(16)
static
OMX_S32
GetUserDataLength( OMX_U8 *pbyUserData )
{
	OMX_S32 data_cnt;
	OMX_U8 *p_sizedata = pbyUserData;
	OMX_S32 data_length;
	OMX_S32 total_length = 8 * 17;
	OMX_S32 i;

	p_sizedata = pbyUserData;
	data_cnt = SHIFT_AND_MERGE(p_sizedata[0],p_sizedata[1]); //(p_sizedata[0] << 8) | p_sizedata[1];
	if( data_cnt > USERDATA_MAX ) {
		total_length = 0;
	} else {
		p_sizedata += 8;
		for (i = 0; i < data_cnt; i++) {
			data_length = (OMX_S32)SHIFT_AND_MERGE(p_sizedata[2],p_sizedata[3]);	//(((OMX_U32)p_sizedata[2] << 8) | p_sizedata[3]);
			data_length = (data_length+7)/8*8;	// 20160407
			total_length += data_length;
			p_sizedata += 8;
		}
	}

	return total_length;
}

static
void
SendUserData(
	vdec_private_t  *pstVDecPrivate,
	OMX_U8          *pbyUserData,
	OMX_S32          lDataLength,
	OMX_TICKS        llTimestamp
	)
{
	OMX_S32 data_cnt;
	OMX_U8 *p_sizedata = pbyUserData;
	OMX_U8 *p_realdata = pbyUserData + (8 * 17);
	OMX_S32 data_length;
	OMX_S32 i;
	OMX_EVENT_USERDATAAVAILABLE data;
	setHeader(&data, sizeof(OMX_EVENT_USERDATAAVAILABLE));
	data.nTimestamp = llTimestamp;

	p_sizedata = pbyUserData;
	data_cnt = SHIFT_AND_MERGE(p_sizedata[0],p_sizedata[1]); //(p_sizedata[0] << 8) | p_sizedata[1];

	if( data_cnt <= USERDATA_MAX )
	{
		p_sizedata += 8;
		for (i = 0; i < data_cnt; i++)
		{
			data_length = (OMX_S32)SHIFT_AND_MERGE(p_sizedata[2],p_sizedata[3]); //(((OMX_U32)p_sizedata[2] << 8) | p_sizedata[3]);
			data_length = (data_length+7)/8*8;	// 20160407
			data.nDataLength = data_length;
			data.pUserData = p_realdata;
			SendEventToClient2(pstVDecPrivate,
							  OMX_EventUserDataAvailable,
							  0, 0,
							  &data);

			p_sizedata += 8;
			p_realdata += data_length;
		}
	}
}


static OMX_S32 OMXVideoDecode_Init(vdec_private_t *p_private, OMX_BUFFERHEADERTYPE *pInputBuffer, OMX_BUFFERHEADERTYPE *pOutputBuffer)
{
	OMX_S32 ret, ret_value = -1;

	if( p_private->pbyTempInputBuff == NULL ) {
		omx_base_video_PortType *p_port = (omx_base_video_PortType *)p_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];
		p_private->pbyTempInputBuff = (unsigned char*) TCC_malloc(p_port->sPortParam.nBufferSize);
		if( p_private->pbyTempInputBuff == NULL ) {
			ERROR("out of memory - temporary input buffer allocation failed");
			return ret_value;
		}
	}

	if( CHECK_MODE(p_private, MODE_RINGBUFFER_MODE) == 0UL ) {
		if ((p_private->stVDecInit.m_iBitstreamFormat == STD_AVC) && (CHECK_MODE(p_private, MODE_SLICE_MERGING) != 0UL)
			&& (CHECK_STATE(p_private, STATE_VDEC_BUFFER_PROTECTION) == 0UL)) {
			SET_STATE(p_private, STATE_SLICE_COUNTING);
			CLEAR_STATE(p_private, STATE_SLICE_COUNTED);
			CLEAR_STATE(p_private, STATE_SLICE_MERGING);
		}
	}

	if(CHECK_ERROR(p_private, ERROR_INPUT_BUFFER_REMAINNING) != 0UL ) {
		p_private->pbyVDecInputPtr = NULL;
		p_private->lVDecInputLength = 0;
		CLEAR_ERROR(p_private, ERROR_INPUT_BUFFER_REMAINNING);
	}

	if (CHECK_STATE(p_private, STATE_STREAM_CONVERTION_NEEDED) != 0UL) {
		if (ConvertBitstream(p_private, pInputBuffer->pBuffer+pInputBuffer->nOffset, pInputBuffer->nFilledLen, OMX_TRUE) == OMX_TRUE) {
			p_private->pbyVDecInputPtr = p_private->pbyTempInputBuff;
			p_private->lVDecInputLength = p_private->lTempInputLength;
			//pInputBuffer->nFilledLen = 0;	//DEL
		}
		else {
			p_private->pbyVDecInputPtr = NULL;
			p_private->lVDecInputLength = 0;
		}
	}

	if( CHECK_STATE(p_private, STATE_STREAM_CONVERTION_NEEDED) != 0UL)
	{
		if (CHECK_STATE(p_private, STATE_SEQUENCE_HEADER_FOUND) == 0UL) {
			OMX_S32 seqhead_pos = ScanSequenceHeader(p_private,
													 p_private->pbyTempInputBuff,
													 p_private->lTempInputLength,
													 p_private->stVDecInit.m_iBitstreamFormat);

			if( seqhead_pos < 0 ) {
				p_private->lSeqInitFailCount++;
				if( p_private->lSeqInitFailCount >= p_private->lSeqInitFailMax ) {
					ERROR("Sequence header is not found! (SEARCH-CNT: %ld / %ld)",
							p_private->lSeqInitFailCount,
							p_private->lSeqInitFailMax);
					(void)ErrorProcess(p_private, pInputBuffer, pOutputBuffer, __LINE__,PROCESS_NONE);
				}
				pInputBuffer->nFilledLen = 0;
				return ret_value;
			}

			SET_STATE(p_private, STATE_SEQUENCE_HEADER_FOUND);

			if( CHECK_MODE(p_private, MODE_RINGBUFFER_MODE) != 0UL) {
				DecideLimitTimeDiffForFeeding(p_private, OUTPUT_FRAME_RATE_DEFAULT);
			}
		}
	}
	else
	{
		if (CHECK_STATE(p_private, STATE_SEQUENCE_HEADER_FOUND) == 0UL) {
			OMX_S32 seqhead_pos = ScanSequenceHeader(p_private,
													 pInputBuffer->pBuffer+pInputBuffer->nOffset,
													 pInputBuffer->nFilledLen,
													 p_private->stVDecInit.m_iBitstreamFormat);
			if( seqhead_pos < 0 ) {
				p_private->lSeqInitFailCount++;
				if( p_private->lSeqInitFailCount >= p_private->lSeqInitFailMax ) {
					ERROR("Sequence header is not found! (SEARCH-CNT: %ld / %ld)",
							p_private->lSeqInitFailCount,
							p_private->lSeqInitFailMax);
					(void)ErrorProcess(p_private, pInputBuffer, pOutputBuffer, __LINE__,PROCESS_NONE);
				}
				pInputBuffer->nFilledLen = 0;
				return ret_value;
			}

			pInputBuffer->nOffset += seqhead_pos;
			pInputBuffer->nFilledLen -= seqhead_pos;
			p_private->lSeqInitFailCount = 0;

			SET_STATE(p_private, STATE_SEQUENCE_HEADER_FOUND);

			if( CHECK_MODE(p_private, MODE_RINGBUFFER_MODE) != 0UL) {
				DecideLimitTimeDiffForFeeding(p_private, OUTPUT_FRAME_RATE_DEFAULT);
			}
		}

		if (CHECK_MODE(p_private, MODE_RINGBUFFER_MODE) == 0UL) {
			if ((CHECK_STATE(p_private, STATE_SEQHEAD_ATTACHMENT_NEEDED) != 0UL)
				&& (CHECK_STATE(p_private, STATE_VDEC_BUFFER_PROTECTION) == 0UL)) {
				if((p_private->pbySequenceHeader != NULL) && (p_private->lSeqHeaderLength > 0)) {
					(void)memcpy((void*)p_private->pbyTempInputBuff, (void*)p_private->pbySequenceHeader, p_private->lSeqHeaderLength);
					(void)memcpy((void*)(p_private->pbyTempInputBuff+p_private->lSeqHeaderLength), (void*)(pInputBuffer->pBuffer+pInputBuffer->nOffset), pInputBuffer->nFilledLen);
					p_private->lTempInputLength = p_private->lSeqHeaderLength + (OMX_S32)pInputBuffer->nFilledLen;
					p_private->pbyVDecInputPtr = p_private->pbyTempInputBuff;
					p_private->lVDecInputLength = p_private->lTempInputLength;
					//pInputBuffer->nFilledLen = 0;	//DEL

					INFO("Sequence header attached");
				}
			}
		}
	}

	/* Decoder Initialize */
	if( CHECK_STATE(p_private, STATE_DECODER_OPENED) == 0UL)
	{
		omx_base_video_PortType *p_outport = (omx_base_video_PortType *)p_private->ports[OMX_BASE_FILTER_OUTPUTPORT_INDEX];
		if( p_outport->sVideoParam.eColorFormat == OMX_COLOR_FormatYUV420PlanarTc || p_outport->sVideoParam.eColorFormat == OMX_COLOR_FormatYUV420SemiPlanarTc ) {
			SET_MODE(p_private, MODE_PHYSICAL_ADDRESS_OUTPUT);
		}
		else {
			CLEAR_MODE(p_private, MODE_PHYSICAL_ADDRESS_OUTPUT);
		}

		if( (ret = DecoderInit(p_private)) < 0 ) {
			(void)ErrorProcess(p_private, pInputBuffer, pOutputBuffer, ret, PROCESS_OPEN);
			return ret_value;
		}
	}

	if( CHECK_MODE(p_private, MODE_RINGBUFFER_MODE) != 0UL) {
		/* resolution with re-init step6 - re-init (restore ring-buffer data) */
		if( (CHECK_STATE(p_private, STATE_RESOLUTION_CHANGING_WITH_REINIT) != 0UL) && p_private->stRingBuffState.lBackupStartOffset >= 0 ) {
			if( RestoreRingBuffer(p_private) == OMX_FALSE ) {
				(void)ErrorProcess(p_private, pInputBuffer, pOutputBuffer, __LINE__,PROCESS_NONE);
				return ret_value;
			}
		}

		/* Fill ring-buffer with sequence header */
		if( (p_private->lInputCount == 0) && (p_private->lSeqHeaderLength > 0) ) {
			if( FeedDecoder(p_private, NULL) == FEED_FAILED ) {	//NULL: from sequence header buffer
				(void)ErrorProcess(p_private, pInputBuffer, pOutputBuffer, __LINE__,PROCESS_NONE);
				return ret_value;
			}
		}

		/* Fill ring-buffer with input data */
		if( CHECK_STATE(p_private, STATE_IN_DEC_RESET_PROCESS) == 0UL) {
			if( ret = FeedDecoder(p_private, pInputBuffer) ) {
				if( ret == FEED_FAILED ) {
					(void)ErrorProcess(p_private, pInputBuffer, pOutputBuffer, __LINE__,PROCESS_NONE);
				}
				return ret_value; //NEED_MORE_DATA
			}
		}
	}
	else {
#if defined(TC_SECURE_MEMORY_COPY)
		if (CHECK_STATE(p_private, STATE_VDEC_BUFFER_PROTECTION) != 0UL)
		{
			p_private->stVDecInput.m_pInp[PA] = (unsigned int)pInputBuffer->pBuffer;
			//p_private->stVDecInput.m_pInp[VA] = vpu_getBitstreamBufAddr(VA, p_private->pVDecInstance);
			p_private->stVDecInput.m_iInpLen  = pInputBuffer->nFilledLen;
		}
		else
#endif
		{
			if ((p_private->pbyVDecInputPtr != NULL) && (p_private->lVDecInputLength > 0)) {
				p_private->stVDecInput.m_pInp[PA] = p_private->stVDecInput.m_pInp[VA] = p_private->pbyVDecInputPtr;
				p_private->stVDecInput.m_iInpLen  = p_private->lVDecInputLength;
			}
			else {
				p_private->stVDecInput.m_pInp[PA] = p_private->stVDecInput.m_pInp[VA] = pInputBuffer->pBuffer + pInputBuffer->nOffset;
				p_private->stVDecInput.m_iInpLen  = pInputBuffer->nFilledLen;
			}
		}
	}

	/* Sequence header init */
	if( (ret = DecoderSeqInit(p_private)) < 0 ) {
		(void)ErrorProcess(p_private, pInputBuffer, pOutputBuffer, ret, PROCESS_SEQUENCEINIT);
		return ret_value;
	}

	if( ret == RETRY_WITH_MORE_DATA ) {
		#if defined(TC_SECURE_MEMORY_COPY)
		if(CHECK_STATE(p_private, STATE_VDEC_BUFFER_PROTECTION))
		{
			pInputBuffer->nFilledLen = 0;
			return ret_value;
		}
		#endif
		if( p_private->pbySequenceHeader != NULL) {
			free(p_private->pbySequenceHeader);
			p_private->pbySequenceHeader = NULL;
			p_private->lSeqHeaderLength = 0;
		}
		if (p_private->stVDecInput.m_pInp[VA] == p_private->pbyVDecInputPtr) {
			p_private->pbyVDecInputPtr = NULL;
			p_private->lVDecInputLength = 0;
		}
		else if (p_private->stVDecInput.m_pInp[VA] == (pInputBuffer->pBuffer+pInputBuffer->nOffset)) {
			pInputBuffer->nFilledLen = 0;
		}

		return ret_value;
	}

	/* set init flags */
	SET_STATE(p_private, STATE_VDEC_INITIALIZED);

	CLEAR_STATE(p_private, STATE_IN_DEC_RESET_PROCESS);

	/* clear error flags */
	p_private->ulErrorFlags = 0;

	/* Check port configuration info. change */
	if( ret = CheckPortConfigChange(p_private) )
	{
		if( ret == MAX_RESOLUTION_EXCEEDED ) {
			(void)ErrorProcess(p_private, pInputBuffer, pOutputBuffer, __LINE__,PROCESS_NONE);
			return ret_value;
		}
		if( ret > 0 ) {
			if( ret & PORT_AR_CHANGE_NEEDED ){
				SendEventToClient(p_private,
								  OMX_EventPortSettingsChanged,
								  OMX_IndexConfigCommonScale,
								  OMX_DirOutput);
			}
			if( ret & PORT_CROP_CHANGE_NEEDED ) {
				SendEventToClient(p_private,
								  OMX_EventPortSettingsChanged,
								  OMX_IndexConfigCommonOutputCrop,
								  OMX_DirOutput);
			}
			if( ret & PORT_RECONFIGURATION_NEEDED ) {
				SendEventToClient(p_private,
								  OMX_EventPortSettingsChanged,
								  0,
								  OMX_DirOutput);
			}
			return ret_value;
		}
	}
	return 0;
}

static OMX_S32 OMXVideoDecode_FeedInputData(vdec_private_t *p_private, OMX_BUFFERHEADERTYPE *pInputBuffer, OMX_BUFFERHEADERTYPE *pOutputBuffer, OMX_U32 input_flags)
{
	OMX_S32 ret = 0;
	if( CHECK_MODE(p_private, MODE_RINGBUFFER_MODE) != 0UL) {
		if( p_private->bEventFlush == OMX_TRUE ) {
			pInputBuffer->nFilledLen = 0;
			p_private->bEventFlush = OMX_FALSE;
		}

		/* resolution with re-init step3 - flush delayed output frame (ring-buffer mode) */
		if( (CHECK_STATE(p_private, STATE_RESOLUTION_CHANGING_WITH_REINIT) == 0UL) && (pInputBuffer->nFilledLen > 0UL)) {
			if( ret = FeedDecoder(p_private, pInputBuffer) ) {
				if( ret == FEED_FAILED ) {
					(void)ErrorProcess(p_private, pInputBuffer, pOutputBuffer, __LINE__,PROCESS_NONE);
				}
				ret = (-1); //FEED_MORE_DATA
			}
			//FEED_COMPLETE
		}
	}
	else {
		if (p_private->pbyVDecInputPtr == NULL)
		{
			OMX_BOOL seqhead_attach = OMX_FALSE;

			/* resolution with crop step0 - attach sequence header */
			if( CHECK_STATE(p_private, STATE_RESOLUTION_CHANGING_WITH_CROP) != 0UL) {
				seqhead_attach = OMX_TRUE;
				CLEAR_STATE(p_private, STATE_RESOLUTION_CHANGING_WITH_CROP);
			}

			if (CHECK_STATE(p_private, STATE_STREAM_CONVERTION_NEEDED) != 0UL) {	//avcc
				if (ConvertBitstream(p_private, pInputBuffer->pBuffer+pInputBuffer->nOffset, pInputBuffer->nFilledLen, seqhead_attach) == OMX_TRUE) {
					p_private->pbyVDecInputPtr = p_private->pbyTempInputBuff;
					p_private->lVDecInputLength = p_private->lTempInputLength;
				}
				else {
					p_private->pbyVDecInputPtr = pInputBuffer->pBuffer+pInputBuffer->nOffset;
					p_private->lVDecInputLength = pInputBuffer->nFilledLen;
				}
			}
			else {
				if( CHECK_STATE(p_private, STATE_SLICE_COUNTING|STATE_SLICE_MERGING) != 0UL) {
					(void)memcpy((void*)(p_private->pbyTempInputBuff+p_private->lTempInputOffset), (void*)(pInputBuffer->pBuffer+pInputBuffer->nOffset), pInputBuffer->nFilledLen);
					p_private->lTempInputLength = p_private->lTempInputOffset+pInputBuffer->nFilledLen;

					p_private->pbyVDecInputPtr = p_private->pbyTempInputBuff;
					p_private->lVDecInputLength = p_private->lTempInputLength;
				}
				else {
					if( (seqhead_attach > 0) && (CHECK_STATE(p_private, STATE_SEQHEAD_ATTACHMENT_NEEDED) != 0UL) ) {
						if((p_private->pbySequenceHeader != NULL) && (p_private->lSeqHeaderLength > 0)) {
							(void)memcpy((void*)p_private->pbyTempInputBuff, (void*)p_private->pbySequenceHeader, p_private->lSeqHeaderLength);
							p_private->lTempInputLength = p_private->lSeqHeaderLength;

							INFO("Sequence header attached");
						}

						(void)memcpy((void*)(p_private->pbyTempInputBuff+p_private->lTempInputLength), (void*)(pInputBuffer->pBuffer+pInputBuffer->nOffset), pInputBuffer->nFilledLen);
						p_private->lTempInputLength += pInputBuffer->nFilledLen;

						p_private->pbyVDecInputPtr = p_private->pbyTempInputBuff;
						p_private->lVDecInputLength = p_private->lTempInputLength;
					}
					else {
						p_private->pbyVDecInputPtr = pInputBuffer->pBuffer+pInputBuffer->nOffset;
						p_private->lVDecInputLength = pInputBuffer->nFilledLen;
					}
				}
			}

			if( CHECK_STATE(p_private, STATE_SLICE_COUNTING) != 0UL ) {
				if( (p_private->lSliceCount > 0) &&
					IsAvcFrameStart(p_private->pbyTempInputBuff+p_private->lTempInputOffset, p_private->lTempInputLength-p_private->lTempInputOffset) )
				{
					p_private->lTempInputLength -= p_private->lTempInputOffset;
					(void)memcpy(p_private->pbyTempInputBuff, p_private->pbyTempInputBuff+p_private->lTempInputOffset, p_private->lTempInputLength);
					p_private->lTempInputOffset = 0;
					p_private->lSliceCount = 0;
					p_private->pbyVDecInputPtr = p_private->pbyTempInputBuff;
					p_private->lVDecInputLength = p_private->lTempInputLength;
				}
			}

			if( CHECK_STATE(p_private, STATE_SLICE_MERGING) != 0UL) {
				p_private->lSliceCount++;
				if( p_private->lSliceCount < p_private->lNumberOfSlice ) {
					p_private->lTempInputOffset = p_private->lVDecInputLength;
					pInputBuffer->nFilledLen = 0;
					p_private->pbyVDecInputPtr = NULL;
					ret = (-1);
				}
				else {
					p_private->lTempInputOffset = 0;
					p_private->lSliceCount = 0;
				}
			}
		}

		if (ret == 0) {
			/* resolution with re-init step3 - flush delayed output frame (fileplay mode) */
			if( (CHECK_STATE(p_private, STATE_RESOLUTION_CHANGING_WITH_REINIT) == 0UL)
				&& (p_private->lVDecInputLength > 0) )
			{
			    p_private->stVDecInput.m_pInp[PA] = p_private->stVDecInput.m_pInp[VA] = p_private->pbyVDecInputPtr;
			    p_private->stVDecInput.m_iInpLen  = p_private->lVDecInputLength;
			}
			else if((input_flags & OMX_BUFFERFLAG_EOS) != 0UL)
			{
			    p_private->stVDecInput.m_iInpLen  = pInputBuffer->nFilledLen;
			}

#if defined(TC_SECURE_MEMORY_COPY)
			if (CHECK_STATE(p_private, STATE_VDEC_BUFFER_PROTECTION) != 0UL)
			{
			    p_private->stVDecInput.m_pInp[PA] = p_private->stVDecInput.m_pInp[VA] = pInputBuffer->pBuffer;
			    p_private->stVDecInput.m_iInpLen  = pInputBuffer->nFilledLen;
			}
#endif
		}
	}
	return ret;
}
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//  OpenMax main functions
//
//
static
OMX_ERRORTYPE
omx_videodec_component_Initialize(
	OMX_COMPONENTTYPE *openmaxStandComp);

static
OMX_ERRORTYPE
omx_videodec_component_Constructor(
	OMX_COMPONENTTYPE *openmaxStandComp,
	OMX_STRING cComponentName);

static
OMX_ERRORTYPE
omx_videodec_component_Destructor(
	OMX_COMPONENTTYPE *openmaxStandComp);

static
OMX_ERRORTYPE
omx_videodec_component_Deinitialize(
	OMX_COMPONENTTYPE *openmaxStandComp);

static
OMX_ERRORTYPE
omx_videodec_component_MessageHandler(
	OMX_COMPONENTTYPE           *openmaxStandComp,
	internalRequestMessageType  *message);

static
void
omx_videodec_component_BufferMgmtCallback(
  OMX_COMPONENTTYPE *openmaxStandComp,
  OMX_BUFFERHEADERTYPE* inputbuffer,
  OMX_BUFFERHEADERTYPE* outputbuffer);

static
OMX_ERRORTYPE
omx_videodec_component_GetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_INOUT OMX_PTR ComponentParameterStructure);

static
OMX_ERRORTYPE
omx_videodec_component_SetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_IN  OMX_PTR ComponentParameterStructure);

static
OMX_ERRORTYPE
omx_videodec_component_ComponentRoleEnum(
  OMX_IN OMX_HANDLETYPE hComponent,
  OMX_OUT OMX_U8 *cRole,
  OMX_IN OMX_U32 nIndex);

static
OMX_ERRORTYPE
omx_videodec_component_GetConfig(
  OMX_HANDLETYPE hComponent,
  OMX_INDEXTYPE nIndex,
  OMX_PTR pComponentConfigStructure);

static
OMX_ERRORTYPE
omx_videodec_component_SetConfig(
  OMX_HANDLETYPE hComponent,
  OMX_INDEXTYPE nIndex,
  OMX_PTR pComponentConfigStructure);

static
OMX_ERRORTYPE
omx_videodec_component_GetExtensionIndex(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_STRING cParameterName,
  OMX_OUT OMX_INDEXTYPE* pIndexType);


/** The Constructor of the video decoder component
  * @param openmaxStandComp the component handle to be constructed
  * @param cComponentName is the name of the constructed component
  */
static
OMX_ERRORTYPE
omx_videodec_component_Constructor(
	OMX_COMPONENTTYPE  *openmaxStandComp,
	OMX_STRING          cComponentName
	)
{
	OMX_ERRORTYPE eError = OMX_ErrorNone;
	vdec_private_t* p_private;
	omx_base_video_PortType *inPort,*outPort;
	OMX_VIDEO_CODINGTYPE coding_type = OMX_VIDEO_CodingUnused;
	OMX_U32 i;

    LOG_STEP("In %s ", __func__);

	/** First of all, we fine the video coding type of the component */
	coding_type = GetCodingTypeFromName(cComponentName);
	if( coding_type == OMX_VIDEO_CodingUnused ) {
		eError = OMX_ErrorInvalidComponentName;
		goto CONSTRUCT_FAILED;
	}

	/** Allocate component private */
	if (openmaxStandComp->pComponentPrivate == NULL) {
		if ((p_private = calloc(1, sizeof(vdec_private_t))) == NULL) {
			ERROR("out of memory - component private allocation failed");
			eError = OMX_ErrorInsufficientResources;
			goto CONSTRUCT_FAILED;
		}
		openmaxStandComp->pComponentPrivate = p_private;
	} else {
		ERROR("component private already allocated");
		p_private = openmaxStandComp->pComponentPrivate;
	}

	if (p_private->pstCodecSyncSem == NULL) {
		if ((p_private->pstCodecSyncSem = calloc(1, sizeof(tsem_t))) == NULL) {
			ERROR("out of memory - semaphore allocation failed");
			eError = OMX_ErrorInsufficientResources;
			goto CONSTRUCT_FAILED;
		}
		tsem_init(p_private->pstCodecSyncSem, 0);
	}

	/** Init component members */
	SET_STATE(p_private, STATE_WAIT_RESOURCE_INIT);
	SET_STATE(p_private, STATE_FRAME_RATE_UPDATE);
	SET_MODE(p_private, MODE_DECODING_ERROR_REPORTING);
	//SET_MODE(p_private, MODE_DECODED_KEYFRAME_OUTPUT);

	/** set parameters */
	p_private->stScaleFactor.xWidth 	= 0x10000;
	p_private->stScaleFactor.xHeight 	= 0x10000;

	p_private->llQueuedStartTime	= -1;
	p_private->llFeedMinTimeDiff    = FEED_LIMIT_TIMEDIFF_INIT;
	p_private->llFeedMaxTimeDiff    = FEED_LIMIT_TIMEDIFF_INIT*2;

#if SINGLE_FRAME_INPUT_ENABLE
	CLEAR_STATE(p_private, STATE_FRAME_UNIT_INPUT);
#endif

	/** output fail counting (after I-frame searching) */
	p_private->lDecodingFailCount   = 0;
	p_private->lSkippedCount        = 0;
	p_private->lSkipMax             = VDEC_FRAME_SKIP_MAX;

#if 0
	/** decoded frame skip counting after I-frame searching */
	p_private->lDecFrameSkipCount   = 0;
	p_private->lDecFrameSkipMax     = DECODE_ONLY_SKIP_MAX;
	p_private->lFrameSkipErrorCount = 0;
#endif

    /** Secure Video Path **/
	CLEAR_STATE(p_private, STATE_VDEC_BUFFER_PROTECTION);

	/** decoder ring-buffer state */
	p_private->stRingBuffState.lBackupStartOffset = -1;

	/** reset MVC base-view address */
	p_private->lMVCBaseViewIndex = -1;
	p_private->pMVCBaseView[0] = NULL;
	p_private->pMVCBaseView[1] = NULL;
	p_private->pMVCBaseView[2] = NULL;

	/** seqinit/decoding/output fail counting */
	p_private->lSeqInitFailMax = SEQ_INIT_FAIL_MAX_INIT;
	p_private->lDecodingFailMax = DECODING_FAIL_MAX_INIT;

#ifdef TCC_VP9_INCLUDE
	if (p_private->stVDecInit.m_iBitstreamFormat == STD_VP9) {
		p_private->lSeqInitFailMax = 10;
	}
#endif

	/** Init component base */
	eError = omx_base_filter_Constructor(openmaxStandComp, cComponentName);

	/** Allocate Ports and call port constructor. */
	p_private->sPortTypesParam[OMX_PortDomainVideo].nStartPortNumber = 0;
	p_private->sPortTypesParam[OMX_PortDomainVideo].nPorts = 2;

	if (p_private->ports == NULL)
	{
		if ((p_private->ports = calloc(2, sizeof(omx_base_PortType *))) == NULL) {
			eError = OMX_ErrorInsufficientResources;
			goto CONSTRUCT_FAILED;
		}

		if ((p_private->ports[0] = calloc(1, sizeof(omx_base_video_PortType))) == NULL) {
			eError = OMX_ErrorInsufficientResources;
			goto CONSTRUCT_FAILED;
		}

		if ((p_private->ports[1] = calloc(1, sizeof(omx_base_video_PortType))) == NULL) {
			eError = OMX_ErrorInsufficientResources;
			goto CONSTRUCT_FAILED;
		}

		base_video_port_Constructor(openmaxStandComp, &p_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX], 0, OMX_TRUE);
		base_video_port_Constructor(openmaxStandComp, &p_private->ports[OMX_BASE_FILTER_OUTPUTPORT_INDEX], 1, OMX_FALSE);
	}

	/** here we can override whatever defaults the base_component constructor set
	  * e.g. we can override the function pointers in the private struct
	  */
	openmaxStandComp->SetParameter      = omx_videodec_component_SetParameter;
	openmaxStandComp->GetParameter      = omx_videodec_component_GetParameter;
	openmaxStandComp->SetConfig         = omx_videodec_component_SetConfig;
	openmaxStandComp->GetConfig         = omx_videodec_component_GetConfig;
	openmaxStandComp->ComponentRoleEnum = omx_videodec_component_ComponentRoleEnum;
	openmaxStandComp->GetExtensionIndex = omx_videodec_component_GetExtensionIndex;
	p_private->messageHandler           = omx_videodec_component_MessageHandler;
	p_private->destructor               = omx_videodec_component_Destructor;
	p_private->BufferMgmtCallback       = omx_videodec_component_BufferMgmtCallback;

	/** Domain specific section for the ports.
	  * first we set the parameter common to both formats
	  */

	/** common parameters related to input port */
	inPort = (omx_base_video_PortType *)p_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];
#if defined (TCC_HEVC_INCLUDE) || defined (TCC_VPU_4K_D2_INCLUDE)
	if (coding_type == OMX_VIDEO_CodingHEVC){
		inPort->sPortParam.nBufferSize               = HEVC_INPUT_BUFFER_SIZE;
		inPort->sPortParam.nBufferCountMin           = HEVC_INPUT_BUFFER_COUNT_MIN;
		inPort->sPortParam.nBufferCountActual        = HEVC_INPUT_BUFFER_COUNT_MIN;
		inPort->sPortParam.format.video.xFramerate   = K2Q16(OUTPUT_FRAME_RATE_DEFAULT);
		inPort->sPortParam.format.video.nFrameWidth  = HEVC_OUTPUT_WIDTH_MAX;
		inPort->sPortParam.format.video.nFrameHeight = HEVC_OUTPUT_HEIGHT_MAX;
	} else
#endif
#if defined (TCC_VPU_4K_D2_INCLUDE)
	if (coding_type == OMX_VIDEO_CodingVP9){
		inPort->sPortParam.nBufferSize               = VP9_INPUT_BUFFER_SIZE;
		inPort->sPortParam.nBufferCountMin           = VP9_INPUT_BUFFER_COUNT_MIN;
		inPort->sPortParam.nBufferCountActual        = VP9_INPUT_BUFFER_COUNT_MIN;
		inPort->sPortParam.format.video.xFramerate   = K2Q16(OUTPUT_FRAME_RATE_DEFAULT);
		inPort->sPortParam.format.video.nFrameWidth  = VP9_OUTPUT_WIDTH_MAX;
		inPort->sPortParam.format.video.nFrameHeight = VP9_OUTPUT_HEIGHT_MAX;
	} else
#endif
	{
		inPort->sPortParam.nBufferSize               = INPUT_BUFFER_SIZE;
		inPort->sPortParam.nBufferCountMin           = INPUT_BUFFER_COUNT_MIN;
		inPort->sPortParam.nBufferCountActual        = INPUT_BUFFER_COUNT_MIN;
		inPort->sPortParam.format.video.xFramerate   = K2Q16(OUTPUT_FRAME_RATE_DEFAULT);
		inPort->sPortParam.format.video.nFrameWidth  = OUTPUT_WIDTH_MAX;
		inPort->sPortParam.format.video.nFrameHeight = OUTPUT_HEIGHT_MAX;
	}


	/** common parameters related to output port */
	outPort = (omx_base_video_PortType *)p_private->ports[OMX_BASE_FILTER_OUTPUTPORT_INDEX];

#if defined (TCC_HEVC_INCLUDE) || defined (TCC_VPU_4K_D2_INCLUDE)
	if (coding_type == OMX_VIDEO_CodingHEVC){
		outPort->sPortParam.nBufferSize               = HEVC_OUTPUT_WIDTH_MAX * HEVC_OUTPUT_HEIGHT_MAX * 3 / 2;
		outPort->sPortParam.nBufferCountMin           = OUTPUT_BUFFER_COUNT_MIN;
		outPort->sPortParam.nBufferCountActual        = OUTPUT_BUFFER_COUNT_MIN;
		outPort->sPortParam.format.video.xFramerate   = K2Q16(OUTPUT_FRAME_RATE_DEFAULT);
		outPort->sPortParam.format.video.nFrameWidth  = HEVC_OUTPUT_WIDTH_MAX;
		outPort->sPortParam.format.video.nFrameHeight = HEVC_OUTPUT_HEIGHT_MAX;
		outPort->sPortParam.format.video.nStride      = HEVC_OUTPUT_WIDTH_MAX;
		outPort->sPortParam.format.video.eColorFormat = OUTPUT_FORMAT_DEFAULT;

		p_private->lPicWidthMax  = HEVC_OUTPUT_WIDTH_MAX;
		p_private->lPicHeightMax = HEVC_OUTPUT_HEIGHT_MAX;
	} else
#endif
#if defined (TCC_VPU_4K_D2_INCLUDE)
	if (coding_type == OMX_VIDEO_CodingVP9){
		outPort->sPortParam.nBufferSize               = VP9_OUTPUT_WIDTH_MAX * VP9_OUTPUT_HEIGHT_MAX * 3 / 2;
		outPort->sPortParam.nBufferCountMin           = OUTPUT_BUFFER_COUNT_MIN;
		outPort->sPortParam.nBufferCountActual        = OUTPUT_BUFFER_COUNT_MIN;
		outPort->sPortParam.format.video.xFramerate   = K2Q16(OUTPUT_FRAME_RATE_DEFAULT);
		outPort->sPortParam.format.video.nFrameWidth  = VP9_OUTPUT_WIDTH_MAX;
		outPort->sPortParam.format.video.nFrameHeight = VP9_OUTPUT_HEIGHT_MAX;
		outPort->sPortParam.format.video.nStride      = VP9_OUTPUT_WIDTH_MAX;
		outPort->sPortParam.format.video.eColorFormat = OUTPUT_FORMAT_DEFAULT;

		p_private->lPicWidthMax  = VP9_OUTPUT_WIDTH_MAX;
		p_private->lPicHeightMax = VP9_OUTPUT_HEIGHT_MAX;
	} else
#endif

	{
		outPort->sPortParam.nBufferSize               = OUTPUT_WIDTH_MAX * OUTPUT_HEIGHT_MAX * 3 / 2;
		outPort->sPortParam.nBufferCountMin           = OUTPUT_BUFFER_COUNT_MIN;
		outPort->sPortParam.nBufferCountActual        = OUTPUT_BUFFER_COUNT_MIN;
		outPort->sPortParam.format.video.xFramerate   = K2Q16(OUTPUT_FRAME_RATE_DEFAULT);
		outPort->sPortParam.format.video.nFrameWidth  = OUTPUT_WIDTH_MAX;
		outPort->sPortParam.format.video.nFrameHeight = OUTPUT_HEIGHT_MAX;
		outPort->sPortParam.format.video.nStride      = OUTPUT_WIDTH_MAX;
		outPort->sPortParam.format.video.eColorFormat = OUTPUT_FORMAT_DEFAULT;

		p_private->lPicWidthMax  = OUTPUT_WIDTH_MAX;
		p_private->lPicHeightMax = OUTPUT_HEIGHT_MAX;
	}

#if (OUTPUT_FORMAT_DEFAULT == OMX_COLOR_FormatYUV420PlanarTc) || (OUTPUT_FORMAT_DEFAULT == OMX_COLOR_FormatYUV420SemiPlanarTc)
	outPort->sPortParam.nBufferSize = sizeof(tcc_video_out_info);
#endif

	/** settings of output port parameter definition */
	outPort->sVideoParam.xFramerate   = K2Q16(OUTPUT_FRAME_RATE_DEFAULT);
	outPort->sVideoParam.eColorFormat = OUTPUT_FORMAT_DEFAULT;

	/** settings of video decoder internal */
	p_private->enVideoCodingType = coding_type;

	/** JPG image decoding **/
	p_private->pAddedInputBuffer = NULL;
	p_private->lAddedInputBufferLen = 0;

	/** pmap **/
	p_private->mTmem_fd = -1;
#if defined(TC_SECURE_MEMORY_COPY)
	{
		p_private->mTmem_fd = open(TMEM_DEVICE, O_RDWR);
		if (p_private->mTmem_fd < 0) {
			LOGE("can't open[%s] '%s'", strerror(errno), TMEM_DEVICE);
		}
		else
		{
			/** secure video path **/
			st_func_get_pmap_info("video_sbackup", &p_private->mSeqBackupPmap);
			if( p_private->mSeqBackupPmap.iSize > 0 )
			{
                if( ( p_private->mSeqBackupMapInfo = (unsigned long*)mmap(0, p_private->mSeqBackupPmap.iSize, PROT_READ | PROT_WRITE, MAP_SHARED, p_private->mTmem_fd, p_private->mSeqBackupPmap.iAddress) ) == MAP_FAILED )
				{
					LOGE("%s device's secured_inbuffer's mmap failed.", TMEM_DEVICE);
				}
			}
		}
	}
#endif
	if (SetInternalVideoParameters(p_private) == FALSE) {
		eError = OMX_ErrorNotImplemented;
		goto CONSTRUCT_FAILED;
	}
    p_private->mInputFrameCount = 0;
CONSTRUCT_FAILED:

	return eError;
}

/** The destructor of the video decoder component
  */
static
OMX_ERRORTYPE
omx_videodec_component_Destructor(
	OMX_COMPONENTTYPE *openmaxStandComp
	)
{
	vdec_private_t* p_private = GET_VIDEODEC_PRIVATE(openmaxStandComp);
	OMX_U32 i;

    LOG_STEP("In %s ", __func__);

#if defined(TC_SECURE_MEMORY_COPY)
	#if !defined(TCC_892X_INCLUDE)
	if( CHECK_STATE(p_private, STATE_VDEC_BUFFER_PROTECTION) )
	{
		TC_SecureMemoryAPI_ServiceEnd();
	}
	#endif
	if(p_private->mSeqBackupMapInfo != NULL)
	{
		munmap(p_private->mSeqBackupMapInfo, p_private->mSeqBackupPmap.iSize);
	}

#endif

#if defined(TC_SECURE_MEMORY_COPY)
	if (p_private->mTmem_fd > 0) {
		close(p_private->mTmem_fd);
		p_private->mTmem_fd = -1;
	}
#endif

	/* free component private members */
	if (p_private->pstCodecSyncSem) {
		tsem_deinit(p_private->pstCodecSyncSem);
		free(p_private->pstCodecSyncSem);
		p_private->pstCodecSyncSem = NULL;
	}

	if (p_private->pVideoParam) {
		free(p_private->pVideoParam);
		p_private->pVideoParam = NULL;
	}

	/* Release decoder instance */
	if (p_private->pVDecInstance) {
		vdec_release_instance(p_private->pVDecInstance);
		p_private->pVDecInstance = NULL;
	}

	/* free ports */
	if (p_private->ports) {
		if (p_private->ports[0])
			p_private->ports[0]->PortDestructor(p_private->ports[0]);
		if (p_private->ports[1])
			p_private->ports[1]->PortDestructor(p_private->ports[1]);
		free(p_private->ports);
		p_private->ports = NULL;
	}

	omx_base_filter_Destructor(openmaxStandComp);

	return OMX_ErrorNone;
}

/** It initializates the VPU framework, and opens an VPU
 *  videodecoder of type specified by IL client
  */
static
OMX_ERRORTYPE
omx_videodec_component_InitResource(
	vdec_private_t *pstVDecPrivate
	)
{
	OMX_ERRORTYPE err;

    LOG_STEP("In %s ", __func__);

	tsem_up(pstVDecPrivate->pstCodecSyncSem);

	return OMX_ErrorNone;
}

/** It Deinitializates the VPU framework, and close the VPU
 *  video decoder of selected coding type
  */
static
void
omx_videodec_component_DeinitResource(
	vdec_private_t *pstVDecPrivate
	)
{
	long ret;

	LOG_STEP("In %s ", __func__);

	if (pstVDecPrivate->pbyTempInputBuff) {
		TCC_free(pstVDecPrivate->pbyTempInputBuff);
		pstVDecPrivate->pbyTempInputBuff = NULL;
	}

	if (pstVDecPrivate->pbyExtraDataBuff) {
		TCC_free(pstVDecPrivate->pbyExtraDataBuff);
		pstVDecPrivate->pbyExtraDataBuff = NULL;
	}

	if (pstVDecPrivate->pbyThumbnailBuff) {
		TCC_free(pstVDecPrivate->pbyThumbnailBuff);
		pstVDecPrivate->pbyThumbnailBuff = NULL;
	}

	if (pstVDecPrivate->pbySequenceHeader) {
		TCC_free(pstVDecPrivate->pbySequenceHeader);
		pstVDecPrivate->pbySequenceHeader = NULL;
	}

	if( CHECK_MODE(pstVDecPrivate, MODE_RINGBUFFER_MODE) ) {
		if (pstVDecPrivate->stRingBuffState.pBackupBuffer) {
			TCC_free(pstVDecPrivate->stRingBuffState.pBackupBuffer);
			pstVDecPrivate->stRingBuffState.pBackupBuffer = NULL;
		}
		DeinitInputQueue(&pstVDecPrivate->stInputInfoQueue);
	}

	DeinitDispInfoManager(&pstVDecPrivate->stDispInfoMgr);

	DecoderDeinit(pstVDecPrivate);
}

/** The Initialization function of the video decoder
  */
static
OMX_ERRORTYPE
omx_videodec_component_Initialize(
	OMX_COMPONENTTYPE *openmaxStandComp
	)
{
	vdec_private_t* p_private = GET_VIDEODEC_PRIVATE(openmaxStandComp);
	OMX_ERRORTYPE eError = OMX_ErrorNone;

	LOG_STEP("In %s ", __func__);

	CLEAR_STATE(p_private, STATE_OUTPUT_PORT_FLUSHED);

#if ES_DUMP
    {
		FILE *fp = fopen("/run/media/sda1/dump/hevc_hdr10.es", "wb");
		if (fp)
			fclose(fp);
	}
#endif
	return eError;
}

/** The Deinitialization function of the video decoder
  */
static
OMX_ERRORTYPE
omx_videodec_component_Deinitialize(
	OMX_COMPONENTTYPE *openmaxStandComp
	)
{
	vdec_private_t* p_private = GET_VIDEODEC_PRIVATE(openmaxStandComp);
	OMX_ERRORTYPE err;

	LOG_STEP("In %s ", __func__);

	if (CHECK_STATE(p_private, STATE_RESOURCE_INITIALIZED)) {
		omx_videodec_component_DeinitResource(p_private);
		CLEAR_STATE(p_private, STATE_RESOURCE_INITIALIZED);
	}

	return OMX_ErrorNone;
}

/** This function is used to process the input buffer and provide one output buffer
  */
static
void
omx_videodec_component_BufferMgmtCallback(
   OMX_COMPONENTTYPE 	*openmaxStandComp,
   OMX_BUFFERHEADERTYPE	*pInputBuffer,
   OMX_BUFFERHEADERTYPE	*pOutputBuffer
   )
{
	vdec_private_t *p_private = GET_VIDEODEC_PRIVATE(openmaxStandComp);
	OMX_TICKS  curr_timestamp;
	OMX_U32    dec_result = 0;
	OMX_S32    ret = 0;
	OMX_U32    input_flags = pInputBuffer->nFlags;

#if defined(TCC_VSYNC_INCLUDE)
	if(!((input_flags & OMX_BUFFERFLAG_STARTTIME) || (input_flags & OMX_BUFFERFLAG_CODECCONFIG)) )
	{
		if(p_private->bClearDisplayIndex == OMX_TRUE)
		{
			if(ClearDisplayedBuffer(p_private, OMX_FALSE) < 0)
			{
				//if gst FlushStart event arrived, pInputBuffer len should be 0 to return buffer
				if(p_private->bEventFlush == OMX_TRUE)
				{
					p_private->bEventFlush = OMX_FALSE;
					pInputBuffer->nFilledLen = 0;
				}

				pOutputBuffer->nFilledLen = 0;
				p_private->bClearDisplayIndex = OMX_TRUE;
				usleep(100);
				return;
			}
			p_private->bClearDisplayIndex = OMX_FALSE;
		}
	}
#endif

#if LOG_INPUT_BUFF_INFO
	{
		static OMX_TICKS prev_pts;
		static OMX_U8 *p_prev_buff;

		LOG_INPUT("[LENGTH: %6ld] [PTS: %8ld (diff: %4ld)] [BUFF: 0x%08x (%6ld)] - [Flags: %c%c%c%c (0x%08lX)] %s %s"
				  , pInputBuffer->nFilledLen
				  , (OMX_S32)(pInputBuffer->nTimeStamp/1000)
				  , (OMX_S32)((pInputBuffer->nTimeStamp - prev_pts)/1000)
				  , pInputBuffer->pBuffer
				  , pInputBuffer->nOffset
				  , input_flags & OMX_BUFFERFLAG_CODECCONFIG ?    'C' : '-'
				  , input_flags & OMX_BUFFERFLAG_STARTTIME ?      'S' : '-'
				  //, input_flags & OMX_BUFFERFLAG_IFRAME_ONLY ?    'I' : '-'
				  , '-'
				  //, input_flags & OMX_BUFFERFLAG_BFRAME_SKIP ?    'B' : '-'
				  , '-'
				  , input_flags
				  , p_prev_buff != pInputBuffer->pBuffer ?        " NEWB" : " ----"
				  , prev_pts != pInputBuffer->nTimeStamp ?        " NEWP" : " ----"
				  );

		p_prev_buff = pInputBuffer->pBuffer;
		prev_pts = pInputBuffer->nTimeStamp;
	}
#endif

#if ES_DUMP
	{
		FILE *fp = fopen("/run/media/sda1/dump/hevc_hdr10.es", "a+b");
		if (fp) {
			fwrite(&pInputBuffer->nFilledLen, 1, 4, fp);
			fwrite(pInputBuffer->pBuffer, 1, pInputBuffer->nFilledLen, fp);
			fclose(fp);
		}
        pInputBuffer->nFilledLen = 0;
        pOutputBuffer->nFilledLen = 0;
        return;
	}
#endif

	if (CHECK_STATE(p_private, STATE_WAIT_RESOURCE_INIT)) {
		tsem_down(p_private->pstCodecSyncSem);
		CLEAR_STATE(p_private, STATE_WAIT_RESOURCE_INIT);
	}

	if( input_flags & OMX_BUFFERFLAG_STARTTIME ) {
		p_private->bDecStarted = FALSE;
	}

	pOutputBuffer->nFilledLen = 0;

	if( pInputBuffer->nFilledLen == 0 ) {
		if (((input_flags & OMX_BUFFERFLAG_EOS) == 0UL) || ((input_flags & OMX_BUFFERFLAG_CONTINUE) != 0UL)) {
			return;
		}
		if ((p_private->stVDecInit.m_iBitstreamFormat != STD_MJPG)
#ifdef INCLUDE_WMV78_DEC
		&& (p_private->stVDecInit.m_iBitstreamFormat != STD_WMV78)
#endif
		){
			SET_STATE(p_private, STATE_FLUSHING_DELAYED_FRAME);
		}
	}

	/* resolution with re-init step5 - re-init (reset decoding status) */
	if (CHECK_STATE(p_private, STATE_RESOLUTION_CHANGING_WITH_REINIT) && CHECK_STATE(p_private, STATE_READY_TO_RESET)) {
		CLEAR_STATE(p_private, STATE_READY_TO_RESET);
		PrepareResolutionChange(p_private);
	}

	//////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////
	/// extract config data (from TCC extractor or the others)
	if ((input_flags & OMX_BUFFERFLAG_CODECCONFIG) && !CHECK_STATE(p_private, STATE_VDEC_BUFFER_PROTECTION))
	{
		ParseConfigData(p_private, pInputBuffer);	//store sequence header

		if (p_private->pbySequenceHeader && p_private->lSeqHeaderLength > 0) {
#if SINGLE_FRAME_INPUT_ENABLE
			SET_STATE(p_private, STATE_FRAME_UNIT_INPUT);
#endif
			//TAG:MOD - Because frame-rate info isn't exist,
			//          STATE_FRAME_RATE_UPDATE will be used.
			//CLEAR_STATE(p_private, STATE_FRAME_RATE_UPDATE);
			SET_STATE(p_private, STATE_SEQUENCE_HEADER_FOUND);
		}
		else {
#ifdef INCLUDE_WMV78_DEC
			if (p_private->stVDecInit.m_iBitstreamFormat != STD_WMV78)
#endif
			{
				SET_STATE(p_private, STATE_STREAM_CONVERTION_NEEDED);
			}
		}

		if( p_private->lDecodedCount > 0 ) {
			if( p_private->stVDecUserInfo.extFunction & EXT_FUNC_MAX_FRAMEBUFFER ) {
				/* resolution with crop step0 - attach sequence header */
				SET_STATE(p_private, STATE_RESOLUTION_CHANGING_WITH_CROP);
			}
			else  {
				/* resolution with re-init step1 - set flag */
				INFO("Resolution Changed - new config data");
				SET_STATE(p_private, STATE_RESOLUTION_CHANGING_WITH_REINIT);

				if( CHECK_STATE(p_private, STATE_OUTPUT_PORT_FLUSHED) ) {
					SET_STATE(p_private, STATE_READY_TO_RESET);
				}
			}
		}

		pInputBuffer->nFilledLen = 0;
		return;
	}

	if (CHECK_STATE(p_private, STATE_OUTPUT_PORT_FLUSHED))  {
		CLEAR_STATE(p_private, STATE_OUTPUT_PORT_FLUSHED);
		input_flags |= OMX_BUFFERFLAG_STARTTIME;
	}

	//////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////
	/// Decoder initialization
	///  - find sequence header (for MPEG2 Trasnport Stream)
	///  - decoder init (open)
	///  - feed decoder
	///  - decoder sequence init
	///  - check needs port reconfiguration
	if( CHECK_STATE(p_private, STATE_VDEC_INITIALIZED) == 0UL )
	{
		if (OMXVideoDecode_Init(p_private, pInputBuffer, pOutputBuffer) < 0) {
			return;
		}
	}

	if( CHECK_STATE(p_private, STATE_DECODER_OPENED) == 0UL ) {
		(void)ErrorProcess(p_private, pInputBuffer, pOutputBuffer, __LINE__,PROCESS_NONE);
		return;
	}


	//////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////
	/// Skimming / Fast-playback support
	//TAG:DEL non-standard
#if 0
	if( input_flags & OMX_BUFFERFLAG_BFRAME_SKIP ) {
		SetFrameSkipMode(p_private, SKIPMODE_B_SKIP);
		CLEAR_MODE(p_private, MODE_DECODED_FRAME_OUTPUT);
	}
	else if( input_flags & OMX_BUFFERFLAG_IFRAME_ONLY )
		SET_MODE(p_private, MODE_DECODED_FRAME_OUTPUT);
	else
		CLEAR_MODE(p_private, MODE_DECODED_FRAME_OUTPUT);
#endif


	//////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////
	/// sequence start or seeking process
	if((( input_flags & OMX_BUFFERFLAG_STARTTIME ) != 0UL)
#if CHECK_ERROR_MACROBLOCK
	  || (CHECK_STATE(p_private, STATE_DETECT_ERROR_MACROBLOCK) != 0UL)
#endif
	)
	{
		OMX_BOOL is_seeking = (CHECK_MODE(p_private, MODE_DECODED_FRAME_OUTPUT) != 0UL) ? OMX_FALSE : OMX_TRUE;

		INFO("START-TIME FLAG RECEIVED%s", is_seeking ? "" : " - SKIMMING");

		if( CHECK_STATE(p_private, STATE_FLUSHING_DELAYED_FRAME) != 0UL ) {
			while( VDEC_FUNC(p_private, VDEC_DEC_FLUSH_OUTPUT, NULL, &p_private->stVDecInput, &p_private->stVDecOutput) >= 0 &&
			       p_private->stVDecOutput.m_DecOutInfo.m_iDispOutIdx >= 0 );
		    CLEAR_STATE(p_private, STATE_FLUSHING_DELAYED_FRAME);
		}
		((omx_base_filter_PrivateType*)p_private)->delayedOutputBufferCount = 0;

		/* reset MVC base-view address */
		p_private->lMVCBaseViewIndex = -1;
		p_private->pMVCBaseView[0] = NULL;
		p_private->pMVCBaseView[1] = NULL;
		p_private->pMVCBaseView[2] = NULL;

		if( CHECK_MODE(p_private, MODE_RINGBUFFER_MODE) != 0UL )
		{
			/* reset decoder ring-buffer state */
			ResetRingBuffer(&p_private->stRingBuffState, OMX_TRUE);
			/* reset input buffer information queue */
			ClearInputQueue(&p_private->stInputInfoQueue);
			UpdateRingBuffer(&p_private->stRingBuffState, OMX_TRUE);
			p_private->bEventFlush = OMX_FALSE;
		}

		/* reset display information manager */
		ResetDispInfoManager(&p_private->stDispInfoMgr);

		/* clear all display buffer */
		if( (ret = ClearAllDisplayBuffers(p_private)) < 0 ) {
			ERROR("ClearAllDisplayBuffers() - returns %ld", ret);
			(void)ErrorProcess(p_private, pInputBuffer, pOutputBuffer, ret, PROCESS_OPEN);
			return;
		}
		ClearDispIdxQueue(&p_private->stDispIdxQueue);

		/* feeder reset */
		p_private->llQueuedStartTime = -1;

		/* I-frame search enable */
		IFrameSearchEnable(p_private);

		CLEAR_ERROR(p_private, ERROR_INSUFFICIENT_BITSTREAM);
		SET_STATE(p_private, STATE_SKIP_OUTPUT_B_FRAME);

		if( CHECK_MODE(p_private, MODE_DECODED_KEYFRAME_OUTPUT) != 0UL )
			SET_STATE(p_private, STATE_WAIT_DECODED_KEYFRAME);

		/* reset slice merging */
		p_private->lSliceCount = 0;

		if(CHECK_ERROR(p_private, ERROR_INPUT_BUFFER_REMAINNING) != 0UL ) {
			p_private->pbyVDecInputPtr = NULL;
			p_private->lVDecInputLength = 0;
			CLEAR_ERROR(p_private, ERROR_INPUT_BUFFER_REMAINNING);
		}

		/* provide for HEVC prescan */
		p_private->llPrevPrevInputPts = p_private->llPrevInputPts = pInputBuffer->nTimeStamp;

		CLEAR_STATE(p_private, STATE_DETECT_ERROR_MACROBLOCK);

	}

	if ( !p_private->bDecStarted ) {
		SET_STATE(p_private, STATE_FRAME_DISCONTINUITY);
		p_private->bDecStarted = TRUE;
	}

	if(CHECK_MODE(p_private, MODE_I_FRAME_ONLY_OUTPUT) != 0UL) {
#if defined(TCC_HEVC_INCLUDE) || defined (TCC_VPU_4K_D2_INCLUDE)
		if( p_private->stVDecInit.m_iBitstreamFormat == STD_HEVC
			#if defined (TCC_VPU_4K_D2_INCLUDE)
			|| p_private->stVDecInit.m_iBitstreamFormat == STD_VP9
			#endif
			) {
			IFrameSearchEnable(p_private);
		}
		else
#endif
		{
			SetFrameSkipMode(p_private, SKIPMODE_EXCEPT_I_SKIP);
		}
	}

#if 0 // currently not used
	if( CHECK_STATE(p_private, STATE_IDR_SLICE_SCAN) )
	{
		OMX_S32 idr_pos = ScanAvcIdrSlice(pInputBuffer);
		if( idr_pos >= 0 )
		{
			INFO("IDR-FRAME FOUND");
			/* IDR slice is found */
			if( p_private->stVDecInput.m_iFrameSearchEnable == AVC_NONIDR_PICTURE_SEARCH_MODE ) {
				p_private->lSkippedCount = 0;
				p_private->lIdrSliceScanCount = 0;
				p_private->lDecFrameSkipCount = 0;
				SetFrameSkipMode(p_private, SKIPMODE_IDR_SEARCH);
			}
		}
		else
		{
			/* IDR slice is not found */
			if( p_private->stVDecInput.m_iFrameSearchEnable == AVC_IDR_PICTURE_SEARCH_MODE ) {
				if( p_private->lIdrSliceScanCount++ < AVC_IDR_SLICE_SCAN_MAX ) {
					pInputBuffer->nFilledLen = 0;
					return;
				}
				INFO("IDR-FRAME NOT FOUND");
				p_private->lDecFrameSkipCount = 0;
				SetFrameSkipMode(p_private, SKIPMODE_NONIDR_SEARCH);
			}
		}

		CLEAR_STATE(p_private, STATE_IDR_SLICE_SCAN);
	}
#endif


	//////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////
	/// Feed decoder (fill ring-buffer)

	if( CHECK_STATE(p_private, STATE_FLUSHING_DELAYED_FRAME) == 0UL) {
		if (OMXVideoDecode_FeedInputData(p_private, pInputBuffer, pOutputBuffer, input_flags) < 0) {
			return;
		}
	}

	if( CHECK_MODE(p_private, MODE_RINGBUFFER_MODE) != 0UL)
	{
		OMX_S32 validsize = 0;
		OMX_BOOL need_more_data;
		need_more_data = CheckRemainRingBufferSize(&p_private->stRingBuffState, p_private, &validsize);
		if( CHECK_ERROR(p_private, ERROR_INSUFFICIENT_BITSTREAM) && !(input_flags & OMX_BUFFERFLAG_EOS)){
			LOGW("After Unset ERROR_INSUFFICIENT_BITSTREAM, Do DecoderDecode, bufferRemainSize: %ld (ret: %ld)",validsize,(OMX_S32)need_more_data);
			return;
		}
		//INFO("[DECODER Before] bufferRemainSize: %ld",validsize);
	}

	//////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////
	/// Decoding
	if( (ret = OMXVideoDecode_Decode(p_private, &dec_result)) < 0 ) {
		if(ret != -RETCODE_INSUFFICIENT_BITSTREAM) {
			ERROR("DecoderDecode() - returns %ld", ret);
		}

		if( ErrorProcess(p_private, pInputBuffer, pOutputBuffer, ret, PROCESS_DECODE) ) {
			return;
		}
	}

	//////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////
	/// Check resolution changing
	//if( dec_result & (DECODING_SUCCESS | RESOLUTION_CHANGED) ) // from commit 5921315de8a5d4348f11b22c418bfa7567461819 (2013.03.08)
	if( (dec_result & RESOLUTION_CHANGED || dec_result & DECODING_SUCCESS_FRAME) &&
	    (CHECK_STATE(p_private, STATE_RESOLUTION_CHANGING_WITH_CROP|STATE_RESOLUTION_CHANGING_WITH_REINIT) == 0UL) &&
		(    p_private->stVDecInit.m_iBitstreamFormat == STD_AVC
		  || p_private->stVDecInit.m_iBitstreamFormat == STD_MVC
		  || p_private->stVDecInit.m_iBitstreamFormat == STD_MPEG2
#if defined (TCC_HEVC_INCLUDE) || defined (TCC_VPU_4K_D2_INCLUDE)
		  || p_private->stVDecInit.m_iBitstreamFormat == STD_HEVC
#endif
#if defined (TCC_VPU_4K_D2_INCLUDE)
		  || p_private->stVDecInit.m_iBitstreamFormat == STD_VP9
#endif
	  ))
	{
		if( p_private->stVDecUserInfo.extFunction & EXT_FUNC_MAX_FRAMEBUFFER )
		{
			if( IsCropChange(p_private) )
			{
#if defined(SET_FRAMEBUFFER_INTO_MAX)
				/* resolution with crop step1 - set flag */
				INFO("Resolution Change Detected (crop changing)");
				SET_STATE(p_private, STATE_RESOLUTION_CHANGING_WITH_CROP);
				SET_STATE(p_private, STATE_WAIT_CHANGED_OUTPUT);
				vpu_update_sizeinfo(p_private->stVDecInit.m_iBitstreamFormat,
									p_private->stVDecUserInfo.bitrate_mbps,
									p_private->stVDecUserInfo.frame_rate,
									p_private->stCropRect.nWidth,
									p_private->stCropRect.nHeight,
									p_private->pVDecInstance);
#else
				INFO("Resolution Change Detected (not supported)");
				SendEventToClient(p_private,
								  OMX_EventError,
								  OMX_ErrorStreamCorrupt,
								  0);
				return;
#endif
			}
		}
		else
		{
			if( IsResolutionChanged(p_private) ) {
#if defined (TCC_HEVC_INCLUDE) //|| defined (TCC_VPU_4K_D2_INCLUDE)
				if( p_private->stVDecInit.m_iBitstreamFormat == STD_HEVC ) {
					dec_result &= ~DECODING_SUCCESS;
					////////////////////////////////////////////////////////////////////
					/// This code was temporarily inserted, because of the system hangup problem
					if( CHECK_MODE(p_private, MODE_COMPRESSED_OUTPUT) ) {
						SendEventToClient(p_private,
										  OMX_EventError,
										  OMX_ErrorStreamCorrupt,
										  0);
					}
					////////////////////////////////////////////////////////////////////
				}
#endif

				if( (dec_result & DECODING_SUCCESS_FRAME) != 0UL )
				{
					IsCropChange(p_private);
					/* resolution with crop step1 - set flag */
					INFO("Resolution Change Detected (crop changing)");
					SET_STATE(p_private, STATE_RESOLUTION_CHANGING_WITH_CROP);
					SET_STATE(p_private, STATE_WAIT_CHANGED_OUTPUT);
					vpu_update_sizeinfo(p_private->stVDecInit.m_iBitstreamFormat,
										p_private->stVDecUserInfo.bitrate_mbps,
										p_private->stVDecUserInfo.frame_rate,
										p_private->stCropRect.nWidth,
										p_private->stCropRect.nHeight,
										p_private->pVDecInstance);
				}
				else
				{
					/* resolution with re-init step1 - set flag */
					INFO("Resolution Change Detected (reinit)");
					SET_STATE(p_private, STATE_RESOLUTION_CHANGING_WITH_REINIT);	//FIXME: with crop
					CLEAR_STATE(p_private, STATE_READY_TO_RESET);
					if( CHECK_MODE(p_private, MODE_RINGBUFFER_MODE) ) {
						if( BackupRingBuffer(p_private) == OMX_FALSE ) {
							SendEventToClient(p_private,
											  OMX_EventError,
											  OMX_ErrorStreamCorrupt,
											  0);
						}
					}
				}
			}
		}
	}

	//////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////
	/// Get timestamp of the currently decoded frame
	///  - update ring buffer status
	///  - clear input information from the input info queue
	if( CHECK_MODE(p_private, MODE_RINGBUFFER_MODE) != 0UL ) {
		if( GetCurrTimestamp(  p_private
							 , &curr_timestamp
							 , dec_result & DECODING_SUCCESS
							 , p_private->stVDecOutput.m_DecOutInfo.m_iInterlacedFrame ) < 0 ) {
			(void)ErrorProcess(p_private, pInputBuffer, pOutputBuffer, __LINE__, PROCESS_NONE);
			return;
		}
	}
	else {
#if defined (TCC_HEVC_INCLUDE) || defined (TCC_VPU_4K_D2_INCLUDE)
		if( p_private->stVDecInit.m_iBitstreamFormat == STD_HEVC ) {
#if defined (TCC_HEVC_INCLUDE)
			curr_timestamp = p_private->llPrevInputPts;
			p_private->llPrevInputPts = pInputBuffer->nTimeStamp;
#endif
#if defined (TCC_VPU_4K_D2_INCLUDE)
			if( CHECK_MODE(p_private, MODE_EXT_FUNC_USE_CQ_1) != 0UL ){
				curr_timestamp = p_private->llPrevInputPts;
				p_private->llPrevInputPts = pInputBuffer->nTimeStamp;
			}
			else {
				curr_timestamp = p_private->llPrevPrevInputPts;
				p_private->llPrevPrevInputPts = p_private->llPrevInputPts;
				p_private->llPrevInputPts = pInputBuffer->nTimeStamp;
			}
#endif
		}
		else
#endif
		{
			curr_timestamp = pInputBuffer->nTimeStamp;
		}
	}

	/* resolution with re-init step2 - store sequence header */
	/* resolution with crop step2 - store sequence header */
	if( CHECK_STATE(p_private, STATE_RESOLUTION_CHANGING_WITH_CROP) != 0UL) {
		if(StoreSequenceHeader(p_private,
					p_private->stRingBuffState.pPrevReadPtr,
					p_private->stRingBuffState.lUsedByte) == OMX_FALSE){
			ERROR("StoreSequenceHeader() - failed");
		}
		CLEAR_STATE(p_private, STATE_RESOLUTION_CHANGING_WITH_CROP);
	}

	if (p_private->stVDecOutput.m_DecOutInfo.m_iNumOfErrMBs > 0)
	{
		LOGI("There is %d Error MB",p_private->stVDecOutput.m_DecOutInfo.m_iNumOfErrMBs);
		SET_STATE(p_private, STATE_DETECT_ERROR_MACROBLOCK);
#if CHECK_ERROR_MACROBLOCK
		pOutputBuffer->nFilledLen = 0;
		pInputBuffer->nFilledLen = 0;
		return;
#endif
	}

	//////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////
	/// Decoding success process
	///  - calculate duration factor of the decoded frame
	///  - update information of the decoded frame
	///  - set next frame skip mode
	///  - (create thumbnail)
	if( (dec_result & DECODING_SUCCESS) != 0UL )
	{
		p_private->lDecodingFailCount = 0;

		/* Update display information */
		if( (dec_result & DECODING_SUCCESS_FRAME) != 0UL )
		{
			OMX_S32 decoded_idx = p_private->stVDecOutput.m_DecOutInfo.m_iDecodedIdx;
			char char_m_iDecodedIdx, char_m_iDispOutIdx;
			#if defined(TCC_VPU_C7_INCLUDE)
			char_m_iDecodedIdx = (p_private->stVDecOutput.m_DecOutInfo.m_iDecodedIdx < 0) ? '-' : (p_private->stVDecOutput.m_DecOutInfo.m_MvcPicInfo.m_iViewIdxDecoded ? 'D' : 'B');
			char_m_iDispOutIdx = (p_private->stVDecOutput.m_DecOutInfo.m_iDispOutIdx < 0) ? '-' : (p_private->stVDecOutput.m_DecOutInfo.m_MvcPicInfo.m_iViewIdxDisplay ? 'D' : 'B');
			#else
			char_m_iDecodedIdx = '-';
			char_m_iDispOutIdx = '-';
			#endif
			LOG_DEC("[TYPE: %s (%3d/%d)][PTS: %8ld (diff: %4ld)] [State: %2d/%2d][BuffIdx: %2d(%c)/%2d(%c)] [FieldSeq: %d][Interlaced: %d (%d)] [MBerr: %d] [KEY-DIST: %3ld]"
					, GetFrameTypeString(p_private->stVDecInit.m_iBitstreamFormat
										 , p_private->stVDecOutput.m_DecOutInfo.m_iPicType
										 , p_private->stVDecOutput.m_DecOutInfo.m_iPictureStructure)
					, p_private->stVDecOutput.m_DecOutInfo.m_iPicType
					, p_private->stVDecOutput.m_DecOutInfo.m_iPictureStructure
					, (OMX_S32)(curr_timestamp/1000)
					, (OMX_S32)((curr_timestamp - p_private->llQueuedStartTime) /1000)
					, p_private->stVDecOutput.m_DecOutInfo.m_iDecodingStatus
					, p_private->stVDecOutput.m_DecOutInfo.m_iOutputStatus
					, p_private->stVDecOutput.m_DecOutInfo.m_iDecodedIdx
					, char_m_iDecodedIdx
					, p_private->stVDecOutput.m_DecOutInfo.m_iDispOutIdx
					, char_m_iDispOutIdx
					, p_private->stVDecOutput.m_DecOutInfo.m_iM2vFieldSequence
					, ((p_private->stVDecOutput.m_DecOutInfo.m_iM2vProgressiveFrame == 0 && p_private->stVDecOutput.m_DecOutInfo.m_iPictureStructure == 3 ) || p_private->stVDecOutput.m_DecOutInfo.m_iInterlacedFrame) ? 1 : 0
					, p_private->stVDecOutput.m_DecOutInfo.m_iTopFieldFirst
					, p_private->stVDecOutput.m_DecOutInfo.m_iNumOfErrMBs
					, p_private->lKeyFrameDistance
					);

			/* Frame-rate recalcuation */
			p_private->lFrameSuccessCount++;
			if( (CHECK_STATE(p_private, STATE_FRAME_RATE_UPDATE) != 0UL)
				&& (p_private->llQueuedStartTime != curr_timestamp) )
			{//TAG:MOD
				if( p_private->llQueuedStartTime < curr_timestamp ) {
					CLEAR_STATE(p_private, STATE_FRAME_RATE_UPDATE);
				}
				else if( p_private->lFrameSuccessCount > 1 ) {
					OMX_TICKS frame_due = (p_private->llQueuedStartTime - curr_timestamp) / p_private->lFrameSuccessCount;
					p_private->lNewFrameRate = (OMX_S32)((OMX_TICKS)1000000000 / frame_due);
				}
				p_private->lFrameSuccessCount = 0;
			}

			if( decoded_idx >= 0 )
			{
				/* Decoding success process */
				if( (ret = DecSuccessProcess(p_private, curr_timestamp, decoded_idx, FALSE)) < 0 ){
					ERROR("DecSuccessProcess() - returns %ld", ret);
					(void)ErrorProcess(p_private, pInputBuffer, pOutputBuffer, __LINE__, PROCESS_NONE);
					return;
				}

				/* set decoded buffer output flag */
				if( CHECK_MODE(p_private, MODE_DECODED_FRAME_OUTPUT) != 0UL)
				{
					dec_result |= DECODED_OUTPUT_SUCCESS;
				}
				else if( CHECK_STATE(p_private, STATE_WAIT_DECODED_KEYFRAME) != 0UL)
				{
					if( (CHECK_STATE(p_private, STATE_SLICE_COUNTING|STATE_SLICE_COUNTED) == 0UL) ||
						(p_private->stVDecOutput.m_DecOutInfo.m_iNumOfErrMBs == 0) )
					{
						OMX_S32 pic_type = GetFrameType(  p_private->stVDecInit.m_iBitstreamFormat
														, p_private->stVDecOutput.m_DecOutInfo.m_iPicType
														, p_private->stVDecOutput.m_DecOutInfo.m_iPictureStructure);
						if( pic_type == PIC_TYPE_I ) {
							CLEAR_STATE(p_private, STATE_WAIT_DECODED_KEYFRAME);
							SET_STATE(p_private, STATE_WAIT_DISPLAY_KEYFRAME);
							p_private->lKeyframeIndex = decoded_idx;
							dec_result |= DECODED_OUTPUT_SUCCESS;
						}
					}
				}
			}
			if( CHECK_MODE(p_private, MODE_RINGBUFFER_MODE) == 0UL ) {
				p_private->lVDecInputLength = 0;
				pInputBuffer->nFilledLen = 0;
			}
		}
		else if( (dec_result & DECODING_SUCCESS_FIELD) != 0UL )	//field success is only possible for fileplay mode
		{
			OMX_S32 decoded_idx = p_private->stVDecOutput.m_DecOutInfo.m_iDecodedIdx;
			OMX_S32 pts_diff;
			char char_m_iDecodedIdx, char_m_iDispOutIdx;
			#if defined(TCC_VPU_C7_INCLUDE)
			char_m_iDecodedIdx = (p_private->stVDecOutput.m_DecOutInfo.m_iDecodedIdx < 0) ? '-' : (p_private->stVDecOutput.m_DecOutInfo.m_MvcPicInfo.m_iViewIdxDecoded ? 'D' : 'B');
			char_m_iDispOutIdx = (p_private->stVDecOutput.m_DecOutInfo.m_iDispOutIdx < 0) ? '-' : (p_private->stVDecOutput.m_DecOutInfo.m_MvcPicInfo.m_iViewIdxDisplay ? 'D' : 'B');
			#else
			char_m_iDecodedIdx = '-';
			char_m_iDispOutIdx = '-';
			#endif

			if( CHECK_MODE(p_private, MODE_RINGBUFFER_MODE) != 0UL ) {
				pts_diff = (OMX_S32)((curr_timestamp - p_private->stDispInfoMgr.llLastDecTimestamp) /1000);
			}
			else {
				p_private->llQueuedStartTime = p_private->stDispInfoMgr.llLastDecTimestamp;
				pts_diff = (OMX_S32)((curr_timestamp - p_private->llQueuedStartTime) /1000);
			}

			LOG_DEC("[TYPE: %s (%3d/%d)][PTS: %8ld (diff: %4ld)] [State: %2d/%2d][BuffIdx: %2d(%c)/%2d(%c)] [FieldSeq: %d][Interlaced: %d (%d)] [MBerr: %d] [KEY-DIST: %3ld]"
					, GetFrameTypeString(p_private->stVDecInit.m_iBitstreamFormat
										 , p_private->stVDecOutput.m_DecOutInfo.m_iPicType
										 , p_private->stVDecOutput.m_DecOutInfo.m_iPictureStructure)
					, p_private->stVDecOutput.m_DecOutInfo.m_iPicType
					, p_private->stVDecOutput.m_DecOutInfo.m_iPictureStructure
					, (OMX_S32)(curr_timestamp/1000)
					, pts_diff
					, p_private->stVDecOutput.m_DecOutInfo.m_iDecodingStatus
					, p_private->stVDecOutput.m_DecOutInfo.m_iOutputStatus
					, p_private->stVDecOutput.m_DecOutInfo.m_iDecodedIdx
					, char_m_iDecodedIdx
					, p_private->stVDecOutput.m_DecOutInfo.m_iDispOutIdx
					, char_m_iDispOutIdx
					, p_private->stVDecOutput.m_DecOutInfo.m_iM2vFieldSequence
					, ((p_private->stVDecOutput.m_DecOutInfo.m_iM2vProgressiveFrame == 0 && p_private->stVDecOutput.m_DecOutInfo.m_iPictureStructure == 3 ) || p_private->stVDecOutput.m_DecOutInfo.m_iInterlacedFrame) ? 1 : 0
					, p_private->stVDecOutput.m_DecOutInfo.m_iTopFieldFirst
					, p_private->stVDecOutput.m_DecOutInfo.m_iNumOfErrMBs
					, p_private->lKeyFrameDistance
					);

			if( decoded_idx >= 0 )
			{
				/* Decoding success process */
				if( (ret = DecSuccessProcess(p_private, curr_timestamp, decoded_idx, TRUE)) < 0 ){
					ERROR("DecSuccessProcess() - returns %ld", ret);
					(void)ErrorProcess(p_private, pInputBuffer, pOutputBuffer, __LINE__, PROCESS_NONE);
					return;
				}
			}
		}

		if( CHECK_MODE(p_private, MODE_RINGBUFFER_MODE) == 0UL ) {
			p_private->pbyVDecInputPtr = NULL;
			p_private->lVDecInputLength = 0;
			pInputBuffer->nFilledLen = 0;
		}
	}
	else
	{
		char char_m_iDecodedIdx, char_m_iDispOutIdx;
		#if defined(TCC_VPU_C7_INCLUDE)
		char_m_iDecodedIdx = (p_private->stVDecOutput.m_DecOutInfo.m_iDecodedIdx < 0) ? '-' : (p_private->stVDecOutput.m_DecOutInfo.m_MvcPicInfo.m_iViewIdxDecoded ? 'D' : 'B');
		char_m_iDispOutIdx = (p_private->stVDecOutput.m_DecOutInfo.m_iDispOutIdx < 0) ? '-' : (p_private->stVDecOutput.m_DecOutInfo.m_MvcPicInfo.m_iViewIdxDisplay ? 'D' : 'B');
		#else
		char_m_iDecodedIdx = '-';
		char_m_iDispOutIdx = '-';
		#endif
		if( CHECK_MODE(p_private, MODE_RINGBUFFER_MODE) == 0UL ) {
			p_private->llQueuedStartTime = p_private->stDispInfoMgr.llLastDecTimestamp;
		}

		LOG_DECE("[TYPE: %s (%3d/%d)][PTS: %8ld (diff: %4ld)] [State: %2d/%2d][BuffIdx: %2d(%c)/%2d(%c)] [FieldSeq: %d][Interlaced: %d (%d)] [MBerr: %d] [KEY-DIST: %3ld]"
			 , GetFrameTypeString(p_private->stVDecInit.m_iBitstreamFormat
								  , p_private->stVDecOutput.m_DecOutInfo.m_iPicType
								  , p_private->stVDecOutput.m_DecOutInfo.m_iPictureStructure)
			 , p_private->stVDecOutput.m_DecOutInfo.m_iPicType
			 , p_private->stVDecOutput.m_DecOutInfo.m_iPictureStructure
			 , (OMX_S32)(curr_timestamp/1000)
			 , (OMX_S32)((p_private->llQueuedStartTime - curr_timestamp) /1000)
			 , p_private->stVDecOutput.m_DecOutInfo.m_iDecodingStatus
			 , p_private->stVDecOutput.m_DecOutInfo.m_iOutputStatus
			 , p_private->stVDecOutput.m_DecOutInfo.m_iDecodedIdx
			 , char_m_iDecodedIdx
			 , p_private->stVDecOutput.m_DecOutInfo.m_iDispOutIdx
			 , char_m_iDispOutIdx
			 , p_private->stVDecOutput.m_DecOutInfo.m_iM2vFieldSequence
			 , ((p_private->stVDecOutput.m_DecOutInfo.m_iM2vProgressiveFrame == 0 && p_private->stVDecOutput.m_DecOutInfo.m_iPictureStructure == 3 ) || p_private->stVDecOutput.m_DecOutInfo.m_iInterlacedFrame) ? 1 : 0
			 , p_private->stVDecOutput.m_DecOutInfo.m_iTopFieldFirst
			 , p_private->stVDecOutput.m_DecOutInfo.m_iNumOfErrMBs
			 , p_private->lKeyFrameDistance
			 );

		if( CHECK_MODE(p_private, MODE_RINGBUFFER_MODE) == 0UL ) {
			if ( !(dec_result & DECODING_BUFFER_FULL) && !(dec_result & RESOLUTION_CHANGED) ) {
				p_private->pbyVDecInputPtr = NULL;
				p_private->lVDecInputLength = 0;
				pInputBuffer->nFilledLen = 0;
			}
			else {
				SET_ERROR(p_private, ERROR_INPUT_BUFFER_REMAINNING);
			}
		}

		/* frame search/skip mode error process */
		if( p_private->lFrameSkipMode == SKIPMODE_I_SEARCH ) {
			if( p_private->lSkippedCount++ >= p_private->lSkipMax ) {
				ERROR("Skipped %ld frames", p_private->lSkippedCount);
				SetFrameSkipMode(p_private, SKIPMODE_NONE);
			}
		}

		if( (CHECK_MODE(p_private, MODE_DECODING_ERROR_REPORTING) != 0UL) &&
		    !(dec_result & DECODING_BUFFER_FULL ||
		      (dec_result & DECODING_SUCCESS_SKIPPED) &&
		      ((CHECK_STATE(p_private, STATE_RESOLUTION_CHANGING_WITH_REINIT|STATE_FLUSHING_DELAYED_FRAME) != 0UL) ||
		       p_private->lFrameSkipMode != SKIPMODE_NONE))
		  ) {
			p_private->lDecodingFailCount++;
			if( (p_private->lDecodingFailCount % p_private->lDecodingFailMax) == 0 ) {
				ERROR("Send decoding-fail event (fail count: %ld)", p_private->lDecodingFailCount);
				if( CHECK_MODE(p_private, MODE_DECODING_ERROR_REPORTING_REPEAT) != 0UL){
					p_private->lDecodingFailCount = 0;
				}
				SendEventToClient(p_private,
					  OMX_EventError,
					  OMX_ErrorStreamCorrupt,
					  0
					  );
			}
		}
	}

	//////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////
	/// Output success process
	///  - output buffer setup
	///  - queue display buffer index
	///  - clear displayed buffer
	if( (dec_result & OUTPUT_SUCCESS) != 0UL )
	{
		OMX_S32 mb_err = 0;

		/* Output success process */
		if( (ret = OutputProcess(p_private, pOutputBuffer, dec_result&DECODED_OUTPUT_SUCCESS, &mb_err)) < 0 ){
			if( ret != ERROR_SKIP_OUTPUT_FRAME ){
				ERROR("OutputProcess() - returns %ld", ret);
			}
			(void)ErrorProcess(p_private, pInputBuffer, pOutputBuffer, ret, PROCESS_DECODE);
			return;
		}

		if( mb_err > 0)
		{
			LOGI("There is %ld Error MB",mb_err);
			SET_STATE(p_private, STATE_DETECT_ERROR_MACROBLOCK);
		}

		if( CHECK_MODE(p_private, MODE_MB_ERROR_REPORTING) != 0UL ) {
			if( mb_err > 0 ) {
				SendEventToClient(p_private,
						  OMX_EventError,
						  OMX_ErrorMbErrorsInFrame,
						  mb_err
						  );
			}
		}

#if defined(SET_FRAMEBUFFER_INTO_MAX)
		/* resolution with crop step4 - event sending & complete */
		if( ret == SUCCESS_RESOLUTION_CHANGED ) {
			INFO("Resolution Change Complete");
#if !RESOLUTION_CHANGE_WITHOUT_EVENT
			SendEventToClient(p_private,
					  OMX_EventPortSettingsChanged,
					  OMX_IndexConfigCommonOutputCrop,
					  OMX_DirOutput);
#endif
		}
#endif
	}
	else
	{
		/* resolution with re-init step4 - complete flushing */
		if( CHECK_STATE(p_private, STATE_RESOLUTION_CHANGING_WITH_REINIT) != 0UL ) {
			SET_STATE(p_private, STATE_READY_TO_RESET);
			pOutputBuffer->nFilledLen = 0;
			return;
		}

		if ((dec_result & DECODING_BUFFER_FULL) != 0UL)
		{
#if defined(TCC_VSYNC_INCLUDE)
			p_private->bClearDisplayIndex = OMX_TRUE;
			if(p_private->lMaxFifoCount > p_private->lAdditionalBuffCount)
				p_private->lMaxFifoCount--;
#else
			ClearDisplayedBuffer(p_private, OMX_TRUE);
#endif
		}
	}
}

static
OMX_ERRORTYPE
omx_videodec_component_SetParameter(
	OMX_IN  OMX_HANDLETYPE 	hComponent,
	OMX_IN  OMX_INDEXTYPE 	nParamIndex,
	OMX_IN  OMX_PTR 		ComponentParameterStructure
	)
{
	vdec_private_t* p_private = GET_VIDEODEC_PRIVATE(hComponent);
	OMX_ERRORTYPE eError = OMX_ErrorNone;

	if (ComponentParameterStructure == NULL) {
		return OMX_ErrorBadParameter;
	}

	LOG_STEP("In %s (param index: 0x%08X) ", __func__, nParamIndex);

	switch (nParamIndex)
	{
	case OMX_IndexParamVideoMpeg2:
		{
			OMX_VIDEO_PARAM_MPEG2TYPE *pst_param = ComponentParameterStructure;
			OMX_U32 portIndex = pst_param->nPortIndex;
			size_t param_size = sizeof(OMX_VIDEO_PARAM_MPEG2TYPE);

			eError = omx_base_component_ParameterSanityCheck(hComponent, portIndex, pst_param, param_size);
			if (eError != OMX_ErrorNone) {
				LOGE("In %s Parameter Check Error=%x", __func__, eError);
				break;
			}

			if (pst_param->nPortIndex == 0) {
				(void)memcpy((void*)p_private->pVideoParam, (void*)pst_param, param_size);
			} else {
				eError = OMX_ErrorBadPortIndex;
				break;
			}
		}
		break;

	case OMX_IndexParamVideoMpeg4:
		{
			OMX_VIDEO_PARAM_MPEG4TYPE *pst_param = ComponentParameterStructure;
			OMX_U32 portIndex = pst_param->nPortIndex;
			size_t param_size = sizeof(OMX_VIDEO_PARAM_MPEG4TYPE);

			eError = omx_base_component_ParameterSanityCheck(hComponent, portIndex, pst_param, param_size);
			if (eError != OMX_ErrorNone) {
				LOGE("In %s Parameter Check Error=%x", __func__, eError);
				break;
			}

			if (pst_param->nPortIndex == 0) {
				(void)memcpy((void*)p_private->pVideoParam, (void*)pst_param, param_size);
			} else {
				eError = OMX_ErrorBadPortIndex;
				break;
			}
		}
		break;

	case OMX_IndexParamVideoWmv:
		{
			OMX_VIDEO_PARAM_WMVTYPE *pst_param = ComponentParameterStructure;
			OMX_U32 portIndex = pst_param->nPortIndex;
			size_t param_size = sizeof(OMX_VIDEO_PARAM_WMVTYPE);

			eError = omx_base_component_ParameterSanityCheck(hComponent, portIndex, pst_param, param_size);
			if (eError != OMX_ErrorNone) {
				LOGE("In %s Parameter Check Error=%x\n", __func__, eError);
				break;
			}

			p_private->eCodecVersion = pst_param->eFormat;
			if (pst_param->nPortIndex == 0) {
				if (SetInternalVideoParameters(p_private) == FALSE){
					eError = OMX_ErrorNotImplemented;
					break;
				}
				(void)memcpy((void*)p_private->pVideoParam, (void*)pst_param, param_size);
			} else {
				eError = OMX_ErrorBadPortIndex;
				break;
			}

			if (CHECK_STATE(p_private, STATE_DECODER_OPENED)) {
				ERROR("Decoder opened already !!");
				eError = OMX_ErrorUnsupportedSetting;
			}
		}
		break;

	case OMX_IndexParamVideoRv:
		{
			OMX_VIDEO_PARAM_RVTYPE *pst_param = ComponentParameterStructure;
			OMX_U32 portIndex = pst_param->nPortIndex;
			size_t param_size = sizeof(OMX_VIDEO_PARAM_RVTYPE);

			eError = omx_base_component_ParameterSanityCheck(hComponent, portIndex, pst_param, param_size);
			if (eError != OMX_ErrorNone) {
				LOGE("In %s Parameter Check Error=%x\n", __func__, eError);
				break;
			}

			if (pst_param->nPortIndex == 0) {
				(void)memcpy((void*)p_private->pVideoParam, (void*)pst_param, param_size);
			} else {
				eError = OMX_ErrorBadPortIndex;
				break;
			}
		}
		break;

	case OMX_IndexParamVideoAvc:
		{
			OMX_VIDEO_PARAM_AVCTYPE *pst_param = ComponentParameterStructure;
			OMX_U32 portIndex = pst_param->nPortIndex;
			size_t param_size = sizeof(OMX_VIDEO_PARAM_AVCTYPE);

			eError = omx_base_component_ParameterSanityCheck(hComponent, portIndex, pst_param, param_size);
			if (eError != OMX_ErrorNone) {
				LOGE("In %s Parameter Check Error=%x", __func__, eError);
				break;
			}

			(void)memcpy(p_private->pVideoParam, pst_param, param_size);
		}
		break;

	case OMX_IndexParamVideoH263:
		{
			OMX_VIDEO_PARAM_H263TYPE *pst_param = ComponentParameterStructure;
			OMX_U32 portIndex = pst_param->nPortIndex;
			size_t param_size = sizeof(OMX_VIDEO_PARAM_H263TYPE);

			eError = omx_base_component_ParameterSanityCheck(hComponent, portIndex, pst_param, param_size);
			if (eError != OMX_ErrorNone) {
				LOGE("In %s Parameter Check Error=%x", __func__, eError);
				break;
			}

			(void)memcpy((void*)p_private->pVideoParam, (void*)pst_param, param_size);
		}
		break;

	case OMX_IndexParamPortDefinition:
		{
			OMX_PARAM_PORTDEFINITIONTYPE *pPortDef = (OMX_PARAM_PORTDEFINITIONTYPE *)ComponentParameterStructure;
			OMX_U32 portIndex = pPortDef->nPortIndex;

			if ((eError = omx_base_component_SetParameter(hComponent, nParamIndex, ComponentParameterStructure)) == OMX_ErrorNone)
			{
				omx_base_video_PortType *port = (omx_base_video_PortType *)p_private->ports[portIndex];
				port->sVideoParam.eColorFormat = port->sPortParam.format.video.eColorFormat;

				if (portIndex == OMX_BASE_FILTER_INPUTPORT_INDEX)
				{
					if (p_private->enVideoCodingType != pPortDef->format.video.eCompressionFormat) {
						p_private->enVideoCodingType = pPortDef->format.video.eCompressionFormat;
						if (SetInternalVideoParameters(p_private) == FALSE){
							eError = OMX_ErrorNotImplemented;
							break;
						}
					}
				}

				if( p_private->lDecodedCount > 0 )
				{
					if( p_private->stCropRect.nWidth  != port->sPortParam.format.video.nFrameWidth ||
					    p_private->stCropRect.nHeight != port->sPortParam.format.video.nFrameHeight )
					{
						if( (p_private->stVDecUserInfo.extFunction & EXT_FUNC_MAX_FRAMEBUFFER) == 0 ) {
							/* resolution with re-init step1 - set flag */
							INFO("Resolution Changed - parameter changed");
							SET_STATE(p_private, STATE_RESOLUTION_CHANGING_WITH_REINIT);

							if( CHECK_STATE(p_private, STATE_OUTPUT_PORT_FLUSHED) ){
								SET_STATE(p_private, STATE_READY_TO_RESET);
							}
						}
					}
				}
#if RESOLUTION_CHANGE_WITHOUT_EVENT
				else
#endif
				{
					p_private->stCropRect.nLeft     = 0;
					p_private->stCropRect.nTop      = 0;
					p_private->stCropRect.nWidth    = port->sPortParam.format.video.nFrameWidth;
					p_private->stCropRect.nHeight   = port->sPortParam.format.video.nFrameHeight;

					if( UpdateFrameSize(p_private) < 0 ){
						eError = OMX_ErrorUnsupportedSetting;
						break;
					}
				}
			}
		}
		break;

	case OMX_IndexParamVideoPortFormat:
		{
			OMX_VIDEO_PARAM_PORTFORMATTYPE *pVideoPortFormat = ComponentParameterStructure;
			OMX_U32 portIndex = pVideoPortFormat->nPortIndex;
			omx_base_video_PortType *port = (omx_base_video_PortType *)p_private->ports[portIndex];

			/*Check Structure Header and verify component state*/
			eError = omx_base_component_ParameterSanityCheck(hComponent, portIndex, pVideoPortFormat, sizeof(OMX_VIDEO_PARAM_PORTFORMATTYPE));
			if (eError != OMX_ErrorNone) {
				LOGE("In %s Parameter Check Error=%x", __func__, eError);
				break;
			}

			(void)memcpy((void*)&port->sVideoParam, (void*)pVideoPortFormat, sizeof(OMX_VIDEO_PARAM_PORTFORMATTYPE));

			if (portIndex == OMX_BASE_FILTER_OUTPUTPORT_INDEX) {
				p_private->ports[portIndex]->sPortParam.format.video.eColorFormat = port->sVideoParam.eColorFormat;
				if( UpdateFrameSize(p_private) < 0 ){
					eError = OMX_ErrorUnsupportedSetting;
					break;
				}
			} else {
				eError = OMX_ErrorBadPortIndex;
				break;
			}
		}
		break;

	case OMX_IndexParamStandardComponentRole:
		{
			OMX_VIDEO_CODINGTYPE coding_type;
			OMX_PARAM_COMPONENTROLETYPE *pComponentRole = (OMX_PARAM_COMPONENTROLETYPE*)ComponentParameterStructure;

			coding_type = GetCodingTypeFromRole(pComponentRole->cRole);
			if (coding_type == OMX_VIDEO_CodingUnused) {
				eError = OMX_ErrorBadParameter;
				break;
			}

			p_private->enVideoCodingType = coding_type;
			if (SetInternalVideoParameters(p_private) == FALSE){
				eError = OMX_ErrorNotImplemented;
				break;
			}
		}
		break;

	case OMX_IndexParamVideoProfileLevelCurrent:
		{
			OMX_VIDEO_PARAM_PROFILELEVELTYPE *pProfileLevel = (OMX_VIDEO_PARAM_PROFILELEVELTYPE*)ComponentParameterStructure;

			if (p_private->enVideoCodingType == OMX_VIDEO_CodingWMV && p_private->stVDecInit.m_iBitstreamFormat == STD_VC1) {
				p_private->stProfileLevel.eProfile = pProfileLevel->eProfile;
				p_private->stProfileLevel.eLevel   = pProfileLevel->eLevel;

				INFO("WMV9 Profile: %s"
					 , pProfileLevel->eProfile == OMX_VIDEO_WMV9ProfileSimple ? "OMX_VIDEO_WMV9ProfileSimple" :
					   pProfileLevel->eProfile == OMX_VIDEO_WMV9ProfileMain ? "OMX_VIDEO_WMV9ProfileMain" :
					   pProfileLevel->eProfile == OMX_VIDEO_WMV9ProfileAdvanced ? "OMX_VIDEO_WMV9ProfileAdvanced" :
					   "unknown");
			}
		}
		break;

	case OMX_IndexParamVideoMsMpeg4Version:
		{
			OMX_VIDEO_PARAM_MSMPEG4VERSIONTYPE *p_msmpeg4_param = (OMX_VIDEO_PARAM_MSMPEG4VERSIONTYPE*)ComponentParameterStructure;

			p_private->eCodecVersion = p_msmpeg4_param->eVersion;
			if (p_msmpeg4_param->eVersion == OMX_VIDEO_MSMPEG4Version3) {
				INFO("Microsoft MPEG-4 version 3");
				if (SetInternalVideoParameters(p_private) == FALSE) {
					eError = OMX_ErrorNotImplemented;
					break;
				}
			}
			else {
				eError = OMX_ErrorNotImplemented;
				break;
			}
		}
		break;

	case OMX_IndexParamVideoDivxVersion:
		{
			OMX_VIDEO_PARAM_DIVXVERSIONTYPE *p_divx_param = (OMX_VIDEO_PARAM_DIVXVERSIONTYPE*)ComponentParameterStructure;

			p_private->eCodecVersion = p_divx_param->eVersion;
			if (p_divx_param->eVersion == OMX_VIDEO_DIVXVersion3) {
				INFO("Divx version 3");
				if (SetInternalVideoParameters(p_private) == FALSE) {
					eError = OMX_ErrorNotImplemented;
					break;
				}
			}
		}
		break;

	case OMX_IndexParamNalStreamFormatSelect:
		{
			OMX_NALSTREAMFORMATTYPE *p_nalformat = (OMX_NALSTREAMFORMATTYPE *)ComponentParameterStructure;

			switch (p_nalformat->eNaluFormat) {
			case OMX_NaluFormatOneByteInterleaveLength:
				SET_STATE(p_private, STATE_STREAM_CONVERTION_NEEDED);
				p_private->lNalLengthSize = 1;
				break;
			case OMX_NaluFormatTwoByteInterleaveLength:
				SET_STATE(p_private, STATE_STREAM_CONVERTION_NEEDED);
				p_private->lNalLengthSize = 2;
				break;
			case OMX_NaluFormatFourByteInterleaveLength:
				SET_STATE(p_private, STATE_STREAM_CONVERTION_NEEDED);
				p_private->lNalLengthSize = 4;
				break;
			default:
				CLEAR_STATE(p_private, STATE_STREAM_CONVERTION_NEEDED);
				p_private->lNalLengthSize = 0;
				break;
			}
		}
		break;

	default: /*Call the base component function*/
		eError = omx_base_component_SetParameter(hComponent, nParamIndex, ComponentParameterStructure);
		break;
	}
	return eError;
}

static
OMX_ERRORTYPE
omx_videodec_component_GetParameter(
	OMX_IN  	OMX_HANDLETYPE 	hComponent,
	OMX_IN  	OMX_INDEXTYPE 	nParamIndex,
	OMX_INOUT 	OMX_PTR 	ComponentParameterStructure
	)
{
	vdec_private_t* p_private = GET_VIDEODEC_PRIVATE(hComponent);
	omx_base_video_PortType *port;
	OMX_ERRORTYPE eError = OMX_ErrorNone;

	LOG_STEP("In %s ", __func__);

	if (ComponentParameterStructure == NULL) {
		return OMX_ErrorBadParameter;
	}

	/* Check which structure we are being fed and fill its header */
	switch (nParamIndex)
	{
		case OMX_IndexParamVideoAvc:
		{
			OMX_VIDEO_PARAM_AVCTYPE *pst_param;
			pst_param = ComponentParameterStructure;
			if (pst_param->nPortIndex != 0) {
				eError = OMX_ErrorBadPortIndex;
				break;
			}
			if ((eError = checkHeader(pst_param, sizeof(OMX_VIDEO_PARAM_AVCTYPE))) != OMX_ErrorNone) {
				break;
			}
			(void)memcpy((void*)pst_param, (void*)p_private->pVideoParam, sizeof(OMX_VIDEO_PARAM_AVCTYPE));
		}
		break;

		case OMX_IndexParamVideoMpeg4:
		{
			OMX_VIDEO_PARAM_MPEG4TYPE *pst_param;
			pst_param = ComponentParameterStructure;
			if (pst_param->nPortIndex != 0) {
				eError = OMX_ErrorBadPortIndex;
				break;
			}
			if ((eError = checkHeader(pst_param, sizeof(OMX_VIDEO_PARAM_MPEG4TYPE))) != OMX_ErrorNone) {
				break;
			}
			(void)memcpy((void*)pst_param, (void*)p_private->pVideoParam, sizeof(OMX_VIDEO_PARAM_MPEG4TYPE));
		}
		break;

		case OMX_IndexParamVideoWmv: //vc-1
		{
			OMX_VIDEO_PARAM_WMVTYPE *pst_param;
			pst_param = ComponentParameterStructure;
			if (pst_param->nPortIndex != 0) {
				eError = OMX_ErrorBadPortIndex;
				break;
			}
			if ((eError = checkHeader(pst_param, sizeof(OMX_VIDEO_PARAM_WMVTYPE))) != OMX_ErrorNone) {
				break;
			}
			(void)memcpy((void*)pst_param, (void*)p_private->pVideoParam, sizeof(OMX_VIDEO_PARAM_WMVTYPE));
		}
		break;

		case OMX_IndexParamVideoInit:
		{
			if ((eError = checkHeader(ComponentParameterStructure, sizeof(OMX_PORT_PARAM_TYPE))) != OMX_ErrorNone) {
				break;
			}
			(void)memcpy((void*)ComponentParameterStructure, (void*)&p_private->sPortTypesParam[OMX_PortDomainVideo], sizeof(OMX_PORT_PARAM_TYPE));
		}
		break;

		case OMX_IndexParamVideoPortFormat:
		{
			OMX_VIDEO_PARAM_PORTFORMATTYPE *pVideoPortFormat;
			pVideoPortFormat = ComponentParameterStructure;
			if ((eError = checkHeader(ComponentParameterStructure, sizeof(OMX_VIDEO_PARAM_PORTFORMATTYPE))) != OMX_ErrorNone) {
				break;
			}

			if (pVideoPortFormat->nIndex != 0) {
				eError = OMX_ErrorNoMore;
				break;
			}

			if (pVideoPortFormat->nPortIndex <= 1) {
				port = (omx_base_video_PortType *)p_private->ports[pVideoPortFormat->nPortIndex];
				(void)memcpy((void*)pVideoPortFormat, (void*)&port->sVideoParam, sizeof(OMX_VIDEO_PARAM_PORTFORMATTYPE));
			} else {
				eError = OMX_ErrorBadPortIndex;
			}
			break;
		}

		case OMX_IndexParamStandardComponentRole:
		{
			OMX_PARAM_COMPONENTROLETYPE *pComponentRole;
			pComponentRole = ComponentParameterStructure;
			if ((eError = checkHeader(ComponentParameterStructure, sizeof(OMX_PARAM_COMPONENTROLETYPE))) != OMX_ErrorNone) {
				break;
			}

			switch( p_private->enVideoCodingType )
			{
				case OMX_VIDEO_CodingH263:
					(void)strncpy((char *)pComponentRole->cRole, VIDEO_DEC_H263_ROLE, sizeof(VIDEO_DEC_H263_ROLE));
					break;
				case OMX_VIDEO_CodingAVC:
					(void)strncpy((char *)pComponentRole->cRole, VIDEO_DEC_H264_ROLE, sizeof(VIDEO_DEC_H264_ROLE));
					break;
				case OMX_VIDEO_CodingMPEG4:
					(void)strncpy((char *)pComponentRole->cRole, VIDEO_DEC_MPEG4_ROLE, sizeof(VIDEO_DEC_MPEG4_ROLE));
					break;
				case OMX_VIDEO_CodingWMV:
					(void)strncpy((char *)pComponentRole->cRole, VIDEO_DEC_WMV_ROLE, sizeof(VIDEO_DEC_WMV_ROLE));
					break;
				case OMX_VIDEO_CodingDIVX:
					(void)strncpy((char *)pComponentRole->cRole, VIDEO_DEC_DIVX_ROLE, sizeof(VIDEO_DEC_DIVX_ROLE));
					break;
				case OMX_VIDEO_CodingMPEG2:
					(void)strncpy((char *)pComponentRole->cRole, VIDEO_DEC_MPEG2_ROLE, sizeof(VIDEO_DEC_MPEG2_ROLE));
					break;
				case OMX_VIDEO_CodingFLV1:
					(void)strncpy((char *)pComponentRole->cRole, VIDEO_DEC_SORENSON_H263_ROLE, sizeof(VIDEO_DEC_SORENSON_H263_ROLE));
					break;
				case OMX_VIDEO_CodingMJPEG:
					(void)strncpy((char *)pComponentRole->cRole, VIDEO_DEC_MJPEG_ROLE, sizeof(VIDEO_DEC_MJPEG_ROLE));
					break;
				case OMX_VIDEO_CodingVP8:
					(void)strncpy((char *)pComponentRole->cRole, VIDEO_DEC_VP8_ROLE, sizeof(VIDEO_DEC_VP8_ROLE));
					break;
#if defined (TCC_VPU_4K_D2_INCLUDE)
				case OMX_VIDEO_CodingVP9:
					(void)strncpy((char *)pComponentRole->cRole, VIDEO_DEC_VP9_ROLE, sizeof(VIDEO_DEC_VP9_ROLE));
					break;
#endif
				case OMX_VIDEO_CodingMVC:
					(void)strncpy((char *)pComponentRole->cRole, VIDEO_DEC_MVC_ROLE, sizeof(VIDEO_DEC_MVC_ROLE));
					break;
#ifdef TCC_EXT_INCLUDED
				case OMX_VIDEO_CodingRV:
					(void)strncpy((char *)pComponentRole->cRole, VIDEO_DEC_RV_ROLE, sizeof(VIDEO_DEC_RV_ROLE));
					break;
#endif
#ifdef TCC_AVS_INCLUDED
				case OMX_VIDEO_CodingAVS:
					(void)strncpy((char *)pComponentRole->cRole, VIDEO_DEC_AVS_ROLE, sizeof(VIDEO_DEC_AVS_ROLE));
					break;
#endif
#if defined (TCC_HEVC_INCLUDE) || defined (TCC_VPU_4K_D2_INCLUDE)
				case OMX_VIDEO_CodingHEVC:
					(void)strncpy((char *)pComponentRole->cRole, VIDEO_DEC_H265_ROLE, sizeof(VIDEO_DEC_H265_ROLE));
					break;
#endif
				default:
					(void)strncpy((char *)pComponentRole->cRole, "\0", sizeof("\0"));
					break;
			}
		}
		break;

		case OMX_IndexParamVideoProfileLevelQuerySupported:
		{
			OMX_VIDEO_PARAM_PROFILELEVELTYPE *profileLevel = (OMX_VIDEO_PARAM_PROFILELEVELTYPE *)ComponentParameterStructure;
			const VIDEO_PROFILE_LEVEL_TYPE *pProfileLevel = NULL;
			OMX_U32 nNumberOfProfiles = 0;

			if (profileLevel->nPortIndex != 0) {
				LOGE("Invalid port index: %ld", profileLevel->nPortIndex);
				eError = OMX_ErrorUnsupportedIndex;
				break;
			}

			/* Choose table based on compression format */
			switch (p_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat)
			{
			case OMX_VIDEO_CodingH263:
				pProfileLevel = SupportedH263ProfileLevels;
				nNumberOfProfiles = sizeof(SupportedH263ProfileLevels) / sizeof(VIDEO_PROFILE_LEVEL_TYPE);
				break;
			case OMX_VIDEO_CodingMPEG4:
				pProfileLevel = SupportedMPEG4ProfileLevels;
				nNumberOfProfiles = sizeof(SupportedMPEG4ProfileLevels) / sizeof(VIDEO_PROFILE_LEVEL_TYPE);
				break;
			case OMX_VIDEO_CodingAVC:
				pProfileLevel = SupportedAVCProfileLevels;
				nNumberOfProfiles = sizeof(SupportedAVCProfileLevels) / sizeof(VIDEO_PROFILE_LEVEL_TYPE);
				break;
			case OMX_VIDEO_CodingMPEG2:
				pProfileLevel = SupportedMPEG2ProfileLevels;
				nNumberOfProfiles = sizeof(SupportedMPEG2ProfileLevels) / sizeof (VIDEO_PROFILE_LEVEL_TYPE);
				break;
			case OMX_VIDEO_CodingVP8:
				pProfileLevel = SupportedVP8ProfileLevels;
				nNumberOfProfiles = sizeof(SupportedVP8ProfileLevels) / sizeof (VIDEO_PROFILE_LEVEL_TYPE);
				break;
#if defined (TCC_VPU_4K_D2_INCLUDE)
			case OMX_VIDEO_CodingVP9:
				//pProfileLevel = SupportedVP8ProfileLevels;
				//nNumberOfProfiles = sizeof(SupportedVP8ProfileLevels) / sizeof (VIDEO_PROFILE_LEVEL_TYPE);
				break;
#endif
#if defined (TCC_HEVC_INCLUDE) || defined (TCC_VPU_4K_D2_INCLUDE)
			case OMX_VIDEO_CodingHEVC:
				//TODO: profile/level
				break;
#endif
			default:
				eError = OMX_ErrorBadParameter;
				break;
			}

			if (eError != OMX_ErrorNone){
				break;
			}

			if (profileLevel->nProfileIndex >= nNumberOfProfiles) {
				eError = OMX_ErrorNoMore;
				break;
			}

			profileLevel->eProfile = pProfileLevel[profileLevel->nProfileIndex].nProfile;
			profileLevel->eLevel = pProfileLevel[profileLevel->nProfileIndex].nLevel;

//			eError = OMX_ErrorNone;
		}
		break;

		case OMX_IndexParamCommonDeblocking:
		{
			break;
		}

		case OMX_IndexParamVideoStride:
		{
			OMX_VIDEO_PARAM_STRIDETYPE *p_stride = (OMX_VIDEO_PARAM_STRIDETYPE *)ComponentParameterStructure;
			omx_base_video_PortType *p_outport = (omx_base_video_PortType *)p_private->ports[OMX_BASE_FILTER_OUTPUTPORT_INDEX];
			vdec_initial_info_t *p_init_info = p_private->stVDecOutput.m_pInitialInfo;
#if defined(TCC_HEVC_INCLUDE) || defined (TCC_VPU_4K_D2_INCLUDE)
			if (p_private->enVideoCodingType == OMX_VIDEO_CodingHEVC) {
				p_stride->nStride = ((p_init_info->m_iPicWidth+31) >> 5) << 5;;
				p_stride->nStrideY = ((p_init_info->m_iPicWidth+31) >> 5) << 5;;
				if( p_outport->sVideoParam.eColorFormat == OMX_COLOR_FormatYUV420Planar ||
					p_outport->sVideoParam.eColorFormat == OMX_COLOR_FormatYUV420PlanarTc ){
					p_stride->nStrideCbCr = (((p_init_info->m_iPicWidth/2)+15) >> 4) << 4;
				}
				else if( p_outport->sVideoParam.eColorFormat == OMX_COLOR_FormatYUV420SemiPlanar ||
						 p_outport->sVideoParam.eColorFormat == OMX_COLOR_FormatYUV420SemiPlanarTc ){
					p_stride->nStrideCbCr = ((p_init_info->m_iPicWidth+15) >> 4) << 4;
				}
			}
			else
#endif
#if defined (TCC_VPU_4K_D2_INCLUDE)
			if (p_private->enVideoCodingType == OMX_VIDEO_CodingVP9 ) {
				p_stride->nStride = ((p_init_info->m_iPicWidth+31) >> 5) << 5;;
				p_stride->nStrideY = ((p_init_info->m_iPicWidth+31) >> 5) << 5;;
				if( p_outport->sVideoParam.eColorFormat == OMX_COLOR_FormatYUV420Planar ||
					p_outport->sVideoParam.eColorFormat == OMX_COLOR_FormatYUV420PlanarTc ){
					p_stride->nStrideCbCr = (((p_init_info->m_iPicWidth/2)+15) >> 4) << 4;
				}
				else if( p_outport->sVideoParam.eColorFormat == OMX_COLOR_FormatYUV420SemiPlanar ||
						 p_outport->sVideoParam.eColorFormat == OMX_COLOR_FormatYUV420SemiPlanarTc ){
					p_stride->nStrideCbCr = ((p_init_info->m_iPicWidth+15) >> 4) << 4;
				}
			}
			else
#endif
			{
				p_stride->nStride = ((p_init_info->m_iPicWidth+15) >> 4) << 4;;
				p_stride->nStrideY = ((p_init_info->m_iPicWidth+15) >> 4) << 4;;
				if( p_outport->sVideoParam.eColorFormat == OMX_COLOR_FormatYUV420Planar ||
					p_outport->sVideoParam.eColorFormat == OMX_COLOR_FormatYUV420PlanarTc ) {
					p_stride->nStrideCbCr = (((p_init_info->m_iPicWidth/2)+7) >> 3) << 3;
				}
				else if( p_outport->sVideoParam.eColorFormat == OMX_COLOR_FormatYUV420SemiPlanar ||
						 p_outport->sVideoParam.eColorFormat == OMX_COLOR_FormatYUV420SemiPlanarTc ) {
					p_stride->nStrideCbCr = ((p_init_info->m_iPicWidth+7) >> 3) << 3;
				}
			}
		}
		break;

		case OMX_IndexParamNalStreamFormatSelect:
		{
			OMX_NALSTREAMFORMATTYPE *p_nalformat = (OMX_NALSTREAMFORMATTYPE *)ComponentParameterStructure;

			switch (p_private->lNalLengthSize) {
			case 1:
				p_nalformat->eNaluFormat = OMX_NaluFormatOneByteInterleaveLength;
				break;
			case 2:
				p_nalformat->eNaluFormat = OMX_NaluFormatTwoByteInterleaveLength;
				break;
			case 4:
				p_nalformat->eNaluFormat = OMX_NaluFormatFourByteInterleaveLength;
				break;
			default:
				p_nalformat->eNaluFormat = OMX_NaluFormatStartCodes;
				break;
			}
		}
		break;

		case OMX_IndexParamVideoMacroblocksPerFrame:
		{
			OMX_PARAM_MACROBLOCKSTYPE *p_macroblocks = (OMX_PARAM_MACROBLOCKSTYPE*)ComponentParameterStructure;
			OMX_S32 mb_width = 0;
			OMX_S32 mb_height = 0;
			omx_base_video_PortType *p_outport = (omx_base_video_PortType *)p_private->ports[OMX_BASE_FILTER_OUTPUTPORT_INDEX];
			OMX_U32 port_width = p_outport->sPortParam.format.video.nFrameWidth;
			OMX_U32 port_height = p_outport->sPortParam.format.video.nFrameHeight;

			if( (port_width == 0) || (port_height == 0) ) {
				p_macroblocks->nMacroblocks = 0;
				break;
			}

			switch (p_private->enVideoCodingType) {
				case OMX_VIDEO_CodingH263:
				case OMX_VIDEO_CodingFLV1:
				case OMX_VIDEO_CodingMPEG2:
				case OMX_VIDEO_CodingMPEG4:
				case OMX_VIDEO_CodingMSMPEG4:
				case OMX_VIDEO_CodingAVC:
				case OMX_VIDEO_CodingDIVX:
				case OMX_VIDEO_CodingVP8:
				case OMX_VIDEO_CodingMVC:
#ifdef TCC_EXT_INCLUDED
				case OMX_VIDEO_CodingRV:
#endif
#ifdef TCC_AVS_INCLUDED
				case OMX_VIDEO_CodingAVS:
#endif
#if defined(INCLUDE_WMV78_DEC) || defined(INCLUDE_WMV9_DEC)
				case OMX_VIDEO_CodingWMV:
#endif
					mb_width = (port_width+15) >> 4;
					mb_height= (port_height+15) >> 4;
					break;
#if defined (TCC_VPU_4K_D2_INCLUDE)
				case OMX_VIDEO_CodingVP9:
#endif
#if defined (TCC_HEVC_INCLUDE) || defined (TCC_VPU_4K_D2_INCLUDE)
				case OMX_VIDEO_CodingHEVC:
					mb_width = (port_width+63) >> 6;
					mb_height= (port_height+63) >> 6;
					break;
#endif

				case OMX_VIDEO_CodingMJPEG:
					mb_width = (port_width+7) >> 3;
					mb_height = (port_height+7) >> 3;
					break;

				default:
					mb_width = mb_height = 0;
					break;
			}

			p_macroblocks->nMacroblocks = mb_width * mb_height;
		}
		break;
#if 0
		case OMX_IndexParamPortDefinition:
		{
			OMX_PARAM_PORTDEFINITIONTYPE *pPortDef  = (OMX_PARAM_PORTDEFINITIONTYPE *)ComponentParameterStructure;
			eError = omx_base_component_GetParameter(hComponent, nParamIndex, ComponentParameterStructure);
			INFO("[OMX_COM] cMIMEType             : %s ", pPortDef->format.video.cMIMEType);
			INFO("[OMX_COM] nFrameWidth           : %d ", pPortDef->format.video.nFrameWidth);
			INFO("[OMX_COM] nFrameHeight          : %d ", pPortDef->format.video.nFrameHeight);
			INFO("[OMX_COM] nStride               : %d ", pPortDef->format.video.nStride);
			INFO("[OMX_COM] nSliceHeight          : %d ", pPortDef->format.video.nSliceHeight);
			INFO("[OMX_COM] nBitrate              : %d ", pPortDef->format.video.nBitrate);
			INFO("[OMX_COM] xFramerate            : %d ", pPortDef->format.video.xFramerate);
			INFO("[OMX_COM] bFlagErrorConcealment : %d ", pPortDef->format.video.bFlagErrorConcealment);
			INFO("[OMX_COM] eCompressionFormat    : %d ", pPortDef->format.video.eCompressionFormat);
			INFO("[OMX_COM] eColorFormat          : 0x%08X ", pPortDef->format.video.eColorFormat);
			INFO("[OMX_COM] eError                : %d ", eError);
			return eError;
		}
		break;
#endif

		default: /*Call the base component function*/
		{
			eError = omx_base_component_GetParameter(hComponent, nParamIndex, ComponentParameterStructure);
		}
		break;
	}
	return eError;
}

static
OMX_ERRORTYPE
omx_videodec_component_MessageHandler(
	OMX_COMPONENTTYPE           *openmaxStandComp,
	internalRequestMessageType  *message
	)
{
  vdec_private_t* p_private = GET_VIDEODEC_PRIVATE(openmaxStandComp);
  OMX_ERRORTYPE err;
  OMX_STATETYPE eCurrentState = p_private->state;

  LOG_STEP("In %s ", __func__);

  if (message->messageType == OMX_CommandStateSet){
    if ((message->messageParam == OMX_StateExecuting) && (p_private->state == OMX_StateIdle)) {
      if (!CHECK_STATE(p_private, STATE_RESOURCE_INITIALIZED)) {
        err = omx_videodec_component_InitResource(p_private);
        if (err != OMX_ErrorNone) {
          return OMX_ErrorNotReady;
        }
		SET_STATE(p_private, STATE_RESOURCE_INITIALIZED);
      }
    }
    else if ((message->messageParam == OMX_StateIdle) && (p_private->state == OMX_StateLoaded)) {
      err = omx_videodec_component_Initialize(openmaxStandComp);
      if(err!=OMX_ErrorNone) {
        LOGE( "In %s Video Decoder Init Failed Error=%x",__func__,err);
        return err;
      }
    } else if ((message->messageParam == OMX_StateLoaded) && (p_private->state == OMX_StateIdle)) {
      err = omx_videodec_component_Deinitialize(openmaxStandComp);
      if(err!=OMX_ErrorNone) {
        LOGE( "In %s Video Decoder Deinit Failed Error=%x",__func__,err);
        return err;
      }
    }
  }

  // Execute the base message handling
  err = omx_base_component_MessageHandler(openmaxStandComp,message);

  if (message->messageType == OMX_CommandStateSet){
   if ((message->messageParam == OMX_StateIdle) && (eCurrentState == OMX_StateExecuting || eCurrentState == OMX_StatePause)) {
	   if (CHECK_STATE(p_private, STATE_RESOURCE_INITIALIZED)){
		   omx_videodec_component_DeinitResource(p_private);
		   CLEAR_STATE(p_private, STATE_RESOURCE_INITIALIZED);
	   }
    }
  }

  // flush all ports to start new stream
  if (message->messageType == OMX_CommandFlush) {
    switch (message->messageParam) {
    case OMX_BASE_FILTER_OUTPUTPORT_INDEX:
      SET_STATE(p_private, STATE_OUTPUT_PORT_FLUSHED);
      break;
    case OMX_ALL:
    //case OMX_BASE_FILTER_ALLPORT_INDEX: // == OMX_ALL
      SET_STATE(p_private, STATE_OUTPUT_PORT_FLUSHED);
      p_private->pbyVDecInputPtr = NULL;
      p_private->lVDecInputLength = 0;
      break;
    case OMX_BASE_FILTER_INPUTPORT_INDEX:
      p_private->pbyVDecInputPtr = NULL;
      p_private->lVDecInputLength = 0;
      break;
    }
    INFO("port flushed - index: %d", message->messageParam);
  }

  return err;
}

#define COPY_IF_MATCHED(pszComponent, component_name, copy_data, des_add) \
	{if ((pszComponent) == (component_name)) {\
		(void)strncpy((char *)(des_add), (copy_data), sizeof((copy_data))); \
	}}\

static
OMX_ERRORTYPE
omx_videodec_component_ComponentRoleEnum(
	OMX_IN  OMX_HANDLETYPE   hComponent,
	OMX_OUT OMX_U8          *cRole,
	OMX_IN  OMX_U32          nIndex
	)
{
	vdec_private_t* p_private = GET_VIDEODEC_PRIVATE(hComponent);
	OMX_ERRORTYPE err = OMX_ErrorNone;

	LOG_STEP("In %s ", __func__);

	if (nIndex == 0) {
		COPY_IF_MATCHED(p_private->enVideoCodingType, OMX_VIDEO_CodingH263, VIDEO_DEC_H263_ROLE, cRole);
		COPY_IF_MATCHED(p_private->enVideoCodingType, OMX_VIDEO_CodingAVC, VIDEO_DEC_H264_ROLE, cRole);

		COPY_IF_MATCHED(p_private->enVideoCodingType, OMX_VIDEO_CodingMPEG4, VIDEO_DEC_MPEG4_ROLE, cRole);
		COPY_IF_MATCHED(p_private->enVideoCodingType, OMX_VIDEO_CodingWMV, VIDEO_DEC_WMV_ROLE, cRole);
		COPY_IF_MATCHED(p_private->enVideoCodingType, OMX_VIDEO_CodingDIVX, VIDEO_DEC_DIVX_ROLE, cRole);
		COPY_IF_MATCHED(p_private->enVideoCodingType, OMX_VIDEO_CodingMPEG2, VIDEO_DEC_MPEG2_ROLE, cRole);
		COPY_IF_MATCHED(p_private->enVideoCodingType, OMX_VIDEO_CodingFLV1, VIDEO_DEC_SORENSON_H263_ROLE, cRole);
		COPY_IF_MATCHED(p_private->enVideoCodingType, OMX_VIDEO_CodingMJPEG, VIDEO_DEC_MJPEG_ROLE, cRole);
		COPY_IF_MATCHED(p_private->enVideoCodingType, OMX_VIDEO_CodingVP8, VIDEO_DEC_VP8_ROLE, cRole);
		COPY_IF_MATCHED(p_private->enVideoCodingType, OMX_VIDEO_CodingMVC, VIDEO_DEC_MVC_ROLE, cRole);
#if defined (TCC_VPU_4K_D2_INCLUDE)
		COPY_IF_MATCHED(p_private->enVideoCodingType, OMX_VIDEO_CodingVP9, VIDEO_DEC_VP9_ROLE, cRole);
#endif
#ifdef TCC_EXT_INCLUDED
		COPY_IF_MATCHED(p_private->enVideoCodingType, OMX_VIDEO_CodingRV, VIDEO_DEC_RV_ROLE, cRole);
#endif
#ifdef TCC_AVS_INCLUDED
		COPY_IF_MATCHED(p_private->enVideoCodingType, OMX_VIDEO_CodingAVS, VIDEO_DEC_AVS_ROLE, cRole);
#endif
#if defined (TCC_HEVC_INCLUDE) || defined (TCC_VPU_4K_D2_INCLUDE)
		COPY_IF_MATCHED(p_private->enVideoCodingType, OMX_VIDEO_CodingHEVC, VIDEO_DEC_H265_ROLE, cRole);
#endif
	} else {
		err = OMX_ErrorUnsupportedIndex;
	}

	return err;
}

static
OMX_ERRORTYPE
omx_videodec_component_SetConfig(
	OMX_HANDLETYPE   hComponent,
	OMX_INDEXTYPE    nIndex,
	OMX_PTR          pComponentConfigStructure
	)
{
	vdec_private_t *p_private = GET_VIDEODEC_PRIVATE(hComponent);
	OMX_ERRORTYPE err = OMX_ErrorNone;

	LOG_STEP("In %s ", __func__);

	if (pComponentConfigStructure == NULL) {
		return OMX_ErrorBadParameter;
	}

/* Check which structure we are being fed and fill its header */
	switch (nIndex)
	{
		case OMX_IndexConfigTcDispBufIndex:
		{
			OMX_U32	nBufIndex = *((OMX_U32 *)pComponentConfigStructure);
			if(p_private)
			{
				IncDispBufCount(&p_private->stDispIdxQueue,nBufIndex);
			}
		}
		break;

		case OMX_IndexConfigVideoMBErrorReporting:
		{
			OMX_CONFIG_MBERRORREPORTINGTYPE *p_config = (OMX_CONFIG_MBERRORREPORTINGTYPE*)pComponentConfigStructure;
			if( p_config->bEnabled == OMX_TRUE ) {
				SET_MODE(p_private, MODE_MB_ERROR_REPORTING);
			} else {
				CLEAR_MODE(p_private, MODE_MB_ERROR_REPORTING);
			}
		}
		break;

		case OMX_IndexConfigMergeAvcSlice:
		if( *((OMX_BOOL *)pComponentConfigStructure) ) {
			SET_MODE(p_private, MODE_SLICE_MERGING);
		} else {
			CLEAR_MODE(p_private, MODE_SLICE_MERGING);
		}
		break;

		case OMX_IndexConfigVideoThumbnailMode:
		err = OMX_ErrorNotImplemented;
		break;

		case OMX_IndexConfigCurrentFrameOutput:
		if( *((OMX_BOOL *)pComponentConfigStructure) ) {
			SET_MODE(p_private, MODE_DECODED_FRAME_OUTPUT);
		} else {
			CLEAR_MODE(p_private, MODE_DECODED_FRAME_OUTPUT);
		}
		break;

		case OMX_IndexConfigCurrentKeyframeOutput:
		if( *((OMX_BOOL *)pComponentConfigStructure) ) {
			SET_MODE(p_private, MODE_DECODED_KEYFRAME_OUTPUT);
		} else {
			CLEAR_MODE(p_private, MODE_DECODED_KEYFRAME_OUTPUT);
		}
		break;

#if 0
		case OMX_IndexConfigVideoPlayDirection:
		if (*(OMX_BOOL *)pComponentConfigStructure) {
			p_private->stDispInfoMgr.ulFlags &= ~DISPMGR_BACKWARD_PLAYBACK;
		} else {
			p_private->stDispInfoMgr.ulFlags |= DISPMGR_BACKWARD_PLAYBACK;
		}
		break;

		case OMX_IndexConfigVideoOutputKeyFrameOnly:
		if( *(OMX_BOOL*)pComponentConfigStructure ) {
			SET_MODE(p_private, MODE_DECODED_FRAME_OUTPUT);
		} else {
			CLEAR_MODE(p_private, MODE_DECODED_FRAME_OUTPUT);
		}
		break;
#endif

		case OMX_IndexConfigVideoSequenceInitFailMax:
		p_private->lSeqInitFailMax = *(OMX_U32*)pComponentConfigStructure;
		break;

		case OMX_IndexConfigDecodingErrorReporting:
		{
			OMX_CONFIG_DECODINGERRORREPORTINGTYPE *p_config = (OMX_CONFIG_DECODINGERRORREPORTINGTYPE*)pComponentConfigStructure;
			if( p_config->bEnable == TRUE ) {
				SET_MODE(p_private, MODE_DECODING_ERROR_REPORTING);
			} else {
				CLEAR_MODE(p_private, MODE_DECODING_ERROR_REPORTING);
			}

			if( p_config->bRepeat == OMX_TRUE ) {
				SET_MODE(p_private, MODE_DECODING_ERROR_REPORTING_REPEAT);
			} else {
				CLEAR_MODE(p_private, MODE_DECODING_ERROR_REPORTING_REPEAT);
			}

			p_private->lDecodingFailMax = p_config->nPeriod;
			if( p_private->lDecodingFailMax == 0 ) {
				CLEAR_MODE(p_private, MODE_DECODING_ERROR_REPORTING);
			}
		}
		break;

		case OMX_IndexConfigUserDataHandling:
		if( *((OMX_BOOL *)pComponentConfigStructure) ) {
			SET_MODE(p_private, MODE_USERDATA_HANDLING);
		} else {
			CLEAR_MODE(p_private, MODE_USERDATA_HANDLING);
		}
		break;

		case OMX_IndexConfigVideoRingModeEnable:
		if( *((OMX_BOOL *)pComponentConfigStructure) ) {
			SET_MODE(p_private, MODE_RINGBUFFER_MODE);
		} else {
			CLEAR_MODE(p_private, MODE_RINGBUFFER_MODE);
		}
		break;

		case OMX_IndexConfigVideoIFrameOnlyOutput:
		if( *((OMX_BOOL *)pComponentConfigStructure) ) {
			SET_MODE(p_private, MODE_I_FRAME_ONLY_OUTPUT);
		} else {
			CLEAR_MODE(p_private, MODE_I_FRAME_ONLY_OUTPUT);
		}
		break;

		case OMX_IndexConfigEventFlush:
		{
			int *start = (int*)pComponentConfigStructure;
			p_private->bEventFlush = *start;
		}
		break;

		case OMX_IndexDecoderSecuredInBuffers:
		if( (p_private->mSeqBackupPmap.iSize == 0) || (p_private->mTmem_fd <= 0) ) {
			CLEAR_STATE(p_private, STATE_VDEC_BUFFER_PROTECTION);
		} else {
			SET_STATE(p_private, STATE_VDEC_BUFFER_PROTECTION);
		}
		break;
		case OMX_IndexConfigTcProperty: // 170724.1.no-buffer-delay
		{
			OMX_U32          nProperty = *((OMX_U32 *)pComponentConfigStructure);
			if((nProperty & EXT_FUNC_NO_BUFFER_DELAY) != 0)
			{
				SET_MODE(p_private, MODE_EXT_FUNC_NO_BUFFER_DELAY);
			}
			else
			{
				CLEAR_MODE(p_private, MODE_EXT_FUNC_NO_BUFFER_DELAY);
			}
		}
		break;
		default: // delegate to superclass
		err=  omx_base_component_SetConfig(hComponent, nIndex, pComponentConfigStructure);
		break;
        }
        return err;
}

static
OMX_ERRORTYPE
omx_videodec_component_GetConfig(
	OMX_HANDLETYPE   hComponent,
	OMX_INDEXTYPE    nIndex,
	OMX_PTR          pComponentConfigStructure
	)
{
	vdec_private_t* p_private = GET_VIDEODEC_PRIVATE(hComponent);
	OMX_ERRORTYPE err = OMX_ErrorNone;

	LOG_STEP("In %s ", __func__);

	if (pComponentConfigStructure == NULL) {
		return OMX_ErrorBadParameter;
	}

	/* Check which structure we are being fed and fill its header */
	switch (nIndex) {
	case OMX_IndexConfigVideoMBErrorReporting:
		{
			OMX_CONFIG_MBERRORREPORTINGTYPE *p_config = (OMX_CONFIG_MBERRORREPORTINGTYPE*)pComponentConfigStructure;
			if( CHECK_MODE(p_private, MODE_MB_ERROR_REPORTING) ) {
				p_config->bEnabled = OMX_TRUE;
			} else {
				p_config->bEnabled = OMX_FALSE;
			}
		}
		break;

	case OMX_IndexConfigCommonOutputCrop:
		{
			OMX_CONFIG_RECTTYPE *rectParams = (OMX_CONFIG_RECTTYPE *)pComponentConfigStructure;
			if (rectParams->nPortIndex != 1) {
				err = OMX_ErrorUndefined;
				break;
			}
			rectParams->nLeft 	= p_private->stCropRect.nLeft;
			rectParams->nTop 	= p_private->stCropRect.nTop;
			rectParams->nWidth 	= p_private->stCropRect.nWidth;
			rectParams->nHeight = p_private->stCropRect.nHeight;
		}
		break;

	case OMX_IndexConfigMergeAvcSlice:
		if( CHECK_MODE(p_private, MODE_SLICE_MERGING) ) {
			*((OMX_BOOL *)pComponentConfigStructure) = TRUE;
		} else {
			*((OMX_BOOL *)pComponentConfigStructure) = FALSE;
		}
		break;

	case OMX_IndexConfigVideoThumbnailMode:
		return OMX_ErrorNotImplemented;

	case OMX_IndexConfigCurrentFrameOutput:
		if( CHECK_MODE(p_private, MODE_DECODED_FRAME_OUTPUT) ) {
			*((OMX_BOOL *)pComponentConfigStructure) = TRUE;
		} else {
			*((OMX_BOOL *)pComponentConfigStructure) = FALSE;
		}
		break;

	case OMX_IndexConfigCurrentKeyframeOutput:
		if( CHECK_MODE(p_private, MODE_DECODED_KEYFRAME_OUTPUT) ) {
			*((OMX_BOOL *)pComponentConfigStructure) = TRUE;
		} else {
			*((OMX_BOOL *)pComponentConfigStructure) = FALSE;
		}
		break;

	case OMX_IndexConfigVideoSequenceInitFailMax:
		*(OMX_U32*)pComponentConfigStructure = p_private->lSeqInitFailMax;
		break;

	case OMX_IndexConfigDecodingErrorReporting:
		{
			OMX_CONFIG_DECODINGERRORREPORTINGTYPE *p_config = (OMX_CONFIG_DECODINGERRORREPORTINGTYPE*)pComponentConfigStructure;
			if( CHECK_MODE(p_private, MODE_DECODING_ERROR_REPORTING) ) {
				p_config->bEnable = OMX_TRUE;
			} else {
				p_config->bEnable = OMX_FALSE;
			}

			if( CHECK_MODE(p_private, MODE_DECODING_ERROR_REPORTING_REPEAT) ) {
				p_config->bRepeat = OMX_TRUE;
			} else {
				p_config->bRepeat = OMX_FALSE;
			}

			p_config->nPeriod = p_private->lDecodingFailMax;
		}
		break;

	case OMX_IndexConfigVideoRingModeEnable:
		if( CHECK_MODE(p_private, MODE_RINGBUFFER_MODE) ) {
			*((OMX_BOOL *)pComponentConfigStructure) = TRUE;
		} else {
			*((OMX_BOOL *)pComponentConfigStructure) = FALSE;
		}
		break;

	case OMX_IndexConfigVideoIFrameOnlyOutput:
		if( CHECK_MODE(p_private, MODE_I_FRAME_ONLY_OUTPUT) ) {
			*((OMX_BOOL *)pComponentConfigStructure) = TRUE;
		} else {
			*((OMX_BOOL *)pComponentConfigStructure) = FALSE;
		}
		break;
	case OMX_IndexConfigTcProperty: // 170724.1.no-buffer-delay
	{
		OMX_U32          nProperty = 0;
		if( CHECK_MODE(p_private, MODE_EXT_FUNC_NO_BUFFER_DELAY) ){
			nProperty |= EXT_FUNC_NO_BUFFER_DELAY;
		}

		*((OMX_U32 *)pComponentConfigStructure) = nProperty;
	}
		break;
	case OMX_IndexConfigTcDispBufIndex:
	{
	}
		break;
	default: // delegate to superclass
		err = omx_base_component_GetConfig(hComponent, nIndex, pComponentConfigStructure);
		break;
	}

	return err;
}

static
OMX_ERRORTYPE
omx_videodec_component_GetExtensionIndex(
  OMX_IN  OMX_HANDLETYPE  hComponent,
  OMX_IN  OMX_STRING      cParameterName,
  OMX_OUT OMX_INDEXTYPE  *pIndexType
  )
{
	OMX_ERRORTYPE err;
	OMX_INDEXTYPE indextype = OMX_IndexTcExtVideoStartUnused;
	LOG_STEP("In %s ", __func__);

	SET_IF_MATCHED(cParameterName, "OMX.TCC.index.param.msmpeg4.version", OMX_IndexParamVideoMsMpeg4Version, indextype);
	SET_IF_MATCHED(cParameterName, "OMX.TCC.index.param.divx.version", OMX_IndexParamVideoDivxVersion, indextype);
	SET_IF_MATCHED(cParameterName, "OMX.TCC.index.param.video.stride", OMX_IndexParamVideoStride, indextype);
	SET_IF_MATCHED(cParameterName, "OMX.TCC.index.param.config.merge.avc.slice", OMX_IndexConfigMergeAvcSlice, indextype);
	SET_IF_MATCHED(cParameterName, "OMX.TCC.index.param.config.video.thumbnail.mode", OMX_IndexConfigVideoThumbnailMode, indextype);
	SET_IF_MATCHED(cParameterName, "OMX.TCC.index.param.config.current.frame.output", OMX_IndexConfigCurrentFrameOutput, indextype);
	SET_IF_MATCHED(cParameterName, "OMX.TCC.index.param.config.current.keyframe.output", OMX_IndexConfigCurrentKeyframeOutput, indextype);
	SET_IF_MATCHED(cParameterName, "OMX.TCC.index.config.video.playback.direction", OMX_IndexConfigVideoPlayDirection, indextype);
	SET_IF_MATCHED(cParameterName, "OMX.TCC.index.config.video.output.keyframe.only", OMX_IndexConfigVideoOutputKeyFrameOnly, indextype);
	SET_IF_MATCHED(cParameterName, "OMX.TCC.index.config.video.sequence.init.fail.max", OMX_IndexConfigVideoSequenceInitFailMax, indextype);
	SET_IF_MATCHED(cParameterName, "OMX.TCC.index.config.decoding.error.reporting", OMX_IndexConfigDecodingErrorReporting, indextype);
	SET_IF_MATCHED(cParameterName, "OMX.TCC.index.InputBuffer.SecureMode", OMX_IndexDecoderSecuredInBuffers, indextype);

	if (indextype != OMX_IndexTcExtVideoStartUnused){
		*pIndexType = indextype;
		err = OMX_ErrorNone;
	}
	else {
		err = OMX_ErrorBadParameter;
	}

	return err;
}

OMX_ERRORTYPE
OMX_ComponentInit(
	OMX_HANDLETYPE openmaxStandComp,
	OMX_STRING cCompontName
	)
{
	OMX_ERRORTYPE err = OMX_ErrorNone;

	CHECK_DECLOG_ENABLE();
	CHECK_VDECLOG_LEVEL();
	LOG_STEP("In %s ", __func__);

//  DEBUG(DEB_LEV_NO_OUTPUT, "In %s DEB_LEV_NO_OUTPUT \n", __func__);
//	DEBUG(DEB_LEV_ERR, "In %s DEB_LEV_ERR \n", __func__);
//  DEBUG(DEB_LEV_PARAMS, "In %s DEB_LEV_PARAMS \n", __func__);
//  DEBUG(DEB_LEV_SIMPLE_SEQ, "In %s DEB_LEV_SIMPLE_SEQ \n", __func__);
//  DEBUG(DEB_LEV_FULL_SEQ, "In %s DEB_LEV_FULL_SEQ \n", __func__);
//  DEBUG(DEB_LEV_FUNCTION_NAME, "In %s DEB_LEV_FUNCTION_NAME \n", __func__);
//  DEBUG(DEFAULT_MESSAGES, "In %s DEFAULT_MESSAGES \n", __func__);
//  DEBUG(DEB_ALL_MESS, "In %s DEB_ALL_MESS \n", __func__);

	err = omx_videodec_component_Constructor(openmaxStandComp,cCompontName);
	return err;
}


