// SPDX-License-Identifier: LGPL-2.1-or later
/****************************************************************************
 *   FileName    : vdec_k_vpu_4kd2.c
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
#include "vdec.h"
#ifdef TCC_VPU_4K_D2_INCLUDE

#define LOG_TAG "VPU_DEC_K_VPU_4K_D2"

#include "TCCMemory.h"
#include <sys/mman.h>
#include <errno.h>

#include <sys/ioctl.h>
#include <tcc_vpu_ioctl.h>
#include <tcc_4k_d2_ioctl.h>

#include <dlfcn.h>

#include <memory.h>
#include <stdio.h>
#include <string.h>

/********************************************************************************************/
/* 4KD2 Setting */
/********************************************************************************************/
#define DISABLE_4KD2_10BIT_OUT 1
/********************************************************************************************/

#if defined(TC_SECURE_MEMORY_COPY)
extern int
TC_SecureMemoryCopy(
  unsigned int paTarget,
  unsigned int paSource,
  unsigned int nSize
);
#endif

/* VPU F/W OPTEE load/unload option */
#define OPTEE_VPU_FW_ACCESS_MODE 0

#define OPT_COMMEND_QUEUE_DEPTH_2                   (0x40000)


#include <fcntl.h>         // O_RDWR
#include <sys/poll.h>

#define DEBUG_VPU_USER //for debug vpu drv in userspace side (e.g. omx)
#ifdef DEBUG_VPU_USER
typedef struct debug_usr_vpu_isr_t {
  int ret_code_vmgr_hdr;
  unsigned int vpu_k_isr_cnt_hit;
  unsigned int wakeup_interrupt_cnt;
} debug_usr_vpu_isr_t;
#endif
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
static void update_hdr_color_config(_vdec_ *pInst, void *hdr_input)
{
  if (pInst != NULL && hdr_input != NULL) {
    switch (pInst->codec_format) {
      case STD_HEVC:
      {
        hevc_userdata_output_t *hdr = (hevc_userdata_output_t *)hdr_input;
        pInst->stHdrColor.mID = 0; // Static Metadata Type 1
        pInst->stHdrColor.mR[0] = (uint16_t)hdr->display_primaries_x[0];
        pInst->stHdrColor.mR[1] = (uint16_t)hdr->display_primaries_y[0];
        pInst->stHdrColor.mG[0] = (uint16_t)hdr->display_primaries_x[1];
        pInst->stHdrColor.mG[1] = (uint16_t)hdr->display_primaries_y[1];
        pInst->stHdrColor.mB[0] = (uint16_t)hdr->display_primaries_x[2];
        pInst->stHdrColor.mB[1] = (uint16_t)hdr->display_primaries_y[2];
        pInst->stHdrColor.mW[0] = (uint16_t)hdr->white_point_x;
        pInst->stHdrColor.mW[1] = (uint16_t)hdr->white_point_y;

        pInst->stHdrColor.mMaxDisplayLuminance = (uint16_t)hdr->max_display_mastering_luminance;
        pInst->stHdrColor.mMinDisplayLuminance = (uint16_t)hdr->min_display_mastering_luminance;
        pInst->stHdrColor.mMaxContentLightLevel = (uint16_t)hdr->max_content_light_level;
        pInst->stHdrColor.mMaxFrameAverageLightLevel = (uint16_t)hdr->max_pic_average_light_level;
     } break;

     default:
        LOGE("%s: unsupported !!", __func__);
     return;
   }
 }
}

static void vpu_4k_d2_copy_dec_out_info(vpu_4K_D2_dec_output_t *src, vdec_output_t *dst, uint32_t bFlushOut, _vdec_*pInst)
{
  int m;
  int n;
  for (m = 0; m < 2; m++) {
    for (n = 0; n < 3; n++) {
      dst->m_pDispOut[m][n] = (uint8_t *)src->m_pDispOut[m][n];
      dst->m_pCurrOut[m][n] = (uint8_t *)src->m_pCurrOut[m][n];
      dst->m_pPrevOut[m][n] = (uint8_t *)src->m_pPrevOut[m][n];
    }
  }
  dst->m_DecOutInfo.m_iPicType        = src->m_DecOutInfo.m_iPicType;
  dst->m_DecOutInfo.m_iDispOutIdx     = src->m_DecOutInfo.m_iDispOutIdx;
  dst->m_DecOutInfo.m_iDecodedIdx     = src->m_DecOutInfo.m_iDecodedIdx;
  dst->m_DecOutInfo.m_iOutputStatus   = src->m_DecOutInfo.m_iOutputStatus;
  dst->m_DecOutInfo.m_iDecodingStatus = src->m_DecOutInfo.m_iDecodingStatus;
  dst->m_DecOutInfo.m_iNumOfErrMBs    = src->m_DecOutInfo.m_iNumOfErrMBs;

  // [CQ1] dec. size info: on decoding, disp. size info: on flushing
  // [CQ2] disp. size info: on decoding/flushing (USE_PREV_STREAMBUFF_DECODING)
  uint32_t bUseDecOutDisplayInfo = bFlushOut;
#ifdef USE_PREV_STREAMBUFF_DECODING
  dst->m_NumDisplayPending = 0;
  if (!bFlushOut) {
    bUseDecOutDisplayInfo = (src->m_DecOutInfo.m_iDisplayWidth  != 0) ||
                            (src->m_DecOutInfo.m_iDisplayHeight != 0);
  }
#endif

  if (bUseDecOutDisplayInfo) {
    dst->m_DecOutInfo.m_iWidth  = src->m_DecOutInfo.m_iDisplayWidth;
    dst->m_DecOutInfo.m_iHeight = src->m_DecOutInfo.m_iDisplayHeight;
    dst->m_DecOutInfo.m_CropInfo.m_iCropTop    = src->m_DecOutInfo.m_DisplayCropInfo.m_iCropTop;
    dst->m_DecOutInfo.m_CropInfo.m_iCropBottom = src->m_DecOutInfo.m_DisplayCropInfo.m_iCropBottom;
    dst->m_DecOutInfo.m_CropInfo.m_iCropRight  = src->m_DecOutInfo.m_DisplayCropInfo.m_iCropRight;
    dst->m_DecOutInfo.m_CropInfo.m_iCropLeft   = src->m_DecOutInfo.m_DisplayCropInfo.m_iCropLeft;
  } else {
    dst->m_DecOutInfo.m_iWidth  = src->m_DecOutInfo.m_iDecodedWidth;
    dst->m_DecOutInfo.m_iHeight = src->m_DecOutInfo.m_iDecodedHeight;
    dst->m_DecOutInfo.m_CropInfo.m_iCropTop    = src->m_DecOutInfo.m_DecodedCropInfo.m_iCropTop;
    dst->m_DecOutInfo.m_CropInfo.m_iCropBottom = src->m_DecOutInfo.m_DecodedCropInfo.m_iCropBottom;
    dst->m_DecOutInfo.m_CropInfo.m_iCropRight  = src->m_DecOutInfo.m_DecodedCropInfo.m_iCropRight;
    dst->m_DecOutInfo.m_CropInfo.m_iCropLeft   = src->m_DecOutInfo.m_DecodedCropInfo.m_iCropLeft;
  }

  dst->m_DecOutInfo.m_Reserved[6]  = src->m_DecOutInfo.m_Reserved[6];
  dst->m_DecOutInfo.m_Reserved[7]  = src->m_DecOutInfo.m_Reserved[7];
  dst->m_DecOutInfo.m_Reserved[8]  = src->m_DecOutInfo.m_Reserved[8];
  dst->m_DecOutInfo.m_Reserved[9]  = src->m_DecOutInfo.m_Reserved[9];
  dst->m_DecOutInfo.m_Reserved[10] = src->m_DecOutInfo.m_Reserved[10];

  dst->m_DecOutInfo.m_UserDataAddress[0] = src->m_DecOutInfo.m_UserDataAddress[0];
  dst->m_DecOutInfo.m_UserDataAddress[1] = src->m_DecOutInfo.m_UserDataAddress[1];
  dst->m_DecOutInfo.m_iConsumedBytes     = src->m_DecOutInfo.m_iConsumedBytes;
  dst->m_DecOutInfo.m_iInvalidDispCount  = src->m_DecOutInfo.m_iInvalidDispCount; // of no use

  dst->m_iIsSuperFrame = src->m_DecOutInfo.m_iIsSuperFrame;

  if (src->m_DecOutInfo.m_iIsSuperFrame){
    DSTATUS("[super-frame] sub-frames: %u, index: curr. %u, done. %u",
            src->m_DecOutInfo.m_SuperFrameInfo.m_uiNframes,
            src->m_DecOutInfo.m_SuperFrameInfo.m_uiCurrentIdx,
            src->m_DecOutInfo.m_SuperFrameInfo.m_uiDoneIdx);
  }
#if 0 // [CHECK] AFBC compressed output mode
  dst->m_AfbcInfo = &src->m_DecOutInfo.m_DispAfbcDecInfo;
  dst->m_Vp9SuperFrameInfo = &src->m_DecOutInfo.m_SuperFrameInfo;
#endif

#ifdef USE_MAP_CONVERTER
  (void)memcpy(&dst->m_MapConvInfo, &src->m_DecOutInfo.m_DispMapConvInfo, sizeof(hevc_MapConv_info_t));

  if (pInst->extFunction & EXT_FUNC_USE_MAP_CONVERTER) {
    DSTATUS("[4KD2-%d|MC|PA] Y:%#zx, Cb:%#zx, FBC-Y:%#zx, FBC-C:%#zx", pInst->vdec_instance_index,
            (size_t)dst->m_MapConvInfo.m_CompressedY[0],
            (size_t)dst->m_MapConvInfo.m_CompressedCb[0],
            (size_t)dst->m_MapConvInfo.m_FbcYOffsetAddr[0],
            (size_t)dst->m_MapConvInfo.m_FbcCOffsetAddr[0]);

    DSTATUS("[4KD2-%d|MC|VA] Y:%#zx, Cb:%#zx, FBC-Y:%#zx, FBC-C:%#zx", pInst->vdec_instance_index,
            (size_t)dst->m_MapConvInfo.m_CompressedY[1],
            (size_t)dst->m_MapConvInfo.m_CompressedCb[1],
            (size_t)dst->m_MapConvInfo.m_FbcYOffsetAddr[1],
            (size_t)dst->m_MapConvInfo.m_FbcCOffsetAddr[1]);

    DSTATUS("[4KD2-%d|MC] Stride (L:%d, C:%d), BitDepth (L:%#x, C:%#x), Endian(%u)", pInst->vdec_instance_index,
            dst->m_MapConvInfo.m_uiLumaStride, dst->m_MapConvInfo.m_uiChromaStride,
            dst->m_MapConvInfo.m_uiLumaBitDepth, dst->m_MapConvInfo.m_uiChromaBitDepth,
            dst->m_MapConvInfo.m_uiFrameEndian);
  }
#endif
}

static int32_t vpu_4k_d2_cmd_process(int32_t cmd, void *args, _vdec_ *pVdec)
{
  int32_t ret = 0;
  uint32_t polling_success = 0;
  _vdec_ * pInst = pVdec;
  static const uint32_t VPU_4K_D2_POLL_RETRY_COUNT = 2;

  uint32_t retry_cnt = VPU_4K_D2_POLL_RETRY_COUNT;
  uint32_t all_retry_cnt = CMD_RETRY_COUNT;
#ifdef USE_VPU_HANGUP_RELEASE
  uint32_t try_hangup_rel = 0;
#endif

  if (ioctl(pInst->dec_fd, cmd, args) < 0) {
    if (ret == -0x999) {
      LOGE("[4KD2-%d][ret:0x999] invalid command: %d", pInst->vdec_instance_index, cmd);
      return RETCODE_INVALID_COMMAND;
    }
    LOGE("[4KD2-%d] %s: ioctl err (%s, cmd: %d)", pInst->vdec_instance_index, __func__, strerror(errno), cmd);
  }

  uint32_t poll_timeout_exit = 0;
Retry:
  while (retry_cnt > 0) {
    memset(pInst->tcc_event, 0, sizeof(pInst->tcc_event));
    pInst->tcc_event[0].fd = pInst->dec_fd;
    pInst->tcc_event[0].events = POLLIN;

    int timeout = ((cmd == V_DEC_BUF_FLAG_CLEAR) || (cmd == V_DEC_SEQ_HEADER)) ? 600 : 1000;
    ret = poll((struct pollfd *)&pInst->tcc_event, 1, timeout);
    if (ret < 0) {
      LOGE("[4KD2-%d][cmd:%#x][retry:%d] poll error '%s'", pInst->vdec_instance_index,  cmd, retry_cnt, strerror(errno));
      retry_cnt--;
      continue;
    }

    if (ret == 0) {
      LOGE("[4KD2-%d][cmd:%d][retry:%d] poll timeout at %d'th frames, len %d",
              pInst->vdec_instance_index, cmd, retry_cnt, pInst->total_frm,
              pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_iBitstreamDataSize);

      if ((cmd == V_DEC_CLOSE) || (cmd == V_DEC_BUF_FLAG_CLEAR) || (cmd == V_DEC_SEQ_HEADER)) {
        poll_timeout_exit = 1;
        break;
      }

      if (pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_iBitstreamDataSize <= 1){
        break;
      }

      retry_cnt--;
      continue;
    }

    if (ret > 0) {
      if (pInst->tcc_event[0].revents & POLLERR) {
        LOGE("[4KD2-%d] poll POLLERR", pInst->vdec_instance_index);
        break;
      } else if (pInst->tcc_event[0].revents & POLLIN) {
        polling_success = 1;
        break;
      }
    }
  }

#ifdef USE_VPU_HANGUP_RELEASE
  if (!polling_success && !try_hangup_rel) {
    LOGI("4KD2[%d] Try to release VPU-Hangup!!", pInst->vdec_instance_index);
    if (ioctl(pInst->mgr_fd, VPU_TRY_HANGUP_RELEASE, NULL) < 0) {
      LOGE("4KD2[%d] ioctl(%#x) error[%s]!!", pInst->vdec_instance_index, VPU_TRY_HANGUP_RELEASE, strerror(errno));
    }
    try_hangup_rel = 1;
    retry_cnt = POLL_RETRY_COUNT;
    goto Retry;
  }
#endif

    switch (cmd) {
    case V_DEC_INIT:
    {
      VPU_4K_D2_INIT_t *p_init_info = (VPU_4K_D2_INIT_t *)args;

      if (ioctl(pInst->dec_fd, V_DEC_INIT_RESULT, args) < 0) {
        LOGE("[4KD2-%d] ioctl(%#x) error[%s]!!", pInst->vdec_instance_index, V_DEC_INIT_RESULT, strerror(errno));
      }
      ret = p_init_info->result;
    }
    break;

    case V_DEC_SEQ_HEADER:
    {
      void *p_seq_info = args;

      if (ioctl(pInst->dec_fd, V_DEC_SEQ_HEADER_RESULT, args) < 0) {
        LOGE("[4KD2-%d] ioctl(%#x) error[%s]!!", pInst->vdec_instance_index, V_DEC_SEQ_HEADER_RESULT, strerror(errno));
      }
      ret = ((VPU_4K_D2_SEQ_HEADER_t *)p_seq_info)->result;
    }
    break;

    case V_DEC_DECODE:
    {
      VPU_4K_D2_DECODE_t *p_dec_info = (VPU_4K_D2_DECODE_t *)args;

      if (ioctl(pInst->dec_fd, V_DEC_DECODE_RESULT, args) < 0) {
        LOGE("[4KD2-%d] ioctl(%#x) error[%s]!!", pInst->vdec_instance_index, V_DEC_DECODE_RESULT, strerror(errno));
      }
      ret = p_dec_info->result;
    }
    break;

    case V_DEC_GET_OUTPUT_INFO:
    {
      VPU_4K_D2_DECODE_t *p_dec_info = (VPU_4K_D2_DECODE_t *)args;

      if (ioctl(pInst->dec_fd, V_DEC_GET_OUTPUT_INFO_RESULT, args) < 0) {
        LOGE("[4KD2-%d] ioctl(%#x) error[%s]!!", pInst->vdec_instance_index, V_DEC_GET_OUTPUT_INFO_RESULT, strerror(errno));
      }
      ret = p_dec_info->result;
    }
    break;

    case V_DEC_FLUSH_OUTPUT:
    {
      VPU_4K_D2_DECODE_t *p_dec_info = (VPU_4K_D2_DECODE_t *)args;

      if (ioctl(pInst->dec_fd, V_DEC_FLUSH_OUTPUT_RESULT, args) < 0) {
        LOGE("[4KD2-%d] ioctl(%#x) error[%s]!!", pInst->vdec_instance_index, V_DEC_FLUSH_OUTPUT_RESULT, strerror(errno));
      }
      ret = p_dec_info->result;
    }
    break;

    case V_GET_RING_BUFFER_STATUS:
    {
      VPU_4K_D2_RINGBUF_GETINFO_t *p_param = (VPU_4K_D2_RINGBUF_GETINFO_t *)args;
      if (ioctl(pInst->dec_fd, V_GET_RING_BUFFER_STATUS_RESULT, args) < 0) {
        LOGE("4KD2[%d] ioctl(%#x) error[%s]!!", pInst->vdec_instance_index, V_GET_RING_BUFFER_STATUS_RESULT, strerror(errno));
      }
      ret = p_param->result;
    }
    break;

    case V_FILL_RING_BUFFER_AUTO:
    {
      VPU_4K_D2_RINGBUF_SETBUF_t *p_param = (VPU_4K_D2_RINGBUF_SETBUF_t *)args;

      if (ioctl(pInst->dec_fd, V_FILL_RING_BUFFER_AUTO_RESULT, args) < 0) {
        LOGE("[4KD2-%d] ioctl(%#x) error[%s]!!", pInst->vdec_instance_index, V_FILL_RING_BUFFER_AUTO_RESULT, strerror(errno));
      }
      ret = p_param->result;
    }
    break;

    case V_DEC_UPDATE_RINGBUF_WP:
    {
      VPU_4K_D2_RINGBUF_SETBUF_PTRONLY_t *p_param = (VPU_4K_D2_RINGBUF_SETBUF_PTRONLY_t *)args;
      if (ioctl(pInst->dec_fd, V_DEC_UPDATE_RINGBUF_WP_RESULT, args) < 0) {
        LOGE("[4KD2-%d] ioctl(%#x) error[%s]!!", pInst->vdec_instance_index, V_DEC_UPDATE_RINGBUF_WP_RESULT, strerror(errno));
      }
      ret = p_param->result;
    }
    break;

    case V_GET_INITIAL_INFO_FOR_STREAMING_MODE_ONLY:
    {
      void *p_seq_info = args;

      if (ioctl(pInst->dec_fd, V_GET_INITIAL_INFO_FOR_STREAMING_MODE_ONLY_RESULT, args) < 0) {
        LOGE("4KD2[%d] ioctl(%#x) error[%s]!!", pInst->vdec_instance_index, V_GET_INITIAL_INFO_FOR_STREAMING_MODE_ONLY_RESULT, strerror(errno));
      }
      ret = ((VPU_4K_D2_SEQ_HEADER_t *)p_seq_info)->result;
    }
    break;

    case V_GET_VPU_VERSION:
    {
      VPU_4K_D2_GET_VERSION_t *p_param = (VPU_4K_D2_GET_VERSION_t *)args;
      if (ioctl(pInst->dec_fd, V_GET_VPU_VERSION_RESULT, args) < 0) {
        LOGE("4KD2[%d] ioctl(%#x) error[%s]!!", pInst->vdec_instance_index, V_GET_VPU_VERSION_RESULT, strerror(errno));
      }
      ret = p_param->result;
    }
    break;

    case V_DEC_REG_FRAME_BUFFER:
    case V_DEC_BUF_FLAG_CLEAR:
    case V_DEC_CLOSE:
    case V_DEC_GET_INFO:
    case V_DEC_REG_FRAME_BUFFER2:
    default:
      if (ioctl(pInst->dec_fd, V_DEC_GENERAL_RESULT, &ret) < 0) {
        LOGE("4KD2[%d] ioctl(%#x) error[%s]!!", pInst->vdec_instance_index, V_DEC_GENERAL_RESULT, strerror(errno));
      }
      break;
    }

    // we skip the invalid return because vpu might not process current command yet.
    if ((poll_timeout_exit == 0) && ((ret & 0xf000) != 0x0000)) {
      all_retry_cnt--;
      if (all_retry_cnt > 0) {
        retry_cnt = VPU_4K_D2_POLL_RETRY_COUNT;
        goto Retry;
      }
      LOGE("[4KD2-%d][cmd:%d] abnormal exception !!", pInst->vdec_instance_index, cmd);
    }

    if ((!polling_success) || poll_timeout_exit || ((ret & 0xf000) != 0x0000 /* unknown error in vpu */)) {
      debug_usr_vpu_isr_t vpu_isr_param_usr_debug;
      if (ioctl(pInst->dec_fd, VPU_DEBUG_ISR, &vpu_isr_param_usr_debug) >= 0) {
        if (vpu_isr_param_usr_debug.vpu_k_isr_cnt_hit > vpu_isr_param_usr_debug.wakeup_interrupt_cnt + 10)
          LOGE("[4KD2-%d] entered VPU ISR %d times, woke up from interruptable sleep: "
              "(wake-up cnt:%d, ret of vmrg handler:%d)!!", pInst->vdec_instance_index,
              vpu_isr_param_usr_debug.vpu_k_isr_cnt_hit,
              vpu_isr_param_usr_debug.wakeup_interrupt_cnt,
              vpu_isr_param_usr_debug.ret_code_vmgr_hdr
        );
      }

      LOGE("[4KD2-%d][cmd:%#x] VPU hang-up !! (no-return: %#x)", pInst->vdec_instance_index, cmd, ret);
      ret = RETCODE_CODEC_EXIT;
    }

    return ret;
}

static void print_vpu_4k_d2_initial_info(vpu_4K_D2_dec_initial_info_t *pInitialInfo, _vdec_ *pInst)
{
  uint32_t fRateInfoRes = pInitialInfo->m_uiFrameRateRes;
  uint32_t fRateInfoDiv = pInitialInfo->m_uiFrameRateDiv;

  int32_t profile = pInitialInfo->m_iProfile;
  int32_t level = pInitialInfo->m_iLevel;;
  int32_t tier = pInitialInfo->m_iTier;

  LOGD("[4KD2-%d] ---------------- 4K-D2 INITIAL INFO ---------------", pInst->vdec_instance_index);
  LOGD("[4KD2-%d] Dec InitialInfo => profile: %d level: %d tier: %d", pInst->vdec_instance_index, profile, level, tier);
  LOGD("[4KD2-%d] Aspect Ratio [%1d]", pInst->vdec_instance_index, pInitialInfo->m_iAspectRateInfo);
  LOGD("[4KD2-%d] FrameBuffer Format [%s Bit]", pInst->vdec_instance_index, pInitialInfo->m_iFrameBufferFormat == 0 ? "8" : "10");
  LOGD("[4KD2-%d] MinFrameBufferCount: %u m_iPicWidth: %u m_iPicHeight: %u",
          pInst->vdec_instance_index, pInitialInfo->m_iMinFrameBufferCount, pInitialInfo->m_iPicWidth, pInitialInfo->m_iPicHeight);
  LOGD("[4KD2-%d] frameBufDelay: %u", pInst->vdec_instance_index, pInitialInfo->m_iFrameBufDelay);
  LOGD("[4KD2-%d] Crop Info %d - %d - %d, %d - %d - %d", pInst->vdec_instance_index,
          pInitialInfo->m_iPicWidth, pInitialInfo->m_PicCrop.m_iCropLeft, pInitialInfo->m_PicCrop.m_iCropRight,
          pInitialInfo->m_iPicHeight, pInitialInfo->m_PicCrop.m_iCropTop, pInitialInfo->m_PicCrop.m_iCropBottom);
  LOGD("[4KD2-%d]  frRes: %u frDiv: %u", pInst->vdec_instance_index, fRateInfoRes, fRateInfoDiv);
  LOGD("[4KD2-%d] ---------------------------------------------------", pInst->vdec_instance_index);
}

int32_t vpu_4k_d2_dec_ready(vpu_4K_D2_dec_init_t *psHDecInit, _vdec_ *pInst)
{
  //------------------------------------------------------------
  //! [x] bitstream buffer for each VPU_4K_D2 decoder
  //------------------------------------------------------------
#ifdef USE_PREV_STREAMBUFF_DECODING
  pInst->gsCurrStreamBuffer_index = 0;
  pInst->bBitstreamSplitIndexPinned = 0;
#endif
  pInst->gsBitstreamBufSize = (psHDecInit->m_iBitstreamBufSize > WAVE5_STREAM_BUF_SIZE)
                            ? ALIGNED_BUFF(psHDecInit->m_iBitstreamBufSize, 64 * 1024) // hees: ???
                            : WAVE5_STREAM_BUF_SIZE;

  pInst->gsBitstreamBufSize = ALIGNED_BUFF(pInst->gsBitstreamBufSize, ALIGN_LEN);
  pInst->gsBitstreamBufAddr[PA] = cdk_sys_malloc_physical_addr(
          &pInst->gsBitstreamBufAddr[K_VA], pInst->gsBitstreamBufSize, BUFFER_STREAM, pInst);

  if (pInst->gsBitstreamBufAddr[PA] == 0) {
    LOGE("[4KD2-%d] bitstream_buf_addr[PA] malloc() failed", pInst->vdec_instance_index);
    return -(VPU_NOT_ENOUGH_MEM);
  }

  DSTATUS("[4KD2-%d] bitstream_buf_addr[PA] = %#zx, %#x", pInst->vdec_instance_index,
          (size_t)pInst->gsBitstreamBufAddr[PA], pInst->gsBitstreamBufSize);

  pInst->gsBitstreamBufAddr[VA] = cdk_sys_malloc_virtual_addr(
          pInst->gsBitstreamBufAddr[PA], pInst->gsBitstreamBufSize, pInst);

  if (pInst->gsBitstreamBufAddr[VA] == 0) {
    LOGE("[4KD2-%d] bitstream_buf_addr[VA] malloc() failed", pInst->vdec_instance_index);
    return -(VPU_NOT_ENOUGH_MEM);
  }

  //memset((void*)pInst->gsBitstreamBufAddr[VA], 0x00 , pInst->gsBitstreamBufSize);

  DSTATUS("[4KD2-%d] bitstream_buf_addr[VA] = %#zx, %#x", pInst->vdec_instance_index,
          (size_t)pInst->gsBitstreamBufAddr[VA], pInst->gsBitstreamBufSize);

  psHDecInit->m_BitstreamBufAddr[PA]  = pInst->gsBitstreamBufAddr[PA];
  psHDecInit->m_BitstreamBufAddr[VA]  = pInst->gsBitstreamBufAddr[K_VA];
  psHDecInit->m_iBitstreamBufSize     = pInst->gsBitstreamBufSize;

  if (pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_iFilePlayEnable != 0) {
    pInst->gsIntermediateBufSize = 0;
    pInst->gsIntermediateBufAddr[PA] = 0;
    pInst->gsIntermediateBufAddr[VA] = 0;
    pInst->gsIntermediateBufAddr[K_VA] = 0;
  }

  /* Set the maximum size of input bitstream. */
  pInst->gsMaxBitstreamSize = pInst->gsBitstreamBufSize;

  {
    //------------------------------------------------------------
    //! [x] user data buffer for each VPU_4K_D2 decoder
    //------------------------------------------------------------
    if (pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_bEnableUserData) {
      pInst->gsUserdataBufSize = 512 * 1024; // hees: ???
      pInst->gsUserdataBufSize = ALIGNED_BUFF(pInst->gsUserdataBufSize, ALIGN_LEN);
      pInst->gsUserdataBufAddr[PA] = cdk_sys_malloc_physical_addr(
              &pInst->gsUserdataBufAddr[K_VA], pInst->gsUserdataBufSize, BUFFER_USERDATA, pInst);
      if (pInst->gsUserdataBufAddr[PA] == 0) {
          LOGE("[4KD2-%d:Err] pInst->gsUserdataBufAddr physical alloc failed", pInst->vdec_instance_index);
          return -(VPU_NOT_ENOUGH_MEM);
      }

      DSTATUS("[4KD2-%d] pInst->gsUserdataBufAddr[PA] = %#zx, %#x", pInst->vdec_instance_index,
              (size_t)pInst->gsUserdataBufAddr[PA], pInst->gsUserdataBufSize);

      pInst->gsUserdataBufAddr[VA] = cdk_sys_malloc_virtual_addr(
              pInst->gsUserdataBufAddr[PA], pInst->gsUserdataBufSize, pInst);
      if (pInst->gsUserdataBufAddr[VA] == 0) {
          DPRINTF( "[4KD2-%d:Err] pInst->gsUserdataBufAddr virtual alloc failed", pInst->vdec_instance_index);
          return -(VPU_NOT_ENOUGH_MEM);
      }

      DSTATUS("[4KD2-%d] pInst->gsUserdataBufAddr[VA] = %#zx, %#x", pInst->vdec_instance_index,
              (size_t)pInst->gsUserdataBufAddr[VA], pInst->gsUserdataBufSize);

      psHDecInit->m_UserDataAddr[PA] = pInst->gsUserdataBufAddr[PA];
      psHDecInit->m_UserDataAddr[VA] = pInst->gsUserdataBufAddr[K_VA];
      psHDecInit->m_iUserDataBufferSize = pInst->gsUserdataBufSize;
    }

    //----------------------------------------------------------------
    // [x] code buffer, work buffer and parameter buffer for VPU_4K_D2
    //----------------------------------------------------------------
    pInst->gsBitWorkBufSize = WAVE5_WORK_CODE_BUF_SIZE;
    pInst->gsBitWorkBufSize = ALIGNED_BUFF(pInst->gsBitWorkBufSize, ALIGN_LEN);
    pInst->gsBitWorkBufAddr[PA] = cdk_sys_malloc_physical_addr(
            &pInst->gsBitWorkBufAddr[K_VA], pInst->gsBitWorkBufSize, BUFFER_WORK, pInst);
    if (pInst->gsBitWorkBufAddr[PA] == 0) {
      LOGE("[4KD2-%d] bit_work_buf_addr[PA] malloc() failed", pInst->vdec_instance_index);
      return -(VPU_NOT_ENOUGH_MEM);
    }

    DSTATUS("[4KD2-%d] bit_work_buf_addr[PA] = %#zx, %#x", pInst->vdec_instance_index,
            (size_t)pInst->gsBitWorkBufAddr[PA], pInst->gsBitWorkBufSize);

    pInst->gsBitWorkBufAddr[VA] = cdk_sys_malloc_virtual_addr(pInst->gsBitWorkBufAddr[PA], pInst->gsBitWorkBufSize, pInst);
    if (pInst->gsBitWorkBufAddr[VA] == 0) {
      DPRINTF("[4KD2-%d] bit_work_buf_addr[VA] malloc() failed ", pInst->vdec_instance_index);
      return -(VPU_NOT_ENOUGH_MEM);
    }

    DSTATUS("[4KD2-%d] bit_work_buf_addr[VA] = %#zx, %#x", pInst->vdec_instance_index,
            (size_t)pInst->gsBitWorkBufAddr[VA], pInst->gsBitWorkBufSize);
  }

  psHDecInit->m_BitWorkAddr[PA] = pInst->gsBitWorkBufAddr[PA];
  psHDecInit->m_BitWorkAddr[VA] = pInst->gsBitWorkBufAddr[K_VA];

  pInst->m_bCbCrInterleaveMode = psHDecInit->m_bCbCrInterleaveMode;
  DSTATUS("[4KD2-%d] CbCrInterleaveMode %s", pInst->vdec_instance_index, (psHDecInit->m_bCbCrInterleaveMode == 0) ? "OFF" : "ON");

  return 0;
}

int32_t vpu_4k_d2_dec_seq_header(int32_t iSize, int32_t iIsThumbnail, _vdec_ *pInst)
{
  int32_t ret = 0;

  vpu_4K_D2_dec_initial_info_t *gsV4kd2DecInitialInfo = &pInst->gsVpu4KD2SeqHeaderInfo.gsV4kd2DecInitialInfo;

  LOGD("[4KD2-%d] VPU_DEC_SEQ_HEADER: size(%d), format(%d)",
          pInst->vdec_instance_index, iSize, pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_iBitstreamFormat);

  (void)memset(&pInst->gsVpu4KD2SeqHeaderInfo, 0, sizeof(VPU_4K_D2_SEQ_HEADER_t));
  pInst->gsVpu4KD2SeqHeaderInfo.stream_size = iSize;

  ret = vpu_4k_d2_cmd_process(V_DEC_SEQ_HEADER, (void *)&pInst->gsVpu4KD2SeqHeaderInfo, pInst);

  if (ret != RETCODE_SUCCESS) {
    DPRINTF("[4KD2-%d|VPU_DEC_SEQ_HEADER] failed Error code: %#x. ErrorReason: %d",
            pInst->vdec_instance_index, ret, gsV4kd2DecInitialInfo->m_iReportErrorReason);
    if (ret == RETCODE_CODEC_SPECOUT) {
      // This is a very common error. Notice the detailed reason to users.
      DPRINTF("[4KD2-%d] NOT SUPPORTED CODEC. VPU SPEC OUT!!", pInst->vdec_instance_index);
    }
    return -ret;
  }

  if (gsV4kd2DecInitialInfo->m_iAspectRateInfo > 0) {
    pInst->aspect_ratio = gsV4kd2DecInitialInfo->m_iAspectRateInfo;
  }
  
  print_vpu_4k_d2_initial_info(gsV4kd2DecInitialInfo, pInst);
  pInst->mRealPicWidth  = gsV4kd2DecInitialInfo->m_iPicWidth
                        - gsV4kd2DecInitialInfo->m_PicCrop.m_iCropLeft
                        - gsV4kd2DecInitialInfo->m_PicCrop.m_iCropRight;
  pInst->mRealPicHeight = gsV4kd2DecInitialInfo->m_iPicHeight
                        - gsV4kd2DecInitialInfo->m_PicCrop.m_iCropBottom
                        - gsV4kd2DecInitialInfo->m_PicCrop.m_iCropTop;

  LOGD("[4KD2-%d] Real-Resolution: %d x %d (max. %d x %d)", pInst->vdec_instance_index,
          pInst->mRealPicWidth, pInst->mRealPicHeight, pInst->mMaxAdaptiveWidth, pInst->mMaxAdaptiveHeight);

  if (pInst->extFunction & EXT_FUNC_MAX_FRAMEBUFFER) {
    gsV4kd2DecInitialInfo->m_iPicWidth =
        (pInst->mMaxAdaptiveWidth < 0) ? AVAILABLE_4K_MAX_WIDTH : pInst->mMaxAdaptiveWidth;
    gsV4kd2DecInitialInfo->m_iPicHeight =
        (pInst->mMaxAdaptiveHeight < 0) ? AVAILABLE_4K_MAX_HEIGHT : pInst->mMaxAdaptiveHeight;
  }

  LOGD("[4KD2-%d] initial pic resolution: %d x %d", pInst->vdec_instance_index,
          gsV4kd2DecInitialInfo->m_iPicWidth, gsV4kd2DecInitialInfo->m_iPicHeight);

  set_dec_common_info(gsV4kd2DecInitialInfo->m_iPicWidth, gsV4kd2DecInitialInfo->m_iPicHeight,
                      (pic_crop_t *)&gsV4kd2DecInitialInfo->m_PicCrop, 0, 0, pInst);

  pInst->gsSliceBufSize = pInst->gsSliceBufAddr = pInst->gsMbSaveSize = pInst->gsMbSaveAddr = 0;

  //------------------------------------------------------------
  // [x] frame buffer for each VPU_4K_D2 decoder
  //------------------------------------------------------------
  pInst->gsV4kd2DecBufferInfo.gsV4kd2DecBuffer.m_iFrameBufferCount = gsV4kd2DecInitialInfo->m_iMinFrameBufferCount;

  if (!iIsThumbnail) {
    pInst->gsV4kd2DecBufferInfo.gsV4kd2DecBuffer.m_iFrameBufferCount =
          gsV4kd2DecInitialInfo->m_iMinFrameBufferCount + pInst->gsAdditionalFrameCount;
  }

  int32_t max_count = cdk_sys_remain_memory_size(pInst) / gsV4kd2DecInitialInfo->m_iMinFrameBufferSize;

  if (pInst->gsV4kd2DecBufferInfo.gsV4kd2DecBuffer.m_iFrameBufferCount > max_count) {
    pInst->gsAdditionalFrameCount = max_count - gsV4kd2DecInitialInfo->m_iMinFrameBufferCount;
    pInst->gsV4kd2DecBufferInfo.gsV4kd2DecBuffer.m_iFrameBufferCount = max_count;
  }

  if (pInst->gsV4kd2DecBufferInfo.gsV4kd2DecBuffer.m_iFrameBufferCount > MAX_FRAME_BUFFER_COUNT) {
    pInst->gsAdditionalFrameCount = MAX_FRAME_BUFFER_COUNT - gsV4kd2DecInitialInfo->m_iMinFrameBufferCount;
    pInst->gsV4kd2DecBufferInfo.gsV4kd2DecBuffer.m_iFrameBufferCount = MAX_FRAME_BUFFER_COUNT;
  }

  if (iIsThumbnail) {
    if (pInst->gsV4kd2DecBufferInfo.gsV4kd2DecBuffer.m_iFrameBufferCount < gsV4kd2DecInitialInfo->m_iMinFrameBufferCount)
   	{
      LOGE("[4KD2-%d] Not enough memory for VPU frame buffer, Available[%d], Min[%d], Need[%d], min_size[%d]",
              pInst->vdec_instance_index,
              max_count,
              gsV4kd2DecInitialInfo->m_iMinFrameBufferCount,
              pInst->gsV4kd2DecBufferInfo.gsV4kd2DecBuffer.m_iFrameBufferCount,
              gsV4kd2DecInitialInfo->m_iMinFrameBufferSize);

      return -(VPU_NOT_ENOUGH_MEM);
    }
  } else {
    if (pInst->gsV4kd2DecBufferInfo.gsV4kd2DecBuffer.m_iFrameBufferCount <
            gsV4kd2DecInitialInfo->m_iMinFrameBufferCount + pInst->gsAdditionalFrameCount)
    {
      LOGE("[4KD2-%d] Not enough memory for VPU frame buffer, Available[%d], Min[%d], Need[%d], min_size[%d]",
              pInst->vdec_instance_index,
              max_count,
              gsV4kd2DecInitialInfo->m_iMinFrameBufferCount,
              gsV4kd2DecInitialInfo->m_iMinFrameBufferCount + pInst->gsAdditionalFrameCount,
              gsV4kd2DecInitialInfo->m_iMinFrameBufferSize);

      return -(VPU_NOT_ENOUGH_MEM);
    }
  }

  pInst->gsFrameBufCount = pInst->gsV4kd2DecBufferInfo.gsV4kd2DecBuffer.m_iFrameBufferCount;
  pInst->gsTotalFrameCount = pInst->gsV4kd2DecBufferInfo.gsV4kd2DecBuffer.m_iFrameBufferCount;
  pInst->gsFrameBufSize = pInst->gsFrameBufCount * gsV4kd2DecInitialInfo->m_iMinFrameBufferSize;

  LOGI("[4KD2-%d] FrameBufDelay %d, FrameBufferCount %d [min %d, add %d], min_size = %d", pInst->vdec_instance_index,
        gsV4kd2DecInitialInfo->m_iFrameBufDelay,
        pInst->gsV4kd2DecBufferInfo.gsV4kd2DecBuffer.m_iFrameBufferCount,
        gsV4kd2DecInitialInfo->m_iMinFrameBufferCount,
        pInst->gsAdditionalFrameCount,
        gsV4kd2DecInitialInfo->m_iMinFrameBufferSize);

  pInst->gsFrameBufSize = ALIGNED_BUFF(pInst->gsFrameBufSize, ALIGN_LEN);

  pInst->gsFrameBufAddr[PA] = cdk_sys_malloc_physical_addr(
          &pInst->gsFrameBufAddr[K_VA], pInst->gsFrameBufSize, BUFFER_FRAMEBUFFER, pInst);

  if (pInst->gsFrameBufAddr[PA] == 0) {
    DPRINTF("[4KD2-%d] frame_buf_addr[PA] malloc() failed", pInst->vdec_instance_index);
    return -(VPU_NOT_ENOUGH_MEM);
  }

  DSTATUS("[4KD2-%d] MinFrameBufferSize %d bytes",
          pInst->vdec_instance_index, gsV4kd2DecInitialInfo->m_iMinFrameBufferSize);

  DSTATUS("[4KD2-%d] frame_buf_addr[PA] = %#zx, size:%d, index: %d", pInst->vdec_instance_index,
          (size_t)pInst->gsFrameBufAddr[PA], pInst->gsFrameBufSize, pInst->vdec_instance_index);

  pInst->gsFrameBufAddr[VA] = cdk_sys_malloc_virtual_addr(pInst->gsFrameBufAddr[PA], pInst->gsFrameBufSize, pInst);

  if (pInst->gsFrameBufAddr[VA] == 0) {
    DPRINTF("[4KD2-%d] frame_buf_addr[VA] malloc() failed", pInst->vdec_instance_index);
    return -(VPU_NOT_ENOUGH_MEM);
  }

  DSTATUS("[4KD2-%d] frame_buf_addr[VA] = %#zx, frame_buf_addr[K_VA] = %#zx", pInst->vdec_instance_index,
          (size_t)pInst->gsFrameBufAddr[VA], (size_t)pInst->gsFrameBufAddr[K_VA]);

  pInst->gsV4kd2DecBufferInfo.gsV4kd2DecBuffer.m_FrameBufferStartAddr[PA] = pInst->gsFrameBufAddr[PA];
  pInst->gsV4kd2DecBufferInfo.gsV4kd2DecBuffer.m_FrameBufferStartAddr[VA] = pInst->gsFrameBufAddr[K_VA];

  ret = vpu_4k_d2_cmd_process(V_DEC_REG_FRAME_BUFFER, &pInst->gsV4kd2DecBufferInfo, pInst);
  if (ret != RETCODE_SUCCESS) {
    LOGE("[4KD2-%d] DEC_REG_FRAME_BUFFER failed Error code is %#x", pInst->vdec_instance_index, ret);
    return -ret;
  }

  DSTATUS("[4KD2-%d] TCC_VPU_DEC VPU_DEC_REG_FRAME_BUFFER OK!", pInst->vdec_instance_index);
  return ret;
}

static int32_t vpu_4k_d2_set_user_data_info(vpu_4K_D2_dec_UserData_info_t *src, hevc_userdata_output_t *dst)
{
  if (src->m_VuiParam.transfer_characteristics) {
    dst->colour_primaries         = src->m_VuiParam.colour_primaries;
    dst->transfer_characteristics = src->m_VuiParam.transfer_characteristics;
    dst->matrix_coefficients      = src->m_VuiParam.matrix_coefficients;
  }

  if (src->m_MasteringDisplayColorVolume.max_display_mastering_luminance) {
    dst->display_primaries_x[0] = src->m_MasteringDisplayColorVolume.display_primaries_x[0];
    dst->display_primaries_x[1] = src->m_MasteringDisplayColorVolume.display_primaries_x[1];
    dst->display_primaries_x[2] = src->m_MasteringDisplayColorVolume.display_primaries_x[2];
    dst->display_primaries_y[0] = src->m_MasteringDisplayColorVolume.display_primaries_y[0];
    dst->display_primaries_y[1] = src->m_MasteringDisplayColorVolume.display_primaries_y[1];
    dst->display_primaries_y[2] = src->m_MasteringDisplayColorVolume.display_primaries_y[2];
    dst->white_point_x          = src->m_MasteringDisplayColorVolume.white_point_x;
    dst->white_point_y          = src->m_MasteringDisplayColorVolume.white_point_y;
    dst->max_display_mastering_luminance = (src->m_MasteringDisplayColorVolume.max_display_mastering_luminance == 0)
        ? 0 : (src->m_MasteringDisplayColorVolume.max_display_mastering_luminance/ 10000);
    dst->min_display_mastering_luminance = (src->m_MasteringDisplayColorVolume.min_display_mastering_luminance == 0)
        ? 0 : (src->m_MasteringDisplayColorVolume.min_display_mastering_luminance/ 10000);
  }

  if (src->m_ContentLightLevelInfo.max_content_light_level) {
    dst->max_content_light_level        = src->m_ContentLightLevelInfo.max_content_light_level;
    dst->max_pic_average_light_level    = src->m_ContentLightLevelInfo.max_pic_average_light_level;
  }

  dst->version = 1;
  dst->struct_size = 26;//# of input data

  if (dst->eotf == 0) {
    if (dst->transfer_characteristics == 16) {
      dst->eotf = 2; // HDR 10
    } else if ((dst->transfer_characteristics == 18) ||
          ((dst->transfer_characteristics == 14) &&
           (src->m_AlternativeTransferCharacteristicsInfo.preferred_transfer_characteristics == 18)))
    {
      dst->eotf = 3;
    }
  }

  LOGD("==== VPU 4K_D2 userData ====");
  LOGD(" ver %d size %d", dst->version, dst->struct_size);
  LOGD(" colour_primaries %d transfer %d matrix_coefficients %d preferred_transfer_characteristics %d",
          src->m_VuiParam.colour_primaries, src->m_VuiParam.transfer_characteristics, src->m_VuiParam.matrix_coefficients,
          src->m_AlternativeTransferCharacteristicsInfo.preferred_transfer_characteristics);
  LOGD(" primaries_x[0] %d [1] %d [2] %d",
          src->m_MasteringDisplayColorVolume.display_primaries_x[0],
          src->m_MasteringDisplayColorVolume.display_primaries_x[1],
          src->m_MasteringDisplayColorVolume.display_primaries_x[2]);
  LOGD(" primaries_y[0] %d [1] %d [2] %d",
          src->m_MasteringDisplayColorVolume.display_primaries_y[0],
          src->m_MasteringDisplayColorVolume.display_primaries_y[1],
          src->m_MasteringDisplayColorVolume.display_primaries_y[2]);
  LOGD(" white point x %d y %d luminance max %d min %d",
          src->m_MasteringDisplayColorVolume.white_point_x,
          src->m_MasteringDisplayColorVolume.white_point_y,
          src->m_MasteringDisplayColorVolume.max_display_mastering_luminance,
          src->m_MasteringDisplayColorVolume.min_display_mastering_luminance);
  LOGD(" light level max %d average %d",
          src->m_ContentLightLevelInfo.max_content_light_level, src->m_ContentLightLevelInfo.max_pic_average_light_level);

  LOGD(" ver %d size %d", dst->version, dst->struct_size);
  LOGD(" colour_primaries %d transfer %d matrix_coefficients %d",
          dst->colour_primaries, dst->transfer_characteristics, dst->matrix_coefficients);
  LOGD(" primaries_x[0] %d [1] %d [2] %d",
          dst->display_primaries_x[0], dst->display_primaries_x[1], dst->display_primaries_x[2]);
  LOGD(" primaries_y[0] %d [1] %d [2] %d",
          dst->display_primaries_y[0], dst->display_primaries_y[1], dst->display_primaries_y[2]);
  LOGD(" white point x %d y %d luminance max %d min %d",
          dst->white_point_x, dst->white_point_y, dst->max_display_mastering_luminance, dst->min_display_mastering_luminance);
  LOGD(" light level max %d average %d", dst->max_content_light_level, dst->max_pic_average_light_level);

  return dst->eotf == 0 ? -1 : 0;
}

int32_t vdec_vpu_4k_d2( int32_t iOpCode, unsigned long* pHandle, unsigned long* pParam1, unsigned long* pParam2, unsigned long* pParam3 )
{
  int32_t ret = 0;
  _vdec_ *pInst = (_vdec_ *)pParam3;
  (void)pHandle; // remove unused param warning

  if (!pInst) {
    LOGE("[4KD2-unknown] OPCODE %d: nullified instance error !!", iOpCode);
    return -RETCODE_NOT_INITIALIZED;
  }

  if (iOpCode != VDEC_INIT && iOpCode != VDEC_CLOSE && !pInst->vdec_codec_opened) {
    LOGE("[4KD2-%d] OPCODE %d: didn't codec opened !!", pInst->vdec_instance_index, iOpCode);
    return -RETCODE_NOT_INITIALIZED;
  }

#ifdef DEBUG_TIME_LOG
  nsecs_t start, end;
  start = systemTime(CLOCK_MONOTONIC);
#endif

  switch (iOpCode) {
  case VDEC_INIT:
  {
    vdec_init_t *p_init_param = (vdec_init_t *)pParam1;
    vdec_option_t *p_init_user_param = (vdec_option_t*)pParam2;

    pInst->gsUserInfo.frame_rate     = p_init_user_param->frame_rate;
    pInst->gsUserInfo.jpg_ScaleRatio = p_init_user_param->jpg_ScaleRatio;

    pInst->codec_format = p_init_param->m_iBitstreamFormat;
    pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_uiDecOptFlags = 0;
    ret = vpu_env_open(p_init_param->m_iBitstreamFormat, p_init_user_param->bitrate_mbps,
                       p_init_user_param->frame_rate, p_init_param->m_iPicWidth,
                       p_init_param->m_iPicHeight, pInst);
    if (ret < 0)
    {
      return -(VPU_ENV_INIT_ERROR);
    }
    pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_iBitstreamFormat    = p_init_param->m_iBitstreamFormat;
    pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_bEnableUserData     = p_init_param->m_bEnableUserData;
    pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_bCbCrInterleaveMode = p_init_param->m_bCbCrInterleaveMode;

    pInst->extFunction = p_init_user_param->extFunction;
    if (pInst->extFunction & EXT_FUNC_MAX_FRAMEBUFFER) {
      int32_t max_fb_width  = (pInst->mMaxAdaptiveWidth < 0)
                        ? AVAILABLE_4K_MAX_WIDTH : pInst->mMaxAdaptiveWidth;
      int32_t max_fb_height = (pInst->mMaxAdaptiveHeight < 0)
                        ? AVAILABLE_4K_MAX_HEIGHT : pInst->mMaxAdaptiveHeight;

      pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_uiDecOptFlags |= (1 << 16);
      pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_Reserved[3] = max_fb_width;
      pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_Reserved[4] = max_fb_height;

      if (pInst->extFunction & EXT_FUNC_USE_MAP_CONVERTER) {
        pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_Reserved[5] = 10; // Max Bitdepth
      } else {
        // [work-around] reduce total memory size of min. frame buffers
        pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_Reserved[5] = 8; // Max Bitdepth
        pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_uiDecOptFlags |= (1 << 3); // 10 to 8 bit shit
      }

      LOGI("[4KD2-%d] Set framebuffer into max (%d x %d, %u bit)",
            pInst->vdec_instance_index, max_fb_width, max_fb_height, pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_Reserved[5]);
      }

      pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_iFilePlayEnable = p_init_param->m_bFilePlayEnable;

  #if OPTEE_VPU_FW_ACCESS_MODE
      pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_uiDecOptFlags |= (1 << 7);
  #endif

#if (WAVE5_COMMAND_QUEUE_DEPTH == 2)
      if (pInst->extFunction & EXT_FUNC_4KD2_USE_CQ_LEVEL_1) {
        tcc_printf("[4KD2-%d] force set : COMMAND_QUEUE_DEPTH = 1\n",pInst->vdec_instance_index);
      } else {
        pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_uiDecOptFlags |= OPT_COMMEND_QUEUE_DEPTH_2;
        tcc_printf("[4KD2-%d] COMMAND_QUEUE_DEPTH = 2\n",pInst->vdec_instance_index);
      }
#endif
      pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_iBitstreamBufSize = p_init_param->m_iBitstreamBufSize;
      ret = vpu_4k_d2_dec_ready(&pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit, pInst);
      if (ret != RETCODE_SUCCESS) {
        return ret;
      }

      DSTATUS("[4KD2-%d] workbuff %#zx/%#zx, Reg: %#zx, format: %d, Stream(%#zx/%#zx, %d)", pInst->vdec_instance_index,
              (size_t)pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_BitWorkAddr[PA],
              (size_t)pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_BitWorkAddr[VA],
              (size_t)pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_RegBaseVirtualAddr,
              pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_iBitstreamFormat,
              (size_t)pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_BitstreamBufAddr[PA],
              (size_t)pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_BitstreamBufAddr[VA],
              pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_iBitstreamBufSize);

      DSTATUS("[4KD2-%d] optFlag %#x, Userdata(%d), VCache: %d, Inter: %d, PlayEn: %d", pInst->vdec_instance_index,
              pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_uiDecOptFlags,
              pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_bEnableUserData,
              0, //pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_bEnableVideoCache
              pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_bCbCrInterleaveMode,
              pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_iFilePlayEnable);

      if (pInst->extFunction & EXT_FUNC_NO_BUFFER_DELAY) {
        LOGI("[4KD2-%d] Set no-buffer-delay for non-B-frame stream", pInst->vdec_instance_index);
        pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_uiDecOptFlags |= (1 << 2);
      }

  #ifdef USE_MAP_CONVERTER
      if (pInst->extFunction & EXT_FUNC_USE_MAP_CONVERTER) {
        LOGI("[4KD2-%d] Enable Map Converter Mode !! (%d x %d)", pInst->vdec_instance_index,
                  p_init_param->m_iPicWidth, p_init_param->m_iPicHeight);
      }
      else
  #endif
      {
        LOGI("[4KD2-%d] Disable Map Converter Mode !! (%d x %d)", pInst->vdec_instance_index,
                p_init_param->m_iPicWidth, p_init_param->m_iPicHeight);
        // No compressed output!!, Must use 2 x framebuffer.
        pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_uiDecOptFlags |= WAVE5_WTL_ENABLE;
      }

      if (pInst->extFunction & EXT_FUNC_DISABLE_10BIT_OUTPUT) {
        pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_uiDecOptFlags |= WAVE5_10BITS_DISABLE;
      }

#if DISABLE_4KD2_10BIT_OUT
      LOGI("[4KD2-%d] $ Disable 10bit output $", pInst->vdec_instance_index);
      pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_uiDecOptFlags |= WAVE5_10BITS_DISABLE;
#endif
      DSTATUS("[4KD2-%d] Format: %d, Stream(%#zx, %d)", pInst->vdec_instance_index,
              pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_iBitstreamFormat,
              (size_t)pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_BitstreamBufAddr[PA],
              pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_iBitstreamBufSize);
 
      ret = vpu_4k_d2_cmd_process(V_DEC_INIT, &pInst->gsVpu4KD2InitInfo, pInst);
      if (ret != RETCODE_SUCCESS) {
        LOGE("[4KD2-%d] V_DEC_INIT failed Error code is %#x", pInst->vdec_instance_index, ret);
        return -ret;
      }

      LOGD("[4KD2-%d] optFlag %#x, Userdata(%d), Inter: %d, PlayEn: %d", pInst->vdec_instance_index,
              pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_uiDecOptFlags,
              pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_bEnableUserData,
              pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_bCbCrInterleaveMode,
              pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_iFilePlayEnable);

      if (pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_Reserved[10] == 5) {
        LOGW("[4KD2-%d] map converter mode is refused !!", pInst->vdec_instance_index);

        pInst->extFunction &= ~EXT_FUNC_USE_MAP_CONVERTER;
        p_init_user_param->extFunction &= ~EXT_FUNC_USE_MAP_CONVERTER;
      }

      if (!(pInst->extFunction & (EXT_FUNC_MEM_PROTECTION_WITH_INSTREAM | EXT_FUNC_MEM_PROTECTION_ONLY)))
      {
        memset((void*)(pInst->gsBitstreamBufAddr[VA] + (pInst->gsBitstreamBufSize - 100)), 0x00, 100);
        pInst->gsVpu4KD2Version.pszVersion =
                    (char*)(pInst->gsBitstreamBufAddr[K_VA] + (pInst->gsBitstreamBufSize - 100));
        pInst->gsVpu4KD2Version.pszBuildData =
                    (char*)(pInst->gsBitstreamBufAddr[K_VA] + (pInst->gsBitstreamBufSize - 50));
        ret = vpu_4k_d2_cmd_process(V_GET_VPU_VERSION, &pInst->gsVpu4KD2Version, pInst);
        if (ret != RETCODE_SUCCESS) {
          //If this operation returns fail, it doesn't mean that there's a problem in vpu
          //so do not return error to host.
          DPRINTF("[4KD2-%d] V_GET_4KD2_VERSION failed Error code is %#x", pInst->vdec_instance_index, ret);
        } else {
          if (0 <= ioctl(pInst->mgr_fd, VPU_CHECK_CODEC_STATUS, &pInst->vpu_closed)) {
            LOGD("[4KD2-%d] multi-instance status: DEC [%d:%s][%d:%s][%d:%s][%d:%s], ENC [%d:%s]", pInst->vdec_instance_index,
                      VPU_DEC, pInst->vpu_closed[VPU_DEC] ? "x" : "o",
                      VPU_DEC_EXT, pInst->vpu_closed[VPU_DEC_EXT] ? "x" : "o",
                      VPU_DEC_EXT2, pInst->vpu_closed[VPU_DEC_EXT2] ? "x" : "o",
                      VPU_DEC_EXT3, pInst->vpu_closed[VPU_DEC_EXT3] ? "x" : "o",
                      VPU_ENC, pInst->vpu_closed[VPU_ENC] ? "x" : "o");
          }

          LOGI("[4KD2-%d] V_GET_4KD2_VERSION OK. Version is %.34s and it's built at %.10s", pInst->vdec_instance_index,
                (char *)(pInst->gsBitstreamBufAddr[VA] + (pInst->gsBitstreamBufSize - 100)),
                (char *)(pInst->gsBitstreamBufAddr[VA] + (pInst->gsBitstreamBufSize - 50)));
        }
      }
      pInst->vdec_codec_opened = 1;
    }
    break;

    case VDEC_DEC_SEQ_HEADER:
    {
      vdec_input_t* p_input_param = (vdec_input_t*)pParam1;
      vdec_output_t* p_output_param = (vdec_output_t*)pParam2;
      int32_t seq_stream_size = (p_input_param->m_iInpLen > pInst->gsMaxBitstreamSize)
                              ? pInst->gsMaxBitstreamSize : p_input_param->m_iInpLen;
      uint32_t iIsThumbnail = p_input_param->m_iIsThumbnail;

      vpu_4K_D2_dec_initial_info_t *pV4kd2DecInitialInfo =
        /*  pInst->diminishedInputCopy ? &pInst->gsVpu4KD2InOutInfo.gsV4kd2DecInitialInfo : */ //helena
          &pInst->gsVpu4KD2SeqHeaderInfo.gsV4kd2DecInitialInfo;

      if (pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_iFilePlayEnable) {
        if (((codec_addr_t)p_input_param->m_pInp[PA] == pInst->gsBitstreamBufAddr[PA]) &&
            ((codec_addr_t)p_input_param->m_pInp[VA] == pInst->gsBitstreamBufAddr[VA]))
        {
          pInst->gsVpu4KD2InOutInfo.gsV4kd2DecInput.m_BitstreamDataAddr[PA] = pInst->gsBitstreamBufAddr[PA];
          pInst->gsVpu4KD2InOutInfo.gsV4kd2DecInput.m_BitstreamDataAddr[VA] = pInst->gsBitstreamBufAddr[K_VA];
        }
        else {
          pInst->gsVpu4KD2InOutInfo.gsV4kd2DecInput.m_BitstreamDataAddr[PA] = pInst->gsBitstreamBufAddr[PA];
          pInst->gsVpu4KD2InOutInfo.gsV4kd2DecInput.m_BitstreamDataAddr[VA] = pInst->gsBitstreamBufAddr[K_VA];
          (void)memcpy((void *)pInst->gsBitstreamBufAddr[VA], (void *)p_input_param->m_pInp[VA], seq_stream_size);
        }

        if ((DEBUG_ON == 1) && !(pInst->extFunction & (EXT_FUNC_MEM_PROTECTION_WITH_INSTREAM | EXT_FUNC_MEM_PROTECTION_ONLY))) {
          uint8_t* ps = (uint8_t*)pInst->gsBitstreamBufAddr[VA];
          DSTATUS("[4KD2-%d Seq %d]", pInst->vdec_instance_index, seq_stream_size);
          //vdec_hexdump(ps, seq_stream_size > 80 ? 80 : seq_stream_size, "Seq");
        }
      } else {
        seq_stream_size = 1;
      }

      DSTATUS("[4KD2-%d] VDEC_DEC_SEQ_HEADER start: len = %d / %d", pInst->vdec_instance_index, seq_stream_size, p_input_param->m_iInpLen);
      ret = vpu_4k_d2_dec_seq_header(seq_stream_size, iIsThumbnail, pInst);
      if (ret != RETCODE_SUCCESS) {
        return ret;
      }

    #ifdef VPU_ALL_FRAME_DUMP
      save_input_stream("seq_stream.bin", seq_stream_size, pInst, 0, p_input_param->m_pInp[VA]);
    #endif

      p_output_param->m_pInitialInfo = &pInst->gsCommDecInfo;

      /* hdr metadata */
      LOGD("[4KD2-%d] HasColorAspects(%d), HasHdrInfo(%d), hasUserData(%d)", pInst->vdec_instance_index,
              pInst->bHasColorAspects, pInst->bHasHdrInfo, pV4kd2DecInitialInfo->m_uiUserData);

      if (pV4kd2DecInitialInfo->m_uiUserData)
      {
        if (vpu_4k_d2_set_user_data_info(&pV4kd2DecInitialInfo->m_UserDataInfo,
                                         &p_output_param->m_UserDataInfo) < 0)
        {
            pV4kd2DecInitialInfo->m_uiUserData = 0;
        }

      #if defined(SUPPORT_HDMI_DRM_DIRECT_CONFIG)
        notify_hdmi_drm_config(pInst, &p_output_param->m_UserDataInfo);
      #else
        update_hdr_color_config(pInst, &p_output_param->m_UserDataInfo);
      #endif
        pInst->bHasHdrInfo = 1;
      } else {
      #if defined(SUPPORT_HDMI_DRM_DIRECT_CONFIG)
        if (pInst->bHasColorAspects && pInst->bHasHdrInfo) {
          notify_hdmi_drm_config(pInst, NULL); // vp9 hdr static meta
          fill_hdr_static_info_into_output_userdata(pInst, &p_output_param->m_UserDataInfo);

          pInst->bUseUserDataBearer = 1;
        }
      #endif
        pInst->bHasHdrInfo = 0;
      }

      /* color aspects */
      if (pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_iBitstreamFormat == STD_HEVC) {
        pInst->stColorAspects.fullRange = (uint32_t)pV4kd2DecInitialInfo->m_UserDataInfo.m_VuiParam.video_full_range_flag;
        pInst->stColorAspects.primaries = pV4kd2DecInitialInfo->m_UserDataInfo.m_VuiParam.colour_primaries;
        pInst->stColorAspects.matrix_coeffs = pV4kd2DecInitialInfo->m_UserDataInfo.m_VuiParam.matrix_coefficients;
        pInst->stColorAspects.transfer = pV4kd2DecInitialInfo->m_UserDataInfo.m_VuiParam.transfer_characteristics;

        LOGD("[4KD2-%d][SEQ_HEADER][HEVC|CA] R:%d, P:%d, M:%d, T:%d", pInst->vdec_instance_index,
                pInst->stColorAspects.fullRange, pInst->stColorAspects.primaries,
                pInst->stColorAspects.matrix_coeffs, pInst->stColorAspects.transfer);
      }

      p_output_param->m_pInitialInfo_4KD2 = pV4kd2DecInitialInfo;
      // to support 10 bits display
      p_output_param->m_pInitialInfo_4KD2->m_iFrameBufferFormat = pV4kd2DecInitialInfo->m_iFrameBufferFormat;

      LOGD("[4KD2-%d] m_iFrameBufferFormat: %d", pInst->vdec_instance_index, pV4kd2DecInitialInfo->m_iFrameBufferFormat);

      // check the maximum/minimum video resolution limitation
      int32_t max_width, max_height;
      int32_t min_width, min_height;

      max_width   = (AVAILABLE_4K_MAX_WIDTH  + 15) & 0xFFF0;
      max_height  = (AVAILABLE_4K_MAX_HEIGHT + 15) & 0xFFF0;
      min_width   = AVAILABLE_MIN_WIDTH;
      min_height  = AVAILABLE_MIN_HEIGHT;

      //testStagefright_bug_34360591, testStagefright_bug_32873375
      int32_t pic_width  = pV4kd2DecInitialInfo->m_iPicWidth;
      int32_t pic_height = pV4kd2DecInitialInfo->m_iPicHeight;
      if (pInst->extFunction & EXT_FUNC_MAX_FRAMEBUFFER) {
        pic_width  = pInst->mRealPicWidth  + pV4kd2DecInitialInfo->m_PicCrop.m_iCropLeft
                   + pV4kd2DecInitialInfo->m_PicCrop.m_iCropRight;
        pic_height = pInst->mRealPicHeight + pV4kd2DecInitialInfo->m_PicCrop.m_iCropBottom
                   + pV4kd2DecInitialInfo->m_PicCrop.m_iCropTop;
      }

      if ((pic_width > max_width) || ((pic_width * pic_height) > AVAILABLE_4K_MAX_REGION) ||
          (pic_width < min_width) || (pic_height < min_height))
      {
        ret = 0 - RETCODE_INVALID_STRIDE;
        DPRINTF("[4KD2-%d] VDEC_DEC_SEQ_HEADER - don't support the resolution %dx%d",
                pInst->vdec_instance_index, pic_width, pic_height);
        return ret;
      }

      LOGD("[4KD2-%d] [%d x %d] VDEC_DEC_SEQ_HEADER - mem_free = %#x",
              pInst->vdec_instance_index, pic_width, pic_height, cdk_sys_final_free_mem(pInst));
    }
    break;

    case VDEC_DECODE:
    {
      vdec_input_t* p_input_param = (vdec_input_t*)pParam1;
      vdec_output_t* p_output_param = (vdec_output_t*)pParam2;

      pInst->gsVpu4KD2InOutInfo.gsV4kd2DecInput.m_iBitstreamDataSize =
         (p_input_param->m_iInpLen > pInst->gsMaxBitstreamSize) ? pInst->gsMaxBitstreamSize : p_input_param->m_iInpLen;

      if (pInst->extFunction & EXT_FUNC_MEM_PROTECTION_WITH_INSTREAM) {
        //LOGI("Usable Input Addr: %#x", p_input_param->m_pInp[PA]);
        pInst->gsVpu4KD2InOutInfo.gsV4kd2DecInput.m_BitstreamDataAddr[PA] = (codec_addr_t)p_input_param->m_pInp[PA];
        pInst->gsVpu4KD2InOutInfo.gsV4kd2DecInput.m_BitstreamDataAddr[VA] = (codec_addr_t)p_input_param->m_pInp[VA];
      }
      else if (pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_iFilePlayEnable) {
        if ((codec_addr_t)p_input_param->m_pInp[PA] == pInst->gsBitstreamBufAddr[PA] &&
            (codec_addr_t)p_input_param->m_pInp[VA] == pInst->gsBitstreamBufAddr[VA])
        {
          pInst->gsVpu4KD2InOutInfo.gsV4kd2DecInput.m_BitstreamDataAddr[PA] = pInst->gsBitstreamBufAddr[PA];
          pInst->gsVpu4KD2InOutInfo.gsV4kd2DecInput.m_BitstreamDataAddr[VA] = pInst->gsBitstreamBufAddr[K_VA];
        }
        else {
          if (pInst->gsVpu4KD2InOutInfo.gsV4kd2DecInput.m_iBitstreamDataSize > 0) {
          #ifdef USE_PREV_STREAMBUFF_DECODING
            if (pInst->gsCurrStreamBuffer_index == 0) {
              pInst->gsVpu4KD2InOutInfo.gsV4kd2DecInput.m_BitstreamDataAddr[PA] = pInst->gsBitstreamBufAddr[PA];
              pInst->gsVpu4KD2InOutInfo.gsV4kd2DecInput.m_BitstreamDataAddr[VA] = pInst->gsBitstreamBufAddr[K_VA];
              (void)memcpy((void*)pInst->gsBitstreamBufAddr[VA], (void*)p_input_param->m_pInp[VA],
                      pInst->gsVpu4KD2InOutInfo.gsV4kd2DecInput.m_iBitstreamDataSize);
            } else {
              pInst->gsVpu4KD2InOutInfo.gsV4kd2DecInput.m_BitstreamDataAddr[PA] =
                  pInst->gsBitstreamBufAddr[PA] + pInst->gsBitstreamBufSize / 2;
              pInst->gsVpu4KD2InOutInfo.gsV4kd2DecInput.m_BitstreamDataAddr[VA] =
                  pInst->gsBitstreamBufAddr[K_VA] + pInst->gsBitstreamBufSize / 2;
              (void)memcpy((void *)(pInst->gsBitstreamBufAddr[VA] + pInst->gsBitstreamBufSize / 2),
                     (void *)p_input_param->m_pInp[VA], pInst->gsVpu4KD2InOutInfo.gsV4kd2DecInput.m_iBitstreamDataSize);
            }
          #else
            pInst->gsVpu4KD2InOutInfo.gsV4kd2DecInput.m_BitstreamDataAddr[PA] = pInst->gsBitstreamBufAddr[PA];
            pInst->gsVpu4KD2InOutInfo.gsV4kd2DecInput.m_BitstreamDataAddr[VA] = pInst->gsBitstreamBufAddr[K_VA];

            (void)memcpy((void *)pInst->gsBitstreamBufAddr[VA], (void *)p_input_param->m_pInp[VA],
                    pInst->gsVpu4KD2InOutInfo.gsV4kd2DecInput.m_iBitstreamDataSize);
          #endif
          }
        }
      } else {
        pInst->gsVpu4KD2InOutInfo.gsV4kd2DecInput.m_BitstreamDataAddr[PA] = pInst->gsBitstreamBufAddr[PA];
        pInst->gsVpu4KD2InOutInfo.gsV4kd2DecInput.m_BitstreamDataAddr[VA] = pInst->gsBitstreamBufAddr[K_VA];
        pInst->gsVpu4KD2InOutInfo.gsV4kd2DecInput.m_iBitstreamDataSize = 1;
      }

      if (pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_bEnableUserData) {
        pInst->gsVpu4KD2InOutInfo.gsV4kd2DecInput.m_UserDataAddr[PA] = pInst->gsUserdataBufAddr[PA];
        pInst->gsVpu4KD2InOutInfo.gsV4kd2DecInput.m_UserDataAddr[VA] = pInst->gsUserdataBufAddr[K_VA];
        pInst->gsVpu4KD2InOutInfo.gsV4kd2DecInput.m_iUserDataBufferSize = pInst->gsUserdataBufSize;
      }

      if (p_input_param->m_iFrameSearchEnable) {
        LOGD("[4KD2-%d] I-Frame Search Mode Enabled !!", pInst->vdec_instance_index);
        pInst->gsVpu4KD2InOutInfo.gsV4kd2DecInput.m_iSkipFrameMode = 1;
      } else {
        pInst->gsVpu4KD2InOutInfo.gsV4kd2DecInput.m_iSkipFrameMode = 0;
      }

    #ifdef VPU_ALL_FRAME_DUMP
      if (pInst->bStartInputDump && pInst->gsVpu4KD2InOutInfo.gsV4kd2DecInput.m_iBitstreamDataSize > 0) {
        save_input_stream("all_stream.bin", pInst->gsVpu4KD2InOutInfo.gsV4kd2DecInput.m_iBitstreamDataSize,
                               pInst, 1, p_input_param->m_pInp[VA]);
      }
    #endif

      if (pInst->extFunction & EXT_FUNC_HIDE_SUPERFRAME_OUTPUT) {
          pInst->gsVpu4KD2InOutInfo.gsV4kd2DecInput.m_Reserved[20] = 20;
      }

      // Start decoding a frame.
      ret = vpu_4k_d2_cmd_process(V_DEC_DECODE, &pInst->gsVpu4KD2InOutInfo, pInst);
      pInst->total_frm++;

    #ifdef USE_PREV_STREAMBUFF_DECODING
      if (pInst->bBitstreamSplitIndexPinned == 0) {
          pInst->gsCurrStreamBuffer_index ^= 1;
      }
    #endif

      if (ret != RETCODE_SUCCESS) {
        if (ret != RETCODE_INSUFFICIENT_BITSTREAM){
          LOGE("[4KD2-%d] VPU_4K_D2_DEC_DECODE Error !! (ret-code:%d, max-fb:%d, dispIdx:%d, buf-full:%d)",
                 pInst->vdec_instance_index, ret, (pInst->extFunction & EXT_FUNC_MAX_FRAMEBUFFER) ? 1 : 0,
                 pInst->gsVpu4KD2InOutInfo.gsV4kd2DecOutput.m_DecOutInfo.m_iDispOutIdx,
                 (pInst->gsVpu4KD2InOutInfo.gsV4kd2DecOutput.m_DecOutInfo.m_iDecodingStatus == VPU_DEC_BUF_FULL) ? 1 : 0);
        }
    #ifdef USE_PREV_STREAMBUFF_DECODING
        if (ret == RETCODE_INVALID_STRIDE && (pInst->extFunction & EXT_FUNC_MAX_FRAMEBUFFER) &&
                pInst->gsVpu4KD2InOutInfo.gsV4kd2DecOutput.m_DecOutInfo.m_iDispOutIdx == -3 &&
                pInst->gsVpu4KD2InOutInfo.gsV4kd2DecOutput.m_DecOutInfo.m_iDecodingStatus == VPU_DEC_BUF_FULL)
        {
            LOGE("[4KD2-%d][MAX-FB][BUF-FULL] Ignore error code (%d -> %d)", pInst->vdec_instance_index, -ret, RETCODE_SUCCESS);
            ret = RETCODE_SUCCESS;
        }
        else
    #endif
        {
        	return -ret;
        }
      }

      if (pInst->gsVpu4KD2InOutInfo.gsV4kd2DecOutput.m_DecOutInfo.m_iPicType == 0) {
        DSTATUS("[4KD2-%d] I-Frame (%d)", pInst->vdec_instance_index, pInst->total_frm);
      }

      vpu_4k_d2_copy_dec_out_info(&pInst->gsVpu4KD2InOutInfo.gsV4kd2DecOutput, p_output_param, 0, pInst);
      p_output_param->m_pInitialInfo = &pInst->gsCommDecInfo;

      p_output_param->m_pDispOut[VA][COMP_Y] = (uint8_t *)vpu_getFrameBufVirtAddr(p_output_param->m_pDispOut[VA][COMP_Y], K_VA, pInst);
      p_output_param->m_pDispOut[VA][COMP_U] = (uint8_t *)vpu_getFrameBufVirtAddr(p_output_param->m_pDispOut[VA][COMP_U], K_VA, pInst);
      p_output_param->m_pDispOut[VA][COMP_V] = (uint8_t *)vpu_getFrameBufVirtAddr(p_output_param->m_pDispOut[VA][COMP_V], K_VA, pInst);

      p_output_param->m_pCurrOut[VA][COMP_Y] = (uint8_t *)vpu_getFrameBufVirtAddr(p_output_param->m_pCurrOut[VA][COMP_Y], K_VA, pInst);
      p_output_param->m_pCurrOut[VA][COMP_U] = (uint8_t *)vpu_getFrameBufVirtAddr(p_output_param->m_pCurrOut[VA][COMP_U], K_VA, pInst);
      p_output_param->m_pCurrOut[VA][COMP_V] = (uint8_t *)vpu_getFrameBufVirtAddr(p_output_param->m_pCurrOut[VA][COMP_V], K_VA, pInst);

      DISPLAY_BUFFER("[4KD2-%d] Display :: %d :: %p %p %p / %p %p %p",
                      pInst->vdec_instance_index, p_output_param->m_DecOutInfo.m_iDispOutIdx,
                      p_output_param->m_pDispOut[PA][COMP_Y],
                      p_output_param->m_pDispOut[PA][COMP_U],
                      p_output_param->m_pDispOut[PA][COMP_V],
                      p_output_param->m_pDispOut[VA][COMP_Y],
                      p_output_param->m_pDispOut[VA][COMP_U],
                      p_output_param->m_pDispOut[VA][COMP_V]);

      DISPLAY_BUFFER("[4KD2-%d] Dec :: %d :: %p %p %p / %p %p %p",
                      pInst->vdec_instance_index, p_output_param->m_DecOutInfo.m_iDecodedIdx,
                      p_output_param->m_pCurrOut[PA][COMP_Y],
                      p_output_param->m_pCurrOut[PA][COMP_U],
                      p_output_param->m_pCurrOut[PA][COMP_V],
                      p_output_param->m_pCurrOut[VA][COMP_Y],
                      p_output_param->m_pCurrOut[VA][COMP_U],
                      p_output_param->m_pCurrOut[VA][COMP_V]);

      if (pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_bEnableUserData) {
        codec_addr_t addr_gap = pInst->gsUserdataBufAddr[K_VA]
                              - pInst->gsVpu4KD2InOutInfo.gsV4kd2DecOutput.m_DecOutInfo.m_UserDataAddress[VA];

        p_output_param->m_DecOutInfo.m_UserDataAddress[VA] = pInst->gsUserdataBufAddr[VA] + addr_gap;

        if (pInst->gsVpu4KD2InOutInfo.gsV4kd2DecOutput.m_DecOutInfo.m_uiUserData
        ){
          p_output_param->m_IsHdrPlus = 0;
          if (vpu_4k_d2_set_user_data_info(&pInst->gsVpu4KD2InOutInfo.gsV4kd2DecOutput.m_DecOutInfo.m_UserDataInfo,
                                          &p_output_param->m_UserDataInfo) < 0) {
            pInst->gsVpu4KD2InOutInfo.gsV4kd2DecOutput.m_DecOutInfo.m_uiUserData = 0;
          }

        #if defined(SUPPORT_HDMI_DRM_DIRECT_CONFIG)
          notify_hdmi_drm_config(pInst, &p_output_param->m_UserDataInfo);
        #else
          update_hdr_color_config(pInst, &p_output_param->m_UserDataInfo);
        #endif
          pInst->bHasHdrInfo = 1;
        } else {
        #if defined(SUPPORT_HDMI_DRM_DIRECT_CONFIG)
          if (pInst->bUseUserDataBearer) {
            fill_hdr_static_info_into_output_userdata(pInst, &p_output_param->m_UserDataInfo);
          }
        #endif
          pInst->bHasHdrInfo = 0;
        }

        /* color aspects */
        if (pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_iBitstreamFormat == STD_HEVC) {
          vpu_4K_D2_dec_UserData_info_t *pstUserDataInfo =
              &pInst->gsVpu4KD2InOutInfo.gsV4kd2DecOutput.m_DecOutInfo.m_UserDataInfo;

          pInst->stColorAspects.fullRange = (uint32_t)pstUserDataInfo->m_VuiParam.video_full_range_flag;
          pInst->stColorAspects.primaries = pstUserDataInfo->m_VuiParam.colour_primaries;
          pInst->stColorAspects.matrix_coeffs = pstUserDataInfo->m_VuiParam.matrix_coefficients;
          pInst->stColorAspects.transfer = pstUserDataInfo->m_VuiParam.transfer_characteristics;

          LOGI("[4KD2-%d][DEC][HEVC|CA] R:%d, P:%d, M:%d, T:%d", pInst->vdec_instance_index,
                  pInst->stColorAspects.fullRange, pInst->stColorAspects.primaries,
                  pInst->stColorAspects.matrix_coeffs, pInst->stColorAspects.transfer);
        }

      } else {
        p_output_param->m_DecOutInfo.m_UserDataAddress[VA] = 0;
      }
      p_output_param->m_pInitialInfo_4KD2->m_uiUserData = pInst->gsVpu4KD2InOutInfo.gsV4kd2DecOutput.m_DecOutInfo.m_uiUserData;

    #ifdef VPU_OUT_FRAME_DUMP
      if (pInst->gsVpu4KD2InOutInfo.gsV4kd2DecOutput.m_DecOutInfo.m_iOutputStatus == VPU_DEC_OUTPUT_SUCCESS) {
          vpu_4K_D2_dec_initial_info_t *gsV4kd2DecInitialInfo = &pInst->gsVpu4KD2SeqHeaderInfo.gsV4kd2DecInitialInfo;
          //LOGE("Displayed addr %#x", p_output_param->m_pDispOut[PA][0]);
          save_decoded_frame((uint8_t *)vpu_getFrameBufVirtAddr(p_output_param->m_pDispOut[PA][COMP_Y], PA, pInst),
                             (uint8_t *)vpu_getFrameBufVirtAddr(p_output_param->m_pDispOut[PA][COMP_U], PA, pInst),
                             (uint8_t *)vpu_getFrameBufVirtAddr(p_output_param->m_pDispOut[PA][COMP_V], PA, pInst),
                             gsV4kd2DecInitialInfo->m_iPicWidth,
                             gsV4kd2DecInitialInfo->m_iPicHeight, pInst);
      }
    #endif

      DISPLAY_BUFFER("[4KD2-%d] Display idx = %d", pInst->vdec_instance_index,
              pInst->gsVpu4KD2InOutInfo.gsV4kd2DecOutput.m_DecOutInfo.m_iDispOutIdx);
    }
    break;

    case VDEC_GET_OUTPUT_INFO:
    {
      vdec_output_t *p_output_param = (vdec_output_t *)pParam2;

      ret = vpu_4k_d2_cmd_process(V_DEC_GET_OUTPUT_INFO, &pInst->gsVpu4KD2InOutInfo, pInst);
      if (ret != RETCODE_SUCCESS) {
        LOGE("[4KD2-%d] DEC_GET_OUTPUT_INFO failed Error !! (ret: %#x)", pInst->vdec_instance_index, ret);
        return (ret != RETCODE_REPORT_NOT_READY) ? -ret : ret;
      }

      vpu_4k_d2_copy_dec_out_info(&pInst->gsVpu4KD2InOutInfo.gsV4kd2DecOutput, p_output_param, 0, pInst);
      p_output_param->m_pInitialInfo = &pInst->gsCommDecInfo;

      p_output_param->m_pDispOut[VA][COMP_Y] = (uint8_t *)vpu_getFrameBufVirtAddr(p_output_param->m_pDispOut[VA][COMP_Y], K_VA, pInst);
      p_output_param->m_pDispOut[VA][COMP_U] = (uint8_t *)vpu_getFrameBufVirtAddr(p_output_param->m_pDispOut[VA][COMP_U], K_VA, pInst);
      p_output_param->m_pDispOut[VA][COMP_V] = (uint8_t *)vpu_getFrameBufVirtAddr(p_output_param->m_pDispOut[VA][COMP_V], K_VA, pInst);

      p_output_param->m_pCurrOut[VA][COMP_Y] = (uint8_t *)vpu_getFrameBufVirtAddr(p_output_param->m_pCurrOut[VA][COMP_Y], K_VA, pInst);
      p_output_param->m_pCurrOut[VA][COMP_U] = (uint8_t *)vpu_getFrameBufVirtAddr(p_output_param->m_pCurrOut[VA][COMP_U], K_VA, pInst);
      p_output_param->m_pCurrOut[VA][COMP_V] = (uint8_t *)vpu_getFrameBufVirtAddr(p_output_param->m_pCurrOut[VA][COMP_V], K_VA, pInst);

      if (pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_bEnableUserData) {
        codec_addr_t addr_gap = pInst->gsUserdataBufAddr[K_VA]
                              - pInst->gsVpu4KD2InOutInfo.gsV4kd2DecOutput.m_DecOutInfo.m_UserDataAddress[VA];
        p_output_param->m_DecOutInfo.m_UserDataAddress[VA] = pInst->gsUserdataBufAddr[VA] + addr_gap;
      } else {
        p_output_param->m_DecOutInfo.m_UserDataAddress[VA] = 0;
      }
    }
    break;

    case VDEC_GET_RING_BUFFER_STATUS:
    {
      vdec_ring_buffer_out_t* p_out_param = (vdec_ring_buffer_out_t*)pParam2;

      // get the available space in the ring buffer
      ret = vpu_4k_d2_cmd_process(V_GET_RING_BUFFER_STATUS, &pInst->gsVpu4KD2BufStatus, pInst);
      if (ret != RETCODE_SUCCESS) {
        DPRINTF("[4KD2-%d] GET_RING_BUFFER_STATUS failed Error code is %#x", pInst->vdec_instance_index, ret);
        return -ret;
      }

      p_out_param->m_ulAvailableSpaceInRingBuffer = pInst->gsVpu4KD2BufStatus.gsV4kd2DecRingStatus.m_ulAvailableSpaceInRingBuffer;
      p_out_param->m_ptrReadAddr_PA = pInst->gsVpu4KD2BufStatus.gsV4kd2DecRingStatus.m_ptrReadAddr_PA;
      p_out_param->m_ptrWriteAddr_PA = pInst->gsVpu4KD2BufStatus.gsV4kd2DecRingStatus.m_ptrWriteAddr_PA;
    #if 0
      LOGE("[VDEC] [AVAIL: %8d] [RP: %#08X / WP: %#08X]",
              pInst->gsVpu4KD2BufStatus.gsV4kd2DecRingStatus.m_ulAvailableSpaceInRingBuffer,
              pInst->gsVpu4KD2BufStatus.gsV4kd2DecRingStatus.m_ptrReadAddr_PA,
              pInst->gsVpu4KD2BufStatus.gsV4kd2DecRingStatus.m_ptrWriteAddr_PA);
    #endif
    }
    break;

    case VDEC_FILL_RING_BUFFER:
    {
      vdec_ring_buffer_set_t* p_set_param = (vdec_ring_buffer_set_t*)pParam1;

      (void)memcpy((void*)pInst->gsIntermediateBufAddr[VA],(void*)p_set_param->m_pbyBuffer, p_set_param->m_uiBufferSize);
      pInst->gsVpu4KD2BufFill.gsV4kd2DecRingFeed.m_iOnePacketBufferSize = p_set_param->m_uiBufferSize;
      pInst->gsVpu4KD2BufFill.gsV4kd2DecRingFeed.m_OnePacketBufferAddr = pInst->gsIntermediateBufAddr[K_VA];

      ret = vpu_4k_d2_cmd_process(V_FILL_RING_BUFFER_AUTO, &pInst->gsVpu4KD2BufFill, pInst);
      if (ret != RETCODE_SUCCESS) {
        DPRINTF("[4KD2-%d] FILL_RING_BUFFER_AUTO failed Error code is %#x", pInst->vdec_instance_index, ret);
        return -ret;
      }
    }
    break;

    case VDEC_GET_INTERMEDIATE_BUF_INFO:
    {
      *(uint32_t *)pParam1 = pInst->gsIntermediateBufAddr[VA];
      *(uint32_t *)pParam2 = pInst->gsIntermediateBufSize;
      return 0;
    }
    break;

    case VDEC_UPDATE_WRITE_BUFFER_PTR:
    {
      pInst->gsVpu4KD2UpdateWP.iCopiedSize = (int32_t)pParam1;
      pInst->gsVpu4KD2UpdateWP.iFlushBuf = (int32_t)pParam2;

      ret = vpu_4k_d2_cmd_process(V_DEC_UPDATE_RINGBUF_WP, &pInst->gsVpu4KD2UpdateWP,  pInst);
      if (ret != RETCODE_SUCCESS) {
        DPRINTF("[4KD2-%d] UPDATE_WRITE_BUFFER_PTR failed Error code is %#x", pInst->vdec_instance_index, ret);
        return -ret;
      }
    }
    break;

    case VDEC_BUF_FLAG_CLEAR:
    {
      int32_t displayed_index = *(int32_t*)pParam1;
      CLEAR_BUFFER("[4KD2-%d] cleared index = %d", pInst->vdec_instance_index, displayed_index);

      ret = vpu_4k_d2_cmd_process(V_DEC_BUF_FLAG_CLEAR, &displayed_index, pInst);
      if (ret != RETCODE_SUCCESS) {
        DPRINTF("[4KD2-%d] DEC_BUF_FLAG_CLEAR failed Error code is %#x", pInst->vdec_instance_index, ret);
        return -ret;
      }
    }
    break;

    case VDEC_DEC_FLUSH_OUTPUT:
    {
      int32_t flush_all_magic = pParam1 ? *(int32_t *)pParam1 : 0;
      vdec_output_t *p_output_param = (vdec_output_t *)pParam2;

      pInst->gsVpu4KD2InOutInfo.gsV4kd2DecInput.m_Reserved[22] = flush_all_magic;
      pInst->gsVpu4KD2InOutInfo.gsV4kd2DecInput.m_BitstreamDataAddr[PA] = pInst->gsBitstreamBufAddr[PA];
      pInst->gsVpu4KD2InOutInfo.gsV4kd2DecInput.m_BitstreamDataAddr[VA] = pInst->gsBitstreamBufAddr[K_VA];
      pInst->gsVpu4KD2InOutInfo.gsV4kd2DecInput.m_iBitstreamDataSize = 0;
      pInst->gsVpu4KD2InOutInfo.gsV4kd2DecInput.m_iSkipFrameMode = VDEC_SKIP_FRAME_DISABLE;
  #if 0
      pInst->gsVpu4KD2InOutInfo.gsV4kd2DecInput.m_iFrameSearchEnable = 0;
      pInst->gsVpu4KD2InOutInfo.gsV4kd2DecInput.m_iSkipFrameNum = 0;
  #endif

      ret = vpu_4k_d2_cmd_process(V_DEC_FLUSH_OUTPUT, &pInst->gsVpu4KD2InOutInfo, pInst);

      if (ret != RETCODE_SUCCESS) {
        DPRINTF("[4KD2-%d] VDEC_DEC_FLUSH_OUTPUT failed Error code is %#x", pInst->vdec_instance_index, ret);
        return -ret;
      }

      vpu_4k_d2_copy_dec_out_info(&pInst->gsVpu4KD2InOutInfo.gsV4kd2DecOutput, p_output_param, 1, pInst);
      if(p_output_param->m_DecOutInfo.m_iWidth == 0 || p_output_param->m_DecOutInfo.m_iHeight == 0){
        LOGE("[4KD2-%d][VDEC_DEC_FLUSH_OUTPUT] Invalid Resolution: w x h = dec: %d x %d, disp: %d x %d, dst: %d x %d",
              pInst->vdec_instance_index,
              pInst->gsVpu4KD2InOutInfo.gsV4kd2DecOutput.m_DecOutInfo.m_iDecodedWidth,
              pInst->gsVpu4KD2InOutInfo.gsV4kd2DecOutput.m_DecOutInfo.m_iDecodedHeight,
              pInst->gsVpu4KD2InOutInfo.gsV4kd2DecOutput.m_DecOutInfo.m_iDisplayWidth,
              pInst->gsVpu4KD2InOutInfo.gsV4kd2DecOutput.m_DecOutInfo.m_iDisplayHeight,
              p_output_param->m_DecOutInfo.m_iWidth, p_output_param->m_DecOutInfo.m_iHeight);
      }
      p_output_param->m_pInitialInfo = &pInst->gsCommDecInfo;

      p_output_param->m_pDispOut[VA][COMP_Y] = (uint8_t *)vpu_getFrameBufVirtAddr(p_output_param->m_pDispOut[VA][COMP_Y], K_VA, pInst);
      p_output_param->m_pDispOut[VA][COMP_U] = (uint8_t *)vpu_getFrameBufVirtAddr(p_output_param->m_pDispOut[VA][COMP_U], K_VA, pInst);
      p_output_param->m_pDispOut[VA][COMP_V] = (uint8_t *)vpu_getFrameBufVirtAddr(p_output_param->m_pDispOut[VA][COMP_V], K_VA, pInst);

      p_output_param->m_pCurrOut[VA][COMP_Y] = (uint8_t *)vpu_getFrameBufVirtAddr(p_output_param->m_pCurrOut[VA][COMP_Y], K_VA, pInst);
      p_output_param->m_pCurrOut[VA][COMP_U] = (uint8_t *)vpu_getFrameBufVirtAddr(p_output_param->m_pCurrOut[VA][COMP_U], K_VA, pInst);
      p_output_param->m_pCurrOut[VA][COMP_V] = (uint8_t *)vpu_getFrameBufVirtAddr(p_output_param->m_pCurrOut[VA][COMP_V], K_VA, pInst);

      if (pInst->gsVpu4KD2InitInfo.gsV4kd2DecInit.m_bEnableUserData) {
        codec_addr_t addr_gap = pInst->gsUserdataBufAddr[K_VA]
                              - pInst->gsVpu4KD2InOutInfo.gsV4kd2DecOutput.m_DecOutInfo.m_UserDataAddress[VA];
        p_output_param->m_DecOutInfo.m_UserDataAddress[VA] = pInst->gsUserdataBufAddr[VA] + addr_gap;
      } else {
        p_output_param->m_DecOutInfo.m_UserDataAddress[VA] = 0;
      }
    }
    break;

    case VDEC_SW_RESET:
    {
      ret = vpu_4k_d2_cmd_process(V_DEC_SWRESET, NULL, pInst);
      if (ret != RETCODE_SUCCESS) {
        DPRINTF("[4KD2-%d] V_DEC_SWRESET failed Error code is %#x", pInst->vdec_instance_index, ret);
        return -ret;
      }
    }
    break;

    case VDEC_CLOSE:
    {
      if (pInst->vdec_codec_opened) {
        ret = vpu_4k_d2_cmd_process(V_DEC_CLOSE, &pInst->gsVpu4KD2InOutInfo, pInst);
        if (ret != RETCODE_SUCCESS) {
          DPRINTF("[4KD2-%d] DEC_CLOSE failed Error code is %#x", pInst->vdec_instance_index, ret);
          ret = -ret;
        }
        pInst->vdec_codec_opened = 0;
      }

      cdk_sys_free_virtual_addr((unsigned long*)pInst->gsBitstreamBufAddr[VA], pInst->gsBitstreamBufSize);
      cdk_sys_free_virtual_addr((unsigned long*)pInst->gsUserdataBufAddr[VA], pInst->gsUserdataBufSize);
      cdk_sys_free_virtual_addr((unsigned long*)pInst->gsBitWorkBufAddr[VA], pInst->gsBitWorkBufSize);
      cdk_sys_free_virtual_addr((unsigned long*)pInst->gsFrameBufAddr[VA], pInst->gsFrameBufSize);

      vpu_env_close(pInst);
    }
    break;

    default:
      DPRINTF("[4KD2-%d] Invalid Operation!!", pInst->vdec_instance_index);
      return -ret;
    }

#ifdef DEBUG_TIME_LOG
    end = systemTime(CLOCK_MONOTONIC);

    if (iOpCode == VDEC_INIT) {
      LOGD("[4KD2-%d] VDEC_INIT_TIME %d ms", pInst->vdec_instance_index, (end - start) / 1000000);
    } else if (iOpCode == VDEC_DEC_SEQ_HEADER) {
      LOGD("[4KD2-%d] VDEC_SEQ_TIME %d ms", pInst->vdec_instance_index, (end - start) / 1000000);
    } else if (iOpCode == VDEC_DECODE) {
      dt[time_cnt] = (end - start) * 1000 / CLOCKS_PER_SEC;
      total_dec_time += dt[time_cnt];
      if (time_cnt != 0 && time_cnt % 29 == 0) {
        LOGD("[4KD2-%d] VDEC_DEC_TIME %.1f ms: "
             "%3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, "
             "%3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d",
             pInst->vdec_instance_index, total_dec_time/(float)pInst->total_frm,
             dt[0], dt[1], dt[2], dt[3], dt[4], dt[5], dt[6], dt[7],
             dt[8], dt[9], dt[10], dt[11], dt[12], dt[13], dt[14], dt[15],
             dt[16], dt[17], dt[18], dt[19], dt[20], dt[21], dt[22], dt[23],
             dt[24], dt[25], dt[26], dt[27], dt[28], dt[29]);
        time_cnt = 0;
      } else {
        time_cnt++;
      }
    }
#endif
    return ret;
}
#endif //TCC_VPU_4K_D2_INCLUDE

