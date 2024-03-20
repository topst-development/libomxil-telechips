// SPDX-License-Identifier: LGPL-2.1-or later
/****************************************************************************
 *   FileName  : vdec_k_hevc.c
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

#ifdef TCC_HEVC_INCLUDE

#define LOG_TAG "VPU_DEC_K_HEVC"

#include "TCCMemory.h"

#include <sys/mman.h>
#include <errno.h>

#include <sys/ioctl.h>
#if defined(USE_COMMON_KERNEL_LOCATION)
#include <tcc_hevc_ioctl.h>
#else //use chipset folder
#include <mach/tcc_hevc_ioctl.h>
#endif

#include <dlfcn.h>

#include <memory.h>
#include <stdio.h>
#include <string.h>

/********************************************************************************************/
/* HEVC Setting */
/********************************************************************************************/
#   define HEVC_DISABLE_10BIT_OUT 1
/********************************************************************************************/

#if defined(TC_SECURE_MEMORY_COPY)
extern int
TC_SecureMemoryCopy(
  unsigned int paTarget,
  unsigned int paSource,
  unsigned int nSize
);
#endif


#ifdef HEVC_OUT_FRAME_DUMP
static void save_MapConverter_frame(unsigned char* Y, unsigned char* CB, unsigned char *OffY,  unsigned char *OffCB, int width, int height)
{
  FILE *pComY, *pComCB, *pOffY, *pOffCB;

  LOGW("save_MapConverter_frame: Y[%p] CB[%p] OffY[%p] OffCB[%p] size:%d x %d", Y, CB, OffY, OffCB, width, height);

  pComY = fopen("/data/frameComY.data", "ab+");
  pComCB = fopen("/data/frameComCB.data", "ab+");
  pOffY = fopen("/data/frameComOffY.data", "ab+");
  pOffCB = fopen("/data/frameComOffCB.data", "ab+");

  if (!pComY || !pComCB || !pOffY || !pOffCB) {
    LOGE("Cannot open '/data folder %p %p %p %p", pComY, pComCB, pOffY, pOffCB);
    return;
  }

  fwrite( Y, width*height, 1, pComY);
  fwrite( CB, width*height, 1, pComCB);
  fwrite( OffY, width*height/4, 1, pOffY);
  fwrite( OffCB, width*height/4, 1, pOffCB);

  fclose(pComY);
  fclose(pComCB);
  fclose(pOffY);
  fclose(pOffCB);
}
#endif//HEVC_OUT_FRAME_DUMP


#ifdef TCC_HEVC_INCLUDE

static void hevc_copy_dec_out_info(hevc_dec_output_t *src, vdec_output_t *dst, _vdec_*  pInst)
{
  (void)memcpy((uintptr_t)dst + sizeof(dec_output_info_t), (uintptr_t)src + sizeof(hevc_dec_output_info_t),
      sizeof(dst->m_pDispOut) + sizeof(dst->m_pCurrOut) + sizeof(dst->m_pPrevOut));

  dst->m_DecOutInfo.m_iPicType  = src->m_DecOutInfo.m_iPicType;
  dst->m_DecOutInfo.m_iDispOutIdx   = src->m_DecOutInfo.m_iDispOutIdx;
  dst->m_DecOutInfo.m_iDecodedIdx   = src->m_DecOutInfo.m_iDecodedIdx;
  dst->m_DecOutInfo.m_iOutputStatus   = src->m_DecOutInfo.m_iOutputStatus;
  dst->m_DecOutInfo.m_iDecodingStatus = src->m_DecOutInfo.m_iDecodingStatus;
  dst->m_DecOutInfo.m_iNumOfErrMBs  = src->m_DecOutInfo.m_iNumOfErrMBs;

  dst->m_DecOutInfo.m_iHeight     = src->m_DecOutInfo.m_iDisplayHeight;
  dst->m_DecOutInfo.m_iWidth      = src->m_DecOutInfo.m_iDisplayWidth;

  (void)memcpy(&dst->m_DecOutInfo.m_CropInfo, &src->m_DecOutInfo.m_DisplayCropInfo, sizeof(pic_crop_t));

  dst->m_DecOutInfo.m_UserDataAddress[0]  = src->m_DecOutInfo.m_UserDataAddress[0];
  dst->m_DecOutInfo.m_UserDataAddress[1]  = src->m_DecOutInfo.m_UserDataAddress[1];
  dst->m_DecOutInfo.m_iConsumedBytes  = src->m_DecOutInfo.m_iConsumedBytes;
  dst->m_DecOutInfo.m_iInvalidDispCount   = src->m_DecOutInfo.m_iInvalidDispCount;

#ifdef USE_MAP_CONVERTER
  (void)memcpy(&dst->m_MapConvInfo, &src->m_DecOutInfo.m_DispMapConvInfo, sizeof(hevc_dec_MapConv_info_t));

  DSTATUS("Map Converter Info from HEVC !!");

  DSTATUS("m_CompressedY_PA/m_CompressedCb_PA(0x%x/0x%x), m_FbcYOffsetAddr_PA/m_FbcCOffsetAddr_PA(0x%x/0x%x) !!",
      dst->m_MapConvInfo.m_CompressedY[0], dst->m_MapConvInfo.m_CompressedCb[0],
      dst->m_MapConvInfo.m_FbcYOffsetAddr[0], dst->m_MapConvInfo.m_FbcCOffsetAddr[0]);
  DSTATUS("m_CompressedY_VA/m_CompressedCb_VA(0x%x/0x%x), m_FbcYOffsetAddr_VA/m_FbcCOffsetAddr_VA(0x%x/0x%x) !!",
      dst->m_MapConvInfo.m_CompressedY[1], dst->m_MapConvInfo.m_CompressedCb[1],
      dst->m_MapConvInfo.m_FbcYOffsetAddr[1], dst->m_MapConvInfo.m_FbcCOffsetAddr[1]);
  DSTATUS("m_uiLumaStride/m_uiChromaStride(%d/%d), m_uiLumaBitDepth/m_uiChromaBitDepth(0x%x/0x%x), m_uiFrameEndian(%d) !!",
      dst->m_MapConvInfo.m_uiLumaStride, dst->m_MapConvInfo.m_uiChromaStride,
      dst->m_MapConvInfo.m_uiLumaBitDepth, dst->m_MapConvInfo.m_uiChromaBitDepth, dst->m_MapConvInfo.m_uiFrameEndian);
#endif
}

static int hevc_cmd_process(int cmd, unsigned long* args, _vdec_ *pVdec)
{
  int ret;
  int success = 0;
  _vdec_ * pInst = pVdec;
  int retry_cnt = POLL_RETRY_COUNT;
  int all_retry_cnt = CMD_RETRY_COUNT;

  if((ret = ioctl(pInst->dec_fd, cmd, args)) < 0)
  {
    if( ret == -0x999 )
    {
      LOGE("HEVC[%d] Invalid command(0x%x) ", pInst->vdec_instance_index, cmd);
      return RETCODE_INVALID_COMMAND;
    }
    else
    {
      LOGE("HEVC[%d] ioctl err[%s] : cmd = 0x%x", pInst->vdec_instance_index, strerror(errno), cmd);
    }
  }

Retry:
  while (retry_cnt > 0) {
    memset(pInst->tcc_event, 0, sizeof(pInst->tcc_event));
    pInst->tcc_event[0].fd = pInst->dec_fd;
    pInst->tcc_event[0].events = POLLIN;
  
    ret = poll((struct pollfd *)&pInst->tcc_event, 1, 1000); // 1 sec
    if (ret < 0) {
      LOGE("HEVC[%d]-retry(%d:cmd(%d)) poll error '%s'", pInst->vdec_instance_index, retry_cnt, cmd, strerror(errno));
      retry_cnt--;
      continue;
    }else if (ret == 0) {
      LOGE("HEVC[%d]-retry(%d:cmd(%d)) poll timeout: %u'th frames, len %d", pInst->vdec_instance_index, retry_cnt, cmd, pInst->total_frm, pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_iBitstreamDataSize );
      retry_cnt--;
      continue;
    }else if (ret > 0) {
      if (pInst->tcc_event[0].revents & POLLERR) {
        LOGE("HEVC[%d] poll POLLERR", pInst->vdec_instance_index);
        break;
      } else if (pInst->tcc_event[0].revents & POLLIN) {
        success = 1;
        break;
      }
    }
  }
  /* todo */

  switch(cmd)
  {
  case V_DEC_INIT:
  {
    HEVC_INIT_t* init_info = args;

    if(ioctl(pInst->dec_fd, V_DEC_INIT_RESULT, args) < 0){
      LOGE("HEVC[%d] ioctl(0x%x) error[%s]!!", pInst->vdec_instance_index, V_DEC_INIT_RESULT, strerror(errno));
    }
    ret = init_info->result;
  }
  break;

  case V_DEC_SEQ_HEADER:
  {
#if REMOVE_SECURE_COPY
    HEVC_DECODE_t* seq_info = args;
#else
    HEVC_SEQ_HEADER_t* seq_info = args;
#endif
    if(ioctl(pInst->dec_fd, V_DEC_SEQ_HEADER_RESULT, args) < 0){
      LOGE("HEVC[%d] ioctl(0x%x) error[%s]!!", pInst->vdec_instance_index, V_DEC_SEQ_HEADER_RESULT, strerror(errno));
    }
    ret = seq_info->result;
  #ifdef ERROR_TEST
    err_test = 0;
  #endif

  }
  break;

  case V_DEC_DECODE:
  {
    HEVC_DECODE_t* decoded_info = args;

    if(ioctl(pInst->dec_fd, V_DEC_DECODE_RESULT, args) < 0){
      LOGE("HEVC[%d] ioctl(0x%x) error[%s]!!", pInst->vdec_instance_index, V_DEC_DECODE_RESULT, strerror(errno));
    }
    ret = decoded_info->result;
  }
  break;

  case V_DEC_FLUSH_OUTPUT:
  {
    HEVC_DECODE_t* decoded_info = args;

    if(ioctl(pInst->dec_fd, V_DEC_FLUSH_OUTPUT_RESULT, args) < 0){
      LOGE("HEVC[%d] ioctl(0x%x) error[%s]!!", pInst->vdec_instance_index, V_DEC_FLUSH_OUTPUT_RESULT, strerror(errno));
    }
    ret = decoded_info->result;
  }
  break;

  case V_GET_RING_BUFFER_STATUS:
  {
    HEVC_RINGBUF_GETINFO_t* p_param = args;
    if(ioctl(pInst->dec_fd, V_GET_RING_BUFFER_STATUS_RESULT, args) < 0){
      LOGE("HEVC[%d] ioctl(0x%x) error[%s]!!", pInst->vdec_instance_index, V_GET_RING_BUFFER_STATUS_RESULT, strerror(errno));
    }
    ret = p_param->result;
  }
  break;
  case V_FILL_RING_BUFFER_AUTO:
  {
    HEVC_RINGBUF_SETBUF_t* p_param = args;
    if(ioctl(pInst->dec_fd, V_FILL_RING_BUFFER_AUTO_RESULT, args) < 0){
      LOGE("HEVC[%d] ioctl(0x%x) error[%s]!!", pInst->vdec_instance_index, V_FILL_RING_BUFFER_AUTO_RESULT, strerror(errno));
    }
    ret = p_param->result;
  }
  break;
  case V_DEC_UPDATE_RINGBUF_WP:
  {
    HEVC_RINGBUF_SETBUF_PTRONLY_t* p_param = args;
    if(ioctl(pInst->dec_fd, V_DEC_UPDATE_RINGBUF_WP_RESULT, args) < 0){
      LOGE("HEVC[%d] ioctl(0x%x) error[%s]!!", pInst->vdec_instance_index, V_DEC_UPDATE_RINGBUF_WP_RESULT, strerror(errno));
    }
    ret = p_param->result;
  }
  break;
  case V_GET_INITIAL_INFO_FOR_STREAMING_MODE_ONLY:
  {
    HEVC_SEQ_HEADER_t* p_param = args;
    if(ioctl(pInst->dec_fd, V_GET_INITIAL_INFO_FOR_STREAMING_MODE_ONLY_RESULT, args) < 0){
      LOGE("HEVC[%d] ioctl(0x%x) error[%s]!!", pInst->vdec_instance_index, V_GET_INITIAL_INFO_FOR_STREAMING_MODE_ONLY_RESULT, strerror(errno));
    }
    ret = p_param->result;
  }
  break;
  case V_GET_VPU_VERSION:
  {
    HEVC_GET_VERSION_t* p_param = args;
    if(ioctl(pInst->dec_fd, V_GET_VPU_VERSION_RESULT, args) < 0){
      LOGE("HEVC[%d] ioctl(0x%x) error[%s]!!", pInst->vdec_instance_index, V_GET_VPU_VERSION_RESULT, strerror(errno));
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
    if(ioctl(pInst->dec_fd, V_DEC_GENERAL_RESULT, &ret) < 0){
      LOGE("HEVC[%d] ioctl(0x%x) error[%s]!!", pInst->vdec_instance_index, V_DEC_GENERAL_RESULT, strerror(errno));
    }
    break;
  }

  if((ret&0xf000) != 0x0000){ //If there is an invalid return, we skip it because this return means that vpu didn't process current command yet.
    all_retry_cnt--;
    if( all_retry_cnt > 0)
    {
      retry_cnt = POLL_RETRY_COUNT;
      goto Retry;
    }
    else
    {
      LOGE("abnormal exception!!");
    }
  }

#ifdef ERROR_TEST
  if (err_test++ == 1000){
    ret = 0xf000;
  }
#endif

  if(!success
  || ((ret&0xf000) != 0x0000) /* vpu can not start or finish its processing with unknown reason!! */
  )
  {
    LOGE("HEVC[%d] command(0x%x) didn't work properly. maybe hangup(no return(0x%x))!!", pInst->vdec_instance_index, cmd, ret);

    if(ret != RETCODE_CODEC_EXIT && ret != RETCODE_MULTI_CODEC_EXIT_TIMEOUT){
//    ioctl(pInst->mgr_fd, VPU_HW_RESET, (void*)NULL);
    }

    return RETCODE_CODEC_EXIT;
  }

  return ret;
}

static void print_hevc_initial_info( hevc_dec_initial_info_t* pInitialInfo, _vdec_ *pVdec)
{
  uint32_t fRateInfoRes = pInitialInfo->m_uiFrameRateRes;
  uint32_t fRateInfoDiv = pInitialInfo->m_uiFrameRateDiv;
  _vdec_ * pInst = pVdec;
  int userDataEnable = 0;
  int profile = 0;
  int level =0;
  int tier = 0;

  DSTATUS("[HEVC-%d] -------------------INITIAL INFO-------------------", pInst->vdec_instance_index);

  profile = pInitialInfo->m_iProfile;
  level = pInitialInfo->m_iLevel;
  tier = pInitialInfo->m_iTier;

  DSTATUS("[HEVC-%d] Dec InitialInfo => profile: %d level: %d tier: %d", pInst->vdec_instance_index, profile, level, tier);

  DSTATUS("[HEVC-%d] Aspect Ratio [%1d]", pInst->vdec_instance_index, pInitialInfo->m_iAspectRateInfo);

  DSTATUS("[HEVC-%d] Dec InitialInfo => minframeBuffercount: %u m_iPicWidth: %u m_iPicHeight: %u ", pInst->vdec_instance_index,
    pInitialInfo->m_iMinFrameBufferCount, pInitialInfo->m_iPicWidth, pInitialInfo->m_iPicHeight);

  DSTATUS("[HEVC-%d] Crop Info %d - %d - %d, %d - %d - %d ", pInst->vdec_instance_index,
    pInitialInfo->m_iPicWidth, pInitialInfo->m_PicCrop.m_iCropLeft, pInitialInfo->m_PicCrop.m_iCropRight,
    pInitialInfo->m_iPicHeight, pInitialInfo->m_PicCrop.m_iCropTop, pInitialInfo->m_PicCrop.m_iCropBottom);

  DSTATUS("[HEVC-%d]  frRes: %u frDiv: %u", pInst->vdec_instance_index, fRateInfoRes, fRateInfoDiv);
  DSTATUS("[HEVC-%d]  Source Format %d bit,  Output Format %d", pInst->vdec_instance_index,
    pInitialInfo->m_iSourceFormat == 0 ? 8 : 10,  pInitialInfo->m_iFrameBufferFormat == 0 ? 8 : 10 );
  DSTATUS("[HEVC-%d] ---------------------------------------------------", pInst->vdec_instance_index);

}

int
hevc_dec_ready( hevc_dec_init_t* psHDecInit, _vdec_ *pVdec)
{
  _vdec_ * pInst = pVdec;

  //------------------------------------------------------------
  //! [x] bitstream buffer for each HEVC decoder
  //------------------------------------------------------------
#ifdef USE_PREV_STREAMBUFF_DECODING
  pInst->gsCurrStreamBuffer_index = 0;
#endif
  if(psHDecInit->m_iBitstreamBufSize > WAVE4_STREAM_BUF_SIZE)
  pInst->gsBitstreamBufSize = ALIGNED_BUFF( psHDecInit->m_iBitstreamBufSize, 64*1024 );
  else
  pInst->gsBitstreamBufSize = WAVE4_STREAM_BUF_SIZE;
  pInst->gsBitstreamBufSize = ALIGNED_BUFF( pInst->gsBitstreamBufSize, ALIGN_LEN );

  {
    pInst->gsBitstreamBufAddr[PA] = (codec_addr_t)cdk_sys_malloc_physical_addr(&pInst->gsBitstreamBufAddr[K_VA], pInst->gsBitstreamBufSize, BUFFER_STREAM, pInst );

    if( pInst->gsBitstreamBufAddr[PA] == 0 )
    {
      DPRINTF( "[HEVC-%d] bitstream_buf_addr[PA] malloc() failed ", pInst->vdec_instance_index);
      return -(VPU_NOT_ENOUGH_MEM);
    }
    DSTATUS( "[HEVC-%d] bitstream_buf_addr[PA] = 0x%lx, 0x%x ", pInst->vdec_instance_index, (codec_addr_t)pInst->gsBitstreamBufAddr[PA], pInst->gsBitstreamBufSize );
    pInst->gsBitstreamBufAddr[VA] = (codec_addr_t)cdk_sys_malloc_virtual_addr( (uint32_t)pInst->gsBitstreamBufAddr[PA], pInst->gsBitstreamBufSize, pInst );
    if( pInst->gsBitstreamBufAddr[VA] == 0 )
    {
      DPRINTF( "[HEVC-%d] bitstream_buf_addr[VA] malloc() failed ", pInst->vdec_instance_index);
      return -(VPU_NOT_ENOUGH_MEM);
    }
    if( (pInst->extFunction & (EXT_FUNC_MEM_PROTECTION_WITH_INSTREAM | EXT_FUNC_MEM_PROTECTION_ONLY)) == 0x0 )
    {
      memset( (unsigned long*)pInst->gsBitstreamBufAddr[VA], 0x00 , pInst->gsBitstreamBufSize);
    }
    DSTATUS("[HEVC-%d] bitstream_buf_addr[VA] = 0x%lx, 0x%x ", pInst->vdec_instance_index, (codec_addr_t)pInst->gsBitstreamBufAddr[VA], pInst->gsBitstreamBufSize );
  }

  psHDecInit->m_BitstreamBufAddr[PA]  = pInst->gsBitstreamBufAddr[PA];
  psHDecInit->m_BitstreamBufAddr[VA]  = pInst->gsBitstreamBufAddr[K_VA];
  psHDecInit->m_iBitstreamBufSize   = pInst->gsBitstreamBufSize;

  if(pInst->gsHevcDecInit_Info.gsHevcDecInit.m_iFilePlayEnable == 0)
  {
#if 0 // intermediate buffer is not used.
    pInst->gsIntermediateBufSize = WAVE4_STREAM_BUF_SIZE;
    pInst->gsIntermediateBufSize = ALIGNED_BUFF( pInst->gsIntermediateBufSize, ALIGN_LEN );
    pInst->gsIntermediateBufAddr[PA] = (codec_addr_t)cdk_sys_malloc_physical_addr(&pInst->gsIntermediateBufAddr[K_VA], pInst->gsIntermediateBufSize, BUFFER_STREAM, pInst );
  
    if( pInst->gsIntermediateBufAddr[PA] == 0 )
    {
      DPRINTF( "[HEVC-%d] gsIntermediateBufAddr[PA] malloc() failed ", pInst->vdec_instance_index);
      return -(VPU_NOT_ENOUGH_MEM);
    }
    DSTATUS( "[HEVC-%d] bitstream_buf_addr[PA] = 0x%lx, 0x%x ", pInst->vdec_instance_index, (codec_addr_t)pInst->gsIntermediateBufAddr[PA], pInst->gsIntermediateBufSize );
    pInst->gsIntermediateBufAddr[VA] = (codec_addr_t)cdk_sys_malloc_virtual_addr( pInst->gsIntermediateBufAddr[PA], pInst->gsIntermediateBufSize, pInst );
    if( pInst->gsIntermediateBufAddr[VA] == 0 )
    {
      DPRINTF( "[HEVC-%d] gsIntermediateBufAddr[VA] malloc() failed ", pInst->vdec_instance_index);
      return -(VPU_NOT_ENOUGH_MEM);
    }
    memset( (unsigned long*)pInst->gsIntermediateBufAddr[VA], 0x00 , pInst->gsIntermediateBufSize);
    DSTATUS("[HEVC-%d] gsIntermediateBufAddr[VA] = 0x%lx, 0x%x ", pInst->vdec_instance_index, (codec_addr_t)pInst->gsIntermediateBufAddr[VA], pInst->gsIntermediateBufSize );
#endif
  }
  else
  {
    pInst->gsIntermediateBufSize = 0;
    pInst->gsIntermediateBufAddr[PA] = 0;
    pInst->gsIntermediateBufAddr[VA] = 0;
    pInst->gsIntermediateBufAddr[K_VA] = 0;
  }

  /* Set the maximum size of input bitstream. */
//  gsMaxBitstreamSize = MAX_BITSTREAM_SIZE;
//  gsMaxBitstreamSize = ALIGNED_BUFF(gsMaxBitstreamSize, (4 * 1024));
//  if (gsMaxBitstreamSize > gsBitstreamBufSize)
//  {
  pInst->gsMaxBitstreamSize = pInst->gsBitstreamBufSize;
//  }

  {
    //------------------------------------------------------------
    //! [x] user data buffer for each HEVC decoder
    //------------------------------------------------------------
    if(pInst->gsHevcDecInit_Info.gsHevcDecInit.m_bEnableUserData)
    {
      pInst->gsUserdataBufSize = WAVE4_USERDATA_BUF_SIZE;
      pInst->gsUserdataBufSize = ALIGNED_BUFF( pInst->gsUserdataBufSize, ALIGN_LEN );
      pInst->gsUserdataBufAddr[PA] = (codec_addr_t)cdk_sys_malloc_physical_addr(&pInst->gsUserdataBufAddr[K_VA], pInst->gsUserdataBufSize, BUFFER_USERDATA, pInst );
      if( pInst->gsUserdataBufAddr[PA] == 0 )
      {
        DPRINTF( "[HEVC-%d:Err%d] pInst->gsUserdataBufAddr physical alloc failed ", pInst->vdec_instance_index, -1 );
        return -(VPU_NOT_ENOUGH_MEM);
      }
      DSTATUS( "[HEVC-%d] pInst->gsUserdataBufAddr[PA] = 0x%lx, 0x%x ", pInst->vdec_instance_index, (codec_addr_t)pInst->gsUserdataBufAddr[PA], pInst->gsUserdataBufSize );
      pInst->gsUserdataBufAddr[VA] = (codec_addr_t)cdk_sys_malloc_virtual_addr( pInst->gsUserdataBufAddr[PA], pInst->gsUserdataBufSize, pInst );
      if( pInst->gsUserdataBufAddr[VA] == 0 )
      {
        DPRINTF( "[HEVC-%d:Err%d] pInst->gsUserdataBufAddr virtual alloc failed ", pInst->vdec_instance_index, -1 );
        return -(VPU_NOT_ENOUGH_MEM);
      }
      //memset( (void*)pInst->gsUserdataBufAddr[VA], 0 , gsUserdataBufSize);

      psHDecInit->m_UserDataAddr[PA] = pInst->gsUserdataBufAddr[PA];
      psHDecInit->m_UserDataAddr[VA] = pInst->gsUserdataBufAddr[K_VA];
      psHDecInit->m_iUserDataBufferSize = pInst->gsUserdataBufSize;
      psHDecInit->m_bEnableUserData = pInst->gsHevcDecInit_Info.gsHevcDecInit.m_bEnableUserData;
      
      DSTATUS("[HEVC-%d] pInst->gsUserdataBufAddr[VA] = 0x%lx, 0x%x ", pInst->vdec_instance_index, (codec_addr_t)pInst->gsUserdataBufAddr[VA], pInst->gsUserdataBufSize );
    }

    //------------------------------------------------------------
    // [x] code buffer, work buffer and parameter buffer for HEVC
    //------------------------------------------------------------
    pInst->gsBitWorkBufSize = WAVE4_WORK_CODE_BUF_SIZE;
    pInst->gsBitWorkBufSize = ALIGNED_BUFF(pInst->gsBitWorkBufSize, ALIGN_LEN);
    pInst->gsBitWorkBufAddr[PA] = (codec_addr_t)cdk_sys_malloc_physical_addr(&pInst->gsBitWorkBufAddr[K_VA], pInst->gsBitWorkBufSize, BUFFER_WORK, pInst );
    if( pInst->gsBitWorkBufAddr[PA] == 0 )
    {
      DPRINTF( "[HEVC-%d] bit_work_buf_addr[PA] malloc() failed ", pInst->vdec_instance_index);
      return -(VPU_NOT_ENOUGH_MEM);
    }
    DSTATUS("[HEVC-%d] bit_work_buf_addr[PA] = 0x%lx, 0x%x ", pInst->vdec_instance_index, (codec_addr_t)pInst->gsBitWorkBufAddr[PA], pInst->gsBitWorkBufSize );

    pInst->gsBitWorkBufAddr[VA] = (codec_addr_t)cdk_sys_malloc_virtual_addr( pInst->gsBitWorkBufAddr[PA], pInst->gsBitWorkBufSize, pInst );
    if( pInst->gsBitWorkBufAddr[VA] == 0 )
    {
      DPRINTF( "[HEVC-%d] bit_work_buf_addr[VA] malloc() failed ", pInst->vdec_instance_index);
      return -(VPU_NOT_ENOUGH_MEM);
    }
    DSTATUS("[HEVC-%d] bit_work_buf_addr[VA] = 0x%lx, 0x%x ", pInst->vdec_instance_index, (codec_addr_t)pInst->gsBitWorkBufAddr[VA], pInst->gsBitWorkBufSize );
  }

  psHDecInit->m_BitWorkAddr[PA] = pInst->gsBitWorkBufAddr[PA];
  psHDecInit->m_BitWorkAddr[VA] = pInst->gsBitWorkBufAddr[K_VA];
  LOGI("[%s] PA 0x%x VA 0x%x 0x%x\n", __func__, pVdec->gsBitWorkBufAddr[PA], pVdec->gsBitWorkBufAddr[VA], pVdec->gsBitWorkBufAddr[K_VA]);

  if( psHDecInit->m_bCbCrInterleaveMode == 0 ){
    DSTATUS("[HEVC-%d] CbCrInterleaveMode OFF", pInst->vdec_instance_index);
  }
  else{
    DSTATUS("[HEVC-%d] CbCrInterleaveMode ON", pInst->vdec_instance_index);
  }

  return 0;
}

int
hevc_dec_seq_header( int iSize, int iIsThumbnail, _vdec_ *pVdec )
{
  int ret = 0;
  _vdec_ * pInst = pVdec;
#if REMOVE_SECURE_COPY
  LOGI("[HEVC-%d]vpu_dec_seq_header in :: size(%d), JpegOnly(%d), format(%d)", pInst->vdec_instance_index,
      iSize ,pInst->gsUserInfo.m_bStillJpeg, pInst->gsHevcDecInit_Info.gsHevcDecInit.m_iBitstreamFormat);
  ret = hevc_cmd_process(V_DEC_SEQ_HEADER, &pInst->gsHevcDecInOut_Info, pInst);
  if( ret != RETCODE_SUCCESS )
  {
    DPRINTF( "[HEVC-%d] VPU_DEC_SEQ_HEADER failed Error code is 0x%x. ErrorReason is %d", \
    pInst->vdec_instance_index, ret, pInst->gsHevcDecInOut_Info.gsHevcDecInitialInfo.m_iReportErrorReason);

    if(ret == RETCODE_CODEC_SPECOUT){
      DPRINTF("[HEVC-%d] NOT SUPPORTED CODEC. VPU SPEC OUT!!", pInst->vdec_instance_index);   // This is a very common error. Notice the detailed reason to users.
    }
    return -ret;
  }

  print_hevc_initial_info( &pInst->gsHevcDecInOut_Info.gsHevcDecInitialInfo, pInst );

  if( pInst->gsHevcDecInit_Info.gsHevcDecInit.m_iBitstreamFormat == STD_HEVC ){
    pInst->mRealPicWidth = pInst->gsHevcDecInOut_Info.gsHevcDecInitialInfo.m_iPicWidth - pInst->gsHevcDecInOut_Info.gsHevcDecInitialInfo.m_PicCrop.m_iCropLeft - pInst->gsHevcDecInOut_Info.gsHevcDecInitialInfo.m_PicCrop.m_iCropRight;
    pInst->mRealPicHeight = pInst->gsHevcDecInOut_Info.gsHevcDecInitialInfo.m_iPicHeight - pInst->gsHevcDecInOut_Info.gsHevcDecInitialInfo.m_PicCrop.m_iCropBottom - pInst->gsHevcDecInOut_Info.gsHevcDecInitialInfo.m_PicCrop.m_iCropTop;
  }
  else{
    pInst->mRealPicWidth = pInst->gsHevcDecInOut_Info.gsHevcDecInitialInfo.m_iPicWidth;
    pInst->mRealPicHeight = pInst->gsHevcDecInOut_Info.gsHevcDecInitialInfo.m_iPicHeight;
  }
  DSTATUS( "[HEVC-%d] 1 Real-Resolution: %d x %d", pVdec->vdec_instance_index, pInst->mRealPicWidth, pInst->mRealPicHeight);

#ifdef SET_FRAMEBUFFER_INTO_MAX
  if( (pInst->extFunction & EXT_FUNC_MAX_FRAMEBUFFER) != 0x0 )
  {
    pInst->gsHevcDecInOut_Info.gsHevcDecInitialInfo.m_iPicWidth = AVAILABLE_HEVC_MAX_WIDTH;
    pInst->gsHevcDecInOut_Info.gsHevcDecInitialInfo.m_iPicHeight = AVAILABLE_HEVC_MAX_HEIGHT;
    
    LOGI("[HEVC-%d]Set seq framebuffer into (%d x %d) <- (%d x %d)", pInst->vdec_instance_index,
     pInst->gsHevcDecInOut_Info.gsHevcDecInitialInfo.m_iPicWidth,
     pInst->gsHevcDecInOut_Info.gsHevcDecInitialInfo.m_iPicHeight,
     pInst->mRealPicWidth, pInst->mRealPicHeight);
  }
  else
#endif
  {
    pInst->gsHevcDecInOut_Info.gsHevcDecInitialInfo.m_iPicWidth = ((pInst->mRealPicWidth+31)>>5)<<5;
    pInst->gsHevcDecInOut_Info.gsHevcDecInitialInfo.m_iPicHeight = pInst->mRealPicHeight;
  }

  set_dec_common_info(pInst->gsHevcDecInOut_Info.gsHevcDecInitialInfo.m_iPicWidth,
      pInst->gsHevcDecInOut_Info.gsHevcDecInitialInfo.m_iPicHeight,
      (unsigned long*)&pInst->gsHevcDecInOut_Info.gsHevcDecInitialInfo.m_PicCrop, 0,
      0, pInst );

  // bit Depth
  #if defined(TCC_803X_INCLUDE)
  pInst->gsCommDecInfo.m_iFrameBufferFormat  = pInst->gsHevcDecInOut_Info.gsHevcDecInitialInfo.m_iFrameBufferFormat;
  pInst->gsCommDecInfo.m_iSourceFormat= pInst->gsHevcDecInOut_Info.gsHevcDecInitialInfo.m_iSourceFormat;
  pInst->gsHevcUserDataInfo.bit_depth = pInst->gsCommDecInfo.m_iSourceFormat;
  #else
  pInst->gsCommDecInfo.m_iFrameBufferFormat  = pInst->gsHevcDecInOut_Info.gsHevcDecInitialInfo.m_iFrameBufferFormat;
  pInst->gsCommDecInfo.m_iSourceFormat= 0;
  #endif

  pInst->gsSliceBufSize = 0;
  pInst->gsSliceBufAddr = 0;
  pInst->gsMbSaveSize = 0;
  pInst->gsMbSaveAddr = 0;

  //------------------------------------------------------------
  // [x] frame buffer for each HEVC decoder
  //------------------------------------------------------------
  pInst->gsHevcDecBuffer_Info.gsHevcDecBuffer.m_iFrameBufferCount = pInst->gsHevcDecInOut_Info.gsHevcDecInitialInfo.m_iMinFrameBufferCount;
  LOGD( "[HEVC-%d] FrameBufDelay %d, MinFrameBufferCount %d", pInst->vdec_instance_index, pInst->gsHevcDecInOut_Info.gsHevcDecInitialInfo.m_iFrameBufDelay , pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo.m_iMinFrameBufferCount );
  {
    int max_count;
    
    if(!iIsThumbnail){
#ifdef USE_MAP_CONVERTER
      if( (pInst->extFunction & EXT_FUNC_USE_MAP_CONVERTER) != 0x0 )
      {
        pInst->gsHevcDecBuffer_Info.gsHevcDecBuffer.m_iFrameBufferCount = pInst->gsHevcDecInOut_Info.gsHevcDecInitialInfo.m_iMinFrameBufferCount + pInst->gsAdditionalFrameCount;
      }
      else
#endif
      {
        pInst->gsHevcDecBuffer_Info.gsHevcDecBuffer.m_iFrameBufferCount = pInst->gsHevcDecInOut_Info.gsHevcDecInitialInfo.m_iMinFrameBufferCount + pInst->gsAdditionalFrameCount;
      }
    }
    max_count = cdk_sys_remain_memory_size(pInst) / pInst->gsHevcDecInOut_Info.gsHevcDecInitialInfo.m_iMinFrameBufferSize;

    if(pInst->gsHevcDecBuffer_Info.gsHevcDecBuffer.m_iFrameBufferCount > max_count){
      pInst->gsHevcDecBuffer_Info.gsHevcDecBuffer.m_iFrameBufferCount = max_count;
    }

#ifdef USE_MAP_CONVERTER
    if( (pInst->extFunction & EXT_FUNC_USE_MAP_CONVERTER) != 0x0 )
    {
      if(pInst->gsHevcDecBuffer_Info.gsHevcDecBuffer.m_iFrameBufferCount > MAX_FRAME_BUFFER_COUNT)
      pInst->gsHevcDecBuffer_Info.gsHevcDecBuffer.m_iFrameBufferCount = MAX_FRAME_BUFFER_COUNT;
    }
    else
#endif
    {
      if(pInst->gsHevcDecBuffer_Info.gsHevcDecBuffer.m_iFrameBufferCount > MAX_FRAME_BUFFER_COUNT)
      pInst->gsHevcDecBuffer_Info.gsHevcDecBuffer.m_iFrameBufferCount = MAX_FRAME_BUFFER_COUNT;
    }

    if(iIsThumbnail)
    {
      if(pInst->gsHevcDecBuffer_Info.gsHevcDecBuffer.m_iFrameBufferCount < (pInst->gsHevcDecInOut_Info.gsHevcDecInitialInfo.m_iMinFrameBufferCount))
      {
        LOGE( "[HEVC-%d] Not enough memory for VPU frame buffer, Available[%d], Min[%d], Need[%d], min_size = %d", pInst->vdec_instance_index, max_count, pInst->gsHevcDecInOut_Info.gsHevcDecInitialInfo.m_iMinFrameBufferCount, pInst->gsHevcDecBuffer_Info.gsHevcDecBuffer.m_iFrameBufferCount, pInst->gsHevcDecInOut_Info.gsHevcDecInitialInfo.m_iMinFrameBufferSize);
        return -(VPU_NOT_ENOUGH_MEM);
      }
    }
    else
    {
      if(pInst->gsHevcDecBuffer_Info.gsHevcDecBuffer.m_iFrameBufferCount < (pInst->gsHevcDecInOut_Info.gsHevcDecInitialInfo.m_iMinFrameBufferCount+pInst->gsAdditionalFrameCount))
      {
        LOGE( "[HEVC-%d] Not enough memory for VPU frame buffer, Available[%d], Min[%d], Need[%d], min_size = %d", pInst->vdec_instance_index, max_count, pInst->gsHevcDecInOut_Info.gsHevcDecInitialInfo.m_iMinFrameBufferCount, pInst->gsHevcDecInOut_Info.gsHevcDecInitialInfo.m_iMinFrameBufferCount + pInst->gsAdditionalFrameCount, pInst->gsHevcDecInOut_Info.gsHevcDecInitialInfo.m_iMinFrameBufferSize);
        return -(VPU_NOT_ENOUGH_MEM);
      }
    }

    pInst->gsTotalFrameCount = pInst->gsHevcDecBuffer_Info.gsHevcDecBuffer.m_iFrameBufferCount;
  }

  pInst->gsFrameBufSize = pInst->gsHevcDecBuffer_Info.gsHevcDecBuffer.m_iFrameBufferCount * pInst->gsHevcDecInOut_Info.gsHevcDecInitialInfo.m_iMinFrameBufferSize;
  LOGD( "[HEVC-%d] FrameBufferCount %d [min %d], min_size = %d ", pInst->vdec_instance_index,
         pInst->gsHevcDecBuffer_Info.gsHevcDecBuffer.m_iFrameBufferCount,
         pInst->gsHevcDecInOut_Info.gsHevcDecInitialInfo.m_iMinFrameBufferCount,
         pInst->gsHevcDecInOut_Info.gsHevcDecInitialInfo.m_iMinFrameBufferSize);

  pInst->gsFrameBufSize = ALIGNED_BUFF( pInst->gsFrameBufSize, ALIGN_LEN );
  pInst->gsFrameBufAddr[PA] = (codec_addr_t)cdk_sys_malloc_physical_addr(&pInst->gsFrameBufAddr[K_VA], pInst->gsFrameBufSize, BUFFER_FRAMEBUFFER, pInst );
  if( pInst->gsFrameBufAddr[PA] == 0 )
  {
    DPRINTF( "[HEVC-%d] frame_buf_addr[PA] malloc() failed ", pInst->vdec_instance_index);
    return -(VPU_NOT_ENOUGH_MEM);
  }

  DSTATUS( "[HEVC-%d] MinFrameBufferSize %d bytes ", pInst->vdec_instance_index, pInst->gsHevcDecInOut_Info.gsHevcDecInitialInfo.m_iMinFrameBufferSize );
  DSTATUS( "[HEVC-%d] frame_buf_addr[PA] = 0x%lx, 0x%x , index = %d ", pInst->vdec_instance_index,
           (codec_addr_t)pInst->gsFrameBufAddr[PA], pInst->gsFrameBufSize, pInst->vdec_instance_index );
  pInst->gsFrameBufAddr[VA] = (codec_addr_t)cdk_sys_malloc_virtual_addr( pInst->gsFrameBufAddr[PA], pInst->gsFrameBufSize, pInst );
  if( pInst->gsFrameBufAddr[VA] == 0 )
  {
    DPRINTF( "[HEVC-%d] frame_buf_addr[VA] malloc() failed ", pInst->vdec_instance_index);
    return -(VPU_NOT_ENOUGH_MEM);
  }
  DSTATUS("[HEVC-%d] frame_buf_addr[VA] = 0x%lx, frame_buf_addr[K_VA] = 0x%x ", pInst->vdec_instance_index,
           (codec_addr_t)pInst->gsFrameBufAddr[VA], pInst->gsFrameBufAddr[K_VA] );
  pInst->gsHevcDecBuffer_Info.gsHevcDecBuffer.m_FrameBufferStartAddr[PA] = pInst->gsFrameBufAddr[PA];
  pInst->gsHevcDecBuffer_Info.gsHevcDecBuffer.m_FrameBufferStartAddr[VA] = pInst->gsFrameBufAddr[K_VA];

  ret = hevc_cmd_process(V_DEC_REG_FRAME_BUFFER, &pInst->gsHevcDecBuffer_Info, pInst);

  if( ret != RETCODE_SUCCESS )
  {
    DPRINTF( "[HEVC-%d] DEC_REG_FRAME_BUFFER failed Error code is 0x%x ", pInst->vdec_instance_index, ret );
    return -ret;
  }
#else
  LOGI("[HEVC-%d]vpu_dec_seq_header in :: size(%d), JpegOnly(%d), format(%d)", pInst->vdec_instance_index, iSize ,pInst->gsUserInfo.m_bStillJpeg, pInst->gsHevcDecInit_Info.gsHevcDecInit.m_iBitstreamFormat);
  pInst->gsHevcDecSeqHeader_Info.stream_size = iSize;
  ret = hevc_cmd_process(V_DEC_SEQ_HEADER, &pInst->gsHevcDecSeqHeader_Info, pInst);
  if( ret != RETCODE_SUCCESS )
  {
    DPRINTF( "[HEVC-%d] VPU_DEC_SEQ_HEADER failed Error code is 0x%x. ErrorReason is %d", pInst->vdec_instance_index,
             ret, pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo.m_iReportErrorReason);
    if(ret == RETCODE_CODEC_SPECOUT){
      DPRINTF("[HEVC-%d] NOT SUPPORTED CODEC. VPU SPEC OUT!!", pInst->vdec_instance_index);   // This is a very common error. Notice the detailed reason to users.
    }
    return -ret;
  }

  print_hevc_initial_info( &pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo, pInst );

  if( pInst->gsHevcDecInit_Info.gsHevcDecInit.m_iBitstreamFormat == STD_HEVC ){
    pInst->mRealPicWidth = pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo.m_iPicWidth - pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo.m_PicCrop.m_iCropLeft - pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo.m_PicCrop.m_iCropRight;
    pInst->mRealPicHeight = pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo.m_iPicHeight - pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo.m_PicCrop.m_iCropBottom - pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo.m_PicCrop.m_iCropTop;
  }
  else{
    pInst->mRealPicWidth = pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo.m_iPicWidth;
    pInst->mRealPicHeight = pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo.m_iPicHeight;
  }
  DSTATUS( "[HEVC-%d] 1 Real-Resolution: %d x %d", pVdec->vdec_instance_index, pInst->mRealPicWidth, pInst->mRealPicHeight);

#ifdef SET_FRAMEBUFFER_INTO_MAX
  if( (pInst->extFunction & EXT_FUNC_MAX_FRAMEBUFFER) != 0x0 )
  {
    pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo.m_iPicWidth = AVAILABLE_HEVC_MAX_WIDTH;
    pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo.m_iPicHeight = AVAILABLE_HEVC_MAX_HEIGHT;
    
    LOGI("[HEVC-%d]Set seq framebuffer into (%d x %d) <- (%d x %d)", pInst->vdec_instance_index,
      pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo.m_iPicWidth,
      pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo.m_iPicHeight,
      pInst->mRealPicWidth, pInst->mRealPicHeight);
  }
  else
#endif
  {
    pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo.m_iPicWidth = ((pInst->mRealPicWidth+31)>>5)<<5;
    pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo.m_iPicHeight = pInst->mRealPicHeight;
  }

  set_dec_common_info(pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo.m_iPicWidth, pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo.m_iPicHeight,
      (unsigned long*)&pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo.m_PicCrop, 0,
      0, pInst );

  // bit Depth
  #if defined(TCC_803X_INCLUDE)
  pInst->gsCommDecInfo.m_iFrameBufferFormat  = pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo.m_iFrameBufferFormat;
  pInst->gsCommDecInfo.m_iSourceFormat= pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo.m_iSourceFormat;
  pInst->gsHevcUserDataInfo.bit_depth = pInst->gsCommDecInfo.m_iSourceFormat;
  #else
  pInst->gsCommDecInfo.m_iFrameBufferFormat  = pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo.m_iFrameBufferFormat;
  pInst->gsCommDecInfo.m_iSourceFormat= 0;
  #endif

  pInst->gsSliceBufSize = 0;
  pInst->gsSliceBufAddr = 0;
  pInst->gsMbSaveSize = 0;
  pInst->gsMbSaveAddr = 0;

  //------------------------------------------------------------
  // [x] frame buffer for each HEVC decoder
  //------------------------------------------------------------
  pInst->gsHevcDecBuffer_Info.gsHevcDecBuffer.m_iFrameBufferCount = pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo.m_iMinFrameBufferCount;
  LOGD( "[HEVC-%d] FrameBufDelay %d, MinFrameBufferCount %d", pInst->vdec_instance_index,
           pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo.m_iFrameBufDelay , pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo.m_iMinFrameBufferCount );
  {
    int max_count;
  
    if(!iIsThumbnail){
#ifdef USE_MAP_CONVERTER
      if( (pInst->extFunction & EXT_FUNC_USE_MAP_CONVERTER) != 0x0 )
      {
        pInst->gsHevcDecBuffer_Info.gsHevcDecBuffer.m_iFrameBufferCount = pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo.m_iMinFrameBufferCount + pInst->gsAdditionalFrameCount;
      }
      else
#endif
      {
        pInst->gsHevcDecBuffer_Info.gsHevcDecBuffer.m_iFrameBufferCount = pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo.m_iMinFrameBufferCount + pInst->gsAdditionalFrameCount;
      }
    }
    max_count = cdk_sys_remain_memory_size(pInst) / pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo.m_iMinFrameBufferSize;
  
    if(pInst->gsHevcDecBuffer_Info.gsHevcDecBuffer.m_iFrameBufferCount > max_count){
      pInst->gsHevcDecBuffer_Info.gsHevcDecBuffer.m_iFrameBufferCount = max_count;
    }

#ifdef USE_MAP_CONVERTER
    if( (pInst->extFunction & EXT_FUNC_USE_MAP_CONVERTER) != 0x0 )
    {
      if(pInst->gsHevcDecBuffer_Info.gsHevcDecBuffer.m_iFrameBufferCount > MAX_FRAME_BUFFER_COUNT)
      pInst->gsHevcDecBuffer_Info.gsHevcDecBuffer.m_iFrameBufferCount = MAX_FRAME_BUFFER_COUNT;
    }
    else
#endif
    {
      if(pInst->gsHevcDecBuffer_Info.gsHevcDecBuffer.m_iFrameBufferCount > MAX_FRAME_BUFFER_COUNT)
      pInst->gsHevcDecBuffer_Info.gsHevcDecBuffer.m_iFrameBufferCount = MAX_FRAME_BUFFER_COUNT;
    }

    if(iIsThumbnail)
    {
      if(pInst->gsHevcDecBuffer_Info.gsHevcDecBuffer.m_iFrameBufferCount < (pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo.m_iMinFrameBufferCount))
      {
        tcc_printf( "[HEVC-%d][line:%d] Not enough memory for VPU frame buffer, Available[%d], Min[%d], Need[%d], min_size = %d", pInst->vdec_instance_index, __LINE__, max_count, pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo.m_iMinFrameBufferCount, pInst->gsHevcDecBuffer_Info.gsHevcDecBuffer.m_iFrameBufferCount, pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo.m_iMinFrameBufferSize);
        return -(VPU_NOT_ENOUGH_MEM);
      }
    }
    else
    {
      if(pInst->gsHevcDecBuffer_Info.gsHevcDecBuffer.m_iFrameBufferCount < (pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo.m_iMinFrameBufferCount/*+pInst->gsAdditionalFrameCount*/))
      {
        tcc_printf( "[HEVC-%d][line:%d] Not enough memory for VPU frame buffer, Available[%d], Min[%d], Need[%d], min_size = %d", pInst->vdec_instance_index, __LINE__, max_count, pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo.m_iMinFrameBufferCount, pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo.m_iMinFrameBufferCount + pInst->gsAdditionalFrameCount, pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo.m_iMinFrameBufferSize);
        return -(VPU_NOT_ENOUGH_MEM);
      }
    }

    pInst->gsTotalFrameCount = pInst->gsHevcDecBuffer_Info.gsHevcDecBuffer.m_iFrameBufferCount;
  }

  pInst->gsFrameBufSize = pInst->gsHevcDecBuffer_Info.gsHevcDecBuffer.m_iFrameBufferCount * pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo.m_iMinFrameBufferSize;
  tcc_printf( "[HEVC-%d] FrameBufferCount %d [min %d], min_size = %d ", pInst->vdec_instance_index,
             pInst->gsHevcDecBuffer_Info.gsHevcDecBuffer.m_iFrameBufferCount,
             pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo.m_iMinFrameBufferCount,
             pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo.m_iMinFrameBufferSize);

  pInst->gsFrameBufSize = ALIGNED_BUFF( pInst->gsFrameBufSize, ALIGN_LEN );
  pInst->gsFrameBufAddr[PA] = (codec_addr_t)cdk_sys_malloc_physical_addr(&pInst->gsFrameBufAddr[K_VA], pInst->gsFrameBufSize, BUFFER_FRAMEBUFFER, pInst );
  if( pInst->gsFrameBufAddr[PA] == 0 )
  {
    DPRINTF( "[HEVC-%d] frame_buf_addr[PA] malloc() failed ", pInst->vdec_instance_index);
    return -(VPU_NOT_ENOUGH_MEM);
  }

  DSTATUS( "[HEVC-%d] MinFrameBufferSize %d bytes ", pInst->vdec_instance_index, pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo.m_iMinFrameBufferSize );
  DSTATUS( "[HEVC-%d] frame_buf_addr[PA] = 0x%lx, 0x%x , index = %d ", pInst->vdec_instance_index,
             (codec_addr_t)pInst->gsFrameBufAddr[PA], pInst->gsFrameBufSize, pInst->vdec_instance_index );
  pInst->gsFrameBufAddr[VA] = (codec_addr_t)cdk_sys_malloc_virtual_addr( pInst->gsFrameBufAddr[PA], pInst->gsFrameBufSize, pInst );
  if( pInst->gsFrameBufAddr[VA] == 0 )
  {
    DPRINTF( "[HEVC-%d] frame_buf_addr[VA] malloc() failed ", pInst->vdec_instance_index);
    return -(VPU_NOT_ENOUGH_MEM);
  }
  DSTATUS("[HEVC-%d] frame_buf_addr[VA] = 0x%lx, frame_buf_addr[K_VA] = 0x%x ", pInst->vdec_instance_index,
         (codec_addr_t)pInst->gsFrameBufAddr[VA], pInst->gsFrameBufAddr[K_VA] );
  pInst->gsHevcDecBuffer_Info.gsHevcDecBuffer.m_FrameBufferStartAddr[PA] = pInst->gsFrameBufAddr[PA];
  pInst->gsHevcDecBuffer_Info.gsHevcDecBuffer.m_FrameBufferStartAddr[VA] = pInst->gsFrameBufAddr[K_VA];

  ret = hevc_cmd_process(V_DEC_REG_FRAME_BUFFER, &pInst->gsHevcDecBuffer_Info, pInst);

  if( ret != RETCODE_SUCCESS )
  {
    DPRINTF( "[HEVC-%d] DEC_REG_FRAME_BUFFER failed Error code is 0x%x ", pInst->vdec_instance_index, ret );
    return -ret;
  }
#endif
  DSTATUS("[HEVC-%d] TCC_VPU_DEC VPU_DEC_REG_FRAME_BUFFER OK!", pInst->vdec_instance_index);
  return ret;
}

static int hevc_set_user_data_info(hevc_dec_UserData_info_t* src, hevc_userdata_output_t *dst, _vdec_*  pInst) {

  if (src->m_VuiParam.transfer_characteristics)
  {
    dst->colour_primaries = src->m_VuiParam.colour_primaries;
    dst->transfer_characteristics = src->m_VuiParam.transfer_characteristics;
    dst->matrix_coefficients = src->m_VuiParam.matrix_coefficients;
  }

  if (src->m_MasteringDisplayColorVolume.max_display_mastering_luminance)
  {
    dst->display_primaries_x[0] = src->m_MasteringDisplayColorVolume.display_primaries_x[0];
    dst->display_primaries_x[1] = src->m_MasteringDisplayColorVolume.display_primaries_x[1];
    dst->display_primaries_x[2] = src->m_MasteringDisplayColorVolume.display_primaries_x[2];
    dst->display_primaries_y[0] = src->m_MasteringDisplayColorVolume.display_primaries_y[0];
    dst->display_primaries_y[1] = src->m_MasteringDisplayColorVolume.display_primaries_y[1];
    dst->display_primaries_y[2] = src->m_MasteringDisplayColorVolume.display_primaries_y[2];
    dst->white_point_x      = src->m_MasteringDisplayColorVolume.white_point_x;
    dst->white_point_y      = src->m_MasteringDisplayColorVolume.white_point_y;
    dst->max_display_mastering_luminance  =
      src->m_MasteringDisplayColorVolume.max_display_mastering_luminance == 0 ? 0 :
        src->m_MasteringDisplayColorVolume.max_display_mastering_luminance/ 10000;
    dst->min_display_mastering_luminance  =
      src->m_MasteringDisplayColorVolume.min_display_mastering_luminance == 0 ? 0 :
        src->m_MasteringDisplayColorVolume.min_display_mastering_luminance/ 10000;
  }

  if (src->m_ContentLightLevelInfo.max_content_light_level)
  {
    dst->max_content_light_level      = src->m_ContentLightLevelInfo.max_content_light_level;
    dst->max_pic_average_light_level    = src->m_ContentLightLevelInfo.max_pic_average_light_level;
  }

  dst->version = 1;
  dst->struct_size = 26;//# of input data
  dst->static_metadata_descriptor = 0;

  if (dst->eotf != 0 && dst->transfer_characteristics != src->m_VuiParam.transfer_characteristics)
  {
    dst->colour_primaries = src->m_VuiParam.colour_primaries;
    dst->transfer_characteristics = src->m_VuiParam.transfer_characteristics;
    dst->matrix_coefficients = src->m_VuiParam.matrix_coefficients;
    dst->eotf = 0;
  }

  pInst->gsHLGFLAG |= (dst->transfer_characteristics == 14) ? 1  : pInst->gsHLGFLAG;
  pInst->gsHLGFLAG |= (src->m_AlternativeTransferCharacteristicsInfo.preferred_transfer_characteristics == 18) ? 2 : pInst->gsHLGFLAG;

  if (dst->eotf == 0)
  {
    if (dst->transfer_characteristics == 16)
    {
      dst->eotf = 2; //HDR10
      pInst->gsHLGFLAG = 0;
    }
    else if ((dst->transfer_characteristics == 18)
        || ((pInst->gsHLGFLAG & 3) == 3))
    {
      dst->eotf = 3;  //HLG
    }
    else
    {
      dst->eotf = 0; //SDR
      pInst->gsHLGFLAG = 0;
    }
  }

#if 0
  LOGI("\x1b[1;33m[%s][%d] ver %d size %d eotf %d src transfer_characteristics %d preferred_transfer_characteristics %d\x1b[0m\n",
      __func__, __LINE__, dst->version, dst->struct_size, dst->eotf, src->m_VuiParam.transfer_characteristics,
      src->m_AlternativeTransferCharacteristicsInfo.preferred_transfer_characteristics);
  LOGI("\x1b[1;33m[%s][%d] colour_primaries %d transfer %d matrix_coefficients %d\x1b[0m\n", __func__, __LINE__, dst->colour_primaries, dst->transfer_characteristics, dst->matrix_coefficients);
#endif
  return dst->eotf == 0 ? -1 : 0;
}

int32_t
vdec_hevc( int32_t iOpCode, unsigned long* pHandle, unsigned long* pParam1, unsigned long* pParam2, unsigned long* pParam3 )
{
  int ret = 0;
  _vdec_ *pInst = (_vdec_ *)pParam3;

  if(!pInst){
    LOGE("vdec_hevc(OP:%d) :: Instance is null!!", iOpCode);
    return -RETCODE_NOT_INITIALIZED;
  }

  if( iOpCode != VDEC_INIT && iOpCode != VDEC_CLOSE && !pInst->vdec_codec_opened) {
    return -RETCODE_NOT_INITIALIZED;
  }

#ifdef DEBUG_TIME_LOG
  clock_t start, end;
  start = clock();
#endif

  if( iOpCode == VDEC_INIT )
  {
  vdec_init_t* p_init_param = (vdec_init_t*)pParam1;

  vdec_user_info_t* p_init_user_param = (vdec_user_info_t*)pParam2;

  pInst->gsUserInfo.bitrate_mbps  = p_init_user_param->bitrate_mbps;
  pInst->gsUserInfo.frame_rate    = p_init_user_param->frame_rate;
  pInst->gsUserInfo.m_bStillJpeg    = p_init_user_param->m_bStillJpeg;
  pInst->gsUserInfo.jpg_ScaleRatio  = p_init_user_param->jpg_ScaleRatio;

  pInst->codec_format = p_init_param->m_iBitstreamFormat;
  if(vpu_env_open(p_init_param->m_iBitstreamFormat, p_init_user_param->bitrate_mbps, p_init_user_param->frame_rate, p_init_param->m_iPicWidth, p_init_param->m_iPicHeight, pInst ) < 0)
    return -VPU_ENV_INIT_ERROR;

  pInst->gsHevcDecInit_Info.gsHevcDecInit.m_iBitstreamFormat  = p_init_param->m_iBitstreamFormat;
  //pInst->gsHevcDecInit_Info.gsHevcDecInit.m_iPicWidth     = p_init_param->m_iPicWidth;
  //pInst->gsHevcDecInit_Info.gsHevcDecInit.m_iPicHeight    = p_init_param->m_iPicHeight;
  pInst->gsHevcDecInit_Info.gsHevcDecInit.m_bEnableUserData   = p_init_param->m_bEnableUserData;
//  pInst->gsHevcDecInit_Info.gsHevcDecInit.m_bEnableVideoCache   = p_init_param->m_bEnableVideoCache;
  pInst->gsHevcDecInit_Info.gsHevcDecInit.m_bCbCrInterleaveMode   = p_init_param->m_bCbCrInterleaveMode;

  pInst->extFunction = p_init_user_param->extFunction;
#ifdef SET_FRAMEBUFFER_INTO_MAX
  if( (pInst->extFunction & EXT_FUNC_MAX_FRAMEBUFFER) != 0x0 )
  {
    pInst->gsHevcDecInit_Info.gsHevcDecInit.m_uiDecOptFlags |= (1<<16);
    DSTATUS("[HEVC-%d]Set framebuffer into 2160p", pInst->vdec_instance_index);

    pInst->gsHevcDecInit_Info.gsHevcDecInit.m_Reserved[3] = AVAILABLE_HEVC_MAX_WIDTH;
    pInst->gsHevcDecInit_Info.gsHevcDecInit.m_Reserved[4] = AVAILABLE_HEVC_MAX_HEIGHT;
    pInst->gsHevcDecInit_Info.gsHevcDecInit.m_Reserved[5] = 10;
  }
#endif

    #if REMOVE_SECURE_COPY
    pInst->gsHevcDecInit_Info.gsHevcDecInit.m_uiDecOptFlags |= (1<<26);
    #endif

#if HEVC_SUPPORT_SINGLE
    tcc_printf("\n\n[HEVC-%d] $ single core $\n\n", pInst->vdec_instance_index);
    pInst->gsHevcDecInit_Info.gsHevcDecInit.m_uiDecOptFlags |= (1<<6);
#endif

#if HEVC_DISABLE_10BIT_OUT
    tcc_printf("\n\n[HEVC-%d] $ Disable 10bit output $\n\n", pInst->vdec_instance_index);
    pInst->gsHevcDecInit_Info.gsHevcDecInit.m_uiDecOptFlags |= WAVE4_10BITS_DISABLE;
#endif

  pInst->gsHevcDecInit_Info.gsHevcDecInit.m_iFilePlayEnable   = p_init_param->m_bFilePlayEnable;
  pInst->gsbHasSeqHeader = 0;//p_init_param->m_bHasSeqHeader;

  pInst->gsHevcDecInit_Info.gsHevcDecInit.m_iBitstreamBufSize = p_init_param->m_iBitstreamBufSize;
  ret = hevc_dec_ready( &pInst->gsHevcDecInit_Info.gsHevcDecInit, pInst );
  if( ret != RETCODE_SUCCESS )
  {
    return ret;
  }

  DSTATUS("[HEVC-%d]workbuff 0x%x/0x%x, Reg: 0x%x, format : %d, Stream(0x%x/0x%x, %d)", pInst->vdec_instance_index,
      pInst->gsHevcDecInit_Info.gsHevcDecInit.m_BitWorkAddr[PA], pInst->gsHevcDecInit_Info.gsHevcDecInit.m_BitWorkAddr[VA], pInst->gsHevcDecInit_Info.gsHevcDecInit.m_RegBaseVirtualAddr,
      pInst->gsHevcDecInit_Info.gsHevcDecInit.m_iBitstreamFormat, pInst->gsHevcDecInit_Info.gsHevcDecInit.m_BitstreamBufAddr[PA], pInst->gsHevcDecInit_Info.gsHevcDecInit.m_BitstreamBufAddr[VA], pInst->gsHevcDecInit_Info.gsHevcDecInit.m_iBitstreamBufSize);
  DSTATUS("[HEVC-%d]optFlag 0x%x, Userdata(%d), VCache: %d, Inter: %d, PlayEn: %d", pInst->vdec_instance_index, pInst->gsHevcDecInit_Info.gsHevcDecInit.m_uiDecOptFlags,
      pInst->gsHevcDecInit_Info.gsHevcDecInit.m_bEnableUserData, /*pInst->gsHevcDecInit_Info.gsHevcDecInit.m_bEnableVideoCache*/ 0,
      pInst->gsHevcDecInit_Info.gsHevcDecInit.m_bCbCrInterleaveMode, pInst->gsHevcDecInit_Info.gsHevcDecInit.m_iFilePlayEnable);

  if( (pInst->extFunction & EXT_FUNC_NO_BUFFER_DELAY) != 0x0 )
  {
    LOGI("[HEVC-%d] : No BufferDelay Mode....", pInst->vdec_instance_index);
    pInst->gsHevcDecInit_Info.gsHevcDecInit.m_uiDecOptFlags |= (1<<2);
  }

#ifdef USE_MAP_CONVERTER
  if( (pInst->extFunction & EXT_FUNC_USE_MAP_CONVERTER) != 0x0 )
  {
    LOGI("[HEVC-%d] : Map Converter Mode ON!!", pInst->vdec_instance_index);
  }
  else
#endif
  {
    LOGI("[HEVC-%d] : Map Converter Mode OFF!!", pInst->vdec_instance_index);
    pInst->gsHevcDecInit_Info.gsHevcDecInit.m_uiDecOptFlags |= WAVE4_WTL_ENABLE; // No compressed output!!, Must use 2 x framebuffer.
  }

  DSTATUS("[HEVC-%d]Format : %d, Stream(0x%x, %d)", pInst->vdec_instance_index, pInst->gsHevcDecInit_Info.gsHevcDecInit.m_iBitstreamFormat, pInst->gsHevcDecInit_Info.gsHevcDecInit.m_BitstreamBufAddr[PA], pInst->gsHevcDecInit_Info.gsHevcDecInit.m_iBitstreamBufSize);
  ret = hevc_cmd_process(V_DEC_INIT, &pInst->gsHevcDecInit_Info, pInst);
  if( ret != RETCODE_SUCCESS )
  {
    DPRINTF( "[HEVC-%d] HEVC_DEC_INIT failed Error code is 0x%x ", pInst->vdec_instance_index, ret );
    return -ret;
  }

  if( (pInst->extFunction & (EXT_FUNC_MEM_PROTECTION_WITH_INSTREAM | EXT_FUNC_MEM_PROTECTION_ONLY)) == 0x0 )
  {
    pInst->gsHevcDecVersion.pszVersion = (char*)(pInst->gsBitstreamBufAddr[K_VA] + (pInst->gsBitstreamBufSize - 100));
    pInst->gsHevcDecVersion.pszBuildData = (char*)(pInst->gsBitstreamBufAddr[K_VA] + (pInst->gsBitstreamBufSize - 50));

    ret = hevc_cmd_process(V_GET_VPU_VERSION, &pInst->gsHevcDecVersion, pInst);
    if( ret != RETCODE_SUCCESS )
    {
    //If this operation returns fail, it doesn't mean that there's a problem in vpu
    //so do not return error to host.
    DPRINTF( "[HEVC-%d] V_GET_HEVC_VERSION failed Error code is 0x%x ", pInst->vdec_instance_index, ret );
    }
    else
    {
    int vpu_closed[VPU_MAX*2];

    if( 0 <= ioctl(pInst->mgr_fd, VPU_CHECK_CODEC_STATUS, &vpu_closed) ) {
      LOGI("[HEVC-%d] Multi-instance status : %d/%d/%d/%d/%d", pInst->vdec_instance_index, vpu_closed[VPU_DEC], vpu_closed[VPU_DEC_EXT], vpu_closed[VPU_DEC_EXT2], vpu_closed[VPU_DEC_EXT3], vpu_closed[VPU_ENC]);
    }
    tcc_printf( "[HEVC-%d] V_GET_HEVC_VERSION OK. Version is %.32s, and it's built at %.10s ", pInst->vdec_instance_index,
        (char*)(pInst->gsBitstreamBufAddr[VA] + (pInst->gsBitstreamBufSize - 100)),
        (char*)(pInst->gsBitstreamBufAddr[VA] + (pInst->gsBitstreamBufSize - 50)));
    }
  }

  pInst->vdec_codec_opened = 1;

  LOGI( "[HEVC-%d] HEVC_DEC_INIT OK( has seq = %d) ", pInst->vdec_instance_index, pInst->gsbHasSeqHeader );

  }
  else if( iOpCode == VDEC_DEC_SEQ_HEADER )
  {
  vdec_input_t* p_input_param = (vdec_input_t*)pParam1;
  vdec_output_t* p_output_param = (vdec_output_t*)pParam2;
  int seq_stream_size = (p_input_param->m_iInpLen > pInst->gsMaxBitstreamSize) ? pInst->gsMaxBitstreamSize : p_input_param->m_iInpLen;
    uint32_t iIsThumbnail = p_input_param->m_iIsThumbnail;

  if(pInst->gsHevcDecInit_Info.gsHevcDecInit.m_iFilePlayEnable)
  {
    if (  ((codec_addr_t)p_input_param->m_pInp[PA] == pInst->gsBitstreamBufAddr[PA])
       && ((codec_addr_t)p_input_param->m_pInp[VA] == pInst->gsBitstreamBufAddr[VA]) )
    {
    pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_BitstreamDataAddr[PA] = pInst->gsBitstreamBufAddr[PA];
    pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_BitstreamDataAddr[VA] = pInst->gsBitstreamBufAddr[K_VA];
    }
    else
    {
    pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_BitstreamDataAddr[PA] = pInst->gsBitstreamBufAddr[PA];
    pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_BitstreamDataAddr[VA] = pInst->gsBitstreamBufAddr[K_VA];

#if defined(TC_SECURE_MEMORY_COPY)
    if (pInst->extFunction & EXT_FUNC_MEM_PROTECTION_WITH_INSTREAM) {
      #if REMOVE_SECURE_COPY
      pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_iBitstreamDataSize = seq_stream_size;
      pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_BitstreamDataAddr[PA] = p_input_param->m_pInp[PA];
      pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_BitstreamDataAddr[VA] = p_input_param->m_pInp[VA];
      #else
      TC_SecureMemoryCopy(pInst->gsBitstreamBufAddr[PA], p_input_param->m_pInp[PA], seq_stream_size);
      #endif
    }
    else
#endif
    {
          (void)memcpy( (unsigned long*)pInst->gsBitstreamBufAddr[VA], (unsigned long*)p_input_param->m_pInp[VA], seq_stream_size);
    }

#ifdef CHANGE_INPUT_STREAM
    change_input_stream((unsigned char *)pInst->gsBitstreamBufAddr[VA], &seq_stream_size, iOpCode, pInst);
#endif
    }

    if( (pInst->extFunction & (EXT_FUNC_MEM_PROTECTION_WITH_INSTREAM | EXT_FUNC_MEM_PROTECTION_ONLY)) == 0x0 )
    {
    unsigned char* ps = (unsigned char*)pInst->gsBitstreamBufAddr[VA];
    DSTATUS( "[HEVC-%d Seq %d] " "0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x " "0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x "
          "0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x " "0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x "
          "0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x ",
          pInst->vdec_instance_index, seq_stream_size,
          ps[0], ps[1], ps[2], ps[3], ps[4], ps[5], ps[6], ps[7], ps[8], ps[9], ps[10], ps[11], ps[12], ps[13], ps[14], ps[15],
          ps[16], ps[17], ps[18], ps[19], ps[20], ps[21], ps[22], ps[23], ps[24], ps[25], ps[26], ps[27], ps[28], ps[29], ps[30], ps[31],
          ps[32], ps[33], ps[34], ps[35], ps[36], ps[37], ps[38], ps[39], ps[40], ps[41], ps[42], ps[43], ps[44], ps[45], ps[46], ps[47],
          ps[48], ps[49], ps[50], ps[51], ps[52], ps[53], ps[54], ps[55], ps[56], ps[57], ps[58], ps[59], ps[60], ps[61], ps[62], ps[63],
          ps[64], ps[65], ps[66], ps[67], ps[68], ps[69], ps[70], ps[71], ps[72], ps[73], ps[74], ps[75], ps[76], ps[77], ps[78], ps[79]);
    }
  }
  else
  {
    seq_stream_size = 1;
  }

  DSTATUS( "[HEVC-%d] VDEC_DEC_SEQ_HEADER start  :: len = %d / %d ", pInst->vdec_instance_index, seq_stream_size, p_input_param->m_iInpLen);
  ret = hevc_dec_seq_header(seq_stream_size, iIsThumbnail, pInst);
  if( ret != RETCODE_SUCCESS )
  {
    return ret;
  }

  pInst->gsbHasSeqHeader = 1;
  p_output_param->m_pInitialInfo = &pInst->gsCommDecInfo;
#if REMOVE_SECURE_COPY
    if (pInst->gsHevcDecInOut_Info.gsHevcDecInitialInfo.m_uiUserData) {
      if (hevc_set_user_data_info(&pInst->gsHevcDecInOut_Info.gsHevcDecInitialInfo.m_UserDataInfo,
                    &p_output_param->m_UserDataInfo, pInst) < 0) {
        pInst->gsHevcDecInOut_Info.gsHevcDecInitialInfo.m_uiUserData = 0;
      }

      notify_hdmi_drm_config(pInst, &p_output_param->m_UserDataInfo);
    }

  //check the maximum/minimum video resolution limitation
  {
    vdec_info_t * pVdecInfo = (vdec_info_t *)&pInst->gsHevcDecInOut_Info.gsHevcDecInitialInfo;
    int max_width, max_height;
    int min_width, min_height;

    max_width   = ((AVAILABLE_HEVC_MAX_WIDTH+15)&0xFFF0);
    max_height  = ((AVAILABLE_HEVC_MAX_HEIGHT+15)&0xFFF0);
    min_width   = AVAILABLE_MIN_WIDTH;
    min_height  = AVAILABLE_MIN_HEIGHT;

    if(  (pVdecInfo->m_iPicWidth > max_width)
    || ((pVdecInfo->m_iPicWidth * pVdecInfo->m_iPicHeight) > AVAILABLE_HEVC_MAX_REGION)
    || (pVdecInfo->m_iPicWidth < min_width)
    || (pVdecInfo->m_iPicHeight < min_height) )
    {
    ret = 0 - RETCODE_INVALID_STRIDE;
    DPRINTF( "[HEVC-%d] VDEC_DEC_SEQ_HEADER - don't support the resolution %dx%d  ", pInst->vdec_instance_index,
          pVdecInfo->m_iPicWidth, pVdecInfo->m_iPicHeight);
    return ret;
    }
  }

#else
    if (pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo.m_uiUserData) {
      if (hevc_set_user_data_info(&pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo.m_UserDataInfo,
                    &p_output_param->m_UserDataInfo, pInst) < 0) {
        pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo.m_uiUserData = 0;
      }

      notify_hdmi_drm_config(pInst, &p_output_param->m_UserDataInfo);
    }

  //check the maximum/minimum video resolution limitation
  {
    vdec_info_t * pVdecInfo = (vdec_info_t *)&pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo;
    int max_width, max_height;
    int min_width, min_height;

    max_width   = ((AVAILABLE_HEVC_MAX_WIDTH+15)&0xFFF0);
    max_height  = ((AVAILABLE_HEVC_MAX_HEIGHT+15)&0xFFF0);
    min_width   = AVAILABLE_MIN_WIDTH;
    min_height  = AVAILABLE_MIN_HEIGHT;

    if(  (pVdecInfo->m_iPicWidth > max_width)
    || ((pVdecInfo->m_iPicWidth * pVdecInfo->m_iPicHeight) > AVAILABLE_HEVC_MAX_REGION)
    || (pVdecInfo->m_iPicWidth < min_width)
    || (pVdecInfo->m_iPicHeight < min_height) )
    {
    ret = 0 - RETCODE_INVALID_STRIDE;
    DPRINTF( "[HEVC-%d] VDEC_DEC_SEQ_HEADER - don't support the resolution %dx%d  ", pInst->vdec_instance_index,
          pVdecInfo->m_iPicWidth, pVdecInfo->m_iPicHeight);
    return ret;
    }
  }
#endif
  LOGI( "[HEVC-%d] VDEC_DEC_SEQ_HEADER - Success mem_free = 0x%x ", pInst->vdec_instance_index, cdk_sys_final_free_mem(pInst) );
  DSTATUS( "[HEVC] =======================================================" );
  }
  else if( iOpCode == VDEC_DECODE )
  {
  vdec_input_t* p_input_param = (vdec_input_t*)pParam1;
  vdec_output_t* p_output_param = (vdec_output_t*)pParam2;

#ifdef USE_PREV_STREAMBUFF_DECODING
  pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_iBitstreamDataSize = (p_input_param->m_iInpLen > pInst->gsMaxBitstreamSize/2) ? pInst->gsMaxBitstreamSize/2 : p_input_param->m_iInpLen;
#else
  pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_iBitstreamDataSize = (p_input_param->m_iInpLen > pInst->gsMaxBitstreamSize) ? pInst->gsMaxBitstreamSize : p_input_param->m_iInpLen;
#endif

#if defined(TC_SECURE_MEMORY_COPY)
  if( (pInst->extFunction & EXT_FUNC_MEM_PROTECTION_WITH_INSTREAM) != 0x0 )
  {
  #if REMOVE_SECURE_COPY
    pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_BitstreamDataAddr[PA] = p_input_param->m_pInp[PA];
    pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_BitstreamDataAddr[VA] = p_input_param->m_pInp[VA];
    #else
    if( pInst->gsCurrStreamBuffer_index == 0 ){
    pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_BitstreamDataAddr[PA] = pInst->gsBitstreamBufAddr[PA] + (pInst->gsBitstreamBufSize / 3);
    pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_BitstreamDataAddr[VA] = pInst->gsBitstreamBufAddr[K_VA] + (pInst->gsBitstreamBufSize / 3);
    //memcpy( (void*)pInst->gsBitstreamBufAddr[VA], (void*)p_input_param->m_pInp[VA], pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_iBitstreamDataSize);
    TC_SecureMemoryCopy(pInst->gsBitstreamBufAddr[PA] + (pInst->gsBitstreamBufSize / 3), p_input_param->m_pInp[PA], pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_iBitstreamDataSize);
    }
    else {
    pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_BitstreamDataAddr[PA] = pInst->gsBitstreamBufAddr[PA] + (pInst->gsBitstreamBufSize / 3 * 2);
    pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_BitstreamDataAddr[VA] = pInst->gsBitstreamBufAddr[K_VA] + (pInst->gsBitstreamBufSize / 3 * 2);
    //memcpy( (void*)(pInst->gsBitstreamBufAddr[VA] + pInst->gsBitstreamBufSize / 2), (void*)p_input_param->m_pInp[VA], pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_iBitstreamDataSize);
    TC_SecureMemoryCopy((pInst->gsBitstreamBufAddr[PA] + (pInst->gsBitstreamBufSize / 3 * 2)), p_input_param->m_pInp[PA], pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_iBitstreamDataSize);
    }
  #endif

  }
  else
#endif
  if( pInst->gsHevcDecInit_Info.gsHevcDecInit.m_iFilePlayEnable )
  {
    if (  ((codec_addr_t)p_input_param->m_pInp[PA] == pInst->gsBitstreamBufAddr[PA])
     && ((codec_addr_t)p_input_param->m_pInp[VA] == pInst->gsBitstreamBufAddr[VA]) )
    {
    pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_BitstreamDataAddr[PA] = pInst->gsBitstreamBufAddr[PA];
    pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_BitstreamDataAddr[VA] = pInst->gsBitstreamBufAddr[K_VA];
    }
    else
    {
#ifdef USE_PREV_STREAMBUFF_DECODING
    if( pInst->gsCurrStreamBuffer_index == 0 ){
      pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_BitstreamDataAddr[PA] = pInst->gsBitstreamBufAddr[PA];
      pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_BitstreamDataAddr[VA] = pInst->gsBitstreamBufAddr[K_VA];
      (void)memcpy( (unsigned long*)pInst->gsBitstreamBufAddr[VA], (unsigned long*)p_input_param->m_pInp[VA], pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_iBitstreamDataSize);
    }
    else {
      pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_BitstreamDataAddr[PA] = pInst->gsBitstreamBufAddr[PA] + pInst->gsBitstreamBufSize / 2;
      pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_BitstreamDataAddr[VA] = pInst->gsBitstreamBufAddr[K_VA] + pInst->gsBitstreamBufSize / 2;
      (void)memcpy( (unsigned long*)(pInst->gsBitstreamBufAddr[VA] + pInst->gsBitstreamBufSize / 2), (unsigned long*)p_input_param->m_pInp[VA], pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_iBitstreamDataSize);
    }
#else
    pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_BitstreamDataAddr[PA] = pInst->gsBitstreamBufAddr[PA];
    pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_BitstreamDataAddr[VA] = pInst->gsBitstreamBufAddr[K_VA];
    (void)memcpy( (unsigned long*)pInst->gsBitstreamBufAddr[VA], (unsigned long*)p_input_param->m_pInp[VA], pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_iBitstreamDataSize);
#endif
#ifdef CHANGE_INPUT_STREAM
    change_input_stream((unsigned char *)pInst->gsBitstreamBufAddr[VA], (&pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_iBitstreamDataSize), iOpCode, pInst);
#endif
    }
  }
  else
  {
    pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_BitstreamDataAddr[PA] = pInst->gsBitstreamBufAddr[PA];
    pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_BitstreamDataAddr[VA] = pInst->gsBitstreamBufAddr[K_VA];
    pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_iBitstreamDataSize = 1;
  }

  if(pInst->gsHevcDecInit_Info.gsHevcDecInit.m_bEnableUserData)
  {
    pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_UserDataAddr[PA] = pInst->gsUserdataBufAddr[PA];
    pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_UserDataAddr[VA] = pInst->gsUserdataBufAddr[K_VA];
    pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_iUserDataBufferSize = pInst->gsUserdataBufSize;
  }

  pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_iSkipFrameMode = 0;
  if (p_input_param->m_iFrameSearchEnable)
  {
    pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_iSkipFrameMode = 1;
  }
  else
  {
    if(p_input_param->m_iSkipFrameMode == 2)
    {
    pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_iSkipFrameMode = p_input_param->m_iSkipFrameMode;
    }
  }

  // Start decoding a frame.
  ret = hevc_cmd_process(V_DEC_DECODE, &pInst->gsHevcDecInOut_Info, pInst);
  pInst->total_frm++;
//  if(gsHevcDecInOut_Info.gsHevcDecOutput.m_DecOutInfo.m_iOutputStatus != VPU_DEC_OUTPUT_SUCCESS)
//    LOGD("systemtime:: ## decoded frame but no-output");
//  else
//    LOGD("systemtime:: ## decoded frame");

//  if( ret == VPU_DEC_FINISH )
//    return ERR_END_OF_FILE;
  if( ret != RETCODE_SUCCESS )
  {
    DPRINTF( "[HEVC-%d] HEVC_DEC_DECODE failed Error code is 0x%x ", pInst->vdec_instance_index, ret );

    return -ret;
  }

  if(pInst->gsHevcDecInOut_Info.gsHevcDecOutput.m_DecOutInfo.m_iPicType == 0){
    DSTATUS( "[HEVC-%d] I-Frame (%d)", pInst->vdec_instance_index, pInst->total_frm);
  }

  hevc_copy_dec_out_info(&pInst->gsHevcDecInOut_Info.gsHevcDecOutput, p_output_param, pInst);

  p_output_param->m_pInitialInfo = &pInst->gsCommDecInfo;

#if defined(HEVC_OUT_FRAME_DUMP) && defined(USE_MAP_CONVERTER)
  if(pInst->total_frm == 2)
  {
    codec_addr_t vYaddr, vCBaddr, vOffYaddr, vOffCBaddr;

    vYaddr =(codec_addr_t)cdk_sys_malloc_virtual_addr(  p_output_param->m_MapConvInfo.m_CompressedY[0], p_output_param->m_pInitialInfo->m_iPicWidth *p_output_param->m_pInitialInfo->m_iPicHeight, pInst );
    vCBaddr =(codec_addr_t)cdk_sys_malloc_virtual_addr(  p_output_param->m_MapConvInfo.m_CompressedCb[0], p_output_param->m_pInitialInfo->m_iPicWidth *p_output_param->m_pInitialInfo->m_iPicHeight, pInst );
    vOffYaddr =(codec_addr_t)cdk_sys_malloc_virtual_addr(  p_output_param->m_MapConvInfo.m_FbcYOffsetAddr[0], p_output_param->m_pInitialInfo->m_iPicWidth *p_output_param->m_pInitialInfo->m_iPicHeight, pInst );
    vOffCBaddr =(codec_addr_t)cdk_sys_malloc_virtual_addr(  p_output_param->m_MapConvInfo.m_FbcCOffsetAddr[0], p_output_param->m_pInitialInfo->m_iPicWidth *p_output_param->m_pInitialInfo->m_iPicHeight, pInst );

    save_MapConverter_frame(vYaddr, vCBaddr,vOffYaddr, vOffCBaddr, p_output_param->m_pInitialInfo->m_iPicWidth , p_output_param->m_pInitialInfo->m_iPicHeight);

    cdk_sys_free_virtual_addr(vYaddr,  p_output_param->m_pInitialInfo->m_iPicWidth *p_output_param->m_pInitialInfo->m_iPicHeight);
    cdk_sys_free_virtual_addr(vCBaddr,  p_output_param->m_pInitialInfo->m_iPicWidth *p_output_param->m_pInitialInfo->m_iPicHeight);
    cdk_sys_free_virtual_addr(vOffYaddr,  p_output_param->m_pInitialInfo->m_iPicWidth *p_output_param->m_pInitialInfo->m_iPicHeight);
    cdk_sys_free_virtual_addr(vOffCBaddr,  p_output_param->m_pInitialInfo->m_iPicWidth *p_output_param->m_pInitialInfo->m_iPicHeight);
  }
#endif//HEVC_OUT_FRAME_DUMP

  p_output_param->m_pDispOut[VA][COMP_Y] = (unsigned char *)vpu_getFrameBufVirtAddr(p_output_param->m_pDispOut[VA][COMP_Y], K_VA, pInst);
  p_output_param->m_pDispOut[VA][COMP_U] = (unsigned char *)vpu_getFrameBufVirtAddr(p_output_param->m_pDispOut[VA][COMP_U], K_VA, pInst);
  p_output_param->m_pDispOut[VA][COMP_V] = (unsigned char *)vpu_getFrameBufVirtAddr(p_output_param->m_pDispOut[VA][COMP_V], K_VA, pInst);

  p_output_param->m_pCurrOut[VA][COMP_Y] = (unsigned char *)vpu_getFrameBufVirtAddr(p_output_param->m_pCurrOut[VA][COMP_Y], K_VA, pInst);
  p_output_param->m_pCurrOut[VA][COMP_U] = (unsigned char *)vpu_getFrameBufVirtAddr(p_output_param->m_pCurrOut[VA][COMP_U], K_VA, pInst);
  p_output_param->m_pCurrOut[VA][COMP_V] = (unsigned char *)vpu_getFrameBufVirtAddr(p_output_param->m_pCurrOut[VA][COMP_V], K_VA, pInst);

  DSTATUS("Display :: %d :: %p %p %p / %p %p %p ", p_output_param->m_DecOutInfo.m_iDispOutIdx,
      p_output_param->m_pDispOut[PA][COMP_Y], p_output_param->m_pDispOut[PA][COMP_U], p_output_param->m_pDispOut[PA][COMP_V],
      p_output_param->m_pDispOut[VA][COMP_Y], p_output_param->m_pDispOut[VA][COMP_U], p_output_param->m_pDispOut[VA][COMP_V]);

  DSTATUS("Dec :: %d :: %p %p %p / %p %p %p ", p_output_param->m_DecOutInfo.m_iDecodedIdx,
      p_output_param->m_pCurrOut[PA][COMP_Y], p_output_param->m_pCurrOut[PA][COMP_U], p_output_param->m_pCurrOut[PA][COMP_V],
      p_output_param->m_pCurrOut[VA][COMP_Y], p_output_param->m_pCurrOut[VA][COMP_U], p_output_param->m_pCurrOut[VA][COMP_V]);

  if(pInst->gsHevcDecInit_Info.gsHevcDecInit.m_bEnableUserData)
  {
      uint32_t addr_gap = 0;

    addr_gap = pInst->gsUserdataBufAddr[K_VA] - pInst->gsHevcDecInOut_Info.gsHevcDecOutput.m_DecOutInfo.m_UserDataAddress[VA];
    p_output_param->m_DecOutInfo.m_UserDataAddress[VA] = pInst->gsUserdataBufAddr[VA] + addr_gap;

    if (pInst->gsHevcDecInOut_Info.gsHevcDecOutput.m_DecOutInfo.m_uiUserData) {
        if (hevc_set_user_data_info(&pInst->gsHevcDecInOut_Info.gsHevcDecOutput.m_DecOutInfo.m_UserDataInfo,
                      &p_output_param->m_UserDataInfo, pInst) < 0) {
          pInst->gsHevcDecInOut_Info.gsHevcDecOutput.m_DecOutInfo.m_uiUserData = 0;
        }

        notify_hdmi_drm_config(pInst, &p_output_param->m_UserDataInfo);
        p_output_param->m_uiUserData = pInst->gsHevcDecInOut_Info.gsHevcDecOutput.m_DecOutInfo.m_uiUserData;
      }
  }

  if(pInst->gsHevcDecInOut_Info.gsHevcDecOutput.m_DecOutInfo.m_iDecodingStatus != VPU_DEC_BUF_FULL) {
    if( pInst->gsCurrStreamBuffer_index == 0 )
    pInst->gsCurrStreamBuffer_index = 1;
    else
    pInst->gsCurrStreamBuffer_index = 0;
  }

  if(pInst->gsHevcDecInOut_Info.gsHevcDecOutput.m_DecOutInfo.m_iOutputStatus == VPU_DEC_OUTPUT_SUCCESS){
//    LOGE("Displayed addr 0x%x", p_output_param->m_pDispOut[PA][0]);
#ifdef VPU_OUT_FRAME_DUMP
    save_decoded_frame((unsigned char *)vpu_getFrameBufVirtAddr(p_output_param->m_pDispOut[PA][0], PA, pInst),
        (unsigned char *)vpu_getFrameBufVirtAddr(p_output_param->m_pDispOut[PA][1], PA, pInst),
        (unsigned char *)vpu_getFrameBufVirtAddr(p_output_param->m_pDispOut[PA][2], PA, pInst),
        pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo.m_iPicWidth, pInst->gsHevcDecSeqHeader_Info.gsHevcDecInitialInfo.m_iPicHeight, pInst);
#endif
  }

  DISPLAY_BUFFER("[HEVC-%d] Display idx = %d", pInst->vdec_instance_index, pInst->gsHevcDecInOut_Info.gsHevcDecOutput.m_DecOutInfo.m_iDispOutIdx);
  }
  else if( iOpCode == VDEC_GET_RING_BUFFER_STATUS )
  {
  vdec_ring_buffer_out_t* p_out_param = (vdec_ring_buffer_out_t*)pParam2;

  ret = hevc_cmd_process(V_GET_RING_BUFFER_STATUS, &pInst->gsHevcDecBufStatus, pInst); // get the available space in the ring buffer
  if( ret != RETCODE_SUCCESS )
  {
    DPRINTF( "[HEVC-%d] GET_RING_BUFFER_STATUS failed Error code is 0x%x ", pInst->vdec_instance_index, ret );
    return -ret;
  }
  p_out_param->m_ulAvailableSpaceInRingBuffer = pInst->gsHevcDecBufStatus.gsHevcDecRingStatus.m_ulAvailableSpaceInRingBuffer;
  p_out_param->m_ptrReadAddr_PA = pInst->gsHevcDecBufStatus.gsHevcDecRingStatus.m_ptrReadAddr_PA;
  p_out_param->m_ptrWriteAddr_PA = pInst->gsHevcDecBufStatus.gsHevcDecRingStatus.m_ptrWriteAddr_PA;
//  LOGE("[VDEC] [AVAIL: %8d] [RP: 0x%08X / WP: 0x%08X]"
//    , pInst->gsHevcDecBufStatus.gsHevcDecRingStatus.m_ulAvailableSpaceInRingBuffer
//    , pInst->gsHevcDecBufStatus.gsHevcDecRingStatus.m_ptrReadAddr_PA
//    , pInst->gsHevcDecBufStatus.gsHevcDecRingStatus.m_ptrWriteAddr_PA
//    );
  }
  else if( iOpCode == VDEC_FILL_RING_BUFFER )
  {
  vdec_ring_buffer_set_t* p_set_param = (vdec_ring_buffer_set_t*)pParam1;

    (void)memcpy((unsigned long*)pInst->gsIntermediateBufAddr[VA],(unsigned long*)p_set_param->m_pbyBuffer, p_set_param->m_uiBufferSize);
    pInst->gsHevcDecBufFill.gsHevcDecRingFeed.m_iOnePacketBufferSize = p_set_param->m_uiBufferSize;
  pInst->gsHevcDecBufFill.gsHevcDecRingFeed.m_OnePacketBufferAddr = pInst->gsIntermediateBufAddr[K_VA];

  ret = hevc_cmd_process(V_FILL_RING_BUFFER_AUTO, &pInst->gsHevcDecBufFill, pInst);  // fille the Ring Buffer

  if( ret != RETCODE_SUCCESS )
  {
    DPRINTF( "[HEVC-%d] FILL_RING_BUFFER_AUTO failed Error code is 0x%x ", pInst->vdec_instance_index, ret );
    return -ret;
  }
  }
  else if( iOpCode == VDEC_GET_INTERMEDIATE_BUF_INFO )
  {
    *(uint32_t*)pParam1 = pInst->gsIntermediateBufAddr[VA];
    *(uint32_t*)pParam2 = pInst->gsIntermediateBufSize;
  return 0;
  }
  else if( iOpCode == VDEC_UPDATE_WRITE_BUFFER_PTR )
  {
  pInst->gsHevcDecUpdateWP.iCopiedSize = (int)pParam1;
  pInst->gsHevcDecUpdateWP.iFlushBuf = (int)pParam2;

  ret = hevc_cmd_process(V_DEC_UPDATE_RINGBUF_WP, &pInst->gsHevcDecUpdateWP,  pInst);  // fille the Ring Buffer

  if( ret != RETCODE_SUCCESS )
  {
    DPRINTF( "[HEVC-%d] VDEC_UPDATE_WRITE_BUFFER_PTR failed Error code is 0x%x ", pInst->vdec_instance_index, ret );
    return -ret;
  }
  }
  else if( iOpCode == VDEC_BUF_FLAG_CLEAR )
  {
    unsigned long idx_display = *(unsigned long*)pParam1;
    CLEAR_BUFFER("[HEVC-%d] ************* cleared idx = %d", pInst->vdec_instance_index, idx_display);
    ret = hevc_cmd_process(V_DEC_BUF_FLAG_CLEAR, &idx_display, pInst);

    if( ret != RETCODE_SUCCESS )
    {
      DPRINTF( "[HEVC-%d] HEVC_DEC_BUF_FLAG_CLEAR failed Error code is 0x%x ", pInst->vdec_instance_index, ret );
      return -ret;
    }
  }
  else if( iOpCode == VDEC_DEC_FLUSH_OUTPUT)
  {
  vdec_input_t* p_input_param = (vdec_input_t*)pParam1;
  vdec_output_t* p_output_param = (vdec_output_t*)pParam2;

  pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_BitstreamDataAddr[PA] = pInst->gsBitstreamBufAddr[PA];
  pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_BitstreamDataAddr[VA] = pInst->gsBitstreamBufAddr[K_VA];
  pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_iBitstreamDataSize = 0;
  pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_iSkipFrameMode = VDEC_SKIP_FRAME_DISABLE;
//  pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_iFrameSearchEnable = 0;
//  pInst->gsHevcDecInOut_Info.gsHevcDecInput.m_iSkipFrameNum = 0;

  ret = hevc_cmd_process(V_DEC_FLUSH_OUTPUT, &pInst->gsHevcDecInOut_Info, pInst);

  if( ret != RETCODE_SUCCESS )
  {
    DPRINTF( "[HEVC-%d] VDEC_DEC_FLUSH_OUTPUT failed Error code is 0x%x ", pInst->vdec_instance_index, ret );
    return -ret;
  }

  hevc_copy_dec_out_info(&pInst->gsHevcDecInOut_Info.gsHevcDecOutput, p_output_param, pInst);
  p_output_param->m_pInitialInfo = &pInst->gsCommDecInfo;

  p_output_param->m_pDispOut[VA][COMP_Y] = (unsigned char *)vpu_getFrameBufVirtAddr(p_output_param->m_pDispOut[VA][COMP_Y], K_VA, pInst);
  p_output_param->m_pDispOut[VA][COMP_U] = (unsigned char *)vpu_getFrameBufVirtAddr(p_output_param->m_pDispOut[VA][COMP_U], K_VA, pInst);
  p_output_param->m_pDispOut[VA][COMP_V] = (unsigned char *)vpu_getFrameBufVirtAddr(p_output_param->m_pDispOut[VA][COMP_V], K_VA, pInst);

  p_output_param->m_pCurrOut[VA][COMP_Y] = (unsigned char *)vpu_getFrameBufVirtAddr(p_output_param->m_pCurrOut[VA][COMP_Y], K_VA, pInst);
  p_output_param->m_pCurrOut[VA][COMP_U] = (unsigned char *)vpu_getFrameBufVirtAddr(p_output_param->m_pCurrOut[VA][COMP_U], K_VA, pInst);
  p_output_param->m_pCurrOut[VA][COMP_V] = (unsigned char *)vpu_getFrameBufVirtAddr(p_output_param->m_pCurrOut[VA][COMP_V], K_VA, pInst);

  if(pInst->gsHevcDecInit_Info.gsHevcDecInit.m_bEnableUserData)
  {
      uint32_t addr_gap = 0;

    addr_gap = pInst->gsUserdataBufAddr[K_VA] - pInst->gsHevcDecInOut_Info.gsHevcDecOutput.m_DecOutInfo.m_UserDataAddress[VA];
    p_output_param->m_DecOutInfo.m_UserDataAddress[VA] = pInst->gsUserdataBufAddr[VA] + addr_gap;
  }
  }
  else if( iOpCode == VDEC_SW_RESET)
  {
  ret = hevc_cmd_process(V_DEC_SWRESET, NULL, pInst);

  if( ret != RETCODE_SUCCESS )
  {
    DPRINTF( "[HEVC-%d] V_DEC_SWRESET failed Error code is 0x%x ", pInst->vdec_instance_index, ret );
    return -ret;
  }
  }
  else if( iOpCode == VDEC_CLOSE )
  {

  if(pInst->vdec_codec_opened)
  {
    ret = hevc_cmd_process(V_DEC_CLOSE, &pInst->gsHevcDecInOut_Info, pInst);
    if( ret != RETCODE_SUCCESS )
    {
    DPRINTF( "[HEVC-%d] HEVC_DEC_CLOSE failed Error code is 0x%x ", pInst->vdec_instance_index, ret );
    ret = -ret;
    }

    pInst->vdec_codec_opened = 0;
  }

  if(!pInst->vdec_env_opened)
    return -RETCODE_NOT_INITIALIZED;

  cdk_sys_free_virtual_addr( (unsigned long*)pInst->gsBitstreamBufAddr[VA], pInst->gsBitstreamBufSize);
  cdk_sys_free_virtual_addr( (unsigned long*)pInst->gsUserdataBufAddr[VA], pInst->gsUserdataBufSize );
  cdk_sys_free_virtual_addr( (unsigned long*)pInst->gsBitWorkBufAddr[VA], pInst->gsBitWorkBufSize );
  cdk_sys_free_virtual_addr( (unsigned long*)pInst->gsFrameBufAddr[VA], pInst->gsFrameBufSize );

  vpu_env_close(pInst);
  }
  else
  {
  DPRINTF( "[HEVC-%d] Invalid Operation!!", pInst->vdec_instance_index );
  return -ret;
  }

  return ret;
}
#endif

#endif //TCC_HEVC_INCLUDE
