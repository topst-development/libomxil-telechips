// SPDX-License-Identifier: LGPL-2.1-or later
/****************************************************************************
 *   FileName  : vdec_k_vpu.c
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

#ifdef TCC_VPU_INCLUDE
#define LOG_TAG "VPU_DEC_K"

#include "TCCMemory.h"

#include <sys/mman.h>
#include <errno.h>

#include <sys/ioctl.h>
#if defined(USE_COMMON_KERNEL_LOCATION)
#include <tcc_vpu_ioctl.h>
#else //use chipset folder
#include <mach/tcc_vpu_ioctl.h>
#endif

#include <dlfcn.h>

#include <memory.h>
#include <stdio.h>
#include <string.h>

#include <fcntl.h>     // O_RDWR
#include <sys/poll.h>

static int32_t vdec_cmd_process(int32_t cmd, unsigned long* args, _vdec_ *pVdec)
{
  int32_t ret;
  int32_t cmd_success = 0;
  int32_t retry_cnt = POLL_RETRY_COUNT;
  int32_t all_retry_cnt = CMD_RETRY_COUNT;

  _vdec_ * pInst = (_vdec_ *)pVdec;

  if((ret = ioctl(pInst->dec_fd, cmd, args)) < 0)
  {
    if( ret == -0x999 )
    {
      LOGE("VDEC[%d] Invalid command(0x%x) ", pInst->vdec_instance_index, cmd);
      ret = RETCODE_INVALID_COMMAND;
      goto err_vdec_cmd_process;
    }
    else
    {
      LOGE("VDEC[%d] ioctl err[%s] : cmd = 0x%x", pInst->vdec_instance_index, strerror(errno), cmd);
    }
  }

Retry:
  while (retry_cnt > 0) {
    memset(pInst->tcc_event, 0, sizeof(pInst->tcc_event));
    pInst->tcc_event[0].fd = pInst->dec_fd;
    pInst->tcc_event[0].events = POLLIN;

    ret = poll((struct pollfd *)&pInst->tcc_event, 1, 1000); // 1 sec
    if (ret < 0) {
      LOGE("VDEC[%d] -retry(%d:cmd(%d)) poll error '%s'", pInst->vdec_instance_index, retry_cnt, cmd, strerror(errno));
      retry_cnt--;
      continue;
    }else if (ret == 0) {
      LOGE("VDEC[%d] -retry(%d:cmd(%d)) poll timeout: %u'th frames, len %d", pInst->vdec_instance_index, retry_cnt, cmd, pInst->total_frm, pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_iBitstreamDataSize );
      retry_cnt--;
      continue;
    }else if (ret > 0) {
      if (pInst->tcc_event[0].revents & POLLERR) {
        LOGE("VDEC[%d]  poll POLLERR", pInst->vdec_instance_index);
        break;
      } else if (pInst->tcc_event[0].revents & POLLIN) {
        cmd_success = 1;
        break;
      }
    }
  }
  /* todo */

  switch(cmd){
    case V_DEC_INIT:
    {
      VDEC_INIT_t* init_info = (VDEC_INIT_t *)args;

      if(ioctl(pInst->dec_fd, V_DEC_INIT_RESULT, args) < 0){
        LOGE("VDEC[%d] ioctl(0x%x) error[%s]!!", pInst->vdec_instance_index, V_DEC_INIT_RESULT, strerror(errno));
      }
      ret = init_info->result;
    }
    break;

    case V_DEC_SEQ_HEADER:
    {
#if REMOVE_SECURE_COPY
      VDEC_DECODE_t* seq_info = (VDEC_DECODE_t *)args;
#else
      VDEC_SEQ_HEADER_t* seq_info = (VDEC_SEQ_HEADER_t *)args;
#endif
      if(ioctl(pInst->dec_fd, V_DEC_SEQ_HEADER_RESULT, args) < 0){
        LOGE("VDEC[%d] ioctl(0x%x) error[%s]!!", pInst->vdec_instance_index, V_DEC_SEQ_HEADER_RESULT, strerror(errno));
      }
      ret = seq_info->result;
  #ifdef ERROR_TEST
      err_test = 0;
  #endif
    }
    break;

    case V_DEC_DECODE:
    {
      VDEC_DECODE_t* decoded_info = (VDEC_DECODE_t *)args;

      if(ioctl(pInst->dec_fd, V_DEC_DECODE_RESULT, args) < 0){
        LOGE("VDEC[%d] ioctl(0x%x) error[%s]!!", pInst->vdec_instance_index, V_DEC_DECODE_RESULT, strerror(errno));
      }
      ret = decoded_info->result;
    }
    break;

    case V_DEC_FLUSH_OUTPUT:
    {
      VDEC_DECODE_t* decoded_info = (VDEC_DECODE_t *)args;

      if(ioctl(pInst->dec_fd, V_DEC_FLUSH_OUTPUT_RESULT, args) < 0){
        LOGE("VDEC[%d] ioctl(0x%x) error[%s]!!", pInst->vdec_instance_index, V_DEC_FLUSH_OUTPUT_RESULT, strerror(errno));
      }
      ret = decoded_info->result;
    }
    break;

    case V_GET_RING_BUFFER_STATUS:
    {
      VDEC_RINGBUF_GETINFO_t* p_param = (VDEC_RINGBUF_GETINFO_t *)args;
      if(ioctl(pInst->dec_fd, V_GET_RING_BUFFER_STATUS_RESULT, args) < 0){
        LOGE("VDEC[%d] ioctl(0x%x) error[%s]!!", pInst->vdec_instance_index, V_GET_RING_BUFFER_STATUS_RESULT, strerror(errno));
      }
      ret = p_param->result;
    }
    break;
    case V_FILL_RING_BUFFER_AUTO:
    {
      VDEC_RINGBUF_SETBUF_t* p_param = (VDEC_RINGBUF_SETBUF_t *)args;
      if(ioctl(pInst->dec_fd, V_FILL_RING_BUFFER_AUTO_RESULT, args) < 0){
        LOGE("VDEC[%d] ioctl(0x%x) error[%s]!!", pInst->vdec_instance_index, V_FILL_RING_BUFFER_AUTO_RESULT, strerror(errno));
      }
      ret = p_param->result;
    }
    break;
    case V_DEC_UPDATE_RINGBUF_WP:
    {
      VDEC_RINGBUF_SETBUF_PTRONLY_t* p_param = (VDEC_RINGBUF_SETBUF_PTRONLY_t *)args;
      if(ioctl(pInst->dec_fd, V_DEC_UPDATE_RINGBUF_WP_RESULT, args) < 0){
        LOGE("VDEC[%d] ioctl(0x%x) error[%s]!!", pInst->vdec_instance_index, V_DEC_UPDATE_RINGBUF_WP_RESULT, strerror(errno));
      }
      ret = p_param->result;
    }
    break;
    case V_GET_INITIAL_INFO_FOR_STREAMING_MODE_ONLY:
    {
      VDEC_SEQ_HEADER_t* p_param = (VDEC_SEQ_HEADER_t *)args;
      if(ioctl(pInst->dec_fd, V_GET_INITIAL_INFO_FOR_STREAMING_MODE_ONLY_RESULT, args) < 0){
        LOGE("VDEC[%d] ioctl(0x%x) error[%s]!!", pInst->vdec_instance_index, V_GET_INITIAL_INFO_FOR_STREAMING_MODE_ONLY_RESULT, strerror(errno));
      }
      ret = p_param->result;
    }
    break;
    case V_GET_VPU_VERSION:
    {
      VDEC_GET_VERSION_t* p_param = (VDEC_GET_VERSION_t *)args;
      if(ioctl(pInst->dec_fd, V_GET_VPU_VERSION_RESULT, args) < 0){
        LOGE("VDEC[%d] ioctl(0x%x) error[%s]!!", pInst->vdec_instance_index, V_GET_VPU_VERSION_RESULT, strerror(errno));
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
    {
      if(ioctl(pInst->dec_fd, V_DEC_GENERAL_RESULT, &ret) < 0){
        LOGE("VDEC[%d] ioctl(0x%x) error[%s]!!", pInst->vdec_instance_index, V_DEC_GENERAL_RESULT, strerror(errno));
      }
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

  if((cmd_success == 0) ||
  	 ((ret&0xf000) != 0x0000) /* vpu can not start or finish its processing with unknown reason!! */
  )
  {
    LOGE("VDEC[%d] command(0x%x) didn't work properly. maybe hangup(no return(0x%x))!!", pInst->vdec_instance_index, cmd, ret);

    if((ret != RETCODE_CODEC_EXIT) && (ret != RETCODE_MULTI_CODEC_EXIT_TIMEOUT)){
//    ioctl(pInst->mgr_fd, VPU_HW_RESET, (void*)NULL);
    }
    ret = RETCODE_CODEC_EXIT;
  }

err_vdec_cmd_process:
  return ret;
}


int vpu_dec_ready( dec_init_t* psVDecInit, _vdec_ *pVdec)
{
  _vdec_ * pInst = (_vdec_ *)pVdec;

  //------------------------------------------------------------
  //! [x] bitstream buffer for each VPU decoder
  //------------------------------------------------------------

  if(psVDecInit->m_iBitstreamBufSize > LARGE_STREAM_BUF_SIZE){
    pInst->gsBitstreamBufSize = ALIGNED_BUFF( psVDecInit->m_iBitstreamBufSize, 64*1024 );
  } else {
    pInst->gsBitstreamBufSize = LARGE_STREAM_BUF_SIZE;
    pInst->gsBitstreamBufSize = ALIGNED_BUFF( pInst->gsBitstreamBufSize, ALIGN_LEN );
  }

  {
    pInst->gsBitstreamBufAddr[PA] = (codec_addr_t)cdk_sys_malloc_physical_addr(&pInst->gsBitstreamBufAddr[K_VA], pInst->gsBitstreamBufSize, BUFFER_STREAM, pInst );

    if( pInst->gsBitstreamBufAddr[PA] == 0 )
    {
      DPRINTF( "[VDEC-%d] bitstream_buf_addr[PA] malloc() failed ", pInst->vdec_instance_index);
      return -(VPU_NOT_ENOUGH_MEM);
    }
    DSTATUS( "[VDEC-%d] bitstream_buf_addr[PA] = 0x%lx, 0x%x ",
        pInst->vdec_instance_index, (codec_addr_t)pInst->gsBitstreamBufAddr[PA], pInst->gsBitstreamBufSize );
    pInst->gsBitstreamBufAddr[VA] = (codec_addr_t)cdk_sys_malloc_virtual_addr( (uint32_t)pInst->gsBitstreamBufAddr[PA], pInst->gsBitstreamBufSize, pInst );
    if( pInst->gsBitstreamBufAddr[VA] == 0 )
    {
      DPRINTF( "[VDEC-%d] bitstream_buf_addr[VA] malloc() failed ", pInst->vdec_instance_index);
      return -(VPU_NOT_ENOUGH_MEM);
    }
#if 0
    if( (pInst->extFunction & (EXT_FUNC_MEM_PROTECTION_WITH_INSTREAM | EXT_FUNC_MEM_PROTECTION_ONLY)) == 0x0 )
    {
      memset( (unsigned long*)pInst->gsBitstreamBufAddr[VA], 0x00 , pInst->gsBitstreamBufSize);
    }
#endif
    DSTATUS("[VDEC-%d] bitstream_buf_addr[VA] = 0x%lx, 0x%x ",
        pInst->vdec_instance_index, (codec_addr_t)pInst->gsBitstreamBufAddr[VA], pInst->gsBitstreamBufSize );
  }

  psVDecInit->m_BitstreamBufAddr[PA]  = pInst->gsBitstreamBufAddr[PA];
  psVDecInit->m_BitstreamBufAddr[VA]  = pInst->gsBitstreamBufAddr[K_VA];
  psVDecInit->m_iBitstreamBufSize   = pInst->gsBitstreamBufSize;

  //------------------------------------------------------------
  // [x] PS(SPS/PPS) buffer for each VPU decoder
  //------------------------------------------------------------
  if( psVDecInit->m_iBitstreamFormat == STD_AVC || psVDecInit->m_iBitstreamFormat == STD_MVC)
  {
    pInst->gsSpsPpsSize = PS_SAVE_SIZE;
    pInst->gsSpsPpsSize = ALIGNED_BUFF( pInst->gsSpsPpsSize, ALIGN_LEN );
    pInst->gsSpsPpsAddr = (codec_addr_t)cdk_sys_malloc_physical_addr( NULL, pInst->gsSpsPpsSize, BUFFER_PS, pInst );
    if( pInst->gsSpsPpsAddr == 0 )
    {
      DPRINTF( "[VDEC-%d] sps_pps_buf_addr malloc() failed ", pInst->vdec_instance_index);
      return -(VPU_NOT_ENOUGH_MEM);
    }
    DSTATUS("[VDEC-%d] sps_pps_buf_addr = 0x%x, 0x%x ", pInst->vdec_instance_index, (codec_addr_t)pInst->gsSpsPpsAddr, pInst->gsSpsPpsSize );

    psVDecInit->m_pSpsPpsSaveBuffer = (unsigned char*)pInst->gsSpsPpsAddr;
    psVDecInit->m_iSpsPpsSaveBufferSize = pInst->gsSpsPpsSize;
  }


  if(pInst->gsVpuDecInit_Info.gsVpuDecInit.m_iFilePlayEnable == 0)
  {
#if 0 // intermediate buffer is not used.
    pInst->gsIntermediateBufSize = LARGE_STREAM_BUF_SIZE;
    pInst->gsIntermediateBufSize = ALIGNED_BUFF( pInst->gsIntermediateBufSize, ALIGN_LEN );
    pInst->gsIntermediateBufAddr[PA] = (codec_addr_t)cdk_sys_malloc_physical_addr(&pInst->gsIntermediateBufAddr[K_VA], pInst->gsIntermediateBufSize, BUFFER_STREAM, pInst );

    if( pInst->gsIntermediateBufAddr[PA] == 0 )
    {
      DPRINTF( "[VDEC-%d] gsIntermediateBufAddr[PA] malloc() failed ", pInst->vdec_instance_index);
      return -(VPU_NOT_ENOUGH_MEM);
    }
    DSTATUS( "[VDEC-%d] bitstream_buf_addr[PA] = 0x%lx, 0x%x ",
        pInst->vdec_instance_index, (codec_addr_t)pInst->gsIntermediateBufAddr[PA], pInst->gsIntermediateBufSize );
    pInst->gsIntermediateBufAddr[VA] = (codec_addr_t)cdk_sys_malloc_virtual_addr( pInst->gsIntermediateBufAddr[PA], pInst->gsIntermediateBufSize, pInst );
    if( pInst->gsIntermediateBufAddr[VA] == 0 )
    {
      DPRINTF( "[VDEC-%d] gsIntermediateBufAddr[VA] malloc() failed ", pInst->vdec_instance_index);
      return -(VPU_NOT_ENOUGH_MEM);
    }
    memset( (unsigned long*)pInst->gsIntermediateBufAddr[VA], 0x00 , pInst->gsIntermediateBufSize);
    DSTATUS("[VDEC-%d] gsIntermediateBufAddr[VA] = 0x%lx, 0x%x ",
        pInst->vdec_instance_index, (codec_addr_t)pInst->gsIntermediateBufAddr[VA], pInst->gsIntermediateBufSize );
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

  //------------------------------------------------------------
  //! [x] user data buffer for each VPU decoder
  //------------------------------------------------------------
  if(pInst->gsVpuDecInit_Info.gsVpuDecInit.m_bEnableUserData != 0U)
  {
    pInst->gsUserdataBufSize = 50 * 1024;
    pInst->gsUserdataBufSize = ALIGNED_BUFF( pInst->gsUserdataBufSize, ALIGN_LEN );
    pInst->gsUserdataBufAddr[PA] = (codec_addr_t)cdk_sys_malloc_physical_addr(&pInst->gsUserdataBufAddr[K_VA], pInst->gsUserdataBufSize, BUFFER_USERDATA, pInst );
    if( pInst->gsUserdataBufAddr[PA] == 0UL )
    {
      DPRINTF( "[VDEC-%d:Err%d] pInst->gsUserdataBufAddr physical alloc failed ", pInst->vdec_instance_index, -1 );
      return -(VPU_NOT_ENOUGH_MEM);
    }
    DSTATUS( "[VDEC-%d] pInst->gsUserdataBufAddr[PA] = 0x%lx, 0x%x ",
        pInst->vdec_instance_index, (codec_addr_t)pInst->gsUserdataBufAddr[PA], pInst->gsUserdataBufSize );
    pInst->gsUserdataBufAddr[VA] = (codec_addr_t)cdk_sys_malloc_virtual_addr( (uint32_t)pInst->gsUserdataBufAddr[PA], pInst->gsUserdataBufSize, pInst );
    if( pInst->gsUserdataBufAddr[VA] == 0UL )
    {
      DPRINTF( "[VDEC-%d:Err%d] pInst->gsUserdataBufAddr virtual alloc failed ", pInst->vdec_instance_index, -1 );
      return -(VPU_NOT_ENOUGH_MEM);
    }
    //memset( (void*)pInst->gsUserdataBufAddr[VA], 0 , gsUserdataBufSize);
    DSTATUS("[VDEC-%d] pInst->gsUserdataBufAddr[VA] = 0x%x, 0x%x ", pInst->vdec_instance_index, (codec_addr_t)pInst->gsUserdataBufAddr[VA], pInst->gsUserdataBufSize );
  }

  //------------------------------------------------------------
  // [x] code buffer, work buffer and parameter buffer for VPU
  //------------------------------------------------------------
  pInst->gsBitWorkBufSize = WORK_CODE_PARA_BUF_SIZE;
  pInst->gsBitWorkBufSize = ALIGNED_BUFF(pInst->gsBitWorkBufSize, ALIGN_LEN);
  pInst->gsBitWorkBufAddr[PA] = (codec_addr_t)cdk_sys_malloc_physical_addr(&pInst->gsBitWorkBufAddr[K_VA], pInst->gsBitWorkBufSize, BUFFER_WORK, pInst );
  if( pInst->gsBitWorkBufAddr[PA] == 0U )
  {
    DPRINTF( "[VDEC-%d] bit_work_buf_addr[PA] malloc() failed ", pInst->vdec_instance_index);
    return -(VPU_NOT_ENOUGH_MEM);
  }
  DSTATUS("[VDEC-%d] bit_work_buf_addr[PA] = 0x%lx, 0x%x ",
      pInst->vdec_instance_index, (codec_addr_t)pInst->gsBitWorkBufAddr[PA], pInst->gsBitWorkBufSize );
  pInst->gsBitWorkBufAddr[VA] = (codec_addr_t)cdk_sys_malloc_virtual_addr( (uint32_t)pInst->gsBitWorkBufAddr[PA], pInst->gsBitWorkBufSize, pInst );
  if( pInst->gsBitWorkBufAddr[VA] == 0U )
  {
    DPRINTF( "[VDEC-%d] bit_work_buf_addr[VA] malloc() failed ", pInst->vdec_instance_index);
    return -(VPU_NOT_ENOUGH_MEM);
  }
  DSTATUS("[VDEC-%d] bit_work_buf_addr[VA] = 0x%lx, 0x%x ",
      pInst->vdec_instance_index, (codec_addr_t)pInst->gsBitWorkBufAddr[VA], pInst->gsBitWorkBufSize );

  psVDecInit->m_BitWorkAddr[PA] = pInst->gsBitWorkBufAddr[PA];
  psVDecInit->m_BitWorkAddr[VA] = pInst->gsBitWorkBufAddr[K_VA];
#ifdef SUPPORT_VCACHE_CTRL
  if( psVDecInit->m_bEnableVideoCache == 0U ){
    DSTATUS("[VDEC-%d] Cache OFF", pInst->vdec_instance_index);
  }
  else{
    DSTATUS("[VDEC-%d] Cache ON", pInst->vdec_instance_index);
  }
#endif

  if( psVDecInit->m_bCbCrInterleaveMode == 0U ){
    DSTATUS("[VDEC-%d] CbCrInterleaveMode OFF", pInst->vdec_instance_index);
  }
  else{
    DSTATUS("[VDEC-%d] CbCrInterleaveMode ON", pInst->vdec_instance_index);
  }

  if( psVDecInit->m_uiDecOptFlags&M4V_DEBLK_ENABLE != 0U){
    DSTATUS( "[VDEC-%d] MPEG-4 Deblocking ON" , pInst->vdec_instance_index);
  }
  if( psVDecInit->m_uiDecOptFlags&M4V_GMC_FRAME_SKIP != 0U){
    DSTATUS( "[VDEC-%d] MPEG-4 GMC Frame Skip" , pInst->vdec_instance_index);
  }

  return 0;
}

int vpu_dec_seq_header( int iSize, int iIsThumbnail, _vdec_ *pVdec )
{
  int ret = 0;
  _vdec_ * pInst = (_vdec_ *)pVdec;
#if REMOVE_SECURE_COPY
  LOGI("[VDEC-%d] vpu_dec_seq_header in :: size(%d), JpegOnly(%d), format(%d)",
      pInst->vdec_instance_index, iSize ,pInst->gsUserInfo.m_bStillJpeg, pInst->gsVpuDecInit_Info.gsVpuDecInit.m_iBitstreamFormat);

  if( (pInst->extFunction & (EXT_FUNC_MEM_PROTECTION_WITH_INSTREAM | EXT_FUNC_MEM_PROTECTION_ONLY)) == 0x0 )
  {
    unsigned char* ps = (unsigned char*)pInst->gsBitstreamBufAddr[VA];
    SEQ_EXTRACTOR( "[VDEC-%d Seq %d] " "0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x " "0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x "
        "0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x " "0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x "
        "0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x ",
        pInst->vdec_instance_index, iSize,
        ps[0], ps[1], ps[2], ps[3], ps[4], ps[5], ps[6], ps[7], ps[8], ps[9], ps[10], ps[11], ps[12], ps[13], ps[14], ps[15],
        ps[16], ps[17], ps[18], ps[19], ps[20], ps[21], ps[22], ps[23], ps[24], ps[25], ps[26], ps[27], ps[28], ps[29], ps[30], ps[31],
        ps[32], ps[33], ps[34], ps[35], ps[36], ps[37], ps[38], ps[39], ps[40], ps[41], ps[42], ps[43], ps[44], ps[45], ps[46], ps[47],
        ps[48], ps[49], ps[50], ps[51], ps[52], ps[53], ps[54], ps[55], ps[56], ps[57], ps[58], ps[59], ps[60], ps[61], ps[62], ps[63],
        ps[64], ps[65], ps[66], ps[67], ps[68], ps[69], ps[70], ps[71], ps[72], ps[73], ps[74], ps[75], ps[76], ps[77], ps[78], ps[79]);
  }

  ret = vdec_cmd_process(V_DEC_SEQ_HEADER, &pInst->gsVpuDecInOut_Info, pInst);
  if( ret != RETCODE_SUCCESS )
  {
    DPRINTF( "[VDEC-%d] VPU_DEC_SEQ_HEADER failed Error code is 0x%x. ErrorReason is %d",
        pInst->vdec_instance_index, ret, pInst->gsVpuDecInOut_Info.gsVpuDecInitialInfo.m_iReportErrorReason);
    if(ret == RETCODE_CODEC_SPECOUT){
      DPRINTF("[VDEC-%d] NOT SUPPORTED CODEC. VPU SPEC OUT!!", pInst->vdec_instance_index);   // This is a very common error. Notice the detailed reason to users.
    }
    pInst->gsCommDecInfo.m_iErrCode = pInst->gsVpuDecInOut_Info.gsVpuDecInitialInfo.m_iReportErrorReason; // mm008
    return -ret;
  }

  print_dec_initial_info( &pInst->gsVpuDecInit_Info.gsVpuDecInit, &pInst->gsVpuDecInOut_Info.gsVpuDecInitialInfo, pInst );

  if( (pInst->gsVpuDecInit_Info.gsVpuDecInit.m_iBitstreamFormat == STD_AVC) || (pInst->gsVpuDecInit_Info.gsVpuDecInit.m_iBitstreamFormat == STD_MVC) ){
    pInst->mRealPicWidth = pInst->gsVpuDecInOut_Info.gsVpuDecInitialInfo.m_iPicWidth
                         - pInst->gsVpuDecInOut_Info.gsVpuDecInitialInfo.m_iAvcPicCrop.m_iCropLeft
                         - pInst->gsVpuDecInOut_Info.gsVpuDecInitialInfo.m_iAvcPicCrop.m_iCropRight;
    pInst->mRealPicHeight = pInst->gsVpuDecInOut_Info.gsVpuDecInitialInfo.m_iPicHeight
                         - pInst->gsVpuDecInOut_Info.gsVpuDecInitialInfo.m_iAvcPicCrop.m_iCropBottom
                         - pInst->gsVpuDecInOut_Info.gsVpuDecInitialInfo.m_iAvcPicCrop.m_iCropTop;
  }
  else{
    pInst->mRealPicWidth = pInst->gsVpuDecInOut_Info.gsVpuDecInitialInfo.m_iPicWidth;
    pInst->mRealPicHeight = pInst->gsVpuDecInOut_Info.gsVpuDecInitialInfo.m_iPicHeight;
  }
#ifdef SET_FRAMEBUFFER_INTO_MAX
  if( (pInst->extFunction & EXT_FUNC_MAX_FRAMEBUFFER) != 0x0 )
  {
    pInst->gsVpuDecInOut_Info.gsVpuDecInitialInfo.m_iPicWidth = VPU_FRAMEBUFFER_MAX_WIDTH;//AVAILABLE_MAX_WIDTH;
    pInst->gsVpuDecInOut_Info.gsVpuDecInitialInfo.m_iPicHeight = VPU_FRAMEBUFFER_MAX_HEIGHT;//AVAILABLE_MAX_HEIGHT;
    LOGI("[VDEC-%d]Set seq framebuffer into 1080p (<- %d x %d)", pInst->vdec_instance_index, pInst->mRealPicWidth, pInst->mRealPicHeight);
  }
  else
#endif
  {
    pInst->gsVpuDecInOut_Info.gsVpuDecInitialInfo.m_iPicWidth = ((pInst->mRealPicWidth+15)>>4)<<4;
    pInst->gsVpuDecInOut_Info.gsVpuDecInitialInfo.m_iPicHeight = pInst->mRealPicHeight;
  }

  set_dec_common_info(pInst->gsVpuDecInOut_Info.gsVpuDecInitialInfo.m_iPicWidth,
      pInst->gsVpuDecInOut_Info.gsVpuDecInitialInfo.m_iPicHeight,
      &pInst->gsVpuDecInOut_Info.gsVpuDecInitialInfo.m_iAvcPicCrop,
      pInst->gsVpuDecInOut_Info.gsVpuDecInitialInfo.m_iInterlace,
      0, pInst );

  //------------------------------------------------------------
  // [x] slice buffer for VPU
  //------------------------------------------------------------
  if( pInst->gsVpuDecInit_Info.gsVpuDecInit.m_iBitstreamFormat == STD_AVC || pInst->gsVpuDecInit_Info.gsVpuDecInit.m_iBitstreamFormat == STD_MVC )
  {
    pInst->gsSliceBufSize = SLICE_SAVE_SIZE;
    pInst->gsSliceBufSize = ALIGNED_BUFF( pInst->gsSliceBufSize, ALIGN_LEN );
    pInst->gsSliceBufAddr = (codec_addr_t)cdk_sys_malloc_physical_addr( NULL, pInst->gsSliceBufSize, BUFFER_SLICE, pInst );
    if( pInst->gsSliceBufAddr == 0 )
    {
      DPRINTF( "[VDEC-%d] slice_buf_addr malloc() failed ", pInst->vdec_instance_index);
      return -(VPU_NOT_ENOUGH_MEM);
    }
    DSTATUS("[VDEC-%d] slice_buf_addr = 0x%x, 0x%x ", pInst->vdec_instance_index,
        (codec_addr_t)pInst->gsSliceBufAddr, pInst->gsSliceBufSize );

    pInst->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_AvcSliceSaveBufferAddr  = pInst->gsSliceBufAddr;
    pInst->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_iAvcSliceSaveBufferSize = pInst->gsSliceBufSize;
  }
  else
  {
    pInst->gsSliceBufSize = 0;
    pInst->gsSliceBufAddr = 0;
  }

#if defined(SUPPORT_VP8_DECODER)
  if( pInst->gsVpuDecInit_Info.gsVpuDecInit.m_iBitstreamFormat == STD_VP8 )
  {
    pInst->gsMbSaveSize = 17*4*((pInst->gsVpuDecInOut_Info.gsVpuDecInitialInfo.m_iPicWidth * pInst->gsVpuDecInOut_Info.gsVpuDecInitialInfo.m_iPicHeight)>>8);
    pInst->gsMbSaveSize = ALIGNED_BUFF( pInst->gsMbSaveSize, 4*1024 );
    pInst->gsMbSaveAddr = (codec_addr_t)cdk_sys_malloc_physical_addr( NULL, pInst->gsMbSaveSize, BUFFER_ELSE, pInst );
    if( pInst->gsMbSaveAddr == 0 )
    {
      DPRINTF( "[VDEC-%d] MbSaveAddr malloc() failed ", pInst->vdec_instance_index);
      return -(VPU_NOT_ENOUGH_MEM);
    }
    DSTATUS("[VDEC-%d] MbSaveAddr = 0x%x, 0x%x ", pInst->vdec_instance_index, (codec_addr_t)pInst->gsMbSaveAddr, pInst->gsMbSaveSize );

    pInst->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_Vp8MbDataSaveBufferAddr = pInst->gsMbSaveAddr;
    pInst->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_iVp8MbDataSaveBufferSize = pInst->gsMbSaveSize;
  }
  else
  {
    pInst->gsMbSaveSize = 0;
    pInst->gsMbSaveAddr = 0;
  }
#endif

  //------------------------------------------------------------
  // [x] frame buffer for each VPU decoder
  //------------------------------------------------------------
  pInst->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_iFrameBufferCount = pInst->gsVpuDecInOut_Info.gsVpuDecInitialInfo.m_iMinFrameBufferCount;
  LOGD( "[VDEC-%d] FrameBufDelay %d, MinFrameBufferCount %d", pInst->vdec_instance_index,
      pInst->gsVpuDecInOut_Info.gsVpuDecInitialInfo.m_iFrameBufDelay, pInst->gsVpuDecInOut_Info.gsVpuDecInitialInfo.m_iMinFrameBufferCount );

  {
    int max_count;

    if(!iIsThumbnail){
      pInst->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_iFrameBufferCount =
          pInst->gsVpuDecInOut_Info.gsVpuDecInitialInfo.m_iMinFrameBufferCount + pInst->gsAdditionalFrameCount;
    }
    max_count = cdk_sys_remain_memory_size(pInst) / pInst->gsVpuDecInOut_Info.gsVpuDecInitialInfo.m_iMinFrameBufferSize;

    if(pInst->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_iFrameBufferCount > max_count){
      pInst->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_iFrameBufferCount = max_count;
    }
    if(pInst->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_iFrameBufferCount > MAX_FRAME_BUFFER_COUNT){
      pInst->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_iFrameBufferCount = MAX_FRAME_BUFFER_COUNT;
    }
    if(iIsThumbnail)
    {
      if(pInst->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_iFrameBufferCount < (pInst->gsVpuDecInOut_Info.gsVpuDecInitialInfo.m_iMinFrameBufferCount))
      {
        LOGE( "[VDEC-%d] Not enough memory for VPU frame buffer, Available[%d], Min[%d], Need[%d], min_size = %d",
           pInst->vdec_instance_index, max_count, pInst->gsVpuDecInOut_Info.gsVpuDecInitialInfo.m_iMinFrameBufferCount,
           pInst->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_iFrameBufferCount, pInst->gsVpuDecInOut_Info.gsVpuDecInitialInfo.m_iMinFrameBufferSize);
        return -(VPU_NOT_ENOUGH_MEM);
      }
    }
    else
    {
      if(pInst->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_iFrameBufferCount <
      	(pInst->gsVpuDecInOut_Info.gsVpuDecInitialInfo.m_iMinFrameBufferCount+pInst->gsAdditionalFrameCount))
      {
        LOGE( "[VDEC-%d] Not enough memory for VPU frame buffer, Available[%d], Min[%d], Need[%d], min_size = %d",
           pInst->vdec_instance_index, max_count, pInst->gsVpuDecInOut_Info.gsVpuDecInitialInfo.m_iMinFrameBufferCount,
           pInst->gsVpuDecInOut_Info.gsVpuDecInitialInfo.m_iMinFrameBufferCount + pInst->gsAdditionalFrameCount,
           pInst->gsVpuDecInOut_Info.gsVpuDecInitialInfo.m_iMinFrameBufferSize);
        return -(VPU_NOT_ENOUGH_MEM);
      }
#ifdef SET_FRAMEBUFFER_INTO_MAX
      if( (pInst->extFunction & EXT_FUNC_MAX_FRAMEBUFFER) != 0x0 )
      {
        if( pInst->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_iFrameBufferCount < (max_count - 1) ){
          pInst->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_iFrameBufferCount = (max_count - 1);
        }
        if(pInst->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_iFrameBufferCount > MAX_FRAME_BUFFER_COUNT){
          pInst->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_iFrameBufferCount = MAX_FRAME_BUFFER_COUNT;
        }
      }
#endif
    }
  }

  pInst->gsTotalFrameCount = pInst->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_iFrameBufferCount;

  pInst->gsFrameBufSize = pInst->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_iFrameBufferCount * pInst->gsVpuDecInOut_Info.gsVpuDecInitialInfo.m_iMinFrameBufferSize;
  LOGD( "[VDEC-%d] FrameBufferCount %d [min %d], min_size = %d ", pInst->vdec_instance_index,
      pInst->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_iFrameBufferCount, pInst->gsVpuDecInOut_Info.gsVpuDecInitialInfo.m_iMinFrameBufferCount,
      pInst->gsVpuDecInOut_Info.gsVpuDecInitialInfo.m_iMinFrameBufferSize);

  pInst->gsFrameBufSize = ALIGNED_BUFF( pInst->gsFrameBufSize, ALIGN_LEN );
  pInst->gsFrameBufAddr[PA] = (codec_addr_t)cdk_sys_malloc_physical_addr(&pInst->gsFrameBufAddr[K_VA], pInst->gsFrameBufSize, BUFFER_FRAMEBUFFER, pInst );
  if( pInst->gsFrameBufAddr[PA] == 0 )
  {
    DPRINTF( "[VDEC-%d] frame_buf_addr[PA] malloc() failed ", pInst->vdec_instance_index);
    return -(VPU_NOT_ENOUGH_MEM);
  }

  DSTATUS( "[VDEC-%d] MinFrameBufferSize %d bytes ", pInst->vdec_instance_index, pInst->gsVpuDecInOut_Info.gsVpuDecInitialInfo.m_iMinFrameBufferSize );
  DSTATUS( "[VDEC-%d] frame_buf_addr[PA] = 0x%lx, 0x%x , index = %d ", pInst->vdec_instance_index,
      (codec_addr_t)pInst->gsFrameBufAddr[PA], pInst->gsFrameBufSize, pInst->vdec_instance_index );
  pInst->gsFrameBufAddr[VA] = (codec_addr_t)cdk_sys_malloc_virtual_addr( (uint32_t)pInst->gsFrameBufAddr[PA], pInst->gsFrameBufSize, pInst );
  if( pInst->gsFrameBufAddr[VA] == 0 )
  {
    DPRINTF( "[VDEC-%d] frame_buf_addr[VA] malloc() failed ", pInst->vdec_instance_index);
    return -(VPU_NOT_ENOUGH_MEM);
  }
  DSTATUS("[VDEC-%d] frame_buf_addr[VA] = 0x%lx, frame_buf_addr[K_VA] = 0x%x ", pInst->vdec_instance_index,
      (codec_addr_t)pInst->gsFrameBufAddr[VA], pInst->gsFrameBufAddr[K_VA] );
  pInst->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_FrameBufferStartAddr[PA] = pInst->gsFrameBufAddr[PA];
  pInst->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_FrameBufferStartAddr[VA] = pInst->gsFrameBufAddr[K_VA];

#else
  LOGI("[VDEC-%d] vpu_dec_seq_header in :: size(%d), JpegOnly(%d), format(%d)",
      pInst->vdec_instance_index, iSize ,pInst->gsUserInfo.m_bStillJpeg, pInst->gsVpuDecInit_Info.gsVpuDecInit.m_iBitstreamFormat);
  pInst->gsVpuDecSeqHeader_Info.stream_size = (uint32_t)iSize;
  if( (pInst->extFunction & (EXT_FUNC_MEM_PROTECTION_WITH_INSTREAM | EXT_FUNC_MEM_PROTECTION_ONLY)) == 0x0 )
  {
    char* ps = (char*)pInst->gsBitstreamBufAddr[VA];
    SEQ_EXTRACTOR( "[VDEC-%d Seq %d] " "0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x " "0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x "
        "0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x " "0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x "
        "0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x ",
        pInst->vdec_instance_index, iSize,
        ps[0], ps[1], ps[2], ps[3], ps[4], ps[5], ps[6], ps[7], ps[8], ps[9], ps[10], ps[11], ps[12], ps[13], ps[14], ps[15],
        ps[16], ps[17], ps[18], ps[19], ps[20], ps[21], ps[22], ps[23], ps[24], ps[25], ps[26], ps[27], ps[28], ps[29], ps[30], ps[31],
        ps[32], ps[33], ps[34], ps[35], ps[36], ps[37], ps[38], ps[39], ps[40], ps[41], ps[42], ps[43], ps[44], ps[45], ps[46], ps[47],
        ps[48], ps[49], ps[50], ps[51], ps[52], ps[53], ps[54], ps[55], ps[56], ps[57], ps[58], ps[59], ps[60], ps[61], ps[62], ps[63],
        ps[64], ps[65], ps[66], ps[67], ps[68], ps[69], ps[70], ps[71], ps[72], ps[73], ps[74], ps[75], ps[76], ps[77], ps[78], ps[79]);
  }

  ret = vdec_cmd_process(V_DEC_SEQ_HEADER, &pInst->gsVpuDecSeqHeader_Info, pInst);
  if( ret != RETCODE_SUCCESS )
  {
    DPRINTF( "[VDEC-%d] VPU_DEC_SEQ_HEADER failed Error code is 0x%x. ErrorReason is %d",
        pInst->vdec_instance_index, ret, pInst->gsVpuDecSeqHeader_Info.gsVpuDecInitialInfo.m_iReportErrorReason);
    if(ret == RETCODE_CODEC_SPECOUT){
      DPRINTF("[VDEC-%d] NOT SUPPORTED CODEC. VPU SPEC OUT!!", pInst->vdec_instance_index);   // This is a very common error. Notice the detailed reason to users.
    }
    pInst->gsCommDecInfo.m_iErrCode = pInst->gsVpuDecSeqHeader_Info.gsVpuDecInitialInfo.m_iReportErrorReason; // mm008
    return -ret;
  }

  print_dec_initial_info( &pInst->gsVpuDecInit_Info.gsVpuDecInit, &pInst->gsVpuDecSeqHeader_Info.gsVpuDecInitialInfo, pInst );

  if( (pInst->gsVpuDecInit_Info.gsVpuDecInit.m_iBitstreamFormat == STD_AVC) || (pInst->gsVpuDecInit_Info.gsVpuDecInit.m_iBitstreamFormat == STD_MVC) ){
    pInst->mRealPicWidth = pInst->gsVpuDecSeqHeader_Info.gsVpuDecInitialInfo.m_iPicWidth
                         - pInst->gsVpuDecSeqHeader_Info.gsVpuDecInitialInfo.m_iAvcPicCrop.m_iCropLeft
                         - pInst->gsVpuDecSeqHeader_Info.gsVpuDecInitialInfo.m_iAvcPicCrop.m_iCropRight;
    pInst->mRealPicHeight = pInst->gsVpuDecSeqHeader_Info.gsVpuDecInitialInfo.m_iPicHeight
                          - pInst->gsVpuDecSeqHeader_Info.gsVpuDecInitialInfo.m_iAvcPicCrop.m_iCropBottom
                          - pInst->gsVpuDecSeqHeader_Info.gsVpuDecInitialInfo.m_iAvcPicCrop.m_iCropTop;
  }
  else{
    pInst->mRealPicWidth = pInst->gsVpuDecSeqHeader_Info.gsVpuDecInitialInfo.m_iPicWidth;
    pInst->mRealPicHeight = pInst->gsVpuDecSeqHeader_Info.gsVpuDecInitialInfo.m_iPicHeight;
  }
#ifdef SET_FRAMEBUFFER_INTO_MAX
  if( (pInst->extFunction & EXT_FUNC_MAX_FRAMEBUFFER) != 0x0 )
  {
    pInst->gsVpuDecSeqHeader_Info.gsVpuDecInitialInfo.m_iPicWidth = VPU_FRAMEBUFFER_MAX_WIDTH;//AVAILABLE_MAX_WIDTH;
    pInst->gsVpuDecSeqHeader_Info.gsVpuDecInitialInfo.m_iPicHeight = VPU_FRAMEBUFFER_MAX_HEIGHT;//AVAILABLE_MAX_HEIGHT;
    LOGI("[VDEC-%d]Set seq framebuffer into 1080p (<- %d x %d)", pInst->vdec_instance_index, pInst->mRealPicWidth, pInst->mRealPicHeight);
  }
  else
#endif
  {
    pInst->gsVpuDecSeqHeader_Info.gsVpuDecInitialInfo.m_iPicWidth = ((pInst->mRealPicWidth+15)>>4)<<4;
    pInst->gsVpuDecSeqHeader_Info.gsVpuDecInitialInfo.m_iPicHeight = pInst->mRealPicHeight;
  }

  set_dec_common_info(pInst->gsVpuDecSeqHeader_Info.gsVpuDecInitialInfo.m_iPicWidth,
      pInst->gsVpuDecSeqHeader_Info.gsVpuDecInitialInfo.m_iPicHeight,
     &pInst->gsVpuDecSeqHeader_Info.gsVpuDecInitialInfo.m_iAvcPicCrop,
      pInst->gsVpuDecSeqHeader_Info.gsVpuDecInitialInfo.m_iInterlace,
      0, pInst );

  //------------------------------------------------------------
  // [x] slice buffer for VPU
  //------------------------------------------------------------
  if( (pInst->gsVpuDecInit_Info.gsVpuDecInit.m_iBitstreamFormat == STD_AVC) || (pInst->gsVpuDecInit_Info.gsVpuDecInit.m_iBitstreamFormat == STD_MVC) )
  {
    pInst->gsSliceBufSize = SLICE_SAVE_SIZE;
    pInst->gsSliceBufSize = ALIGNED_BUFF( pInst->gsSliceBufSize, ALIGN_LEN );
    pInst->gsSliceBufAddr = (codec_addr_t)cdk_sys_malloc_physical_addr( NULL, pInst->gsSliceBufSize, BUFFER_SLICE, pInst );
    if( pInst->gsSliceBufAddr == 0 )
    {
      DPRINTF( "[VDEC-%d] slice_buf_addr malloc() failed ", pInst->vdec_instance_index);
      return -(VPU_NOT_ENOUGH_MEM);
    }
    DSTATUS("[VDEC-%d] slice_buf_addr = 0x%x, 0x%x ", pInst->vdec_instance_index, (codec_addr_t)pInst->gsSliceBufAddr, pInst->gsSliceBufSize );

    pInst->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_AvcSliceSaveBufferAddr  = pInst->gsSliceBufAddr;
    pInst->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_iAvcSliceSaveBufferSize = pInst->gsSliceBufSize;
  }
  else
  {
    pInst->gsSliceBufSize = 0;
    pInst->gsSliceBufAddr = 0;
  }

#if defined(SUPPORT_VP8_DECODER)
  if( pInst->gsVpuDecInit_Info.gsVpuDecInit.m_iBitstreamFormat == STD_VP8 )
  {
    pInst->gsMbSaveSize = 17*4*((pInst->gsVpuDecSeqHeader_Info.gsVpuDecInitialInfo.m_iPicWidth * pInst->gsVpuDecSeqHeader_Info.gsVpuDecInitialInfo.m_iPicHeight)>>8);
    pInst->gsMbSaveSize = ALIGNED_BUFF( pInst->gsMbSaveSize, 4*1024 );
    pInst->gsMbSaveAddr = (codec_addr_t)cdk_sys_malloc_physical_addr( NULL, pInst->gsMbSaveSize, BUFFER_ELSE, pInst );
    if( pInst->gsMbSaveAddr == 0 )
    {
      DPRINTF( "[VDEC-%d] MbSaveAddr malloc() failed ", pInst->vdec_instance_index);
      return -(VPU_NOT_ENOUGH_MEM);
    }
    DSTATUS("[VDEC-%d] MbSaveAddr = 0x%x, 0x%x ", pInst->vdec_instance_index, (codec_addr_t)pInst->gsMbSaveAddr, pInst->gsMbSaveSize );

    pInst->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_Vp8MbDataSaveBufferAddr = pInst->gsMbSaveAddr;
    pInst->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_iVp8MbDataSaveBufferSize = pInst->gsMbSaveSize;
  }
  else
  {
    pInst->gsMbSaveSize = 0;
    pInst->gsMbSaveAddr = 0;
  }
#endif

  //------------------------------------------------------------
  // [x] frame buffer for each VPU decoder
  //------------------------------------------------------------
  pInst->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_iFrameBufferCount = pInst->gsVpuDecSeqHeader_Info.gsVpuDecInitialInfo.m_iMinFrameBufferCount;
  LOGD( "[VDEC-%d] FrameBufDelay %d, MinFrameBufferCount %d", pInst->vdec_instance_index,
       pInst->gsVpuDecSeqHeader_Info.gsVpuDecInitialInfo.m_iFrameBufDelay, pInst->gsVpuDecSeqHeader_Info.gsVpuDecInitialInfo.m_iMinFrameBufferCount);

  {
    int max_count;

    if(iIsThumbnail == 0){
      pInst->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_iFrameBufferCount =
          pInst->gsVpuDecSeqHeader_Info.gsVpuDecInitialInfo.m_iMinFrameBufferCount + pInst->gsAdditionalFrameCount;
    }
    max_count = (int32_t)cdk_sys_remain_memory_size(pInst) / pInst->gsVpuDecSeqHeader_Info.gsVpuDecInitialInfo.m_iMinFrameBufferSize;

    if(pInst->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_iFrameBufferCount > max_count){
      pInst->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_iFrameBufferCount = max_count;
    }
    if(pInst->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_iFrameBufferCount > MAX_FRAME_BUFFER_COUNT){
      pInst->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_iFrameBufferCount = MAX_FRAME_BUFFER_COUNT;
    }
    if(iIsThumbnail != 0)
    {
      if(pInst->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_iFrameBufferCount < (pInst->gsVpuDecSeqHeader_Info.gsVpuDecInitialInfo.m_iMinFrameBufferCount))
      {
        tcc_printf( "[VDEC-%d] [line:%d] Not enough memory for VPU frame buffer, Available[%d], Min[%d], Need[%d], min_size = %d",
                      pInst->vdec_instance_index, __LINE__, max_count,
                      pInst->gsVpuDecSeqHeader_Info.gsVpuDecInitialInfo.m_iMinFrameBufferCount,
                      pInst->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_iFrameBufferCount,
                      pInst->gsVpuDecSeqHeader_Info.gsVpuDecInitialInfo.m_iMinFrameBufferSize);
        return -(VPU_NOT_ENOUGH_MEM);
      }
    }
    else
    {
      if(pInst->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_iFrameBufferCount < (pInst->gsVpuDecSeqHeader_Info.gsVpuDecInitialInfo.m_iMinFrameBufferCount/*+pInst->gsAdditionalFrameCount*/))
      {
        tcc_printf( "[VDEC-%d] [line:%d] Not enough memory for VPU frame buffer, Available[%d], Min[%d], Need[%d], min_size = %d",
                      pInst->vdec_instance_index, __LINE__, max_count,
                      pInst->gsVpuDecSeqHeader_Info.gsVpuDecInitialInfo.m_iMinFrameBufferCount,
                      pInst->gsVpuDecSeqHeader_Info.gsVpuDecInitialInfo.m_iMinFrameBufferCount + pInst->gsAdditionalFrameCount,
                      pInst->gsVpuDecSeqHeader_Info.gsVpuDecInitialInfo.m_iMinFrameBufferSize);
        return -(VPU_NOT_ENOUGH_MEM);
      }
#ifdef SET_FRAMEBUFFER_INTO_MAX
      if( (pInst->extFunction & EXT_FUNC_MAX_FRAMEBUFFER) != 0x0 )
      {
        if( pInst->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_iFrameBufferCount < (max_count - 1) ){
          pInst->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_iFrameBufferCount = (max_count - 1);
        }
        if(pInst->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_iFrameBufferCount > MAX_FRAME_BUFFER_COUNT){
          pInst->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_iFrameBufferCount = MAX_FRAME_BUFFER_COUNT;
        }
      }
#endif
    }
  }

  pInst->gsTotalFrameCount = pInst->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_iFrameBufferCount;
  pInst->gsFrameBufSize = pInst->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_iFrameBufferCount * pInst->gsVpuDecSeqHeader_Info.gsVpuDecInitialInfo.m_iMinFrameBufferSize;
  LOGI( "[VDEC-%d] [line:%d] FrameBufferCount %d [min %d], min_size = %d ", pInst->vdec_instance_index, __LINE__,
       pInst->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_iFrameBufferCount,
       pInst->gsVpuDecSeqHeader_Info.gsVpuDecInitialInfo.m_iMinFrameBufferCount,
       pInst->gsVpuDecSeqHeader_Info.gsVpuDecInitialInfo.m_iMinFrameBufferSize);

  pInst->gsFrameBufSize = ALIGNED_BUFF( pInst->gsFrameBufSize, ALIGN_LEN );
  pInst->gsFrameBufAddr[PA] = (codec_addr_t)cdk_sys_malloc_physical_addr(&pInst->gsFrameBufAddr[K_VA], pInst->gsFrameBufSize, BUFFER_FRAMEBUFFER, pInst );
  if( pInst->gsFrameBufAddr[PA] == 0 )
  {
    tcc_printf( "[VDEC-%d] [line:%d] frame_buf_addr[PA] malloc() failed ", pInst->vdec_instance_index,__LINE__);
    return -(VPU_NOT_ENOUGH_MEM);
  }

  DSTATUS( "[VDEC-%d] MinFrameBufferSize %d bytes ", pInst->vdec_instance_index,
       pInst->gsVpuDecSeqHeader_Info.gsVpuDecInitialInfo.m_iMinFrameBufferSize );
  DSTATUS( "[VDEC-%d] frame_buf_addr[PA] = 0x%lx, 0x%x , index = %d ", pInst->vdec_instance_index,
      (codec_addr_t)pInst->gsFrameBufAddr[PA], pInst->gsFrameBufSize, pInst->vdec_instance_index );
  pInst->gsFrameBufAddr[VA] = (codec_addr_t)cdk_sys_malloc_virtual_addr( (uint32_t)pInst->gsFrameBufAddr[PA], pInst->gsFrameBufSize, pInst );
  if( pInst->gsFrameBufAddr[VA] == 0 )
  {
    DPRINTF( "[VDEC-%d] frame_buf_addr[VA] malloc() failed ", pInst->vdec_instance_index);
    return -(VPU_NOT_ENOUGH_MEM);
  }
  DSTATUS("[VDEC-%d] frame_buf_addr[VA] = 0x%lx, frame_buf_addr[K_VA] = 0x%x ", pInst->vdec_instance_index,
      (codec_addr_t)pInst->gsFrameBufAddr[VA], pInst->gsFrameBufAddr[K_VA] );
  pInst->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_FrameBufferStartAddr[PA] = pInst->gsFrameBufAddr[PA];
  pInst->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_FrameBufferStartAddr[VA] = pInst->gsFrameBufAddr[K_VA];
#endif
#if 0//def VPU_PERFORMANCE_UP
  {
     uint32_t regAddr = ((uint32_t)gsRegisterBase + 0x10000); //0xB0910000

   //VCACHE_CTRL
     *(volatile uint32_t *)(regAddr+0x00)   = (1<<0);       //CACHEON

   //VCACHE_REG
     *(volatile uint32_t *)(regAddr+0x04)   = (3<<0);       //WR0|RD0
     *(volatile uint32_t *)(regAddr+0x024)  = gsFrameBufAddr[PA];//VIDEO_PHY_ADDR;  //VCACHE_R0MIN
     *(volatile uint32_t *)(regAddr+0x028)  = VIDEO_PHY_ADDR+ VIDEO_MEM_SIZE;   //VCACHE_R0MAX
     *(volatile uint32_t *)(regAddr+0x02C)  = 0; //VCACHE_R1MIN
     *(volatile uint32_t *)(regAddr+0x030)  = 0; //VCACHE_R1MAX
     *(volatile uint32_t *)(regAddr+0x034)  = 0; //VCACHE_R2MIN
     *(volatile uint32_t *)(regAddr+0x038)  = 0; //VCACHE_R2MAX
     *(volatile uint32_t *)(regAddr+0x03C)  = 0; //VCACHE_R3MIN
     *(volatile uint32_t *)(regAddr+0x040)  = 0; //VCACHE_R3MAX
   }
#endif

  ret = vdec_cmd_process(V_DEC_REG_FRAME_BUFFER, &pInst->gsVpuDecBuffer_Info, pInst);
  if( ret != RETCODE_SUCCESS )
  {
    DPRINTF( "[VDEC-%d] DEC_REG_FRAME_BUFFER failed Error code is 0x%x ", pInst->vdec_instance_index, ret );
    ret = -ret;
  }
  else {
    DSTATUS("[VDEC-%d] TCC_VPU_DEC VPU_DEC_REG_FRAME_BUFFER OK!", pInst->vdec_instance_index);
  }
  return ret;
}

int32_t
vdec_vpu( int32_t iOpCode, unsigned long* pHandle, unsigned long* pParam1, unsigned long* pParam2, unsigned long* pParam3 )
{
  int ret = 0;
  _vdec_ *pInst = (_vdec_ *)pParam3;
  int buf_idx = 0;

  if(!pInst){
    LOGE("vdec_vpu(OP:%d) :: Instance is null!!", iOpCode);
    return -RETCODE_NOT_INITIALIZED;
  }

  if( (iOpCode != VDEC_INIT) && (iOpCode != VDEC_CLOSE) && (!pInst->vdec_codec_opened)){
    return -RETCODE_NOT_INITIALIZED;
  }

#ifdef DEBUG_TIME_LOG
  clock_t start;
  clock_t end;
  start = clock();
#endif

  switch(iOpCode)
  {
    case VDEC_INIT:
    {
      vdec_init_t* p_init_param = (vdec_init_t*)pParam1;
      vdec_user_info_t* p_init_user_param = (vdec_user_info_t*)pParam2;

      pInst->gsUserInfo.bitrate_mbps = p_init_user_param->bitrate_mbps;
      pInst->gsUserInfo.frame_rate   = p_init_user_param->frame_rate;
      pInst->gsUserInfo.m_bStillJpeg  = p_init_user_param->m_bStillJpeg;
      pInst->gsUserInfo.jpg_ScaleRatio  = p_init_user_param->jpg_ScaleRatio;

      pInst->codec_format = p_init_param->m_iBitstreamFormat;
      ret = vpu_env_open(p_init_param->m_iBitstreamFormat, p_init_user_param->bitrate_mbps,
                         p_init_user_param->frame_rate, p_init_param->m_iPicWidth,
                         p_init_param->m_iPicHeight, pInst);
      if(ret < 0){
        return -VPU_ENV_INIT_ERROR;
      }

      pInst->gsVpuDecInit_Info.gsVpuDecInit.m_RegBaseVirtualAddr = (uint32_t)NULL;
      pInst->gsVpuDecInit_Info.gsVpuDecInit.m_iBitstreamFormat  = p_init_param->m_iBitstreamFormat;
      pInst->gsVpuDecInit_Info.gsVpuDecInit.m_iPicWidth   = p_init_param->m_iPicWidth;
      pInst->gsVpuDecInit_Info.gsVpuDecInit.m_iPicHeight    = p_init_param->m_iPicHeight;
      pInst->gsVpuDecInit_Info.gsVpuDecInit.m_bEnableUserData   = p_init_param->m_bEnableUserData;
  #ifdef SUPPORT_VCACHE_CTRL
      pInst->gsVpuDecInit_Info.gsVpuDecInit.m_bEnableVideoCache = p_init_param->m_bEnableVideoCache;
  #endif
      pInst->gsVpuDecInit_Info.gsVpuDecInit.m_bCbCrInterleaveMode = p_init_param->m_bCbCrInterleaveMode;
      pInst->gsVpuDecInit_Info.gsVpuDecInit.m_Memcpy    = NULL; // No need to set!!
      pInst->gsVpuDecInit_Info.gsVpuDecInit.m_Memset    = NULL; // No need to set!!
      pInst->gsVpuDecInit_Info.gsVpuDecInit.m_Interrupt   = NULL; // No need to set!!

      pInst->extFunction = p_init_user_param->extFunction;
  #ifdef SET_FRAMEBUFFER_INTO_MAX
      if( (pInst->extFunction & EXT_FUNC_MAX_FRAMEBUFFER) != 0x0 )
      {
        pInst->gsVpuDecInit_Info.gsVpuDecInit.m_uiDecOptFlags |= (1<<16);
        DSTATUS("[VDEC-%d]Set framebuffer into 1080p", pInst->vdec_instance_index);
      }
  #endif

      pInst->gsVpuDecInit_Info.gsVpuDecInit.m_iFilePlayEnable   = p_init_param->m_bFilePlayEnable;
      pInst->gsbHasSeqHeader = 0;//p_init_param->m_bHasSeqHeader;

      pInst->gsVpuDecInit_Info.gsVpuDecInit.m_iBitstreamBufSize = p_init_param->m_iBitstreamBufSize;
      ret = vpu_dec_ready( &pInst->gsVpuDecInit_Info.gsVpuDecInit, pInst );
      if( ret != RETCODE_SUCCESS )
      {
        return ret;
      }

      if( (pInst->extFunction & EXT_FUNC_NO_BUFFER_DELAY) != 0x0 )
      {
        LOGI("[VDEC_K] : No BufferDelay Mode....");
        pInst->gsVpuDecInit_Info.gsVpuDecInit.m_uiDecOptFlags |= (1<<2);
      }

  #if REMOVE_SECURE_COPY
      pInst->gsVpuDecInit_Info.gsVpuDecInit.m_uiDecOptFlags |= (1<<26);
  #endif

      DSTATUS("[VDEC-%d]workbuff 0x%lx/0x%lx, Reg: 0x%lx, format : %d, Stream(0x%lx/0x%lx, %d), Res: %d x %d", pInst->vdec_instance_index,
            pInst->gsVpuDecInit_Info.gsVpuDecInit.m_BitWorkAddr[PA],
            pInst->gsVpuDecInit_Info.gsVpuDecInit.m_BitWorkAddr[VA],
            pInst->gsVpuDecInit_Info.gsVpuDecInit.m_RegBaseVirtualAddr,
            pInst->gsVpuDecInit_Info.gsVpuDecInit.m_iBitstreamFormat,
            pInst->gsVpuDecInit_Info.gsVpuDecInit.m_BitstreamBufAddr[PA],
            pInst->gsVpuDecInit_Info.gsVpuDecInit.m_BitstreamBufAddr[VA],
            pInst->gsVpuDecInit_Info.gsVpuDecInit.m_iBitstreamBufSize,
            pInst->gsVpuDecInit_Info.gsVpuDecInit.m_iPicWidth,
            pInst->gsVpuDecInit_Info.gsVpuDecInit.m_iPicHeight);
      DSTATUS("[VDEC-%d]optFlag 0x%x, avcBuff: 0x%x- %d, Userdata(%d), VCache: %d, Inter: %d, PlayEn: %d, MaxRes: %d", pInst->vdec_instance_index,
            pInst->gsVpuDecInit_Info.gsVpuDecInit.m_uiDecOptFlags,
            (uint32_t)pInst->gsVpuDecInit_Info.gsVpuDecInit.m_pSpsPpsSaveBuffer,
            pInst->gsVpuDecInit_Info.gsVpuDecInit.m_iSpsPpsSaveBufferSize,
            pInst->gsVpuDecInit_Info.gsVpuDecInit.m_bEnableUserData,
            0, //SUPPORT_VCACHE_CTRL
            pInst->gsVpuDecInit_Info.gsVpuDecInit.m_bCbCrInterleaveMode,
            pInst->gsVpuDecInit_Info.gsVpuDecInit.m_iFilePlayEnable,
            pInst->gsVpuDecInit_Info.gsVpuDecInit.m_iMaxResolution); 
      DSTATUS("[VDEC-%d]Format : %d, Stream(0x%lx, %d)", pInst->vdec_instance_index,
            pInst->gsVpuDecInit_Info.gsVpuDecInit.m_iBitstreamFormat,
            pInst->gsVpuDecInit_Info.gsVpuDecInit.m_BitstreamBufAddr[PA],
            pInst->gsVpuDecInit_Info.gsVpuDecInit.m_iBitstreamBufSize);

      ret = vdec_cmd_process(V_DEC_INIT, &pInst->gsVpuDecInit_Info, pInst);
      if( ret != RETCODE_SUCCESS )
      {
        DPRINTF( "[VDEC-%d] VPU_DEC_INIT failed Error code is 0x%x ", pInst->vdec_instance_index, ret );
        return -ret;
      }

      if( (pInst->extFunction & (EXT_FUNC_MEM_PROTECTION_WITH_INSTREAM | EXT_FUNC_MEM_PROTECTION_ONLY)) == 0x0 )
      {
        pInst->gsVpuDecVersion.pszVersion = (char*)(pInst->gsBitstreamBufAddr[K_VA] + (pInst->gsBitstreamBufSize - 100));
        pInst->gsVpuDecVersion.pszBuildData = (char*)(pInst->gsBitstreamBufAddr[K_VA] + (pInst->gsBitstreamBufSize - 50));

        ret = vdec_cmd_process(V_GET_VPU_VERSION, &pInst->gsVpuDecVersion, pInst);
        if( ret != RETCODE_SUCCESS )
        {
        //If this operation returns fail, it doesn't mean that there's a problem in vpu
        //so do not return error to host.
          DPRINTF( "[VDEC-%d] V_GET_VPU_VERSION failed Error code is 0x%x ", pInst->vdec_instance_index, ret );
        }
        else
        {
          int vpu_closed[VPU_MAX*2];
          if( 0 <= ioctl(pInst->mgr_fd, VPU_CHECK_CODEC_STATUS, &vpu_closed) ) {
            LOGD("[VDEC-%d] Multi-instance status : %d/%d/%d/%d/%d", pInst->vdec_instance_index,
                  vpu_closed[VPU_DEC], vpu_closed[VPU_DEC_EXT], vpu_closed[VPU_DEC_EXT2],
                  vpu_closed[VPU_DEC_EXT3], vpu_closed[VPU_ENC]);
          }
          LOGI( "[VDEC-%d] V_GET_VPU_VERSION OK. Version is %.27s, and it's built at %.10s ", pInst->vdec_instance_index,
            (char*)(pInst->gsBitstreamBufAddr[VA] + (pInst->gsBitstreamBufSize - 100)),
            (char*)(pInst->gsBitstreamBufAddr[VA] + (pInst->gsBitstreamBufSize - 50)));
        }
      }

      pInst->vdec_codec_opened = 1;
  #ifdef DISPLAY_1ST_DECODED_IDX
      pInst->mdisplayed_1st_IFrm = 0;
  #endif
      LOGI( "[VDEC-%d] VPU_DEC_INIT OK( has seq = %d) ", pInst->vdec_instance_index, pInst->gsbHasSeqHeader );
    }
    break;
    case VDEC_DEC_SEQ_HEADER:
    {
      vdec_input_t* p_input_param = (vdec_input_t*)pParam1;
      vdec_output_t* p_output_param = (vdec_output_t*)pParam2;
      int seq_stream_size = (p_input_param->m_iInpLen > pInst->gsMaxBitstreamSize) ? pInst->gsMaxBitstreamSize : p_input_param->m_iInpLen;
      uint32_t iIsThumbnail = p_input_param->m_iIsThumbnail;

      if(pInst->gsVpuDecInit_Info.gsVpuDecInit.m_iFilePlayEnable != 0)
      {
        if (  ((codec_addr_t)p_input_param->m_pInp[PA] == pInst->gsBitstreamBufAddr[PA])
           && ((codec_addr_t)p_input_param->m_pInp[VA] == pInst->gsBitstreamBufAddr[VA]) )
        {
          pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_BitstreamDataAddr[PA] = pInst->gsBitstreamBufAddr[PA];
          pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_BitstreamDataAddr[VA] = pInst->gsBitstreamBufAddr[K_VA];
        }
        else
        {
          pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_BitstreamDataAddr[PA] = pInst->gsBitstreamBufAddr[PA];
          pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_BitstreamDataAddr[VA] = pInst->gsBitstreamBufAddr[K_VA];
   #if defined(TC_SECURE_MEMORY_COPY)
          if (pInst->extFunction & EXT_FUNC_MEM_PROTECTION_WITH_INSTREAM) {
        #if REMOVE_SECURE_COPY
            pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_iBitstreamDataSize = seq_stream_size;
            pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_BitstreamDataAddr[PA] = (codec_addr_t)p_input_param->m_pInp[PA];
            pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_BitstreamDataAddr[VA] = (codec_addr_t)p_input_param->m_pInp[VA];
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
          DSTATUS( "[VDEC-%d Seq %d] " "0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x " "0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x "
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

      DSTATUS( "[VDEC-%d] VDEC_DEC_SEQ_HEADER start  :: len = %d / %d ", pInst->vdec_instance_index, seq_stream_size, p_input_param->m_iInpLen);
      ret = vpu_dec_seq_header(seq_stream_size, iIsThumbnail, pInst);
      if( ret != RETCODE_SUCCESS )
      {
        p_output_param->m_pInitialInfo = &pInst->gsCommDecInfo; // mm008
        return ret;
      }
    #ifdef VPU_ALL_FRAME_DUMP
      save_input_stream("seqHeader_stream.bin", seq_stream_size, pInst, 0);
    #endif
      pInst->gsbHasSeqHeader = 1;
      p_output_param->m_pInitialInfo = &pInst->gsCommDecInfo;
      //check the maximum/minimum video resolution limitation
      if( pInst->gsVpuDecInit_Info.gsVpuDecInit.m_iBitstreamFormat != STD_MJPG )
      {
      #if REMOVE_SECURE_COPY
        vdec_info_t * pVdecInfo = (vdec_info_t *)&pInst->gsVpuDecInOut_Info.gsVpuDecInitialInfo;
      #else
        vdec_info_t * pVdecInfo = (vdec_info_t *)&pInst->gsVpuDecSeqHeader_Info.gsVpuDecInitialInfo;
      #endif
        int max_width;
        int max_height;
        int min_width;
        int min_height;

        max_width   = ((AVAILABLE_MAX_WIDTH+15)&0xFFF0);
        max_height  = ((AVAILABLE_MAX_HEIGHT+15)&0xFFF0);
        min_width   = AVAILABLE_MIN_WIDTH;
        min_height  = AVAILABLE_MIN_HEIGHT;

        if( (pVdecInfo->m_iPicWidth > max_width)
        || ((pVdecInfo->m_iPicWidth * pVdecInfo->m_iPicHeight) > AVAILABLE_MAX_REGION)
        || (pVdecInfo->m_iPicWidth < min_width)
        || (pVdecInfo->m_iPicHeight < min_height) )
        {
          ret = 0 - RETCODE_INVALID_STRIDE;
          DPRINTF( "[VDEC-%d] VDEC_DEC_SEQ_HEADER - don't support the resolution %dx%d  ", pInst->vdec_instance_index,
              pVdecInfo->m_iPicWidth, pVdecInfo->m_iPicHeight);
          return ret;
        }
      }

    #ifdef DISPLAY_1ST_DECODED_IDX
      if( iIsThumbnail ){
        pInst->mdisplayed_1st_IFrm = 1;
      }
    #endif
      LOGI( "[VDEC-%d] VDEC_DEC_SEQ_HEADER - Success mem_free = 0x%x ", pInst->vdec_instance_index, cdk_sys_final_free_mem(pInst) );
      DSTATUS( "[VDEC-%d] =======================================================", pInst->vdec_instance_index );
    }
    break;
    case VDEC_DECODE:
    {
      vdec_input_t* p_input_param = (vdec_input_t*)pParam1;
      vdec_output_t* p_output_param = (vdec_output_t*)pParam2;

      pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_iBitstreamDataSize =
          (p_input_param->m_iInpLen > pInst->gsMaxBitstreamSize) ? pInst->gsMaxBitstreamSize : p_input_param->m_iInpLen;

  #if defined(TC_SECURE_MEMORY_COPY)
      if( (pInst->extFunction & EXT_FUNC_MEM_PROTECTION_WITH_INSTREAM) != 0x0 )
      {
        //LOGI("Usable Input Addr : 0x%x", p_input_param->m_pInp[PA]);
  #if REMOVE_SECURE_COPY
        pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_BitstreamDataAddr[PA] = (codec_addr_t)p_input_param->m_pInp[PA];
        pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_BitstreamDataAddr[VA] = (codec_addr_t)p_input_param->m_pInp[VA];
  #else
        TC_SecureMemoryCopy(pInst->gsBitstreamBufAddr[PA], p_input_param->m_pInp[PA], pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_iBitstreamDataSize);
  #endif
      }
      else
  #endif
      {
        if( pInst->gsVpuDecInit_Info.gsVpuDecInit.m_iFilePlayEnable != 0 )
        {
          if (((codec_addr_t)p_input_param->m_pInp[PA] == pInst->gsBitstreamBufAddr[PA])
           && ((codec_addr_t)p_input_param->m_pInp[VA] == pInst->gsBitstreamBufAddr[VA]))
          {
            pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_BitstreamDataAddr[PA] = pInst->gsBitstreamBufAddr[PA];
            pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_BitstreamDataAddr[VA] = pInst->gsBitstreamBufAddr[K_VA];
          }
          else
          {
            pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_BitstreamDataAddr[PA] = pInst->gsBitstreamBufAddr[PA];
            pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_BitstreamDataAddr[VA] = pInst->gsBitstreamBufAddr[K_VA];
            (void)memcpy( (unsigned long*)pInst->gsBitstreamBufAddr[VA], (unsigned long*)p_input_param->m_pInp[VA]
                   , pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_iBitstreamDataSize);
  #ifdef CHANGE_INPUT_STREAM
            change_input_stream((unsigned char *)pInst->gsBitstreamBufAddr[VA], (&pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_iBitstreamDataSize), iOpCode, pInst);
  #endif
          }
        }
        else
        {
          pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_BitstreamDataAddr[PA] = pInst->gsBitstreamBufAddr[PA];
          pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_BitstreamDataAddr[VA] = pInst->gsBitstreamBufAddr[K_VA];
          pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_iBitstreamDataSize = 1;
        }
      }

      if(pInst->gsVpuDecInit_Info.gsVpuDecInit.m_bEnableUserData)
      {
        pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_UserDataAddr[PA] = pInst->gsUserdataBufAddr[PA];
        pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_UserDataAddr[VA] = pInst->gsUserdataBufAddr[K_VA];
        pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_iUserDataBufferSize = pInst->gsUserdataBufSize;
      }

      pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_iSkipFrameMode = p_input_param->m_iSkipFrameMode;
      pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_iFrameSearchEnable = p_input_param->m_iFrameSearchEnable;
      pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_iSkipFrameNum = 0;
      if( (pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_iSkipFrameMode > 0) || (pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_iFrameSearchEnable > 0) )
      {
        pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_iSkipFrameNum = p_input_param->m_iSkipFrameNum;
      }

      #ifdef VPU_ALL_FRAME_DUMP
      save_input_stream("all_stream.bin", pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_iBitstreamDataSize, pInst, 1);
      #endif

      // Start decoding a frame.
      ret = vdec_cmd_process(V_DEC_DECODE, &pInst->gsVpuDecInOut_Info, pInst);
      pInst->total_frm++;

      if((pInst->gsVpuDecInOut_Info.gsVpuDecOutput.m_DecOutInfo.m_iDecodingStatus == VPU_DEC_BUF_FULL)
        || ((pInst->gsVpuDecInOut_Info.gsVpuDecOutput.m_DecOutInfo.m_iDecodingStatus != VPU_DEC_SUCCESS_FIELD_PICTURE)
        && (pInst->gsVpuDecInOut_Info.gsVpuDecOutput.m_DecOutInfo.m_iWidth <= 64 || pInst->gsVpuDecInOut_Info.gsVpuDecOutput.m_DecOutInfo.m_iHeight <= 64))
        || (ret == RETCODE_CODEC_EXIT)
      )
      {
        if(pInst->gsVpuDecInOut_Info.gsVpuDecOutput.m_DecOutInfo.m_iDecodingStatus == VPU_DEC_BUF_FULL){
          LOGE("Buffer full");
        }
        else if (ret == RETCODE_CODEC_EXIT){
          LOGE("Codec Exit");
        }
        else{
          LOGE("Strange resolution");
        }
        LOGE("Dec In 0x%lx - 0x%lx, %d, 0x%lx - 0x%lx, %d, flsg: %d / %d / %d  \n",
            pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_BitstreamDataAddr[PA],
            pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_BitstreamDataAddr[VA],
            pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_iBitstreamDataSize,
            pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_UserDataAddr[PA],
            pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_UserDataAddr[VA],
            pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_iUserDataBufferSize,
            pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_iFrameSearchEnable,
            pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_iSkipFrameMode,
            pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_iSkipFrameNum);

        LOGE("%d - %d - %d, %d - %d - %d \n",
            pInst->gsVpuDecInOut_Info.gsVpuDecOutput.m_DecOutInfo.m_iWidth,
            pInst->gsVpuDecInOut_Info.gsVpuDecOutput.m_DecOutInfo.m_CropInfo.m_iCropLeft,
            pInst->gsVpuDecInOut_Info.gsVpuDecOutput.m_DecOutInfo.m_CropInfo.m_iCropRight,
            pInst->gsVpuDecInOut_Info.gsVpuDecOutput.m_DecOutInfo.m_iHeight,
            pInst->gsVpuDecInOut_Info.gsVpuDecOutput.m_DecOutInfo.m_CropInfo.m_iCropTop,
            pInst->gsVpuDecInOut_Info.gsVpuDecOutput.m_DecOutInfo.m_CropInfo.m_iCropBottom);

        LOGE("@@ Dec Out[%d] !! PicType[%d], OutIdx[%d/%d], OutStatus[%d/%d] \n", ret,
            pInst->gsVpuDecInOut_Info.gsVpuDecOutput.m_DecOutInfo.m_iPicType,
            pInst->gsVpuDecInOut_Info.gsVpuDecOutput.m_DecOutInfo.m_iDispOutIdx,
            pInst->gsVpuDecInOut_Info.gsVpuDecOutput.m_DecOutInfo.m_iDecodedIdx,
            pInst->gsVpuDecInOut_Info.gsVpuDecOutput.m_DecOutInfo.m_iOutputStatus,
            pInst->gsVpuDecInOut_Info.gsVpuDecOutput.m_DecOutInfo.m_iDecodingStatus);
        LOGE("DispOutIdx : %d \n", pInst->gsVpuDecInOut_Info.gsVpuDecOutput.m_DecOutInfo.m_iDispOutIdx);

        if(ret == RETCODE_CODEC_EXIT) {
          save_input_stream("error_codec_exit.bin", pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_iBitstreamDataSize, pInst, 0);
        }
        else if(pInst->gsVpuDecInOut_Info.gsVpuDecOutput.m_DecOutInfo.m_iDecodingStatus == VPU_DEC_BUF_FULL){
          save_input_stream("error_buffer_full.bin", pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_iBitstreamDataSize, pInst, 0);
        }
        else {
          save_input_stream("error_strange_res.bin", pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_iBitstreamDataSize, pInst, 0);
        }
      }

      if( ret != RETCODE_SUCCESS )
      {
        DPRINTF( "[VDEC-%d] VPU_DEC_DECODE failed Error code is 0x%x ", pInst->vdec_instance_index, ret );
        return -ret;
      }

      if( pInst->gsVpuDecInOut_Info.gsVpuDecOutput.m_DecOutInfo.m_iPicType == 0 ){
  #ifdef DISPLAY_1ST_DECODED_IDX
        if( pInst->gsVpuDecInOut_Info.gsVpuDecOutput.m_DecOutInfo.m_iInterlacedFrame )
        pInst->mdisplayed_1st_IFrm = 1;

        if( pInst->mdisplayed_1st_IFrm == 0 && pInst->gsVpuDecInOut_Info.gsVpuDecOutput.m_DecOutInfo.m_iDecodedIdx >= 0){
        DSTATUS( "[VDEC-%d] mdisplayed_1st_IFrm (%d)", pInst->vdec_instance_index, pInst->total_frm);
        pInst->mdisplayed_1st_IFrm = 1;
        pInst->gsVpuDecInOut_Info.gsVpuDecOutput.m_DecOutInfo.m_iOutputStatus = VPU_DEC_OUTPUT_SUCCESS;
        pInst->gsVpuDecInOut_Info.gsVpuDecOutput.m_DecOutInfo.m_iDispOutIdx = MAX_INDEX-1;
        pInst->gsVpuDecInOut_Info.gsVpuDecOutput.m_pDispOut[PA][COMP_Y] = pInst->gsVpuDecInOut_Info.gsVpuDecOutput.m_pCurrOut[PA][COMP_Y];
        pInst->gsVpuDecInOut_Info.gsVpuDecOutput.m_pDispOut[PA][COMP_U] = pInst->gsVpuDecInOut_Info.gsVpuDecOutput.m_pCurrOut[PA][COMP_U];
        pInst->gsVpuDecInOut_Info.gsVpuDecOutput.m_pDispOut[PA][COMP_V] = pInst->gsVpuDecInOut_Info.gsVpuDecOutput.m_pCurrOut[PA][COMP_V];
        pInst->gsVpuDecInOut_Info.gsVpuDecOutput.m_pDispOut[VA][COMP_Y] = pInst->gsVpuDecInOut_Info.gsVpuDecOutput.m_pCurrOut[VA][COMP_Y];
        pInst->gsVpuDecInOut_Info.gsVpuDecOutput.m_pDispOut[VA][COMP_U] = pInst->gsVpuDecInOut_Info.gsVpuDecOutput.m_pCurrOut[VA][COMP_U];
        pInst->gsVpuDecInOut_Info.gsVpuDecOutput.m_pDispOut[VA][COMP_V] = pInst->gsVpuDecInOut_Info.gsVpuDecOutput.m_pCurrOut[VA][COMP_V];
        }
  #endif
        DSTATUS( "[VDEC-%d] I-Frame (%d)", pInst->vdec_instance_index, pInst->total_frm);
      }

      (void)memcpy((unsigned long*)p_output_param, (unsigned long*)&pInst->gsVpuDecInOut_Info.gsVpuDecOutput, sizeof(dec_output_t ) );

      p_output_param->m_pInitialInfo = &pInst->gsCommDecInfo;

      if( (pInst->gsVpuDecInit_Info.gsVpuDecInit.m_iBitstreamFormat != STD_MJPG) || (!pInst->gsUserInfo.m_bStillJpeg))
      {
        p_output_param->m_pDispOut[VA][COMP_Y] = (unsigned char *)vpu_getFrameBufVirtAddr(p_output_param->m_pDispOut[VA][COMP_Y], K_VA, pInst);
        p_output_param->m_pDispOut[VA][COMP_U] = (unsigned char *)vpu_getFrameBufVirtAddr(p_output_param->m_pDispOut[VA][COMP_U], K_VA, pInst);
        p_output_param->m_pDispOut[VA][COMP_V] = (unsigned char *)vpu_getFrameBufVirtAddr(p_output_param->m_pDispOut[VA][COMP_V], K_VA, pInst);
      }

      p_output_param->m_pCurrOut[VA][COMP_Y] = (unsigned char *)vpu_getFrameBufVirtAddr(p_output_param->m_pCurrOut[VA][COMP_Y], K_VA, pInst);
      p_output_param->m_pCurrOut[VA][COMP_U] = (unsigned char *)vpu_getFrameBufVirtAddr(p_output_param->m_pCurrOut[VA][COMP_U], K_VA, pInst);
      p_output_param->m_pCurrOut[VA][COMP_V] = (unsigned char *)vpu_getFrameBufVirtAddr(p_output_param->m_pCurrOut[VA][COMP_V], K_VA, pInst);

      if(pInst->gsVpuDecInit_Info.gsVpuDecInit.m_bEnableUserData)
      {
        uint32_t addr_gap = 0;
        addr_gap = pInst->gsUserdataBufAddr[K_VA] - pInst->gsVpuDecInOut_Info.gsVpuDecOutput.m_DecOutInfo.m_UserDataAddress[VA];
        p_output_param->m_DecOutInfo.m_UserDataAddress[VA] = pInst->gsUserdataBufAddr[VA] + addr_gap;
      }

      if(pInst->gsVpuDecInOut_Info.gsVpuDecOutput.m_DecOutInfo.m_iOutputStatus == VPU_DEC_OUTPUT_SUCCESS){
  //    LOGE("Displayed addr 0x%x", p_output_param->m_pDispOut[PA][0]);
  #ifdef VPU_OUT_FRAME_DUMP
        save_decoded_frame((unsigned char *)vpu_getFrameBufVirtAddr(p_output_param->m_pDispOut[PA][0], PA, pInst),
          (unsigned char *)vpu_getFrameBufVirtAddr(p_output_param->m_pDispOut[PA][1], PA, pInst),
          (unsigned char *)vpu_getFrameBufVirtAddr(p_output_param->m_pDispOut[PA][2], PA, pInst),
          pInst->gsVpuDecSeqHeader_Info.gsVpuDecInitialInfo.m_iPicWidth, pInst->gsVpuDecSeqHeader_Info.gsVpuDecInitialInfo.m_iPicHeight, pInst);
  #endif
      }

      DISPLAY_BUFFER("[VDEC-%d] Display idx = %d", pInst->vdec_instance_index, pInst->gsVpuDecInOut_Info.gsVpuDecOutput.m_DecOutInfo.m_iDispOutIdx);
    }
    break;
    case VDEC_GET_RING_BUFFER_STATUS:
    {
      vdec_ring_buffer_out_t* p_out_param = (vdec_ring_buffer_out_t*)pParam2;

      ret = vdec_cmd_process(V_GET_RING_BUFFER_STATUS, &pInst->gsVpuDecBufStatus, pInst); // get the available space in the ring buffer
      if( ret != RETCODE_SUCCESS )
      {
        DPRINTF( "[VDEC-%d] GET_RING_BUFFER_STATUS failed Error code is 0x%x ", pInst->vdec_instance_index, ret );
        return -ret;
      }

      p_out_param->m_ulAvailableSpaceInRingBuffer = pInst->gsVpuDecBufStatus.gsVpuDecRingStatus.m_ulAvailableSpaceInRingBuffer;
      p_out_param->m_ptrReadAddr_PA = pInst->gsVpuDecBufStatus.gsVpuDecRingStatus.m_ptrReadAddr_PA;
      p_out_param->m_ptrWriteAddr_PA = pInst->gsVpuDecBufStatus.gsVpuDecRingStatus.m_ptrWriteAddr_PA;
    //  LOGE("[VDEC] [AVAIL: %8d] [RP: 0x%08X / WP: 0x%08X]"
    //    , pInst->gsVpuDecBufStatus.gsVpuDecRingStatus.m_ulAvailableSpaceInRingBuffer
    //    , pInst->gsVpuDecBufStatus.gsVpuDecRingStatus.m_ptrReadAddr_PA
    //    , pInst->gsVpuDecBufStatus.gsVpuDecRingStatus.m_ptrWriteAddr_PA
    //    );
    }
    break;
    case VDEC_FILL_RING_BUFFER:
    {
      vdec_ring_buffer_set_t* p_set_param = (vdec_ring_buffer_set_t*)pParam1;
  
      (void)memcpy((unsigned long*)pInst->gsIntermediateBufAddr[VA],(unsigned long*)p_set_param->m_pbyBuffer, p_set_param->m_uiBufferSize);
      pInst->gsVpuDecBufFill.gsVpuDecRingFeed.m_iOnePacketBufferSize = p_set_param->m_uiBufferSize;
      pInst->gsVpuDecBufFill.gsVpuDecRingFeed.m_OnePacketBufferAddr = pInst->gsIntermediateBufAddr[K_VA];

      ret = vdec_cmd_process(V_FILL_RING_BUFFER_AUTO, &pInst->gsVpuDecBufFill, pInst);  // fille the Ring Buffer

      if( ret != RETCODE_SUCCESS )
      {
        DPRINTF( "[VDEC-%d] FILL_RING_BUFFER_AUTO failed Error code is 0x%x ", pInst->vdec_instance_index, ret );
        return -ret;
      }
    }
    break;
    case VDEC_GET_INTERMEDIATE_BUF_INFO:
    {
      *(uint32_t*)pParam1 = pInst->gsIntermediateBufAddr[VA];
      *(uint32_t*)pParam2 = pInst->gsIntermediateBufSize;
      ret = 0;
    }
    break;
    case VDEC_UPDATE_WRITE_BUFFER_PTR:
    {
      pInst->gsVpuDecUpdateWP.iCopiedSize = (int)pParam1;
      pInst->gsVpuDecUpdateWP.iFlushBuf = (int)pParam2;

      ret = vdec_cmd_process(V_DEC_UPDATE_RINGBUF_WP, &pInst->gsVpuDecUpdateWP,  pInst);  // fille the Ring Buffer

      if( ret != RETCODE_SUCCESS )
      {
        DPRINTF( "[VDEC-%d] VDEC_UPDATE_WRITE_BUFFER_PTR failed Error code is 0x%x ", pInst->vdec_instance_index, ret );
        ret =-ret;
      }
    }
    break;
    case VDEC_BUF_FLAG_CLEAR:
    {
      unsigned long idx_display = *(unsigned long*)pParam1;
      CLEAR_BUFFER("[VDEC-%d] ************* cleared idx = %ld", pInst->vdec_instance_index, idx_display);
  #ifdef DISPLAY_1ST_DECODED_IDX
      if( idx_display == (MAX_INDEX-1) ){
        return RETCODE_SUCCESS;
      }
  #endif
      ret = vdec_cmd_process(V_DEC_BUF_FLAG_CLEAR, &idx_display, pInst);

      if( ret != RETCODE_SUCCESS )
      {
        DPRINTF( "[VDEC-%d] VPU_DEC_BUF_FLAG_CLEAR failed Error code is 0x%x ", pInst->vdec_instance_index, ret );
        ret = -ret;
      }
    }
    break;
    case VDEC_DEC_FLUSH_OUTPUT:
    {
      vdec_input_t* p_input_param = (vdec_input_t*)pParam1;
      vdec_output_t* p_output_param = (vdec_output_t*)pParam2;

      pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_BitstreamDataAddr[PA] = pInst->gsBitstreamBufAddr[PA];
      pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_BitstreamDataAddr[VA] = pInst->gsBitstreamBufAddr[K_VA];
      pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_iBitstreamDataSize = 0;
      pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_iSkipFrameMode = VDEC_SKIP_FRAME_DISABLE;
      pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_iFrameSearchEnable = 0;
      pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_iSkipFrameNum = 0;

      ret = vdec_cmd_process(V_DEC_FLUSH_OUTPUT, &pInst->gsVpuDecInOut_Info, pInst);

      if( ret != RETCODE_SUCCESS )
      {
        DPRINTF( "[VDEC-%d] VDEC_DEC_FLUSH_OUTPUT failed Error code is 0x%x ", pInst->vdec_instance_index, ret );
        return -ret;
      }

      (void)memcpy((unsigned long*)p_output_param, (unsigned long*)&pInst->gsVpuDecInOut_Info.gsVpuDecOutput, sizeof(dec_output_t ) );
      p_output_param->m_pInitialInfo = &pInst->gsCommDecInfo;

      if( pInst->gsVpuDecInit_Info.gsVpuDecInit.m_iBitstreamFormat != STD_MJPG || !pInst->gsUserInfo.m_bStillJpeg)
      {
        p_output_param->m_pDispOut[VA][COMP_Y] = (unsigned char *)vpu_getFrameBufVirtAddr(p_output_param->m_pDispOut[VA][COMP_Y], K_VA, pInst);
        p_output_param->m_pDispOut[VA][COMP_U] = (unsigned char *)vpu_getFrameBufVirtAddr(p_output_param->m_pDispOut[VA][COMP_U], K_VA, pInst);
        p_output_param->m_pDispOut[VA][COMP_V] = (unsigned char *)vpu_getFrameBufVirtAddr(p_output_param->m_pDispOut[VA][COMP_V], K_VA, pInst);
      }

      p_output_param->m_pCurrOut[VA][COMP_Y] = (unsigned char *)vpu_getFrameBufVirtAddr(p_output_param->m_pCurrOut[VA][COMP_Y], K_VA, pInst);
      p_output_param->m_pCurrOut[VA][COMP_U] = (unsigned char *)vpu_getFrameBufVirtAddr(p_output_param->m_pCurrOut[VA][COMP_U], K_VA, pInst);
      p_output_param->m_pCurrOut[VA][COMP_V] = (unsigned char *)vpu_getFrameBufVirtAddr(p_output_param->m_pCurrOut[VA][COMP_V], K_VA, pInst);

      if(pInst->gsVpuDecInit_Info.gsVpuDecInit.m_bEnableUserData)
      {
        uint32_t addr_gap = 0;

        addr_gap = pInst->gsUserdataBufAddr[K_VA] - pInst->gsVpuDecInOut_Info.gsVpuDecOutput.m_DecOutInfo.m_UserDataAddress[VA];
        p_output_param->m_DecOutInfo.m_UserDataAddress[VA] = pInst->gsUserdataBufAddr[VA] + addr_gap;
      }
    }
    break;
    case VDEC_SW_RESET:
    {
      ret = vdec_cmd_process(V_DEC_SWRESET, NULL, pInst);

      if( ret != RETCODE_SUCCESS )
      {
        DPRINTF( "[VDEC-%d] V_DEC_SWRESET failed Error code is 0x%x ", pInst->vdec_instance_index, ret );
        ret = -ret;
      }
    }
    break;
    case VDEC_CLOSE:
    {
      if(pInst->vdec_codec_opened)
      {
        ret = vdec_cmd_process(V_DEC_CLOSE, &pInst->gsVpuDecInOut_Info, pInst);
        if( ret != RETCODE_SUCCESS )
        {
          DPRINTF( "[VDEC-%d] VPU_DEC_CLOSE failed Error code is 0x%x ", pInst->vdec_instance_index, ret );
          ret = -ret;
        }

        pInst->vdec_codec_opened = 0;
      }

      if(!pInst->vdec_env_opened){
        ret = -RETCODE_NOT_INITIALIZED;
      }
      else {
        cdk_sys_free_virtual_addr( (unsigned long*)pInst->gsBitstreamBufAddr[VA], pInst->gsBitstreamBufSize);
        cdk_sys_free_virtual_addr( (unsigned long*)pInst->gsUserdataBufAddr[VA], pInst->gsUserdataBufSize );
        cdk_sys_free_virtual_addr( (unsigned long*)pInst->gsBitWorkBufAddr[VA], pInst->gsBitWorkBufSize );
        cdk_sys_free_virtual_addr( (unsigned long*)pInst->gsFrameBufAddr[VA], pInst->gsFrameBufSize );

        vpu_env_close(pInst);
      }
    }
    break;
    default:
    {
      DPRINTF( "[VDEC-%d] Invalid Operation!!", pInst->vdec_instance_index );
      ret = -ret;
    }
    break;
  }
  return ret;
}

#endif //TCC_VPU_INCLUDE
