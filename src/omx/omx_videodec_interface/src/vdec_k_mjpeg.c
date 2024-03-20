// SPDX-License-Identifier: LGPL-2.1-or later
/****************************************************************************
 *   FileName    : vdec_k_mjpeg.c
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

#ifdef TCC_JPU_INCLUDE

#define LOG_TAG "VPU_DEC_K_MJPEG"

#include "TCCMemory.h"

#include <sys/mman.h>
#include <errno.h>
#include <sys/ioctl.h>
#if defined(USE_COMMON_KERNEL_LOCATION)
#include <tcc_vpu_ioctl.h>
#include <tcc_jpu_ioctl.h>
#else //use chipset folder
#include <mach/tcc_vpu_ioctl.h>
#include <mach/tcc_jpu_ioctl.h>
#endif

#include <dlfcn.h>

#include <memory.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>         // O_RDWR
#include <sys/poll.h>

static int jpu_cmd_process(int cmd, unsigned long* args, _vdec_ *pVdec)
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
      LOGE("JPU[%d] Invalid command(0x%x) ", pInst->vdec_instance_index, cmd);
      return RETCODE_INVALID_COMMAND;
    }
    else
    {
      LOGE("JPU[%d] ioctl err[%s] : cmd = 0x%x", pInst->vdec_instance_index, strerror(errno), cmd);
    }
  }

Retry:
  while (retry_cnt > 0) {
    memset(pInst->tcc_event, 0, sizeof(pInst->tcc_event));
    pInst->tcc_event[0].fd = pInst->dec_fd;
    pInst->tcc_event[0].events = POLLIN;

    ret = poll((struct pollfd *)&pInst->tcc_event, 1, 10000); // 1 sec
    if (ret < 0) {
      LOGE("JPU[%d]-retry(%d:cmd(%d)) poll error '%s'", pInst->vdec_instance_index, retry_cnt, cmd, strerror(errno));
      retry_cnt--;
      continue;
    }else if (ret == 0) {
      LOGE("JPU[%d]-retry(%d:cmd(%d)) poll timeout: %u'th frames, len %d", pInst->vdec_instance_index, retry_cnt, cmd, pInst->total_frm, pInst->gsVpuDecInOut_Info.gsVpuDecInput.m_iBitstreamDataSize );
      retry_cnt--;
      continue;
    }else if (ret > 0) {
      if (pInst->tcc_event[0].revents & POLLERR) {
        LOGE("JPU[%d] poll POLLERR", pInst->vdec_instance_index);
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
      JDEC_INIT_t* init_info = args;
      if(ioctl(pInst->dec_fd, V_DEC_INIT_RESULT, args) < 0){
        LOGE("JPU[%d] ioctl(0x%x) error[%s]!!", pInst->vdec_instance_index, V_DEC_INIT_RESULT, strerror(errno));
      }
      ret = init_info->result;
    }
    break;

    case V_DEC_SEQ_HEADER:
    {
      JDEC_SEQ_HEADER_t* seq_info = args;
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
      JPU_DECODE_t* decoded_info = args;

      if(ioctl(pInst->dec_fd, V_DEC_DECODE_RESULT, args) < 0){
        LOGE("JPU[%d] ioctl(0x%x) error[%s]!!", pInst->vdec_instance_index, V_DEC_DECODE_RESULT, strerror(errno));
      }
      ret = decoded_info->result;
    }
    break;
    case V_GET_VPU_VERSION:
    {
      JPU_GET_VERSION_t* p_param = args;
      if(ioctl(pInst->dec_fd, V_GET_VPU_VERSION_RESULT, args) < 0){
        LOGE("JPU[%d] ioctl(0x%x) error[%s]!!", pInst->vdec_instance_index, V_GET_VPU_VERSION_RESULT, strerror(errno));
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
        LOGE("JPU[%d] ioctl(0x%x) error[%s]!!", pInst->vdec_instance_index, V_DEC_GENERAL_RESULT, strerror(errno));
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

  if(!success
    || ((ret&0xf000) != 0x0000) /* vpu can not start or finish its processing with unknown reason!! */
  )
  {
    LOGE("JPU[%d] command(0x%x) didn't work properly. maybe hangup(no return(0x%x))!!", pInst->vdec_instance_index, cmd, ret);

    if((ret != RETCODE_CODEC_EXIT) && (ret != RETCODE_MULTI_CODEC_EXIT_TIMEOUT)){
//      ioctl(pInst->mgr_fd, VPU_HW_RESET, (void*)NULL);
    }

    return RETCODE_CODEC_EXIT;
  }

  return ret;
}

int32_t vdec_mjpeg_jpu(int32_t iOpCode, unsigned long* pHandle, unsigned long* pParam1, unsigned long* pParam2, unsigned long* pParam3)     // For TCC892XS / TCC893XS, use JPU as a MJPEG H/W decoder
{
  codec_result_t ret = 0;
  _vdec_ *pInst = (_vdec_ *)pParam3;

  if(!pInst){
    LOGE("vdec_mjpeg_jpu(OP:%d) :: Instance is null!!", iOpCode);
    return -RETCODE_NOT_INITIALIZED;
  }

  if( (iOpCode != VDEC_INIT) && (iOpCode != VDEC_CLOSE) && (!pInst->vdec_codec_opened) ){
    return -RETCODE_NOT_INITIALIZED;
  }

  if( iOpCode == VDEC_INIT )
  {
    //! Set Decoder Init
    DSTATUS("[JPU-%d] VDEC_INIT. vdec_mjpeg_jpu", pInst->vdec_instance_index);
    vdec_init_t* p_init_param = (vdec_init_t*)pParam1;
    vdec_user_info_t* p_init_user_param = (vdec_user_info_t*)pParam2;

    pInst->gsUserInfo.bitrate_mbps = p_init_user_param->bitrate_mbps;
    pInst->gsUserInfo.frame_rate   = p_init_user_param->frame_rate;
    pInst->gsUserInfo.m_bStillJpeg  = p_init_user_param->m_bStillJpeg;
    pInst->gsUserInfo.jpg_ScaleRatio  = p_init_user_param->jpg_ScaleRatio;

    pInst->codec_format = p_init_param->m_iBitstreamFormat;
    ret = vpu_env_open(p_init_param->m_iBitstreamFormat,
                       p_init_user_param->bitrate_mbps, p_init_user_param->frame_rate,
                       p_init_param->m_iPicWidth, p_init_param->m_iPicHeight, pInst);
    if(ret < 0)
    {
      LOGE("[JPU-%d] vpu_env_open error", pInst->vdec_instance_index);
      return -VPU_ENV_INIT_ERROR;
    }

    if(p_init_param->m_iBitstreamBufSize > LARGE_STREAM_BUF_SIZE){
      pInst->gsBitstreamBufSize = ALIGNED_BUFF( p_init_param->m_iBitstreamBufSize, 64*1024 );
    } else {
      pInst->gsBitstreamBufSize = LARGE_STREAM_BUF_SIZE;
    }
    pInst->gsBitstreamBufSize = ALIGNED_BUFF( pInst->gsBitstreamBufSize, ALIGN_LEN );
    pInst->gsMaxBitstreamSize = pInst->gsBitstreamBufSize;

    pInst->gsBitstreamBufAddr[PA] = (codec_addr_t)cdk_sys_malloc_physical_addr( &pInst->gsBitstreamBufAddr[K_VA], pInst->gsBitstreamBufSize, BUFFER_STREAM, pInst );
    if( pInst->gsBitstreamBufAddr[PA] == 0 )
    {
      LOGE( "[JPU-%d] bitstream_buf_addr[PA] malloc() failed ", pInst->vdec_instance_index);
      return -(VPU_NOT_ENOUGH_MEM);
    }
    DSTATUS("[JPU-%d] bitstream_buf_addr[PA] = 0x%lx, 0x%x ", pInst->vdec_instance_index,
          (codec_addr_t)pInst->gsBitstreamBufAddr[PA], pInst->gsBitstreamBufSize );

    pInst->gsBitstreamBufAddr[VA] = (codec_addr_t)cdk_sys_malloc_virtual_addr( pInst->gsBitstreamBufAddr[PA], pInst->gsBitstreamBufSize, pInst );
    if( pInst->gsBitstreamBufAddr[VA] == 0 )
    {
      LOGE( "[JPU-%d] bitstream_buf_addr[VA] malloc() failed ", pInst->vdec_instance_index);
      return -(VPU_NOT_ENOUGH_MEM);
    }

    if( (pInst->extFunction & (EXT_FUNC_MEM_PROTECTION_WITH_INSTREAM | EXT_FUNC_MEM_PROTECTION_ONLY)) == 0x0 )
    {
      memset( (unsigned long*)pInst->gsBitstreamBufAddr[VA], 0x00 , pInst->gsBitstreamBufSize);
    }

    DSTATUS("[JPU-%d] bitstream_buf_addr[VA] = 0x%lx, 0x%x ", pInst->vdec_instance_index,
        (codec_addr_t)pInst->gsBitstreamBufAddr[VA], pInst->gsBitstreamBufSize );

    pInst->gsJpuDecInit_Info.gsJpuDecInit.m_BitstreamBufAddr[PA] = (codec_addr_t)pInst->gsBitstreamBufAddr[PA];
    pInst->gsJpuDecInit_Info.gsJpuDecInit.m_BitstreamBufAddr[VA] = (codec_addr_t)pInst->gsBitstreamBufAddr[K_VA];

#if defined(TCC_JPU_C6_INCLUDE)
    pInst->gsJpuDecInit_Info.gsJpuDecInit.m_iBitstreamBufSize = pInst->gsBitstreamBufSize;
    pInst->gsJpuDecInit_Info.gsJpuDecInit.m_iCbCrInterleaveMode  = p_init_param->m_bCbCrInterleaveMode;
    pInst->gsJpuDecInit_Info.gsJpuDecInit.m_uiDecOptFlags = 0;
#else
    pInst->gsJpuDecInit_Info.gsJpuDecInit.m_bCbCrInterleaveMode  = p_init_param->m_bCbCrInterleaveMode;
#endif
    pInst->gsVpuDecInit.m_iBitstreamFormat  = p_init_param->m_iBitstreamFormat;

    pInst->gsFirstFrame = 1;
    pInst->gsIsINITdone = 1;
    pInst->vdec_codec_opened = 1;
    //LOGE("VDEC_INIT SUCCESS. vdec_mjpeg_jpu");

#if defined(TCC_JPU_C6_INCLUDE)
    ret = jpu_cmd_process(V_DEC_INIT, &pInst->gsJpuDecInit_Info, pInst);
    if( ret != JPG_RET_SUCCESS )
    {
      DPRINTF( "[JPU-%d] JPU_DEC_INIT failed \r", pInst->vdec_instance_index);
      return -ret;
    }

    if( (pInst->extFunction & (EXT_FUNC_MEM_PROTECTION_WITH_INSTREAM | EXT_FUNC_MEM_PROTECTION_ONLY)) == 0x0 )
    {
      pInst->gsJpuDecVersion.pszVersion = (char*)(pInst->gsBitstreamBufAddr[K_VA] + (pInst->gsBitstreamBufSize - 100));
      pInst->gsJpuDecVersion.pszBuildData = (char*)(pInst->gsBitstreamBufAddr[K_VA] + (pInst->gsBitstreamBufSize - 50));
  
      ret = jpu_cmd_process(V_GET_VPU_VERSION, &pInst->gsJpuDecVersion, pInst);
      if( ret != RETCODE_SUCCESS )
      {
        //If this operation returns fail, it doesn't mean that there's a problem in Jpu
        //so do not return error to host.
        DPRINTF( "[JPU-%d] V_GET_JPU_VERSION failed Error code is 0x%x ", pInst->vdec_instance_index, ret );
      }
      else
      {
        int vpu_closed[VPU_MAX*2];

        if( 0 <= ioctl(pInst->mgr_fd, VPU_CHECK_CODEC_STATUS, &vpu_closed) ) {
          LOGI("[JPU-%d] Multi-instance status : %d/%d/%d/%d/%d", pInst->vdec_instance_index, vpu_closed[VPU_DEC], vpu_closed[VPU_DEC_EXT], vpu_closed[VPU_DEC_EXT2], vpu_closed[VPU_DEC_EXT3], vpu_closed[VPU_ENC]);
        }
        LOGI( "[JPU-%d] V_GET_JPU_VERSION OK. Version is %.27s, and it's built at %.10s ", pInst->vdec_instance_index,
            (char*)(pInst->gsBitstreamBufAddr[VA] + (pInst->gsBitstreamBufSize - 100)),
            (char*)(pInst->gsBitstreamBufAddr[VA] + (pInst->gsBitstreamBufSize - 50)));
      }
    }
#endif
    return JPG_RET_SUCCESS;
  }
  else if( iOpCode == VDEC_DEC_SEQ_HEADER )
  {
    DSTATUS("[JPU-%d] VDEC_DEC_SEQ_HEADER. vdec_mjpeg_jpu", pInst->vdec_instance_index);

    vdec_input_t* p_input_param = (vdec_input_t*)pParam1;
    vdec_output_t* p_output_param = (vdec_output_t*)pParam2;
    int seq_stream_size = (p_input_param->m_iInpLen > pInst->gsMaxBitstreamSize) ? pInst->gsMaxBitstreamSize : p_input_param->m_iInpLen;
    DSTATUS("[JPU-%d] seq_stream_size = %d(%d). pInst->gsMaxBitstreamSize = %d", pInst->vdec_instance_index,
         seq_stream_size, p_input_param->m_iInpLen, pInst->gsMaxBitstreamSize);
    uint32_t iIsThumbnail = p_input_param->m_iIsThumbnail;

    //------------------------------------------------------------
    //! [x] bitstream buffer for each JPU decoder
    if (    ((codec_addr_t)p_input_param->m_pInp[PA] == pInst->gsBitstreamBufAddr[PA])
         && ((codec_addr_t)p_input_param->m_pInp[VA] == pInst->gsBitstreamBufAddr[VA]) )
    {
      //DPRINTF("BITSTREAM BUFFER ADDRESS IS SAME AS BEFORE");
      pInst->gsJpuDecInit_Info.gsJpuDecInit.m_BitstreamBufAddr[PA] = (codec_addr_t)pInst->gsBitstreamBufAddr[PA];
      pInst->gsJpuDecInit_Info.gsJpuDecInit.m_BitstreamBufAddr[VA] = (codec_addr_t)pInst->gsBitstreamBufAddr[K_VA];
    }
    else
    {
      //DPRINTF("BITSTREAM BUFFER ADDRESS IS CHANGED");
      pInst->gsJpuDecInit_Info.gsJpuDecInit.m_BitstreamBufAddr[PA] = (codec_addr_t)pInst->gsBitstreamBufAddr[PA];
      pInst->gsJpuDecInit_Info.gsJpuDecInit.m_BitstreamBufAddr[VA] = (codec_addr_t)pInst->gsBitstreamBufAddr[K_VA];

#if defined(TC_SECURE_MEMORY_COPY)
      if (pInst->extFunction & EXT_FUNC_MEM_PROTECTION_WITH_INSTREAM) {
    #if !REMOVE_SECURE_COPY
        TC_SecureMemoryCopy(pInst->gsBitstreamBufAddr[PA], p_input_param->m_pInp[PA], seq_stream_size);
    #else
        pInst->gsJpuDecInit_Info.gsJpuDecInit.m_BitstreamBufAddr[PA] = p_input_param->m_pInp[PA];
        pInst->gsJpuDecInit_Info.gsJpuDecInit.m_BitstreamBufAddr[VA] = (codec_addr_t)pInst->gsBitstreamBufAddr[K_VA];
    #endif
      }
      else
#endif
      {
        (void)memcpy( (unsigned long*)pInst->gsBitstreamBufAddr[VA], (unsigned long*)p_input_param->m_pInp[VA], seq_stream_size);
      }

      DSTATUS("[JPU-%d] copyed input-stream = %d (from 0x%lx)", pInst->vdec_instance_index, seq_stream_size, (unsigned long *)p_input_param->m_pInp[VA]);
#ifdef CHANGE_INPUT_STREAM
      change_input_stream((unsigned char *)pInst->gsBitstreamBufAddr[VA], &seq_stream_size, iOpCode, pInst);
#endif
    }

#if defined(TCC_JPU_C5_INCLUDE)
    pInst->gsJpuDecInit_Info.gsJpuDecInit.m_iBitstreamBufSize = pInst->gsBitstreamBufSize;
    pInst->gsJpuDecInit_Info.gsJpuDecInit.m_iRot_angle = 0;
    pInst->gsJpuDecInit_Info.gsJpuDecInit.m_iRot_enalbe = 0;
    pInst->gsJpuDecInit_Info.gsJpuDecInit.m_iMirror_enable = 0;
    pInst->gsJpuDecInit_Info.gsJpuDecInit.m_iMirrordir = 0;
#endif

#ifdef VPU_IN_FRAME_DUMP
    save_input_stream("seqHeader_stream.bin", seq_stream_size, pInst, 0);
#endif

#if defined(TCC_JPU_C6_INCLUDE)
    pInst->gsJpuDecSeqHeader_Info.stream_size = seq_stream_size;
    ret = jpu_cmd_process(V_DEC_SEQ_HEADER, &pInst->gsJpuDecSeqHeader_Info, pInst);
    if( ret != JPG_RET_SUCCESS )
    {
      DPRINTF( "[JPU-%d] V_DEC_SEQ_HEADER failed detail error  code is 0x%x \r", pInst->vdec_instance_index,
               pInst->gsJpuDecSeqHeader_Info.gsJpuDecInitialInfo.m_iErrorReason);
      return -ret;
    }
#else
    ret = jpu_cmd_process(V_DEC_INIT, &pInst->gsJpuDecInit_Info, pInst);
    if( ret != JPG_RET_SUCCESS )
    {
      DPRINTF( "[JPU-%d] JPU_DEC_INIT failed detail errod  code is 0x%x \r", pInst->vdec_instance_index, pInst->gsJpuDecInit_Info.gsJpuDecInitialInfo.m_error_reason);
      return -ret;
    }
#endif
    //------------------------------------------------------------
    //! [x] frame buffer for each JPU decoder
    //------------------------------------------------------------
    // scale factor set
    pInst->gsJpuDecBuffer_Info.gsJpuDecBuffer.m_iJPGScaleRatio = pInst->gsUserInfo.jpg_ScaleRatio;        //!< JPEG Scaling Ratio

#if defined(TCC_JPU_C6_INCLUDE)
    {
      int max_count;
      int minFrameBuffSize = pInst->gsJpuDecSeqHeader_Info.gsJpuDecInitialInfo.m_iMinFrameBufferSize[pInst->gsUserInfo.jpg_ScaleRatio];

      pInst->gsJpuDecBuffer_Info.gsJpuDecBuffer.m_iFrameBufferCount = pInst->gsJpuDecSeqHeader_Info.gsJpuDecInitialInfo.m_iMinFrameBufferCount;
      if(!iIsThumbnail){
        pInst->gsJpuDecBuffer_Info.gsJpuDecBuffer.m_iFrameBufferCount =
                 pInst->gsJpuDecSeqHeader_Info.gsJpuDecInitialInfo.m_iMinFrameBufferCount + pInst->gsAdditionalFrameCount;
      }
      max_count = cdk_sys_remain_memory_size(pInst) / minFrameBuffSize;

      if(pInst->gsJpuDecBuffer_Info.gsJpuDecBuffer.m_iFrameBufferCount > max_count){
        pInst->gsJpuDecBuffer_Info.gsJpuDecBuffer.m_iFrameBufferCount = max_count;
      }

      if(pInst->gsJpuDecBuffer_Info.gsJpuDecBuffer.m_iFrameBufferCount > MAX_FRAME_BUFFER_COUNT){
        pInst->gsJpuDecBuffer_Info.gsJpuDecBuffer.m_iFrameBufferCount = MAX_FRAME_BUFFER_COUNT;
      }

      if(iIsThumbnail)
      {
        if(pInst->gsJpuDecBuffer_Info.gsJpuDecBuffer.m_iFrameBufferCount < (pInst->gsJpuDecSeqHeader_Info.gsJpuDecInitialInfo.m_iMinFrameBufferCount))
        {
          LOGE( "[VDEC-%d] T Not enough memory for VPU frame buffer, Available[%d], Min[%d], Need[%d], min_size = %d",
              pInst->vdec_instance_index, max_count, pInst->gsJpuDecSeqHeader_Info.gsJpuDecInitialInfo.m_iMinFrameBufferCount,
              pInst->gsJpuDecBuffer_Info.gsJpuDecBuffer.m_iFrameBufferCount,
              minFrameBuffSize);
          return -(VPU_NOT_ENOUGH_MEM);
        }
      }
      else
      {
        if(pInst->gsJpuDecBuffer_Info.gsJpuDecBuffer.m_iFrameBufferCount < (pInst->gsJpuDecSeqHeader_Info.gsJpuDecInitialInfo.m_iMinFrameBufferCount/*+pInst->gsAdditionalFrameCount*/))
        {
          tcc_printf( "[VDEC-%d] [line:%d] Not enough memory for VPU frame buffer, Available[%d], Min[%d], Need[%d], min_size = %d",
              pInst->vdec_instance_index, __LINE__, max_count, pInst->gsJpuDecSeqHeader_Info.gsJpuDecInitialInfo.m_iMinFrameBufferCount,
              pInst->gsJpuDecSeqHeader_Info.gsJpuDecInitialInfo.m_iMinFrameBufferCount + pInst->gsAdditionalFrameCount,
              pInst->gsJpuDecSeqHeader_Info.gsJpuDecInitialInfo.m_iMinFrameBufferSize);
          return -(VPU_NOT_ENOUGH_MEM);
        }
      }
      pInst->gsFrameBufSize = pInst->gsJpuDecBuffer_Info.gsJpuDecBuffer.m_iFrameBufferCount * minFrameBuffSize;
      pInst->gsFrameBufSize = ALIGNED_BUFF( pInst->gsFrameBufSize, 4*1024 );
      LOGD( "[VDEC-%d] FrameBufferCount %d [min %d], min_size = %d ", pInst->vdec_instance_index,
            pInst->gsJpuDecBuffer_Info.gsJpuDecBuffer.m_iFrameBufferCount,
            pInst->gsJpuDecSeqHeader_Info.gsJpuDecInitialInfo.m_iMinFrameBufferCount, minFrameBuffSize);
    }
#else
    {
      //different buf size as scale
      pInst->gsFrameBufSize = pInst->gsJpuDecInit_Info.gsJpuDecInitialInfo.m_iJpg_MinFrameBufferSize[pInst->gsJpuDecBuffer_Info.gsJpuDecBuffer.m_iJPGScaleRatio];
      pInst->gsFrameBufSize = ALIGNED_BUFF( pInst->gsFrameBufSize, 4*1024 );
      DSTATUS("[JPU-%d] pInst->gsFrameBufSize = %d", pInst->vdec_instance_index, pInst->gsFrameBufSize);
    }
#endif

    pInst->gsFrameBufAddr[PA] = (codec_addr_t)cdk_sys_malloc_physical_addr( &pInst->gsFrameBufAddr[K_VA], pInst->gsFrameBufSize, BUFFER_FRAMEBUFFER, pInst );
    if( pInst->gsFrameBufAddr[PA] == 0 )
    {
      LOGE("[JPU-%d] pInst->gsFrameBufAddr[PA] malloc() failed \r", pInst->vdec_instance_index);
      return -(VPU_NOT_ENOUGH_MEM);
    }
    DSTATUS("[JPU-%d] pInst->gsFrameBufAddr[PA] = 0x%lx", pInst->vdec_instance_index, pInst->gsFrameBufAddr[PA]);
    pInst->gsFrameBufAddr[VA] = (codec_addr_t)cdk_sys_malloc_virtual_addr( pInst->gsFrameBufAddr[PA], pInst->gsFrameBufSize, pInst );
    if( pInst->gsFrameBufAddr[VA] == 0 )
    {
      LOGE( "[JPU-%d] pInst->gsFrameBufAddr[VA] malloc() failed \r", pInst->vdec_instance_index);
      return -(VPU_NOT_ENOUGH_MEM);
    }
    DSTATUS("[JPU-%d] pInst->gsFrameBufAddr[VA] = 0x%lx", pInst->vdec_instance_index, pInst->gsFrameBufAddr[VA]);

#if defined(TCC_JPU_C6_INCLUDE)
    pInst->mRealPicWidth = pInst->gsJpuDecSeqHeader_Info.gsJpuDecInitialInfo.m_iPicWidth;
    pInst->mRealPicHeight = pInst->gsJpuDecSeqHeader_Info.gsJpuDecInitialInfo.m_iPicHeight;
#else
    pInst->mRealPicWidth = pInst->gsJpuDecInit_Info.gsJpuDecInitialInfo.m_iPicWidth;
    pInst->mRealPicHeight = pInst->gsJpuDecInit_Info.gsJpuDecInitialInfo.m_iPicHeight;

    if( !pInst->gsUserInfo.m_bStillJpeg )
    {
      // Allocate multiple Output Frame buffers
      int buf_idx;

      pInst->decoded_buf_curIdx = 0;
      pInst->decoded_buf_size = pInst->gsFrameBufSize * 1.5;
      pInst->decoded_buf_size = ALIGNED_BUFF(pInst->decoded_buf_size, ALIGN_LEN);

      DSTATUS("[JPU-%d] pInst->gsAdditionalFrameCount = %d", pInst->vdec_instance_index, pInst->gsAdditionalFrameCount);
      for(buf_idx =0; buf_idx < pInst->gsAdditionalFrameCount + 1; buf_idx++)
      {
        pInst->decoded_phyAddr[buf_idx] = (codec_addr_t)cdk_sys_malloc_physical_addr( NULL, pInst->decoded_buf_size, BUFFER_ELSE, pInst );
        if( pInst->decoded_phyAddr[buf_idx] == 0 )
        {
          DPRINTF( "[JPU-%d,Err:%d] vdec_vpu pInst->decoded_virtAddr[PA] alloc failed ", pInst->vdec_instance_index, ret );
          return -(VPU_NOT_ENOUGH_MEM);
        }
        pInst->decoded_virtAddr[buf_idx] = (codec_addr_t)cdk_sys_malloc_virtual_addr( pInst->decoded_phyAddr[buf_idx], pInst->decoded_buf_size, pInst );
        if( pInst->decoded_virtAddr[buf_idx] == 0 )
        {
          DPRINTF( "[JPU-%d,Err:%d] vdec_vpu pInst->decoded_virtAddr[VA] alloc failed ", pInst->vdec_instance_index, ret );
          return -(VPU_NOT_ENOUGH_MEM);
        }
        pInst->decoded_buf_maxcnt = pInst->gsAdditionalFrameCount + 1;
        DSTATUS("[JPU-%d] OUT-Buffer %d ::   PA = 0x%lx, VA = 0x%lx, size = 0x%x!!", pInst->vdec_instance_index,
                    buf_idx, pInst->decoded_phyAddr[buf_idx], pInst->decoded_virtAddr[buf_idx], pInst->decoded_buf_size);
      }
    }
#endif

    pInst->gsJpuDecBuffer_Info.gsJpuDecBuffer.m_FrameBufferStartAddr[PA] = pInst->gsFrameBufAddr[PA];
    pInst->gsJpuDecBuffer_Info.gsJpuDecBuffer.m_FrameBufferStartAddr[VA] = pInst->gsFrameBufAddr[K_VA];

    ret = jpu_cmd_process(V_DEC_REG_FRAME_BUFFER, &pInst->gsJpuDecBuffer_Info, pInst);
    if( ret != JPG_RET_SUCCESS )
    {
      LOGE("[JPU-%d] VDEC_DEC_SEQ_HEADER (REG BUFFER FAIL). ret = %d", pInst->vdec_instance_index, ret);
      return -ret;
    }

    pInst->gsbHasSeqHeader = 1;
    pic_crop_t JpgCrop = {0,};
#if defined(TCC_JPU_C6_INCLUDE)
    set_dec_common_info(pInst->gsJpuDecSeqHeader_Info.gsJpuDecInitialInfo.m_iPicWidth, 
            pInst->gsJpuDecSeqHeader_Info.gsJpuDecInitialInfo.m_iPicHeight,
            &JpgCrop, 0, pInst->gsJpuDecSeqHeader_Info.gsJpuDecInitialInfo.m_iSourceFormat, pInst );

    p_output_param->m_pInitialInfo = &pInst->gsCommDecInfo;
    p_output_param->m_pInitialInfo->m_iPicWidth = pInst->gsJpuDecSeqHeader_Info.gsJpuDecInitialInfo.m_iPicWidth;
    p_output_param->m_pInitialInfo->m_iPicHeight = pInst->gsJpuDecSeqHeader_Info.gsJpuDecInitialInfo.m_iPicHeight;
#else
    set_dec_common_info(pInst->gsJpuDecInit_Info.gsJpuDecInitialInfo.m_iPicWidth,
            pInst->gsJpuDecInit_Info.gsJpuDecInitialInfo.m_iPicHeight,
            &JpgCrop, 0, pInst->gsJpuDecInit_Info.gsJpuDecInitialInfo.m_iJpg_sourceFormat, pInst );

    p_output_param->m_pInitialInfo = &pInst->gsCommDecInfo;
    p_output_param->m_pInitialInfo->m_iPicWidth = pInst->gsJpuDecInit_Info.gsJpuDecInitialInfo.m_iPicWidth;
    p_output_param->m_pInitialInfo->m_iPicHeight = pInst->gsJpuDecInit_Info.gsJpuDecInitialInfo.m_iPicHeight;
#endif

    LOGI( "[JPU-%d] VDEC_DEC_SEQ_HEADER - Success mem_free = 0x%x ", pInst->vdec_instance_index, cdk_sys_final_free_mem(pInst) );
    DSTATUS( "[JPU-%d] =======================================================", pInst->vdec_instance_index );

    return RETCODE_SUCCESS;
  }
  else if( iOpCode == VDEC_DECODE )
  {
    vdec_input_t* p_input_param = (vdec_input_t*)pParam1;
    vdec_output_t* p_output_param = (vdec_output_t*)pParam2;

    pInst->gsJpuDecInOut_Info.gsJpuDecInput.m_iBitstreamDataSize = (p_input_param->m_iInpLen > pInst->gsMaxBitstreamSize) ? pInst->gsMaxBitstreamSize : p_input_param->m_iInpLen;
    DSTATUS("[JPU-%d] pInst->gsJpuDecInOut_Info.gsJpuDecInput.m_iBitstreamDataSize = %d(%d)", pInst->vdec_instance_index
                        , pInst->gsJpuDecInOut_Info.gsJpuDecInput.m_iBitstreamDataSize, p_input_param->m_iInpLen);

    if (    ((codec_addr_t)p_input_param->m_pInp[PA] == pInst->gsBitstreamBufAddr[PA])
         && ((codec_addr_t)p_input_param->m_pInp[VA] == pInst->gsBitstreamBufAddr[VA]) )
    {
      //DPRINTF("BITSTREAM BUFFER IS SAME AS BEFORE.");
      pInst->gsJpuDecInOut_Info.gsJpuDecInput.m_BitstreamDataAddr[PA] = pInst->gsBitstreamBufAddr[PA];
      pInst->gsJpuDecInOut_Info.gsJpuDecInput.m_BitstreamDataAddr[VA] = pInst->gsBitstreamBufAddr[K_VA];
    }
    else
    {
#if defined(TC_SECURE_MEMORY_COPY)
      if( (pInst->extFunction & EXT_FUNC_MEM_PROTECTION_WITH_INSTREAM) != 0x0 ) {
          TC_SecureMemoryCopy(pInst->gsBitstreamBufAddr[PA], p_input_param->m_pInp[PA], pInst->gsJpuDecInOut_Info.gsJpuDecInput.m_iBitstreamDataSize);
      }
      else
#endif
      {
        //DPRINTF("BITSTREAM BUFFER IS CHANGED.");
        pInst->gsJpuDecInOut_Info.gsJpuDecInput.m_BitstreamDataAddr[PA] = pInst->gsBitstreamBufAddr[PA];
        pInst->gsJpuDecInOut_Info.gsJpuDecInput.m_BitstreamDataAddr[VA] = pInst->gsBitstreamBufAddr[K_VA];

        if( !pInst->gsFirstFrame )
        {
          (void)memcpy( (unsigned long*)pInst->gsBitstreamBufAddr[VA], (unsigned long*)p_input_param->m_pInp[VA], pInst->gsJpuDecInOut_Info.gsJpuDecInput.m_iBitstreamDataSize);
          DSTATUS("[JPU-%d] copyed input-stream = %d", pInst->vdec_instance_index, pInst->gsJpuDecInOut_Info.gsJpuDecInput.m_iBitstreamDataSize);
#ifdef CHANGE_INPUT_STREAM
          change_input_stream((unsigned char *)pInst->gsBitstreamBufAddr[VA], &(pInst->gsJpuDecInOut_Info.gsJpuDecInput.m_iBitstreamDataSize), iOpCode, pInst);
#endif
        }
      }
      pInst->gsFirstFrame = 0;
    }

#if defined(TCC_JPU_C5_INCLUDE)
    pInst->gsJpuDecInOut_Info.gsJpuDecInput.m_iLooptogle = 0;
    pInst->gsJpuDecInOut_Info.gsJpuDecInput.m_FrameBufferStartAddr[PA] = (codec_addr_t)pInst->gsFrameBufAddr[PA];
    pInst->gsJpuDecInOut_Info.gsJpuDecInput.m_FrameBufferStartAddr[VA] = (codec_addr_t)pInst->gsFrameBufAddr[K_VA];
#endif

    if( (pInst->extFunction & (EXT_FUNC_MEM_PROTECTION_WITH_INSTREAM | EXT_FUNC_MEM_PROTECTION_ONLY)) == 0x0 )
    {
      unsigned char* ps = (unsigned char*)pInst->gsBitstreamBufAddr[VA];
            uint32_t len = pInst->gsJpuDecInOut_Info.gsJpuDecInput.m_iBitstreamDataSize;
      DSTATUS( "[JPU-%d] " "0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x " "0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x "
                "0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x " "0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x "
                "~ 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x ",
                pInst->vdec_instance_index,
                ps[0], ps[1], ps[2], ps[3], ps[4], ps[5], ps[6], ps[7], ps[8], ps[9], ps[10], ps[11], ps[12], ps[13], ps[14], ps[15],
                ps[16], ps[17], ps[18], ps[19], ps[20], ps[21], ps[22], ps[23], ps[24], ps[25], ps[26], ps[27], ps[28], ps[29], ps[30], ps[31],
                ps[32], ps[33], ps[34], ps[35], ps[36], ps[37], ps[38], ps[39], ps[40], ps[41], ps[42], ps[43], ps[44], ps[45], ps[46], ps[47],
                ps[48], ps[49], ps[50], ps[51], ps[52], ps[53], ps[54], ps[55], ps[56], ps[57], ps[58], ps[59], ps[60], ps[61], ps[62], ps[63],
                ps[len-8], ps[len-7], ps[len-6], ps[len-5], ps[len-4], ps[len-3], ps[len-2], ps[len-1]);
    }


    DSTATUS("[JPU-%d] Decode In", pInst->vdec_instance_index);
    ret = jpu_cmd_process(V_DEC_DECODE, &pInst->gsJpuDecInOut_Info, pInst);

    if( ret != RETCODE_SUCCESS )
    {
      DPRINTF( "[JPU-%d] JPU_DEC_DECODE failed Error code is 0x%x ", pInst->vdec_instance_index, ret );
      return -ret;
    }
    DSTATUS("[JPU-%d] ret = %d, Dec-Status %d", pInst->vdec_instance_index, ret, pInst->gsJpuDecInOut_Info.gsJpuDecOutput.m_DecOutInfo.m_iDecodingStatus);

    p_output_param->m_DecOutInfo.m_iWidth = pInst->gsJpuDecInOut_Info.gsJpuDecOutput.m_DecOutInfo.m_iWidth;
    p_output_param->m_DecOutInfo.m_iHeight = pInst->gsJpuDecInOut_Info.gsJpuDecOutput.m_DecOutInfo.m_iHeight;

    p_output_param->m_DecOutInfo.m_iDecodingStatus = pInst->gsJpuDecInOut_Info.gsJpuDecOutput.m_DecOutInfo.m_iDecodingStatus;
    if( pInst->gsJpuDecInOut_Info.gsJpuDecOutput.m_DecOutInfo.m_iDecodingStatus == 1)
    {
#if defined(TCC_JPU_C5_INCLUDE)
      if( !pInst->gsUserInfo.m_bStillJpeg )
      {
        uint32_t yOffset = pInst->gsJpuDecInOut_Info.gsJpuDecOutput.m_pCurrOut[VA][COMP_U] - pInst->gsJpuDecInOut_Info.gsJpuDecOutput.m_pCurrOut[VA][COMP_Y];
        uint32_t uvOffset = pInst->gsJpuDecInOut_Info.gsJpuDecOutput.m_pCurrOut[VA][COMP_V] - pInst->gsJpuDecInOut_Info.gsJpuDecOutput.m_pCurrOut[VA][COMP_U];
        uint32_t frame_size = yOffset + uvOffset*2;
        (void)memcpy( (unsigned char *)pInst->decoded_virtAddr[pInst->decoded_buf_curIdx],
                (unsigned char *)vpu_getFrameBufVirtAddr(pInst->gsJpuDecInOut_Info.gsJpuDecOutput.m_pCurrOut[VA][COMP_Y], K_VA, pInst),
                frame_size );

        p_output_param->m_pCurrOut[PA][COMP_Y] = (unsigned char *)pInst->decoded_phyAddr[pInst->decoded_buf_curIdx];
        p_output_param->m_pCurrOut[PA][COMP_U] = (unsigned char *)pInst->decoded_phyAddr[pInst->decoded_buf_curIdx] + yOffset;
        p_output_param->m_pCurrOut[PA][COMP_V] = (unsigned char *)pInst->decoded_phyAddr[pInst->decoded_buf_curIdx] + yOffset + uvOffset;
        p_output_param->m_pCurrOut[VA][COMP_Y] = (unsigned char *)pInst->decoded_virtAddr[pInst->decoded_buf_curIdx];
        p_output_param->m_pCurrOut[VA][COMP_U] = (unsigned char *)pInst->decoded_virtAddr[pInst->decoded_buf_curIdx] + yOffset;
        p_output_param->m_pCurrOut[VA][COMP_V] = (unsigned char *)pInst->decoded_virtAddr[pInst->decoded_buf_curIdx] + yOffset + uvOffset;
        p_output_param->m_pDispOut[PA][COMP_Y] = p_output_param->m_pCurrOut[PA][COMP_Y];
        p_output_param->m_pDispOut[PA][COMP_U] = p_output_param->m_pCurrOut[PA][COMP_U];
        p_output_param->m_pDispOut[PA][COMP_V] = p_output_param->m_pCurrOut[PA][COMP_V];
        p_output_param->m_pDispOut[VA][COMP_Y] = p_output_param->m_pCurrOut[VA][COMP_Y];
        p_output_param->m_pDispOut[VA][COMP_U] = p_output_param->m_pCurrOut[VA][COMP_U];
        p_output_param->m_pDispOut[VA][COMP_V] = p_output_param->m_pCurrOut[VA][COMP_V];

        pInst->decoded_buf_curIdx++;
        if(pInst->decoded_buf_curIdx >= pInst->decoded_buf_maxcnt){
          pInst->decoded_buf_curIdx = 0;
        }
        DSTATUS("[JPU-%d] copyed yuv(0x%x) stream", pInst->vdec_instance_index, p_output_param->m_pDispOut[VA][COMP_Y]);
      }
      else
#endif
      {
        p_output_param->m_pCurrOut[PA][COMP_Y] = (unsigned char *)pInst->gsJpuDecInOut_Info.gsJpuDecOutput.m_pCurrOut[PA][COMP_Y];
        p_output_param->m_pCurrOut[PA][COMP_U] = (unsigned char *)pInst->gsJpuDecInOut_Info.gsJpuDecOutput.m_pCurrOut[PA][COMP_U];
        p_output_param->m_pCurrOut[PA][COMP_V] = (unsigned char *)pInst->gsJpuDecInOut_Info.gsJpuDecOutput.m_pCurrOut[PA][COMP_V];
        p_output_param->m_pCurrOut[VA][COMP_Y] = (unsigned char *)vpu_getFrameBufVirtAddr(pInst->gsJpuDecInOut_Info.gsJpuDecOutput.m_pCurrOut[VA][COMP_Y], K_VA, pInst);
        p_output_param->m_pCurrOut[VA][COMP_U] = (unsigned char *)vpu_getFrameBufVirtAddr(pInst->gsJpuDecInOut_Info.gsJpuDecOutput.m_pCurrOut[VA][COMP_U], K_VA, pInst);
        p_output_param->m_pCurrOut[VA][COMP_V] = (unsigned char *)vpu_getFrameBufVirtAddr(pInst->gsJpuDecInOut_Info.gsJpuDecOutput.m_pCurrOut[VA][COMP_V], K_VA, pInst);
        p_output_param->m_pDispOut[PA][COMP_Y] = p_output_param->m_pCurrOut[PA][COMP_Y];
        p_output_param->m_pDispOut[PA][COMP_U] = p_output_param->m_pCurrOut[PA][COMP_U];
        p_output_param->m_pDispOut[PA][COMP_V] = p_output_param->m_pCurrOut[PA][COMP_V];
        p_output_param->m_pDispOut[VA][COMP_Y] = p_output_param->m_pCurrOut[VA][COMP_Y];
        p_output_param->m_pDispOut[VA][COMP_U] = p_output_param->m_pCurrOut[VA][COMP_U];
        p_output_param->m_pDispOut[VA][COMP_V] = p_output_param->m_pCurrOut[VA][COMP_V];
        DSTATUS("[JPU-%d] yuv(0x%x) stream", pInst->vdec_instance_index, p_output_param->m_pDispOut[VA][COMP_Y]);
      }

      p_output_param->m_DecOutInfo.m_iOutputStatus = 1;
    }
    return ret;
  }
  else if( iOpCode == VDEC_BUF_FLAG_CLEAR )
  {
    //DPRINTF("VDEC_BUF_FLAG_CLEAR. vdec_mjpeg_jpu");
    return RETCODE_SUCCESS;
  }
  else if( iOpCode == VDEC_DEC_FLUSH_OUTPUT)
  {
    return RETCODE_SUCCESS;
  }
  else if( iOpCode == VDEC_CLOSE )
  {
    DSTATUS("[JPU-%d] VDEC_CLOSE. vdec_mjpeg_jpu", pInst->vdec_instance_index);
    jpu_dec_output_t dec_output = {{0,},};
    int i;

    // Now that we are done with decoding, close the open instance.

    cdk_sys_free_virtual_addr( (unsigned long*)pInst->gsBitstreamBufAddr[VA], pInst->gsBitstreamBufSize);
    cdk_sys_free_virtual_addr( (unsigned long*)pInst->gsFrameBufAddr[VA], pInst->gsFrameBufSize);

#if defined(TCC_JPU_C5_INCLUDE)
    if( !pInst->gsUserInfo.m_bStillJpeg )
    {
      uint32_t buf_idx;
      for(buf_idx =0; buf_idx < pInst->decoded_buf_maxcnt; buf_idx++)
      {
        cdk_sys_free_virtual_addr( (unsigned long*)pInst->decoded_virtAddr[buf_idx], pInst->decoded_buf_size );
      }
    }
#endif
    ret = jpu_cmd_process(V_DEC_CLOSE, &pInst->gsJpuDecInOut_Info, pInst);
    if(ret != RETCODE_SUCCESS)
    {
      LOGE("[JPU-%d] JPU_DEC_CLOSE FAIL", pInst->vdec_instance_index);
      return -ret;
    }

    vpu_env_close(pInst);
    return RETCODE_SUCCESS;
  }

  LOGE("[JPU-%d] INVALID OP_CODE", pInst->vdec_instance_index);
  return 0;
}

#endif //TCC_JPU_INCLUDE
