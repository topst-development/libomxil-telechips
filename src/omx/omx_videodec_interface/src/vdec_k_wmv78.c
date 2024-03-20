// SPDX-License-Identifier: LGPL-2.1-or later
/****************************************************************************
 *   FileName    : vdec_k_wmv78.c
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

#ifdef INCLUDE_WMV78_DEC

#define LOG_TAG "VPU_DEC_K_WMV78"

#include "TCCMemory.h"
#include "wmv78dec/TCC_WMV78_DEC.h"
#include "wmv78dec/TCC_WMV78_DEC_Huff_table.h"

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

#define EXT_LIB_NAME ("libtccwmv78dec.so")
//! Callback Func
typedef struct vdec_callback_func_t
{
  unsigned long*    (*m_pfMalloc        ) ( size_t );                            //!< malloc
  unsigned long*    (*m_pfNonCacheMalloc) ( size_t );                            //!< non-cacheable malloc
  void     (*m_pfFree          ) ( unsigned long* );                             //!< free
  void     (*m_pfNonCacheFree  ) ( unsigned long* );                             //!< non-cacheable free
  unsigned long*    (*m_pfMemcpy        ) ( unsigned long*, const unsigned long*, size_t );        //!< memcpy
  void     (*m_pfMemset        ) ( unsigned long*, int32_t, size_t );                //!< memset
  unsigned long*    (*m_pfRealloc       ) ( unsigned long*, size_t );                     //!< realloc
  unsigned long*    (*m_pfMemmove       ) ( unsigned long*, const unsigned long*, size_t );        //!< memmove

  unsigned long*    (*m_pfPhysicalAlloc ) ( size_t );
  void     (*m_pfPhysicalFree  ) ( unsigned long*, size_t );
  unsigned long*    (*m_pfVirtualAlloc  ) ( unsigned long*, size_t, size_t );
  void     (*m_pfVirtualFree   ) ( unsigned long*, size_t, size_t );
  int32_t  m_Reserved1[16-12];

  unsigned long*    (*m_pfFopen         ) (const char *, const char *);          //!< fopen
  size_t   (*m_pfFread         ) (unsigned long*, size_t, size_t, unsigned long* );       //!< fread
  int32_t  (*m_pfFseek         ) (unsigned long*, long, int32_t );               //!< fseek
  long     (*m_pfFtell         ) (unsigned long* );                              //!< ftell
  size_t   (*m_pfFwrite        ) (const unsigned long*, size_t, size_t, unsigned long*);  //!< fwrite
  int32_t  (*m_pfFclose        ) (unsigned long*);                              //!< fclose
  int32_t  (*m_pfUnlink        ) (const char *);                       //!< _unlink
  uint32_t (*m_pfFeof          ) (unsigned long*);                              //!< feof
  uint32_t (*m_pfFflush        ) (unsigned long*);                              //!< fflush

  int32_t  (*m_pfFseek64       ) (unsigned long*, int64_t, int32_t );            //!< fseek 64bi io
  int64_t  (*m_pfFtell64       ) (unsigned long* );                              //!< ftell 64bi io
  int32_t  m_Reserved2[16-11];
} vdec_callback_func_t;

int WMV78_dec_seq_header(_vdec_ *pVdec)
{
  int ret = 0;
  _vdec_ * pInst = pVdec;
  memset( &pInst->gsWMV78DecInitialInfo, 0, sizeof(pInst->gsWMV78DecInitialInfo) );

  pInst->gsWMV78DecInitialInfo.m_iAvcPicCrop.m_iCropBottom = 0;
  pInst->gsWMV78DecInitialInfo.m_iAvcPicCrop.m_iCropLeft = 0;
  pInst->gsWMV78DecInitialInfo.m_iAvcPicCrop.m_iCropRight = 0;
  pInst->gsWMV78DecInitialInfo.m_iAvcPicCrop.m_iCropTop = 0;
  pInst->gsWMV78DecInitialInfo.m_iMinFrameBufferCount = 1;
  pInst->gsWMV78DecInitialInfo.m_iPicWidth = pInst->gsWMV78DecInit.m_iWidth;
  pInst->gsWMV78DecInitialInfo.m_iPicHeight = pInst->gsWMV78DecInit.m_iHeight;
  pInst->gsWMV78DecInitialInfo.m_iAspectRateInfo = 0;
  pInst->gsWMV78DecInitialInfo.m_iInterlace = 0;
  print_dec_initial_info( &pInst->gsVpuDecInit, &pInst->gsWMV78DecInitialInfo, pVdec );
  set_dec_common_info(pInst->gsWMV78DecInitialInfo.m_iPicWidth, pInst->gsWMV78DecInitialInfo.m_iPicHeight,
            &pInst->gsWMV78DecInitialInfo.m_iAvcPicCrop, pInst->gsWMV78DecInitialInfo.m_iInterlace,
            0, pInst );

  pInst->mRealPicWidth = pInst->gsWMV78DecInitialInfo.m_iPicWidth;
  pInst->mRealPicHeight = pInst->gsWMV78DecInitialInfo.m_iPicHeight;

  return ret;
}

int WMV78_LoadLibrary(_vdec_ *pVdec)
{
  int ret = 0;
  pVdec->pExtDLLModule = dlopen(EXT_LIB_NAME, RTLD_LAZY | RTLD_GLOBAL);
  if( pVdec->pExtDLLModule == NULL ) {
    LOGE("[SW-WMV12] Load library '%s' failed: %s", EXT_LIB_NAME, dlerror());
    ret = -1;
  } else {
    LOGI("[SW-WMV12] Library '%s' Loaded", EXT_LIB_NAME);
  }

  if (ret == 0) {
    pVdec->gExtDecFunc = dlsym(pVdec->pExtDLLModule, "Video_Proc");
    if( pVdec->gExtDecFunc == NULL ) {
      LOGE("[SW-WMV12] pVdec->gExtDecFunc Error");
      ret = -1;
    }
  }
  
  return ret;
}

void WMV78_CloseLibrary(_vdec_ *pVdec)
{
  if( pVdec->pExtDLLModule != NULL){
    dlclose(pVdec->pExtDLLModule);
  }
}

int
vdec_WMV78( int iOpCode, unsigned long* pHandle, unsigned long* pParam1, unsigned long* pParam2, unsigned long* pParam3 )
{
  int ret = 0;
  _vdec_ *pInst = (_vdec_ *)pParam3;
  int buf_idx;

  if(!pInst){
    LOGE("vdec_WMV78(OP:%d) :: Instance is null!!", iOpCode);
    return (-RETCODE_NOT_INITIALIZED);
  }

  if( (iOpCode != VDEC_INIT) && (iOpCode != VDEC_CLOSE) && (!pInst->vdec_codec_opened)){
    return -RETCODE_NOT_INITIALIZED;
  }
  if( iOpCode == VDEC_INIT )
  {
    uint32_t ChromaSize;
    vdec_init_t* p_init_param = (vdec_init_t*)pParam1;
//    vdec_callback_func_t* pf_callback = (vdec_callback_func_t*)pParam2;

    if( 0 > WMV78_LoadLibrary(pInst)){
      return -(VPU_ENV_INIT_ERROR);
    }
    pInst->codec_format = p_init_param->m_iBitstreamFormat;
    if(vpu_env_open(p_init_param->m_iBitstreamFormat, 0, 0, p_init_param->m_iPicWidth, p_init_param->m_iPicHeight, pInst ) < 0) // to operate Max-clock for s/w codec!!
    {
      return -(VPU_ENV_INIT_ERROR);
    }
    pInst->gsWMV78FrameSize = ( (p_init_param->m_iPicWidth+15)&0xfffffff0 ) * ( (p_init_param->m_iPicHeight+15)&0xfffffff0 );
    ChromaSize = pInst->gsWMV78FrameSize>>2;
    pInst->gsWMV78NCFrameSize = pInst->gsWMV78FrameSize*1.5;
    pInst->gsWMV78NCFrameSize = ALIGNED_BUFF( pInst->gsWMV78NCFrameSize, ALIGN_LEN );

    pInst->gsWMV78CurYFrameAddress = ((unsigned char*)TCC_malloc(pInst->gsWMV78NCFrameSize));
    pInst->gsWMV78CurUFrameAddress  = ((unsigned char*)pInst->gsWMV78CurYFrameAddress + pInst->gsWMV78FrameSize);
    pInst->gsWMV78CurVFrameAddress  = ((unsigned char*)pInst->gsWMV78CurUFrameAddress + ChromaSize);

    pInst->gsWMV78Ref0YFrameAddress = ((unsigned char*)TCC_malloc(pInst->gsWMV78NCFrameSize));
    pInst->gsWMV78Ref0UFrameAddress = ((unsigned char*)pInst->gsWMV78Ref0YFrameAddress + pInst->gsWMV78FrameSize);
    pInst->gsWMV78Ref0VFrameAddress = ((unsigned char*)pInst->gsWMV78Ref0UFrameAddress + ChromaSize);
    pInst->gsWMV78DecInit.m_pExtraData      = p_init_param->m_pExtraData;
    pInst->gsWMV78DecInit.m_iExtraDataLen   = p_init_param->m_iExtraDataLen;
    pInst->gsWMV78DecInit.m_iWidth        = (p_init_param->m_iPicWidth+15)&0xfffffff0;
    pInst->gsWMV78DecInit.m_iHeight     = p_init_param->m_iPicHeight;
    pInst->gsWMV78DecInit.m_iFourCC     = p_init_param->m_iFourCC;
    pInst->gsWMV78DecInit.m_pHeapAddress    = (unsigned char*)TCC_malloc(sizeof(unsigned char)*200*1024);
    pInst->gsWMV78DecInit.m_pHuff_tbl_address = (unsigned char*)WMV78_Huff_table;

    pInst->gsWMV78DecInit.m_pCurFrameAddress  = (tYUV420Frame_WMV*)TCC_malloc(sizeof(tYUV420Frame_WMV));
    pInst->gsWMV78DecInit.m_pRef0FrameAddress = (tYUV420Frame_WMV*)TCC_malloc(sizeof(tYUV420Frame_WMV));
    pInst->gsWMV78DecInit.m_pCurFrameAddress->m_pucYPlane   = (unsigned char*)pInst->gsWMV78CurYFrameAddress;
    pInst->gsWMV78DecInit.m_pCurFrameAddress->m_pucUPlane   = (unsigned char*)pInst->gsWMV78CurUFrameAddress;
    pInst->gsWMV78DecInit.m_pCurFrameAddress->m_pucVPlane   = (unsigned char*)pInst->gsWMV78CurVFrameAddress;
    pInst->gsWMV78DecInit.m_pRef0FrameAddress->m_pucYPlane    = (unsigned char*)pInst->gsWMV78Ref0YFrameAddress;
    pInst->gsWMV78DecInit.m_pRef0FrameAddress->m_pucUPlane    = (unsigned char*)pInst->gsWMV78Ref0UFrameAddress;
    pInst->gsWMV78DecInit.m_pRef0FrameAddress->m_pucVPlane    = (unsigned char*)pInst->gsWMV78Ref0VFrameAddress;
    pInst->gsWMV78DecOutput.m_pDecodedData = (tYUV420Frame_WMV*)TCC_malloc(sizeof(tYUV420Frame_WMV));

    pInst->gsWMV78DecInit.m_sCallbackFunc.m_pMalloc  = (unsigned long*  (*) ( uint32_t ))TCC_malloc;
    pInst->gsWMV78DecInit.m_sCallbackFunc.m_pFree    = (void   (*) ( unsigned long* ))TCC_free;
    pInst->gsWMV78DecInit.m_sCallbackFunc.m_pMemcpy  = (unsigned long*  (*) ( unsigned long*, const unsigned long*, uint32_t ))memcpy;
    pInst->gsWMV78DecInit.m_sCallbackFunc.m_pMemset  = (void  (*) ( unsigned long*, int, uint32_t ))memset;
    pInst->gsWMV78DecInit.m_sCallbackFunc.m_pRealloc = (unsigned long*  (*) ( unsigned long*, uint32_t ))TCC_realloc;
    pInst->gsWMV78DecInit.m_sCallbackFunc.m_pMemmove = (unsigned long*  (*) ( unsigned long*, const unsigned long*, uint32_t ))memmove;

    pInst->decoded_buf_curIdx = 0;
    pInst->decoded_buf_size = pInst->gsWMV78FrameSize * 1.5;
    pInst->decoded_buf_size = ALIGNED_BUFF(pInst->decoded_buf_size, ALIGN_LEN);

    for(buf_idx=0; buf_idx < (pInst->gsAdditionalFrameCount + 1); buf_idx++)
    {
      pInst->decoded_phyAddr[buf_idx] = (codec_addr_t)cdk_sys_malloc_physical_addr( NULL, pInst->decoded_buf_size, BUFFER_ELSE, pInst );
      if( pInst->decoded_phyAddr[buf_idx] == 0 )
      {
        DPRINTF( "[SW-WMV12,Err:%d] vdec_vpu pInst->decoded_virtAddr[PA] alloc failed ", ret );
        return -(VPU_NOT_ENOUGH_MEM);
      }
      pInst->decoded_virtAddr[buf_idx] = (codec_addr_t)cdk_sys_malloc_virtual_addr( pInst->decoded_phyAddr[buf_idx], pInst->decoded_buf_size, pInst );
      if( pInst->decoded_virtAddr[buf_idx] == 0 )
      {
        DPRINTF( "[SW-WMV12,Err:%d] vdec_vpu pInst->decoded_virtAddr[VA] alloc failed ", ret );
        return -(VPU_NOT_ENOUGH_MEM);
      }

      pInst->decoded_buf_maxcnt = pInst->gsAdditionalFrameCount + 1;
      DSTATUS("OUT-Buffer %d ::   PA = %lx, VA = %lx, size = 0x%x!!",
        buf_idx, pInst->decoded_phyAddr[buf_idx], pInst->decoded_virtAddr[buf_idx],   pInst->decoded_buf_size);
    }

    pInst->gsVpuDecInit.m_iBitstreamFormat  = p_init_param->m_iBitstreamFormat;
    pInst->gsFirstFrame = 1;

    DSTATUS( "[SW-WMV12] WMV78_DEC_INIT Enter " );
    ret = pInst->gExtDecFunc( VDEC_INIT, &pInst->gsWMV78DecHandle, &pInst->gsWMV78DecInit, NULL );
    if( ret != RETCODE_SUCCESS )
    {
      DPRINTF( "[SW-WMV12] WMV78_DEC_INIT failed Error code is 0x%x ", ret );
      return -ret;
    }
    pInst->gsIsINITdone = 1;
    pInst->vdec_codec_opened = 1;
    DSTATUS( "[SW-WMV12] WMV78_DEC_INIT OK " );
  }
  else if( iOpCode == VDEC_DEC_SEQ_HEADER )
  {
//    vdec_input_t* p_input_param = (vdec_input_t*)pParam1;
    vdec_output_t* p_output_param = (vdec_output_t*)pParam2;

    if( pInst->gsFirstFrame )
    {
      DSTATUS( "[SW-WMV12] VDEC_DEC_SEQ_HEADER start " );
      p_output_param->m_pInitialInfo = &pInst->gsCommDecInfo;
      ret = WMV78_dec_seq_header(pInst);
      if( ret != RETCODE_SUCCESS )
      {
        DPRINTF( "[SW-WMV12] vpu_dec_seq_header failed Error code is 0x%x ", ret );
        return ret;
      }
      DSTATUS( "[SW-WMV12] VDEC_DEC_SEQ_HEADER - Success " );
      pInst->gsFirstFrame = 0;
    }

    return RETCODE_SUCCESS;
  }
  else if( iOpCode == VDEC_DECODE )
  {
    vdec_input_t* p_input_param = (vdec_input_t*)pParam1;
    vdec_output_t* p_output_param = (vdec_output_t*)pParam2;

    #ifdef PRINT_VPU_INPUT_STREAM
    {
      int kkk;
      unsigned char* p_input = p_input_param->m_pInp[VA];
      int input_size = p_input_param->m_iInpLen;
      tcc_printf("FS = %7d :", input_size);
      for( kkk = 0; kkk < PRINT_BYTES; kkk++ ){
        tcc_printf("%02X ", p_input[kkk] );
      }
      tcc_printf("");
    }
    #endif

    if( pInst->gsFirstFrame )
    {
      DSTATUS( "[SW-WMV12] VDEC_DEC_SEQ_HEADER start " );
      p_output_param->m_pInitialInfo = &pInst->gsCommDecInfo;
      ret = WMV78_dec_seq_header(pInst);
      if( ret != RETCODE_SUCCESS )
      {
        DPRINTF( "[SW-WMV12] vpu_dec_seq_header failed Error code is 0x%x ", ret );
        return ret;
      }
      DSTATUS( "[SW-WMV12] VDEC_DEC_SEQ_HEADER - Success " );
      pInst->gsFirstFrame = 0;
    }

    pInst->gsWMV78DecInput.m_pPacketBuff = (unsigned char*)p_input_param->m_pInp[VA];
    pInst->gsWMV78DecInput.m_iPacketBuffSize = p_input_param->m_iInpLen;

    if (pInst->gsWMV78DecInput.m_iPacketBuffSize == 0)
    {
      DPRINTF( "[SW-WMV12] END_OF_FILE ");
    }

    // Start decoding a frame.
    ret = pInst->gExtDecFunc( VDEC_DECODE, &pInst->gsWMV78DecHandle, &pInst->gsWMV78DecInput, &pInst->gsWMV78DecOutput );
    if( ret < 0 )
    {
      DPRINTF( "[SW-WMV12] VDEC_DECODE failed Error code is 0x%x ", ret );
      return -ret;
    }

    (void)memcpy( (unsigned char *)pInst->decoded_virtAddr[pInst->decoded_buf_curIdx], pInst->gsWMV78DecOutput.m_pDecodedData->m_pucYPlane, pInst->gsWMV78FrameSize*1.5*sizeof(unsigned char) );
    p_output_param->m_pCurrOut[0][0] = p_output_param->m_pDispOut[0][0] = (unsigned char*)pInst->decoded_phyAddr[pInst->decoded_buf_curIdx];
    p_output_param->m_pCurrOut[0][1] = p_output_param->m_pDispOut[0][1] = (unsigned char*)p_output_param->m_pDispOut[0][0] + pInst->gsWMV78FrameSize;
    p_output_param->m_pCurrOut[0][2] = p_output_param->m_pDispOut[0][2] = (unsigned char*)p_output_param->m_pDispOut[0][1] + pInst->gsWMV78FrameSize/4;
    p_output_param->m_pCurrOut[1][0] = p_output_param->m_pDispOut[1][0] = (unsigned char*)pInst->decoded_virtAddr[pInst->decoded_buf_curIdx];
    p_output_param->m_pCurrOut[1][1] = p_output_param->m_pDispOut[1][1] = (unsigned char*)p_output_param->m_pDispOut[1][0] + pInst->gsWMV78FrameSize;
    p_output_param->m_pCurrOut[1][2] = p_output_param->m_pDispOut[1][2] = (unsigned char*)p_output_param->m_pDispOut[1][1] + pInst->gsWMV78FrameSize/4;

    pInst->decoded_buf_curIdx++;
    if(pInst->decoded_buf_curIdx >= pInst->decoded_buf_maxcnt){
      pInst->decoded_buf_curIdx = 0;
    }
    p_output_param->m_DecOutInfo.m_iPicType          = pInst->gsWMV78DecOutput.m_iPictureType;
    p_output_param->m_DecOutInfo.m_iDecodedIdx       = 0;
    p_output_param->m_DecOutInfo.m_iDispOutIdx       = 0;
    p_output_param->m_DecOutInfo.m_iDecodingStatus   = 1;
    p_output_param->m_DecOutInfo.m_iOutputStatus     = 1;
    p_output_param->m_DecOutInfo.m_iInterlacedFrame  = 0;
    p_output_param->m_DecOutInfo.m_iPictureStructure = 0;

    p_output_param->m_pInitialInfo = &pInst->gsCommDecInfo;
  }
  else if( iOpCode == VDEC_BUF_FLAG_CLEAR )
  {
    return RETCODE_SUCCESS;
  }
  else if( iOpCode == VDEC_DEC_FLUSH_OUTPUT)
  {
    return RETCODE_SUCCESS;
  }
  else if( iOpCode == VDEC_CLOSE )
  {
    if(!pInst->vdec_codec_opened){
      return -RETCODE_NOT_INITIALIZED;
    }
    if ( pInst->gsIsINITdone )
    {
      ret = pInst->gExtDecFunc( VDEC_CLOSE, &pInst->gsWMV78DecHandle, NULL, NULL );
      if( ret != RETCODE_SUCCESS )
      {
        DPRINTF( "[SW-WMV12] WMV78_DEC_CLOSE failed Error code is 0x%x ", ret );
        ret = -ret;
      }
    }

    pInst->vdec_codec_opened = 0;
    pInst->gsIsINITdone = 0;

    if ( pInst->gsWMV78DecInit.m_pHeapAddress ){
      TCC_free(pInst->gsWMV78DecInit.m_pHeapAddress);
    }
    if ( pInst->gsWMV78DecOutput.m_pDecodedData ){
      TCC_free(pInst->gsWMV78DecOutput.m_pDecodedData);
    }
    if ( pInst->gsWMV78DecInit.m_pCurFrameAddress ){
      TCC_free(pInst->gsWMV78DecInit.m_pCurFrameAddress);
    }
    if ( pInst->gsWMV78DecInit.m_pRef0FrameAddress ){
      TCC_free(pInst->gsWMV78DecInit.m_pRef0FrameAddress);
    }

    pInst->gsWMV78DecInit.m_pHeapAddress = 0;
    pInst->gsWMV78DecOutput.m_pDecodedData = 0;
    pInst->gsWMV78DecInit.m_pCurFrameAddress = 0;
    pInst->gsWMV78DecInit.m_pRef0FrameAddress = 0;

    cdk_sys_free_virtual_addr((unsigned long*)pInst->gsFrameBufAddr[VA], pInst->gsFrameBufSize);

    if( pInst->gsWMV78CurYFrameAddress ){
      TCC_free( (unsigned long*)pInst->gsWMV78CurYFrameAddress );
    }
    if( pInst->gsWMV78Ref0YFrameAddress ){
      TCC_free( (unsigned long*)pInst->gsWMV78Ref0YFrameAddress );
    }
    pInst->gsWMV78CurYFrameAddress = 0;
    pInst->gsWMV78Ref0YFrameAddress = 0;

    for(buf_idx =0; buf_idx < pInst->decoded_buf_maxcnt; buf_idx++)
    {
      cdk_sys_free_virtual_addr( (unsigned long*)pInst->decoded_virtAddr[buf_idx], pInst->decoded_buf_size );
    }

    WMV78_CloseLibrary(pInst);
    vpu_env_close(pInst);
  }
  else
  {
    DPRINTF( "[SW-WMV12] Invaild Operation!!" );
    return -ret;
  }

  return ret;
}
#endif //INCLUDE_WMV78_DEC
