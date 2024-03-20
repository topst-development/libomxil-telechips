// SPDX-License-Identifier: LGPL-2.1-or later
/****************************************************************************
 *   FileName    : venc_k.c
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
#ifdef USE_VENC_K

#define LOG_TAG "VPU__ENC__K"

#include "venc.h"

#include <dlfcn.h>

#include <sys/mman.h>
#include <errno.h>
#include <memory.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <tcc_vpu_ioctl.h>
#if defined(TCC_JPU_INCLUDE)
#include <tcc_jpu_ioctl.h>
#endif
#include <errno.h>
#ifdef VPU_CLK_CONTROL
#include "vpu_clk_ctrl.h"
#endif

#define INSERT_SEQ_HEADER_IN_FORCED_IFRAME
#ifdef INSERT_SEQ_HEADER_IN_FORCED_IFRAME
#include "TCCMemory.h"
#endif

#ifdef ENABLE_RATE_CONTROL
//#define CHECK_BITRATE  //to check current output bitrate.
#endif

#define ALIGN_LEN (4*1024)


#define ALOGD(...)	//{(void)g_printf( "[%4d:D]", __LINE__);(void)g_printf(__VA_ARGS__);(void)g_printf(" - [%s - %d]\n",__func__,__LINE__);}
#define ALOGI(...)	{(void)g_printf( "[%4d:I]", __LINE__);(void)g_printf(__VA_ARGS__);(void)g_printf(" - [%s - %d]\n",__func__,__LINE__);}
#define ALOGW(...)	{(void)g_printf( "[%4d:W]", __LINE__);(void)g_printf(__VA_ARGS__);(void)g_printf(" - [%s - %d]\n",__func__,__LINE__);}
#define ALOGE(...)	{(void)g_printf( "[%4d:E]", __LINE__);(void)g_printf(__VA_ARGS__);(void)g_printf(" - [%s - %d]\n",__func__,__LINE__);}

#define LOGD    ALOGD
#define LOGE    ALOGE
#define LOGI    ALOGI
#define LOGW    ALOGW

/************************************************************************/
/* TEST and Debugging                                                                                */
/************************************************************************/
static int DEBUG_ON = 0;
#define DPRINTF(msg...)  ALOGE( ": " msg);
#define DSTATUS(msg...)  if (DEBUG_ON == 1) { ALOGD( ": " msg);}
#define DPRINTF_FRAME(msg...) //ALOGD(": " msg);

//#define VPU_FRAME_DUMP
//#define VPU_OUT_FRAME_DUMP
//#define DEBUG_TIME_LOG
#ifdef DEBUG_TIME_LOG
#include "time.h"
#endif

#define TCC_VPU_INPUT_BUF_SIZE      (1024 * 1024)

#define STABILITY_GAP (512)
// delete by shmin for M2TS
//#define REMOVE_NALSTARTCODE //We will remove NAL StartCode(4byte), because VPU output-Data already has it!!

#define VPU_MGR_NAME    "/dev/vpu_dev_mgr"
char *enc_devices[4] =
{
    "/dev/vpu_venc",
    "/dev/vpu_venc_ext",
    "/dev/vpu_venc_ext2",
    "/dev/vpu_venc_ext3"
};

#ifdef TCC_JPU_INCLUDE
static unsigned int jpu_opened_count = 0;
#define JPU_MGR_NAME    "/dev/jpu_dev_mgr"
#endif

#include <fcntl.h>         // O_RDWR
#include <sys/poll.h>

#include "cdk_error.h"
#else
#include "venc.h"
#endif

#ifdef MULTI_SLICES_AVC
const unsigned char avcAudData[8] = { 0x00,0x00,0x00,0x01,0x09,0x50,0x00,0x00 };
#endif
static unsigned int total_opened_encoder = 0;
static unsigned int vpu_opened_count = 0;
/************************************************************************/
/* STATIC MEMBERS                                                       */
/************************************************************************/
typedef struct _venc_ {
    int venc_instance_index;
    unsigned char venc_env_opened;

    int mgr_fd;
    int enc_fd;
    int codec_format;

    unsigned int total_frm;

    int gsBitWorkBufSize;
    codec_addr_t gsBitWorkBufAddr[3];

    codec_addr_t gsFrameBufAddr[3];
    int gsFrameBufSize;

    int gsMESearchBufSize;
    codec_addr_t gsMESearchBufAddr[3];

    VENC_INIT_t gsVpuEncInit_Info;
    VENC_PUT_HEADER_t gsVpuEncPutHeader_Info;
    VENC_SET_BUFFER_t gsVpuEncBuffer_Info;
    VENC_ENCODE_t gsVpuEncInOut_Info;

#ifdef TCC_JPU_INCLUDE
    JENC_INIT_t gsJpuEncInit_Info;
    JPU_ENCODE_t gsJpuEncInOut_Info;
    JPU_GET_VERSION_t gsJpuEncVersion;
#endif

    codec_addr_t gspSeqHeaderAddr[3];
    int gsiSeqHeaderSize;
    unsigned int gsiSeqHeaderCnt;
    unsigned int gsiSzSeqHeader[3];
    unsigned int gsiFrameIdx;
    int gsBitstreamBufSize;
    codec_addr_t gsBitstreamBufAddr[3];

    //#define MAX_NUM_OF_VIDEO_ELEMENT  VIDEO_ENC_BUFFER_COUNT
    int encoded_buf_size;
    codec_addr_t encoded_buf_base_pos[3];
    codec_addr_t encoded_buf_end_pos[3];
    codec_addr_t encoded_buf_cur_pos[3];
    int keyInterval_cnt;

    unsigned int bAvcUsedNALStart; // added by shmin for M2TS
    unsigned int bWFDTranscodingMode; // Transcoding For WFD Source Device
#ifdef MULTI_SLICES_AVC
    int enc_avc_aud_enable;
    codec_addr_t enc_slice_info_addr[3];
    unsigned int enc_slice_info_size;
#endif

    codec_addr_t yuv_buffer_addr[3];
    unsigned int yuv_buffer_size;

    struct pollfd tcc_event[1];

    int gPFrameCnt;
    int gMaxOutputSize_PFrame;
    int gIFrameCnt;
    int gMaxOutputSize_IFrame;

    unsigned char *seq_backup;
    unsigned int seq_len;
#ifdef CHECK_BITRATE
    unsigned int curr_bps;
    unsigned int bps_frames;
    unsigned int total_size;
    unsigned int curr_fps;
#endif

#ifdef DEBUG_TIME_LOG
    unsigned int enc_time[30];
    unsigned int time_cnt;
    unsigned int total_enc_time;
#endif

} _venc_;


static void vpu_env_close_enc(_venc_ *pInst);

static unsigned int cdk_sys_remain_memory_size_enc(_venc_ *pInst)
{
    unsigned int sz_freeed_mem = pInst->venc_instance_index + VPU_ENC;
    if(ioctl(pInst->enc_fd, VPU_GET_FREEMEM_SIZE, &sz_freeed_mem) < 0) {
        LOGE("ioctl(%d) error[%s]!!", VPU_GET_FREEMEM_SIZE, strerror(errno));
    }
    return sz_freeed_mem;
}

static unsigned int cdk_sys_final_free_mem_enc(_venc_ *pInst)
{
    unsigned int sz_freeed_mem = pInst->venc_instance_index + VPU_ENC;
    if(ioctl(pInst->mgr_fd, VPU_GET_FREEMEM_SIZE, &sz_freeed_mem) < 0 ){
        LOGE("ioctl(%d) error[%s]!!", VPU_GET_FREEMEM_SIZE, strerror(errno));
	}
    return sz_freeed_mem;
}

static void *cdk_sys_malloc_physical_addr_enc(codec_addr_t *remap_addr, int *uiSize, Buffer_Type type, _venc_ *pInst)
{
    MEM_ALLOC_INFO_t alloc_mem;

    memset(&alloc_mem, 0x00, sizeof(MEM_ALLOC_INFO_t));

    alloc_mem.request_size = *uiSize;
    alloc_mem.buffer_type = type;
    if(ioctl(pInst->enc_fd, V_ENC_ALLOC_MEMORY, &alloc_mem) < 0){
        LOGE("ioctl(0x%#x) error[%s]!!  request(0x%#x)/free(0x%#x)",
                V_ENC_ALLOC_MEMORY, strerror(errno), *uiSize, cdk_sys_remain_memory_size_enc(pInst));
    }

    if(remap_addr != NULL){
        *remap_addr = (codec_addr_t)alloc_mem.kernel_remap_addr;
    }
    *uiSize = alloc_mem.request_size;

    return (void *)((uintptr_t)alloc_mem.phy_addr);
}

static void *cdk_sys_malloc_virtual_addr_enc(codec_addr_t pPtr, int uiSize, _venc_ *pInst)
{
    return (void *)mmap(NULL, uiSize, PROT_READ | PROT_WRITE, MAP_SHARED, pInst->enc_fd, pPtr);
}

static void cdk_sys_free_virtual_addr_enc(unsigned long* pPtr, unsigned int uiSize)
{
    if (pPtr != NULL) {
      if((munmap((unsigned long*)pPtr, uiSize)) < 0)
      {
        LOGE("munmap failed. addr(%p), size(%u)", pPtr, uiSize);
      }
      pPtr = NULL;
    }
}

static void vpu_update_sizeinfo_enc(unsigned int image_width, unsigned int image_height, _venc_ *pInst)
{
    CONTENTS_INFO info;

    (void)memset(&info, 0x00, sizeof(CONTENTS_INFO));
    info.type = VPU_ENC;
    info.width = image_width;
    info.height = image_height;

    if(ioctl(pInst->mgr_fd, VPU_SET_CLK, &info) < 0 ){
        LOGE("ioctl(%d) error[%s]!!", VPU_SET_CLK, strerror(errno));
	}
    return;
}

static int vpu_env_open_enc(unsigned int image_width, unsigned int image_height, _venc_ *pInst)
{
    DSTATUS("In  %s \n",__func__);

#ifdef  USE_VPU_INTERRUPT
    vpu_intr_fd = open(TCC_INTR_DEV_NAME, O_RDWR);
    if (vpu_intr_fd < 0) {
        LOGE("%s open error", TCC_INTR_DEV_NAME);
        goto err;
    }
#endif

    if(ioctl(pInst->enc_fd, DEVICE_INITIALIZE, &(pInst->codec_format)) < 0){
        LOGE("ioctl(%d) error[%s]!!", DEVICE_INITIALIZE, strerror(errno));
    }

    vpu_update_sizeinfo_enc(image_width, image_height, pInst);

    if(pInst->bWFDTranscodingMode == 1)
    {
        ALOGI("[VENC_K] : Set VPU_USE_WAIT_LIST for Interlaced contents....");
        int enable = 1;
        if (ioctl(pInst->mgr_fd, VPU_USE_WAIT_LIST, &enable) < 0){
            LOGE("ioctl(%d) error[%s]!!", VPU_USE_WAIT_LIST, strerror(errno));
        }
    }

    pInst->venc_env_opened = 1;
    pInst->gsiFrameIdx = 0;
    pInst->gPFrameCnt = pInst->gMaxOutputSize_PFrame = 0;
    pInst->gIFrameCnt = pInst->gMaxOutputSize_IFrame = 0;
    pInst->gsiSeqHeaderCnt = 0;

    DSTATUS("Out  %s \n",__func__);

#ifdef INSERT_SEQ_HEADER_IN_FORCED_IFRAME
    pInst->seq_backup = NULL;
    pInst->seq_len = 0;
#endif

#ifdef DEBUG_TIME_LOG
    pInst->time_cnt = 0;
    pInst->total_enc_time = 0;
#endif
    pInst->total_frm = 0;

    return 0;

#ifdef  USE_VPU_INTERRUPT
err:
    LOGE("vpu_env_open_enc error");
    vpu_env_close_enc(pInst);

    return -1;
#endif

}


static void vpu_env_close_enc(_venc_ *pInst)
{
    DSTATUS("In  %s \n",__func__);

#ifdef INSERT_SEQ_HEADER_IN_FORCED_IFRAME
    if(pInst->seq_backup != NULL)
    {
        TCC_free(pInst->seq_backup);
        pInst->seq_backup = NULL;
        pInst->seq_len = 0;
    }
#endif

#ifdef  USE_VPU_INTERRUPT
    if(vpu_intr_fd > 0)
    {
        if(close(vpu_intr_fd) < 0)
        {
            LOGE("%s close error", TCC_INTR_DEV_NAME);
        }
        vpu_intr_fd = -1;
    }
#endif

    if(pInst->bWFDTranscodingMode == 1)
    {
        ALOGI("[VENC_K] : reSet VPU_USE_WAIT_LIST for Interlaced contents....");
        int enable = 0;
        if (ioctl(pInst->mgr_fd, VPU_USE_WAIT_LIST, &enable) < 0){
            LOGE("ioctl(%d) error[%s]!!", VPU_USE_WAIT_LIST, strerror(errno));
        }
    }

    if( 0 > ioctl(pInst->enc_fd, V_ENC_FREE_MEMORY, NULL))
    {
        LOGE("ioctl(%d) error[%s]!!", V_ENC_FREE_MEMORY, strerror(errno));
    }
    pInst->venc_env_opened = 0;

    DSTATUS("Out  %s \n",__func__);

}

static void filewrite_memory(char* name, char* addr, int size)
{
#ifdef VPU_FRAME_DUMP
    FILE *fp;

    if(!bFirst_frame) {
        return;
    }

    fp = fopen(name, "ab+");
    fwrite( addr, size, 1, fp);
    fclose(fp);
#else
    (void)name;
    (void)addr;
    (void)size;
#endif

}

static void save_output_stream(char* name, int size, unsigned char* addr)
{
#ifdef VPU_OUT_FRAME_DUMP

    int i;
    unsigned char* ps = (unsigned char*)addr;

    if(1)
    {
        FILE *fp;
        fp = fopen(name, "ab+");
        fwrite( ps, size, 1, fp);
        fclose(fp);

        return;
    }

    for(i=0; (i+10 <size) && (i+10 < 100); i += 10){
        DPRINTF_FRAME( "[VENC - Stream] 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x", ps[i], ps[i+1], ps[i+2], ps[i+3], ps[i+4], ps[i+5], ps[i+6], ps[i+7], ps[i+8], ps[i+9] );
    }
#else
    (void)name;
    (void)addr;
    (void)size;
#endif
}

unsigned char *vpu_get_YuvBufAddr(unsigned int index, void *pVenc)
{
    unsigned char *pYuvBufAddr = NULL;
    _venc_ * pInst = pVenc;

    if(!pInst){
        ALOGE("vpu_get_YuvBufAddr :: Instance is null!!");
        return NULL;
    }

    if (index == PA)
    {
        pYuvBufAddr = (unsigned char *)pInst->yuv_buffer_addr[PA];
    }
    else if (index == VA)
    {
        pYuvBufAddr = (unsigned char *)pInst->yuv_buffer_addr[VA];
    }
    else /* default : PA */
    {
        pYuvBufAddr = (unsigned char *)pInst->yuv_buffer_addr[PA];
    }

    return pYuvBufAddr;
}

unsigned char *vpu_getStreamOutVirtAddr(unsigned char *convert_addr, unsigned int base_index, void *pVenc)
{
    unsigned char *pBaseAddr;
    unsigned char *pTargetBaseAddr = NULL;
    unsigned int szAddrGap = 0;
    _venc_ * pInst = pVenc;

    if(!pInst){
        ALOGE("vpu_getStreamOutPhyAddr :: Instance is null!!");
        return NULL;
    }

    pTargetBaseAddr = (unsigned char*)pInst->encoded_buf_base_pos[VA];

    if (base_index == K_VA)
    {
        pBaseAddr = (unsigned char*)pInst->encoded_buf_base_pos[K_VA];
    }
    else /* default : VA */
    {
        pBaseAddr = (unsigned char*)pInst->encoded_buf_base_pos[PA];
    }

    szAddrGap = convert_addr - pBaseAddr;

    return (pTargetBaseAddr+szAddrGap);
}

unsigned char *vpu_getStreamOutPhyAddr(unsigned char *convert_addr, unsigned int base_index, void *pVenc)
{
    unsigned char *pBaseAddr;
    unsigned char *pTargetBaseAddr = NULL;
    unsigned int szAddrGap = 0;
    _venc_ * pInst = pVenc;

    if(!pInst){
        ALOGE("vpu_getStreamOutPhyAddr :: Instance is null!!");
        return NULL;
    }

    pTargetBaseAddr = (unsigned char*)pInst->encoded_buf_base_pos[PA];

    if (base_index == K_VA)
    {
        pBaseAddr = (unsigned char*)pInst->encoded_buf_base_pos[K_VA];
    }
    else /* default : VA */
    {
        pBaseAddr = (unsigned char*)pInst->encoded_buf_base_pos[VA];
    }

    szAddrGap = convert_addr - pBaseAddr;

    return (pTargetBaseAddr+szAddrGap);
}

unsigned char *vpu_getSeqHeaderPhyAddr(unsigned char *convert_addr, unsigned int base_index, void *pVenc)
{
    unsigned char *pBaseAddr;
    unsigned char *pTargetBaseAddr = NULL;
    unsigned int szAddrGap = 0;
    _venc_ * pInst = pVenc;

    if(!pInst){
        ALOGE("vpu_getSeqHeaderPhyAddr :: Instance is null!!");
        return NULL;
    }

    pTargetBaseAddr = (unsigned char*)pInst->gspSeqHeaderAddr[PA];

    if (base_index == K_VA)
    {
        pBaseAddr = (unsigned char*)pInst->gspSeqHeaderAddr[K_VA];
    }
    else /* default : VA */
    {
        pBaseAddr = (unsigned char*)pInst->gspSeqHeaderAddr[VA];
    }

    szAddrGap = convert_addr - pBaseAddr;

    return (pTargetBaseAddr+szAddrGap);
}

#ifdef USE_VPU_INTERRUPT
static void write_reg(unsigned int addr, unsigned int val)
{
    *((volatile unsigned int *)(gsRegisterBase + addr)) = (unsigned int)(val);
}

static unsigned int read_reg(unsigned int addr)
{
    return *(volatile unsigned int *)(gsRegisterBase + addr);
}

static int VpuInterrupt()
{
    int iSuccess = 0;
    int ret;

    while (1) {
        memset(pInst->tcc_event, 0, sizeof(pInst->tcc_event));
        pInst->tcc_event[0].fd = vpu_intr_fd;
        pInst->tcc_event[0].events = POLLIN;

        ret = poll((struct pollfd *)&pInst->tcc_event, 1, 500); // 500 msec
        if (ret < 0) {
            LOGE("vpu poll error\n");
            break;
        }else if (ret == 0) {
            LOGE("vpu poll timeout\n");
            break;
        }else if (ret > 0) {
            if (pInst->tcc_event[0].revents & POLLERR) {
                LOGE("vpu poll POLLERR\n");
                break;
            } else if (pInst->tcc_event[0].revents & POLLIN) {
                iSuccess = 1;
                break;
            }
        }
    }
    /* todo */

    write_reg(0x174, 0);
    write_reg(0x00C, 1);

    if(iSuccess == 1) {
        ret = RETCODE_SUCCESS;
	} else {
        ret = RETCODE_CODEC_EXIT;
	}
    return ret;
}
#endif

static int venc_cmd_process(int cmd, void* args, _venc_ *pInst)
{
    int ret;
    int iSuccess = 0;
    int retry_cnt = 10;
    int all_retry_cnt = 3;

    if((ret = ioctl(pInst->enc_fd, cmd, args)) < 0)
    {
        if( ret == -0x999 )
        {
            LOGE("VPU[%d] Invalid command(%d) ", pInst->venc_instance_index, cmd);
            return RETCODE_INVALID_COMMAND;
        }
        else
        {
            LOGE("VPU[%d] ioctl err[%s] : cmd = %d", pInst->venc_instance_index, strerror(errno), cmd);
        }
    }

Retry:
    while (retry_cnt > 0) {
        memset(pInst->tcc_event, 0, sizeof(pInst->tcc_event));
        pInst->tcc_event[0].fd = pInst->enc_fd;
        pInst->tcc_event[0].events = POLLIN;

        ret = poll((struct pollfd *)&pInst->tcc_event, 1, 10000); // 100 msec
        if (ret < 0) {
            LOGE("vpu(%d)-retry(%d:cmd(%d)) poll error '%s'", cmd, retry_cnt, cmd, strerror(errno));
            retry_cnt--;
            continue;
        }else if (ret == 0) {
            LOGE("vpu(%d)-retry(%d:cmd(%d)) poll timeout", cmd, retry_cnt, cmd);
            retry_cnt--;
            continue;
        }else if (ret > 0) {
            if (pInst->tcc_event[0].revents & POLLERR) {
                LOGE("vpu(%d) poll POLLERR", cmd);
                break;
            } else if (pInst->tcc_event[0].revents & POLLIN) {
                iSuccess = 1;
                break;
            }
        }
    }

    switch(cmd)
    {
        case V_ENC_INIT:
            {
                VENC_INIT_t* init_info = args;

                if(ioctl(pInst->enc_fd, V_ENC_INIT_RESULT, args) < 0){
                    LOGE("ioctl(%d) error[%s]!!", V_ENC_INIT_RESULT, strerror(errno));
                }
                ret = init_info->result;
            }
            break;

        case V_ENC_PUT_HEADER:
            {
                VENC_PUT_HEADER_t* buff_info = args;

                if(ioctl(pInst->enc_fd, V_ENC_PUT_HEADER_RESULT, args) < 0 ){
                    LOGE("ioctl(%d) error[%s]!!", V_ENC_PUT_HEADER_RESULT, strerror(errno));
                }
                ret = buff_info->result;
            }
            break;

        case V_ENC_ENCODE:
            {
                VENC_ENCODE_t* encoded_info = args;

                if(ioctl(pInst->enc_fd, V_ENC_ENCODE_RESULT, args) < 0){
                    LOGE("ioctl(%d) error[%s]!!", V_ENC_ENCODE_RESULT, strerror(errno));
                }
                ret = encoded_info->result;
            }
            break;

        case V_ENC_REG_FRAME_BUFFER:
        case V_ENC_CLOSE:
        default:
            if(ioctl(pInst->enc_fd, V_ENC_GENERAL_RESULT, &ret) < 0){
                LOGE("ioctl(%d) error[%s]!!", V_ENC_GENERAL_RESULT, strerror(errno));
            }
            break;
    }

    if((ret&0xf000) != 0x0000){ //If there is an invalid return, we skip it because this return means that vpu didn't process current command yet.
        all_retry_cnt--;
        if( all_retry_cnt > 0)
        {
            retry_cnt = 10;
            goto Retry;
        }
        else
        {
            LOGE("abnormal exception!!");
        }
    }

    /* todo */
    if((iSuccess == 0)
        || ((ret&0xf000) != 0x0000) /* vpu can not start or finish its processing with unknown reason!! */
    )
    {
        LOGE("VENC command(%d) didn't work properly. maybe hangup(no return(%d))!!", cmd, ret);
#if 0
        if(ret != RETCODE_CODEC_EXIT && ret != RETCODE_MULTI_CODEC_EXIT_TIMEOUT){
//          ioctl(pInst->mgr_fd, VPU_HW_RESET, (void*)NULL);
        }
#endif
        ret = RETCODE_CODEC_EXIT;
    }

    return ret;
}

int
venc_vpu( int iOpCode, unsigned long* pHandle, void* pParam1, void* pParam2, void * pParam3 )
{
    int ret = 0;
    _venc_ *pInst = (_venc_ *)pParam3;
    (void)pHandle;

    if(!pInst){
        ALOGE("venc_vpu(OP:%d) :: Instance is null!!", iOpCode);
        return -RETCODE_NOT_INITIALIZED;
    }

    if( ( iOpCode == VENC_INIT ) && (pInst->venc_env_opened) )
    {
        ALOGE("venc_vpu(OP:%d) :: VENC has been already opened. so have to close!!", iOpCode);
        return -VPU_ENV_INIT_ERROR;
    }
#ifdef DEBUG_TIME_LOG
    clock_t start, end;
    start = clock();
#endif

    if( iOpCode == VENC_INIT )
    {
        venc_init_t* p_init_param = (venc_init_t*)pParam1;

        pInst->bWFDTranscodingMode  = p_init_param->m_iWFDTranscoding;
        if(vpu_env_open_enc(p_init_param->m_iPicWidth, p_init_param->m_iPicHeight, pInst ) < 0) {
            return -VPU_ENV_INIT_ERROR;
        }

#if defined(VPU_CLK_CONTROL)
        vpu_clock_init();
#endif

        // added by shmin for M2TS
        pInst->bAvcUsedNALStart = *((unsigned int*)pParam2);

        pInst->gsVpuEncInit_Info.gsVpuEncInit.m_RegBaseVirtualAddr  = (unsigned int)NULL;
        //pInst->gsVpuEncInit_Info.gsVpuEncInit.m_BitstreamBufferAddr   = p_init_param->m_BitstreamBufferAddr;
        //pInst->gsVpuEncInit_Info.gsVpuEncInit.m_BitstreamBufferAddr_VA = p_init_param->m_BitstreamBufferAddr_VA;
        //pInst->gsVpuEncInit_Info.gsVpuEncInit.m_iBitstreamBufferSize  = p_init_param->m_iBitstreamBufferSize;

        pInst->gsVpuEncInit_Info.gsVpuEncInit.m_iBitstreamFormat    = p_init_param->m_iBitstreamFormat;
        pInst->gsVpuEncInit_Info.gsVpuEncInit.m_iPicWidth           = p_init_param->m_iPicWidth;
        pInst->gsVpuEncInit_Info.gsVpuEncInit.m_iPicHeight          = p_init_param->m_iPicHeight;
        pInst->gsVpuEncInit_Info.gsVpuEncInit.m_iFrameRate          = p_init_param->m_iFrameRate;
        pInst->gsVpuEncInit_Info.gsVpuEncInit.m_iTargetKbps         = p_init_param->m_iTargetKbps;
        pInst->gsVpuEncInit_Info.gsVpuEncInit.m_iKeyInterval        = p_init_param->m_iKeyInterval;// only first picture is I
        pInst->gsVpuEncInit_Info.gsVpuEncInit.m_iUseSpecificRcOption = 1;

        pInst->gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iAvcFastEncoding     = p_init_param->m_iAvcFastEncoding;
        pInst->gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iPicQpY              = -1;
        pInst->gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iIntraMBRefresh      = 0;
        pInst->gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iDeblkDisable        = 0;
        pInst->gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iDeblkAlpha          = 0;
        pInst->gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iDeblkBeta           = 0;
        pInst->gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iDeblkChQpOffset     = 0;
        pInst->gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iConstrainedIntra    = 0;
        pInst->gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iVbvBufferSize       = 0;
        pInst->gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iSearchRange         = 2;
        pInst->gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iPVMDisable          = 0;
        pInst->gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iWeightIntraCost     = 0;
        pInst->gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iRCIntervalMode      = 1;
        pInst->gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iRCIntervalMBNum     = 0;
        pInst->gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iEncQualityLevel     = 11;

        pInst->gsVpuEncInit_Info.gsVpuEncInit.m_bEnableVideoCache               = 0;
        pInst->gsVpuEncInit_Info.gsVpuEncInit.m_bCbCrInterleaveMode             = 0;
        pInst->gsVpuEncInit_Info.gsVpuEncInit.m_uiEncOptFlags                   = 0; //(1 << 10 );

#ifdef MULTI_SLICES_AVC
        if( pInst->gsVpuEncInit_Info.gsVpuEncInit.m_iBitstreamFormat == STD_AVC )
        {
            pInst->gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iSliceMode       = p_init_param->m_iSliceMode;       // multi-slices per picture
            pInst->gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iSliceSizeMode   = p_init_param->m_iSliceSizeMode;
            pInst->gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iSliceSize       = p_init_param->m_iSliceSize;

            if( pInst->gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iSliceMode == 1 ) {
                pInst->enc_avc_aud_enable = 1;
			} else {
                pInst->enc_avc_aud_enable = 0;
            }
        }
        else
#endif
        {
            pInst->gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iSliceMode       = 0;        // 1 slice per picture
            pInst->gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iSliceSizeMode   = 0;
            pInst->gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iSliceSize       = 0;
#ifdef MULTI_SLICES_AVC
            pInst->enc_avc_aud_enable = 0;
#endif
        }
        DSTATUS( "SliceMode[%d] - SizeMode[%d] - %d",
                  pInst->gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iSliceMode,
                  pInst->gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iSliceSizeMode,
                  pInst->gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iSliceSize );

        pInst->gsVpuEncInit_Info.gsVpuEncInit.m_Memcpy              = (void* (*) ( void*, const void*, unsigned int ))memcpy;
        pInst->gsVpuEncInit_Info.gsVpuEncInit.m_Memset              = (void  (*) ( void*, int, unsigned int ))memset;
        pInst->gsVpuEncInit_Info.gsVpuEncInit.m_Interrupt           = (int  (*) ( void ))NULL;

        pInst->keyInterval_cnt = 0;

        //------------------------------------------------------------
        //! [x] bitstream buffer for each VPU decoder
        //------------------------------------------------------------
        pInst->gsBitstreamBufSize = LARGE_STREAM_BUF_SIZE;
        pInst->gsBitstreamBufSize = ALIGNED_BUFF( pInst->gsBitstreamBufSize, ALIGN_LEN );

#ifdef PASS_BUFFER_TO_UPPER_LAYER
        pInst->encoded_buf_size = (pInst->gsVpuEncInit_Info.gsVpuEncInit.m_iTargetKbps/8 /*KB/s*/) * 1024/*Byte*/ * (VIDEO_ENC_BUFFER_COUNT/pInst->gsVpuEncInit_Info.gsVpuEncInit.m_iFrameRate +3 /*add 3sec*/ );
        if(pInst->encoded_buf_size < (LARGE_STREAM_BUF_SIZE * 2)) { //4MB
            pInst->encoded_buf_size = (LARGE_STREAM_BUF_SIZE * 2);
        }
        pInst->encoded_buf_size = ALIGNED_BUFF(pInst->encoded_buf_size, ALIGN_LEN);
#else
        pInst->encoded_buf_size = pInst->gsBitstreamBufSize;
#endif
        pInst->gsBitstreamBufAddr[PA] = (codec_addr_t)cdk_sys_malloc_physical_addr_enc( &pInst->gsBitstreamBufAddr[K_VA], &pInst->encoded_buf_size, BUFFER_ELSE, pInst );
        if( pInst->gsBitstreamBufAddr[PA] == 0 )
        {
            LOGE( "[VENC] bitstream_buf_addr[PA] malloc() failed \n");
            return -1;
        }
        DSTATUS( "[VENC] bitstream_buf_addr[PA] = 0x%zx, %d \n", (size_t)pInst->gsBitstreamBufAddr[PA], pInst->encoded_buf_size );
        pInst->gsBitstreamBufAddr[VA] = (codec_addr_t)cdk_sys_malloc_virtual_addr_enc(  pInst->gsBitstreamBufAddr[PA], pInst->encoded_buf_size, pInst );
        if( pInst->gsBitstreamBufAddr[VA] == 0 )
        {
            LOGE( "[VENC] bitstream_buf_addr[VA] malloc() failed \n");
            return -1;
        }
        //memset( (void*)pInst->gsBitstreamBufAddr[VA], 0x00 , pInst->gsBitstreamBufSize);
        DSTATUS("[VENC] bitstream_buf_addr[VA] = 0x%zx, %d \n", (size_t)pInst->gsBitstreamBufAddr[VA], pInst->encoded_buf_size );

        pInst->gsVpuEncInit_Info.gsVpuEncInit.m_BitstreamBufferAddr     = pInst->gsBitstreamBufAddr[PA];
        pInst->gsVpuEncInit_Info.gsVpuEncInit.m_BitstreamBufferAddr_VA  = pInst->gsBitstreamBufAddr[K_VA];
        pInst->gsVpuEncInit_Info.gsVpuEncInit.m_iBitstreamBufferSize        = pInst->gsBitstreamBufSize;

        //------------------------------------------------------------
        //! [x] code buffer, work buffer and parameter buffer for VPU
        //------------------------------------------------------------
        pInst->gsBitWorkBufSize = WORK_CODE_PARA_BUF_SIZE;
        pInst->gsBitWorkBufSize = ALIGNED_BUFF(pInst->gsBitWorkBufSize, ALIGN_LEN);
        pInst->gsBitWorkBufAddr[PA] = (codec_addr_t)cdk_sys_malloc_physical_addr_enc( &pInst->gsBitWorkBufAddr[K_VA], &pInst->gsBitWorkBufSize, BUFFER_WORK, pInst );
        if( pInst->gsBitWorkBufAddr[PA] == 0 )
        {
            LOGE( "[VENC] pInst->gsBitWorkBufAddr[PA] malloc() failed \n");
            return -1;
        }
        DSTATUS("[VENC] pInst->gsBitWorkBufAddr[PA] = 0x%zx, %d \n", (size_t)pInst->gsBitWorkBufAddr[PA], pInst->gsBitWorkBufSize );
        pInst->gsBitWorkBufAddr[VA] = (codec_addr_t)cdk_sys_malloc_virtual_addr_enc(  pInst->gsBitWorkBufAddr[PA], pInst->gsBitWorkBufSize, pInst );
        if( pInst->gsBitWorkBufAddr[VA] == 0 )
        {
            LOGE( "[VENC] pInst->gsBitWorkBufAddr[VA] malloc() failed \n");
            return -1;
        }
        DSTATUS("[VENC] pInst->gsBitWorkBufAddr[VA] = 0x%zx, %d \n", (size_t)pInst->gsBitWorkBufAddr[VA], pInst->gsBitWorkBufSize );

        //------------------------------------------------------------
        //! [x] me search buffer for each VPU encoder
        //------------------------------------------------------------
        //! Estimate size
        pInst->gsMESearchBufSize = ( ( pInst->gsVpuEncInit_Info.gsVpuEncInit.m_iPicWidth + 15 ) & ~15 ) * 36 + 2048;  // picWidth of searchram size must be a multiple of 16
        pInst->gsMESearchBufSize = ALIGNED_BUFF( pInst->gsMESearchBufSize, ALIGN_LEN );
        //DSTATUS( "[CDK_CORE] pInst->gsMESearchBufSize = %d\n", pInst->gsMESearchBufSize );
        pInst->gsMESearchBufAddr[PA] = (codec_addr_t)cdk_sys_malloc_physical_addr_enc( &pInst->gsMESearchBufAddr[K_VA], &pInst->gsMESearchBufSize, BUFFER_ELSE, pInst);
        if( pInst->gsMESearchBufAddr[PA] == 0 )
        {
            LOGE( "[CDK_CORE] pInst->gsMESearchBufAddr[PA] physical malloc() failed \n");
            return CDK_ERR_MALLOC;
        }
        DSTATUS("[CDK_CORE] pInst->gsMESearchBufAddr[PA] = 0x%zx, %d \n", (size_t)pInst->gsMESearchBufAddr[PA], pInst->gsMESearchBufSize );
        pInst->gsMESearchBufAddr[VA] = (codec_addr_t)cdk_sys_malloc_virtual_addr_enc(  pInst->gsMESearchBufAddr[PA], pInst->gsMESearchBufSize, pInst );
        if( pInst->gsMESearchBufAddr[VA] == 0 )
        {
            LOGE( "[CDK_CORE] pInst->gsMESearchBufAddr[VA] virtual malloc() failed \n");
            return CDK_ERR_MALLOC;
        }
        DSTATUS("[CDK_CORE] pInst->gsMESearchBufAddr[VA] = 0x%zx, %d \n", (size_t)pInst->gsMESearchBufAddr[VA], pInst->gsMESearchBufSize );

        pInst->gsVpuEncInit_Info.gsVpuEncInit.m_BitWorkAddr[PA]     = pInst->gsBitWorkBufAddr[PA];
        pInst->gsVpuEncInit_Info.gsVpuEncInit.m_BitWorkAddr[VA]     = pInst->gsBitWorkBufAddr[K_VA];
        //pInst->gsVpuEncInit_Info.gsVpuEncInit.m_MeSearchRamAddr       = pInst->gsMESearchBufAddr[PA];
        //pInst->gsVpuEncInit_Info.gsVpuEncInit.m_iMeSearchRamSize      = pInst->gsMESearchBufSize;

#ifdef MULTI_SLICES_AVC
        if( pInst->gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iSliceMode == 1 )
        {
            //------------------------------------------------------------
            //! [x] Slice info. buffers requested by the encoder.
            //------------------------------------------------------------
            int iMbWidth, iMbHeight;

            iMbWidth = (pInst->gsVpuEncInit_Info.gsVpuEncInit.m_iPicWidth+15)>>4;
            iMbHeight = (pInst->gsVpuEncInit_Info.gsVpuEncInit.m_iPicHeight+15)>>4;

            pInst->enc_slice_info_size = iMbWidth * iMbHeight * 8 + 48;
            pInst->enc_slice_info_size = ALIGNED_BUFF(pInst->enc_slice_info_size, ALIGN_LEN);
            pInst->enc_slice_info_addr[PA] = (codec_addr_t)cdk_sys_malloc_physical_addr_enc( &pInst->enc_slice_info_addr[K_VA], (int*)&pInst->enc_slice_info_size, BUFFER_ELSE, pInst );
            if( pInst->enc_slice_info_addr[PA] == 0 )
            {
                LOGE( "[CDK_CORE] pInst->enc_slice_info_addr[PA] physical malloc() failed \n");
                return CDK_ERR_MALLOC;
            }

            DSTATUS("[CDK_CORE] pInst->enc_slice_info_addr[PA] = 0x%zx, %d \n", (size_t)pInst->enc_slice_info_addr[PA], pInst->enc_slice_info_size );
            pInst->enc_slice_info_addr[VA] = (codec_addr_t)cdk_sys_malloc_virtual_addr_enc(  pInst->enc_slice_info_addr[PA], pInst->enc_slice_info_size, pInst );
            if( pInst->enc_slice_info_addr[VA] == 0 )
            {
                LOGE( "[CDK_CORE] pInst->enc_slice_info_addr[VA] virtual malloc() failed \n");
                return CDK_ERR_MALLOC;
            }
            DSTATUS("[CDK_CORE] pInst->enc_slice_info_addr[VA] = 0x%zx, %d \n", (size_t)pInst->enc_slice_info_addr[VA], pInst->enc_slice_info_size );
        }
#endif

        {
            //------------------------------------------------------------
            //! [x] input buffer for YUV raw image.
            //------------------------------------------------------------
            int iMbWidth, iMbHeight;

            iMbWidth = ((pInst->gsVpuEncInit_Info.gsVpuEncInit.m_iPicWidth+15)>>4)<<4;
            iMbHeight = ((pInst->gsVpuEncInit_Info.gsVpuEncInit.m_iPicHeight+15)>>4)<<4;

            pInst->yuv_buffer_size = iMbWidth * iMbHeight * 3 / 2;
            pInst->yuv_buffer_size = ALIGNED_BUFF(pInst->yuv_buffer_size, ALIGN_LEN);
            pInst->yuv_buffer_addr[PA] = (codec_addr_t)cdk_sys_malloc_physical_addr_enc( &pInst->yuv_buffer_addr[K_VA], (int*)&pInst->yuv_buffer_size, BUFFER_ELSE, pInst );
            if( pInst->yuv_buffer_addr[PA] == 0 )
            {
                LOGE( "[CDK_CORE] pInst->yuv_buffer_addr[PA] physical malloc() failed \n");
                return CDK_ERR_MALLOC;
            }

            DSTATUS("[CDK_CORE] pInst->yuv_buffer_addr[PA] = 0x%zx, %d \n", (size_t)pInst->yuv_buffer_addr[PA], pInst->yuv_buffer_size );
            pInst->yuv_buffer_addr[VA] = (codec_addr_t)cdk_sys_malloc_virtual_addr_enc(  pInst->yuv_buffer_addr[PA], pInst->yuv_buffer_size, pInst );
            if( pInst->yuv_buffer_addr[VA] == 0 )
            {
                LOGE( "[CDK_CORE] pInst->yuv_buffer_addr[VA] virtual malloc() failed \n");
                return CDK_ERR_MALLOC;
            }
            DSTATUS("[CDK_CORE] pInst->yuv_buffer_addr[VA] = 0x%zx, %d \n", (size_t)pInst->yuv_buffer_addr[VA], pInst->yuv_buffer_size );
        }

        pInst->encoded_buf_base_pos[PA] = pInst->gsBitstreamBufAddr[PA];
        pInst->encoded_buf_base_pos[VA]     = pInst->gsBitstreamBufAddr[VA];
        pInst->encoded_buf_base_pos[K_VA]   = pInst->gsBitstreamBufAddr[K_VA];
        pInst->encoded_buf_cur_pos[PA]  = pInst->encoded_buf_base_pos[PA];
        pInst->encoded_buf_cur_pos[VA]  = pInst->encoded_buf_base_pos[VA];
        pInst->encoded_buf_cur_pos[K_VA]    = pInst->encoded_buf_base_pos[K_VA];
        pInst->encoded_buf_end_pos[PA]  = pInst->encoded_buf_base_pos[PA] + pInst->encoded_buf_size;
        pInst->encoded_buf_end_pos[VA]  = pInst->encoded_buf_base_pos[VA] + pInst->encoded_buf_size;
        pInst->encoded_buf_end_pos[K_VA]    = pInst->encoded_buf_base_pos[K_VA] + pInst->encoded_buf_size;

        DSTATUS("Stream out-Buffer ::   %d Kbps, %d fps!!\n", pInst->gsVpuEncInit_Info.gsVpuEncInit.m_iTargetKbps, pInst->gsVpuEncInit_Info.gsVpuEncInit.m_iFrameRate);
        DSTATUS("               PA = 0x%zx, VA = 0x%zx, size = %d!!\n", (size_t)pInst->encoded_buf_base_pos[PA], (size_t)pInst->encoded_buf_base_pos[VA],   pInst->encoded_buf_size);

        ret = venc_cmd_process(V_ENC_INIT, &pInst->gsVpuEncInit_Info, pInst);

        if( ret != RETCODE_SUCCESS )
        {
            LOGE( "[VENC,Err:%d] venc_vpu VPU_ENC_INIT failed \n", ret );
            return -ret;
        }
        DSTATUS("[VENC] venc_vpu VPU_ENC_INIT ok! \n" );

        //------------------------------------------------------------
        //! [x] Register frame buffers requested by the encoder.
        //------------------------------------------------------------
        pInst->gsFrameBufSize = pInst->gsVpuEncInit_Info.gsVpuEncInitialInfo.m_iMinFrameBufferCount * pInst->gsVpuEncInit_Info.gsVpuEncInitialInfo.m_iMinFrameBufferSize;
        pInst->gsFrameBufSize = ALIGNED_BUFF(pInst->gsFrameBufSize, ALIGN_LEN);
        pInst->gsFrameBufAddr[PA] = (codec_addr_t)cdk_sys_malloc_physical_addr_enc( &pInst->gsFrameBufAddr[K_VA], &pInst->gsFrameBufSize, BUFFER_ELSE, pInst );
        if( pInst->gsFrameBufAddr[PA] == 0 )
        {
            LOGE( "[VENC,Err:%d] venc_vpu pInst->gsFrameBufAddr[PA](%d) alloc failed \n", ret, pInst->gsFrameBufSize );
            return -ret;
        }
        DSTATUS("[VENC] pInst->gsFrameBufAddr[PA] = 0x%zx, %d((%d min) * %d bytes) \n", (size_t)pInst->gsFrameBufAddr[PA], pInst->gsFrameBufSize, pInst->gsVpuEncInit_Info.gsVpuEncInitialInfo.m_iMinFrameBufferCount, pInst->gsVpuEncInit_Info.gsVpuEncInitialInfo.m_iMinFrameBufferSize);
        pInst->gsFrameBufAddr[VA] = (codec_addr_t)cdk_sys_malloc_virtual_addr_enc(  pInst->gsFrameBufAddr[PA], pInst->gsFrameBufSize, pInst );
        if( pInst->gsFrameBufAddr[VA] == 0 )
        {
            LOGE( "[VENC,Err:%d] venc_vpu pInst->gsFrameBufAddr[VA] alloc failed \n", ret );
            return -ret;
        }
        DSTATUS("[VENC] pInst->gsFrameBufAddr[VA] = 0x%zx, pInst->gsFrameBufAddr[K_VA] = 0x%zx \n", (size_t)pInst->gsFrameBufAddr[VA], (size_t)pInst->gsFrameBufAddr[K_VA] );

        pInst->gsVpuEncBuffer_Info.gsVpuEncBuffer.m_FrameBufferStartAddr[PA] = pInst->gsFrameBufAddr[PA];
        pInst->gsVpuEncBuffer_Info.gsVpuEncBuffer.m_FrameBufferStartAddr[VA] = pInst->gsFrameBufAddr[K_VA];

        ret = venc_cmd_process(V_ENC_REG_FRAME_BUFFER, &pInst->gsVpuEncBuffer_Info, pInst);  // register frame buffer

        if( ret != RETCODE_SUCCESS )
        {
            LOGE( "[VENC,Err:%d] venc_vpu VPU_ENC_REG_FRAME_BUFFER failed \n", ret );
            return -ret;
        }

        DSTATUS("[VENC] venc_vpu VPU_ENC_REG_FRAME_BUFFER ok! \n" );
#ifdef CHECK_BITRATE
        pInst->curr_bps = pInst->gsVpuEncInit_Info.gsVpuEncInit.m_iTargetKbps;
        pInst->curr_fps = pInst->gsVpuEncInit_Info.gsVpuEncInit.m_iFrameRate;
        pInst->bps_frames = pInst->total_size = 0;
#endif
    }
    else if( iOpCode == VENC_SEQ_HEADER )
    {
        venc_seq_header_t* p_seq_param = (venc_seq_header_t*)pParam1;
        unsigned char* p_dest = NULL;
        int i_dest_size = 0;
        codec_addr_t m_SeqHeaderBuffer_VA = 0;

        if(pInst->gspSeqHeaderAddr[PA] == 0)
        {
            pInst->gsiSeqHeaderSize = p_seq_param->m_iSeqHeaderBufferSize;

            if(pInst->gsiSeqHeaderSize == 0)
            {
                pInst->gsiSeqHeaderSize = ALIGNED_BUFF( 100*1024, ALIGN_LEN );
            }

    #ifdef INSERT_SEQ_HEADER_IN_FORCED_IFRAME
            if(pInst->seq_backup == NULL){
                pInst->seq_backup = (unsigned char*)TCC_malloc(pInst->gsiSeqHeaderSize);
                pInst->seq_len = 0;
            }
    #endif

            pInst->gspSeqHeaderAddr[PA] = (codec_addr_t)cdk_sys_malloc_physical_addr_enc( &pInst->gspSeqHeaderAddr[K_VA], &pInst->gsiSeqHeaderSize, BUFFER_SEQHEADER, pInst );
            if( pInst->gspSeqHeaderAddr[PA] == 0 )
            {
                LOGE( "[VENC] pInst->gspSeqHeaderAddr[PA] malloc() failed \n");
                return -1;
            }
            DSTATUS( "[VENC] pInst->gspSeqHeaderAddr[PA] = 0x%zx, %d \n", (size_t)pInst->gspSeqHeaderAddr[PA], pInst->gsiSeqHeaderSize );
            pInst->gspSeqHeaderAddr[VA] = (codec_addr_t)cdk_sys_malloc_virtual_addr_enc(  pInst->gspSeqHeaderAddr[PA], pInst->gsiSeqHeaderSize, pInst );
            if( pInst->gspSeqHeaderAddr[VA] == 0 )
            {
                LOGE( "[VENC] gspSeqHeader_VA malloc() failed \n");
                return -1;
            }
            DSTATUS( "[VENC] gspSeqHeader_VA = 0x%zx, %d \n", (size_t)pInst->gspSeqHeaderAddr[VA], pInst->gsiSeqHeaderSize );

            memset( (void*)pInst->gspSeqHeaderAddr[VA], 0x00, pInst->gsiSeqHeaderSize );
        }

        if(pInst->gspSeqHeaderAddr[PA] != 0)
        {
            DSTATUS( "[VENC] gspSeqHeader_Buffer = 0x%zx, %d \n", (size_t)pInst->gspSeqHeaderAddr[PA], pInst->gsiSeqHeaderSize );
            p_seq_param->m_SeqHeaderBuffer[PA]  =   pInst->gspSeqHeaderAddr[PA];
            p_seq_param->m_SeqHeaderBuffer[VA]  =   pInst->gspSeqHeaderAddr[K_VA];
            p_seq_param->m_iSeqHeaderBufferSize =   pInst->gsiSeqHeaderSize;
            m_SeqHeaderBuffer_VA = pInst->gspSeqHeaderAddr[VA];
        }
        else
        {
            DSTATUS( "[VENC] gspSeqHeader_Buffer = 0x%zx, %d \n", (size_t)pInst->gsBitstreamBufAddr[PA], pInst->gsBitstreamBufSize );
            p_seq_param->m_SeqHeaderBuffer[PA]  =   pInst->gsBitstreamBufAddr[PA];
            p_seq_param->m_SeqHeaderBuffer[VA]  =   pInst->gsBitstreamBufAddr[K_VA];
            p_seq_param->m_iSeqHeaderBufferSize =   pInst->gsBitstreamBufSize;
            m_SeqHeaderBuffer_VA = pInst->gsBitstreamBufAddr[VA];
        }

        if( pInst->gsVpuEncInit_Info.gsVpuEncInit.m_iBitstreamFormat == STD_MPEG4 )
        {
            p_dest = (unsigned char*)pInst->seq_backup;

            pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderType = MPEG4_VOS_HEADER;
            pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_HeaderAddr  = p_seq_param->m_SeqHeaderBuffer[PA];
            pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize = p_seq_param->m_iSeqHeaderBufferSize;
        #if defined(TCC_VPU_C7_INCLUDE)
            pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_HeaderAddr_VA  = p_seq_param->m_SeqHeaderBuffer[VA];
        #endif
            ret = venc_cmd_process(V_ENC_PUT_HEADER, &pInst->gsVpuEncPutHeader_Info, pInst);
            if( ret != RETCODE_SUCCESS )
            {
                LOGE( "[VENC:Err:%d] venc_vpu MPEG4_VOS_HEADER failed \n", ret );
                return -ret;
            }
            (void)memcpy( p_dest, (void*)m_SeqHeaderBuffer_VA, pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize );
            p_dest += pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize;
            i_dest_size += pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize;
            LOGD("VOL : %d / %d", pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize, i_dest_size);

            pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderType = MPEG4_VIS_HEADER;
            pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_HeaderAddr  = p_seq_param->m_SeqHeaderBuffer[PA];
            pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize = p_seq_param->m_iSeqHeaderBufferSize;
        #if defined(TCC_VPU_C7_INCLUDE)
            pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_HeaderAddr_VA  = p_seq_param->m_SeqHeaderBuffer[VA];
        #endif
            ret = venc_cmd_process(V_ENC_PUT_HEADER, &pInst->gsVpuEncPutHeader_Info, pInst);
            if( ret != RETCODE_SUCCESS )
            {
                LOGE( "[VENC:Err:%d] venc_vpu MPEG4_VIS_HEADER failed \n", ret );
                return -ret;
            }
            (void)memcpy( p_dest, (void*)m_SeqHeaderBuffer_VA, pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize );
            p_dest += pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize;
            i_dest_size += pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize;
            LOGD("VOS : %d / %d", pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize, i_dest_size);

            pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderType = MPEG4_VOL_HEADER;
            pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_HeaderAddr  = p_seq_param->m_SeqHeaderBuffer[PA];
            pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize = p_seq_param->m_iSeqHeaderBufferSize;
        #if defined(TCC_VPU_C7_INCLUDE)
            pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_HeaderAddr_VA  = p_seq_param->m_SeqHeaderBuffer[VA];
        #endif
            DSTATUS( "[VENC] VPU_ENC_PUT_HEADER for MPEG4_VOL_HEADER \n");
            ret = venc_cmd_process(V_ENC_PUT_HEADER, &pInst->gsVpuEncPutHeader_Info, pInst);

            if( ret != RETCODE_SUCCESS )
            {
                LOGE( "[VENC:Err:%d] venc_vpu MPEG4_VOL_HEADER failed \n", ret );
                return -ret;
            }
            filewrite_memory("/data/enc.dat", (char*)m_SeqHeaderBuffer_VA, pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize);
            (void)memcpy( p_dest, (void*)m_SeqHeaderBuffer_VA, pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize );
            i_dest_size += pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize;
            LOGD("VIS : %d / %d", pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize, i_dest_size);
            pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize = pInst->seq_len = i_dest_size;

            memcpy( (void*)m_SeqHeaderBuffer_VA, (void*)pInst->seq_backup, pInst->seq_len );
        }
        else if( pInst->gsVpuEncInit_Info.gsVpuEncInit.m_iBitstreamFormat == STD_AVC )
        {
            if(pInst->gsiSeqHeaderCnt == 0)
            {
                p_dest = (unsigned char*)pInst->seq_backup;
                pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderType = AVC_SPS_RBSP;
                pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_HeaderAddr  = p_seq_param->m_SeqHeaderBuffer[PA];
                pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize = p_seq_param->m_iSeqHeaderBufferSize;
        #if defined(TCC_VPU_C7_INCLUDE)
                pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_HeaderAddr_VA  = p_seq_param->m_SeqHeaderBuffer[VA];
        #endif

#ifdef MULTI_SLICES_AVC
                if( pInst->enc_avc_aud_enable == 1) {
                    (void)memcpy( (void*)m_SeqHeaderBuffer_VA, avcAudData, 8 ); //H.264 AUD
                    pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_HeaderAddr += 8;
                }
#endif

                DSTATUS( "[VENC] VPU_ENC_PUT_HEADER for AVC_SPS_RBSP \n");
                ret = venc_cmd_process(V_ENC_PUT_HEADER, &pInst->gsVpuEncPutHeader_Info, pInst);
                if( ret != RETCODE_SUCCESS )
                {
                    LOGE( "[VENC:Err:%d] venc_vpu AVC_SPS_RBSP failed \n", ret );
                    return -ret;
                }

                // modified by shmin for M2TS
                if(pInst->bAvcUsedNALStart)
                {
#ifdef MULTI_SLICES_AVC
                    if( pInst->enc_avc_aud_enable == 1) {
                        pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize += 8;
                    }
#endif
                    unsigned char* buffer = (unsigned char*)m_SeqHeaderBuffer_VA;
                    LOGD("SPS(%d) :: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x - p_dest = 0x%x", pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize,
                                    buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], (uint32_t)p_dest );

                    filewrite_memory("/data/enc.dat", (char*)m_SeqHeaderBuffer_VA, pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize);
                    (void)memcpy( (void*)p_dest, (void*)m_SeqHeaderBuffer_VA, pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize );
                    if(pInst->bWFDTranscodingMode == 1)
                    {
                        ALOGI("[VENC_K] : Transcoding For WFD Source Device....");
                        p_dest[6] = 0xc0;
                    }
                }
                else
                {
                    unsigned char* buffer;

                    buffer = (unsigned char*)m_SeqHeaderBuffer_VA;
                    pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize -= 4;
                    (void)memcpy( (void*)p_dest, (void*)(buffer+4), pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize);
                }

                (void)memcpy( p_dest, (void*)m_SeqHeaderBuffer_VA, pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize );
                pInst->seq_len = pInst->gsiSzSeqHeader[pInst->gsiSeqHeaderCnt] = pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize;
            }
            else
            {
                p_dest = (unsigned char*)pInst->seq_backup;
                p_dest += pInst->gsiSzSeqHeader[pInst->gsiSeqHeaderCnt-1];
                pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderType = AVC_PPS_RBSP;
                pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_HeaderAddr  = p_seq_param->m_SeqHeaderBuffer[PA];
                pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize = p_seq_param->m_iSeqHeaderBufferSize;
        #if defined(TCC_VPU_C7_INCLUDE)
                pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_HeaderAddr_VA  = p_seq_param->m_SeqHeaderBuffer[VA];
        #endif

                DSTATUS( "[VENC] VPU_ENC_PUT_HEADER for AVC_PPS_RBSP \n");
                ret = venc_cmd_process(V_ENC_PUT_HEADER, &pInst->gsVpuEncPutHeader_Info, pInst);
                if( ret != RETCODE_SUCCESS )
                {
                    LOGE( "[VENC:Err:%d] venc_vpu AVC_SPS_RBSP failed \n", ret );
                    return -ret;
                }

                // modified by shmin for M2TS
                if(pInst->bAvcUsedNALStart)
                {
                    if (DEBUG_ON == 1) {
                        unsigned char* buffer = (unsigned char*)m_SeqHeaderBuffer_VA;
                        LOGD("PPS(%d) :: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x - p_dest = 0x%x", pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize,
                                        buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], (uint32_t)p_dest );
                    }
                    filewrite_memory("/data/enc.dat", (char*)m_SeqHeaderBuffer_VA, pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize);
                    (void)memcpy( (void*)p_dest, (void*)m_SeqHeaderBuffer_VA, pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize );
                }
                else
                {
                    unsigned char* buffer;

                    buffer = (unsigned char*)m_SeqHeaderBuffer_VA;
                    pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize -= 4;
                    (void)memcpy( (void*)p_dest, (void*)(buffer+4), pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize);
                }

                (void)memcpy( p_dest, (void*)m_SeqHeaderBuffer_VA, pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize );
                pInst->gsiSzSeqHeader[pInst->gsiSeqHeaderCnt] = pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize;
                pInst->seq_len += pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize;

                (void)memcpy( (void*)m_SeqHeaderBuffer_VA, (void*)pInst->seq_backup, pInst->seq_len );
            }
        }

        // output
        p_seq_param->m_pSeqHeaderOut = (unsigned char*)m_SeqHeaderBuffer_VA;
        p_seq_param->m_iSeqHeaderOutSize = pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize;

        pInst->gsiSeqHeaderCnt++;

        LOGI( "[VENC-%d] VENC_ENC_PUT_HEADER - Success mem_free = 0x%x \n", pInst->venc_instance_index, cdk_sys_final_free_mem_enc(pInst) );
        DSTATUS( "[VENC-%d] =======================================================", pInst->venc_instance_index );

    }
    else if( iOpCode == VENC_ENCODE )
    {
        venc_input_t* p_input_param = (venc_input_t*)pParam1;
        venc_output_t* p_output_param = (venc_output_t*)pParam2;
#ifdef INSERT_SEQ_HEADER_IN_FORCED_IFRAME
        int bChanged_fps = 0; //for only MPEG4.
#endif
        //! Start encoding a frame.
        p_output_param->m_iHeaderOutSize = 0;

        //Input Buffer Setting
#ifdef ENABLE_RATE_CONTROL
        pInst->gsVpuEncInOut_Info.gsVpuEncInput.m_iChangeRcParamFlag  =  p_input_param->m_iChangeRcParamFlag;
        pInst->gsVpuEncInOut_Info.gsVpuEncInput.m_iChangeTargetKbps   =  p_input_param->m_iChangeTargetKbps;
        pInst->gsVpuEncInOut_Info.gsVpuEncInput.m_iChangeFrameRate    =  p_input_param->m_iChangeFrameRate;
    #ifdef INSERT_SEQ_HEADER_IN_FORCED_IFRAME
        if( pInst->gsVpuEncInit_Info.gsVpuEncInit.m_iBitstreamFormat == STD_MPEG4
            && ((pInst->gsVpuEncInOut_Info.gsVpuEncInput.m_iChangeRcParamFlag & 0x04) == 0x04))
        {
            bChanged_fps = 1;
        }
    #endif

    #ifdef CHECK_BITRATE
        if((pInst->gsVpuEncInOut_Info.gsVpuEncInput.m_iChangeRcParamFlag & 0x04) == 0x04)
            pInst->curr_fps = pInst->gsVpuEncInOut_Info.gsVpuEncInput.m_iChangeFrameRate;

        if((pInst->gsVpuEncInOut_Info.gsVpuEncInput.m_iChangeRcParamFlag & 0x02) == 0x02)
        {
            unsigned int calc_bps;

            calc_bps = (pInst->total_size*8)/1024;

            if(pInst->curr_fps != pInst->bps_frames)
            {
                unsigned int temp_bps = calc_bps;
                calc_bps = (temp_bps*pInst->curr_fps)/pInst->bps_frames;
            }

            LOGD("Bitrate- %d kbps  => %d kbps :: %d bytes / %d frames (%d fps)", pInst->curr_bps, calc_bps, pInst->total_size, pInst->bps_frames, pInst->curr_fps);

            if(pInst->gsVpuEncInOut_Info.gsVpuEncInput.m_iChangeRcParamFlag & 0x2)
                pInst->curr_bps = pInst->gsVpuEncInOut_Info.gsVpuEncInput.m_iChangeTargetKbps;
            else if(pInst->gsVpuEncInOut_Info.gsVpuEncInput.m_iChangeRcParamFlag & 0x4)
                pInst->curr_fps = pInst->gsVpuEncInOut_Info.gsVpuEncInput.m_iChangeFrameRate;

            pInst->total_size = pInst->bps_frames = 0;
        }
        pInst->bps_frames++;
    #endif
#endif

        if(p_input_param->request_IntraFrame == 1)
        {
            pInst->gsVpuEncInOut_Info.gsVpuEncInput.m_iForceIPicture = 1;//set 1 For IDR-Type I-Frame without P-Frame!!
        }
        else
        {
            pInst->gsVpuEncInOut_Info.gsVpuEncInput.m_iForceIPicture = 0;
        }

        pInst->gsVpuEncInOut_Info.gsVpuEncInput.m_iSkipPicture = 0;
        if( pInst->gsVpuEncInit_Info.gsVpuEncInit.m_iTargetKbps == 0 ) // no rate control
        {
            pInst->gsVpuEncInOut_Info.gsVpuEncInput.m_iQuantParam = 23;
        }
        else
        {
            if( p_input_param->m_iQuantParam > 0) {
                pInst->gsVpuEncInOut_Info.gsVpuEncInput.m_iQuantParam =  p_input_param->m_iQuantParam;
			} else {
                pInst->gsVpuEncInOut_Info.gsVpuEncInput.m_iQuantParam = 10;
            }
        }

        if((pInst->encoded_buf_cur_pos[PA] + ALIGNED_BUFF((pInst->gsVpuEncInit_Info.gsVpuEncInit.m_iPicWidth * pInst->gsVpuEncInit_Info.gsVpuEncInit.m_iPicHeight*3/2), ALIGN_LEN)) > pInst->encoded_buf_end_pos[PA])
        {
            pInst->encoded_buf_cur_pos[PA] = pInst->encoded_buf_base_pos[PA];
            pInst->encoded_buf_cur_pos[VA] = pInst->encoded_buf_base_pos[VA];
            pInst->encoded_buf_cur_pos[K_VA] = pInst->encoded_buf_base_pos[K_VA];
        }
        pInst->gsVpuEncInOut_Info.gsVpuEncInput.m_iBitstreamBufferSize = pInst->gsBitstreamBufSize;

    #ifdef INSERT_SEQ_HEADER_IN_FORCED_IFRAME
        if( !bChanged_fps && (/*pInst->gsiFrameIdx != 0 && */((pInst->gsiFrameIdx%pInst->gsVpuEncInit_Info.gsVpuEncInit.m_iFrameRate) == 0)) && pInst->gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iSliceMode != 1)
        {
            pInst->encoded_buf_cur_pos[PA] = pInst->encoded_buf_base_pos[PA] + ALIGNED_BUFF(pInst->seq_len, ALIGN_LEN);
            pInst->encoded_buf_cur_pos[VA] = pInst->encoded_buf_base_pos[VA] + ALIGNED_BUFF(pInst->seq_len, ALIGN_LEN);
            pInst->encoded_buf_cur_pos[K_VA] = pInst->encoded_buf_base_pos[K_VA] + ALIGNED_BUFF(pInst->seq_len, ALIGN_LEN);
            pInst->gsVpuEncInOut_Info.gsVpuEncInput.m_iBitstreamBufferSize -= ALIGNED_BUFF(pInst->seq_len, ALIGN_LEN);
        }
    #endif

        pInst->gsVpuEncInOut_Info.gsVpuEncInput.m_BitstreamBufferAddr =  (codec_addr_t)pInst->encoded_buf_cur_pos[PA];
        pInst->gsVpuEncInOut_Info.gsVpuEncInput.m_PicYAddr = (codec_addr_t)p_input_param->m_pInputY;
        if( pInst->gsVpuEncInit_Info.gsVpuEncInit.m_bCbCrInterleaveMode == 0 )
        {
            pInst->gsVpuEncInOut_Info.gsVpuEncInput.m_PicCbAddr = (codec_addr_t)p_input_param->m_pInputCbCr[0];
            pInst->gsVpuEncInOut_Info.gsVpuEncInput.m_PicCrAddr = (codec_addr_t)p_input_param->m_pInputCbCr[1];
        }
        else
        {
            pInst->gsVpuEncInOut_Info.gsVpuEncInput.m_PicCbAddr = (codec_addr_t)p_input_param->m_pInputCbCr[0];
        }

#ifdef MULTI_SLICES_AVC
        //H.264 AUD RBSP
        if( pInst->enc_avc_aud_enable == 1 && pInst->gsiFrameIdx > 0 ) {
            (void)memcpy( (void*)pInst->encoded_buf_cur_pos[VA], avcAudData, 8 );
            pInst->gsVpuEncInOut_Info.gsVpuEncInput.m_BitstreamBufferAddr += 8;
            pInst->gsVpuEncInOut_Info.gsVpuEncInput.m_iBitstreamBufferSize -= 8;
        }

        // Slice information buffer setting
        if( pInst->gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iSliceMode == 1 ) {
            pInst->gsVpuEncInOut_Info.gsVpuEncInput.m_iReportSliceInfoEnable = 1;
            pInst->gsVpuEncInOut_Info.gsVpuEncInput.m_SliceInfoAddr[PA] = pInst->enc_slice_info_addr[PA];
            pInst->gsVpuEncInOut_Info.gsVpuEncInput.m_SliceInfoAddr[VA] = pInst->enc_slice_info_addr[K_VA];
        }
        else {
            pInst->gsVpuEncInOut_Info.gsVpuEncInput.m_iReportSliceInfoEnable = 0;
            pInst->gsVpuEncInOut_Info.gsVpuEncInput.m_SliceInfoAddr[PA] = 0;
            pInst->gsVpuEncInOut_Info.gsVpuEncInput.m_SliceInfoAddr[VA] = 0;
        }
#endif

        DPRINTF_FRAME(" 0x%x-0x%x-0x%x, %d-%d-%d, %d-%d-%d, 0x%x-%d", pInst->gsVpuEncInOut_Info.gsVpuEncInput.m_PicYAddr, pInst->gsVpuEncInOut_Info.gsVpuEncInput.m_PicCbAddr, pInst->gsVpuEncInOut_Info.gsVpuEncInput.m_PicCrAddr,
                pInst->gsVpuEncInOut_Info.gsVpuEncInput.m_iForceIPicture, pInst->gsVpuEncInOut_Info.gsVpuEncInput.m_iSkipPicture, pInst->gsVpuEncInOut_Info.gsVpuEncInput.m_iQuantParam,
                pInst->gsVpuEncInOut_Info.gsVpuEncInput.m_iChangeRcParamFlag, pInst->gsVpuEncInOut_Info.gsVpuEncInput.m_iChangeTargetKbps, pInst->gsVpuEncInOut_Info.gsVpuEncInput.m_iChangeFrameRate,
                pInst->gsVpuEncInOut_Info.gsVpuEncInput.m_BitstreamBufferAddr, pInst->gsVpuEncInOut_Info.gsVpuEncInput.m_iBitstreamBufferSize);
        ret = venc_cmd_process(V_ENC_ENCODE, &pInst->gsVpuEncInOut_Info, pInst);

        pInst->total_frm++;
//      LOGD("systemtime:: encoded frame");

        if( ret != RETCODE_SUCCESS )
        {
            if( ret == RETCODE_WRAP_AROUND )
            {
                LOGE( "[VENC] Warning!! BitStream buffer wrap arounded. prepare more large buffer = %d \n", ret );
            }
            LOGE( "[VENC:Err:0x%x] %d'th VPU_ENC_ENCODE failed \n", ret, pInst->gPFrameCnt );
            LOGE( "[VENC:Err] BitAddr 0x%zx - %d \n", (size_t)pInst->gsVpuEncInOut_Info.gsVpuEncInput.m_BitstreamBufferAddr, pInst->gsVpuEncInOut_Info.gsVpuEncInput.m_iBitstreamBufferSize );
            return -ret;
        }

#ifdef MULTI_SLICES_AVC
        if( pInst->gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iSliceMode == 1 )
        {
            if( pInst->enc_avc_aud_enable == 1 && pInst->gsiFrameIdx > 0 )
            {
                pInst->gsVpuEncInOut_Info.gsVpuEncOutput.m_BitstreamOut[VA] -= 8;
                pInst->gsVpuEncInOut_Info.gsVpuEncOutput.m_BitstreamOut[PA] -= 8;
                pInst->gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize += 8;
            }
        }
#endif

        // output
        if( pInst->gsVpuEncInit_Info.gsVpuEncInit.m_iBitstreamFormat == STD_AVC)
        {
            // modified by shmin for M2TS
            if(pInst->bAvcUsedNALStart)
            {
                p_output_param->m_pBitstreamOut     = (unsigned char*)pInst->encoded_buf_cur_pos[VA];
                p_output_param->m_iBitstreamOutSize = pInst->gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize;
                filewrite_memory("/data/enc.dat", (char*)pInst->encoded_buf_cur_pos[VA], pInst->gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize);
                save_output_stream("/sdcard/vpu_outEnc.bin", pInst->gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize, (unsigned char*)pInst->encoded_buf_cur_pos[VA]);
            }
            else
            {
                p_output_param->m_pBitstreamOut     = (unsigned char*)pInst->encoded_buf_cur_pos[VA];
                p_output_param->m_pBitstreamOut     += 4;
                p_output_param->m_iBitstreamOutSize = pInst->gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize - 4;
            }
        }
        else
        {
            p_output_param->m_pBitstreamOut     = (unsigned char*)pInst->encoded_buf_cur_pos[VA];
            p_output_param->m_iBitstreamOutSize = pInst->gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize;
        }

#ifdef MULTI_SLICES_AVC
        if( pInst->gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iSliceMode == 1 )
        {
            unsigned int *pSliceSize;
            unsigned int extra_size = pInst->gsVpuEncInOut_Info.gsVpuEncOutput.m_iSliceInfoNum * sizeof(unsigned int);

            if (DEBUG_ON == 1) {
                unsigned char *p = p_output_param->m_pBitstreamOut;
                DSTATUS("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x ",
                    p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9], p[10], p[11], p[12], p[13], p[14], p[15]);
            }
            pSliceSize = (unsigned int*)(p_output_param->m_pBitstreamOut + p_output_param->m_iBitstreamOutSize + STABILITY_GAP);
            pSliceSize = (unsigned int *)((uintptr_t)ALIGNED_BUFF(pSliceSize, 256));
            p_output_param->m_pSliceInfo = (unsigned int*)pSliceSize;
            p_output_param->m_iSliceCount = 0;

            if( pInst->gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iSliceMode == 1 )
            {
                p_output_param->m_iSliceCount = pInst->gsVpuEncInOut_Info.gsVpuEncOutput.m_iSliceInfoNum;
                if( pInst->gsVpuEncInOut_Info.gsVpuEncOutput.m_iSliceInfoNum > 1 && (void*)pInst->gsVpuEncInOut_Info.gsVpuEncOutput.m_SliceInfoAddr != NULL)
                {
                    int i;
                    int iSize;
                    unsigned int * pSliceParaBuf;
                    unsigned char *pS;
                    unsigned int total_bytes = 0;

                    pS = (unsigned char*)p_output_param->m_pBitstreamOut;

                    iSize = pInst->gsVpuEncInOut_Info.gsVpuEncOutput.m_iSliceInfoSize;

                    DSTATUS("[EncSliceNum:%3d], Addr - 0x%zx[0x%zx/0x%zx]", pInst->gsVpuEncInOut_Info.gsVpuEncOutput.m_iSliceInfoNum,
                                (size_t)pInst->gsVpuEncInOut_Info.gsVpuEncOutput.m_SliceInfoAddr, (size_t)pInst->enc_slice_info_addr[VA], (size_t)pInst->enc_slice_info_addr[K_VA]);

                    pSliceParaBuf = (unsigned int *)(pInst->enc_slice_info_addr[VA] + (pInst->gsVpuEncInOut_Info.gsVpuEncOutput.m_SliceInfoAddr - pInst->enc_slice_info_addr[K_VA]));

                    for( i=0 ; i<iSize/8 ; i++, pSliceParaBuf += 2 )
                    {
                        int nMbAddr, nSliceBits;

                        nMbAddr = pSliceParaBuf[1]&0x00FFFF;
                        nSliceBits = pSliceParaBuf[0];
                        pSliceSize[i] = nSliceBits / 8;

                        if( pInst->enc_avc_aud_enable == 1  && pInst->gsiFrameIdx > 0 && i == 0){
                            pSliceSize[i] += 8;
                        }
#if 0
                        if(pInst->gsiFrameIdx < 5)
                        {
                            LOGD(" slice[%d] = %d", i, pSliceSize[i]);
                        }
                        else
                        {
                            if(pSliceSize[i] > 1500){
                                LOGE("[%d'th frames :: OverSlice[%d] = %d", pInst->gsiFrameIdx, i, pSliceSize[i]);
                            }
                        }
#endif
                        if(pInst->gsiFrameIdx < 5 && DEBUG_ON == 1)
                        {
                            DSTATUS( "[%2d] mbAddr.%3d, Bits.%d\n", i, nMbAddr, nSliceBits );
                            DSTATUS( "      0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x", pS[total_bytes+0], pS[total_bytes+1], pS[total_bytes+2],
                                                    pS[total_bytes+3], pS[total_bytes+4], pS[total_bytes+5], pS[total_bytes+6], pS[total_bytes+7]);
                            total_bytes += (nSliceBits/8);
                            DSTATUS( "  ~   0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x", pS[total_bytes-8], pS[total_bytes-7], pS[total_bytes-6],
                                                    pS[total_bytes-5], pS[total_bytes-4], pS[total_bytes-3], pS[total_bytes-2], pS[total_bytes-1]);
                        }

                    }
                }
                else
                {
                    p_output_param->m_iSliceCount = 1;
                    pSliceSize[0] = pInst->gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize;
                }
            }

            pInst->encoded_buf_cur_pos[PA] += ALIGNED_BUFF(pInst->gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize + (STABILITY_GAP*2) + extra_size, ALIGN_LEN);
            pInst->encoded_buf_cur_pos[VA] += ALIGNED_BUFF(pInst->gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize + (STABILITY_GAP*2) + extra_size, ALIGN_LEN);
            pInst->encoded_buf_cur_pos[K_VA] += ALIGNED_BUFF(pInst->gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize + (STABILITY_GAP*2) + extra_size, ALIGN_LEN);
        }
        else
#endif
        {
    #ifdef INSERT_SEQ_HEADER_IN_FORCED_IFRAME
            if(!bChanged_fps && /*pInst->gsiFrameIdx != 0 && */((pInst->gsiFrameIdx%pInst->gsVpuEncInit_Info.gsVpuEncInit.m_iFrameRate) == 0))
            {
                DSTATUS("Inserted sequence header prior to Stream.")
                p_output_param->m_pBitstreamOut -= pInst->seq_len;
                (void)memcpy(p_output_param->m_pBitstreamOut, pInst->seq_backup, pInst->seq_len);
                p_output_param->m_iBitstreamOutSize += pInst->seq_len;
            }
    #endif
            pInst->encoded_buf_cur_pos[PA] += ALIGNED_BUFF(pInst->gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize + (STABILITY_GAP*2), ALIGN_LEN);
            pInst->encoded_buf_cur_pos[VA] += ALIGNED_BUFF(pInst->gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize + (STABILITY_GAP*2), ALIGN_LEN);
            pInst->encoded_buf_cur_pos[K_VA] += ALIGNED_BUFF(pInst->gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize + (STABILITY_GAP*2), ALIGN_LEN);
        }

        p_output_param->m_iPicType = pInst->gsVpuEncInOut_Info.gsVpuEncOutput.m_iPicType;

        if( pInst->gsVpuEncInOut_Info.gsVpuEncOutput.m_iPicType == PIC_TYPE_I )
        {
            pInst->keyInterval_cnt = 1;
            if(pInst->gMaxOutputSize_IFrame < pInst->gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize) {
                pInst->gMaxOutputSize_IFrame = pInst->gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize;
            }

            LOGI("[I:%4d/%4d] [interval %d] = %d/%d, P=%d!", pInst->gIFrameCnt, pInst->gsiFrameIdx,
					pInst->gsVpuEncInit_Info.gsVpuEncInit.m_iKeyInterval, pInst->gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize,
					pInst->gMaxOutputSize_IFrame, pInst->gMaxOutputSize_PFrame);
            DSTATUS( "[I:%4d/%4d] Byte:%7d(%5.1lfK) ", pInst->gIFrameCnt, pInst->gsiFrameIdx,
					pInst->gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize,
					pInst->gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize/1024.0 );
            pInst->gIFrameCnt++;
        }
        else if( pInst->gsVpuEncInOut_Info.gsVpuEncOutput.m_iPicType == PIC_TYPE_P )
        {
            pInst->keyInterval_cnt++;
            if(pInst->gMaxOutputSize_PFrame < pInst->gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize) {
                pInst->gMaxOutputSize_PFrame = pInst->gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize;
            }
            DSTATUS( "[P:%4d/%4d] Byte:%7d(%5.1lfK) ", pInst->gIFrameCnt, pInst->gsiFrameIdx,
					pInst->gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize,
					pInst->gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize/1024.0 );

            if(pInst->gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize < 20){
                DSTATUS( "[P:%4d/%4d] Byte:%7d(%5.1lfK) ", pInst->gPFrameCnt, pInst->gsiFrameIdx,
						pInst->gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize,
						pInst->gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize/1024.0 );
            }
            pInst->gPFrameCnt++;
        }
        pInst->gsiFrameIdx++;
#ifdef CHECK_BITRATE
        pInst->total_size += pInst->gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize;
#endif

#ifdef INSERT_SEQ_HEADER_IN_FORCED_IFRAME
        if( pInst->gsVpuEncInit_Info.gsVpuEncInit.m_iBitstreamFormat == STD_MPEG4 && bChanged_fps)
        {
            pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderType = MPEG4_VOL_HEADER;
            pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_HeaderAddr  = pInst->gspSeqHeaderAddr[PA];
            pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize = pInst->gsiSeqHeaderSize;
        #if defined(TCC_VPU_C7_INCLUDE)
            pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_HeaderAddr_VA  = pInst->gspSeqHeaderAddr[K_VA];
        #endif

            DSTATUS( "[VENC] VPU_ENC_PUT_HEADER for MPEG4_VOL_HEADER \n");
            ret = venc_cmd_process(V_ENC_PUT_HEADER, &pInst->gsVpuEncPutHeader_Info, pInst);

            if( ret != RETCODE_SUCCESS )
            {
                LOGE( "[VENC:Err:%d] venc_vpu MPEG4_VOL_HEADER failed \n", ret );
            }
            else
            {
                if(pInst->seq_backup == NULL){
                    pInst->seq_backup = (unsigned char*)TCC_malloc(pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize + (STABILITY_GAP*2));
                }
                (void)memcpy((void*)pInst->seq_backup, (void*)pInst->gspSeqHeaderAddr[VA], pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize);
                pInst->seq_len = pInst->gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize;
            }
        }
#endif

        #if 0
        DSTATUS( "0x%x ", pInst->gsVpuEncInOut_Info.gsVpuEncOutput.m_BitstreamOut[VA] );
        {
            int temp = 0;
            DSTATUS("Data:");
            for( temp = 0; temp < 32; temp+=4 )
            {
                DSTATUS("0x%02X", gspPictureData[temp+0] );
                DSTATUS(  "%02X", gspPictureData[temp+1] );
                DSTATUS(  "%02X", gspPictureData[temp+2] );
                DSTATUS(  "%02X", gspPictureData[temp+3] );
                DSTATUS(  " " );
            }
        }
        #endif
    }
    else if( iOpCode == VENC_CLOSE )
    {
        ret = venc_cmd_process(V_ENC_CLOSE, &pInst->gsVpuEncInOut_Info, pInst);
        if( ret != RETCODE_SUCCESS )
        {
            DPRINTF( "[VENC] VPU_ENC_CLOSE failed Error code is %d \n", ret );
        }

        cdk_sys_free_virtual_addr_enc((unsigned long*)pInst->gsBitstreamBufAddr[VA], (unsigned int)pInst->gsBitstreamBufSize );
        cdk_sys_free_virtual_addr_enc((unsigned long*)pInst->gsBitWorkBufAddr[VA], (unsigned int)pInst->gsBitWorkBufSize );
        cdk_sys_free_virtual_addr_enc((unsigned long*)pInst->gsMESearchBufAddr[VA], (unsigned int)pInst->gsMESearchBufSize );
        cdk_sys_free_virtual_addr_enc((unsigned long*)pInst->gsFrameBufAddr[VA], (unsigned int)pInst->gsFrameBufSize );
        cdk_sys_free_virtual_addr_enc((unsigned long*)pInst->yuv_buffer_addr[VA], (unsigned int)pInst->yuv_buffer_size );

#ifdef MULTI_SLICES_AVC
        cdk_sys_free_virtual_addr_enc((void*)pInst->enc_slice_info_addr[VA], pInst->enc_slice_info_size );
#endif

        cdk_sys_free_virtual_addr_enc((unsigned long*)pInst->gspSeqHeaderAddr[VA], (unsigned int)pInst->gsiSeqHeaderSize);
        cdk_sys_free_virtual_addr_enc((unsigned long*)pInst->encoded_buf_base_pos[VA], (unsigned int)pInst->encoded_buf_size);

        vpu_env_close_enc(pInst);

#if defined(VPU_CLK_CONTROL)
        vpu_clock_deinit();
#endif
        pInst->gsiFrameIdx = 0;
    }
    else
    {
        LOGE( "[VENC] Invalid Operation!!\n" );
        return -ret;
    }

#ifdef DEBUG_TIME_LOG
    end = clock();

    if( iOpCode == VENC_INIT ){
        LOGD("VENC_INIT_TIME %d ms", (end-start)*1000/CLOCKS_PER_SEC);
    }
    else if( iOpCode == VENC_SEQ_HEADER){
        LOGD("VENC_SEQ_TIME %d ms", (end-start)*1000/CLOCKS_PER_SEC);
    }
    else if( iOpCode == VENC_ENCODE )
    {
        pInst->enc_time[pInst->time_cnt] = (end-start)*1000/CLOCKS_PER_SEC;
        pInst->total_enc_time += pInst->enc_time[pInst->time_cnt];
        if (pInst->time_cnt != 0 && pInst->time_cnt % 29 == 0) {
            LOGD("VENC_TIME %2.1f ms: %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d,"
                                    " %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d,"
                                    " %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d,"
                                    " %3d, %3d, %3d, %3d, %3d, %3d",
                pInst->total_enc_time/(float)pInst->total_frm,
                pInst->enc_time[0], pInst->enc_time[1], pInst->enc_time[2], pInst->enc_time[3],
                pInst->enc_time[4], pInst->enc_time[5], pInst->enc_time[6], pInst->enc_time[7],
                pInst->enc_time[8], pInst->enc_time[9], pInst->enc_time[10], pInst->enc_time[11],
                pInst->enc_time[12], pInst->enc_time[13], pInst->enc_time[14], pInst->enc_time[15],
                pInst->enc_time[16], pInst->enc_time[17], pInst->enc_time[18], pInst->enc_time[19],
                pInst->enc_time[20], pInst->enc_time[21], pInst->enc_time[22], pInst->enc_time[23],
                pInst->enc_time[24], pInst->enc_time[25], pInst->enc_time[26], pInst->enc_time[27],
                pInst->enc_time[28], pInst->enc_time[29]);

            pInst->time_cnt = 0;
        } else {
            pInst->time_cnt++;
        }
    }
#endif

    return ret;
}


#ifdef TCC_JPU_INCLUDE
static int jpu_cmd_process(int cmd, void* args, _venc_ *pVenc)
{
    int ret = 0;
    int iSuccess = 0;
    _venc_ * pInst = pVenc;
    int retry_cnt = 10;
    int all_retry_cnt = 3;

    if(ioctl(pInst->enc_fd, cmd, args) < 0)
    {
        if( ret == -0x999 )
        {
            LOGE("JPU[%d] Invalid command(%d) ", pInst->venc_instance_index, cmd);
            return RETCODE_INVALID_COMMAND;
        }
        else
        {
            LOGE("JPU[%d] ioctl err[%s] : cmd = %d", pInst->venc_instance_index, strerror(errno), cmd);
        }
    }

Retry:
    while (retry_cnt > 0) {
        memset(pInst->tcc_event, 0, sizeof(pInst->tcc_event));
        pInst->tcc_event[0].fd = pInst->enc_fd;
        pInst->tcc_event[0].events = POLLIN;

        ret = poll((struct pollfd *)&pInst->tcc_event, 1, 10000); // 1 sec
        if (ret < 0) {
            LOGE("JPU[%d]-retry(%d:cmd(%d)) poll error '%s'", pInst->venc_instance_index, retry_cnt, cmd, strerror(errno));
            retry_cnt--;
            continue;
        }else if (ret == 0) {
            LOGE("JPU[%d]-retry(%d:cmd(%d)) poll timeout: %d'th frames, len %d", pInst->venc_instance_index, retry_cnt, cmd, pInst->total_frm, pInst->gsJpuEncInOut_Info.gsJpuEncInput.m_iBitstreamBufferSize );
            retry_cnt--;
            continue;
        }else if (ret > 0) {
            if (pInst->tcc_event[0].revents & POLLERR) {
                LOGE("JPU[%d] poll POLLERR", pInst->venc_instance_index);
                break;
            } else if (pInst->tcc_event[0].revents & POLLIN) {
                iSuccess = 1;
                break;
            }
        }
    }
    /* todo */

    switch(cmd)
    {
        case V_ENC_INIT:
            {
                JENC_INIT_t* init_info = args;

                if(ioctl(pInst->enc_fd, V_ENC_INIT_RESULT, args) < 0){
                    LOGE("JPU[%d] ioctl(%d) error[%s]!!", pInst->venc_instance_index, V_ENC_INIT_RESULT, strerror(errno));
                }
                ret = init_info->result;
            }
            break;

        case V_ENC_PUT_HEADER:
            {

            }
            break;

        case V_ENC_ENCODE:
            {
                JPU_ENCODE_t* encoded_info = args;

                if(ioctl(pInst->enc_fd, V_ENC_ENCODE_RESULT, args) < 0){
                    LOGE("JPU[%d] ioctl(%d) error[%s]!!", pInst->venc_instance_index, V_ENC_ENCODE_RESULT, strerror(errno));
                }
                ret = encoded_info->result;
            }
            break;
        case V_GET_VPU_VERSION:
            {
                JPU_GET_VERSION_t* p_param = args;
                if(ioctl(pInst->enc_fd, V_GET_VPU_VERSION_RESULT, args) < 0){
                    LOGE("JPU[%d] ioctl(%d) error[%s]!!", pInst->venc_instance_index, V_GET_VPU_VERSION_RESULT, strerror(errno));
                }
                ret = p_param->result;
            }
            break;

        case V_ENC_REG_FRAME_BUFFER:
        case V_ENC_CLOSE:
        default:
            if(ioctl(pInst->enc_fd, V_ENC_GENERAL_RESULT, &ret) < 0){
                LOGE("JPU[%d] ioctl(%d) error[%s]!!", pInst->venc_instance_index, V_ENC_GENERAL_RESULT, strerror(errno));
            }
            break;
    }

    if((ret&0xf000) != 0x0000){ //If there is an invalid return, we skip it because this return means that vpu didn't process current command yet.
        all_retry_cnt--;
        if( all_retry_cnt > 0)
        {
            retry_cnt = 10;
            goto Retry;
        }
        else
        {
            LOGE("abnormal exception!!");
        }
    }

#ifdef ERROR_TEST
    if (err_test++ == 1000)
        ret = 0xf000;
#endif

    if(!iSuccess
        || ((ret&0xf000) != 0x0000) /* vpu can not start or finish its processing with unknown reason!! */
    )
    {
        LOGE("JPU[%d] command(%d) didn't work properly. maybe hangup(no return(%d))!!",
                pInst->venc_instance_index, cmd, ret);
#if 0
        if(ret != RETCODE_CODEC_EXIT && ret != RETCODE_MULTI_CODEC_EXIT_TIMEOUT){
//          ioctl(pInst->mgr_fd, VPU_HW_RESET, (void*)NULL);
        }
#endif
        ret = RETCODE_CODEC_EXIT;
    }

    return ret;
}

// For use JPU as a MJPEG H/W decoder
int venc_mjpeg_jpu(int iOpCode, unsigned long* pHandle, void* pParam1, void* pParam2, void* pParam3)
{
    codec_result_t ret = 0;
    _venc_ *pInst = (_venc_ *)pParam3;

    if(!pInst){
        LOGE("venc_mjpeg_jpu(OP:%d) :: Instance is null!!", iOpCode);
        return -RETCODE_NOT_INITIALIZED;
    }

    if( ( iOpCode == VENC_INIT ) && (pInst->venc_env_opened) )
    {
        ALOGE("venc_mjpeg_jpu(OP:%d) :: VENC has been already opened. so have to close!!", iOpCode);
        return -VPU_ENV_INIT_ERROR;
    }

    if (iOpCode == VENC_INIT) {
        //! Set Encoder Init
        DSTATUS("[JPU-%d] VENC_INIT. venc_mjpeg_jpu", pInst->venc_instance_index);
        venc_init_t* p_init_param = (venc_init_t*)pParam1;

        pInst->codec_format = p_init_param->m_iBitstreamFormat;
        if (vpu_env_open_enc(p_init_param->m_iPicWidth, p_init_param->m_iPicHeight, pInst ) < 0) {
            LOGE("[JPU-%d] vpu_env_open_enc error", pInst->venc_instance_index);
            return -VPU_ENV_INIT_ERROR;
        }

#if defined(TCC_JPU_C6_INCLUDE)
        pInst->gsJpuEncInit_Info.gsJpuEncInit.m_iSourceFormat           = p_init_param->m_iMjpg_sourceFormat; // YUV_FORMAT_422
        pInst->gsJpuEncInit_Info.gsJpuEncInit.m_iCbCrInterleaveMode     = 0;
#else
        pInst->gsJpuEncInit_Info.gsJpuEncInit.m_iMjpg_sourceFormat      = p_init_param->m_iMjpg_sourceFormat; // YUV_FORMAT_422
        pInst->gsJpuEncInit_Info.gsJpuEncInit.m_bCbCrInterleaveMode     = 0;
#endif
        pInst->gsJpuEncInit_Info.gsJpuEncInit.m_RegBaseVirtualAddr      = (unsigned int)NULL; // No need to set!!
        pInst->gsJpuEncInit_Info.gsJpuEncInit.m_iPicWidth               = p_init_param->m_iPicWidth;
        pInst->gsJpuEncInit_Info.gsJpuEncInit.m_iPicHeight              = p_init_param->m_iPicHeight;
        pInst->gsJpuEncInit_Info.gsJpuEncInit.m_iEncQuality             = p_init_param->m_iEncQuality;
        pInst->gsJpuEncInit_Info.gsJpuEncInit.m_uiEncOptFlags           = 0;
        pInst->gsJpuEncInit_Info.gsJpuEncInit.m_Memcpy                  = NULL; // No need to set!!
        pInst->gsJpuEncInit_Info.gsJpuEncInit.m_Memset                  = NULL; // No need to set!!
        pInst->gsJpuEncInit_Info.gsJpuEncInit.m_Interrupt               = NULL; // No need to set!!

        pInst->keyInterval_cnt = 0;

        if(p_init_param->m_iBitstreamBufferSize > LARGE_STREAM_BUF_SIZE)
            pInst->gsBitstreamBufSize = ALIGNED_BUFF( p_init_param->m_iBitstreamBufferSize, 64*1024 );
        else
            pInst->gsBitstreamBufSize = LARGE_STREAM_BUF_SIZE;
        pInst->gsBitstreamBufSize = ALIGNED_BUFF( pInst->gsBitstreamBufSize, ALIGN_LEN );

        pInst->gsBitstreamBufAddr[PA] = (codec_addr_t)cdk_sys_malloc_physical_addr_enc( &pInst->gsBitstreamBufAddr[K_VA], &pInst->gsBitstreamBufSize, BUFFER_STREAM, pInst );
        if( pInst->gsBitstreamBufAddr[PA] == 0 )
        {
            LOGE( "[JPU-%d] bitstream_buf_addr[PA] malloc() failed ", pInst->venc_instance_index);
            return -(VPU_NOT_ENOUGH_MEM);
        }
        DSTATUS("[JPU-%d] bitstream_buf_addr[PA] = 0x%zx, %d ", pInst->venc_instance_index, (size_t)pInst->gsBitstreamBufAddr[PA], pInst->gsBitstreamBufSize );

        pInst->gsBitstreamBufAddr[VA] = (codec_addr_t)cdk_sys_malloc_virtual_addr_enc(  pInst->gsBitstreamBufAddr[PA], pInst->gsBitstreamBufSize, pInst );
        if( pInst->gsBitstreamBufAddr[VA] == 0 )
        {
            LOGE( "[JPU-%d] bitstream_buf_addr[VA] malloc() failed ", pInst->venc_instance_index);
            return -(VPU_NOT_ENOUGH_MEM);
        }

        memset( (void*)pInst->gsBitstreamBufAddr[VA], 0x00 , pInst->gsBitstreamBufSize);
        DSTATUS("[JPU-%d] bitstream_buf_addr[VA] = 0x%zx, %d ", pInst->venc_instance_index, (size_t)pInst->gsBitstreamBufAddr[VA], pInst->gsBitstreamBufSize );

        pInst->gsVpuEncInit_Info.gsVpuEncInit.m_iBitstreamFormat  = p_init_param->m_iBitstreamFormat;

        pInst->gsJpuEncInit_Info.gsJpuEncInit.m_BitstreamBufferAddr[PA] = pInst->gsBitstreamBufAddr[PA];
        pInst->gsJpuEncInit_Info.gsJpuEncInit.m_BitstreamBufferAddr[VA] = pInst->gsBitstreamBufAddr[K_VA];
        pInst->gsJpuEncInit_Info.gsJpuEncInit.m_iBitstreamBufferSize    = pInst->gsBitstreamBufSize;

        ret = jpu_cmd_process(V_ENC_INIT, &pInst->gsJpuEncInit_Info, pInst);
        if( ret != JPG_RET_SUCCESS )
        {
            DPRINTF( "[JPU-%d] [enc_test] JPU_ENC_INIT failed \r", pInst->venc_instance_index);
            return -ret;
        }

        {
            //------------------------------------------------------------
            //! [x] input buffer for YUV raw image.
            //------------------------------------------------------------
            int iMbWidth, iMbHeight;

            iMbWidth = ((pInst->gsJpuEncInit_Info.gsJpuEncInit.m_iPicWidth+15)>>4)<<4;
            iMbHeight = ((pInst->gsJpuEncInit_Info.gsJpuEncInit.m_iPicHeight+15)>>4)<<4;

            pInst->yuv_buffer_size = iMbWidth * iMbHeight * 3 / 2;
            pInst->yuv_buffer_size = ALIGNED_BUFF(pInst->yuv_buffer_size, ALIGN_LEN);
            pInst->yuv_buffer_addr[PA] = (codec_addr_t)(uintptr_t)cdk_sys_malloc_physical_addr_enc( &pInst->yuv_buffer_addr[K_VA], (int *)&pInst->yuv_buffer_size, BUFFER_ELSE, pInst );
            if( pInst->yuv_buffer_addr[PA] == 0 )
            {
                LOGE( "[CDK_CORE] pInst->yuv_buffer_addr[PA] physical malloc() failed \n");
                return CDK_ERR_MALLOC;
            }

            DSTATUS("[CDK_CORE] pInst->yuv_buffer_addr[PA] = 0x%zx, %d \n", (size_t)pInst->yuv_buffer_addr[PA], pInst->yuv_buffer_size );
            pInst->yuv_buffer_addr[VA] = (codec_addr_t)cdk_sys_malloc_virtual_addr_enc(  pInst->yuv_buffer_addr[PA], pInst->yuv_buffer_size, pInst );
            if( pInst->yuv_buffer_addr[VA] == 0 )
            {
                LOGE( "[CDK_CORE] pInst->yuv_buffer_addr[VA] virtual malloc() failed \n");
                return CDK_ERR_MALLOC;
            }
            DSTATUS("[CDK_CORE] pInst->yuv_buffer_addr[VA] = 0x%zx, %d \n", (size_t)pInst->yuv_buffer_addr[VA], pInst->yuv_buffer_size );
        }

        pInst->encoded_buf_base_pos[PA]     = pInst->gsBitstreamBufAddr[PA];
        pInst->encoded_buf_base_pos[VA]     = pInst->gsBitstreamBufAddr[VA];
        pInst->encoded_buf_base_pos[K_VA]   = pInst->gsBitstreamBufAddr[K_VA];
        pInst->encoded_buf_cur_pos[PA]      = pInst->encoded_buf_base_pos[PA];
        pInst->encoded_buf_cur_pos[VA]      = pInst->encoded_buf_base_pos[VA];
        pInst->encoded_buf_cur_pos[K_VA]    = pInst->encoded_buf_base_pos[K_VA];
        pInst->encoded_buf_end_pos[PA]      = pInst->encoded_buf_base_pos[PA] + pInst->gsBitstreamBufSize;
        pInst->encoded_buf_end_pos[VA]      = pInst->encoded_buf_base_pos[VA] + pInst->gsBitstreamBufSize;
        pInst->encoded_buf_end_pos[K_VA]    = pInst->encoded_buf_base_pos[K_VA] + pInst->gsBitstreamBufSize;

        LOGI( "[JPU-%d] VENC_INIT - Success mem_free = 0x%x ", pInst->venc_instance_index, cdk_sys_final_free_mem_enc(pInst) );
        DSTATUS( "[JPU-%d] =======================================================", pInst->venc_instance_index );

        return JPG_RET_SUCCESS;
    }
    else if( iOpCode == VENC_SEQ_HEADER )
    {

    }
    else if( iOpCode == VENC_ENCODE )
    {
        venc_input_t* p_input_param = (venc_input_t*)pParam1;
        venc_output_t* p_output_param = (venc_output_t*)pParam2;

        pInst->gsJpuEncInOut_Info.gsJpuEncInput.m_PicYAddr = (codec_addr_t)p_input_param->m_pInputY;
#if defined(TCC_JPU_C6_INCLUDE)
        if( pInst->gsJpuEncInit_Info.gsJpuEncInit.m_iCbCrInterleaveMode == 0 )
#else
        if( pInst->gsJpuEncInit_Info.gsJpuEncInit.m_bCbCrInterleaveMode == 0 )
#endif
        {
            pInst->gsJpuEncInOut_Info.gsJpuEncInput.m_PicCbAddr = (codec_addr_t)p_input_param->m_pInputCbCr[0];
            pInst->gsJpuEncInOut_Info.gsJpuEncInput.m_PicCrAddr = (codec_addr_t)p_input_param->m_pInputCbCr[1];
        }
        else
        {
            pInst->gsJpuEncInOut_Info.gsJpuEncInput.m_PicCbAddr = (codec_addr_t)p_input_param->m_pInputCbCr[0];
        }

        pInst->gsJpuEncInOut_Info.gsJpuEncInput.m_BitstreamBufferAddr[PA]   = pInst->gsBitstreamBufAddr[PA];
        pInst->gsJpuEncInOut_Info.gsJpuEncInput.m_BitstreamBufferAddr[VA]   = pInst->gsBitstreamBufAddr[K_VA];
        pInst->gsJpuEncInOut_Info.gsJpuEncInput.m_iBitstreamBufferSize      = pInst->gsBitstreamBufSize;

        DSTATUS("[JPU-%d] V_ENC_ENCODE Start. %zu/%zu-%d", pInst->venc_instance_index,
                (size_t)pInst->gsJpuEncInOut_Info.gsJpuEncInput.m_BitstreamBufferAddr[PA],
                (size_t)pInst->gsJpuEncInOut_Info.gsJpuEncInput.m_BitstreamBufferAddr[VA],
                pInst->gsJpuEncInOut_Info.gsJpuEncInput.m_iBitstreamBufferSize);

        ret = jpu_cmd_process(V_ENC_ENCODE, &pInst->gsJpuEncInOut_Info, pInst);
        if(ret != JPG_RET_SUCCESS){
            LOGE("[JPU-%d] DECODE ERROR. ret = %d", pInst->venc_instance_index, ret);
            return -ret;
        }
        DSTATUS("[JPU-%d] ret = %d", pInst->venc_instance_index, ret);

        p_output_param->m_pBitstreamOut     = (uint8_t *)(uintptr_t)vpu_getStreamOutVirtAddr((uint8_t *)(uintptr_t)pInst->gsJpuEncInOut_Info.gsJpuEncOutput.m_BitstreamOut[PA], PA, pInst);
#if defined(TCC_JPU_C6_INCLUDE)
        p_output_param->m_iHeaderOutSize    = pInst->gsJpuEncInOut_Info.gsJpuEncOutput.m_iBitstreamHeaderSize;
#else
        p_output_param->m_iHeaderOutSize    = pInst->gsJpuEncInOut_Info.gsJpuEncOutput.m_iHeaderOutSize;
#endif
        p_output_param->m_iBitstreamOutSize = pInst->gsJpuEncInOut_Info.gsJpuEncOutput.m_iBitstreamOutSize;
        p_output_param->m_iSliceCount       = 0;
        p_output_param->m_pSliceInfo        = 0;
        p_output_param->m_iPicType          = 0;

        DSTATUS("[JPU-%d] V_ENC_ENCODE Done. 0x%zu -> %p-%d/%d", pInst->venc_instance_index,
                (size_t)pInst->gsJpuEncInOut_Info.gsJpuEncOutput.m_BitstreamOut[PA],
                p_output_param->m_pBitstreamOut,
                p_output_param->m_iHeaderOutSize, p_output_param->m_iBitstreamOutSize);

        if (DEBUG_ON == 1)
        {
            unsigned char *ps = (unsigned char *)p_output_param->m_pBitstreamOut;
            DSTATUS( "[JPU-%d] " "0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x " "0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x "
                                "0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x " "0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x "
                                "0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x ",
                                pInst->venc_instance_index,
                                ps[0], ps[1], ps[2], ps[3], ps[4], ps[5], ps[6], ps[7], ps[8], ps[9], ps[10], ps[11], ps[12], ps[13], ps[14], ps[15],
                                ps[16], ps[17], ps[18], ps[19], ps[20], ps[21], ps[22], ps[23], ps[24], ps[25], ps[26], ps[27], ps[28], ps[29], ps[30], ps[31],
                                ps[32], ps[33], ps[34], ps[35], ps[36], ps[37], ps[38], ps[39], ps[40], ps[41], ps[42], ps[43], ps[44], ps[45], ps[46], ps[47],
                                ps[48], ps[49], ps[50], ps[51], ps[52], ps[53], ps[54], ps[55], ps[56], ps[57], ps[58], ps[59], ps[60], ps[61], ps[62], ps[63],
                                ps[64], ps[65], ps[66], ps[67], ps[68], ps[69], ps[70], ps[71], ps[72], ps[73], ps[74], ps[75], ps[76], ps[77], ps[78], ps[79]);
        }

        return ret;
    }
    else if( iOpCode == VENC_CLOSE )
    {
        DSTATUS("[JPU-%d] VENC_CLOSE. venc_mjpeg_jpu", pInst->venc_instance_index);
        // Now that we are done with encoding, close the open instance.

        cdk_sys_free_virtual_addr_enc((unsigned long*)pInst->gsBitstreamBufAddr[VA], (unsigned int)pInst->gsBitstreamBufSize);
        cdk_sys_free_virtual_addr_enc((unsigned long*)pInst->yuv_buffer_addr[VA], (unsigned int)pInst->yuv_buffer_size );

        ret = jpu_cmd_process(V_ENC_CLOSE, &pInst->gsJpuEncInOut_Info, pInst);
        if(ret != RETCODE_SUCCESS)
        {
            LOGE("[JPU-%d] JPU_ENC_CLOSE FAIL", pInst->venc_instance_index);
            return -ret;
        }

        vpu_env_close_enc(pInst);

        return RETCODE_SUCCESS;
    }

    LOGE("[JPU-%d] INVALID OP_CODE", pInst->venc_instance_index);
    return 0;
}
#endif

void * venc_alloc_instance(int codec_format)
{
    _venc_ *pInst = NULL;
    char *mgr_name;
    INSTANCE_INFO iInst_info;

    pInst = (_venc_*)TCC_malloc(sizeof(_venc_));
    if( pInst )
    {
        memset(pInst, 0x00, sizeof(_venc_));

        pInst->mgr_fd = -1;
        pInst->enc_fd = -1;
        pInst->codec_format = codec_format;

#ifdef TCC_JPU_INCLUDE
        if( codec_format == STD_MJPG )
            mgr_name = JPU_MGR_NAME;
        else
#endif
            mgr_name = VPU_MGR_NAME;

        pInst->mgr_fd = open(mgr_name, O_RDWR);
        if(pInst->mgr_fd < 0)
        {
            LOGE("%s open error[%s]!!", mgr_name, strerror(errno));
            goto MGR_OPEN_ERR;
        }

        iInst_info.type = VPU_ENC;
        iInst_info.nInstance = 0;
        if(ioctl(pInst->mgr_fd, VPU_GET_INSTANCE_IDX, &iInst_info) < 0){
            LOGE("%s ioctl(%d) error[%s]!!", mgr_name, VPU_GET_INSTANCE_IDX, strerror(errno));
        }
        if( iInst_info.nInstance < 0 )
        {
            goto INST_GET_ERR;
        }

        pInst->venc_instance_index = iInst_info.nInstance;
        pInst->enc_fd = open(enc_devices[pInst->venc_instance_index], O_RDWR);
        (void)g_printf("@@@@@@@@@@@@@@@@@@@@@@@@@@@@ Instance[%d] = %s", pInst->venc_instance_index, enc_devices[pInst->venc_instance_index]);
        if(pInst->enc_fd < 0)
        {
            LOGE("%s open error[%s]", enc_devices[pInst->venc_instance_index], strerror(errno));
            goto ENC_OPEN_ERR;
        }

        total_opened_encoder++;

#ifdef TCC_JPU_INCLUDE
        if( codec_format == STD_MJPG ){
            jpu_opened_count++;
            LOGI("[JPU-%d] %d/%d :: venc_alloc_instance total", pInst->venc_instance_index, jpu_opened_count, total_opened_encoder);
        }
        else
#endif
        {
            vpu_opened_count++;
            LOGI("[VENC-%d] %d/%d :: venc_alloc_instance total", pInst->venc_instance_index, vpu_opened_count, total_opened_encoder);
        }

    }

    return pInst;

ENC_OPEN_ERR:
    iInst_info.type = VPU_ENC;
    iInst_info.nInstance = pInst->venc_instance_index;
    if( ioctl(pInst->mgr_fd, VPU_CLEAR_INSTANCE_IDX, &iInst_info) < 0){
        LOGE("%s ioctl(%d) error[%s]!!", mgr_name, VPU_CLEAR_INSTANCE_IDX, strerror(errno));
    }
INST_GET_ERR:
    if(close(pInst->mgr_fd) < 0){
        LOGE("%s close error[%s]", mgr_name, strerror(errno));
    }
MGR_OPEN_ERR:
    TCC_free(pInst);
    return NULL;

}

void venc_release_instance(void * pInst, int codec_format)
{
    if(pInst)
    {
        _venc_ * pVenc = (_venc_ *)pInst;
        int used_instance = pVenc->venc_instance_index;
        char *mgr_name;
        INSTANCE_INFO iInst_info;
#ifndef TCC_JPU_INCLUDE
        (void)codec_format;
#endif

#ifdef TCC_JPU_INCLUDE
        if( codec_format == STD_MJPG )
            mgr_name = JPU_MGR_NAME;
        else
#endif
            mgr_name = VPU_MGR_NAME;

        iInst_info.type = VPU_ENC;
        iInst_info.nInstance = used_instance;
        if( ioctl(pVenc->mgr_fd, VPU_CLEAR_INSTANCE_IDX, &iInst_info) < 0){
            LOGE("%s ioctl(%d) error[%s]!!", mgr_name, VPU_CLEAR_INSTANCE_IDX, strerror(errno));
        }

        if(pVenc->enc_fd)
        {
            if(close(pVenc->enc_fd) < 0)
            {
                LOGE("%s close error[%s]", enc_devices[pVenc->venc_instance_index], strerror(errno));
            }
            pVenc->enc_fd = -1;
        }

        if(pVenc->mgr_fd)
        {
            if(close(pVenc->mgr_fd) < 0){
                LOGE("%s close error[%s]", mgr_name, strerror(errno));
            }
            pVenc->mgr_fd = -1;
        }

        TCC_free(pInst);
        pInst = NULL;

        if(total_opened_encoder > 0)
            total_opened_encoder--;

#ifdef TCC_JPU_INCLUDE
        if( codec_format == STD_MJPG ){
            if(jpu_opened_count > 0)
                jpu_opened_count--;
            LOGI("[JPU-%d] %d/%d :: venc_release_instance total", used_instance, jpu_opened_count, total_opened_encoder);
        }
        else
#endif
        {
            if(vpu_opened_count > 0)
                vpu_opened_count--;
            LOGI("[VENC-%d] %d/%d :: venc_release_instance total", used_instance, vpu_opened_count, total_opened_encoder);
        }
    }
}

int venc_get_instance_index(void * pInst)
{
    _venc_ * pVenc = (_venc_ *) pInst;

    if(pVenc)
        return pVenc->venc_instance_index;

    return -1;
}

