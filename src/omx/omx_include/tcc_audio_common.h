/****************************************************************************
 *   FileName    : tcc_audio_common.h
 *   Description : 
 ****************************************************************************
 *
 *   TCC Version 1.0
 *   Copyright (c) Telechips Inc.
 *   All rights reserved 
 
This source code contains confidential information of Telechips.
Any unauthorized use without a written permission of Telechips including not limited to re-distribution in source or binary form is strictly prohibited.
This source code is provided ¡°AS IS¡± and nothing contained in this source code shall constitute any express or implied warranty of any kind, including without limitation, any warranty of merchantability, fitness for a particular purpose or non-infringement of any patent, copyright or other third party intellectual property right. No warranty is made, express or implied, regarding the information¡¯s accuracy, completeness, or performance. 
In no event shall Telechips be liable for any claim, damages or other liability arising from, out of or in connection with this source code or the use in the source code. 
This source code is provided subject to the terms of a Mutual Non-Disclosure Agreement between Telechips and Company.
*
****************************************************************************/

#ifndef TCC_AUDIO_COMMON__
#define TCC_AUDIO_COMMON__


#ifdef __cplusplus
extern "C" {
#endif

#include "adec.h"

#define AUDIO_ID_AAC 0
#define AUDIO_ID_MP3 1
#define AUDIO_ID_AMR 2
#define AUDIO_ID_AC3 3

//#define AUDIO_ID_PCM 4	//--> WAV
#define AUDIO_ID_MP3HD 4
#define AUDIO_ID_MP2 5
#define AUDIO_ID_DTS 6
#define AUDIO_ID_QCELP 7
#define AUDIO_ID_AMRWBP 8
#define AUDIO_ID_WMA 9
#define AUDIO_ID_EVRC 10
#define AUDIO_ID_FLAC 11
#define AUDIO_ID_COOK 12
#define AUDIO_ID_G722 13
#define AUDIO_ID_G729 14
#define AUDIO_ID_APE 15
//#define AUDIO_ID_MSADPCM 16	//--> WAV
#define AUDIO_ID_WAV 16
//#define AUDIO_ID_IMAADPCM 17	//--> WAV
#define AUDIO_ID_DRA 17
#define AUDIO_ID_VORBIS 18
#define AUDIO_ID_BSAC 19

#define AUDIO_ID_MP1  20
#define AUDIO_ID_DDP  21
#define AUDIO_ID_TRUEHD  23
#define AUDIO_ID_DTSHD   1006 // SSG
#define AUDIO_ID_AAC_GOOGLE	24
#define AUDIO_ID_OPUS 25

#define AUDIO_SIZE_MAX ( (4608 * 2) *8*2)
#define AUDIO_MAX_INPUT_SIZE	(1024*96)

typedef int fnCdkAudioDec (int iOpCode, unsigned long* pHandle, void* pParam1, void* pParam2 );

typedef enum
{
	AUDIO_NORMAL_MODE		= 0x0002,	// normal playback mode (ex: file-play)
	AUDIO_BROADCAST_MODE	= 0x0004,	// broadcasting mode (ex: dab, dmb)
	AUDIO_DDP_TO_DD_MODE	= 0x0008,	// ddp to dd converting mode
}TCC_AUDIO_PROCESS_MODE; 

typedef struct decoder_func_list_t
{
	fnCdkAudioDec  *pfMainFunction;
	OMX_PTR pfCodecSpecific[8];
} decoder_func_list_t;

#ifdef __cplusplus
}
#endif

#endif /*TCC_AUDIO_COMMON__*/

