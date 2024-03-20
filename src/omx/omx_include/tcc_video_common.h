/****************************************************************************
 *   FileName    : tcc_video_common.h
 *   Description : 
 ****************************************************************************
 *
 *   TCC Version 1.0
 *   Copyright (c) Telechips Inc.
 *   All rights reserved 
 
This source code contains confidential information of Telechips.
Any unauthorized use without a written permission of Telechips including not limited to re-distribution in source or binary form is strictly prohibited.
This source code is provided “AS IS” and nothing contained in this source code shall constitute any express or implied warranty of any kind, including without limitation, any warranty of merchantability, fitness for a particular purpose or non-infringement of any patent, copyright or other third party intellectual property right. No warranty is made, express or implied, regarding the information’s accuracy, completeness, or performance. 
In no event shall Telechips be liable for any claim, damages or other liability arising from, out of or in connection with this source code or the use in the source code. 
This source code is provided subject to the terms of a Mutual Non-Disclosure Agreement between Telechips and Company.
*
****************************************************************************/

#ifndef TCC_VIDEO_COMMON__
#define TCC_VIDEO_COMMON__

//#define	PORTRAIT_GUI_SUPPORT // temp. definition for Samsung Apollo Project
//#define 	VIDEO_GETFRAME_TEST
//#define	PERFORMANCE_MEASURE

#define MAX_COMPONENT_PARSER 			1

#define VIDEO_FILENAME_LEN 			512
#define MEDIA_EXTRADATA_SIZE           			1024

#define VIDEO_PORT_INDEX          			0  /* video port index on clock component */
#define AUDIO_PORT_INDEX        			1  /* audio port index on clock component */
#define PARSER_PORT_INDEX      			2  /* parser port index on clock component */
#define CLOCK_PORT_INDEX				2  /* clock port index on parser component */

#define CLIENT_CLOCK_PORT_INDEX  		1  /* clock port index on sink (Audio/Video) component */
#define PARSER_CLOCK_PORT_INDEX  		2  /* clock port index on parser component */

#define VIDEO_STREAM 0
#define AUDIO_STREAM 1

#define AUDIO_END 						0x1
#define VIDEO_END  						0x2
#define AV_END  							0x4

#define MKTAG(a,b,c,d) (a | (b << 8) | (c << 16) | (d << 24))


/*Video Decoder Name Definition*/
#define VIDEO_DEC_BASE_NAME	"OMX.tcc.video_decoder"

#define VIDEO_DEC_H264_NAME	"OMX.TCC.avcdec"//"OMX.tcc.video_decoder.avc"
#define VIDEO_DEC_H264_ROLE	"video_decoder.avc"
#define VIDEO_DEC_TCC_H264_ROLE	"video_decoder.tcc_avc"

#define VIDEO_DEC_MPEG4_NAME	"OMX.TCC.mpeg4dec" //"OMX.tcc.video_decoder.mpeg4"
#define VIDEO_DEC_MPEG4_ROLE	"video_decoder.mpeg4"
#define VIDEO_DEC_TCC_MPEG4_ROLE	"video_decoder.tcc_mpeg4"

#define VIDEO_DEC_WMV_NAME	"OMX.TCC.wmvdec" //"OMX.tcc.video_decoder.wmv"
#define VIDEO_DEC_WMV12_NAME	"OMX.TCC.wmv12dec" //"OMX.tcc.video_decoder.wmv"
#define VIDEO_DEC_WMV_ROLE	"video_decoder.wmv"
#define VIDEO_DEC_TCC_WMV_ROLE	"video_decoder.tcc_wmv"
#define VIDEO_DEC_TCC_WMV12_ROLE	"video_decoder.tcc_wmv12"

#define VIDEO_DEC_MPEG2_NAME  "OMX.TCC.mpeg2dec" // "OMX.tcc.video_decoder.mpeg2"
#define VIDEO_DEC_MPEG2_ROLE   "video_decoder.mpeg2"

#define VIDEO_DEC_RV_NAME  "OMX.TCC.rvdec" //"OMX.tcc.video_decoder.rv"
#define VIDEO_DEC_RV_ROLE   "video_decoder.rv"

#define VIDEO_DEC_H263_NAME	"OMX.TCC.h263dec"
#define VIDEO_DEC_H263_ROLE	"video_decoder.h263"
#define VIDEO_DEC_TCC_H263_ROLE	"video_decoder.tcc_h263"

#define VIDEO_DEC_SORENSON_H263_NAME "OMX.TCC.flv1dec"
#define VIDEO_DEC_SORENSON_H263_ROLE "video_decoder.sorenson_h263"

#define VIDEO_DEC_MSMPEG4_NAME	"OMX.TCC.msmpeg4dec"
#define VIDEO_DEC_MSMPEG4_ROLE	"video_decoder.msmpeg4"

#define VIDEO_DEC_DIVX_NAME	"OMX.TCC.divxdec"
#define VIDEO_DEC_DIVX_ROLE	"video_decoder.divx"

#define VIDEO_DEC_MJPEG_NAME "OMX.TCC.mjpegdec"
#define VIDEO_DEC_MJPEG_ROLE "video_decoder.mjpeg"

#define VIDEO_DEC_AVS_NAME "OMX.TCC.avsdec"
#define VIDEO_DEC_AVS_ROLE "video_decoder.avs"

#define VIDEO_DEC_VP8_NAME "OMX.TCC.vp8dec"
#define VIDEO_DEC_VP8_ROLE "video_decoder.vp8"

#define VIDEO_DEC_VP9_NAME "OMX.TCC.vp9dec"
#define VIDEO_DEC_VP9_ROLE "video_decoder.vp9"

#define VIDEO_DEC_MVC_NAME "OMX.TCC.mvcdec"
#define VIDEO_DEC_MVC_ROLE "video_decoder.mvc"

#define VIDEO_DEC_H265_NAME "OMX.TCC.hevcdec"
#define VIDEO_DEC_H265_ROLE "video_decoder.hevc"

/*Video Encoder Name Definition*/
#define VIDEO_ENC_BASE_NAME	"OMX.tcc.video_encoder"

#define VIDEO_ENC_MPEG4_NAME	"OMX.TCC.ENC.mpeg4"
#define VIDEO_ENC_MPEG4_ROLE	"video_encoder.mpeg4"

#define VIDEO_ENC_H264_NAME	"OMX.TCC.ENC.avc"
#define VIDEO_ENC_H264_ROLE	"video_encoder.avc"

#define VIDEO_ENC_H263_NAME	"OMX.TCC.ENC.h263"
#define VIDEO_ENC_H263_ROLE	"video_encoder.h263"

/*Video Parser Name Definition*/
#define VIDEO_PARSER_BASE_NAME "OMX.tcc.parser"

#define VIDEO_PARSER_AVI_NAME "OMX.tcc.parser.avi"
#define VIDEO_PARSER_MP4_NAME "OMX.tcc.parser.mp4"
#define VIDEO_PARSER_ASF_NAME "OMX.tcc.parser.asf"
#define VIDEO_PARSER_MPG_NAME "OMX.tcc.parser.mpg"
#define VIDEO_PARSER_EXT_F_NAME "OMX.tcc.parser.ext_f"

/*A/V Rederer Name Definition*/
#define FBDEV_SINK_NAME		"OMX.tcc.fbdev.fbdev_sink"
#define ALSA_SINK_NAME  		"OMX.tcc.alsa.alsasink"

/*A/V Effector Name Definition*/
#define AUDIO_EFFECT_VOLUME_NAME		"OMX.tcc.volume.component"

/*Clock Sync Component Name Definition*/
#define CLOCK_SOURCE_NAME      			"OMX.tcc.clocksrc"

#define TDMB_REC_SINK_NAME      		"OMX.tcc.tdmb.rec"


typedef struct {
	/*Contents*/
	unsigned long		ulTotalTime; //전체 재생 시간 
		
	/*Video*/
	unsigned long		ulFcc;
	unsigned long		ulWidth;
	unsigned long		ulHeight;
	unsigned long		ulFramesPerSec;

	/*Audio*/
	unsigned long		ulFormatTag; 
	unsigned long		ulSamplesPerSec;
	unsigned long		ulBitsPerSample;
	unsigned long		ulNumChannel;
}contentInfo;

typedef enum {
	
	COMPONENT_AVI_PARSER,
	COMPONENT_MP4_PARSER,
	COMPONENT_ASF_PARSER,
	COMPONENT_MPG_PARSER, 
	COMPONENT_RM_PARSER, 
	
	COMPONENT_MPEG4_DECODER,
	COMPONENT_H264_DECODER,
	COMPONENT_WMV_DECODER,
	COMPONENT_MPEG2_DECODER, 
	COMPONENT_RV_DECODER, 

	COMPONENT_MP3_DECODER,
	COMPONENT_AAC_DECODER,
	COMPONENT_WMA_DECODER,
	COMPONENT_MP2_DECODER, 
#ifdef TCC_AUDIO_AC3_DECODING_INCLUDE		
	COMPONENT_AC3_DECODER,
#endif
	COMPONENT_PCM_DECODER,
	COMPONENT_OPUS_DECODER,
	COMPONENT_RA_DECODER,


	COMPONENT_VOLUME,

	COMPONENT_FBDEV_SINK,
	COMPONENT_ALSA_SINK,

	COMPONENT_CLOCK_SRC,

	COMPONENT_NONE=0xFFFFFFFF
	
}COMPONENT_ID_E;

typedef enum {
	
	CODEC_ID_NONE=0,
		
	/* video codecs */
	CODEC_ID_MPEG1,
	CODEC_ID_MPEG2, ///< preferred ID for MPEG-1/2 video decoding
	CODEC_ID_MPEG4,
	CODEC_ID_H264,
	CODEC_ID_RV,
	CODEC_ID_MJPEG,
	CODEC_ID_WMV,
	CODEC_ID_DIVX,
	CODEC_ID_XVID,

	/* image codec*/
	CODEC_ID_PNG,
	CODEC_ID_BMP,
	CODEC_ID_GIF,

	/* audio codecs */
	CODEC_ID_MP2,
	CODEC_ID_MP3,
	CODEC_ID_AAC,
	CODEC_ID_MPEG4AAC,
	CODEC_ID_AC3,
	CODEC_ID_DTS,
	CODEC_ID_VORBIS,
	CODEC_ID_WMA,
	CODEC_ID_FLAC,
	CODEC_ID_PCM,
	CODEC_ID_ADPCM,
	CODEC_ID_APE,
	CODEC_ID_OPUS,
	CODEC_ID_RA,
}CODEC_ID_E ;

 typedef enum {
		
	PIX_FMT_NONE= -1,
	PIX_FMT_YUV420P,   ///< Planar YUV 4:2:0, 12bpp, (1 Cr & Cb sample per 2x2 Y samples)
	PIX_FMT_YUYV422,   ///< Packed YUV 4:2:2, 16bpp, Y0 Cb Y1 Cr
	PIX_FMT_RGB24,     ///< Packed RGB 8:8:8, 24bpp, RGBRGB...
	PIX_FMT_BGR24,     ///< Packed RGB 8:8:8, 24bpp, BGRBGR...
	PIX_FMT_YUV422P,   ///< Planar YUV 4:2:2, 16bpp, (1 Cr & Cb sample per 2x1 Y samples)
	PIX_FMT_YUV444P,   ///< Planar YUV 4:4:4, 24bpp, (1 Cr & Cb sample per 1x1 Y samples)
	PIX_FMT_RGB32,     ///< Packed RGB 8:8:8, 32bpp, (msb)8A 8R 8G 8B(lsb), in cpu endianness
	PIX_FMT_YUV410P,   ///< Planar YUV 4:1:0,  9bpp, (1 Cr & Cb sample per 4x4 Y samples)
	PIX_FMT_YUV411P,   ///< Planar YUV 4:1:1, 12bpp, (1 Cr & Cb sample per 4x1 Y samples)
	PIX_FMT_RGB565,    ///< Packed RGB 5:6:5, 16bpp, (msb)   5R 6G 5B(lsb), in cpu endianness
	PIX_FMT_RGB555,    ///< Packed RGB 5:5:5, 16bpp, (msb)1A 5R 5G 5B(lsb), in cpu endianness most significant bit to 0
	PIX_FMT_GRAY8,     ///<        Y        ,  8bpp
	PIX_FMT_MONOWHITE, ///<        Y        ,  1bpp, 0 is white, 1 is black
	PIX_FMT_MONOBLACK, ///<        Y        ,  1bpp, 0 is black, 1 is white
	PIX_FMT_PAL8,      ///< 8 bit with PIX_FMT_RGB32 palette
	PIX_FMT_YUVJ420P,  ///< Planar YUV 4:2:0, 12bpp, full scale (jpeg)
	PIX_FMT_YUVJ422P,  ///< Planar YUV 4:2:2, 16bpp, full scale (jpeg)
	PIX_FMT_YUVJ444P,  ///< Planar YUV 4:4:4, 24bpp, full scale (jpeg)
	PIX_FMT_XVMC_MPEG2_MC,///< XVideo Motion Acceleration via common packet passing(xvmc_render.h)
	PIX_FMT_XVMC_MPEG2_IDCT,
	PIX_FMT_UYVY422,   ///< Packed YUV 4:2:2, 16bpp, Cb Y0 Cr Y1
	PIX_FMT_UYYVYY411, ///< Packed YUV 4:1:1, 12bpp, Cb Y0 Y1 Cr Y2 Y3
	PIX_FMT_BGR32,     ///< Packed RGB 8:8:8, 32bpp, (msb)8A 8B 8G 8R(lsb), in cpu endianness
	PIX_FMT_BGR565,    ///< Packed RGB 5:6:5, 16bpp, (msb)   5B 6G 5R(lsb), in cpu endianness
	PIX_FMT_BGR555,    ///< Packed RGB 5:5:5, 16bpp, (msb)1A 5B 5G 5R(lsb), in cpu endianness most significant bit to 1
	PIX_FMT_BGR8,      ///< Packed RGB 3:3:2,  8bpp, (msb)2B 3G 3R(lsb)
	PIX_FMT_BGR4,      ///< Packed RGB 1:2:1,  4bpp, (msb)1B 2G 1R(lsb)
	PIX_FMT_BGR4_BYTE, ///< Packed RGB 1:2:1,  8bpp, (msb)1B 2G 1R(lsb)
	PIX_FMT_RGB8,      ///< Packed RGB 3:3:2,  8bpp, (msb)2R 3G 3B(lsb)
	PIX_FMT_RGB4,      ///< Packed RGB 1:2:1,  4bpp, (msb)2R 3G 3B(lsb)
	PIX_FMT_RGB4_BYTE, ///< Packed RGB 1:2:1,  8bpp, (msb)2R 3G 3B(lsb)
	PIX_FMT_NV12,      ///< Planar YUV 4:2:0, 12bpp, 1 plane for Y and 1 for UV
	PIX_FMT_NV21,      ///< as above, but U and V bytes are swapped

	PIX_FMT_RGB32_1,   ///< Packed RGB 8:8:8, 32bpp, (msb)8R 8G 8B 8A(lsb), in cpu endianness
	PIX_FMT_BGR32_1,   ///< Packed RGB 8:8:8, 32bpp, (msb)8B 8G 8R 8A(lsb), in cpu endianness

	PIX_FMT_GRAY16BE,  ///<        Y        , 16bpp, big-endian
	PIX_FMT_GRAY16LE,  ///<        Y        , 16bpp, little-endian
	PIX_FMT_YUV440P,   ///< Planar YUV 4:4:0 (1 Cr & Cb sample per 1x2 Y samples)
	PIX_FMT_YUVJ440P,  ///< Planar YUV 4:4:0 full scale (jpeg)
	PIX_FMT_YUVA420P,  ///< Planar YUV 4:2:0, 20bpp, (1 Cr & Cb sample per 2x2 Y & A samples)
	PIX_FMT_NB,        ///< number of pixel formats, DO NOT USE THIS if you want to link with shared libav* because the number of formats might differ between versions
}PIXEL_FORMAT_E;

 
 typedef struct
 {
	 unsigned int		width;
	 unsigned int		height;
	 unsigned int		width_offset;
	 unsigned int		height_offset; 
	 unsigned int		rotate; 
 }videoframeinfo;

 /*==================================================================================*/
// BUFFER_TYPE : 
// This enumeration tells the type of buffer pointers coming to OMX in UseBuffer call.
/*==================================================================================*/
typedef enum BUFFER_TYPE
{
	VirtualPtr,   			/*Used when buffer pointers which come from the normal virtual space */
	GrallocPtr,   			/*Used when buffer pointers which come from Gralloc allocations */
	EncoderMetadataPtr		/*Used when buffer pointers which come from Stagefright in camcorder usecase */
} BUFFER_TYPE;
#endif /*TCC_VIDEO_COMMON__*/

