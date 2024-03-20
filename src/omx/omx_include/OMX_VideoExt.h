/*
 * Copyright (c) 2010 The Khronos Group Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

/** OMX_VideoExt.h - OpenMax IL version 1.1.2
 * The OMX_VideoExt header file contains extensions to the
 * definitions used by both the application and the component to
 * access video items.
 */

#ifndef OMX_VideoExt_h
#define OMX_VideoExt_h

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Each OMX header shall include all required header files to allow the
 * header to compile without errors.  The includes below are required
 * for this header file to compile successfully
 */
#include <OMX_Core.h>

/** NALU Formats */
typedef enum OMX_NALUFORMATSTYPE {
    OMX_NaluFormatStartCodes = 1,
    OMX_NaluFormatOneNaluPerBuffer = 2,
    OMX_NaluFormatOneByteInterleaveLength = 4,
    OMX_NaluFormatTwoByteInterleaveLength = 8,
    OMX_NaluFormatFourByteInterleaveLength = 16,
    OMX_NaluFormatCodingMax = 0x7FFFFFFF
} OMX_NALUFORMATSTYPE;

/** NAL Stream Format */
typedef struct OMX_NALSTREAMFORMATTYPE{
    OMX_U32 nSize;
    OMX_VERSIONTYPE nVersion;
    OMX_U32 nPortIndex;
    OMX_NALUFORMATSTYPE eNaluFormat;
} OMX_NALSTREAMFORMATTYPE;

/** Enum for standard video codingtype extensions */
typedef enum OMX_VIDEO_CODINGEXTTYPE {
    OMX_VIDEO_ExtCodingUnused = OMX_VIDEO_CodingVendorStartUnused,
    OMX_VIDEO_CodingVP8,        /**< VP8/WebM */
    OMX_VIDEO_CodingMSMPEG4,    /**< Microsoft MPEG-4 */
    OMX_VIDEO_CodingDIVX,       /**< DIVX 3, 4 */
    OMX_VIDEO_CodingFLV1,       /**< Sorenson's H.263 */
    OMX_VIDEO_CodingAVS,		/**< AVS */
    OMX_VIDEO_CodingMVC,		/**< MVC */
    OMX_VIDEO_CodingHEVC,		/**< HEVC */
    OMX_VIDEO_CodingVP9,        /**< VP9/WebM */
    OMX_VIDEO_CodingExtMax = 0x7FFFFFFF
} OMX_VIDEO_CODINGEXTTYPE;

/** VP8 profiles */
typedef enum OMX_VIDEO_VP8PROFILETYPE {
    OMX_VIDEO_VP8ProfileMain = 0x01,
    OMX_VIDEO_VP8ProfileUnknown = 0x6EFFFFFF,
    OMX_VIDEO_VP8ProfileMax = 0x7FFFFFFF
} OMX_VIDEO_VP8PROFILETYPE;

/** VP8 levels */
typedef enum OMX_VIDEO_VP8LEVELTYPE {
    OMX_VIDEO_VP8Level_Version0 = 0x01,
    OMX_VIDEO_VP8Level_Version1 = 0x02,
    OMX_VIDEO_VP8Level_Version2 = 0x04,
    OMX_VIDEO_VP8Level_Version3 = 0x08,
    OMX_VIDEO_VP8LevelUnknown = 0x6EFFFFFF,
    OMX_VIDEO_VP8LevelMax = 0x7FFFFFFF
} OMX_VIDEO_VP8LEVELTYPE;

/** VP8 Param */
typedef struct OMX_VIDEO_PARAM_VP8TYPE {
    OMX_U32 nSize;
    OMX_VERSIONTYPE nVersion;
    OMX_U32 nPortIndex;
    OMX_VIDEO_VP8PROFILETYPE eProfile;
    OMX_VIDEO_VP8LEVELTYPE eLevel;
    OMX_U32 nDCTPartitions;
    OMX_BOOL bErrorResilientMode;
} OMX_VIDEO_PARAM_VP8TYPE;

/** Structure for configuring VP8 reference frames */
typedef struct OMX_VIDEO_VP8REFERENCEFRAMETYPE {
    OMX_U32 nSize;
    OMX_VERSIONTYPE nVersion;
    OMX_U32 nPortIndex;
    OMX_BOOL bPreviousFrameRefresh;
    OMX_BOOL bGoldenFrameRefresh;
    OMX_BOOL bAlternateFrameRefresh;
    OMX_BOOL bUsePreviousFrame;
    OMX_BOOL bUseGoldenFrame;
    OMX_BOOL bUseAlternateFrame;
} OMX_VIDEO_VP8REFERENCEFRAMETYPE;

/** Structure for querying VP8 reference frame type */
typedef struct OMX_VIDEO_VP8REFERENCEFRAMEINFOTYPE {
    OMX_U32 nSize;
    OMX_VERSIONTYPE nVersion;
    OMX_U32 nPortIndex;
    OMX_BOOL bIsIntraFrame;
    OMX_BOOL bIsGoldenOrAlternateFrame;
} OMX_VIDEO_VP8REFERENCEFRAMEINFOTYPE;

/**
 * Telechips Defined
 */
typedef enum OMX_TC_COLOR_FORMATTYPE {
    OMX_COLOR_FormatTcStartUnused = OMX_COLOR_FormatVendorStartUnused,
    OMX_COLOR_FormatYUV420PlanarTc,
    OMX_COLOR_FormatYUV420SemiPlanarTc,
    OMX_COLOR_FormatYUV422PlanarTc
} OMX_TC_COLOR_FORMATTYPE;

/** VP9 profiles */
typedef enum OMX_VIDEO_VP9PROFILETYPE {
    OMX_VIDEO_VP9Profile0 = 0x1,
    OMX_VIDEO_VP9Profile1 = 0x2,
    OMX_VIDEO_VP9Profile2 = 0x4,
    OMX_VIDEO_VP9Profile3 = 0x8,
    // HDR profiles also support passing HDR metadata
    OMX_VIDEO_VP9Profile2HDR = 0x1000,
    OMX_VIDEO_VP9Profile3HDR = 0x2000,
    OMX_VIDEO_VP9Profile2HDR10Plus = 0x4000,
    OMX_VIDEO_VP9Profile3HDR10Plus = 0x8000,
    OMX_VIDEO_VP9ProfileUnknown = 0x6EFFFFFF,
    OMX_VIDEO_VP9ProfileMax = 0x7FFFFFFF
} OMX_VIDEO_VP9PROFILETYPE;
/** VP9 levels */
typedef enum OMX_VIDEO_VP9LEVELTYPE {
    OMX_VIDEO_VP9Level1  = 0x1,
    OMX_VIDEO_VP9Level11 = 0x2,
    OMX_VIDEO_VP9Level2  = 0x4,
    OMX_VIDEO_VP9Level21 = 0x8,
    OMX_VIDEO_VP9Level3  = 0x10,
    OMX_VIDEO_VP9Level31 = 0x20,
    OMX_VIDEO_VP9Level4  = 0x40,
    OMX_VIDEO_VP9Level41 = 0x80,
    OMX_VIDEO_VP9Level5  = 0x100,
    OMX_VIDEO_VP9Level51 = 0x200,
    OMX_VIDEO_VP9Level52 = 0x400,
    OMX_VIDEO_VP9Level6  = 0x800,
    OMX_VIDEO_VP9Level61 = 0x1000,
    OMX_VIDEO_VP9Level62 = 0x2000,
    OMX_VIDEO_VP9LevelUnknown = 0x6EFFFFFF,
    OMX_VIDEO_VP9LevelMax = 0x7FFFFFFF
} OMX_VIDEO_VP9LEVELTYPE;
/**
* VP9 Parameters.
*   Encoder specific parameters (decoders should ignore these fields):
*     - bErrorResilientMode
*     - nTileRows
*     - nTileColumns
*     - bEnableFrameParallelDecoding
*/
typedef struct OMX_VIDEO_PARAM_VP9TYPE {
    OMX_U32 nSize;
    OMX_VERSIONTYPE nVersion;
    OMX_U32 nPortIndex;
    OMX_VIDEO_VP9PROFILETYPE eProfile;
    OMX_VIDEO_VP9LEVELTYPE eLevel;
    OMX_BOOL bErrorResilientMode;
    OMX_U32 nTileRows;
    OMX_U32 nTileColumns;
    OMX_BOOL bEnableFrameParallelDecoding;
} OMX_VIDEO_PARAM_VP9TYPE;
/** HEVC Profile enum type */
typedef enum OMX_VIDEO_HEVCPROFILETYPE {
    OMX_VIDEO_HEVCProfileUnknown      = 0x0,
    OMX_VIDEO_HEVCProfileMain         = 0x1,
    OMX_VIDEO_HEVCProfileMain10       = 0x2,
    OMX_VIDEO_HEVCProfileMainStill    = 0x4,
    // Main10 profile with HDR SEI support.
    OMX_VIDEO_HEVCProfileMain10HDR10  = 0x1000,
    OMX_VIDEO_HEVCProfileMain10HDR10Plus  = 0x2000,
    OMX_VIDEO_HEVCProfileMax          = 0x7FFFFFFF
} OMX_VIDEO_HEVCPROFILETYPE;
/** HEVC Level enum type */
typedef enum OMX_VIDEO_HEVCLEVELTYPE {
    OMX_VIDEO_HEVCLevelUnknown    = 0x0,
    OMX_VIDEO_HEVCMainTierLevel1  = 0x1,
    OMX_VIDEO_HEVCHighTierLevel1  = 0x2,
    OMX_VIDEO_HEVCMainTierLevel2  = 0x4,
    OMX_VIDEO_HEVCHighTierLevel2  = 0x8,
    OMX_VIDEO_HEVCMainTierLevel21 = 0x10,
    OMX_VIDEO_HEVCHighTierLevel21 = 0x20,
    OMX_VIDEO_HEVCMainTierLevel3  = 0x40,
    OMX_VIDEO_HEVCHighTierLevel3  = 0x80,
    OMX_VIDEO_HEVCMainTierLevel31 = 0x100,
    OMX_VIDEO_HEVCHighTierLevel31 = 0x200,
    OMX_VIDEO_HEVCMainTierLevel4  = 0x400,
    OMX_VIDEO_HEVCHighTierLevel4  = 0x800,
    OMX_VIDEO_HEVCMainTierLevel41 = 0x1000,
    OMX_VIDEO_HEVCHighTierLevel41 = 0x2000,
    OMX_VIDEO_HEVCMainTierLevel5  = 0x4000,
    OMX_VIDEO_HEVCHighTierLevel5  = 0x8000,
    OMX_VIDEO_HEVCMainTierLevel51 = 0x10000,
    OMX_VIDEO_HEVCHighTierLevel51 = 0x20000,
    OMX_VIDEO_HEVCMainTierLevel52 = 0x40000,
    OMX_VIDEO_HEVCHighTierLevel52 = 0x80000,
    OMX_VIDEO_HEVCMainTierLevel6  = 0x100000,
    OMX_VIDEO_HEVCHighTierLevel6  = 0x200000,
    OMX_VIDEO_HEVCMainTierLevel61 = 0x400000,
    OMX_VIDEO_HEVCHighTierLevel61 = 0x800000,
    OMX_VIDEO_HEVCMainTierLevel62 = 0x1000000,
    OMX_VIDEO_HEVCHighTierLevel62 = 0x2000000,
    OMX_VIDEO_HEVCHighTiermax     = 0x7FFFFFFF
} OMX_VIDEO_HEVCLEVELTYPE;
/** Structure for controlling HEVC video encoding */
typedef struct OMX_VIDEO_PARAM_HEVCTYPE {
    OMX_U32 nSize;
    OMX_VERSIONTYPE nVersion;
    OMX_U32 nPortIndex;
    OMX_VIDEO_HEVCPROFILETYPE eProfile;
    OMX_VIDEO_HEVCLEVELTYPE eLevel;
    OMX_U32 nKeyFrameInterval;        // distance between consecutive I-frames (including one
                                      // of the I frames). 0 means interval is unspecified and
                                      // can be freely chosen by the codec. 1 means a stream of
                                      // only I frames.
} OMX_VIDEO_PARAM_HEVCTYPE;

/** WMV9 profiles */
typedef enum OMX_VIDEO_WMV9PROFILETYPE {
    OMX_VIDEO_WMV9ProfileSimple    = 0x01,
    OMX_VIDEO_WMV9ProfileMain      = 0x02,
    OMX_VIDEO_WMV9ProfileAdvanced  = 0x04,
    OMX_VIDEO_WMV9ProfileMax       = 0x7FFFFFFF
} OMX_VIDEO_WMVPROFILETYPE;

/** WMV9 levels */
typedef enum OMX_VIDEO_WMV9LEVELTYPE {
    OMX_VIDEO_WMV9LevelLow      = 0x01,
    OMX_VIDEO_WMV9LevelMedium   = 0x02,
    OMX_VIDEO_WMV9LevelHigh     = 0x04,
    OMX_VIDEO_WMV9LevelLevel0   = 0x08,
    OMX_VIDEO_WMV9LevelLevel1   = 0x10,
    OMX_VIDEO_WMV9LevelLevel2   = 0x20,
    OMX_VIDEO_WMV9LevelLevel3   = 0x40,
    OMX_VIDEO_WMV9LevelLevel4   = 0x80,
    OMX_VIDEO_WMV9LevelMax      = 0x7FFFFFFF
} OMX_VIDEO_WMVLEVELTYPE;

/** Microsoft MPEG-4 */
typedef enum OMX_VIDEO_MSMPEG4VERSIONTYPE {
    OMX_VIDEO_MSMPEG4VersionUnused,
    OMX_VIDEO_MSMPEG4Version1,
    OMX_VIDEO_MSMPEG4Version2,
    OMX_VIDEO_MSMPEG4Version3,
    OMX_VIDEO_MSMPEG4VersionMax       = 0x7FFFFFFF
} OMX_VIDEO_MSMPEG4VERSIONTYPE;

typedef struct OMX_VIDEO_PARAM_MSMPEG4VERSIONTYPE {
    OMX_U32 nSize;
    OMX_VERSIONTYPE nVersion;
    OMX_U32 nPortIndex;
    OMX_VIDEO_MSMPEG4VERSIONTYPE eVersion;
} OMX_VIDEO_PARAM_MSMPEG4VERSIONTYPE;

/** Divx */
typedef enum OMX_VIDEO_DIVXVERSIONTYPE {
    OMX_VIDEO_DIVXVersionUnused,
    OMX_VIDEO_DIVXVersion1,
    OMX_VIDEO_DIVXVersion2,
    OMX_VIDEO_DIVXVersion3,
    OMX_VIDEO_DIVXVersion4,
    OMX_VIDEO_DIVXVersion5,
    OMX_VIDEO_DIVXVersionMax       = 0x7FFFFFFF
} OMX_VIDEO_DIVXVERSIONTYPE;

typedef struct OMX_VIDEO_PARAM_DIVXVERSIONTYPE {
    OMX_U32 nSize;
    OMX_VERSIONTYPE nVersion;
    OMX_U32 nPortIndex;
    OMX_VIDEO_DIVXVERSIONTYPE eVersion;
} OMX_VIDEO_PARAM_DIVXVERSIONTYPE;

/** Output stride */
typedef struct OMX_VIDEO_PARAM_STRIDETYPE {
    OMX_U32 nSize;
    OMX_VERSIONTYPE nVersion;
    OMX_U32 nPortIndex;
    OMX_S32 nStride;
    OMX_S32 nStrideY;
    OMX_S32 nStrideCbCr;
} OMX_VIDEO_PARAM_STRIDETYPE;

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* OMX_VideoExt_h */
/* File EOF */
