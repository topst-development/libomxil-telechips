#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.65])
AC_INIT([libomxil-tcc], [1.1.0], [www.telechips.com])
AM_INIT_AUTOMAKE([foreign subdir-objects -Wall -Werror])

#AC_CONFIG_SRCDIR([include/CDK/cdk/cdk.h])
AC_CONFIG_HEADERS([config.h])

# Set to 'm4' the directory where the extra autoconf macros are stored
AC_CONFIG_MACRO_DIR([m4])

AM_PROG_AR

#Check for argument
AC_ARG_ENABLE(debug,
			 AC_HELP_STRING([--enable-debug],[enable debugging (default:no)]),
			 [CFLAGS="$CFLAGS -w -g3 -O0" CXXFLAGS="$CXXFLAGS -w -g3 -O0"],
			 [CFLAGS="$CFLAGS -w -O3" CXXFLAGS="$CXXFLAGS -w -O3"])

#telechips processor setup
AC_ARG_VAR(PROCESSOR, telechips processor setting (tcc802x, tcc803x, tcc805x, tcc892x, tcc8925s, tcc893x, tcc8935s, tcc896x, tcc897x, tcc898x, tcc899x or simulate))
AS_IF([test "x$PROCESSOR" = x], [PROCESSOR="simulate"])
AM_CONDITIONAL(IS_TCC802X, test "x$PROCESSOR" = "xtcc802x")
AM_CONDITIONAL(IS_TCC803X, test "x$PROCESSOR" = "xtcc803x")
AM_CONDITIONAL(IS_TCC805X, test "x$PROCESSOR" = "xtcc805x")
AM_CONDITIONAL(IS_TCC899X, test "x$PROCESSOR" = "xtcc899x")
AM_CONDITIONAL(IS_TCC898X, test "x$PROCESSOR" = "xtcc898x")
AM_CONDITIONAL(IS_TCC897X, test "x$PROCESSOR" = "xtcc897x")
AM_CONDITIONAL(IS_TCC896X, test "x$PROCESSOR" = "xtcc896x")
AM_CONDITIONAL(IS_TCC893X, test "x$PROCESSOR" = "xtcc893x")
AM_CONDITIONAL(IS_TCC8935S, test "x$PROCESSOR" = "xtcc8935s")
AM_CONDITIONAL(IS_TCC892X, test "x$PROCESSOR" = "xtcc892x")
AM_CONDITIONAL(IS_TCC8925S, test "x$PROCESSOR" = "xtcc8925s")
AM_CONDITIONAL(IS_SIMULATE, test "x$PROCESSOR" = "xsimulate")
#Check whether llog have been requested
AC_ARG_ENABLE(
    [llog],
    [AC_HELP_STRING(
        [--enable-llog],
        [whether to enable the llog (default:no)])],
        [with_llog=$enableval],
        [with_llog=no])

#Check whether AAC Decoder components have been requested
AC_ARG_ENABLE(
    [aacdec],
    [AC_HELP_STRING(
        [--disable-aacdec],
        [whether to disable the aacdec component])],
    [with_aacdec=$enableval],
    [with_aacdec=yes])

#Check whether MP3 Decoder components have been requested
AC_ARG_ENABLE(
    [mp3dec],
    [AC_HELP_STRING(
        [--disable-mp3dec],
        [whether to disable the mp3dec component])],
    [with_mp3dec=$enableval],
    [with_mp3dec=yes])

#Check whether AC3 Decoder components have been requested
AC_ARG_ENABLE(
    [ac3dec],
    [AC_HELP_STRING(
        [--enable-ac3dec],
        [whether to enable the ac3dec component])],
    [with_ac3dec=$enableval],
    [with_ac3dec=no])

#Check whether DDP Decoder components have been requested
AC_ARG_ENABLE(
    [ddpdec],
    [AC_HELP_STRING(
        [--enable-ddpdec],
        [whether to enable the ddpdec component])],
    [with_ddpdec=$enableval],
    [with_ddpdec=no])

#Check whether DTS Decoder components have been requested
AC_ARG_ENABLE(
    [dtsdec],
    [AC_HELP_STRING(
        [--enable-dtsdec],
        [whether to enable the dtsdec component])],
    [with_dtsdec=$enableval],
    [with_dtsdec=no])

#Check whether WAV Decoder components have been requested
AC_ARG_ENABLE(
    [wavdec],
    [AC_HELP_STRING(
        [--disable-wavdec],
        [whether to disable the wavdec component])],
    [with_wavdec=$enableval],
    [with_wavdec=yes])

#Check whether AMRNB Decoder components have been requested
AC_ARG_ENABLE(
    [amrnbdec],
    [AC_HELP_STRING(
        [--disable-amrnbdec],
        [whether to disable the amrnbdec component])],
    [with_amrnbdec=$enableval],
    [with_amrnbdec=yes])

#Check whether FLAC Decoder components have been requested
AC_ARG_ENABLE(
    [flacdec],
    [AC_HELP_STRING(
        [--disable-flacdec],
        [whether to disable the flacdec component])],
    [with_flacdec=$enableval],
    [with_flacdec=yes])

#Check whether APE Decoder components have been requested
AC_ARG_ENABLE(
    [apedec],
    [AC_HELP_STRING(
        [--disable-apedec],
        [whether to disable the apedec component])],
    [with_apedec=$enableval],
    [with_apedec=yes])

#Check whether WMA Decoder components have been requested
AC_ARG_ENABLE(
    [wmadec],
    [AC_HELP_STRING(
        [--enable-wmadec],
        [whether to enable the wmadec component])],
    [with_wmadec=$enableval],
    [with_wmadec=no])

#Check whether WMV7/8 Decoder components have been requested
AC_ARG_ENABLE(
    [wmv78dec],
    [AC_HELP_STRING(
        [--enable-wmv78dec],
        [whether to enable the WMV7/8 component])],
    [with_wmv78dec=$enableval],
    [with_wmv78dec=no])

#Check whether WMV9 Decoder components have been requested
AC_ARG_ENABLE(
    [wmv9dec],
    [AC_HELP_STRING(
        [--enable-wmv9dec],
        [whether to enable the WMV9 component])],
    [with_wmv9dec=$enableval],
    [with_wmv9dec=no])

#Check whether OPUS Decoder components have been requested
AC_ARG_ENABLE(
    [opusdec],
    [AC_HELP_STRING(
        [--disable-opusdec],
        [whether to disable the opusdec component])],
    [with_opusdec=$enableval],
    [with_opusdec=yes])

#Check whether Video Decoder components have been requested
AC_ARG_ENABLE(
    [videodec],
    [AC_HELP_STRING(
        [--disable-videodec],
        [whether to disable the videodec component])],
    [with_videodec=$enableval],
    [with_videodec=yes])

#Check whether Video Encoder components have been requested
AC_ARG_ENABLE(
    [videoenc],
    [AC_HELP_STRING(
        [--enable-videoenc],
        [whether to enable the videoenc component])],
    [with_videoenc=$enableval],
    [with_videoenc=no])

#Check whether to disable all audio components
AC_ARG_ENABLE(
    [audiocomponents],
    [AC_HELP_STRING(
        [--disalbe-audiocomponents],
        [whether to disable all audio components])],
    [with_audiocomponents=$enableval],
    [with_audiocomponents=yes])

#Check whether Secure Video have been requested
AC_ARG_ENABLE(
    [securevideo],
    [AC_HELP_STRING(
        [--enable-securevideo],
        [whether to enable the secure video])],
    [with_securevideo=$enableval],
    [with_securevideo=no])

#Check whether Set MAX framebuffer  have been requested
AC_ARG_ENABLE(
    [maxframebuffer],
    [AC_HELP_STRING(
        [--enable-maxframebuffer],
        [whether to enable the maxframebuffer])],
    [with_maxframebuffer=$enableval],
    [with_maxframebuffer=no])

#Check Dolby-Vision
AC_ARG_ENABLE(
    [dolbyvision],
    [AC_HELP_STRING(
        [--enable-dolbyvision],
        [whether to enable the dolbyvision])],
    [with_dolbyvision=$enableval],
    [with_dolbyvision=no])

if test "x$with_ac3dec" = "xyes" && test "x$with_ddpdec" = "xyes"; then
	with_ac3dec=no
	with_ddpdec=no
	AC_MSG_WARN([AC3 and DDP decoder can not be applied simultaneously!])
	AC_MSG_WARN([both components are all disabled!])
fi

if test "x$with_audiocomponents" = "xno"; then
	with_aacdec=no
	with_ac3dec=no
	with_ddpdec=no
	with_dtsdec=no
	with_mp3dec=no
	with_flacdec=no
	with_wmadec=no
	with_apedec=no
	with_wavdec=no
	with_amrnbdec=no
	with_opusdec=no
fi

# Checks for programs.
AC_PROG_CXX
AC_PROG_CC

# Checks for libraries.

# Check for libtool
AM_PROG_LIBTOOL

# Checks for header files.
AC_CHECK_HEADERS([fcntl.h stddef.h stdint.h stdlib.h string.h sys/ioctl.h sys/time.h unistd.h])

# Checks for typedefs, structures, and compiler characteristics.
AC_HEADER_STDBOOL
AC_C_INLINE
AC_TYPE_INT16_T
AC_TYPE_INT32_T
AC_TYPE_INT64_T
AC_TYPE_INT8_T
AC_TYPE_SIZE_T
AC_TYPE_SSIZE_T
AC_TYPE_UINT16_T
AC_TYPE_UINT32_T
AC_TYPE_UINT64_T
AC_TYPE_UINT8_T

# Checks for library functions.
AC_FUNC_ERROR_AT_LINE
AC_FUNC_MALLOC
AC_FUNC_MMAP
AC_FUNC_REALLOC
AC_CHECK_FUNCS([gettimeofday memmove memset munmap select strchr strerror])

AC_CONFIG_FILES([Makefile
                 src/Makefile
                 src/omx/Makefile
                 src/omx/omx_audio_components/Makefile
                 src/omx/omx_base/Makefile
                 src/omx/omx_core/Makefile
                 src/omx/omx_audio_components/decoder/Makefile
                 src/omx/omx_audio_components/decoder/common/Makefile
                 src/omx/omx_audio_components/decoder/components/Makefile
                 src/omx/omx_videodec_component/Makefile
                 src/omx/omx_videodec_interface/Makefile
                 src/omx/omx_videoenc_component/Makefile
                 src/omx/omx_videoenc_interface/Makefile])

AM_CONDITIONAL([WITH_LLOG], [test x$with_llog = xyes])
AM_CONDITIONAL([WITH_AACDEC], [test x$with_aacdec = xyes])
AM_CONDITIONAL([WITH_AC3DEC], [test x$with_ac3dec = xyes])
AM_CONDITIONAL([WITH_DDPDEC], [test x$with_ddpdec = xyes])
AM_CONDITIONAL([WITH_DTSDEC], [test x$with_dtsdec = xyes])
AM_CONDITIONAL([WITH_MP3DEC], [test x$with_mp3dec = xyes])
AM_CONDITIONAL([WITH_FLACDEC], [test x$with_flacdec = xyes])
AM_CONDITIONAL([WITH_WMADEC], [test x$with_wmadec = xyes])
AM_CONDITIONAL([WITH_APEDEC], [test x$with_apedec = xyes])
AM_CONDITIONAL([WITH_WAVDEC], [test x$with_wavdec = xyes])
AM_CONDITIONAL([WITH_AMRNBDEC], [test x$with_amrnbdec = xyes])
AM_CONDITIONAL([WITH_WMV78DEC], [test x$with_wmv78dec = xyes])
AM_CONDITIONAL([WITH_WMV9DEC], [test x$with_wmv9dec = xyes])
AM_CONDITIONAL([WITH_VIDEODEC], [test x$with_videodec = xyes])
AM_CONDITIONAL([WITH_VIDEOENC], [test x$with_videoenc = xyes])
AM_CONDITIONAL([WITH_SECUREVIDEO], [test x$with_securevideo = xyes])
AM_CONDITIONAL([WITH_MAXFRAMEBUFFER], [test x$with_maxframebuffer = xyes])
AM_CONDITIONAL([WITH_DOLBYVISION], [test x$with_dolbyvision = xyes])
AM_CONDITIONAL([WITH_OPUSDEC], [test x$with_opusdec = xyes])

AC_OUTPUT
