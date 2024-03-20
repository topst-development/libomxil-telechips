lib_LTLIBRARIES =

if WITH_AACDEC
lib_LTLIBRARIES += libomxaacdec.la
libomxaacdec_la_SOURCES = \
			aac/omx_aacdec_component.c \
			aac/omx_aacdec_component.h
libomxaacdec_la_LIBADD = \
			$(top_builddir)/src/omx/omx_base/libomxbase.la \
			$(top_builddir)/src/omx/omx_audio_components/decoder/common/libomxaudiodec.la
libomxaacdec_la_CFLAGS = -I$(top_srcdir)/src/omx/omx_include \
			-I$(top_srcdir)/src/omx/omx_base/include \
			-I$(top_srcdir)/src/omx/omx_audio_components/include \
			-I$(top_srcdir)/src/omx/omx_audio_components/decoder/common \
			-I$(top_srcdir)/src/omx/omx_audio_components/decoder/components/aac
endif

if WITH_MP3DEC
lib_LTLIBRARIES += libomxmp3dec.la
libomxmp3dec_la_SOURCES = \
			mp3/omx_mp3dec_component.c \
			mp3/omx_mp3dec_component.h
libomxmp3dec_la_LIBADD = \
			$(top_builddir)/src/omx/omx_base/libomxbase.la \
			$(top_builddir)/src/omx/omx_audio_components/decoder/common/libomxaudiodec.la
libomxmp3dec_la_CFLAGS = \
			-I$(top_srcdir)/src/omx/omx_include \
			-I$(top_srcdir)/src/omx/omx_base/include \
			-I$(top_srcdir)/src/omx/omx_audio_components/include \
			-I$(top_srcdir)/src/omx/omx_audio_components/decoder/common \
			-I$(top_srcdir)/src/omx/omx_audio_components/decoder/components/mp3
endif

if WITH_FLACDEC
lib_LTLIBRARIES += libomxflacdec.la
libomxflacdec_la_SOURCES = \
				flac/omx_flacdec_component.c \
				flac/omx_flacdec_component.h
libomxflacdec_la_LIBADD = \
			$(top_builddir)/src/omx/omx_base/libomxbase.la \
			$(top_builddir)/src/omx/omx_audio_components/decoder/common/libomxaudiodec.la
libomxflacdec_la_CFLAGS = \
			-I$(top_srcdir)/src/omx/omx_include \
			-I$(top_srcdir)/src/omx/omx_base/include \
			-I$(top_srcdir)/src/omx/omx_audio_components/include \
			-I$(top_srcdir)/src/omx/omx_audio_components/decoder/common \
			-I$(top_srcdir)/src/omx/omx_audio_components/decoder/components/flac
endif

if WITH_APEDEC
lib_LTLIBRARIES += libomxapedec.la
libomxapedec_la_SOURCES = \
			ape/omx_apedec_component.c \
			ape/omx_apedec_component.h
libomxapedec_la_LIBADD = \
			$(top_builddir)/src/omx/omx_base/libomxbase.la \
			$(top_builddir)/src/omx/omx_audio_components/decoder/common/libomxaudiodec.la
libomxapedec_la_CFLAGS = \
			-I$(top_srcdir)/src/omx/omx_include \
			-I$(top_srcdir)/src/omx/omx_base/include \
			-I$(top_srcdir)/src/omx/omx_audio_components/include \
			-I$(top_srcdir)/src/omx/omx_audio_components/decoder/common \
			-I$(top_srcdir)/src/omx/omx_audio_components/decoder/components/ape
endif

if WITH_WAVDEC
lib_LTLIBRARIES += libomxpcmdec.la
libomxpcmdec_la_SOURCES = \
			wav/omx_wavdec_component.c \
			wav/omx_wavdec_component.h
libomxpcmdec_la_LIBADD = \
			$(top_builddir)/src/omx/omx_base/libomxbase.la \
			$(top_builddir)/src/omx/omx_audio_components/decoder/common/libomxaudiodec.la
libomxpcmdec_la_CFLAGS = -I$(top_srcdir)/src/omx/omx_include \
			-I$(top_srcdir)/src/omx/omx_base/include \
			-I$(top_srcdir)/src/omx/omx_audio_components/include \
			-I$(top_srcdir)/src/omx/omx_audio_components/decoder/common \
			-I$(top_srcdir)/src/omx/omx_audio_components/decoder/components/wav
endif

if WITH_AMRNBDEC
lib_LTLIBRARIES += libomxamrnbdec.la
libomxamrnbdec_la_SOURCES = \
			amrnb/omx_amrnbdec_component.c \
			amrnb/omx_amrnbdec_component.h
libomxamrnbdec_la_LIBADD = \
			$(top_builddir)/src/omx/omx_base/libomxbase.la \
			$(top_builddir)/src/omx/omx_audio_components/decoder/common/libomxaudiodec.la
libomxamrnbdec_la_CFLAGS = -I$(top_srcdir)/src/omx/omx_include \
			-I$(top_srcdir)/src/omx/omx_base/include \
			-I$(top_srcdir)/src/omx/omx_audio_components/include \
			-I$(top_srcdir)/src/omx/omx_audio_components/decoder/common \
			-I$(top_srcdir)/src/omx/omx_audio_components/decoder/components/amrnb
endif

if WITH_AC3DEC
lib_LTLIBRARIES += libomxac3dec.la
libomxac3dec_la_SOURCES = \
			ac3/omx_ac3dec_component.c \
			ac3/omx_ac3dec_component.h
libomxac3dec_la_LIBADD = \
			$(top_builddir)/src/omx/omx_base/libomxbase.la \
			$(top_builddir)/src/omx/omx_audio_components/decoder/common/libomxaudiodec.la
libomxac3dec_la_CFLAGS = -I$(top_srcdir)/src/omx/omx_include \
			-I$(top_srcdir)/src/omx/omx_base/include \
			-I$(top_srcdir)/src/omx/omx_audio_components/include \
			-I$(top_srcdir)/src/omx/omx_audio_components/decoder/common \
			-I$(top_srcdir)/src/omx/omx_audio_components/decoder/components/ac3
endif

if WITH_DDPDEC
lib_LTLIBRARIES += libomxac3dec.la
libomxac3dec_la_SOURCES = \
			ddp/omx_ddpdec_component.c \
			ddp/omx_ddpdec_component.h
libomxac3dec_la_LIBADD = \
			$(top_builddir)/src/omx/omx_base/libomxbase.la \
			$(top_builddir)/src/omx/omx_audio_components/decoder/common/libomxaudiodec.la
libomxac3dec_la_CFLAGS = -I$(top_srcdir)/src/omx/omx_include \
			-I$(top_srcdir)/src/omx/omx_base/include \
			-I$(top_srcdir)/src/omx/omx_audio_components/include \
			-I$(top_srcdir)/src/omx/omx_audio_components/decoder/common \
			-I$(top_srcdir)/src/omx/omx_audio_components/decoder/components/ddp
endif

if WITH_DTSDEC
lib_LTLIBRARIES += libomxdtsdec.la
libomxdtsdec_la_SOURCES = \
			dts/omx_dtsdec_component.c \
			dts/omx_dtsdec_component.h
libomxdtsdec_la_LIBADD = \
			$(top_builddir)/src/omx/omx_base/libomxbase.la \
			$(top_builddir)/src/omx/omx_audio_components/decoder/common/libomxaudiodec.la
libomxdtsdec_la_CFLAGS = -I$(top_srcdir)/src/omx/omx_include \
			-I$(top_srcdir)/src/omx/omx_base/include \
			-I$(top_srcdir)/src/omx/omx_audio_components/include \
			-I$(top_srcdir)/src/omx/omx_audio_components/decoder/common \
			-I$(top_srcdir)/src/omx/omx_audio_components/decoder/components/dts
endif

if WITH_WMADEC
lib_LTLIBRARIES += libomxwmadec.la
libomxwmadec_la_SOURCES = \
			wma/omx_wmadec_component.c \
			wma/omx_wmadec_component.h
libomxwmadec_la_LIBADD = \
			$(top_builddir)/src/omx/omx_base/libomxbase.la \
			$(top_builddir)/src/omx/omx_audio_components/decoder/common/libomxaudiodec.la
libomxwmadec_la_CFLAGS = -I$(top_srcdir)/src/omx/omx_include \
			-I$(top_srcdir)/src/omx/omx_base/include \
			-I$(top_srcdir)/src/omx/omx_audio_components/include \
			-I$(top_srcdir)/src/omx/omx_audio_components/decoder/common \
			-I$(top_srcdir)/src/omx/omx_audio_components/decoder/components/wma
endif

if WITH_OPUSDEC
lib_LTLIBRARIES += libomxopusdec.la
libomxopusdec_la_SOURCES = \
			opus/omx_opusdec_component.c \
			opus/omx_opusdec_component.h
libomxopusdec_la_LIBADD = \
			$(top_builddir)/src/omx/omx_base/libomxbase.la \
			$(top_builddir)/src/omx/omx_audio_components/decoder/common/libomxaudiodec.la
libomxopusdec_la_CFLAGS = \
			-I$(top_srcdir)/src/omx/omx_include \
			-I$(top_srcdir)/src/omx/omx_base/include \
			-I$(top_srcdir)/src/omx/omx_audio_components/include \
			-I$(top_srcdir)/src/omx/omx_audio_components/decoder/common \
			-I$(top_srcdir)/src/omx/omx_audio_components/decoder/components/opus
endif
