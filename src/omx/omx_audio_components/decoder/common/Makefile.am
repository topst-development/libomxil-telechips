lib_LTLIBRARIES = libomxaudiodec.la

libomxaudiodec_la_SOURCES = omx_audiodec_component.c omx_audiodec_component.h

libomxaudiodec_la_CFLAGS = -I$(top_srcdir)/src/omx/omx_include \
			-I$(top_srcdir)/src/omx/omx_base/include \
			-I$(top_srcdir)/src/omx/omx_audio_components/include

libomxaudiodec_la_LIBADD = \
			$(top_builddir)/src/omx/omx_base/libomxbase.la

if WITH_LLOG
libomxaudiodec_la_LIBADD += -llog
libomxaudiodec_la_CFLAGS += -DTCC_LLOG_ENABLE
endif
