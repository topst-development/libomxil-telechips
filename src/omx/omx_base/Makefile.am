# Build order is important
# First base, then core, finally components
lib_LTLIBRARIES = libomxbase.la

libomxbase_la_SOURCES = src/omx_base_component.c \
			src/omx_base_port.c \
			src/omx_base_audio_port.c \
			src/omx_base_clock_port.c \
			src/omx_base_filter.c \
			src/omx_base_sink.c \
			src/omx_base_source.c \
			src/omx_base_video_port.c \
			src/omx_base_image_port.c \
			src/TCCMemory.c \
			src/queue.c \
			src/tsemaphore.c

libomxbase_la_CFLAGS = -I$(top_srcdir)/src/omx/omx_include \
		-I$(top_srcdir)/src/omx/omx_base/include

EXTRA_DIST = include
