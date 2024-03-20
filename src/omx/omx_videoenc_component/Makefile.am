lib_LTLIBRARIES = libomxvideoenc.la

libomxvideoenc_la_SOURCES = \
	src/omx_videoenc_component.c \
	include/omx_videoenc_component.h

libomxvideoenc_la_LIBADD = \
	$(top_builddir)/src/omx/omx_base/libomxbase.la \
	$(top_builddir)/src/omx/omx_videoenc_interface/libomxvideoenccore.la \
	-lglib-2.0 -lgobject-2.0 -lgio-2.0

libomxvideoenc_la_CFLAGS = \
	-I$(top_srcdir)/src/omx/omx_include \
	-I$(top_srcdir)/src/omx/omx_base/include \
	-I$(top_srcdir)/src/omx/omx_videoenc_component/include \
	-I$(top_srcdir)/src/omx/omx_videoenc_interface/include \
	-I$(LINUX_PLATFORM_ROOTDIR)/prebuilts/include/glib-2.0 \
	-I$(LINUX_PLATFORM_ROOTDIR)/prebuilts/lib/glib-2.0/include
	
if IS_TCC803X
libomxvideoenc_la_CFLAGS += -DUSE_VENC
libomxvideoenc_la_CFLAGS += -I$(LINUX_PLATFORM_KERNELDIR)/include
libomxvideoenc_la_CFLAGS += -I$(LINUX_PLATFORM_KERNELDIR)/include/video/tcc
libomxvideoenc_la_CFLAGS += -I$(LINUX_PLATFORM_KERNELDIR)/drivers/char/hdmi_v2_0/include
libomxvideoenc_la_CFLAGS += -DTCC_803X_INCLUDE
libomxvideoenc_la_CFLAGS += -DTCC_VPU_C7_INCLUDE
libomxvideoenc_la_CFLAGS += -DTCC_HEVC_INCLUDE
libomxvideoenc_la_CFLAGS += -DTCC_JPU_C6_INCLUDE
libomxvideoenc_la_CFLAGS += -DUSE_COMMON_KERNEL_LOCATION
endif

if IS_TCC805X
libomxvideoenc_la_CFLAGS += -DUSE_VENC
libomxvideoenc_la_CFLAGS += -I$(LINUX_PLATFORM_KERNELDIR)/include
libomxvideoenc_la_CFLAGS += -I$(LINUX_PLATFORM_KERNELDIR)/include/video/tcc
libomxvideoenc_la_CFLAGS += -I$(LINUX_PLATFORM_KERNELDIR)/drivers/char/hdmi_v2_0/include
libomxvideoenc_la_CFLAGS += -DTCC_805X_INCLUDE
libomxvideoenc_la_CFLAGS += -DTCC_VPU_C7_INCLUDE
libomxvideoenc_la_CFLAGS += -DTCC_VPU_4K_D2_INCLUDE
libomxvideoenc_la_CFLAGS += -DTCC_JPU_C6_INCLUDE
libomxvideoenc_la_CFLAGS += -DUSE_COMMON_KERNEL_LOCATION
endif

if WITH_LLOG
libomxvideoenc_la_LIBADD += -llog
libomxvideoenc_la_CFLAGS += -DTCC_LLOG_ENABLE
endif
EXTRA_DIST = include

