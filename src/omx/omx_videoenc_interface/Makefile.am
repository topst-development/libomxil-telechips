lib_LTLIBRARIES = libomxvideoenccore.la

libomxvideoenccore_la_SOURCES = \
	src/venc_k.c \
	include/venc.h

libomxvideoenccore_la_LIBADD = \
	$(top_builddir)/src/omx/omx_base/libomxbase.la

libomxvideoenccore_la_CFLAGS = \
	-I$(top_srcdir)/src/omx/omx_include \
	-I$(top_srcdir)/src/omx/omx_base/include \
	-I$(top_srcdir)/src/omx/omx_videoenc_interface/include

if IS_TCC803X
libomxvideoenccore_la_CFLAGS += -DUSE_VENC_K
libomxvideoenccore_la_CFLAGS += -I$(LINUX_PLATFORM_KERNELDIR)/include
libomxvideoenccore_la_CFLAGS += -I$(LINUX_PLATFORM_KERNELDIR)/include/video/tcc
libomxvideoenccore_la_CFLAGS += -I$(LINUX_PLATFORM_KERNELDIR)/drivers/char/hdmi_v2_0/include
libomxvideoenccore_la_CFLAGS += -DTCC_803X_INCLUDE
libomxvideoenccore_la_CFLAGS += -DTCC_VPU_C7_INCLUDE
libomxvideoenccore_la_CFLAGS += -DTCC_JPU_C6_INCLUDE
endif

if IS_TCC805X
libomxvideoenccore_la_CFLAGS += -DUSE_VENC_K
libomxvideoenccore_la_CFLAGS += -I$(LINUX_PLATFORM_KERNELDIR)/include
libomxvideoenccore_la_CFLAGS += -I$(LINUX_PLATFORM_KERNELDIR)/include/video/tcc
libomxvideoenccore_la_CFLAGS += -I$(LINUX_PLATFORM_KERNELDIR)/drivers/char/hdmi_v2_0/include
libomxvideoenccore_la_CFLAGS += -DTCC_805X_INCLUDE
libomxvideoenccore_la_CFLAGS += -DTCC_VPU_C7_INCLUDE
libomxvideoenccore_la_CFLAGS += -DTCC_VPU_4K_D2_INCLUDE
libomxvideoenccore_la_CFLAGS += -DTCC_JPU_C6_INCLUDE
endif
