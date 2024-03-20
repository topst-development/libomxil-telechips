lib_LTLIBRARIES = libomxvideodec.la

libomxvideodec_la_SOURCES = \
	src/omx_videodec_component.c \
	include/omx_videodec_component.h

libomxvideodec_la_LIBADD = \
	$(top_builddir)/src/omx/omx_base/libomxbase.la \
	$(top_builddir)/src/omx/omx_videodec_interface/libomxvideodeccore.la \
	-lglib-2.0 -lgobject-2.0 -lgio-2.0

libomxvideodec_la_CFLAGS = \
	-I$(top_srcdir)/src/omx/omx_include \
	-I$(top_srcdir)/src/omx/omx_base/include \
	-I$(top_srcdir)/src/omx/omx_videodec_component/include \
	-I$(top_srcdir)/src/omx/omx_videodec_interface/include \
	-I$(LINUX_PLATFORM_ROOTDIR)/prebuilts/include/glib-2.0 \
	-I$(LINUX_PLATFORM_ROOTDIR)/prebuilts/lib/glib-2.0/include


if IS_TCC897X
libomxvideodec_la_CFLAGS += -DUSE_VDEC_K_V2
libomxvideodec_la_CFLAGS += -I$(LINUX_PLATFORM_KERNELDIR)/arch/arm/mach-tcc897x/include
libomxvideodec_la_CFLAGS += -DTCC_897X_INCLUDE
libomxvideodec_la_CFLAGS += -DTCC_VPU_D6_INCLUDE
libomxvideodec_la_CFLAGS += -DTCC_JPU_C6_INCLUDE
libomxvideodec_la_CFLAGS += -DTCC_HEVC_INCLUDE
endif

if IS_TCC802X
libomxvideodec_la_CFLAGS += -DUSE_VDEC_K_V2
libomxvideodec_la_CFLAGS += -I$(LINUX_PLATFORM_KERNELDIR)/arch/arm/mach-tcc802x/include
libomxvideodec_la_CFLAGS += -DTCC_802X_INCLUDE
libomxvideodec_la_CFLAGS += -DTCC_VPU_C7_INCLUDE
libomxvideodec_la_CFLAGS += -DTCC_JPU_C6_INCLUDE
endif

if IS_TCC803X
libomxvideodec_la_CFLAGS += -DUSE_VDEC_K_V2
libomxvideodec_la_CFLAGS += -I$(LINUX_PLATFORM_KERNELDIR)/include
libomxvideodec_la_CFLAGS += -I$(LINUX_PLATFORM_KERNELDIR)/include/video/tcc
libomxvideodec_la_CFLAGS += -I$(LINUX_PLATFORM_KERNELDIR)/drivers/char/hdmi_v2_0/include
libomxvideodec_la_CFLAGS += -DTCC_803X_INCLUDE
libomxvideodec_la_CFLAGS += -DTCC_VPU_C7_INCLUDE
libomxvideodec_la_CFLAGS += -DTCC_HEVC_INCLUDE
libomxvideodec_la_CFLAGS += -DTCC_JPU_C6_INCLUDE
libomxvideodec_la_CFLAGS += -DUSE_COMMON_KERNEL_LOCATION
endif

if IS_TCC805X
libomxvideodec_la_CFLAGS += -DUSE_VDEC_K_V3
libomxvideodec_la_CFLAGS += -I$(LINUX_PLATFORM_KERNELDIR)/include
libomxvideodec_la_CFLAGS += -I$(LINUX_PLATFORM_KERNELDIR)/include/video/tcc
libomxvideodec_la_CFLAGS += -I$(LINUX_PLATFORM_KERNELDIR)/drivers/char/hdmi_v2_0/include
libomxvideodec_la_CFLAGS += -DTCC_805X_INCLUDE
libomxvideodec_la_CFLAGS += -DTCC_VPU_C7_INCLUDE
libomxvideodec_la_CFLAGS += -DTCC_VPU_4K_D2_INCLUDE
libomxvideodec_la_CFLAGS += -DTCC_JPU_C6_INCLUDE
libomxvideodec_la_CFLAGS += -DUSE_COMMON_KERNEL_LOCATION
endif

if WITH_WMV78DEC
libomxvideodec_la_CFLAGS += -I$(top_srcdir)/../cdk/videocodec
libomxvideodec_la_CFLAGS += -DINCLUDE_WMV78_DEC
endif

if WITH_WMV9DEC
libomxvideodec_la_CFLAGS += -DINCLUDE_WMV9_DEC
endif

if WITH_MAXFRAMEBUFFER
libomxvideodec_la_CFLAGS += -DSET_FRAMEBUFFER_INTO_MAX
endif

if WITH_SECUREVIDEO
libomxvideodec_la_LIBADD += \
	-L$(LINUX_PLATFORM_ROOTDIR)/bsp/ramdisk/root_dir/lib \
	-lteesmapi

libomxvideodec_la_CFLAGS += -DTC_SECURE_MEMORY_COPY
endif

if WITH_LLOG
libomxvideodec_la_LIBADD += -llog
libomxvideodec_la_CFLAGS += -DTCC_LLOG_ENABLE
endif
EXTRA_DIST = include
