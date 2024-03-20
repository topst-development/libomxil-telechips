lib_LTLIBRARIES = libomxvideodeccore.la

libomxvideodeccore_la_SOURCES = \
	src/vdec_k_common.c \
	src/vdec_k_vpu.c \
	src/vdec_k_hevc.c \
	src/vdec_k_vpu_4kd2.c \
	src/vdec_k_wmv78.c \
	src/vdec_k_mjpeg.c \
	include/vdec.h

libomxvideodeccore_la_LIBADD = \
	$(top_builddir)/src/omx/omx_base/libomxbase.la

libomxvideodeccore_la_CFLAGS = \
	-I$(top_srcdir)/src/omx/omx_include \
	-I$(top_srcdir)/src/omx/omx_base/include \
	-I$(top_srcdir)/src/omx/omx_videodec_interface/include

if WITH_SECUREVIDEO
libomxvideodeccore_la_LIBADD += -L$(LINUX_PLATFORM_ROOTDIR)/bsp/ramdisk/root_dir/lib
libomxvideodeccore_la_LIBADD += -lteesmapi
libomxvideodeccore_la_CFLAGS += -DTC_SECURE_MEMORY_COPY
endif


if WITH_MAXFRAMEBUFFER
libomxvideodeccore_la_CFLAGS += -DSET_FRAMEBUFFER_INTO_MAX
endif


if IS_TCC897X
libomxvideodeccore_la_CFLAGS += -I$(LINUX_PLATFORM_KERNELDIR)/arch/arm/mach-tcc897x/include
libomxvideodeccore_la_CFLAGS += -DTCC_897X_INCLUDE
libomxvideodeccore_la_CFLAGS += -DTCC_VPU_D6_INCLUDE
libomxvideodeccore_la_CFLAGS += -DTCC_JPU_C6_INCLUDE
libomxvideodeccore_la_CFLAGS += -DTCC_HEVC_INCLUDE
endif

if IS_TCC802X
libomxvideodeccore_la_CFLAGS += -I$(LINUX_PLATFORM_KERNELDIR)/arch/arm/mach-tcc802x/include
libomxvideodeccore_la_CFLAGS += -DTCC_802X_INCLUDE
libomxvideodeccore_la_CFLAGS += -DTCC_VPU_C7_INCLUDE
libomxvideodeccore_la_CFLAGS += -DTCC_JPU_C6_INCLUDE
endif

if IS_TCC803X
libomxvideodeccore_la_CFLAGS += -I$(LINUX_PLATFORM_KERNELDIR)/include
libomxvideodeccore_la_CFLAGS += -I$(LINUX_PLATFORM_KERNELDIR)/include/video/tcc
libomxvideodeccore_la_CFLAGS += -I$(LINUX_PLATFORM_KERNELDIR)/drivers/char/hdmi_v2_0/include
libomxvideodeccore_la_CFLAGS += -DTCC_803X_INCLUDE
libomxvideodeccore_la_CFLAGS += -DTCC_VPU_C7_INCLUDE
libomxvideodeccore_la_CFLAGS += -DTCC_HEVC_INCLUDE
libomxvideodeccore_la_CFLAGS += -DTCC_JPU_C6_INCLUDE
libomxvideodeccore_la_CFLAGS += -DUSE_COMMON_KERNEL_LOCATION
endif

if IS_TCC805X
libomxvideodeccore_la_CFLAGS += -I$(LINUX_PLATFORM_KERNELDIR)/include
libomxvideodeccore_la_CFLAGS += -I$(LINUX_PLATFORM_KERNELDIR)/include/video/tcc
libomxvideodeccore_la_CFLAGS += -I$(LINUX_PLATFORM_KERNELDIR)/drivers/char/hdmi_v2_0/include
libomxvideodeccore_la_CFLAGS += -DTCC_805X_INCLUDE
libomxvideodeccore_la_CFLAGS += -DTCC_VPU_C7_INCLUDE
libomxvideodeccore_la_CFLAGS += -DTCC_VPU_4K_D2_INCLUDE
libomxvideodeccore_la_CFLAGS += -DTCC_JPU_C6_INCLUDE
libomxvideodeccore_la_CFLAGS += -DUSE_COMMON_KERNEL_LOCATION
endif

if WITH_WMV78DEC
libomxvideodeccore_la_CFLAGS += -I$(top_srcdir)/../cdk/videocodec
libomxvideodeccore_la_CFLAGS += -DINCLUDE_WMV78_DEC
endif

if WITH_WMV9DEC
libomxvideodeccore_la_CFLAGS += -DINCLUDE_WMV9_DEC
endif
