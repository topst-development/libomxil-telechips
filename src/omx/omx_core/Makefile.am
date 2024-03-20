# Build order is important
# First base, then core, finally components

lib_LTLIBRARIES = libomxil-tcc.la

libomxil_tcc_la_SOURCES = tcc_omxcore.c

libomxil_tcc_la_CFLAGS = -I$(top_srcdir)/src/omx/omx_include \
			 -I$(top_srcdir)/src/omx/omx_base/include
