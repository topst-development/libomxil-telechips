#!/bin/bash

rm -rf $LINUX_PLATFORM_ROOTDIR/$BUILDDIR/libomxil-tcc-1.0.0/include
rm -rf $LINUX_PLATFORM_ROOTDIR/$BUILDDIR/libomxil-tcc-1.0.0/share

rm -rf `find $LINUX_PLATFORM_ROOTDIR/$BUILDDIR/libomxil-tcc-1.0.0 -name "*.a"`
rm -rf `find $LINUX_PLATFORM_ROOTDIR/$BUILDDIR/libomxil-tcc-1.0.0 -name "*.la"`
rm -rf `find $LINUX_PLATFORM_ROOTDIR/$BUILDDIR/libomxil-tcc-1.0.0 -name "*.pc"`

arm-none-linux-gnueabi-strip --strip-debug --strip-unneeded `find $LINUX_PLATFORM_ROOTDIR/$BUILDDIR/libomxil-tcc-1.0.0/ -name "*.so*"`
