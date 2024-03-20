if WITH_VIDEODEC
    MAYBE_VIDEODEC = omx_videodec_component
    MAYBE_VIDEOCORE = omx_videodec_interface
endif
if WITH_VIDEOENC
    MAYBE_VIDEOENC = omx_videoenc_component
    MAYBE_VIDEOENCCORE = omx_videoenc_interface
endif

# Build order is important
# First base, then core, finally components
SUBDIRS = omx_base omx_core omx_audio_components $(MAYBE_VIDEOCORE) $(MAYBE_VIDEODEC) $(MAYBE_VIDEOENCCORE) $(MAYBE_VIDEOENC)

EXTRA_DIST = omx_include
