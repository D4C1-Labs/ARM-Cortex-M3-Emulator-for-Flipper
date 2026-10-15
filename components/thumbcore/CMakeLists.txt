# The Thumb-2 (ARMv7-M) interpreter core, shared with the Flipper PandoraARM app.
# Pure C99/C11, no platform dependencies beyond the C library.
idf_component_register(
    SRCS "thumb_core.c"
    INCLUDE_DIRS "include"
)
