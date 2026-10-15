# The ESP32 application component.
#
# Firmware source: by default the keyfob firmware is NOT embedded. It is streamed
# from the Flipper over UART as Intel HEX (see protocol.h FW_BEGIN/CHUNK/END and
# ihex.c), parsed incrementally into a flat flash image, then booted.
#
# Optional fallback: build with -DPANDORA_EMBED_FW to also embed firmware.bin and
# boot it immediately. In that case firmware.bin is exposed as the symbols
# _binary_firmware_bin_start / _binary_firmware_bin_end (see main.c), and you may
# select the embedded profile with -DPANDORA_PROFILE="(&PROFILE_PANDORA)".
set(embed_files "")
if(DEFINED PANDORA_EMBED_FW)
    set(embed_files "firmware.bin")
endif()

idf_component_register(
    SRCS
        "main.c"
        "emulator.c"
        "uart_proto.c"
        "ihex.c"
    INCLUDE_DIRS
        "."
    EMBED_FILES
        ${embed_files}
    REQUIRES
        thumbcore
        driver
        esp_timer
        esp_driver_uart
        esp_driver_gpio
)

# Uncomment for the ASCII framebuffer dump over the ESP-IDF log (no Flipper
# needed to validate boot/render):
# target_compile_definitions(${COMPONENT_LIB} PRIVATE PANDORA_DEBUG_ASCII=1)
