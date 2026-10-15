# Top-level ESP-IDF project file for the Pandora D-605 emulator on ESP32.
# Build:  idf.py set-target esp32 && idf.py build
# Flash:  idf.py -p /dev/ttyUSB0 flash monitor
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(pandora-esp32)
