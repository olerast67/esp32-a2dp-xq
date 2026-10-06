# SPDX-License-Identifier: Apache-2.0
# esp32-a2dp-xq
#
# ESP-IDF: this directory is the component (Component Registry: olerast67/esp32-a2dp-xq).
# Host (Linux, macOS, Windows): add_subdirectory() gives the static library `a2dp_xq_portable`
# with the pure-C parts (SBC arithmetic, PCM FIFO, device memory, volume); test/ builds the
# host tests on top of it.

set(A2DP_XQ_PORTABLE_SOURCES src/xq_fifo.c src/xq_devices.c src/xq_sbc.c src/xq_volume.c src/xq_util.c)

if(ESP_PLATFORM)
    idf_component_register(SRCS ${A2DP_XQ_PORTABLE_SOURCES} src/xq_dither.c src/xq_sink.c src/a2dp_xq.c
                           INCLUDE_DIRS src
                           REQUIRES bt
                           PRIV_REQUIRES esp_timer esp_hw_support heap)
    target_compile_options(${COMPONENT_LIB} PRIVATE -O2 -Wno-unused-parameter -Wno-missing-field-initializers)
    return()
endif()

cmake_minimum_required(VERSION 3.16)
project(esp32_a2dp_xq C)

set(_xq_sources)
foreach(_f ${A2DP_XQ_PORTABLE_SOURCES})
    list(APPEND _xq_sources ${CMAKE_CURRENT_LIST_DIR}/${_f})
endforeach()
add_library(a2dp_xq_portable STATIC ${_xq_sources})
target_include_directories(a2dp_xq_portable PUBLIC ${CMAKE_CURRENT_LIST_DIR}/src)
set_target_properties(a2dp_xq_portable PROPERTIES C_STANDARD 17 C_STANDARD_REQUIRED ON)
if(NOT MSVC)
    target_compile_options(a2dp_xq_portable PRIVATE -Wall -Wextra -Wno-unused-parameter
                                                    -Wno-missing-field-initializers)
endif()
if(NOT WIN32)
    target_link_libraries(a2dp_xq_portable PUBLIC m)
endif()
