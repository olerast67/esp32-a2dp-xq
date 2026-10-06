# SPDX-License-Identifier: Apache-2.0
# Host tests for the pure-C parts of esp32-a2dp-xq: SBC arithmetic and the XQ decision, the
# PCM FIFO, the volume scales and the per-device memory file. No ESP-IDF needed.
#
#   cmake -S test -B build/test -G Ninja
#   cmake --build build/test
#   ctest --test-dir build/test --output-on-failure
#
# Windows without a C compiler: python -m pip install ziglang cmake ninja, then add
#   -DCMAKE_TOOLCHAIN_FILE=<absolute path>/cmake/zig-toolchain.cmake
# Sanitizers (Linux/macOS with GCC or Clang): -DA2DP_XQ_SANITIZE=ON
cmake_minimum_required(VERSION 3.16)
project(esp32_a2dp_xq_tests C)

option(A2DP_XQ_SANITIZE "Build with AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
if(A2DP_XQ_SANITIZE)
    add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=undefined)
    add_link_options(-fsanitize=address,undefined)
endif()

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/.. a2dp_xq)

enable_testing()
add_executable(test_a2dp_xq test_a2dp_xq.c)
target_link_libraries(test_a2dp_xq PRIVATE a2dp_xq_portable)
target_include_directories(test_a2dp_xq PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_compile_definitions(test_a2dp_xq PRIVATE TEST_TMP_DIR="${CMAKE_CURRENT_BINARY_DIR}")
if(NOT MSVC)
    target_compile_options(test_a2dp_xq PRIVATE -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers)
endif()
add_test(NAME test_a2dp_xq COMMAND test_a2dp_xq WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
