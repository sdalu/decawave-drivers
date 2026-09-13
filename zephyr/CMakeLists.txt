# CMake
#
# Copyright (c) 2019-2026
# Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
#
# SPDX-License-Identifier: Apache-2.0
#

if (CONFIG_DW1000)

# The file list lives in dw1000.cmake, which every other CMake consumer
# reads too, so adding a source to the driver does not mean remembering
# to add it here as well. tests/check-manifest.sh fails if this file
# starts naming them itself again.
include(${CMAKE_CURRENT_LIST_DIR}/../dw1000.cmake)

zephyr_include_directories(${DW1000_INCLUDE_DIR})
zephyr_include_directories(${DW1000_OSAL_ZEPHYR_INCLUDE_DIR})

zephyr_library_sources(${DW1000_SOURCES})
zephyr_library_sources(${DW1000_OSAL_ZEPHYR_SOURCES})

endif()
