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

# The git part of the release, from scripts/gitversion.sh: no external tool,
# and guarded so a tree copied into another repository reports nothing rather
# than that repository's commit. It is what makes DW1000_VERSION_FULL say
# 1.1.0+59.ga77b193 in a firmware as well as in a Makefile-driven build.
execute_process(COMMAND sh ${CMAKE_CURRENT_LIST_DIR}/../scripts/gitversion.sh
		OUTPUT_VARIABLE DW1000_VERSION_GIT
		OUTPUT_STRIP_TRAILING_WHITESPACE
		ERROR_QUIET)
if(DW1000_VERSION_GIT)
  zephyr_compile_definitions(DW1000_VERSION_GIT="${DW1000_VERSION_GIT}")
endif()

zephyr_include_directories(${DW1000_INCLUDE_DIR})
zephyr_include_directories(${DW1000_OSAL_ZEPHYR_INCLUDE_DIR})

zephyr_library_sources(${DW1000_SOURCES})
zephyr_library_sources(${DW1000_OSAL_ZEPHYR_SOURCES})

endif()
