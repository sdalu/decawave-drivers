# CMake
#
# Copyright (c) 2019 
# Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
#
# SPDX-License-Identifier: Apache-2.0
#

if (CONFIG_DW1000)

zephyr_include_directories(../hw/drivers/dw1000/include)
zephyr_include_directories(../port/zephyr/dw/osal/include)

zephyr_library_sources(../hw/drivers/dw1000/src/dw1000.c)
zephyr_library_sources(../port/zephyr/dw/osal/src/osal.c)

endif()
