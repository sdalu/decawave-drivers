# dw1000 -- source lists for CMake consumers that vendor the tree.
#
#   include(${CMAKE_CURRENT_SOURCE_DIR}/3rd/decawave-drivers/dw1000.cmake)
#
# Defines variables, not targets, on purpose. The compile-time options
# below are not internal to the driver: they add fields to dw1000_config_t
# and entry points to <dw1000/dw1000_send.h>, so the application must be
# compiled with exactly the same set as the driver. A target defined here
# would carry one such set and hide that from you; composing the variables
# yourself keeps the choice where it belongs, and lets one project build
# the driver twice with different options if it needs to.
#
#   DW1000_VERSION           the release this tree is (from
#                            <dw1000/dw1000_version.h>, which is where it
#                            is written; DW1000_VERSION_FULL adds the
#                            commit a between-releases build was made from)
#   DW1000_INCLUDE_DIR       the driver API, add to your include path
#   DW1000_SOURCES           the whole core
#   DW1000_SOURCES_CORE      registers, configuration, receive, events
#   DW1000_SOURCES_SEND      dw1000_tx_send() and the vectored variants
#   DW1000_SOURCES_VALIDATE  human radio values (channel, kbps, MHz,
#                            symbols) to dw1000_radio_t fields, with a
#                            message; part of DW1000_SOURCES, and named
#                            separately for a consumer that takes _CORE
#                            alone
#
#   DW1000_OSAL_PORTS        the ports this tree ships
#   DW1000_OSAL_<PORT>_INCLUDE_DIR   for PORT in CF2 CHIBIOS EMULATION MYNEWT
#   DW1000_OSAL_<PORT>_SOURCES       NULL UNIX ZEPHYR
#
#   DW1000_PROBE_INCLUDE_DIR        the probe API, add to your include path
#   DW1000_PROBE_SOURCES            the probe: _CORE (record, role names, the
#                            settle rule: no chip, no port, deliberately
#                            no <dw1000/dw1000.h>) and _EXCHANGE (the
#                            roles, which drive the chip)
#   DW1000_PROBE_PORTS              the probe ports this tree ships
#   DW1000_PROBE_<PORT>_SOURCES     for PORT in EMULATION; a probe port needs no
#                            include directory of its own, since it only
#                            implements probe/include/dw1000/probe/port.h
#
# Pick exactly one OSAL: it is the port contract the core compiles
# against, and its include directory must come before nothing else that
# offers a <dw1000/osal.h>.
#
# SOURCES_SEND is separable -- dw1000.c never calls into it -- so an
# application that only receives can leave it out. SOURCES_CORE is not
# optional; everything else reaches into it.
#
# dw1000.c uses <math.h>, so a hosted build links -lm.
#
# For example, a receive-only Unix application:
#
#   include(3rd/decawave-drivers/dw1000.cmake)
#   add_library(dw1000 INTERFACE)
#   target_include_directories(dw1000 INTERFACE
#       ${DW1000_INCLUDE_DIR} ${DW1000_OSAL_UNIX_INCLUDE_DIR})
#   target_sources(dw1000 INTERFACE
#       ${DW1000_SOURCES_CORE} ${DW1000_OSAL_UNIX_SOURCES})
#   target_compile_definitions(dw1000 INTERFACE
#       DW1000_WITH_PROPRIETARY_LONG_FRAME=1)
#   target_link_libraries(ranger PRIVATE dw1000 bitters m)
#
# The same definitions must reach every translation unit that includes
# <dw1000/dw1000.h>; an INTERFACE library is the shortest way to be sure
# of it, which is why the example uses one.
#
# Under Zephyr you want none of this: zephyr/CMakeLists.txt is a module
# that consumes this file already, and the options are CONFIG_DW1000_* in
# zephyr/Kconfig. Under MyNewt, hw/drivers/dw1000/pkg.yml does the same
# job. This file is for everyone else.

set(DW1000_INCLUDE_DIR
    ${CMAKE_CURRENT_LIST_DIR}/hw/drivers/dw1000/include)

# The release, read from <dw1000/dw1000_version.h> rather than written
# here. That header is the one place it lives, because a C header can read
# no other file and a consumer must have the version without running
# anything -- so the header holds it and everyone else parses it: this
# file, and scripts/manifest.sh for the Makefile. Nothing keeps a second
# copy, so there is no second copy to drift.
#
# DW1000_VERSION is the release, which is all a source list can honestly
# claim to be. A build made between releases says so through
# DW1000_VERSION_FULL, whose git part is passed on the command line
# (`make -s version-full` prints it, and `make sources` hands it over as
# DW1000_VERSION_GIT) -- not from here, where the tree may be a copy
# sitting in your repository rather than a clone of its own.
file(STRINGS ${DW1000_INCLUDE_DIR}/dw1000/dw1000_version.h
     _dw1000_version_lines
     REGEX "^#define[ \t]+DW1000_VERSION_(MAJOR|MINOR|PATCH)[ \t]+[0-9]+")
foreach(_line IN LISTS _dw1000_version_lines)
    string(REGEX MATCH "DW1000_VERSION_([A-Z]+)[ \t]+([0-9]+)" _m "${_line}")
    set(_dw1000_v_${CMAKE_MATCH_1} ${CMAKE_MATCH_2})
endforeach()
if(NOT DEFINED _dw1000_v_MAJOR OR
   NOT DEFINED _dw1000_v_MINOR OR
   NOT DEFINED _dw1000_v_PATCH)
    message(FATAL_ERROR
	"dw1000: no version in ${DW1000_INCLUDE_DIR}/dw1000/dw1000_version.h")
endif()
set(DW1000_VERSION ${_dw1000_v_MAJOR}.${_dw1000_v_MINOR}.${_dw1000_v_PATCH})
unset(_dw1000_version_lines)
unset(_line)
unset(_m)
unset(_dw1000_v_MAJOR)
unset(_dw1000_v_MINOR)
unset(_dw1000_v_PATCH)

set(DW1000_SOURCES_CORE
    ${CMAKE_CURRENT_LIST_DIR}/hw/drivers/dw1000/src/dw1000.c)
set(DW1000_SOURCES_SEND
    ${CMAKE_CURRENT_LIST_DIR}/hw/drivers/dw1000/src/dw1000_send.c)

# Deliberately NOT part of DW1000_SOURCES: reading the radio
# configuration back off the chip is diagnostic, not operational, and it
# is the only file here that formats strings. A consumer that wants it
# names DW1000_SOURCES_STATE itself. See dw1000/dw1000_state.h.
set(DW1000_SOURCES_STATE
    ${CMAKE_CURRENT_LIST_DIR}/hw/drivers/dw1000/src/dw1000_state.c)

# Turning a channel number or a bitrate in kbps into the field
# dw1000_radio_t wants, with a message when it cannot. Unlike _STATE it IS
# part of DW1000_SOURCES below: anything that configures a radio from a
# value it did not write itself needs this, and a caller left to write the
# table by hand gets it wrong -- which is the history in
# dw1000/dw1000_validate.h.
#
# Still named on its own, because DW1000_SOURCES carries _SEND with it and
# a receive-only consumer takes _CORE instead; sniffer/app/unix is exactly
# that, and asks for this by name.
set(DW1000_SOURCES_VALIDATE
    ${CMAKE_CURRENT_LIST_DIR}/hw/drivers/dw1000/src/dw1000_validate.c)

set(DW1000_SOURCES
    ${DW1000_SOURCES_CORE}
    ${DW1000_SOURCES_SEND}
    ${DW1000_SOURCES_VALIDATE})

# dw1000.c uses <math.h>. Nothing to link on a freestanding target, where
# the compiler's own runtime supplies it.
set(DW1000_LIBS m)

set(DW1000_OSAL_PORTS cf2 chibios emulation mynewt null unix zephyr)

set(DW1000_OSAL_CF2_INCLUDE_DIR
    ${CMAKE_CURRENT_LIST_DIR}/port/cf2/dw/osal/include)
set(DW1000_OSAL_CF2_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/port/cf2/dw/osal/src/osal.c)

set(DW1000_OSAL_CHIBIOS_INCLUDE_DIR
    ${CMAKE_CURRENT_LIST_DIR}/port/chibios/dw/osal/include)
set(DW1000_OSAL_CHIBIOS_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/port/chibios/dw/osal/src/dw_osal.c)

set(DW1000_OSAL_EMULATION_INCLUDE_DIR
    ${CMAKE_CURRENT_LIST_DIR}/port/emulation/dw/osal/include)
set(DW1000_OSAL_EMULATION_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/port/emulation/dw/osal/src/osal.c
    ${CMAKE_CURRENT_LIST_DIR}/port/emulation/dw/osal/src/rsvc.c)

set(DW1000_OSAL_MYNEWT_INCLUDE_DIR
    ${CMAKE_CURRENT_LIST_DIR}/port/mynewt/dw/osal/include)
set(DW1000_OSAL_MYNEWT_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/port/mynewt/dw/osal/src/osal.c)

set(DW1000_OSAL_NULL_INCLUDE_DIR
    ${CMAKE_CURRENT_LIST_DIR}/port/null/dw/osal/include)
set(DW1000_OSAL_NULL_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/port/null/dw/osal/src/osal.c)

set(DW1000_OSAL_UNIX_INCLUDE_DIR
    ${CMAKE_CURRENT_LIST_DIR}/port/unix/dw/osal/include)
set(DW1000_OSAL_UNIX_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/port/unix/dw/osal/src/osal.c)

set(DW1000_OSAL_ZEPHYR_INCLUDE_DIR
    ${CMAKE_CURRENT_LIST_DIR}/port/zephyr/dw/osal/include)
set(DW1000_OSAL_ZEPHYR_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/port/zephyr/dw/osal/src/osal.c)

# --- probe --------------------------------------------------------------
# What one exchange produced, and how it is written down: see
# probe/include/dw1000/probe/record.h. record.c, role.c and settle.c are
# free of <dw1000/dw1000.h> by design; exchange.c and solo.c are not
# (they drive the chip), which is why they are a layer of their own
# rather than folded into the others. All of it is still a source list of
# its own rather than part of DW1000_SOURCES, since a consumer that wants
# the probe wants it addressed separately from the driver core.
set(DW1000_PROBE_INCLUDE_DIR
    ${CMAKE_CURRENT_LIST_DIR}/probe/include)
# Two layers, the same shape as DW1000_SOURCES_CORE / _SEND above. CORE is
# the record, the role names and the settle rule, free of
# <dw1000/dw1000.h>; EXCHANGE drives the chip: the two-node exchange and
# the roles a node runs alone (tx, rx, temperature). A consumer wanting
# the format with no driver takes CORE alone.
set(DW1000_PROBE_SOURCES_CORE
    ${CMAKE_CURRENT_LIST_DIR}/probe/src/role.c
    ${CMAKE_CURRENT_LIST_DIR}/probe/src/record.c
    ${CMAKE_CURRENT_LIST_DIR}/probe/src/settle.c)
set(DW1000_PROBE_SOURCES_EXCHANGE
    ${CMAKE_CURRENT_LIST_DIR}/probe/src/exchange.c
    ${CMAKE_CURRENT_LIST_DIR}/probe/src/solo.c)

set(DW1000_PROBE_SOURCES
    ${DW1000_PROBE_SOURCES_CORE}
    ${DW1000_PROBE_SOURCES_EXCHANGE})

set(DW1000_PROBE_PORTS emulation unix zephyr)

set(DW1000_PROBE_EMULATION_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/probe/port/emulation/src/port.c)

set(DW1000_PROBE_UNIX_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/probe/port/unix/src/port.c)

set(DW1000_PROBE_ZEPHYR_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/probe/port/zephyr/src/port.c)
