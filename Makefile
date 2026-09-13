# dw1000 -- compile checks, vendoring and documentation.
#
# Portable between GNU make and BSD make: no conditionals, no pattern
# rules, no GNU-only functions. Port selection uses variable indirection
# ($(OSAL_INC_$(OSAL))), which both makes expand the same way.
#
# Run `make help` for the targets and the variables you can override.
#
# There is no install target and no pkg-config file, deliberately. The
# compile-time options add fields to dw1000_config_t and entry points to
# <dw1000/dw1000_send.h>, so an application must be compiled with exactly
# the option set the driver was; a shared library and a .pc file that
# cannot carry that would be a trap rather than a convenience. The driver
# is vendored into a project instead -- `make sources` prints what to
# vendor, and dw1000.cmake says the same thing to CMake.
#
# What this Makefile is for is the other half: `make check` compiles the
# core over all 256 combinations of the boolean options, against the null
# port, so that an option nobody here selects cannot quietly stop
# compiling. One already had.

NAME       = dw1000
# Keep in step with the release tag and with dw1000.cmake;
# tests/check-cmake.sh checks the two agree.
VERSION    = 1.1.0

CC        ?= cc
AR        ?= ar
DOXYGEN   ?= doxygen
# BSD make's sys.mk predefines CFLAGS, so `CFLAGS ?=` never applies there
# and the warning set would be silently dropped. Keep the flags the
# project requires in their own variable, always applied, and leave
# CFLAGS to the user and to the environment.
CFLAGS    ?= -O2 -g
WARNINGS   = -Wall -Wextra
LDFLAGS   ?=

# --- port selection ---------------------------------------------------
# Selected by indirection rather than conditionals, so that one Makefile
# serves both makes. An unknown name expands to nothing, which portcheck
# catches before the compiler reports a missing <dw1000/osal.h>.
#
# null is the default because it is the only port that needs nothing
# installed: the others want bitters, or a Zephyr tree, or a vendor HAL,
# and cannot be built here without one.
OSAL      ?= null

OSAL_INC_cf2     = port/cf2/dw/osal/include
OSAL_SRC_cf2     = port/cf2/dw/osal/src/osal.c
OSAL_INC_chibios = port/chibios/dw/osal/include
OSAL_SRC_chibios = port/chibios/dw/osal/src/dw_osal.c
OSAL_INC_mynewt  = port/mynewt/dw/osal/include
OSAL_SRC_mynewt  = port/mynewt/dw/osal/src/osal.c
OSAL_INC_null    = port/null/dw/osal/include
OSAL_SRC_null    = port/null/dw/osal/src/osal.c
OSAL_INC_unix    = port/unix/dw/osal/include
OSAL_SRC_unix    = port/unix/dw/osal/src/osal.c
OSAL_INC_zephyr  = port/zephyr/dw/osal/include
OSAL_SRC_zephyr  = port/zephyr/dw/osal/src/osal.c

OSAL_PORTS       = cf2 chibios mynewt null unix zephyr

OSAL_INC         = $(OSAL_INC_$(OSAL))
OSAL_SRC         = $(OSAL_SRC_$(OSAL))

# Opt-in rather than default: a newer compiler inventing a new warning
# should not break an ordinary user's build, but CI should stay clean.
WERROR    ?= no
W_yes      = -Werror
W_no       =

# --- sources ----------------------------------------------------------
# Listed rather than globbed: $(wildcard) is GNU-only, and an explicit
# list is what a vendoring consumer wants from `make sources` anyway.
#
# dw1000.c never calls into dw1000_send.c, so an application that only
# receives can vendor CORE alone. Nothing works without CORE.
COREDIR    = hw/drivers/dw1000
INCDIR     = $(COREDIR)/include
SRC_CORE   = $(COREDIR)/src/dw1000.c
SRC_SEND   = $(COREDIR)/src/dw1000_send.c
SRC        = $(SRC_CORE) $(SRC_SEND)

HEADERS    = $(INCDIR)/dw1000/dw1000.h $(INCDIR)/dw1000/dw1000_send.h \
             $(INCDIR)/dw1000/dw1000_reg.h $(INCDIR)/dw1000/dw1000_otp.h \
             $(INCDIR)/dw1000/dw1000_bswap.h

OBJ        = $(COREDIR)/src/dw1000.o $(COREDIR)/src/dw1000_send.o
OSALOBJ    = $(OSAL_SRC:.c=.o)
STATIC     = lib$(NAME).a

# dw1000.c uses <math.h>; a hosted link wants -lm.
LIBS       = -lm

ALL_CPPFLAGS = -I$(INCDIR) -I$(OSAL_INC) $(CPPFLAGS)
ALL_CFLAGS   = $(CFLAGS) $(WARNINGS) $(W_$(WERROR))

.SUFFIXES:
.SUFFIXES: .c .o

# The compile check is the default: it is the one target that works
# wherever a C compiler does, and it is what this Makefile is for.
all: check					## run the option matrix (default)

# An unknown OSAL leaves OSAL_INC empty, and the core then picks up
# whatever <dw1000/osal.h> is on the system include path, or none. Say so
# here instead. It cannot be a parse-time check, since $(error) is
# GNU-only.
portcheck:
	@if [ -z "$(OSAL_INC)" ]; then \
	    echo "make: unknown OSAL=$(OSAL); one of: $(OSAL_PORTS)" >&2; \
	    exit 1; fi

# --- checks -----------------------------------------------------------

check: check-options check-cmake		## compile the option matrix, and check dw1000.cmake agrees

# 256 compiles, a few seconds each hundred. CC, CFLAGS and WERROR reach
# the script through the environment.
check-options:					## compile the core over every option combination
	@CC='$(CC)' CFLAGS='$(ALL_CFLAGS)' sh tests/check-options.sh

check-cmake:					## check dw1000.cmake and this Makefile still agree
	@sh tests/check-cmake.sh

# --- building ---------------------------------------------------------

# A local archive, not something to install: see the note at the top.
# OSAL=unix wants bitters on the include path and Linux underneath;
# a cross-compiled port wants CC and CPPFLAGS set to match its tree.
lib: portcheck $(STATIC)			## build libdw1000.a against OSAL=<port>

$(STATIC): $(OBJ) $(OSALOBJ)
	$(AR) rcs $@ $(OBJ) $(OSALOBJ)

.c.o:
	$(CC) $(ALL_CFLAGS) $(ALL_CPPFLAGS) -c -o $@ $<

# --- information ------------------------------------------------------

# Bare, so a script can use it:  v=`make -s version`
version:					## print the driver version
	@echo '$(VERSION)'

ports:						## print the OSAL ports this tree ships
	@echo '$(OSAL_PORTS)'

options:					## print the compile-time options and their defaults
	@echo 'Undefined takes the default below, which is not always off:'
	@echo ''
	@printf '  %-42s %s\n' \
	    DW1000_WITH_PROPRIETARY_PREAMBLE_LENGTH  1 \
	    DW1000_WITH_PROPRIETARY_SFD              1 \
	    DW1000_WITH_PROPRIETARY_LONG_FRAME       0 \
	    DW1000_WITH_EXTENDED_SEND                1 \
	    DW1000_WITH_SFD_TIMEOUT                  0 \
	    DW1000_WITH_SFD_TIMEOUT_DEFAULT          0 \
	    DW1000_WITH_HOTFIX_AAT_IEEE802_15_4_2011 1 \
	    DW1000_WITH_DWM1000_EVK_COMPATIBILITY    0
	@echo ''
	@echo 'Three take a value rather than a flag:'
	@echo ''
	@printf '  %-42s %s\n' \
	    DW1000_SFD_TIMEOUT_DEFAULT              'DW1000_SFD_TIMEOUT_MAX' \
	    DW1000_TX_DELAYED_DEFAULT_DELAY         '2 ms, in clock steps' \
	    DW1000_TX_DELAYED_DEFAULT_RETRY_DELAY   'twice the delay'
	@echo ''
	@echo 'They must be defined identically for the driver and for every'
	@echo 'translation unit that includes <dw1000/dw1000.h>.'

# Everything needed to compile the driver straight into another project,
# emitted as shell variables so a build script can consume it:
#
#     eval "$$(make -s -C 3rd/decawave-drivers sources OSAL=unix)"
#     cc $$DW1000_CFLAGS -c $$DW1000_SOURCES
#
# Paths are absolute, so the caller need not know where the tree sits.
# No option flags are emitted: which ones you want is your choice, and
# emitting a default here would put it in two places at once.
sources: portcheck				## print vendoring files and flags as shell variables
	@d=`pwd`; \
	 printf "DW1000_SOURCES_CORE='%s'\n"  "$$d/$(SRC_CORE)"; \
	 printf "DW1000_SOURCES_SEND='%s'\n"  "$$d/$(SRC_SEND)"; \
	 printf "DW1000_SOURCES='%s'\n"       "$$d/$(SRC_CORE) $$d/$(SRC_SEND)"; \
	 printf "DW1000_OSAL='%s'\n"          "$(OSAL)"; \
	 printf "DW1000_OSAL_SOURCES='%s'\n"  "$$d/$(OSAL_SRC)"; \
	 printf "DW1000_INCLUDE='%s'\n"       "$$d/$(INCDIR)"; \
	 printf "DW1000_OSAL_INCLUDE='%s'\n"  "$$d/$(OSAL_INC)"; \
	 printf "DW1000_CFLAGS='%s'\n"        "-I$$d/$(INCDIR) -I$$d/$(OSAL_INC)"; \
	 printf "DW1000_LIBS='%s'\n"          "$(LIBS)"

doc:						## generate the Doxygen documentation
	$(DOXYGEN) Doxyfile

clean:						## remove build products
	rm -f $(STATIC) $(OBJ) \
	      port/cf2/dw/osal/src/osal.o \
	      port/chibios/dw/osal/src/dw_osal.o \
	      port/mynewt/dw/osal/src/osal.o \
	      port/null/dw/osal/src/osal.o \
	      port/unix/dw/osal/src/osal.o \
	      port/zephyr/dw/osal/src/osal.o

distclean: clean				## clean, plus the generated documentation
	rm -rf doc/generated

help:						## show this help
	@echo 'dw1000 -- a driver for the DecaWave DW1000 transceiver'
	@echo ''
	@echo 'Targets:'
	@awk -F':.*## ' '/^[a-z][a-z-]*:.*## /{printf "  %-14s %s\n", $$1, $$2}' \
	    Makefile
	@echo ''
	@echo 'Variables (current value):'
	@printf '  %-14s %s\n' \
	    OSAL     '$(OSAL)  (one of: $(OSAL_PORTS))' \
	    CC       '$(CC)' \
	    CFLAGS   '$(CFLAGS)  (yours; the project always adds $(WARNINGS))' \
	    WERROR   '$(WERROR)  (yes turns warnings into errors)'
	@echo ''
	@echo 'There is no install target: the compile-time options change the'
	@echo 'public headers, so the driver is vendored, not linked against.'
	@echo 'See `make sources`, dw1000.cmake, and the note atop this file.'
	@echo ''
	@echo 'This Makefile works with both GNU make and BSD make.'

.PHONY: all portcheck check check-options check-cmake lib version ports \
	options sources doc clean distclean help
