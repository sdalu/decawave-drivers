# dw1000 -- compile checks, vendoring and documentation.
#
# Portable between GNU make and BSD make: no conditionals, no pattern
# rules, no GNU-only functions. Port selection uses variable indirection
# ($(OSAL_INC_$(OSAL))), which both makes expand the same way.
#
# Run `make help` for the targets and the variables you can override.
#
# The file list is not here. dw1000.cmake is the manifest -- CMake
# consumers include it, and this Makefile reads it through
# scripts/manifest.sh, with `!=` shell assignments (GNU make 4.0 and any
# BSD make). One place to add a source or a port, and nothing to drift.
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
MANIFEST   = sh scripts/manifest.sh

# Keep in step with the release tag; dw1000.cmake is where it is written.
VERSION   != $(MANIFEST) version

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
# null is the default because it is the only port that needs nothing
# installed: the others want bitters, or a Zephyr tree, or a vendor HAL,
# and cannot be built here without one.
#
# OSAL is read at parse time, so `make lib OSAL=unix` reaches the two
# queries below: a command-line assignment is in force before either make
# expands them. An unknown name gives an empty answer rather than an
# error, which portcheck turns into the message that names the ports
# there are.
OSAL      ?= null

OSAL_PORTS != $(MANIFEST) ports
OSAL_INC   != $(MANIFEST) inc $(OSAL)
OSAL_SRC   != $(MANIFEST) src $(OSAL)

# Opt-in rather than default: a newer compiler inventing a new warning
# should not break an ordinary user's build, but CI should stay clean.
WERROR    ?= no
W_yes      = -Werror
W_no       =

# --- sources ----------------------------------------------------------
# dw1000.c never calls into dw1000_send.c, so an application that only
# receives can vendor SRC_CORE alone. Nothing works without SRC_CORE.
INCDIR    != $(MANIFEST) incdir
SRC_CORE  != $(MANIFEST) core
SRC_SEND  != $(MANIFEST) send
SRC        = $(SRC_CORE) $(SRC_SEND)

# What a hosted link needs; dw1000.c uses <math.h>.
LIBS      != $(MANIFEST) libs

# Which port was built is recorded nowhere, so clean removes every port's
# object rather than guessing.
ALLOSALOBJ != $(MANIFEST) objs

OBJ        = $(SRC_CORE:.c=.o) $(SRC_SEND:.c=.o)
OSALOBJ    = $(OSAL_SRC:.c=.o)
STATIC     = lib$(NAME).a

ALL_CPPFLAGS = -I$(INCDIR) -I$(OSAL_INC) $(CPPFLAGS)
ALL_CFLAGS   = $(CFLAGS) $(WARNINGS) $(W_$(WERROR))

.SUFFIXES:
.SUFFIXES: .c .o

# First target, so a bare `make` runs it: nothing here is worth building
# by default -- there is no artifact a consumer wants out of this tree,
# and `make check` takes the better part of a minute, which a bare `make`
# should not spend. Printing the targets says what to type instead.
# .DEFAULT_GOAL and .MAIN would each say so in one make only; being first
# says it in both.
help:						## show this help (the default)
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
	@echo 'The file list lives in dw1000.cmake; this Makefile reads it.'
	@echo ''
	@echo 'This Makefile works with both GNU make and BSD make.'

# An unknown OSAL leaves OSAL_INC empty, and the core then picks up
# whatever <dw1000/osal.h> is on the system include path, or none. Say so
# here instead. It cannot be a parse-time check, since $(error) is
# GNU-only.
portcheck:
	@if [ -z "$(OSAL_INC)" ]; then \
	    echo "make: unknown OSAL=$(OSAL); one of: $(OSAL_PORTS)" >&2; \
	    exit 1; fi

# --- checks -----------------------------------------------------------

check: check-options check-manifest		## compile the option matrix (~1 min), check the manifest

# 256 compiles, tens of seconds in total, so the script reports progress
# as it goes. CC, CFLAGS and WERROR reach it through the environment.
check-options:					## compile the core over every option combination
	@CC='$(CC)' CFLAGS='$(ALL_CFLAGS)' sh tests/check-options.sh

check-manifest:					## check dw1000.cmake still describes the tree
	@sh tests/check-manifest.sh

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
# The answer comes from dw1000.cmake, the same file a CMake consumer
# includes, so the two cannot disagree about what to vendor.
sources: portcheck				## print vendoring files and flags as shell variables
	@$(MANIFEST) vars $(OSAL) "`pwd`"

doc:						## generate the Doxygen documentation
	$(DOXYGEN) Doxyfile

clean:						## remove build products
	rm -f $(STATIC) $(OBJ) $(ALLOSALOBJ)

distclean: clean				## clean, plus the generated documentation
	rm -rf doc/generated

.PHONY: help portcheck check check-options check-manifest lib version ports \
	options sources doc clean distclean
