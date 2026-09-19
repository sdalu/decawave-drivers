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

# The release, from <dw1000/dw1000_version.h>, which is the one place it
# is written -- bump it there and tag v<VERSION>. `make version` prints it.
VERSION   != $(MANIFEST) version

# What a build between releases adds to it: +58.g3403fe0[.dirty], and
# nothing for a release, for a tarball, or for a tree copied into another
# project's repository (whose git state is not the driver's). It is
# compiled in, where DW1000_VERSION_FULL reports it; `make version-full`
# prints what this tree builds as, and `make sources` hands a vendoring
# build the same answer to pass on if it wants to.
GITVER    != sh scripts/gitversion.sh

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
VERSIONHDR = $(INCDIR)/dw1000/dw1000_version.h
SRC_CORE  != $(MANIFEST) core
SRC_SEND  != $(MANIFEST) send
# Part of the driver's sources, not an extra: see dw1000.cmake's note.
SRC_VALID != $(MANIFEST) validate
SRC        = $(SRC_CORE) $(SRC_SEND) $(SRC_VALID)

# What a hosted link needs; dw1000.c uses <math.h>.
LIBS      != $(MANIFEST) libs

# Which port was built is recorded nowhere, so clean removes every port's
# object rather than guessing.
ALLOSALOBJ != $(MANIFEST) objs

OBJ        = $(SRC_CORE:.c=.o) $(SRC_SEND:.c=.o) $(SRC_VALID:.c=.o)
OSALOBJ    = $(OSAL_SRC:.c=.o)
STATIC     = lib$(NAME).a

# DW1000_VERSION_GIT is the one thing here that no file holds; the version
# header defaults it to "" for everyone who compiles these sources without
# it, which is every vendoring build that does not ask for it.
ALL_CPPFLAGS = -I$(INCDIR) -I$(OSAL_INC) \
               -DDW1000_VERSION_GIT='"$(GITVER)"' $(CPPFLAGS)
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
	@awk -F':.*## ' '/^[a-z][a-z-]*:.*## /{ \
	    pre = sprintf("  %-16s ", $$1); n = split($$2, w, / /); line = ""; \
	    for (i = 1; i <= n; i++) { \
	        cand = (line == "" ? w[i] : line " " w[i]); \
	        if (length(pre) + length(cand) > 80 && line != "") { \
	            print pre line; pre = "                   "; line = w[i]; \
	        } else line = cand; \
	    } \
	    if (line != "") print pre line; \
	}' Makefile
	@echo ''
	@echo 'Variables (current value):'
	@printf '  %-16s %s\n' \
	    OSAL     '$(OSAL)  (one of: $(OSAL_PORTS))' \
	    CC       '$(CC)' \
	    CFLAGS   '$(CFLAGS)  (yours; the project always adds $(WARNINGS))' \
	    WERROR   '$(WERROR)  (yes turns warnings into errors)' \
	    YES      '$(YES)  (1 answers yes to `make tag`)'
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

check: check-options check-manifest check-validate check-emulation check-probe check-sniffer	## compile the option matrix (~9 min), check the manifest, check the radio-value validation, run the emulation smoke test, run the probe tests, run the sniffer tests

# The matrix is 2^n over the options, so eleven of them is 2048 compiles
# and about nine minutes; the script reports progress as it goes, and a
# twelfth option would double both. CC, CFLAGS and WERROR reach it
# through the environment.
check-options:					## compile the core over every option combination
	@CC='$(CC)' CFLAGS='$(ALL_CFLAGS)' sh tests/check-options.sh

check-manifest:					## check dw1000.cmake still describes the tree
	@sh tests/check-manifest.sh

# The cheapest check here: <dw1000/dw1000_validate.h> is a value mapping
# that links on its own, so this needs no chip, no port and no driver --
# two compiles and two runs. Twice because five of the eight preamble
# lengths are proprietary and what the API may accept depends on the
# build's options, which is the half that used to be wrong in the
# hand-written copy this replaced.
check-validate:					## check the radio-value validation against what dw1000_configure() accepts
	@CC='$(CC)' CFLAGS='$(ALL_CFLAGS)' sh tests/check-validate.sh

# The emulation port has no vendor tree to wait for -- unlike unix,
# chibios, mynewt, cf2 and zephyr, it needs nothing installed -- and
# tests/emulation/smoke.c brings its own medium, a thread in the test's
# own process. So this is the one port the tree can *run* the driver
# against rather than only compile, which subsumes the syntax check this
# target used to be and catches what no syntax check would: the frame on
# the wire, the timestamps, and the four callbacks. The script reads the
# paths from the manifest, as everything here does (tests/check-manifest.sh
# fails the Makefile for spelling a port path out itself).
check-emulation:				## run the emulation smoke test (driver, port/emulation, stub medium)
	@CC='$(CC)' CFLAGS='$(ALL_CFLAGS)' sh tests/check-emulation.sh

# probe/include/dw1000/probe/record.h and role.h are free of <dw1000/dw1000.h>
# by design, so this is the one probe check that needs no chip, no
# driver and no radio either -- record, role, port/emulation and the
# format test, the same reason check-emulation exists for the driver.
# The exchange test goes further and runs the responder against
# port/emulation, which is what pins the two things a responder owes a
# link that is not working: that a run ends, and that it says what it
# heard. The script reads the paths from the manifest, as everything
# here does.
check-probe:					## run the probe tests (format, and the responder against port/emulation)
	@CC='$(CC)' CFLAGS='$(ALL_CFLAGS)' sh tests/check-probe.sh

# Three of the sniffer's seven translation units were written to touch
# neither the chip nor Linux, so they run here; the other four are a
# Linux program for a Raspberry Pi and only build there. The script says
# which is which and why. capture.c reaching the chip through a pair of
# function pointers rather than calling into uwb_dw1000.c is what made
# any of this testable: before that the sniffer had no tests at all.
check-sniffer:					## run the sniffer tests (the frame ring, the wire header, the pcapng writer, the dissector registry and loader, and the wireshark dissector: its offsets always, and its behaviour where a Lua is installed)
	@CC='$(CC)' CFLAGS='$(ALL_CFLAGS)' sh tests/check-sniffer.sh

# --- building ---------------------------------------------------------

# A local archive, not something to install: see the note at the top.
# OSAL=unix wants bitters on the include path and Linux underneath;
# a cross-compiled port wants CC and CPPFLAGS set to match its tree.
lib: portcheck $(STATIC)			## build libdw1000.a against OSAL=<port>

$(STATIC): $(OBJ) $(OSALOBJ)
	$(AR) rcs $@ $(OBJ) $(OSALOBJ)

# The git part is in no file, so nothing would make an object stale when
# HEAD moves, and the archive would keep reporting the commit it was first
# built at. This stamp remembers the answer instead. The rule runs every
# time -- FORCE is a target that never exists, which is how both makes are
# told to -- and does nothing at all unless the answer changed, so an
# unmoved HEAD costs a `cat`.
#
# When it did change, the objects are removed rather than left to a
# timestamp comparison: BSD make compares against the mtime it read before
# this rule ran, so it would notice one `make` late. Deleting what was
# compiled with the old answer says the same thing in a way both makes act
# on at once. Every port's object goes, for the reason ALLOSALOBJ exists:
# which port was built is recorded nowhere.
FORCE:

.gitversion: FORCE
	@if [ "`cat $@ 2>/dev/null`" != '$(GITVER)' ]; then \
	    echo '$(GITVER)' > $@; rm -f $(OBJ) $(ALLOSALOBJ); fi

# ... and on the header the release is written in, so that bumping it
# recompiles what carries it. The only header dependency here: the rest of
# the API does not change what an object *says about itself*.
$(OBJ) $(ALLOSALOBJ): .gitversion $(VERSIONHDR)

.c.o:
	$(CC) $(ALL_CFLAGS) $(ALL_CPPFLAGS) -c -o $@ $<

# --- information ------------------------------------------------------

# Bare, so a script can use it:  v=`make -s version`
version:					## print the release version
	@echo '$(VERSION)'

# The same, plus what a build from this tree adds to it -- which is what
# DW1000_VERSION_FULL reports in a build made here. Equal to
# `make version` exactly when this is a release tree.
version-full:					## print the version this tree builds as
	@echo '$(VERSION)$(GITVER)'

# Tag the release from the version header, so that the tag and the header
# cannot say different things: the number is not typed here, it is read
# from $(VERSIONHDR). tests/check-manifest.sh checks the other direction,
# for a tag made by hand -- scripts/checktag.sh, run below so that a tag
# this target just made is confirmed rather than assumed.
#
# Refuses on an unclean worktree: a release tag names committed work, and
# the version a build reports would otherwise include `.dirty`. Nothing is
# pushed; that stays yours.
#
# In that order on purpose: the two refusals are instant, so they come
# before the question -- there is no point asking about a tag that cannot be
# made -- and the full check comes after it, because a minute of tests is
# not worth spending on a `make tag` the answer to which is no. Tagging
# something the suite has not passed is the mistake this exists to prevent,
# so the check is inside the recipe rather than a prerequisite, which would
# have run before the prompt.
#
# `make tag YES=1` answers yes for a script, and a non-interactive run with
# no YES=1 reads EOF and declines -- the safe way round.
tag: $(VERSIONHDR)				## tag this release, from the version header
	@if [ -n "`git status --porcelain --untracked-files=no`" ]; then \
	    echo 'make: uncommitted changes; commit them before tagging' >&2; \
	    exit 1; fi
	@if git rev-parse -q --verify 'refs/tags/v$(VERSION)' >/dev/null; then \
	    echo 'make: v$(VERSION) exists already; bump $(VERSIONHDR) first' >&2; \
	    exit 1; fi
	@if [ "$(YES)" != 1 ]; then \
	    printf 'tag v%s at %s? (the full check runs first) [y/N] ' \
		'$(VERSION)' "`git rev-parse --short HEAD`"; \
	    read -r ans || ans=; \
	    case "$$ans" in \
		y|Y|yes|YES) ;; \
		*) echo 'make: not tagged'; exit 1 ;; \
	    esac; \
	fi
	@$(MAKE) check
	git tag -a -m '$(NAME) $(VERSION)' 'v$(VERSION)'
	@sh scripts/checktag.sh '$(VERSION)'
	@echo 'tagged v$(VERSION) -- push it with: git push origin v$(VERSION)'

ports:						## print the OSAL ports this tree ships
	@echo '$(OSAL_PORTS)'

state:						## print the optional radio-state source
	@$(MANIFEST) state

validate:					## print the optional radio-value validation source
	@$(MANIFEST) validate

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
	    DW1000_WITH_EVENT_COUNTERS               0 \
	    DW1000_WITH_ACCUMULATOR                  0 \
	    DW1000_WITH_TEMP_COMPENSATION            0 \
	    DW1000_WITH_DEBUG                        0
	@echo ''
	@echo 'One takes a value rather than a flag:'
	@echo ''
	@printf '  %-42s %s\n' \
	    DW1000_SFD_TIMEOUT_DEFAULT              'DW1000_SFD_TIMEOUT_MAX'
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

# PROJECT_NUMBER is appended rather than written in the Doxyfile, so the
# documentation says which version it documents without that being a
# second place the version is kept.
doc:						## generate the Doxygen documentation
	{ cat Doxyfile; echo 'PROJECT_NUMBER = $(VERSION)$(GITVER)'; } \
	    | $(DOXYGEN) -

clean:						## remove build products
	rm -f $(STATIC) $(OBJ) $(ALLOSALOBJ) .gitversion

distclean: clean				## clean, plus the generated documentation
	rm -rf doc/generated

.PHONY: help portcheck check check-options check-manifest check-emulation \
	check-probe check-sniffer check-validate \
	lib version version-full tag ports options sources doc clean distclean \
	state validate
