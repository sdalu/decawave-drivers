# A dissector tree for the UWB sniffer, and the reference for the
# interface a dissector tree must present.
#
# Two things are asked of such a tree, and nothing else:
#
#   `make -s sources`   prints shell variable assignments, with absolute
#                       paths, the way this repository's own Makefile and
#                       bitters' do. sniffer/app/unix/build.sh evals the
#                       result when DISSECTORS= names this directory.
#   `make plugin`       builds the same sources as a shared object, for
#                       the --dissector= route.
#
# What `sources` must set:
#
#   DISSECT_SOURCES   the .c files, absolute                  (required)
#   DISSECT_INIT      the registration function's NAME        (required)
#   DISSECT_CFLAGS    flags its sources need                  (optional)
#   DISSECT_LIBS      libraries it needs                      (optional)
#   DISSECT_VERSION   reported in the build line              (optional)
#
# DISSECT_INIT is a name rather than a call, so that build.sh can pass it
# through as -DSNIFFER_DISSECT_INIT= and main.c can declare it extern
# without either of them knowing a protocol.
#
# A real dissector tree lives in the repository that owns the protocol it
# reads, so that it includes the layout's definition rather than keeping a
# second copy of it. This one is in the sniffer's tree because 802.15.4 is
# the shape of every frame the sniffer sees and it needs no definition
# from anywhere else.
#
# Portable between GNU make and BSD make: no conditionals, no pattern
# rules, no GNU-only functions.

NAME     = ieee802154
# Both files for the compile-time route; only the dissector for the
# plugin, because register.c calls into the sniffer and a plugin that
# does will not load. register.c explains it at length.
SOURCES  = dissect_ieee802154.c register.c
PLUGIN_SOURCES = dissect_ieee802154.c
INIT     = dissect_ieee802154_register
VERSION  = 1

CC      ?= cc
WARNINGS = -Wall -Wextra

# Where <dissect.h> is. Two directories up and into the application, for
# a tree that sits here; a tree of your own sets this to wherever it
# vendored or found the sniffer's headers. Only `make plugin` needs it:
# under DISSECTORS= the sources are compiled by build.sh, which is
# already passing -I for the application's own directory.
SNIFFER_INCLUDE ?= ../../app/unix

# -fPIC and -shared for the plugin; -fvisibility=hidden so that the only
# symbol a loaded dissector exports is the one dissect.c asks for by
# name, which is what keeps two plugins from colliding over a helper they
# both happen to call the same thing.
PLUGIN   = $(NAME).so

all: help					## the default: say what to type

help:
	@echo 'A dissector tree for the UWB sniffer.'
	@echo ''
	@echo '  make sources    print the files and flags, for DISSECTORS='
	@echo '  make plugin     build $(PLUGIN), for --dissector='
	@echo '  make clean      remove $(PLUGIN)'
	@echo ''
	@echo 'Compile it into the sniffer:'
	@echo '  DISSECTORS=`pwd` sh ../../app/unix/build.sh'
	@echo ''
	@echo 'Or load it at run time:'
	@echo '  make plugin && uwb-sniffer --dissector=`pwd`/$(PLUGIN) ...'

# Absolute paths, computed in the recipe rather than with a make function,
# so that both makes produce the same answer. Same approach
# scripts/manifest.sh takes for this repository.
sources:					## print the manifest for DISSECTORS=
	@d=`pwd`;							\
	printf "DISSECT_SOURCES='";					\
	for f in $(SOURCES); do printf '%s ' "$$d/$$f"; done;		\
	printf "'\n";							\
	echo "DISSECT_CFLAGS=''";					\
	echo "DISSECT_LIBS=''";						\
	echo "DISSECT_INIT='$(INIT)'";					\
	echo "DISSECT_VERSION='$(VERSION)'"

plugin: $(PLUGIN)				## build the shared object

$(PLUGIN): $(PLUGIN_SOURCES)
	$(CC) -shared -fPIC -fvisibility=hidden $(WARNINGS) $(CFLAGS)	\
	    -I$(SNIFFER_INCLUDE) -o $(PLUGIN) $(PLUGIN_SOURCES)

clean:						## remove the shared object
	rm -f $(PLUGIN)

.PHONY: all help sources plugin clean
