#!/bin/sh
# checktag.sh — does the release the version header names agree with the tags?
# usage: checktag.sh <release>          (e.g. checktag.sh 1.2.3)
set -eu

# The header is the one place the release is written, and git tags are the
# other half of the same statement -- and nothing in the tree makes the two
# agree, so the way they come apart is ordinary: tag v1.2.0, forget to bump
# the header, and the release build reports 1.1.1. scripts/gitversion.sh
# cannot notice, because being *on* a tag with a clean worktree is exactly
# what it takes to be a release.
#
# Two rules, and the difference between them is what a release is:
#
#   on a tag, worktree clean   the header must say what the tag says.
#                              This is a release build: there is no later
#                              chance to be wrong about which one.
#
#   anything else              the header must be at or ahead of the
#                              nearest tag. Ahead is the normal state
#                              between releases -- the numbers are bumped
#                              first and tagged later -- and a dirty
#                              worktree on a tag is that bump in progress.
#                              Behind means a release was tagged that the
#                              header never learned about.
#
# Prints nothing and succeeds when there is nothing to check: no git, a
# tarball, a tree vendored inside another project's repository, or a clone
# with no v* tag in reach. Like gitversion.sh, an answer that would be
# somebody else's is not an answer.
#
# Project-agnostic -- it names no header, no macro and no repository -- so
# the same copy serves every tree that versions itself this way. The caller
# passes the release it read from the header.
#
# Exit status: 0 they agree, or there is nothing to check; 1 they disagree,
# with the reason on stdout; 2 usage.
#
# POSIX sh and awk only.

want=${1:-}
if [ -z "$want" ]; then
    printf 'usage: checktag.sh <release>\n' >&2
    exit 2
fi

# CDPATH is emptied so that cd cannot land somewhere else entirely, and
# given a value rather than left bare, which reads as a typo.
top=$(CDPATH='' cd -- "$(dirname -- "$0")/.." && pwd)

command -v git >/dev/null 2>&1 || exit 0

gittop=$(git -C "$top" rev-parse --show-toplevel 2>/dev/null) || exit 0
if [ "$gittop" != "$top" ]; then
    exit 0
fi
git -C "$top" rev-parse --verify -q HEAD >/dev/null 2>&1 || exit 0

# --long so that the commit count is always there to read, .dirty so that
# an in-progress bump can be told from a release.
d=$(git -C "$top" describe --tags --long --dirty=.dirty \
    --match 'v[0-9]*' 2>/dev/null) || d=''
if [ -z "$d" ]; then
    exit 0
fi

dirty=''
case $d in
    *.dirty)
        dirty=yes
        d=${d%.dirty}
        ;;
esac

# v<tag>-<count>-g<hash>: the tag is what is left once the last two fields
# come off, with the v taken off the front.
rest=${d%-*}
count=${rest##*-}
tag=${rest%-*}
tag=${tag#v}

# lt, eq or gt, comparing field by field and numerically. In awk because
# the arithmetic is awk's to do: a tag is written by hand and need not have
# three fields (v0.9 is a tag in one of these trees), a missing field counts
# as 0, and +0 reads 09 as nine rather than as the bad octal constant
# $(( )) would make of it.
cmp_versions() {
    awk -v a="$1" -v b="$2" 'BEGIN {
        na = split(a, x, ".")
        nb = split(b, y, ".")
        for (i = 1; i <= 3; i++) {
            u = (i <= na ? x[i] + 0 : 0)
            v = (i <= nb ? y[i] + 0 : 0)
            if (u < v) { print "lt"; exit }
            if (u > v) { print "gt"; exit }
        }
        print "eq"
    }'
}

rel=$(cmp_versions "$want" "$tag")

if [ "$count" = 0 ] && [ -z "$dirty" ]; then
    # A release build. Nothing later can correct it.
    if [ "$rel" != eq ]; then
        printf '  release: HEAD is tagged v%s but the header says %s\n' \
            "$tag" "$want"
        printf '           (a build here would call itself %s)\n' "$want"
        exit 1
    fi
elif [ "$rel" = lt ]; then
    printf '  release: the header says %s but v%s is already tagged\n' \
        "$want" "$tag"
    printf '           (bump the header, or the next release build is %s)\n' \
        "$want"
    exit 1
fi

exit 0
