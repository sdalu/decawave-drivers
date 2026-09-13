#!/bin/sh
# gitversion.sh — what a build between releases adds to the version
# usage: gitversion.sh        (prints e.g. +3.gae9c67b, or nothing)
set -eu

# The release itself is fixed: it is written in the version header, which
# is the one place it lives, and read from there by the manifest. This
# adds the part that is in no file -- which commit the build was made
# from -- as SemVer build metadata:
#
#   (nothing)            the release tag, worktree clean: a release build
#   +3.gae9c67b          three commits past the tag, at that commit
#   +3.gae9c67b.dirty    and with uncommitted changes
#
# So `1.2.3` is a release and `1.2.3+3.gae9c67b` is not, which is the
# whole point: a version with nothing appended is one you can go and
# fetch.
#
# Nothing is printed, and the exit status is still 0, when the answer
# cannot be known or would be somebody else's answer:
#
#   * no git, or a tarball with no repository in it
#   * a tree vendored inside another project's repository -- git would
#     happily describe *that* repository, whose tags and dirt have
#     nothing to do with this one. Guarded by comparing git's toplevel
#     against this tree.
#   * a clone with no v* tag in reach (a shallow one, say): the commit is
#     known but its distance from a release is not, so it is reported
#     without the count, +gae9c67b.
#
# An empty answer is never wrong, only less precise: it says "this is the
# release these files say it is", which is what a tarball is.
#
# This script is deliberately project-agnostic -- it names no header, no
# macro and no repository -- so the same copy serves every tree that
# versions itself this way.
#
# POSIX sh, and git used only through plumbing that works in any version.

# CDPATH is emptied so that cd cannot land somewhere else entirely, and
# given a value rather than left bare, which reads as a typo.
top=$(CDPATH='' cd -- "$(dirname -- "$0")/.." && pwd)

command -v git >/dev/null 2>&1 || exit 0

# This tree's own repository, and not one it happens to be sitting
# inside. --show-toplevel resolves symlinks the same way `pwd` above does,
# so the two are comparable.
gittop=$(git -C "$top" rev-parse --show-toplevel 2>/dev/null) || exit 0
if [ "$gittop" != "$top" ]; then
    exit 0
fi

# A worktree with no commit yet has no HEAD to describe.
git -C "$top" rev-parse --verify -q HEAD >/dev/null 2>&1 || exit 0

# --long keeps the shape uniform (v1.2.3-0-gae9c67b even sitting on the
# tag), so one parse does for every case; --dirty must come with it and
# is spelled .dirty because that is how it is wanted in the output.
# An empty answer is not an early exit: a clone with no v* tag in reach
# still knows its commit, and the case below reports that without a count.
d=$(git -C "$top" describe --tags --long --dirty=.dirty \
    --match 'v[0-9]*' 2>/dev/null) || d=''

# Taken off first, so what is left is the fixed three-field shape and the
# fields can be cut off it without the suffix riding along.
dirty=''
case $d in
    *.dirty)
        dirty=.dirty
        d=${d%.dirty}
        ;;
esac

case $d in
    # v<tag>-<count>-g<hash>
    *-*-g*)
        hash=${d##*-}
        rest=${d%-*}
        count=${rest##*-}

        # On the tag with nothing uncommitted: the release, and the plain
        # release number is the honest answer.
        if [ "$count" = 0 ] && [ -z "$dirty" ]; then
            exit 0
        fi

        printf '+%s.%s%s\n' "$count" "$hash" "$dirty"
        ;;

    # No v* tag in reach: the commit, without a distance it cannot know.
    *)
        hash=$(git -C "$top" rev-parse --short HEAD 2>/dev/null) || exit 0
        if ! git -C "$top" diff --quiet HEAD 2>/dev/null; then
            dirty=.dirty
        fi
        printf '+g%s%s\n' "$hash" "$dirty"
        ;;
esac
