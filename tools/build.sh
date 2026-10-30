#!/bin/sh
# Builds the firmware in a Podman container (see tools/Containerfile.avr).
# Usage, from anywhere:  tools/build.sh [make targets...]   e.g. tools/build.sh clean all
set -e
repo=$(cd "$(dirname "$0")/.." && pwd)
image=adapt-ffb-joy-avr

# Git Bash rewrites /wt into a Windows path and needs the host path in Windows form
case "$(uname -s)" in
    MINGW*|MSYS*) repo=$(cd "$repo" && pwd -W); export MSYS_NO_PATHCONV=1 ;;
esac

if ! podman image exists "$image"; then
    podman build -t "$image" -f "$repo/tools/Containerfile.avr" "$repo"
fi

# One make per target: the makefile creates .dep/ while parsing, so "make clean all" in one call fails
[ $# -eq 0 ] && set -- all
for target in "$@"; do
    podman run --rm -v "$repo:/wt" "$image" make "$target"
done
