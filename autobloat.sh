#!/usr/bin/env bash
set -euo pipefail

echo "==> autobloat: running full autotools build sequence"

# Ensure we are in a project with configure.ac
if [ ! -f "configure.ac" ]; then
    echo "Error: No configure.ac found in this directory."
    echo "Run this script from the project root."
    exit 1
fi

echo "==> Running autoreconf -fi"
autoreconf -fi

if [ ! -f "./configure" ]; then
    echo "Error: configure script was not generated."
    exit 1
fi

echo "==> Running ./configure $*"
./configure "$@"

echo "==> Running make"
make

echo "==> Build complete."
