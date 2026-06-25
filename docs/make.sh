#!/bin/bash

# Command file for Sphinx documentation

export PATH="/Library/Frameworks/Python.framework/Versions/3.13/bin:$PATH"
cd "$(dirname "$0")"

SPHINXBUILD=${SPHINXBUILD:-sphinx-build}
SOURCEDIR="source"
BUILDDIR="build"

if [ -z "$1" ]; then
    echo "Usage: $0 [target]"
    echo "Available targets are: html, latex, epub, etc."
    exit 1
fi

# Check if sphinx-build is available
if ! command -v "$SPHINXBUILD" &> /dev/null; then
    echo
    echo "The 'sphinx-build' command was not found. Make sure you have Sphinx"
    echo "installed, then set the SPHINXBUILD environment variable to point"
    echo "to the full path of the 'sphinx-build' executable. Alternatively, you"
    echo "may add the Sphinx directory to PATH."
    echo
    echo "If you don't have Sphinx installed, grab it from"
    echo "http://sphinx-doc.org/"
    exit 1
fi

# Run sphinx-build with the provided target
"$SPHINXBUILD" -M "$1" "$SOURCEDIR" "$BUILDDIR" $SPHINXOPTS $O