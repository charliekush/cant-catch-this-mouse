#!/usr/bin/env bash
set -euo pipefail

bad_files="$(
    grep -RInE '^\s*//\s*[-=*_/]{8,}|^\s*/\*\s*[-=*_/]{8,}|[-=*_/]{12,}|-' \
        --include='*.h' \
        --include='*.hpp' \
        --include='*.c' \
        --include='*.cpp' \
        . \
        | grep -v '.git' || true
)"

if [[ -n "$bad_files" ]]; then
    echo "Found AI-looking separator comments:"
    echo "$bad_files"
    exit 1
fi