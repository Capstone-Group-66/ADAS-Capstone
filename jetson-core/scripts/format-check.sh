#!/usr/bin/env bash
set -e

# Find all C/C++ sources under jetson-core/src.
FILES=$(find jetson-core/src -type f \( -name '*.cpp' -o -name '*.cc' -o -name '*.c' -o -name '*.h' -o -name '*.hpp' \) 2>/dev/null || true)

if [ -z "$FILES" ]; then
  echo "No C/C++ sources under jetson-core/src; skipping clang-format check."
  exit 0
fi

echo "Running clang-format check on:"
echo "$FILES"

clang-format --dry-run --Werror $FILES