---
description: how to format C++ code for the ADAS pipeline before committing
---
# C++ Code Formatting Workflow

The ADAS project uses `clang-format` with LLVM style (see `.clang-format` in repo root).
**All C++ code in `jetson-core/src/` must pass clang-format checks before pushing.**

> [!IMPORTANT]
> **The CI uses clang-format 18.x (from ubuntu-latest apt package).**
> You MUST use clang-format 18 locally to avoid formatting mismatches.

## Prerequisites

Install clang-format 18 via pip (NOT npm, which provides v15):
```bash
pip install clang-format==18.1.8
```

On Windows, the executable will be in:
```
%LOCALAPPDATA%\Packages\PythonSoftwareFoundation.Python.3.13_qbz5n2kfra8p0\LocalCache\local-packages\Python313\Scripts\clang-format.exe
```

Verify version:
```bash
clang-format --version
# Should show: clang-format version 18.1.8
```

## Before Committing

// turbo-all

1. Navigate to repository root:
```bash
cd ADAS-Capstone
```

2. Run clang-format on all source files:
```bash
clang-format -i jetson-core/src/**/*.cpp jetson-core/src/*.cpp
```

3. Verify formatting passes:
```bash
clang-format --dry-run --Werror jetson-core/src/**/*.cpp jetson-core/src/*.cpp
```

4. If step 3 returns no errors, stage and commit:
```bash
git add jetson-core/
git commit -m "your message"
```

## Key Formatting Rules (from .clang-format)

- **BasedOnStyle**: LLVM
- **IndentWidth**: 4
- **ColumnLimit**: 100
- **AllowShortIfStatementsOnASingleLine**: false
- **SortIncludes**: true

## Common Issues

1. **Version mismatch**: npm's clang-format is v15, which formats differently than CI's v18
2. **Reference/Pointer spacing**: LLVM style uses `const std::string &param` (space before `&`)
3. **Short statements**: `if (x) return;` must be on separate lines
4. **Line breaks**: Long function calls should be broken at specific points

## GitHub CI Check

The CI runs `jetson-core/scripts/format-check.sh` which executes:
```bash
clang-format --dry-run --Werror <files>
```

If this fails, run the formatting commands above locally and push again.
