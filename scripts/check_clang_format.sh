#!/usr/bin/env bash
# 校验 C++ 源码是否符合仓库 .clang-format 规范（CI / 本地均可运行）。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

mapfile -t files < <(find src -type f \( -name '*.cpp' -o -name '*.hpp' \) | sort)
if [[ ${#files[@]} -eq 0 ]]; then
    echo "clang-format: no source files found"
    exit 1
fi

for f in "${files[@]}"; do
    clang-format --dry-run --Werror "$f"
done

echo "clang-format: ${#files[@]} files OK"
