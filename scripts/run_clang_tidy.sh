#!/usr/bin/env bash
# 对核心库与 ROS 驱动包运行 clang-tidy（需先 colcon build 并导出 compile_commands.json）。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

run_tidy_for_package() {
    local pkg="$1"
    local build_dir="build/${pkg}"
    shift

    if [[ ! -f "${build_dir}/compile_commands.json" ]]; then
        echo "clang-tidy: missing ${build_dir}/compile_commands.json (run colcon build first)" >&2
        return 1
    fi

    local failures=0
    while IFS= read -r -d '' file; do
        local output
        if ! output="$(clang-tidy -p "${build_dir}" "${file}" --quiet 2>&1)"; then
            echo "${output}"
            failures=$((failures + 1))
            continue
        fi
        if echo "${output}" | grep -qE 'warning:|error:'; then
            echo "=== ${file} ==="
            echo "${output}"
            failures=$((failures + 1))
        fi
    done < <(find "$@" -type f -name '*.cpp' -print0)

    if [[ ${failures} -gt 0 ]]; then
        echo "clang-tidy: ${failures} file(s) failed in ${pkg}" >&2
        return 1
    fi

    echo "clang-tidy: ${pkg} OK"
}

run_tidy_for_package inspire_serial_core src/inspire_serial_core/src src/inspire_serial_core/tests
run_tidy_for_package inspire_control_ros2 src/driver/src

echo "clang-tidy: all packages OK"
