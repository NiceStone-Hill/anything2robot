#!/usr/bin/env bash
set -euo pipefail
postprocess_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(dirname -- "$postprocess_dir")"
python_executable="${LAYOUT_PYTHON:-$project_dir/.conda-env/voxel_tool/bin/python}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
exec "$python_executable" -u "$postprocess_dir/test_electronics_layout.py" "$@"
