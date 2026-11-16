#!/usr/bin/env bash
# Recompute the cactus postprocessing stages and open their recorded playback.
set -euo pipefail

robot_demo_open=false
robot_demo_live_schema=false
for robot_demo_argument in "$@"; do
  case "$robot_demo_argument" in
    --open) robot_demo_open=true ;;
    --live-schema) robot_demo_live_schema=true ;;
    *) echo "Usage: bash run_generation_demo.sh [--open] [--live-schema]" >&2; exit 64 ;;
  esac
done

robot_demo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
robot_demo_python="${ROBOT_DEMO_PYTHON:-$robot_demo_dir/../.conda-env/voxel_tool/bin/python}"
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-1}"
mkdir -p "$robot_demo_dir/outputs"
robot_demo_output="$(mktemp -d "$robot_demo_dir/outputs/generation_demo_$(date +%Y%m%d_%H%M%S)_XXXXXX")"

echo "GENERATION_OUTPUT: $robot_demo_output"
robot_demo_schema_args=()
if [[ "$robot_demo_live_schema" == true ]]; then
  echo "VLM: saved opening ranking replay; LIVE restrict-schema generation using the existing .env"
else
  robot_demo_schema_source="$robot_demo_dir/outputs/cactus_restrict_schema_live_verified/layout_constraints_raw.json"
  if [[ ! -f "$robot_demo_schema_source" ]]; then
    echo "Saved real VLM restrict schema is missing. Use --live-schema to generate one, or supply --layout-schema-proposal to assemble_robot.py." >&2
    exit 66
  fi
  robot_demo_schema_args=(--layout-schema-proposal "$robot_demo_schema_source")
  echo "VLM: saved real-model opening ranking and restrict-schema replay; no new model call"
fi
if "$robot_demo_python" -u "$robot_demo_dir/assemble_robot.py" \
    --config "$robot_demo_dir/configs/cactus_carrier_assembly.json" \
    --output "$robot_demo_output" \
    --geometry-cache "$robot_demo_dir/outputs/cactus_carrier_assembly_raw" \
    --proposal "$robot_demo_dir/outputs/cactus_carrier_assembly_verified_live/proposal_raw.json" \
    "${robot_demo_schema_args[@]}"; then
  robot_demo_result=0
else
  robot_demo_result=$?
fi
if [[ "$robot_demo_result" -ne 0 && "$robot_demo_result" -ne 2 ]]; then
  echo "Generation failed; inspect $robot_demo_output/manifest.json" >&2
  exit "$robot_demo_result"
fi
if [[ "$robot_demo_result" -eq 2 ]]; then
  echo "GENERATION_STATUS: partial; physical assembly and load validation remain pending"
fi
"$robot_demo_python" "$robot_demo_dir/verify_shell_fasteners.py" --output "$robot_demo_output"
echo "PROCESS_VIEWER: $robot_demo_output/process.html"
echo "STEP_CHANGES: $robot_demo_output/process_changes.md"
if [[ "$robot_demo_open" == true ]]; then
  xdg-open "$robot_demo_output/process.html"
fi
