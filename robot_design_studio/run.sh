#!/usr/bin/env bash
set -euo pipefail
STUDIO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
STUDIO_PYTHON="${ROBOT_PYTHON:-$STUDIO_ROOT/.venv/bin/python}"
if [[ ! -x "$STUDIO_PYTHON" ]]; then
  echo '请按 README 创建 Python 3.11 环境，或设置 ROBOT_PYTHON。' >&2
  exit 1
fi
cd "$STUDIO_ROOT"
exec "$STUDIO_PYTHON" -m studio.server "$@"
