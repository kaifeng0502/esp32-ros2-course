#!/usr/bin/env bash
set -e
course_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if ! command -v idf.py >/dev/null 2>&1; then
  if [[ -n "${IDF_PATH:-}" && -f "$IDF_PATH/export.sh" ]]; then
    source "$IDF_PATH/export.sh" >/dev/null
  else
    echo '请先安装 ESP-IDF 5.5.6，并在当前终端 source ESP-IDF 的 export.sh。' >&2
    exit 1
  fi
fi
exec idf.py -C "$course_root" "$@"
