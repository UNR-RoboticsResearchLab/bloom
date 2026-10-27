#!/bin/bash
set -euo pipefail
study_root="$(cd "$(dirname "$0")/../.." && pwd)"
study_runtime="$study_root/.runtime/autorsr"
export HF_HOME="$study_runtime/models"
export ARSR_RECORDINGS_DIR="$study_runtime/recordings"
study_site="$study_runtime/venv/lib/python3.11/site-packages"
export LD_LIBRARY_PATH="$study_site/nvidia/cublas/lib:$study_site/nvidia/cudnn/lib:$study_site/torch/lib:${LD_LIBRARY_PATH:-}"
exec "$study_runtime/venv/bin/python" -u "$study_root/scripts/auto-rsr/service.py"
