#!/bin/bash
set -euo pipefail
study_root="$(cd "$(dirname "$0")/../.." && pwd)"
study_runtime="$study_root/.runtime/autorsr"
umask 077
mkdir -p "$study_runtime/source"
python3 -m venv "$study_runtime/venv"
"$study_runtime/venv/bin/python" -m pip install --upgrade pip
"$study_runtime/venv/bin/python" -m pip install torch==2.1.2 torchaudio==2.1.2 --index-url https://download.pytorch.org/whl/cu121
"$study_runtime/venv/bin/python" -m pip install -r "$study_root/scripts/auto-rsr/requirements.txt"
# Pin scoring/normalization code rather than following an upstream branch.
for study_file in auto_rsr.py english.json; do
  curl -fsSL "https://raw.githubusercontent.com/denielleoliva/AutoRsr-NoBert-SARG/bd83acdcf8ff4940538a7864c10142a9845e334a/$study_file" -o "$study_runtime/source/$study_file"
done
# The old S3 endpoint now redirects incorrectly. The official repository copy
# is byte-identical to the VAD weights pinned by WhisperX 3.1.1.
mkdir -p "$study_runtime/torch"
curl -fsSL https://raw.githubusercontent.com/m-bain/whisperX/main/whisperx/assets/pytorch_model.bin \
  -o "$study_runtime/torch/whisperx-vad-segmentation.bin"
printf '%s  %s\n' 0b5b3216d60a2d32fc086b47ea8c67589aaeb26b7e07fcbe620d6d0b83e209ea \
  "$study_runtime/torch/whisperx-vad-segmentation.bin" | sha256sum --check
