#!/bin/bash
set -euo pipefail
study_root="$(cd "$(dirname "$0")/../.." && pwd)"
study_user="$(id -un)"
study_bridge="$(sudo docker network inspect bridge --format '{{(index .IPAM.Config 0).Gateway}}')"
test -n "$study_bridge"
umask 077
mkdir -p "$study_root/.runtime/autorsr"
printf 'ARSR_HOST=%s\nARSR_PORT=5050\n' "$study_bridge" > "$study_root/.runtime/autorsr/service.env"
sudo tee /etc/systemd/system/bloom-study-autorsr.service > /dev/null <<EOF
[Unit]
Description=Bloom study AutoRSR GPU scorer
After=network-online.target docker.service
Wants=network-online.target

[Service]
Type=simple
User=$study_user
WorkingDirectory=$study_root
EnvironmentFile=$study_root/.runtime/autorsr/service.env
Environment=OMP_NUM_THREADS=4
Environment=TOKENIZERS_PARALLELISM=false
ExecStart=/bin/bash $study_root/scripts/auto-rsr/start.sh
Restart=on-failure
RestartSec=30
TimeoutStopSec=10
MemoryMax=4G
UMask=0077
NoNewPrivileges=true
PrivateTmp=true

[Install]
WantedBy=multi-user.target
EOF
sudo systemctl daemon-reload
sudo systemctl enable --now bloom-study-autorsr
