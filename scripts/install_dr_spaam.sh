#!/usr/bin/env bash
# Installeert DR-SPAAM (persoonsdetectie, my_bot/nodes/PersonDetectorNode.py) op de CPU.
# Voor de echte robot zonder Docker; dezelfde stappen als ~/ros2_docker/Dockerfile.
# Mag vaker gedraaid worden. Gebruik: scripts/install_dr_spaam.sh
set -euo pipefail

DR_SPAAM_COMMIT=e5a5f73f69523b90829be06a2558b597c2934f9f
CHECKPOINT_URL=https://github.com/RWTHVision/DR-SPAAM-Detector/releases/download/v1.1/dr_spaam_e40.pth
CHECKPOINT=/opt/dr_spaam/dr_spaam_e40.pth

SUDO=$([ "$(id -u)" -eq 0 ] || echo sudo)
# Ubuntu 24.04+ weigert systeembrede pip-installaties zonder deze vlag (PEP 668).
PIP_FLAGS=$(python3 -c 'import sys,glob; print("--break-system-packages" if glob.glob(f"/usr/lib/python3.{sys.version_info.minor}/EXTERNALLY-MANAGED") else "")')

$SUDO apt-get update
$SUDO apt-get install -y python3-pip python3-scipy python3-opencv wget

# torch 2.3: nieuwere versies laden checkpoints standaard met weights_only=True.
$SUDO pip3 install $PIP_FLAGS --no-cache-dir torch==2.3.1 --index-url https://download.pytorch.org/whl/cpu
$SUDO pip3 install $PIP_FLAGS --no-cache-dir \
    "git+https://github.com/VisualComputingInstitute/DR-SPAAM-Detector.git@${DR_SPAAM_COMMIT}#subdirectory=dr_spaam"

if [ ! -f "$CHECKPOINT" ]; then
    tmp=$(mktemp --suffix=.pth)
    wget -q -O "$tmp" "$CHECKPOINT_URL"
    $SUDO mkdir -p "$(dirname "$CHECKPOINT")"
    # Op een GPU opgeslagen en Detector laadt zonder map_location: gewichten opnieuw opslaan voor CPU.
    $SUDO python3 -c "import torch; c = torch.load('$tmp', map_location='cpu'); \
torch.save({'model_state': c['model_state']}, '$CHECKPOINT')"
    rm -f "$tmp"
fi

python3 -c "import torch, dr_spaam; print('DR-SPAAM klaar, torch', torch.__version__)"
echo "Controle: python3 -m pytest $(cd "$(dirname "$0")/.." && pwd)/test/test_person_detector.py -v"
