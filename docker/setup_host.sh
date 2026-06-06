#!/usr/bin/env bash
# One-time host setup for the GR_Assignment ROS Docker workspace.
# Run with: bash docker/setup_host.sh
# Requires sudo (will prompt once).

set -euo pipefail

echo "==> [1/6] Adding NVIDIA Container Toolkit apt repo"
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
  | sudo gpg --dearmor --yes -o /usr/share/keyrings/nvidia-container-toolkit.gpg

curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit.gpg] https://#g' \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list >/dev/null

echo "==> [2/6] Installing nvidia-container-toolkit"
sudo apt-get update -qq
sudo apt-get install -y nvidia-container-toolkit

echo "==> [3/6] Configuring Docker to use the NVIDIA runtime"
sudo nvidia-ctk runtime configure --runtime=docker

echo "==> [4/6] Restarting Docker"
sudo systemctl restart docker

echo "==> [5/6] Allowing local Docker containers to use the X server"
xhost +local:docker

echo "==> [6/6] Seeding X auth file for containers"
touch /tmp/.docker.xauth
chmod 644 /tmp/.docker.xauth
xauth nlist "${DISPLAY:-:1}" | sed -e 's/^..../ffff/' | xauth -f /tmp/.docker.xauth nmerge - || true

echo
echo "=== Done. Verifying ==="
docker info 2>/dev/null | grep -iE 'runtime' || true
echo
echo "Now run:"
echo "  cd docker && docker compose build jazzy && docker compose build humble"
