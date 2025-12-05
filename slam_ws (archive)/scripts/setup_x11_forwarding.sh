#!/usr/bin/env bash
# One-time setup on the Jetson to enable SSH X11 forwarding for RViz
# Usage: sudo bash scripts/setup_x11_forwarding.sh

set -eo pipefail

backup_file() {
  local f="$1"
  if [[ -f "$f" && ! -f "${f}.bak" ]]; then
    cp "$f" "${f}.bak"
    echo "Backed up $f to ${f}.bak"
  fi
}

require_root() {
  if [[ $EUID -ne 0 ]]; then
    echo "Please run as root: sudo bash $0"
    exit 1
  fi
}

require_root

# Ensure X11 auth packages are installed
apt-get update -y
DEBIAN_FRONTEND=noninteractive apt-get install -y xauth x11-apps mesa-utils >/dev/null 2>&1 || true

# Enable X11 forwarding in sshd_config
SSHD_CFG="/etc/ssh/sshd_config"
backup_file "$SSHD_CFG"

ensure_kv() {
  local key="$1" val="$2"
  if grep -qE "^\s*#?\s*${key}\s+" "$SSHD_CFG"; then
    sed -i "s|^\s*#\?\s*${key}\s\+.*|${key} ${val}|" "$SSHD_CFG"
  else
    echo "${key} ${val}" >> "$SSHD_CFG"
  fi
}

ensure_kv X11Forwarding yes
ensure_kv X11UseLocalhost no
ensure_kv AllowTcpForwarding yes
# Optionally reduce conflicts if multiple sessions
ensure_kv X11DisplayOffset 10

systemctl restart ssh || service ssh restart || true

echo "X11 forwarding enabled. On your laptop run:"
echo "  ssh -Y eagle@$(hostname -I | awk '{print $1}')"
echo "Then on the Jetson session:"
echo "  cd /home/eagle/Adversary_DRONE/slam_ws && ./start_rviz.sh"
