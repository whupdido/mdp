#!/bin/bash
# Denzel: start the Pi side automatically at boot, so on the day you only
# power the robot on and press Connect on the tablet -- no SSH.
#
# Run ONCE on the Pi, as your normal user (it uses sudo itself):
#
#     cd rpi
#     ./install_autostart.sh <tablet-bt-mac> [task2|task1] [laptop-ip]
#
#     ./install_autostart.sh AA:BB:CC:DD:EE:FF task2 192.168.4.40
#
# What it installs (systemd service "mdp"):
#   - binds /dev/rfcomm0 to the tablet          (sudo rfcomm bind 0 <mac>)
#   - runs a1_bridge.py (task2) or run_task1.py (task1)
#   - restarts it 3 s after it exits, so it simply retries until the tablet,
#     the STM (/dev/ttyACM0) and the laptop are all there
#
# Switch task: run this again with the other task.
# Logs:        journalctl -u mdp -f
# Stop/start:  sudo systemctl stop mdp / sudo systemctl start mdp
# Remove:      sudo systemctl disable --now mdp && sudo rm /etc/systemd/system/mdp.service

set -euo pipefail

TABLET_MAC="${1:?usage: ./install_autostart.sh <tablet-bt-mac> [task2|task1] [laptop-ip]}"
TASK="${2:-task2}"
LAPTOP_IP="${3:-192.168.4.40}"

case "$TASK" in
    task2) SCRIPT=a1_bridge.py ;;
    task1) SCRIPT=run_task1.py ;;
    *) echo "task must be task1 or task2, got: $TASK" >&2; exit 1 ;;
esac

RPI_DIR="$(cd "$(dirname "$0")" && pwd)"
RUN_AS="$(id -un)"
PYTHON="$(command -v python3)"
RFCOMM="$(command -v rfcomm)"

sudo tee /etc/systemd/system/mdp.service > /dev/null <<EOF
[Unit]
Description=MDP Pi bridge ($TASK)
After=bluetooth.target network-online.target
Wants=network-online.target bluetooth.target

[Service]
User=$RUN_AS
WorkingDirectory=$RPI_DIR
Environment=MDP_LAPTOP_IP=$LAPTOP_IP
# '+' runs as root: binding rfcomm needs it, the bridge itself does not.
ExecStartPre=+/bin/sh -c '$RFCOMM release 0 2>/dev/null; $RFCOMM bind 0 $TABLET_MAC'
# -u so prints reach journalctl straight away instead of sitting in a buffer.
ExecStart=$PYTHON -u $RPI_DIR/$SCRIPT
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable mdp
sudo systemctl restart mdp

echo "Installed: $SCRIPT as $RUN_AS, tablet $TABLET_MAC, laptop $LAPTOP_IP"
echo "Follow the logs with: journalctl -u mdp -f"
