#!/bin/bash
# Denzel: start the Pi side automatically at boot, so on the day you only
# power the robot on and press Connect on the tablet -- no SSH.
#
# Run ONCE on the Pi, as your normal user (it uses sudo itself):
#
#     cd rpi
#     ./install_autostart.sh [task2|task1] [laptop-ip]
#
#     ./install_autostart.sh task2 192.168.4.40
#
# What it installs (systemd service "mdp"):
#   - listens on RFCOMM channel 1 for the tablet, the same as the manual
#     `sudo rfcomm listen /dev/rfcomm0 1` -- the tablet dials in, so no MAC
#   - once the tablet connects, runs a1_bridge.py (task2) or run_task1.py
#     (task1) on /dev/rfcomm0
#   - when the tablet disconnects or the program exits, listens again; if
#     rfcomm itself dies, systemd restarts it 3 s later
#
# Switch task: run this again with the other task.
# Logs:        journalctl -u mdp -f
# Stop/start:  sudo systemctl stop mdp / sudo systemctl start mdp
# Remove:      sudo systemctl disable --now mdp && sudo rm /etc/systemd/system/mdp.service
#
# Pair the tablet with the Pi once first. Stop this service before running
# `rfcomm listen` or the test scripts by hand: both want /dev/rfcomm0 and
# /dev/ttyACM0.

set -euo pipefail

TASK="${1:-task2}"
LAPTOP_IP="${2:-192.168.4.40}"

case "$TASK" in
    task2) SCRIPT=a1_bridge.py ;;
    task1) SCRIPT=run_task1.py ;;
    *) echo "usage: ./install_autostart.sh [task2|task1] [laptop-ip]" >&2; exit 1 ;;
esac

RPI_DIR="$(cd "$(dirname "$0")" && pwd)"
RUN_AS="$(id -un)"
PYTHON="$(command -v python3)"
RFCOMM="$(command -v rfcomm)"
RUNUSER="$(command -v runuser)"

sudo tee /etc/systemd/system/mdp.service > /dev/null <<EOF
[Unit]
Description=MDP Pi bridge ($TASK)
After=bluetooth.target network-online.target
Wants=network-online.target bluetooth.target

[Service]
WorkingDirectory=$RPI_DIR
Environment=MDP_LAPTOP_IP=$LAPTOP_IP
# Free rfcomm0 in case an older install bound it to the tablet.
ExecStartPre=-$RFCOMM release 0
# rfcomm needs root to listen; the program itself runs as $RUN_AS.
# 'watch' is 'listen' that goes back to listening after each connection.
# -u so prints reach journalctl straight away instead of sitting in a buffer.
ExecStart=$RFCOMM watch /dev/rfcomm0 1 $RUNUSER -u $RUN_AS -- $PYTHON -u $RPI_DIR/$SCRIPT
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable mdp
sudo systemctl restart mdp

echo "Installed: $SCRIPT as $RUN_AS, listening on RFCOMM channel 1, laptop $LAPTOP_IP"
echo "Press Connect on the tablet, then follow the logs with: journalctl -u mdp -f"
