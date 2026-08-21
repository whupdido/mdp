#!/usr/bin/env python3
"""Minimal Android Bluetooth-to-STM32 bridge for checklist A1."""

import re
import sys
import time

import serial


BT_DEVICE = "/dev/rfcomm0"
STM_DEVICE = "/dev/ttyACM0"
BAUD_RATE = 115200
STM_TIMEOUT_SECONDS = 25

# Zhenxi: split the old COMMAND_PATTERN in two.
#
# The tablet sends two different kinds of thing down the same link. Motion
# commands are for the STM board. Map messages -- the obstacle edits behind
# checklist C.6 and C.7 -- are for the algorithm and must never reach the
# board. The single pattern here rejected the map messages outright, so the
# tablet got ERR,INVALID_COMMAND every time an obstacle was placed, moved or
# annotated. Kenneth: swap MAP_PATTERN's branch for a real handoff to the
# algorithm side when you have somewhere to put them.
MOVE_PATTERN = re.compile(r"^(?:F[WLR]|B[WLR])\d{3}$|^STOP$")
MAP_PATTERN = re.compile(r"^(?:ADD|SUB|FACE),")

FINAL_REPLIES = {"DONE", "STALL", "TIMEOUT", "ACK", "BUSY", "ERR"}


def send_line(port, message):
    port.write((message + "\n").encode("ascii"))
    port.flush()


def main():
    print(f"Opening STM32 on {STM_DEVICE} at {BAUD_RATE} baud")
    with serial.Serial(STM_DEVICE, BAUD_RATE, timeout=1) as stm:
        print(f"Waiting for Android RFCOMM device {BT_DEVICE}")
        with serial.Serial(BT_DEVICE, BAUD_RATE, timeout=1) as android:
            send_line(android, "STATUS,RPi bridge ready")
            print("Bridge ready")

            while True:
                raw = android.readline()
                if not raw:
                    continue

                command = raw.decode("ascii", errors="ignore").strip().upper()
                if not command:
                    continue

                print(f"Android -> RPi: {command}")

                # Zhenxi: map edits are acknowledged and dropped rather than
                # rejected. Acknowledging matters -- the tablet shows the user a
                # warning for every ERR it receives, so silently refusing these
                # made it look like the map was broken.
                if MAP_PATTERN.match(command):
                    print(f"map message (not for STM32): {command}")
                    send_line(android, f"STATUS,MAP,{command}")
                    continue

                if not MOVE_PATTERN.fullmatch(command):
                    send_line(android, "ERR,INVALID_COMMAND")
                    continue

                # The STM parser requires a CR or LF terminated ASCII command.
                send_line(stm, command)
                send_line(android, f"STATUS,SENT,{command}")
                print(f"RPi -> STM32: {command}")

                # Zhenxi: worth knowing before the timed runs -- while this loop
                # waits (up to STM_TIMEOUT_SECONDS) nothing is read from Android,
                # so anything the tablet sends mid-move queues in the RFCOMM
                # buffer until the move finishes. Fine for a checklist demo,
                # a problem once obstacles are being edited during a run.
                deadline = time.monotonic() + STM_TIMEOUT_SECONDS
                while time.monotonic() < deadline:
                    reply_raw = stm.readline()
                    if not reply_raw:
                        continue

                    reply = reply_raw.decode("ascii", errors="replace").strip()
                    if not reply:
                        continue

                    print(f"STM32 -> RPi: {reply}")
                    send_line(android, f"STM,{reply}")
                    if reply in FINAL_REPLIES:
                        break
                else:
                    send_line(android, "STM,NO_REPLY")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nBridge stopped")
    except serial.SerialException as exc:
        print(f"Serial error: {exc}", file=sys.stderr)
        sys.exit(1)
