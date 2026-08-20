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

COMMAND_PATTERN = re.compile(r"^(?:F[WLR]|B[WLR])\d{3}$|^STOP$")
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
                if not COMMAND_PATTERN.fullmatch(command):
                    send_line(android, "ERR,INVALID_COMMAND")
                    continue

                # The STM parser requires a CR or LF terminated ASCII command.
                send_line(stm, command)
                send_line(android, f"STATUS,SENT,{command}")
                print(f"RPi -> STM32: {command}")

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
