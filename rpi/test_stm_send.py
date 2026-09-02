#!/usr/bin/env python3
"""
Minimal, standalone STM32 send test -- no Android, no Bluetooth, just:
open the serial port, wait for the boot READY, send one command, print
whatever comes back. Confirms the wiring and protocol work before building
anything more complicated (like A.5's navigate-around-obstacle behaviour)
on top of it.

Run this ON THE RPI (needs the real serial connection to the board):

    python3 rpi/test_stm_send.py FW010

Defaults to FW010 (forward 10cm) if no command is given. See
stm32/STM32_motion_spec.md for the full command list.
"""

import sys
import time

import serial

STM_DEVICE = "/dev/ttyACM0"  # matches a1_bridge.py -- change if your board enumerates differently
BAUD_RATE = 115200


def main():
    command = sys.argv[1].upper() if len(sys.argv) > 1 else "FW010"

    print(f"Opening {STM_DEVICE} at {BAUD_RATE} baud...")
    with serial.Serial(STM_DEVICE, BAUD_RATE, timeout=1) as stm:
        print("Waiting for READY (sent once at boot -- don't send before this)...")
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            line = stm.readline().decode("ascii", errors="replace").strip()
            if line:
                print(f"  <- {line}")
            if line == "READY":
                break
        else:
            print("Never saw READY in 10s -- board may already be past boot, "
                  "or not wired correctly. Continuing anyway.")

        print(f"Sending: {command}")
        stm.write((command + "\n").encode("ascii"))
        stm.flush()

        print("Waiting for reply (READY / DONE / STALL / TIMEOUT / ACK / BUSY / ERR)...")
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            line = stm.readline().decode("ascii", errors="replace").strip()
            if line:
                print(f"  <- {line}")
                break
        else:
            print("No reply within 20s.")


if __name__ == "__main__":
    main()
