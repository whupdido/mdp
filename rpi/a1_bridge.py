#!/usr/bin/env python3
"""Minimal Android Bluetooth-to-STM32 bridge for checklist A1."""

# Defers type-hint evaluation -- the Pi's Python is older than 3.9, which
# can't evaluate `dict[int, dict]` at runtime without this (same class of
# issue as the `str | None` fix in capture_and_report.py).
from __future__ import annotations

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
# annotated.
MOVE_PATTERN = re.compile(r"^(?:F[WLR]|B[WLR])\d{3}$|^STOP$")
MAP_PATTERN = re.compile(r"^(?:ADD|SUB|FACE),")

# Real parsers for the three map message shapes Android actually sends
# (see Android/PROTOCOL.md -- these are Android's fixed outbound formats,
# not the tolerant set of things it accepts as input).
ADD_PATTERN = re.compile(r"^ADD,B(\d+),\((\d+),(\d+)\)$")
SUB_PATTERN = re.compile(r"^SUB,B(\d+)$")
FACE_PATTERN = re.compile(r"^FACE,B(\d+),([NESW])$")

FINAL_REPLIES = {"DONE", "STALL", "TIMEOUT", "ACK", "BUSY", "ERR"}

# obstacle_number -> {"pos": (x, y), "face": "N"/"E"/"S"/"W"/None}
# This is what image recognition needs before it can call report_obstacle():
# the obstacle number and which face to look at. Whatever decides "we've
# arrived at obstacle N, go check it" (the real navigation loop -- not
# written yet) should read from this dict once the robot is in position.
obstacles: dict[int, dict] = {}


def handle_map_message(command: str) -> str:
    """Parse one ADD/SUB/FACE message and update `obstacles`. Returns the
    status text to echo back to Android (mirrors the old unconditional ack,
    but now actually does something with the data first)."""
    m = ADD_PATTERN.match(command)
    if m:
        n, x, y = int(m.group(1)), int(m.group(2)), int(m.group(3))
        obstacles[n] = {"pos": (x, y), "face": obstacles.get(n, {}).get("face")}
        print(f"[MAP] obstacle {n} placed at {(x, y)}")
        return command

    m = SUB_PATTERN.match(command)
    if m:
        n = int(m.group(1))
        obstacles.pop(n, None)
        print(f"[MAP] obstacle {n} removed")
        return command

    m = FACE_PATTERN.match(command)
    if m:
        n, face = int(m.group(1)), m.group(2)
        if n in obstacles:
            obstacles[n]["face"] = face
            print(f"[MAP] obstacle {n} face set to {face} -- ready for detection once robot arrives")
        else:
            # FACE for an obstacle we never saw an ADD for -- shouldn't
            # happen if Android's already validating this, but don't crash
            # the bridge over it.
            obstacles[n] = {"pos": None, "face": face}
            print(f"[MAP] face {face} set for obstacle {n} with no known position yet")
        return command

    # Matched MAP_PATTERN's loose prefix check but not any real shape --
    # log it so a format mismatch is visible instead of silently swallowed.
    print(f"[MAP] unrecognised map message, ignoring: {command}")
    return command


def send_line(port, message):
    port.write((message + "\n").encode("ascii"))
    port.flush()


def main(on_face_known=None):
    """Run the bridge. `on_face_known(stm, android, obstacle_number)`, if
    given, is called once -- not on every resend -- the moment an obstacle
    goes from "no face known yet" to "face known", with the same open stm
    and android connections this loop already holds. Left as None by
    default so test_a1_bridge.py's existing behaviour is unchanged."""
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

                # Map edits are acknowledged (never rejected -- the tablet
                # shows the user a warning for every ERR it receives) AND
                # now actually recorded in `obstacles`, instead of just
                # being echoed back and dropped.
                if MAP_PATTERN.match(command):
                    face_match = FACE_PATTERN.match(command)
                    n = int(face_match.group(1)) if face_match else None
                    face_was_known = (
                        n is not None and obstacles.get(n, {}).get("face") is not None
                    )

                    handle_map_message(command)
                    send_line(android, f"STATUS,MAP,{command}")

                    if on_face_known is not None and n is not None and not face_was_known:
                        if obstacles.get(n, {}).get("face") is not None:
                            on_face_known(stm, android, n)
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
