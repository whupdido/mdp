#!/usr/bin/env python3
"""Minimal Android Bluetooth-to-STM32 bridge for checklist A1."""

# Defers type-hint evaluation -- the Pi's Python is older than 3.9, which
# can't evaluate `dict[int, dict]` at runtime without this (same class of
# issue as the `str | None` fix in capture_and_report.py).
from __future__ import annotations

import re
import sys
import time
import math

import serial


BT_DEVICE = "/dev/rfcomm0"
STM_DEVICE = "/dev/ttyACM0"
BAUD_RATE = 115200
STM_TIMEOUT_SECONDS = 25

# Zhenxi: START2 runs the entire Task 2 routine on the board and only answers
# DONE when it returns, so it cannot share the per-move timeout. The rules give
# Task 2 three minutes; this is that plus enough slack to see the board's own
# reply rather than give up one second early and report NO_REPLY.
TASK2_TIMEOUT_SECONDS = 200

# Zhenxi: split the old COMMAND_PATTERN in two.
#
# The tablet sends two different kinds of thing down the same link. Motion
# commands are for the STM board. Map messages -- the obstacle edits behind
# checklist C.6 and C.7 -- are for the algorithm and must never reach the
# board. The single pattern here rejected the map messages outright, so the
# tablet got ERR,INVALID_COMMAND every time an obstacle was placed, moved or
# annotated.
MOVE_PATTERN = re.compile(r"^(?:F[WLR]|B[WLR])\d{3}$|^STOP$")
# Zhenxi: CLEAR is a map message too -- see handle_map_message.
MAP_PATTERN = re.compile(r"^(?:(?:ADD|SUB|FACE),|CLEAR$)")

# Real parsers for the three map message shapes Android actually sends
# (see Android/PROTOCOL.md -- these are Android's fixed outbound formats,
# not the tolerant set of things it accepts as input).
ADD_PATTERN = re.compile(r"^ADD,B(\d+),\((\d+),(\d+)\)$")
SUB_PATTERN = re.compile(r"^SUB,B(\d+)$")
FACE_PATTERN = re.compile(r"^FACE,B(\d+),([NESW])$")

# Zhenxi: the tablet's start pose, ROBOT,<x>,<y>,<D>. Same shape as the line
# we send the other way, because it means the same thing in both directions.
# run_task1.py consumes it; here it only needs recognising so it is not
# rejected as an invalid command.
TASK1_START_POSE = re.compile(r"^ROBOT,\d+,\d+,[NESW]$")

# Zhenxi: BLOCKED added. The board now stops short when its IR sees an
# obstacle (stm32/Core/Src/control.c) and, since command.c reports how a
# move ended rather than just that it ended, that comes back as BLOCKED.
# Without it here the bridge would relay BLOCKED and then keep waiting for
# a DONE that never comes, until STM_TIMEOUT_SECONDS -> NO_REPLY.
# ACK is only a terminal reply for a STOP.  A delayed ACK from a previous
# STOP can arrive before the next movement's result; treating every ACK as
# terminal lets the bridge send the next command while that movement is still
# running.  The wait loop handles ACK explicitly and only ends on it when the
# current command actually included STOP.
FINAL_REPLIES = {"DONE", "STALL", "TIMEOUT", "BLOCKED", "BUSY", "ERR"}

# Zhenxi: how long one stm.readline() blocks inside the wait loop below.
# It was 1 s (the port's open timeout), which is also how long a STOP from
# the tablet could sit unread. 0.02 s keeps a STOP under 20 ms and is still
# several times longer than the longest line the board sends at 115200 baud.
STM_POLL_SECONDS = 0.02

# Zhenxi: commands that arrived from the tablet while a move was running,
# other than STOP. They are handled after the move, in order, exactly as if
# they had arrived then -- see the wait loop in main().
inbox: list[str] = []

# obstacle_number -> {"pos": (x, y), "face": "N"/"E"/"S"/"W"/None}
# This is what image recognition needs before it can call report_obstacle():
# the obstacle number and which face to look at. Whatever decides "we've
# arrived at obstacle N, go check it" (the real navigation loop -- not
# written yet) should read from this dict once the robot is in position.
obstacles: dict[int, dict] = {}


class PoseTracker:
    """Estimate the Android map pose from commands confirmed by the STM32.

    This is command odometry: it is useful for keeping the app's map in sync,
    but it is not a substitute for wheel encoders or a physical re-reference.
    Coordinates are kept continuously in centimetres and reported as the
    nearest Android cell centre.
    """

    _HEADINGS = ("E", "N", "W", "S")
    _RADIUS_CM = {"FL": 27.7, "FR": 36.5, "BL": 28.1, "BR": 38.3}

    def __init__(self):
        self.x_cm = 15.0
        self.y_cm = 15.0
        self.heading_rad = math.pi / 2.0

    def wire_pose(self) -> str:
        # Android's grid uses cell centres at 5, 15, ... cm.  The robot centre
        # cell is the nearest centre to the estimated continuous position.
        x = max(1, min(18, math.floor(self.x_cm / 10.0)))
        y = max(1, min(18, math.floor(self.y_cm / 10.0)))
        heading = min(
            self._HEADINGS,
            key=lambda token: abs((self.heading_rad - self._heading(token) + math.pi) % (2 * math.pi) - math.pi),
        )
        return f"ROBOT,{x},{y},{heading}"

    @classmethod
    def _heading(cls, token: str) -> float:
        return {"E": 0.0, "N": math.pi / 2.0, "W": math.pi, "S": -math.pi / 2.0}[token]

    def apply(self, command: str) -> str | None:
        match = re.fullmatch(r"(FW|BW|FL|FR|BL|BR)(\d{3})", command)
        if not match:
            return None
        verb, raw = match.groups()
        magnitude = float(raw)
        if verb in ("FW", "BW"):
            distance = magnitude if verb == "FW" else -magnitude
            self.x_cm += distance * math.cos(self.heading_rad)
            self.y_cm += distance * math.sin(self.heading_rad)
            return self.wire_pose()

        angle = math.radians(magnitude)
        if verb in ("FR", "BL"):
            angle = -angle
        gear_sign = -1.0 if verb.startswith("B") else 1.0
        radius = self._RADIUS_CM[verb]
        # Keep the signed turn angle: FR/BL use a negative mathematical
        # heading change while FL/BR use a positive one.  Dropping that sign
        # mirrors right turns and sends the reported map pose to the wrong
        # side of the arena.
        signed_radius = gear_sign * abs(angle) * radius / angle
        start_heading = self.heading_rad
        self.x_cm += signed_radius * (math.sin(start_heading + angle) - math.sin(start_heading))
        self.y_cm -= signed_radius * (math.cos(start_heading + angle) - math.cos(start_heading))
        self.heading_rad = (start_heading + angle + math.pi) % (2 * math.pi) - math.pi
        return self.wire_pose()


def send_pose(android, tracker: PoseTracker) -> None:
    send_line(android, tracker.wire_pose())


def handle_map_message(command: str) -> str:
    """Parse one ADD/SUB/FACE message and update `obstacles`. Returns the
    status text to echo back to Android (mirrors the old unconditional ack,
    but now actually does something with the data first)."""
    # Zhenxi: forget the whole map.
    #
    # The Pi only ever forgot an obstacle on SUB, and the tablet's Undo, Clear
    # and Demo buttons never sent one. So anything removed that way lived on
    # here as a ghost: key in five, press Clear, key in three, and the planner
    # still routed to all five. The tablet now sends CLEAR before every full
    # map it publishes, which makes that publish authoritative -- whatever the
    # buttons did in between, the Pi ends up holding exactly what is on screen.
    if command == "CLEAR":
        forgotten = len(obstacles)
        obstacles.clear()
        print(f"[MAP] cleared ({forgotten} obstacle(s) forgotten)")
        return command

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
    return None


def send_line(port, message):
    port.write((message + "\n").encode("ascii"))
    port.flush()


def read_command(port):
    """One line from the tablet, normalised the way the main loop expects,
    or None if the port had nothing / only whitespace."""
    raw = port.readline()
    if not raw:
        return None
    command = raw.decode("ascii", errors="ignore").strip().upper()
    return command or None


def discard_pending_android(android):
    """Discard complete commands already buffered by the tablet.

    STOP is a queue reset: commands that arrived before it must not run after
    the current move ends. The serial driver may already have copied some of
    those commands into its receive buffer, so clearing ``inbox`` alone is
    insufficient.
    """
    discarded = 0
    while android.in_waiting:
        if read_command(android) is not None:
            discarded += 1
    return discarded


def clear_command_queues(android):
    """Clear commands held by both Python and the RFCOMM serial driver."""
    discarded = len(inbox)
    inbox.clear()
    return discarded + discard_pending_android(android)


def next_command(android):
    """Return the next command, giving a buffered STOP priority.

    After a move completes, queued commands may be waiting in ``inbox`` while
    a STOP is still in Android's receive buffer. Drain available Android data
    first so STOP can clear the queue instead of waiting behind it.
    """
    # With no queued command, read exactly one new command so a STOP that is
    # already behind the first movement still interrupts that movement. Once
    # a queue exists, drain the available Android data to give STOP priority
    # over the queued work.
    if not inbox:
        return read_command(android)

    stop_seen = False
    while android.in_waiting:
        command = read_command(android)
        if command is None:
            continue
        if command == "STOP":
            stop_seen = True
        else:
            inbox.append(command)

    if stop_seen:
        return "STOP"
    return inbox.pop(0)


def forward_stop_if_pending(android, stm):
    """
    Zhenxi: let a STOP through while a move is running.

    The wait loop in main() used to read only the board until the move
    ended, so a STOP pressed on the tablet mid-move sat in the RFCOMM
    buffer until the robot had finished on its own -- which is the one
    moment a STOP button must not wait. The board can now act on a STOP
    mid-move (control.c polls the UART inside its move loops), so this
    side has to hand it over promptly too.

    Only STOP jumps the queue. Anything else that arrives mid-move goes
    into `inbox` and is handled after the move, in order, as before --
    forwarding it now would just earn a BUSY from the board and lose it.
    """
    stop_forwarded = False
    while android.in_waiting:
        command = read_command(android)
        if command is None:
            continue
        if command == "STOP":
            cleared = clear_command_queues(android)
            print("Android -> RPi: STOP (mid-move)")
            if cleared:
                print(f"[QUEUE] cleared {cleared} pending command(s)")
            send_line(stm, "STOP")
            send_line(android, "STATUS,SENT,STOP")
            print("RPi -> STM32: STOP")
            stop_forwarded = True
        else:
            inbox.append(command)
    return stop_forwarded


def relay_stm_replies(stm, android, timeout_s, command, pose=None, on_scan=None):
    """
    Zhenxi: wait for the board to finish `command`, relaying everything it
    says on the way, and keep watching the tablet for a STOP while we wait.

    Was inline in main(); pulled out so START2 can reuse it with the Task 2
    budget instead of the per-move one. `pose` is the dead-reckoner, and is
    left None for anything that is not a single move primitive -- START2 runs
    a whole routine, so there is nothing sensible to add to the estimate.

    Denzel: `on_scan(stm, android)` is called when the board says SCAN, i.e.
    task_2() has stopped in front of an arrow and is waiting ~3 s for
    IM038/IM039. It returns the Image ID it sent, or None if it sent nothing.
    """
    stop_requested = command == "STOP"
    # Denzel: the board answers each IMxxx with its own DONE, while task_2()
    # is still running. Count them so that DONE is not mistaken for the
    # routine's final DONE -- which would end the wait mid-run and leave the
    # second SCAN with nobody listening.
    im_dones_pending = 0
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        stop_requested = forward_stop_if_pending(android, stm) or stop_requested
        reply_raw = stm.readline()  # blocks STM_POLL_SECONDS at most
        if not reply_raw:
            continue

        reply = reply_raw.decode("ascii", errors="replace").strip()
        if not reply:
            continue

        print(f"STM32 -> RPi: {reply}")
        if reply == "DONE" and im_dones_pending:
            # The IM's own reply, not the routine's -- keep it off the
            # tablet, which would read STM,DONE as Task 2 finished.
            im_dones_pending -= 1
            continue
        send_line(android, f"STM,{reply}")
        if reply == "ACK":
            if stop_requested:
                break
            # ACK belongs to a STOP, or to START2 saying it accepted the
            # run -- neither is the result we are waiting for. Relay it for
            # diagnostics and keep waiting for the real one.
            continue
        if reply == "SCAN" and on_scan is not None:
            if on_scan(stm, android) is not None:
                im_dones_pending += 1
            continue
        if reply in FINAL_REPLIES:
            if reply == "DONE" and pose is not None and command != "STOP":
                pose.apply(command)
                send_pose(android, pose)
            break
    else:
        send_line(android, "STM,NO_REPLY")


def task2_scan(stm, android):
    """Denzel: answer the board's SCAN with the arrow it is looking at.

    Imported here rather than at the top because capture_and_report imports
    this module, and because it pulls in cv2 -- which the offline tests and
    a Pi with no camera should not need just to run the bridge.
    """
    from capture_and_report import read_arrow

    try:
        class_id = read_arrow(stm)
    except Exception as exc:  # camera or detection server down
        print(f"[TASK2] Arrow read failed: {exc}")
        send_line(android, "MSG,Arrow read failed - board will use its default")
        return None
    if class_id is None:
        send_line(android, "MSG,No arrow seen - board will use its default")
    else:
        side = "RIGHT" if class_id == 38 else "LEFT"
        send_line(android, f"MSG,Arrow {side} ({class_id})")
    return class_id


def main(on_face_known=None, on_scan=task2_scan):
    """Run the bridge. `on_face_known(stm, android, obstacle_number)`, if
    given, is called once -- not on every resend -- the moment an obstacle
    goes from "no face known yet" to "face known", with the same open stm
    and android connections this loop already holds. Left as None by
    default so test_a1_bridge.py's existing behaviour is unchanged."""
    print(f"Opening STM32 on {STM_DEVICE} at {BAUD_RATE} baud")
    with serial.Serial(STM_DEVICE, BAUD_RATE, timeout=STM_POLL_SECONDS) as stm:
        print(f"Waiting for Android RFCOMM device {BT_DEVICE}")
        with serial.Serial(BT_DEVICE, BAUD_RATE, timeout=1) as android:
            send_line(android, "STATUS,RPi bridge ready")
            pose = PoseTracker()
            print("Bridge ready")

            while True:
                # Zhenxi: anything that queued up during the last move comes
                # first, so ordering from the tablet's point of view is kept.
                command = next_command(android)
                if command is None:
                    continue

                print(f"Android -> RPi: {command}")

                # STOP is a queue barrier.  Even when it is the first command
                # read by the main loop (rather than arriving mid-move), drop
                # every older buffered command before it reaches the STM.
                if command == "STOP":
                    cleared = clear_command_queues(android)
                    if cleared:
                        print(f"[QUEUE] cleared {cleared} pending command(s)")

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

                    # Zhenxi: only acknowledge what was actually understood.
                    #
                    # This acknowledged every message whose prefix looked like
                    # a map edit, including ones handle_map_message() then
                    # dropped as an unrecognised shape. The tablet would show
                    # the edit as accepted while the Pi had discarded it --
                    # a disagreement neither side could see. handle_map_message
                    # now returns None for those, and they get an ERR the
                    # tablet surfaces as a warning.
                    handled = handle_map_message(command)
                    if handled is None:
                        send_line(android, "ERR,MALFORMED_MAP_MESSAGE")
                        continue
                    send_line(android, f"STATUS,MAP,{command}")

                    if on_face_known is not None and n is not None and not face_was_known:
                        if obstacles.get(n, {}).get("face") is not None:
                            on_face_known(stm, android, n)
                    continue

                # Zhenxi: Task 1's trigger is not ours -- run_task1.py owns
                # the plan-and-drive loop, and this plain bridge cannot do it.
                # Say so rather than answering ERR, which the tablet paints as
                # a red warning for a button that did nothing wrong.
                # Zhenxi: Task 1's triggers and its start pose are not ours --
                # run_task1.py owns the plan-and-drive loop, and this plain
                # bridge cannot do it. Say so rather than answering ERR, which
                # the tablet paints as a red warning for a button that did
                # nothing wrong. The tablet sends these whenever it is running,
                # including during checklist demos when this is what listens.
                # Zhenxi: SETUP and PLAN get a plan failure, not a chat line.
                #
                # The tablet sits on "PLANNING..." / "CHECKING..." until it
                # hears a STATUS,PLAN reply. A plain MSG told the operator what
                # was wrong but left the buttons dead, so the wrong program --
                # the likeliest mistake on the day -- froze the tablet. FAILED
                # puts it straight back to SETUP with the reason on screen.
                if command in ("COMPUTE", "ARM"):
                    print(f"[RUN] {command} refused -- this is a1_bridge, run run_task1.py for Task 1")
                    send_line(android, "STATUS,PLAN,FAILED,wrong program on the Pi - run run_task1.py")
                    continue
                if command == "START" or TASK1_START_POSE.match(command):
                    print(f"[RUN] {command} ignored -- this is a1_bridge, run run_task1.py for Task 1")
                    send_line(android, "MSG,Bridge only. Start Task 1 from run_task1.py.")
                    continue

                # Zhenxi: Task 2 IS ours, as of Wen Rong's START2 in
                # command.c -- the whole routine lives on the board, so the
                # bridge just has to hand the command over and relay what
                # comes back. The board answers ACK when it accepts, then
                # DONE when the routine returns, which is why this waits on
                # the Task 2 budget rather than the per-move one.
                if command == "START2":
                    send_line(stm, command)
                    send_line(android, f"STATUS,SENT,{command}")
                    print(f"RPi -> STM32: {command} (Task 2, up to {TASK2_TIMEOUT_SECONDS}s)")
                    relay_stm_replies(
                        stm, android, TASK2_TIMEOUT_SECONDS, command, on_scan=on_scan
                    )
                    continue

                if not MOVE_PATTERN.fullmatch(command):
                    send_line(android, "ERR,INVALID_COMMAND")
                    continue

                # The STM parser requires a CR or LF terminated ASCII command.
                send_line(stm, command)
                send_line(android, f"STATUS,SENT,{command}")
                print(f"RPi -> STM32: {command}")

                relay_stm_replies(stm, android, STM_TIMEOUT_SECONDS, command, pose)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nBridge stopped")
    except serial.SerialException as exc:
        print(f"Serial error: {exc}", file=sys.stderr)
        sys.exit(1)
