"""Task 1 SSH entrypoint: relay Android's obstacle placements, let the
operator trigger planning once they're all in, then drive the STM through
the planned route and report each capture to Android.

Reuses a1_bridge.py's tested primitives (send_line, read_command,
handle_map_message, the obstacles dict, the STOP-forwarding wait loop)
instead of duplicating them -- same reasoning as integration_loop.py.

Run on the RPi, after server/yolo_task1.py and server/algo_server.py are
both running on the laptop:
    python3 run_task1.py
"""

from __future__ import annotations

import re
import select
import sys
import time

import serial

import a1_bridge
import algo_client
from capture_and_report import report_obstacle


# Zhenxi: how long the Pi waits after START before it drives.
#
# Asked for so the team can step back and put the tablet on the table, as the
# rules require, without the robot already moving. Be aware it is spent out of
# the six minutes -- the supervisor times from the button press, not from the
# first wheel turn -- so 30 s is 8% of the budget. Lower it if that starts to
# matter. The tablet counts it down from the number announced below, so the
# two cannot disagree.
ARMING_DELAY_SECONDS = 30

# Where the robot is parked. Android sends this before COMPUTE; the default is
# only a fallback for driving the runner by hand over SSH.
start_pose = {"x": 1, "y": 1, "face": "N"}

START_PATTERN = re.compile(r"^ROBOT,(\d+),(\d+),([NESW])$")


def handle_start_pose(command):
    """Record ROBOT,<x>,<y>,<D> as the pose the planner routes from.

    Zhenxi: the planner used to assume (1,1,N). The robot starts in the
    carpark, but which cell of it and facing which way is the supervisor's
    call on the day, so the tablet now says.
    """
    match = START_PATTERN.match(command)
    if not match:
        return False
    start_pose["x"] = int(match.group(1))
    start_pose["y"] = int(match.group(2))
    start_pose["face"] = match.group(3)
    print(f"[TASK1] Start pose set to {start_pose}")
    return True


def pump_map(android, command):
    """Fold one tablet message into the stored map. True if it was one."""
    if command is None:
        return False
    if handle_start_pose(command):
        a1_bridge.send_line(android, f"STATUS,MAP,{command}")
        return True
    if a1_bridge.MAP_PATTERN.match(command):
        handled = a1_bridge.handle_map_message(command)
        if handled is None:
            a1_bridge.send_line(android, "ERR,MALFORMED_MAP_MESSAGE")
        else:
            a1_bridge.send_line(android, f"STATUS,MAP,{command}")
        return True
    return False


def wait_for(android, trigger, prompt):
    """Collect map traffic until `trigger` arrives from the tablet.

    Zhenxi: the triggers come from the tablet, not from this terminal. The
    rules do not allow touching the laptop during an attempt (Task 1 item 6),
    and a keypress in an SSH session is touching the laptop.

    Enter still works, because it is how you drive the runner while debugging
    with no tablet paired. On the day, use the buttons.
    """
    print(f"[TASK1] {prompt}")
    while True:
        if android.in_waiting:
            command = a1_bridge.read_command(android)
            if command == trigger:
                print(f"[TASK1] {trigger} received from the tablet.")
                return True
            pump_map(android, command)
        if select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.readline()
            print(f"[TASK1] {trigger} given from the terminal.")
            return True
        time.sleep(0.05)


def arm_and_wait(android):
    """Count down before driving, telling the tablet how long it has."""
    a1_bridge.send_line(android, f"STATUS,PLAN,ARMED,{ARMING_DELAY_SECONDS}")
    print(f"[TASK1] Starting in {ARMING_DELAY_SECONDS}s -- step back.")
    deadline = time.monotonic() + ARMING_DELAY_SECONDS
    while time.monotonic() < deadline:
        # A STOP during the countdown has to be honoured: the robot has not
        # moved yet, and this is exactly when someone notices it is pointing
        # the wrong way.
        if android.in_waiting and a1_bridge.read_command(android) == "STOP":
            print("[TASK1] STOP during countdown -- not starting.")
            a1_bridge.send_line(android, "MSG,Run cancelled before moving")
            return False
        time.sleep(0.1)
    return True


def obstacles_payload():
    payload = []
    for n, data in a1_bridge.obstacles.items():
        if data.get("pos") is None or data.get("face") is None:
            print(f"[TASK1] Skipping obstacle {n}: missing position or face")
            continue
        x, y = data["pos"]
        payload.append({"id": n, "x": x, "y": y, "face": data["face"]})
    return payload


def wait_for_stm_reply(stm, android):
    """Same wait-for-DONE/STALL/... loop as a1_bridge.main(), reused instead
    of duplicated, including forwarding a mid-move STOP from Android."""
    deadline = time.monotonic() + a1_bridge.STM_TIMEOUT_SECONDS
    while time.monotonic() < deadline:
        a1_bridge.forward_stop_if_pending(android, stm)
        reply_raw = stm.readline()
        if not reply_raw:
            continue
        reply = reply_raw.decode("ascii", errors="replace").strip()
        if not reply:
            continue
        print(f"STM32 -> RPi: {reply}")
        a1_bridge.send_line(android, f"STM,{reply}")
        if reply in a1_bridge.FINAL_REPLIES:
            return reply
    a1_bridge.send_line(android, "STM,NO_REPLY")
    return "NO_REPLY"


def run_route(steps, stm, android):
    pose = a1_bridge.PoseTracker()
    a1_bridge.send_pose(android, pose)
    for step in steps:
        if step["type"] == "move":
            command = step["command"]
            a1_bridge.send_line(stm, command)
            a1_bridge.send_line(android, f"STATUS,SENT,{command}")
            print(f"RPi -> STM32: {command}")
            reply = wait_for_stm_reply(stm, android)
            if reply == "DONE":
                pose.apply(command)
                a1_bridge.send_pose(android, pose)
            # Any non-success terminal reply means the planner's pose is no
            # longer trustworthy.  In particular, continuing after BUSY can
            # flood the STM32 with commands that it will reject while the
            # route runner incorrectly proceeds to later captures.
            if reply in ("BLOCKED", "STALL", "TIMEOUT", "BUSY", "ERR", "NO_REPLY"):
                print(f"[TASK1] Move ended in {reply} -- stopping route early.")
                return
        elif step["type"] == "capture":
            obstacle_number = step["obstacle_id"]
            print(f"[TASK1] Reached obstacle {obstacle_number}, capturing...")
            report_obstacle(obstacle_number, android_serial=android, stm_serial=stm)
    print("[TASK1] Route complete.")
    a1_bridge.send_line(android, "MSG,Task 1 route complete")


def main():
    print(f"Opening STM32 on {a1_bridge.STM_DEVICE} at {a1_bridge.BAUD_RATE} baud")
    with serial.Serial(a1_bridge.STM_DEVICE, a1_bridge.BAUD_RATE, timeout=a1_bridge.STM_POLL_SECONDS) as stm:
        print(f"Waiting for Android RFCOMM device {a1_bridge.BT_DEVICE}")
        with serial.Serial(a1_bridge.BT_DEVICE, a1_bridge.BAUD_RATE, timeout=1) as android:
            a1_bridge.send_line(android, "STATUS,Task1 runner ready")

            # Zhenxi: two presses, not one.
            #
            # Planning is slow enough that doing it inside the six minutes
            # would be giving budget away, and the rules give two minutes of
            # preparation for exactly this. So COMPUTE plans, off the clock,
            # and START only drives a route that already exists.
            while True:
                wait_for(android, "COMPUTE", "Place obstacles, then press PLAN ROUTE on the tablet.")

                obstacles = obstacles_payload()
                if not obstacles:
                    print("[TASK1] No obstacles with both position and face -- nothing to plan.")
                    a1_bridge.send_line(android, "STATUS,PLAN,FAILED,no obstacles with a face")
                    continue

                a1_bridge.send_line(android, "STATUS,PLAN,WORKING")
                print(f"[TASK1] Planning for {len(obstacles)} obstacle(s) from {start_pose}...")
                steps = algo_client.plan_route(obstacles, start=dict(start_pose))
                if steps is None:
                    a1_bridge.send_line(android, "STATUS,PLAN,FAILED,planner returned no route")
                    continue

                moves = sum(1 for step in steps if step["type"] == "move")
                a1_bridge.send_line(android, f"STATUS,PLAN,READY,{moves}")
                print(f"[TASK1] Route ready: {moves} moves. Waiting for START.")
                break

            wait_for(android, "START", "Press START on the tablet when the supervisor says go.")
            if not arm_and_wait(android):
                return

            run_route(steps, stm, android)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[TASK1] Stopped")
    except serial.SerialException as exc:
        print(f"Serial error: {exc}", file=sys.stderr)
        sys.exit(1)
