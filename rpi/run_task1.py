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


# Zhenxi: how long the Pi holds on the PLAN press before declaring ready.
#
# Zero by default, and it should stay zero unless someone has a reason.
#
# This started as a 30 s pause to make it look like planning happened inside
# the run window. It does not need to: Prof Smitha confirmed the setup time
# may overlap the execution time, so computing the route during preparation
# is simply allowed. With nothing to disguise, the pause is pure cost -- it
# happens before START only if you put it there, and anywhere after START it
# comes straight out of the six minutes, which also breaks ties between teams
# on equal scores (FAQ 14).
#
# Kept as a variable because Peter asked for one, and because a couple of
# seconds is genuinely useful if the robot lurches before you have put the
# tablet down. The Pi announces whatever this is and the tablet counts that
# number down, so the two cannot disagree.
ARMING_DELAY_SECONDS = 0

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
        # Zhenxi: look in the queue before the port.
        #
        # Waiting on the board drains Android through
        # a1_bridge.forward_stop_if_pending(), which parks everything that is
        # not a STOP in a1_bridge.inbox. So a press that lands while we are
        # mid-pre-flight -- exactly when an impatient operator presses START,
        # since the two presses are seconds apart -- ends up in that queue and
        # never on the port. Reading only the port lost it, and the button
        # simply did nothing.
        while a1_bridge.inbox:
            command = a1_bridge.inbox.pop(0)
            if command == trigger:
                print(f"[TASK1] {trigger} received from the tablet (queued).")
                return True
            pump_map(android, command)

        if android.in_waiting:
            command = a1_bridge.read_command(android)
            if command == trigger:
                print(f"[TASK1] {trigger} received from the tablet.")
                return True
            # Zhenxi: say so rather than ignoring it. Task 2 runs on the
            # board via a1_bridge.py, not here, so START2 arriving means the
            # wrong program is running on the Pi -- and silence would look
            # exactly like a dead button.
            if command == "START2":
                print("[TASK1] START2 is Task 2 -- stop this and run a1_bridge.py instead.")
                a1_bridge.send_line(android, "MSG,This is the Task 1 runner. Run a1_bridge.py for Task 2.")
                continue
            pump_map(android, command)
        if select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.readline()
            print(f"[TASK1] {trigger} given from the terminal.")
            return True
        time.sleep(0.05)


def preflight(stm, android):
    """Is the robot actually fit to run? Report honestly either way.

    Zhenxi: this is what earns the PLAN press.

    It happens in the preparation window, where time is free, and it is the
    last chance to discover that the board is asleep or that nobody started
    the detection server on the laptop. Finding either of those out after the
    clock has begun costs images, and there is no second chance for a run
    that was never going to work.

    Nothing here moves the robot: a zero-length move asks the board whether
    it is awake and idle without turning a wheel.
    """
    a1_bridge.send_line(android, "STATUS,PLAN,CHECKING")

    a1_bridge.send_line(stm, "FW000")
    reply = wait_for_stm_reply(stm, android)
    if reply != "DONE":
        print(f"[TASK1] Pre-flight: board answered {reply}, not DONE.")
        a1_bridge.send_line(android, f"STATUS,PLAN,FAILED,board answered {reply}")
        return False
    print("[TASK1] Pre-flight: board awake.")

    if not algo_client.server_reachable():
        print("[TASK1] Pre-flight: cannot reach the laptop's algo server.")
        a1_bridge.send_line(android, "STATUS,PLAN,FAILED,laptop algo server unreachable")
        return False
    print("[TASK1] Pre-flight: laptop reachable.")
    return True


def arm_and_wait(stm, android):
    """Pre-flight, then hold for ARMING_DELAY_SECONDS, then declare ready."""
    if not preflight(stm, android):
        return False

    a1_bridge.send_line(android, f"STATUS,PLAN,ARMED,{ARMING_DELAY_SECONDS}")
    if ARMING_DELAY_SECONDS:
        print(f"[TASK1] Holding {ARMING_DELAY_SECONDS}s -- step back.")
    deadline = time.monotonic() + ARMING_DELAY_SECONDS
    while time.monotonic() < deadline:
        # A STOP during the hold has to be honoured: the robot has not moved
        # yet, and this is exactly when someone notices it is pointing the
        # wrong way.
        if android.in_waiting and a1_bridge.read_command(android) == "STOP":
            print("[TASK1] STOP during the hold -- not starting.")
            a1_bridge.send_line(android, "MSG,Run cancelled before moving")
            return False
        time.sleep(0.1)

    a1_bridge.send_line(android, "STATUS,PLAN,SET")
    print("[TASK1] Ready. Waiting for START.")
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


# Zhenxi: a blocked move is recoverable; a jammed or confused one is not.
#
# BLOCKED means the IR saw something and the board stopped short on purpose:
# the robot is stationary, upright and safe, it is just not where the route
# expected. That is worth re-planning around. STALL and TIMEOUT mean the
# wheels stopped or the move never ended -- the robot may be wedged, and
# driving more at that point is how a run stops being able to stop itself.
# BUSY and ERR are protocol faults; continuing would only flood the board.
RECOVERABLE_REPLIES = ("BLOCKED",)
FATAL_REPLIES = ("STALL", "TIMEOUT", "BUSY", "ERR", "NO_REPLY")

# How many times a route may be re-planned mid-run before giving up. Each
# attempt costs a planning round trip out of the six minutes, and a robot
# that is blocked twice in quick succession is usually wedged rather than
# unlucky.
MAX_REPLANS = 2

# Backing off before re-planning restores the clearance the IR objected to,
# and gives the new route somewhere to turn.
BACKOFF_CM = 10


def remaining_targets(steps, done):
    """Obstacle numbers the route still has to photograph."""
    return [
        step["obstacle_id"]
        for step in steps
        if step["type"] == "capture" and step["obstacle_id"] not in done
    ]


def replan_from_here(remaining, android):
    """Ask for a fresh route over the obstacles we have not reached yet.

    Zhenxi: the pose is the honest problem here. After a BLOCKED we know the
    move was cut short but not by how much, so the planner is given the last
    pose we actually confirmed. That is wrong by at most one move length, and
    being approximately right beats abandoning the remaining targets -- each
    one is ten points, and the run is scored on images found, not on how
    tidily we got there.
    """
    payload = [
        {"id": n, "x": a1_bridge.obstacles[n]["pos"][0],
         "y": a1_bridge.obstacles[n]["pos"][1], "face": a1_bridge.obstacles[n]["face"]}
        for n in remaining
        if a1_bridge.obstacles.get(n, {}).get("pos") and a1_bridge.obstacles[n].get("face")
    ]
    if not payload:
        return None
    a1_bridge.send_line(android, f"MSG,Re-planning for {len(payload)} obstacle(s)")
    print(f"[TASK1] Re-planning for {[p['id'] for p in payload]} from {start_pose}")
    return algo_client.plan_route(payload, start=dict(start_pose))


def run_route(steps, stm, android):
    pose = a1_bridge.PoseTracker()
    a1_bridge.send_pose(android, pose)
    done = set()
    replans = 0

    while steps is not None:
        restart = False
        for index, step in enumerate(steps):
            if step["type"] == "move":
                command = step["command"]
                a1_bridge.send_line(stm, command)
                a1_bridge.send_line(android, f"STATUS,SENT,{command}")
                print(f"RPi -> STM32: {command}")
                reply = wait_for_stm_reply(stm, android)

                if reply == "DONE":
                    pose.apply(command)
                    a1_bridge.send_pose(android, pose)
                    continue

                if reply in FATAL_REPLIES:
                    print(f"[TASK1] Move ended in {reply} -- stopping route.")
                    a1_bridge.send_line(android, f"MSG,Run ended early: {reply}")
                    return

                if reply in RECOVERABLE_REPLIES:
                    left = remaining_targets(steps[index:], done)
                    if replans >= MAX_REPLANS or not left:
                        print(f"[TASK1] {reply} and no re-plan left -- stopping route.")
                        a1_bridge.send_line(android, f"MSG,Run ended early: {reply}")
                        return
                    replans += 1
                    print(f"[TASK1] {reply} -- backing off and re-planning "
                          f"({replans}/{MAX_REPLANS}).")
                    a1_bridge.send_line(stm, f"BW{BACKOFF_CM:03d}")
                    wait_for_stm_reply(stm, android)
                    fresh = replan_from_here(left, android)
                    if fresh is None:
                        a1_bridge.send_line(android, "MSG,Re-plan failed, stopping")
                        return
                    steps = fresh
                    restart = True
                    break

            elif step["type"] == "capture":
                obstacle_number = step["obstacle_id"]
                # Zhenxi: never photograph the same obstacle twice.
                #
                # A re-planned route can legitimately pass an obstacle we
                # already have, and a second look may detect something
                # different -- which is not a wasted second but a changed
                # answer, and a wrong image ID is minus ten points (FAQ 4).
                # The first reading stands.
                if obstacle_number in done:
                    print(f"[TASK1] Obstacle {obstacle_number} already done, not re-reading.")
                    continue
                print(f"[TASK1] Reached obstacle {obstacle_number}, capturing...")
                report_obstacle(obstacle_number, android_serial=android, stm_serial=stm)
                done.add(obstacle_number)

        if not restart:
            break

    print(f"[TASK1] Route complete. {len(done)} obstacle(s) photographed.")
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
                wait_for(android, "COMPUTE", "Place obstacles, then press SETUP on the tablet.")

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

            # Second press: pre-flight and hold, still in the prep window.
            while True:
                wait_for(android, "ARM", "Press PLAN on the tablet to check the robot.")
                if arm_and_wait(stm, android):
                    break
                print("[TASK1] Pre-flight failed -- fix it and press PLAN again.")

            # Third press: go. The clock is the supervisor's from here.
            wait_for(android, "START", "Press START when the supervisor says go.")
            run_route(steps, stm, android)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[TASK1] Stopped")
    except serial.SerialException as exc:
        print(f"Serial error: {exc}", file=sys.stderr)
        sys.exit(1)
