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
STATUS_PROBE_INTERVAL_SECONDS = 1.5
PROBE_SETTLE_SECONDS = 0.5
MAX_MOVEMENT_SEND_ATTEMPTS = 3

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


def handle_other(android, command):
    """Deal with anything that arrived but is not the trigger we want.

    Zhenxi: one function because there are two ways in.

    Commands reach wait_for() either straight off the port or out of
    a1_bridge.inbox, where waiting on the board parks them. Both paths have
    to treat a non-trigger the same way, and when they were written out
    separately they drifted immediately: the port path learned about START2
    and the queue path silently dropped it, so whether you got a diagnostic
    or a dead-looking button depended on how close together you pressed.
    """
    if command == "START2":
        # Task 2 runs on the board via a1_bridge.py, not here. Arriving
        # means the wrong program is running on the Pi, and saying nothing
        # looks exactly like a broken button.
        print("[TASK1] START2 is Task 2 -- stop this and run a1_bridge.py instead.")
        a1_bridge.send_line(android, "MSG,This is the Task 1 runner. Run a1_bridge.py for Task 2.")
        return
    if command in ("ARM", "START"):
        # Zhenxi: a press the Pi is not ready for. The tablet only sends these
        # once it believes the earlier steps are done, so this means the two
        # have fallen out of step -- most often because this program was
        # restarted after the tablet had already planned. Ignoring it left the
        # tablet on "CHECKING..." for good. FAILED sends it back to SETUP,
        # which this runner accepts from any state.
        print(f"[TASK1] {command} arrived before the Pi was ready for it -- asking for SETUP.")
        a1_bridge.send_line(android, "STATUS,PLAN,FAILED,the Pi has no route yet - press SETUP")
        return
    pump_map(android, command)


def wait_for(android, trigger, prompt):
    """Collect map traffic until `trigger` arrives. Returns True."""
    wait_for_any(android, (trigger,), prompt)
    return True


def wait_for_any(android, triggers, prompt):
    """Collect map traffic until one of `triggers` arrives; return which.

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
            if command in triggers:
                print(f"[TASK1] {command} received from the tablet (queued).")
                return command
            handle_other(android, command)

        if android.in_waiting:
            command = a1_bridge.read_command(android)
            if command in triggers:
                print(f"[TASK1] {command} received from the tablet.")
                return command
            handle_other(android, command)
        if select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.readline()
            print(f"[TASK1] {triggers[-1]} given from the terminal.")
            return triggers[-1]
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
    reply = wait_for_stm_reply(stm, android, expected_command="FW000")
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


def wait_for_stm_reply(
    stm, android, relay_to_android=True, expected_command=None
):
    """Wait for a terminal move result and recover a lost result by probing.

    ``?`` never repeats the movement. New STM firmware answers STATUS,BUSY
    while it is still executing or STATUS,IDLE,<result> after it has stopped.
    A probe response from older firmware is ignored, so the RPi can be
    deployed before the STM is reflashed.

    If the movement's original terminal line races ahead of an outstanding
    probe response, keep reading briefly. This prevents the next movement
    command from overtaking the queued probe on the STM UART.
    """
    stop_requested = False
    probe_outstanding = False
    deferred_terminal = None
    probe_settle_deadline = None
    command_accepted = False
    send_attempts = 1
    deadline = time.monotonic() + a1_bridge.STM_TIMEOUT_SECONDS
    last_probe = time.monotonic()
    while time.monotonic() < deadline:
        stop_requested = (
            a1_bridge.forward_stop_if_pending(android, stm) or stop_requested
        )
        reply_raw = stm.readline()
        if reply_raw:
            reply = reply_raw.decode("ascii", errors="replace").strip()
            if reply:
                print(f"STM32 -> RPi: {reply}")
                if relay_to_android:
                    a1_bridge.send_line(android, f"STM,{reply}")
                if reply.startswith("ACK,"):
                    acknowledged_command = reply.split(",", 1)[1]
                    if expected_command == acknowledged_command:
                        command_accepted = True
                    continue
                if reply == "ACK":
                    if stop_requested:
                        return "STOPPED"
                    # A delayed ACK from an earlier STOP is not completion of
                    # the movement currently being awaited.
                    continue
                if reply.startswith("STATUS,BUSY"):
                    probe_outstanding = False
                    parts = reply.split(",", 2)
                    reported_command = parts[2] if len(parts) > 2 else None
                    if (
                        expected_command is not None
                        and reported_command is not None
                        and reported_command != expected_command
                    ):
                        return "BUSY"
                    if reported_command == expected_command:
                        command_accepted = True
                    probe_settle_deadline = None
                    continue
                if reply.startswith("STATUS,IDLE,"):
                    response_to_probe = probe_outstanding
                    probe_outstanding = False
                    parts = reply.split(",", 3)
                    result = parts[2]
                    reported_command = parts[3] if len(parts) > 3 else None
                    if (
                        expected_command is not None
                        and reported_command != expected_command
                    ):
                        # The probe still names an older move, so this command
                        # was not accepted. Retry only in that provable case.
                        deferred_terminal = None
                        probe_settle_deadline = None
                        if (
                            response_to_probe
                            and not command_accepted
                            and send_attempts < MAX_MOVEMENT_SEND_ATTEMPTS
                        ):
                            send_attempts += 1
                            a1_bridge.send_line(stm, expected_command)
                            if relay_to_android:
                                a1_bridge.send_line(
                                    android,
                                    f"STATUS,RETRY,{expected_command},{send_attempts}",
                                )
                            print(
                                f"[TASK1] STM did not accept {expected_command}; "
                                f"retrying ({send_attempts}/{MAX_MOVEMENT_SEND_ATTEMPTS})"
                            )
                            last_probe = time.monotonic()
                        continue
                    command_accepted = True
                    if result == "STOPPED":
                        return "STOPPED"
                    if result in a1_bridge.FINAL_REPLIES:
                        return result
                    continue
                if probe_outstanding and reply in ("BUSY", "ERR"):
                    # Older firmware interprets '?' as an ordinary command.
                    probe_outstanding = False
                    if deferred_terminal is not None:
                        return deferred_terminal
                    continue
                if reply in a1_bridge.FINAL_REPLIES:
                    if probe_outstanding:
                        deferred_terminal = reply
                        probe_settle_deadline = min(
                            deadline,
                            time.monotonic() + PROBE_SETTLE_SECONDS,
                        )
                        continue
                    return reply

        now = time.monotonic()
        if deferred_terminal is not None:
            if now >= probe_settle_deadline:
                return deferred_terminal
            continue
        if now - last_probe >= STATUS_PROBE_INTERVAL_SECONDS:
            last_probe = now
            a1_bridge.send_line(stm, "?")
            probe_outstanding = True
            print("RPi -> STM32: ? (status probe)")
    if deferred_terminal is not None:
        return deferred_terminal
    if relay_to_android:
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


def run_route(steps, stm, android, run_id=None):
    pose = a1_bridge.PoseTracker()
    a1_bridge.send_pose(android, pose)
    expected_images = sum(step["type"] == "capture" for step in steps)
    capture_index = 0
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
                reply = wait_for_stm_reply(
                    stm, android, expected_command=command
                )

                if reply == "DONE":
                    pose.apply(command)
                    a1_bridge.send_pose(android, pose)
                    continue

                if reply == "STOPPED":
                    print(f"[TASK1] {command} was stopped from Android -- ending route.")
                    a1_bridge.send_line(android, "MSG,Task 1 stopped")
                    return

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
                    backoff = f"BW{BACKOFF_CM:03d}"
                    a1_bridge.send_line(stm, backoff)
                    wait_for_stm_reply(stm, android, expected_command=backoff)
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
                capture_index += 1
                print(f"[TASK1] Reached obstacle {obstacle_number}, capturing...")
                # Task 1 reports classifications to Android and the laptop
                # collage. The STM does not consume IMxxx for this route.
                report_obstacle(
                    obstacle_number,
                    android_serial=android,
                    run_id=run_id,
                    expected_images=expected_images,
                    capture_index=capture_index,
                )
                done.add(obstacle_number)

        if not restart:
            break

    print(f"[TASK1] Route complete. {len(done)} obstacle(s) photographed.")
    a1_bridge.send_line(android, "MSG,Task 1 route complete")


# Zhenxi: what each stage will act on. COMPUTE is in every one, which is
# what lets the tablet always get back to SETUP.
STAGE_TRIGGERS = {
    "plan": ("COMPUTE",),
    "arm": ("COMPUTE", "ARM"),
    "start": ("COMPUTE", "ARM", "START"),
}
STAGE_PROMPTS = {
    "plan": "Place obstacles, then press SETUP on the tablet.",
    "arm": "Press PLAN on the tablet to check the robot (or SETUP to re-plan).",
    "start": "Press START when the supervisor says go (or SETUP / PLAN to redo).",
}


def prepare(stm, android):
    """Run SETUP / PLAN / START until START arrives; return the route.

    Kept apart from main() so it can be driven in a test without opening any
    ports -- the whole point of the stage table is behaviour that only shows
    up across several presses, which is exactly what a unit test of one
    function at a time would miss.
    """
    stage = "plan"
    steps = None
    while True:
        got = wait_for_any(android, STAGE_TRIGGERS[stage], STAGE_PROMPTS[stage])
        if got == "COMPUTE":
            steps = plan_once(android)
            stage = "arm" if steps is not None else "plan"
        elif got == "ARM":
            stage = "start" if arm_and_wait(stm, android) else "arm"
        else:   # START
            return steps


def plan_once(android):
    """Plan a route over the map the tablet just sent. None if it failed."""
    obstacles = obstacles_payload()
    if not obstacles:
        print("[TASK1] No obstacles with both position and face -- nothing to plan.")
        a1_bridge.send_line(android, "STATUS,PLAN,FAILED,no obstacles with a face")
        return None

    a1_bridge.send_line(android, "STATUS,PLAN,WORKING")
    print(f"[TASK1] Planning for {len(obstacles)} obstacle(s) from {start_pose}...")
    steps = algo_client.plan_route(obstacles, start=dict(start_pose))
    if steps is None:
        a1_bridge.send_line(android, "STATUS,PLAN,FAILED,planner returned no route")
        return None

    moves = sum(1 for step in steps if step["type"] == "move")
    a1_bridge.send_line(android, f"STATUS,PLAN,READY,{moves}")
    print(f"[TASK1] Route ready: {moves} moves.")
    return steps


def main():
    print(f"Opening STM32 on {a1_bridge.STM_DEVICE} at {a1_bridge.BAUD_RATE} baud")
    with serial.Serial(a1_bridge.STM_DEVICE, a1_bridge.BAUD_RATE, timeout=a1_bridge.STM_POLL_SECONDS) as stm:
        print(f"Waiting for Android RFCOMM device {a1_bridge.BT_DEVICE}")
        with serial.Serial(a1_bridge.BT_DEVICE, a1_bridge.BAUD_RATE, timeout=1) as android:
            a1_bridge.send_line(android, "STATUS,Task1 runner ready")

            # Zhenxi: three presses, in order -- but SETUP may come back.
            #
            # Planning is slow enough that doing it inside the six minutes
            # would be giving budget away, and the rules give two minutes of
            # preparation for exactly this. So COMPUTE plans, off the clock,
            # ARM pre-flights, and START only drives a route that exists.
            #
            # This used to be three loops in a row, so it could only go
            # forwards. The tablet can go backwards -- PLAN AGAIN, editing the
            # map after planning, a failed pre-flight -- and each of those
            # sends COMPUTE while this was still waiting for ARM or START. The
            # COMPUTE was ignored and the tablet sat on "PLANNING..." for good.
            # Now COMPUTE is accepted at every stage before the run, so SETUP
            # always works, and the tablet can drop back to it after any
            # failure knowing the Pi will follow.
            steps = prepare(stm, android)

            # The clock is the supervisor's from here.
            run_id = time.strftime("%Y%m%d-%H%M%S")
            run_route(steps, stm, android, run_id=run_id)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[TASK1] Stopped")
    except serial.SerialException as exc:
        print(f"Serial error: {exc}", file=sys.stderr)
        sys.exit(1)
