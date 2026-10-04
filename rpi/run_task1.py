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

import select
import sys
import time

import serial

import a1_bridge
import algo_client
from capture_and_report import report_obstacle


STATUS_PROBE_INTERVAL_SECONDS = 1.5
PROBE_SETTLE_SECONDS = 0.5
MAX_MOVEMENT_SEND_ATTEMPTS = 3


def wait_for_go(android):
    """Drain Android's ADD/SUB/FACE traffic into a1_bridge.obstacles until
    the run is triggered. Returns when it is.

    Zhenxi: the trigger is now START from the tablet.

    This waited on Enter at the SSH terminal, which the competition rules do
    not allow: during a Task 1 attempt the team may not touch any equipment
    except the start button on the Android device (rules, Task 1 item 6), and
    a keypress in a laptop SSH session is touching the laptop. The tablet now
    has a START button that sends this.

    Enter still works, because it is how you start a run while debugging over
    SSH with no tablet paired. On the day, use the button.
    """
    print("[TASK1] Place obstacles + faces on Android.")
    print("[TASK1] Press START on the tablet when ready (or Enter here, for testing).")
    while True:
        if android.in_waiting:
            command = a1_bridge.read_command(android)
            if command == "START":
                print("[TASK1] START received from the tablet.")
                a1_bridge.send_line(android, "STATUS,Run starting")
                return
            if command and a1_bridge.MAP_PATTERN.match(command):
                handled = a1_bridge.handle_map_message(command)
                if handled is None:
                    a1_bridge.send_line(android, "ERR,MALFORMED_MAP_MESSAGE")
                else:
                    a1_bridge.send_line(android, f"STATUS,MAP,{command}")
        if select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.readline()
            print("[TASK1] Started from the terminal.")
            return
        time.sleep(0.05)


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

    ``?`` never repeats the movement.  New STM firmware answers STATUS,BUSY
    while it is still executing or STATUS,IDLE,<result> after it has stopped.
    A probe response from older firmware is ignored, so deploying the RPi
    side before reflashing the STM does not turn a running move into BUSY.

    If the movement's original terminal line races ahead of an outstanding
    probe response, keep reading briefly.  This prevents the next movement
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
                        # A different command is executing, so ours was not
                        # accepted and it is unsafe to continue the route.
                        return "BUSY"
                    if reported_command == expected_command:
                        command_accepted = True
                    deferred_terminal = None
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
                        # A STATUS response to a probe sent after our command
                        # still names an older move: the STM never accepted
                        # our command. Retry only in that provable case, and
                        # cap attempts so a broken link cannot move forever.
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
                    # NONE means no completed movement is available yet.
                    continue
                if probe_outstanding and reply in ("BUSY", "ERR"):
                    # Pre-status-protocol firmware interprets '?' as an
                    # ordinary command.  Do not confuse that response with
                    # rejection of the movement that preceded the probe.
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


def run_route(steps, stm, android, run_id=None):
    pose = a1_bridge.PoseTracker()
    a1_bridge.send_pose(android, pose)
    expected_images = sum(step["type"] == "capture" for step in steps)
    capture_index = 0
    for step in steps:
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
            elif reply == "STOPPED":
                print(f"[TASK1] {command} was stopped from Android -- ending route.")
                a1_bridge.send_line(android, "MSG,Task 1 stopped")
                return
            elif reply == "NO_REPLY":
                # Keep Task 1 moving when the STM's terminal reply is lost.
                # The physical pose may be uncertain, so do not advance the
                # Android dead-reckoner as though the command were confirmed.
                print(f"[TASK1] No STM reply for {command} -- continuing route.")
                a1_bridge.send_line(android, f"MSG,No STM reply for {command}; continuing route")
                continue
            # Any non-success terminal reply means the planner's pose is no
            # longer trustworthy.  In particular, continuing after BUSY can
            # flood the STM32 with commands that it will reject while the
            # route runner incorrectly proceeds to later captures.
            if reply in ("BLOCKED", "STALL", "TIMEOUT", "BUSY", "ERR"):
                print(f"[TASK1] Move ended in {reply} -- stopping route early.")
                return
        elif step["type"] == "capture":
            capture_index += 1
            obstacle_number = step["obstacle_id"]
            print(f"[TASK1] Reached obstacle {obstacle_number}, capturing...")
            # Task 1 only needs the classification on Android and in the
            # laptop collage.  The STM does not use it to execute the planned
            # route, so do not send IMxxx or make route progress depend on an
            # unrelated image-result acknowledgement.  Task 2/A.5 keep their
            # separate IMxxx path in capture_and_report.py.
            report_obstacle(
                obstacle_number,
                android_serial=android,
                run_id=run_id,
                expected_images=expected_images,
                capture_index=capture_index,
            )
    print("[TASK1] Route complete.")
    a1_bridge.send_line(android, "MSG,Task 1 route complete")


def main():
    print(f"Opening STM32 on {a1_bridge.STM_DEVICE} at {a1_bridge.BAUD_RATE} baud")
    with serial.Serial(a1_bridge.STM_DEVICE, a1_bridge.BAUD_RATE, timeout=a1_bridge.STM_POLL_SECONDS) as stm:
        print(f"Waiting for Android RFCOMM device {a1_bridge.BT_DEVICE}")
        with serial.Serial(a1_bridge.BT_DEVICE, a1_bridge.BAUD_RATE, timeout=1) as android:
            a1_bridge.send_line(android, "STATUS,Task1 runner ready")
            wait_for_go(android)

            obstacles = obstacles_payload()
            if not obstacles:
                print("[TASK1] No obstacles with both position and face -- nothing to plan.")
                return

            print(f"[TASK1] Planning route for {len(obstacles)} obstacle(s)...")
            steps = algo_client.plan_route(obstacles)
            if steps is None:
                a1_bridge.send_line(android, "MSG,Planning failed, check RPi logs")
                return

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
