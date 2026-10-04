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
import json
import sys
import time

import serial

import a1_bridge
import algo_client
from capture_and_report import report_obstacle


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


def wait_for_stm_reply(stm, android, *, lines=None, on_first_response=None, execution_started=False):
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
        if lines is not None:
            lines.append(reply)
        if on_first_response is not None:
            on_first_response()
        a1_bridge.send_line(android, f"STM,{reply}")
        if reply == "READY" and execution_started:
            print("[TASK1] POSSIBLE STM RESET: unexpected READY after Task 1 execution began")
            a1_bridge.send_line(android, "MSG,POSSIBLE STM RESET")
            return "POSSIBLE_STM_RESET"
        if reply in a1_bridge.FINAL_REPLIES:
            return reply
    a1_bridge.send_line(android, "STM,NO_REPLY")
    return "NO_REPLY"


def run_route(steps, stm, android):
    pose = a1_bridge.PoseTracker()
    sequence = 0
    a1_bridge.send_pose(android, pose)
    for step in steps:
        if step["type"] == "move":
            command = step["command"]
            sequence += 1
            started = time.monotonic()
            response_lines = []
            first_response = [None]
            serial_error = None
            try:
                a1_bridge.send_line(stm, command)
            except serial.SerialException as exc:
                serial_error = repr(exc)
                elapsed = time.monotonic() - started
                _log_movement(sequence, started, command, response_lines, None, "SERIAL_EXCEPTION", elapsed, serial_error)
                return
            try:
                a1_bridge.send_line(android, f"STATUS,SENT,{command}")
            except serial.SerialException as exc:
                serial_error = repr(exc)
                elapsed = time.monotonic() - started
                _log_movement(sequence, started, command, response_lines, None,
                              "ANDROID_SERIAL_EXCEPTION", elapsed, serial_error)
                print("[TASK1] Stopping route after Android serial exception.")
                return
            print(f"RPi -> STM32: {command}")
            def mark_first():
                if first_response[0] is None:
                    first_response[0] = time.monotonic() - started
            try:
                reply = wait_for_stm_reply(stm, android, lines=response_lines,
                                           on_first_response=mark_first, execution_started=True)
            except serial.SerialException as exc:
                reply = "SERIAL_EXCEPTION"
                serial_error = repr(exc)
            elapsed = time.monotonic() - started
            _log_movement(sequence, started, command, response_lines, first_response[0], reply, elapsed, serial_error)
            if reply == "DONE":
                pose.apply(command)
                a1_bridge.send_pose(android, pose)
            # Any non-success terminal reply means the planner's pose is no
            # longer trustworthy.  In particular, continuing after BUSY can
            # flood the STM32 with commands that it will reject while the
            # route runner incorrectly proceeds to later captures.
            if reply in ("BLOCKED", "STALL", "TIMEOUT", "BUSY", "ERR", "NO_REPLY", "POSSIBLE_STM_RESET", "SERIAL_EXCEPTION"):
                print(f"[TASK1] Move ended in {reply} -- stopping route early.")
                return
        elif step["type"] == "capture":
            obstacle_number = step["obstacle_id"]
            print(f"[TASK1] Reached obstacle {obstacle_number}, capturing...")
            report_obstacle(obstacle_number, android_serial=android, stm_serial=stm)
    print("[TASK1] Route complete.")
    a1_bridge.send_line(android, "MSG,Task 1 route complete")


def _log_movement(sequence, sent_at, command, lines, first_response_latency, terminal, elapsed, exception):
    event = {
        "sequence": sequence,
        "monotonic_send_timestamp": sent_at,
        "raw_command": command,
        "stm_lines": list(lines),
        "first_response_latency_s": first_response_latency,
        "terminal_response": terminal,
        "total_elapsed_s": elapsed,
        "serial_exception": exception,
        "unexpected_ready": "READY" in lines,
    }
    print("[TASK1 STM DIAGNOSTIC] " + json.dumps(event, sort_keys=True))


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

            run_route(steps, stm, android)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[TASK1] Stopped")
    except serial.SerialException as exc:
        print(f"Serial error: {exc}", file=sys.stderr)
        sys.exit(1)
