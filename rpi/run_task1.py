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


def wait_for_go(android):
    """Drain Android's ADD/SUB/FACE traffic into a1_bridge.obstacles until
    the operator presses Enter at this SSH terminal."""
    print("[TASK1] Place obstacles + faces on Android. Press Enter here when ready to plan+run.")
    while True:
        if android.in_waiting:
            command = a1_bridge.read_command(android)
            if command and a1_bridge.MAP_PATTERN.match(command):
                handled = a1_bridge.handle_map_message(command)
                if handled is None:
                    a1_bridge.send_line(android, "ERR,MALFORMED_MAP_MESSAGE")
                else:
                    a1_bridge.send_line(android, f"STATUS,MAP,{command}")
        if select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.readline()
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
    for step in steps:
        if step["type"] == "move":
            command = step["command"]
            a1_bridge.send_line(stm, command)
            a1_bridge.send_line(android, f"STATUS,SENT,{command}")
            print(f"RPi -> STM32: {command}")
            reply = wait_for_stm_reply(stm, android)
            if reply in ("BLOCKED", "STALL", "NO_REPLY"):
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
