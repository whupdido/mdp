#!/usr/bin/env python3
"""
Denzel: Task 2 without the tablet -- camera -> laptop YOLO server -> STM.

Run ON THE PI, with `python -m server.yolo_task1` already running on the
laptop. Two modes:

    python3 test_task2_no_tablet.py scan
        The robot does NOT move. Captures until it sees a left/right arrow,
        sends IM038/IM039 to the board, prints the board's reply. Proves the
        camera, the laptop server and the STM link all work.

    python3 test_task2_no_tablet.py run
        The whole Task 2: sends START2, and answers each SCAN with the arrow,
        exactly as a1_bridge.py does when the tablet presses START. The robot
        DRIVES -- put it on the course, or lift the wheels.

Set MDP_LAPTOP_IP if the laptop is not at capture_and_report's default.
"""

import sys
import time

import serial

import a1_bridge
from capture_and_report import DETECTION_SERVER_IP, read_arrow


class NoTablet:
    """Stands in for the tablet's serial port: never sends anything, and
    prints what the bridge would have shown on the tablet."""

    in_waiting = 0

    def readline(self):
        return b""

    def write(self, data):
        print(f"  (tablet would show) {data.decode('ascii').strip()}")

    def flush(self):
        pass


def scan_only(stm):
    class_id = read_arrow(stm)
    if class_id is None:
        print("No arrow seen. Is it ~30 cm in front of the camera, and is the server running?")
        return
    side = "RIGHT" if class_id == 38 else "LEFT"
    print(f"Arrow {side} ({class_id}) sent to the board, waiting for its reply...")
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        reply = stm.readline().decode("ascii", errors="replace").strip()
        if reply:
            print(f"STM32 -> RPi: {reply}")
            if reply in a1_bridge.FINAL_REPLIES:
                return
    print("The board did not answer the IM line.")


def full_run(stm):
    tablet = NoTablet()
    # Zero the gyro first, and wait for it: GC answers DONE (or ERR) itself,
    # and a START2 sent while GC is still running is rejected as BUSY.
    a1_bridge.send_line(stm, "GC")
    print("RPi -> STM32: GC -- zeroing the gyro, keep the robot still")
    a1_bridge.relay_stm_replies(stm, tablet, a1_bridge.STM_TIMEOUT_SECONDS, "GC")
    a1_bridge.send_line(stm, "START2")
    print("RPi -> STM32: START2 -- the robot will drive now")
    a1_bridge.task2_frames.clear()
    a1_bridge.relay_stm_replies(
        stm, tablet, a1_bridge.TASK2_TIMEOUT_SECONDS, "START2",
        on_scan=a1_bridge.task2_scan,
    )
    # Zhenxi: and the end-of-run sheet, as a1_bridge does after START2 --
    # look for task1_collage_task2-*.jpg in the server's yolo_logs/.
    a1_bridge.show_task2_images(tablet)


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "scan"
    if mode not in ("scan", "run"):
        sys.exit("usage: python3 test_task2_no_tablet.py [scan|run]")

    print(f"Laptop YOLO server: {DETECTION_SERVER_IP}:5001")
    print(f"Opening STM32 on {a1_bridge.STM_DEVICE}")
    with serial.Serial(
        a1_bridge.STM_DEVICE, a1_bridge.BAUD_RATE, timeout=a1_bridge.STM_POLL_SECONDS
    ) as stm:
        time.sleep(0.5)
        stm.reset_input_buffer()  # drop anything the board printed before we listened
        if mode == "scan":
            scan_only(stm)
        else:
            full_run(stm)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped")
