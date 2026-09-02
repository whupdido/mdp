"""
A.5 demo: "Demonstrate that your robot can navigate towards a given obstacle
having a visual marker indicating obstacle. Your robot needs to navigate
around the obstacle in search of face which has a valid image from the
image list."

This is a SIMPLE, standalone demo for checklist sign-off -- not the final
integrated navigation. It uses a fixed step-and-turn pattern (drive forward,
turn a little, detect, repeat) rather than real path planning, which is
algorithm/pathfinding's job. Good enough to demonstrate the required
behaviour; the step size and turn angle below are guesses and will likely
need tuning against the real obstacle size once you're testing on hardware.

Run ON THE RPI, with all of the following actually working first:
  - STM32 connected and flashed with working motion firmware (test with
    rpi/test_stm_send.py first)
  - camera working (test with rpi/capture_and_report.py standalone)
  - your Mac's `python -m server.yolo_task1` running and reachable

Usage:
    python3 a5_search.py <obstacle_number>
"""

import sys
import time

import serial

from capture_and_report import capture_frame, detect, send_to_stm

STM_DEVICE = "/dev/ttyACM0"
BAUD_RATE = 115200

BULLSEYE_ID = 41       # the marker itself -- not a valid face, keep searching
MAX_ATTEMPTS = 8        # give up after this many steps around the obstacle
STEP_FORWARD_CM = 15     # how far to creep forward each step
TURN_DEGREES = 30         # how much to turn each step -- FL preferred, needs less clearance (see STM32_motion_spec.md)


def send_stm_command(stm: serial.Serial, command: str, timeout: float = 25):
    """Send one command, wait for the STM's reply. Returns the reply string,
    or None if nothing came back in time."""
    stm.write((command + "\n").encode("ascii"))
    stm.flush()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        line = stm.readline().decode("ascii", errors="replace").strip()
        if line:
            return line
    return None


def search_for_face(stm: serial.Serial, obstacle_number: int, android_serial=None):
    """Step around the obstacle, detecting after each move, until a valid
    face (not the marker) is found. Returns the detected Image ID, or None
    if MAX_ATTEMPTS is reached without finding one."""
    for attempt in range(1, MAX_ATTEMPTS + 1):
        print(f"[SEARCH] Attempt {attempt}/{MAX_ATTEMPTS}: capturing...")
        frame = capture_frame()
        class_id = detect(frame)
        print(f"[SEARCH]   detected: {class_id}")

        if class_id is not None and class_id != BULLSEYE_ID:
            print(f"[SEARCH] Found a valid face: Image ID {class_id}")

            if android_serial is not None:
                message = f"TARGET,{obstacle_number},{class_id}\n"
                android_serial.write(message.encode("ascii"))
                android_serial.flush()
                print(f"[SEARCH] Sent to Android: {message.strip()}")

            send_to_stm(class_id, stm)
            return class_id

        print("[SEARCH]   nothing useful yet, stepping around obstacle...")
        send_stm_command(stm, f"FL{TURN_DEGREES:03d}")
        send_stm_command(stm, f"FW{STEP_FORWARD_CM:03d}")

    print(f"[SEARCH] Gave up after {MAX_ATTEMPTS} attempts -- no valid face found")
    return None


if __name__ == "__main__":
    obstacle_number = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    print(f"Opening {STM_DEVICE}...")
    with serial.Serial(STM_DEVICE, BAUD_RATE, timeout=1) as stm:
        search_for_face(stm, obstacle_number)
