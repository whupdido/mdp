"""
A.5 signal loop: repeatedly capture + detect + send the result to the STM.

Protocol: IM000 means "not found yet" (nothing detected, or only the
bullseye/marker seen) -- the STM keeps rotating. Any other IMxxx means
"found it" -- the STM stops. The rotation itself is the STM's own logic
(per your description); this script's only job is to keep signaling.

Run on the RPi:
    python3 a5_signal_loop.py

Ctrl+C to stop manually. Also stops on its own once a real face (not
IM000) is sent, since at that point the STM should have stopped.
"""

import time

import serial

from capture_and_report import capture_frame, detect, signal_search_result

STM_DEVICE = "/dev/ttyACM0"
BAUD_RATE = 115200
POLL_INTERVAL_SECONDS = 1.0  # how often to capture + check


def main():
    print(f"Opening {STM_DEVICE}...")
    with serial.Serial(STM_DEVICE, BAUD_RATE, timeout=1) as stm:
        while True:
            frame = capture_frame()
            class_id = detect(frame)
            print(f"[LOOP] detected: {class_id}")

            message = signal_search_result(class_id, stm)

            if message != "IM000":
                print(f"[LOOP] Found it ({message}), stopping.")
                break

            time.sleep(POLL_INTERVAL_SECONDS)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[LOOP] Stopped by user")
