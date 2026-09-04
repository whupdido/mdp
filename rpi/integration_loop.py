"""
Full integration loop: a1_bridge.py's Android<->STM relay, plus automatic
obstacle search+report the moment a target face becomes known.

This does NOT duplicate a1_bridge.py's relay logic -- it calls a1_bridge's
own, already-tested main() with a callback hook, so the Android<->STM
behaviour Zhenxi's test_a1_bridge.py exercises stays exactly the same.

Flow:
  1. Android sends ADD,B<n>,(x,y) then FACE,B<n>,<D> -- a1_bridge.py already
     records both into a1_bridge.obstacles.
  2. The moment an obstacle's face becomes known for the first time, this
     script assumes the robot is already positioned in front of it (same
     manual-setup pattern as the A.5 demo: place the robot, then trigger)
     and runs a5_search.search_for_face() -- which drives/searches/detects
     and reports to both Android and the STM.
  3. Everything else (motion command relay, STM replies) behaves exactly
     as a1_bridge.py normally does.

Run on the RPi:
    python3 integration_loop.py
"""

import sys

import serial

import a1_bridge
from a5_search import search_for_face


def on_face_known(stm, android, obstacle_number):
    print(f"[INTEGRATION] Obstacle {obstacle_number} face known -- starting search")
    try:
        search_for_face(stm, obstacle_number, android_serial=android)
    except Exception as exc:
        # A failed search shouldn't take down the whole bridge -- motion
        # commands and other obstacles still need to keep working.
        print(f"[INTEGRATION] search_for_face failed for obstacle {obstacle_number}: {exc}")


if __name__ == "__main__":
    try:
        a1_bridge.main(on_face_known=on_face_known)
    except KeyboardInterrupt:
        print("\nIntegration loop stopped")
    except serial.SerialException as exc:
        print(f"Serial error: {exc}", file=sys.stderr)
        sys.exit(1)
