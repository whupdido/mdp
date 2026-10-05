#!/usr/bin/env python3
"""
Zhenxi: offline test for run_task1.py's start trigger.

The competition rules say a Task 1 run must be started from a button on the
Android tablet and nothing else. wait_for_go() used to block on Enter at the
SSH terminal, which is the laptop. This pins the replacement.

    python3 rpi/test_run_task1.py

No Pi, no radio, no pyserial.
"""

import sys
import types
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

# Stub the hardware libs before anything imports them.
serial = types.ModuleType("serial")
serial.Serial = lambda *a, **k: None
serial.SerialException = type("SerialException", (Exception,), {})
sys.modules.setdefault("serial", serial)
cv2 = types.ModuleType("cv2")
cv2.VideoCapture = lambda *a, **k: None
sys.modules.setdefault("cv2", cv2)

import a1_bridge
import run_task1

FAILURES = []


def check(label, actual, expected):
    ok = actual == expected
    print(f"  {'PASS' if ok else 'FAIL'}  {label}")
    if not ok:
        print(f"          expected: {expected}")
        print(f"          actual:   {actual}")
        FAILURES.append(label)


class FakeAndroid:
    """Serial-ish port that plays a script, then goes quiet."""

    def __init__(self, script):
        self.script = list(script)
        self.written = []

    @property
    def in_waiting(self):
        return len(self.script)

    def readline(self):
        if not self.script:
            return b""
        return (self.script.pop(0) + "\n").encode("ascii")

    def write(self, data):
        self.written.append(data.decode("ascii").rstrip("\n"))

    def flush(self):
        pass


def drive(script, trigger):
    """Run wait_for() against a script, as the competition path: stdin is
    never ready, so only the tablet can end it."""
    android = FakeAndroid(script)
    run_task1.select.select = lambda *a, **k: ([], [], [])
    guard = {"n": 0}

    def counting_sleep(_):
        guard["n"] += 1
        if guard["n"] > 500:
            raise AssertionError(f"wait_for never saw {trigger}")

    run_task1.time.sleep = counting_sleep
    run_task1.wait_for(android, trigger, "test")
    return android


def go(script, trigger="START"):
    """Run wait_for() against a script with a clean obstacle map. stdin is
    never ready, so only the tablet can end it -- if the trigger is not
    honoured this would hang, which is why the guard below fails loudly."""
    a1_bridge.obstacles.clear()
    android = drive(script, trigger)
    return android, True


print("exercising rpi/run_task1.py\n")

# --- the rule: START from the tablet begins the run ----------------------
android, returned = go(["START"])
check("START from the tablet begins the run", returned, True)

# --- obstacles keyed in before START still land --------------------------
android, _ = go(["ADD,B1,(5,13)", "FACE,B1,W", "ADD,B2,(5,7)", "FACE,B2,S", "START"])
check("obstacles keyed in before START are recorded", a1_bridge.obstacles, {
    1: {"pos": (5, 13), "face": "W"},
    2: {"pos": (5, 7), "face": "S"},
})
check(
    "each map edit is acknowledged, then the run starts",
    android.written,
    [
        "STATUS,MAP,ADD,B1,(5,13)",
        "STATUS,MAP,FACE,B1,W",
        "STATUS,MAP,ADD,B2,(5,7)",
        "STATUS,MAP,FACE,B2,S",
    ],
)

# --- a malformed edit is still reported, and does not start anything -----
android, _ = go(["FACE,B9,Q", "START"])
check("a malformed edit before START is reported", android.written[:1], ["ERR,MALFORMED_MAP_MESSAGE"])
check("and START still ends the wait after it", returned, True)

# --- the payload the planner receives ------------------------------------
a1_bridge.obstacles.clear()
a1_bridge.obstacles.update({
    1: {"pos": (5, 13), "face": "W"},
    2: {"pos": (5, 7), "face": None},     # face never set
    3: {"pos": None, "face": "E"},        # position never set
})
check(
    "obstacles missing a face or position are dropped from the plan",
    run_task1.obstacles_payload(),
    [{"id": 1, "x": 5, "y": 13, "face": "W"}],
)

# --- the start pose is dynamic ------------------------------------------
# The planner used to assume (1,1,N). The robot starts in the carpark, but
# which cell and which way round is the supervisor's call on the day.
a1_bridge.obstacles.clear()
run_task1.start_pose.update({"x": 1, "y": 1, "face": "N"})
android = drive(["ROBOT,2,2,E", "COMPUTE"], "COMPUTE")
check("the tablet's start pose is recorded", dict(run_task1.start_pose),
      {"x": 2, "y": 2, "face": "E"})
check("and acknowledged like any map edit", android.written[:1], ["STATUS,MAP,ROBOT,2,2,E"])

check("a malformed start pose is not mistaken for one",
      run_task1.handle_start_pose("ROBOT,2,2,Q"), False)
check("nor is a bare ROBOT", run_task1.handle_start_pose("ROBOT"), False)

# --- COMPUTE and START are separate presses ------------------------------
a1_bridge.obstacles.clear()
android = drive(["ADD,B1,(5,13)", "FACE,B1,W", "COMPUTE"], "COMPUTE")
check("obstacles keyed in before COMPUTE are recorded",
      a1_bridge.obstacles, {1: {"pos": (5, 13), "face": "W"}})
check("COMPUTE is not swallowed by the map pump", android.written, [
    "STATUS,MAP,ADD,B1,(5,13)", "STATUS,MAP,FACE,B1,W",
])

android = drive(["START"], "START")
check("START ends its own wait", android.written, [])

# --- the arming countdown ------------------------------------------------
# The Pi waits before driving so the team can step back. It announces how
# long so the tablet can count the same number down.
run_task1.ARMING_DELAY_SECONDS = 0
android = FakeAndroid([])
run_task1.time.sleep = lambda _: None
ok = run_task1.arm_and_wait(android)
check("the arming delay is announced to the tablet", android.written[:1],
      ["STATUS,PLAN,ARMED,0"])
check("and the run proceeds when it expires", ok, True)

# A STOP during the countdown must cancel: the robot has not moved yet, and
# this is exactly when someone notices it is pointing the wrong way.
run_task1.ARMING_DELAY_SECONDS = 5
android = FakeAndroid(["STOP"])
ok = run_task1.arm_and_wait(android)
check("STOP during the countdown cancels the run", ok, False)
check("and says so", android.written[-1:], ["MSG,Run cancelled before moving"])

print()
if FAILURES:
    print(f"{len(FAILURES)} FAILED: {FAILURES}")
    sys.exit(1)
print("all checks passed")
