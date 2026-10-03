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


def go(script):
    """Run wait_for_go against a script. stdin is never ready, so only the
    tablet can end it -- if START is not honoured this would hang, which is
    why the script is finite and the fake returns b'' forever after."""
    a1_bridge.obstacles.clear()
    android = FakeAndroid(script)
    # select.select must never report stdin ready: this is the competition
    # path, where nobody is allowed to touch the laptop.
    run_task1.select.select = lambda *a, **k: ([], [], [])
    run_task1.time.sleep = lambda _: None

    returned = False
    # wait_for_go loops forever if it never sees START. Cap the attempts by
    # making the script finite and failing loudly if we spin past it.
    guard = {"n": 0}
    real_sleep = run_task1.time.sleep

    def counting_sleep(_):
        guard["n"] += 1
        if guard["n"] > 500:
            raise AssertionError("wait_for_go never returned -- START ignored?")

    run_task1.time.sleep = counting_sleep
    run_task1.wait_for_go(android)
    returned = True
    return android, returned


print("exercising rpi/run_task1.py\n")

# --- the rule: START from the tablet begins the run ----------------------
android, returned = go(["START"])
check("START from the tablet begins the run", returned, True)
check("and the tablet is told", android.written[-1:], ["STATUS,Run starting"])

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
        "STATUS,Run starting",
    ],
)

# --- a malformed edit is still reported, and does not start anything -----
android, _ = go(["FACE,B9,Q", "START"])
check("a malformed edit before START is reported", android.written[:1], ["ERR,MALFORMED_MAP_MESSAGE"])
check("and START still works after it", android.written[-1:], ["STATUS,Run starting"])

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

print()
if FAILURES:
    print(f"{len(FAILURES)} FAILED: {FAILURES}")
    sys.exit(1)
print("all checks passed")
