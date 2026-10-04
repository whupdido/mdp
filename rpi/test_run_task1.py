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
        item = self.script.pop(0)
        return b"" if item is None else (item + "\n").encode("ascii")

    def write(self, data):
        self.written.append(data.decode("ascii").rstrip("\n"))

    def flush(self):
        pass


class FakeSTM(FakeAndroid):
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

# --- command ACK/status correlation and conservative recovery --------------
def wait_script(script, expected="FW010"):
    stm = FakeSTM(script)
    android = FakeAndroid([])
    original_forward = a1_bridge.forward_stop_if_pending
    original_monotonic = run_task1.time.monotonic
    original_interval = run_task1.STATUS_PROBE_INTERVAL_SECONDS
    original_timeout = a1_bridge.STM_TIMEOUT_SECONDS
    ticks = {"n": 0}

    def clock():
        ticks["n"] += 1
        return float(ticks["n"])

    a1_bridge.forward_stop_if_pending = lambda *_: False
    run_task1.time.monotonic = clock
    run_task1.STATUS_PROBE_INTERVAL_SECONDS = 0
    a1_bridge.STM_TIMEOUT_SECONDS = 20
    try:
        result = run_task1.wait_for_stm_reply(
            stm, android, expected_command=expected, execution_started=True
        )
    finally:
        a1_bridge.forward_stop_if_pending = original_forward
        run_task1.time.monotonic = original_monotonic
        run_task1.STATUS_PROBE_INTERVAL_SECONDS = original_interval
        a1_bridge.STM_TIMEOUT_SECONDS = original_timeout
    return result, stm.written


result, _ = wait_script(["ACK,FL090", "ACK,FW010", "DONE"])
check("only a matching ACK accepts the movement", result, "DONE")
result, _ = wait_script(["ACK,FL090", "DONE", None, "STATUS,IDLE,DONE,FW010"])
check("wrong-command ACK cannot validate an untagged completion", result, "DONE")
result, sent = wait_script(
    [None, "STATUS,IDLE,DONE,FW010#old123", "ACK,FW010#new456", "DONE"],
    expected="FW010#new456",
)
check("request IDs distinguish repeated command text", result, "DONE")
check("repeated command is retried only after old ID is reported", sent.count("FW010#new456"), 1)
result, sent = wait_script([None, "STATUS,IDLE,DONE,FL090", "ACK,FW010", "DONE"])
check("stale status is not treated as current completion", result, "DONE")
check("provably unaccepted command is retried once", sent.count("FW010"), 1)
result, sent = wait_script([None, "STATUS,BUSY,FL090", None,
                            "STATUS,IDLE,DONE,FL090", "ACK,FW010", "DONE"])
check("busy stale movement finishes before retry", result, "DONE")
stale_replies = [None]
for _ in range(5):
    stale_replies.extend(["STATUS,IDLE,DONE,FL090", None])
result, sent = wait_script(stale_replies)
check("stale-result retries stay bounded", sent.count("FW010") <= 2, True)
result, sent = wait_script([None, "ACK,FW010", "DONE"])
check("matching ACK and terminal response complete command", result, "DONE")
result, sent = wait_script([None])
check("ambiguous silence does not resend movement", sent.count("FW010"), 0)

# Preserve the existing explicit reset diagnosis.
result, _ = wait_script(["READY"])
check("unexpected READY remains a reset diagnosis", result, "POSSIBLE_STM_RESET")
original_forward = a1_bridge.forward_stop_if_pending
a1_bridge.forward_stop_if_pending = lambda *_: True
try:
    result = run_task1.wait_for_stm_reply(
        FakeSTM(["ACK"]), FakeAndroid([]), expected_command="FW010#stopcheck"
    )
finally:
    a1_bridge.forward_stop_if_pending = original_forward
check("STOP acknowledgement ends the active route wait", result, "STOPPED")

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
