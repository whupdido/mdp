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

REAL_WAIT_FOR_STM_REPLY = run_task1.wait_for_stm_reply

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


class FakeStm:
    def __init__(self, replies=()):
        self.replies = list(replies)
        self.written = []

    def readline(self):
        if not self.replies:
            return b""
        reply = self.replies.pop(0)
        if reply is None:
            return b""
        return (reply + "\n").encode("ascii")

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
# The hold defaults to zero. It only ever existed to disguise planning that
# happened during setup, and Prof Smitha confirmed that overlap is allowed,
# so there is nothing to disguise and the time is pure cost.
check("the hold is zero by default", run_task1.ARMING_DELAY_SECONDS, 0)

run_task1.time.sleep = lambda _: None
run_task1.preflight = lambda stm, android: True

android = FakeAndroid([])
ok = run_task1.arm_and_wait(None, android)
check("the hold is announced to the tablet", android.written[:1], ["STATUS,PLAN,ARMED,0"])
check("and the robot is declared ready", android.written[-1:], ["STATUS,PLAN,SET"])
check("so START becomes live", ok, True)

# A STOP during the hold must cancel: the robot has not moved yet, and this
# is exactly when someone notices it is pointing the wrong way.
run_task1.ARMING_DELAY_SECONDS = 5
android = FakeAndroid(["STOP"])
ok = run_task1.arm_and_wait(None, android)
check("STOP during the hold cancels the run", ok, False)
check("and says so", android.written[-1:], ["MSG,Run cancelled before moving"])
run_task1.ARMING_DELAY_SECONDS = 0

# --- a mid-move STOP's ACK ends the wait cleanly ------------------------
android = FakeAndroid(["STOP"])
stm = FakeStm(["ACK"])
reply = REAL_WAIT_FOR_STM_REPLY(stm, android)
check("a mid-move STOP reaches the STM", stm.written, ["STOP"])
check("the STOP acknowledgement is terminal", reply, "STOPPED")
check("the STOP acknowledgement is relayed", android.written[-1:], ["STM,ACK"])

# An unrelated/delayed ACK must still not complete a normal movement.
android = FakeAndroid([])
stm = FakeStm(["ACK", "DONE"])
reply = REAL_WAIT_FOR_STM_REPLY(stm, android)
check("an ACK without a forwarded STOP is ignored", reply, "DONE")

# A status probe distinguishes an in-progress move from a recovered terminal
# result. STATUS,BUSY is not the same as a rejected command's bare BUSY.
android = FakeAndroid([])
stm = FakeStm(["STATUS,BUSY,FW050", "STATUS,IDLE,DONE,FW050"])
reply = REAL_WAIT_FOR_STM_REPLY(stm, android, expected_command="FW050")
check("STATUS,BUSY keeps waiting for movement completion", reply, "DONE")

# If the original DONE is lost, silence triggers '?' and the retained STM
# result completes the transaction without repeating the movement command.
real_monotonic = run_task1.time.monotonic
clock = {"now": 0.0}


def advancing_monotonic():
    clock["now"] += 1.0
    return clock["now"]


run_task1.time.monotonic = advancing_monotonic
android = FakeAndroid([])
stm = FakeStm([None, "STATUS,IDLE,DONE,FW050"])
reply = REAL_WAIT_FOR_STM_REPLY(stm, android, expected_command="FW050")
run_task1.time.monotonic = real_monotonic
check("silence triggers a non-moving status probe", stm.written, ["?"])
check("a retained DONE recovers the lost terminal line", reply, "DONE")

# A DONE can arrive after '?' was sent but before the STM consumes the probe.
# Drain the matching status response before the caller sends another move.
clock["now"] = 0.0
run_task1.time.monotonic = advancing_monotonic
android = FakeAndroid([])
stm = FakeStm([None, "DONE", "STATUS,IDLE,DONE,FW030"])
reply = REAL_WAIT_FOR_STM_REPLY(stm, android, expected_command="FW030")
run_task1.time.monotonic = real_monotonic
check("DONE after a probe still completes the move", reply, "DONE")
check("the outstanding probe response is drained before returning", stm.replies, [])

# Legacy firmware may never provide a STATUS line. The short settling window
# falls back to the valid terminal reply rather than becoming NO_REPLY.
clock["now"] = 0.0
run_task1.time.monotonic = advancing_monotonic
android = FakeAndroid([])
stm = FakeStm([None, "DONE", None])
reply = REAL_WAIT_FOR_STM_REPLY(stm, android, expected_command="FW030")
run_task1.time.monotonic = real_monotonic
check("a legacy terminal reply survives the probe settling window", reply, "DONE")

# A probe that still reports an older command proves the current command was
# dropped before dispatch, so retrying it is safe and bounded.
clock["now"] = 0.0
run_task1.time.monotonic = advancing_monotonic
android = FakeAndroid([])
stm = FakeStm([None, "STATUS,IDLE,DONE,FL045", "ACK,BW010", "DONE"])
reply = REAL_WAIT_FOR_STM_REPLY(stm, android, expected_command="BW010")
run_task1.time.monotonic = real_monotonic
check("a provably dropped movement is retried", stm.written, ["?", "BW010"])
check("the retried movement waits for completion", reply, "DONE")
check(
    "the tablet is told about the retry",
    "STATUS,RETRY,BW010,2" in android.written,
    True,
)

# Staggered deployment is safe: older firmware answers '?' with bare BUSY.
clock["now"] = 0.0
run_task1.time.monotonic = advancing_monotonic
android = FakeAndroid([])
stm = FakeStm([None, "BUSY", "DONE"])
reply = REAL_WAIT_FOR_STM_REPLY(stm, android)
run_task1.time.monotonic = real_monotonic
check("a legacy probe BUSY does not abort the active move", reply, "DONE")

# --- Task 1 image results belong to Android, not the STM -----------------
capture_calls = []
reply_waits = []


def fake_report_obstacle(obstacle_number, android_serial, stm_serial=None, **kwargs):
    capture_calls.append({
        "obstacle_number": obstacle_number,
        "android_serial": android_serial,
        "stm_serial": stm_serial,
        **kwargs,
    })
    return 16


run_task1.report_obstacle = fake_report_obstacle
run_task1.wait_for_stm_reply = lambda *args, **kwargs: reply_waits.append((args, kwargs)) or "NO_REPLY"
android = FakeAndroid([])
stm = FakeStm()
run_task1.run_route(
    [{"type": "capture", "obstacle_id": 2}],
    stm,
    android,
    run_id="test-run",
)
check("Task 1 capture does not receive the STM connection", capture_calls[0]["stm_serial"], None)
check("Task 1 capture sends no IMxxx command", stm.written, [])
check("Task 1 capture waits for no STM image acknowledgement", reply_waits, [])
check("the route continues after reporting the image", android.written[-1:], ["MSG,Task 1 route complete"])

# --- safety policy after probe recovery is exhausted --------------------
run_task1.wait_for_stm_reply = lambda *args, **kwargs: "NO_REPLY"
android = FakeAndroid([])
stm = FakeStm()
run_task1.run_route(
    [
        {"type": "move", "command": "BL030"},
        {"type": "move", "command": "FW010"},
    ],
    stm,
    android,
)
check("no command follows an unrecovered NO_REPLY", stm.written, ["BL030"])
check("NO_REPLY is surfaced as an early end", android.written[-1:], ["MSG,Run ended early: NO_REPLY"])

# --- a requested STOP ends the route instead of sending the next move ----
run_task1.wait_for_stm_reply = lambda *args, **kwargs: "STOPPED"
android = FakeAndroid([])
stm = FakeStm()
run_task1.run_route(
    [
        {"type": "move", "command": "BW010"},
        {"type": "move", "command": "FL060"},
    ],
    stm,
    android,
)
check("no command is sent after a requested STOP", stm.written, ["BW010"])
check("a requested STOP does not become NO_REPLY", android.written[-1:], ["MSG,Task 1 stopped"])

print()
if FAILURES:
    print(f"{len(FAILURES)} FAILED: {FAILURES}")
    sys.exit(1)
print("all checks passed")
