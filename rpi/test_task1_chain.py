#!/usr/bin/env python3
"""
Zhenxi: end-to-end test of the Task 1 chain, with no hardware.

The per-file tests each prove one piece. This proves the pieces fit: what
the Android app actually sends goes in one end, and what the STM32 actually
receives comes out the other, with the planner and the camera stubbed.

    python3 rpi/test_task1_chain.py

Covers the whole sequence the rules describe:

  prep       tablet sends the start pose and the obstacles
  press 1    SETUP -> COMPUTE -> plan -> STATUS,PLAN,READY,<moves>
  press 2    PLAN  -> ARM -> pre-flight -> STATUS,PLAN,SET
  press 3    START -> drive
  during     TARGET,<n>,<id>,<face> per obstacle, live on the map
  end        route complete, robot stopped by itself

Exits non-zero if anything fails.
"""

import sys
import types
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

# Hardware libraries, stubbed before anything imports them.
_serial = types.ModuleType("serial")
_serial.Serial = lambda *a, **k: None
_serial.SerialException = type("SerialException", (Exception,), {})
sys.modules.setdefault("serial", _serial)
_cv2 = types.ModuleType("cv2")
_cv2.VideoCapture = lambda *a, **k: None
sys.modules.setdefault("cv2", _cv2)

import a1_bridge
import algo_client
import capture_and_report
import run_task1

FAILURES = []


def check(label, actual, expected):
    ok = actual == expected
    print(f"  {'PASS' if ok else 'FAIL'}  {label}")
    if not ok:
        print(f"          expected: {expected}")
        print(f"          actual:   {actual}")
        FAILURES.append(label)


def check_contains(label, haystack, needle):
    ok = needle in haystack
    print(f"  {'PASS' if ok else 'FAIL'}  {label}")
    if not ok:
        print(f"          missing: {needle}")
        print(f"          from:    {haystack}")
        FAILURES.append(label)


class Port:
    """A serial-ish port that plays a script and records what was written."""

    def __init__(self, script=None):
        self.script = list(script or [])
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


class Board(Port):
    """The STM32. Answers every motion command the way the firmware does.

    `fail_on` makes the nth command answer with something other than DONE,
    which is how the recovery paths get exercised.
    """

    def __init__(self, fail_on=None, reply="BLOCKED"):
        super().__init__()
        self.fail_on = fail_on
        self.reply = reply
        self.pending = []
        self.moves = 0

    def write(self, data):
        command = data.decode("ascii").rstrip("\n")
        self.written.append(command)
        self.moves += 1
        if self.fail_on == self.moves:
            # Real firmware prints its reason before the code. Anything
            # reading this port has to skip the chatter.
            self.pending.append("[WARN] COLLISION AVOIDED! Stopping early.")
            self.pending.append(self.reply)
        else:
            self.pending.append("DONE")

    def readline(self):
        if self.pending:
            return (self.pending.pop(0) + "\n").encode("ascii")
        return b""


def install_fakes(plans, detections):
    """Stub the laptop planner and the camera. `plans` is popped per call."""
    calls = {"plan": [], "capture": []}

    def plan_route(obstacles, start=None):
        calls["plan"].append({"obstacles": list(obstacles), "start": start})
        return plans.pop(0) if plans else None

    def report_obstacle(obstacle_number, android_serial, stm_serial=None, face=None, **_ignored):
        calls["capture"].append(obstacle_number)
        target = detections.get(obstacle_number)
        if target is None:
            return None
        known = a1_bridge.obstacles.get(obstacle_number, {}).get("face")
        line = f"TARGET,{obstacle_number},{target}"
        if known:
            line += f",{known}"
        a1_bridge.send_line(android_serial, line)
        return target

    algo_client.plan_route = plan_route
    algo_client.server_reachable = lambda timeout=2.0: True
    run_task1.algo_client = algo_client
    run_task1.report_obstacle = report_obstacle
    capture_and_report.report_obstacle = report_obstacle
    return calls


def fresh():
    a1_bridge.obstacles.clear()
    a1_bridge.inbox.clear()
    run_task1.start_pose.update({"x": 1, "y": 1, "face": "N"})
    run_task1.select.select = lambda *a, **k: ([], [], [])
    run_task1.ARMING_DELAY_SECONDS = 0

    # wait_for() spins until its trigger arrives. If one is ever missed we
    # want a failure that names it, not a test that hangs.
    spins = {"n": 0}

    def guarded_sleep(_):
        spins["n"] += 1
        if spins["n"] > 2000:
            raise AssertionError("wait_for never saw its trigger -- press swallowed?")

    run_task1.time.sleep = guarded_sleep


def move(cmd):
    return {"type": "move", "command": cmd}


def capture(n):
    return {"type": "capture", "obstacle_id": n}


# The five-obstacle layout from the rules PDF, as the tablet sends it.
PREP = [
    "ROBOT,2,2,E",
    "ADD,B1,(5,13)", "FACE,B1,W",
    "ADD,B2,(5,7)", "FACE,B2,S",
    "ADD,B3,(12,9)", "FACE,B3,E",
]

print("exercising the Task 1 chain\n")

# =====================================================================
# 1. Preparation, then PLAN ROUTE, then START -- the whole happy path
# =====================================================================
fresh()
route = [move("FW030"), capture(1), move("FL090"), capture(2), move("FW020"), capture(3)]
calls = install_fakes([route], {1: 35, 2: 17, 3: 31})

android = Port(PREP + ["COMPUTE", "ARM", "START"])
board = Board()
run_task1.wait_for(android, "COMPUTE", "prep")

check("the tablet's start pose reached the runner",
      dict(run_task1.start_pose), {"x": 2, "y": 2, "face": "E"})
check("all three obstacles reached the runner",
      run_task1.obstacles_payload(),
      [{"id": 1, "x": 5, "y": 13, "face": "W"},
       {"id": 2, "x": 5, "y": 7, "face": "S"},
       {"id": 3, "x": 12, "y": 9, "face": "E"}])

steps = algo_client.plan_route(run_task1.obstacles_payload(), start=dict(run_task1.start_pose))
check("the planner is asked to route from the tablet's pose, not (1,1,N)",
      calls["plan"][0]["start"], {"x": 2, "y": 2, "face": "E"})

# Press 2: PLAN. Pre-flight the robot, in the prep window where time is free.
run_task1.wait_for(android, "ARM", "arm")
preflight_board = Board()
check("pre-flight passes when the board and the laptop answer",
      run_task1.arm_and_wait(preflight_board, android), True)
check("pre-flight asks the board without moving it, then zeroes the gyro",
      preflight_board.written, ["FW000", "GC"])
check("the tablet is told the robot is checked and ready",
      [w for w in android.written if w.startswith("STATUS,PLAN,")],
      ["STATUS,PLAN,CHECKING", "STATUS,PLAN,ARMED,0", "STATUS,PLAN,SET"])

# Press 3: START.
run_task1.wait_for(android, "START", "go")
android.written.clear()
run_task1.run_route(steps, board, android)

check("every planned move reached the board", board.written, ["FW030", "FL090", "FW020"])
check("every obstacle was photographed", calls["capture"], [1, 2, 3])
for n, target, face in [(1, 35, "W"), (2, 17, "S"), (3, 31, "E")]:
    check_contains(f"obstacle {n} reported to the tablet with its face",
                   android.written, f"TARGET,{n},{target},{face}")
check_contains("the tablet is told the route finished",
               android.written, "MSG,Task 1 route complete")

# =====================================================================
# 2. A blocked move must not throw away the rest of the run
# =====================================================================
# Each image is ten points. Giving up at obstacle 1 of 3 used to cost the
# other two.
print()
fresh()
first = [move("FW030"), capture(1), move("FL090"), capture(2), move("FW020"), capture(3)]
after = [move("FW010"), capture(2), move("FW010"), capture(3)]
calls = install_fakes([after], {1: 35, 2: 17, 3: 31})
for line in PREP:
    run_task1.pump_map(Port(), line)

android = Port()
# Third command written is the FW020, i.e. after obstacles 1 and 2 are
# already photographed and only 3 is left.
board = Board(fail_on=3)
run_task1.run_route(first, board, android)

check("the run continued past the blocked move and got all three",
      calls["capture"], [1, 2, 3])
check_contains("it backed off before re-planning", board.written, "BW010")
check("the re-plan only covered the obstacle still to do",
      [o["id"] for o in calls["plan"][-1]["obstacles"]], [3])
check_contains("and the tablet was told why", android.written, "MSG,Re-planning for 1 obstacle(s)")
check_contains("the route still finished", android.written, "MSG,Task 1 route complete")

# The re-planned route above also lists obstacle 2, which was already read
# before the block. A second reading is not a free retry -- it can detect
# something different, and a wrong image ID is minus ten points (FAQ 4).
check("an obstacle already photographed is not read again",
      [n for n in calls["capture"] if n == 2], [2])

# =====================================================================
# 3. A wedged robot must stop, not keep driving
# =====================================================================
print()
fresh()
calls = install_fakes([], {1: 35, 2: 17, 3: 31})
for line in PREP:
    run_task1.pump_map(Port(), line)

android = Port()
board = Board(fail_on=1, reply="STALL")
run_task1.run_route([move("FW030"), capture(1), move("FW030"), capture(2)], board, android)

check("a stall stops the route", board.written, ["FW030"])
check("nothing was photographed after it", calls["capture"], [])
check_contains("and the tablet is told", android.written, "MSG,Run ended early: STALL")

# =====================================================================
# 4. Repeated blocking gives up rather than thrashing
# =====================================================================
print()
fresh()
loop = [move("FW030"), capture(1)]
calls = install_fakes([list(loop), list(loop), list(loop)], {1: 35})
for line in PREP:
    run_task1.pump_map(Port(), line)

android = Port()


class AlwaysBlocks(Board):
    def write(self, data):
        command = data.decode("ascii").rstrip("\n")
        self.written.append(command)
        self.pending.append("DONE" if command.startswith("BW") else "BLOCKED")


board = AlwaysBlocks()
run_task1.run_route(list(loop), board, android)
check("re-planning is bounded", len(calls["plan"]), run_task1.MAX_REPLANS)
check_contains("and it ends by saying so", android.written, "MSG,Run ended early: BLOCKED")

# =====================================================================
# 5. A press that lands during pre-flight must not be lost
# =====================================================================
# Waiting on the board drains Android into a1_bridge.inbox, so a START
# pressed while pre-flight is still running never reaches the port. The two
# presses are seconds apart, so an impatient operator hits this every time
# -- and the symptom is simply that the button does nothing.
print()
fresh()
install_fakes([], {})
algo_client.server_reachable = lambda timeout=2.0: True

android = Port(["START"])            # pressed early, during the pre-flight
board = Board()
run_task1.arm_and_wait(board, android)
check("the early press was taken off the port by the pre-flight",
      a1_bridge.inbox, ["START"])
check("and wait_for still finds it", run_task1.wait_for(android, "START", "go"), True)
check("without leaving it in the queue", a1_bridge.inbox, [])

# Same timing, wrong task: switch to Task 2 and press START while the Task 1
# pre-flight is still waiting on the board, and START2 lands in the queue
# rather than on the port. It has to produce the same "wrong runner" notice
# either way -- otherwise whether you get told anything depends on how fast
# you pressed, which is the worst kind of bug to debug on the day.
fresh()
android = Port()
run_task1.handle_other(android, "START2")
check("a non-trigger START2 is answered, not dropped", android.written,
      ["MSG,This is the Task 1 runner. Run a1_bridge.py for Task 2."])

# And through wait_for itself, from the queue, with the real trigger behind it.
fresh()
android = Port(["START"])
a1_bridge.inbox.append("START2")
check("wait_for answers a queued START2 and still waits for its own trigger",
      run_task1.wait_for(android, "START", "go"), True)
check("the notice went out", android.written,
      ["MSG,This is the Task 1 runner. Run a1_bridge.py for Task 2."])

# =====================================================================
# 6. Pre-flight catches a robot that is not fit to run
# =====================================================================
# The whole point of doing this on the PLAN press: find out in the prep
# window, not when the clock is running and the run is already lost.
print()
fresh()
install_fakes([], {})
algo_client.server_reachable = lambda timeout=2.0: True

# A board that is genuinely dead says nothing to anything -- not the move,
# not the status probes, not a resend. (Kenneth's wait_for_stm_reply now
# probes and resends a command that went unanswered, so a board that was
# only silent once recovers; this used to model that, and passed for the
# wrong reason until the recovery landed.)
class DeadBoard(Port):
    def write(self, data):
        self.written.append(data.decode("ascii").rstrip("\n"))

    def readline(self):
        return b""


android = Port()
patience = a1_bridge.STM_TIMEOUT_SECONDS
a1_bridge.STM_TIMEOUT_SECONDS = 0.5          # 25 s on the robot; not in a test
dead = DeadBoard()
check("a board that never answers fails pre-flight",
      run_task1.arm_and_wait(dead, android), False)
check_contains("and the tablet is told why", android.written,
               "STATUS,PLAN,FAILED,board answered NO_REPLY")
a1_bridge.STM_TIMEOUT_SECONDS = patience

# And the recovery itself: silent once, then awake. Pre-flight should pass,
# because a single dropped line is what the resend exists to absorb.
android = Port()
check("a board that drops one reply still passes pre-flight",
      run_task1.arm_and_wait(Board(fail_on=1, reply="NO_REPLY"), android), True)

android = Port()
algo_client.server_reachable = lambda timeout=2.0: False
check("an unreachable laptop fails pre-flight",
      run_task1.arm_and_wait(Board(), android), False)
check_contains("and says so plainly", android.written,
               "STATUS,PLAN,FAILED,laptop algo server unreachable")
algo_client.server_reachable = lambda timeout=2.0: True

# =====================================================================
# 7. Ghost obstacles: the Pi must hold exactly what is on screen
# =====================================================================
# The Pi only forgot an obstacle on SUB, and the tablet's Undo, Clear and
# Demo never send one. Key in five, press Clear, key in three: the planner
# still routed to all five. SETUP now sends CLEAR before the full map.
print()
fresh()
install_fakes([], {})
quiet = Port()
first = [(5, 13, "W"), (5, 7, "S"), (12, 9, "E"), (15, 15, "S"), (15, 4, "N")]
for n, (x, y, f) in enumerate(first, 1):
    run_task1.pump_map(quiet, f"ADD,B{n},({x},{y})")
    run_task1.pump_map(quiet, f"FACE,B{n},{f}")
# Clear on the tablet; then SETUP publishes CLEAR and the three that remain.
run_task1.pump_map(quiet, "CLEAR")
for n, (x, y, f) in enumerate([(3, 10, "E"), (10, 3, "N"), (17, 17, "S")], 1):
    run_task1.pump_map(quiet, f"ADD,B{n},({x},{y})")
    run_task1.pump_map(quiet, f"FACE,B{n},{f}")
check("after CLEAR the Pi plans for exactly what is on screen",
      [o["id"] for o in run_task1.obstacles_payload()], [1, 2, 3])
check_contains("and CLEAR is acknowledged like any map edit", quiet.written, "STATUS,MAP,CLEAR")

# =====================================================================
# 8. SETUP works from every stage, so the tablet can always go back
# =====================================================================
# The runner used to be three loops in a row and could only go forwards.
# PLAN AGAIN, a map edit after planning, or a failed pre-flight all send
# COMPUTE while it was waiting for ARM or START -- ignored, and the tablet
# froze on "PLANNING...".
print()


def prepared(script, plans, board=None):
    fresh()
    calls = install_fakes(plans, {})
    algo_client.server_reachable = lambda timeout=2.0: True
    for line in PREP:
        run_task1.pump_map(Port(), line)
    android = Port(script)
    steps = run_task1.prepare(board or Board(), android)
    return steps, calls, android


route_a = [move("FW030"), capture(1)]
route_b = [move("FW050"), capture(1), move("FL090"), capture(2)]

# PLAN AGAIN after the robot was already checked and ready.
steps, calls, android = prepared(["COMPUTE", "ARM", "COMPUTE", "ARM", "START"], [route_a, route_b])
check("SETUP after PLAN is accepted and re-plans", len(calls["plan"]), 2)
check("and the run drives the new route, not the old one", steps, route_b)

# A failed pre-flight, then the tablet's fallback to SETUP. The board refuses
# the first check only, so the second attempt can complete and reach START.
flaky = Board(fail_on=1, reply="ERR")
steps, calls, android = prepared(
    ["COMPUTE", "ARM", "COMPUTE", "ARM", "START"], [route_a, route_b], board=flaky)
check_contains("a failed pre-flight is reported", android.written,
               "STATUS,PLAN,FAILED,board answered ERR")
check("and SETUP straight after it is accepted, not ignored", len(calls["plan"]), 2)
check("so the run can still go ahead on the fresh route", steps, route_b)

# =====================================================================
# 9. A press the Pi is not ready for gets an answer, not silence
# =====================================================================
# The tablet only sends PLAN or START once it believes the earlier steps
# happened -- so this means the two fell out of step, usually because the
# runner was restarted. Ignoring it left the tablet on "CHECKING..." for good.
print()
for early in ("ARM", "START"):
    fresh()
    android = Port()
    run_task1.handle_other(android, early)
    check(f"{early} with no route sends the tablet back to SETUP", android.written,
          ["STATUS,PLAN,FAILED,the Pi has no route yet - press SETUP"])

print()
if FAILURES:
    print(f"{len(FAILURES)} FAILED: {FAILURES}")
    sys.exit(1)
print("all checks passed")
