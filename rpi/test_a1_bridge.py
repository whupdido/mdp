#!/usr/bin/env python3
"""
Zhenxi: offline test for a1_bridge.py.

Runs the bridge with fake serial ports, so you can check the routing without a
Pi, a robot, a Bluetooth radio or pyserial. Nothing here touches hardware.

    python3 rpi/test_a1_bridge.py

Exits non-zero if anything fails. Run it after touching the command patterns —
it is the cheapest way to know the tablet will still be understood.
"""

import importlib.util
import sys
import types
from pathlib import Path

BRIDGE = Path(sys.argv[1] if len(sys.argv) > 1 else "rpi/a1_bridge.py")


class StopTest(Exception):
    """Raised by the fake Android port once its script is exhausted."""


class FakePort:
    """
    `when_empty="stop"` ends the test once the script runs out — that is the
    tablet side. `when_empty="silence"` keeps returning nothing, which is how a
    real serial read behaves when the board says nothing, and is what lets the
    bridge's own timeout actually expire.
    """

    def __init__(self, name, script=None, when_empty="stop"):
        self.name = name
        self.script = list(script or [])
        self.written = []
        self.when_empty = when_empty

    @property
    def in_waiting(self):
        # pyserial's "bytes waiting" -- the bridge uses it to peek at the
        # tablet without blocking while a move is running. Anything still
        # scripted counts as waiting; an exhausted script is a quiet port.
        return len(self.script)

    def readline(self):
        if not self.script:
            if self.when_empty == "silence":
                return b""
            raise StopTest
        item = self.script.pop(0)
        return item if item is None else (item + "\n").encode("ascii")

    def write(self, data):
        self.written.append(data.decode("ascii").rstrip("\n"))

    def flush(self):
        pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


def load_bridge(android_script, stm_script):
    android = FakePort("android", android_script)
    stm = FakePort("stm", stm_script, when_empty="silence")

    fake = types.ModuleType("serial")

    def Serial(device, baud, timeout=None):
        return stm if "ttyACM" in device else android

    fake.Serial = Serial
    fake.SerialException = type("SerialException", (Exception,), {})
    sys.modules["serial"] = fake

    spec = importlib.util.spec_from_file_location("a1_bridge", BRIDGE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.STM_TIMEOUT_SECONDS = 0.3  # keep the no-reply path quick
    return module, android, stm


def run(android_script, stm_script=None):
    module, android, stm = load_bridge(android_script, stm_script)
    try:
        module.main()
    except StopTest:
        pass
    return android.written, stm.written


FAILURES = []


def check(label, actual, expected):
    comparable = actual if any(item.startswith("ROBOT,") for item in expected) else [
        item for item in actual if not item.startswith("ROBOT,")
    ]
    ok = comparable == expected
    print(f"  {'PASS' if ok else 'FAIL'}  {label}")
    if not ok:
        print(f"          expected: {expected}")
        print(f"          actual:   {comparable}")
        FAILURES.append(label)


print(f"exercising {BRIDGE}\n")

# --- greeting ------------------------------------------------------------
to_android, to_stm = run([])
check("greets the tablet on connect", to_android, ["STATUS,RPi bridge ready"])
check("says nothing to the board yet", to_stm, [])

# --- motion command, happy path -----------------------------------------
to_android, to_stm = run(["FW010"], ["DONE"])
check("forwards a motion command to the board", to_stm, ["FW010"])
check(
    "receipts and relays the board reply",
    to_android,
    ["STATUS,RPi bridge ready", "STATUS,SENT,FW010", "STM,DONE"],
)
check(
    "reports the updated robot cell after a confirmed move",
    [item for item in to_android if item.startswith("ROBOT,")],
    ["ROBOT,1,2,N"],
)

# --- every motion verb ---------------------------------------------------
for cmd in ["FW010", "BW100", "FL090", "FR090", "BL090", "BR090", "STOP"]:
    _, to_stm = run([cmd], ["DONE"])
    check(f"{cmd} reaches the board", to_stm, [cmd])

# --- lower case from the tablet -----------------------------------------
_, to_stm = run(["fw010"], ["DONE"])
check("lower case is upper cased before forwarding", to_stm, ["FW010"])

# --- map messages: the actual bug ---------------------------------------
for cmd, echoed in [
    ("ADD,B1,(10,6)", "ADD,B1,(10,6)"),
    ("SUB,B1", "SUB,B1"),
    ("FACE,B2,N", "FACE,B2,N"),
]:
    to_android, to_stm = run([cmd])
    check(f"{cmd} is NOT sent to the board", to_stm, [])
    check(
        f"{cmd} is acknowledged, not rejected",
        to_android,
        ["STATUS,RPi bridge ready", f"STATUS,MAP,{echoed}"],
    )

# --- map messages that look right but aren't ----------------------------
# These match MAP_PATTERN's loose prefix but no real ADD/SUB/FACE shape.
# They used to be acknowledged anyway, so the tablet believed an edit had
# landed that the Pi had actually thrown away.
for cmd in ["ADD,B1,(10)", "ADD,GARBAGE", "SUB,", "FACE,B2,Q", "FACE,B2"]:
    to_android, to_stm = run([cmd])
    check(f"{cmd} is reported as malformed", to_android[-1:], ["ERR,MALFORMED_MAP_MESSAGE"])
    check(f"{cmd} never reaches the board", to_stm, [])

# --- START belongs to run_task1.py, not to this bridge -------------------
# The tablet's START button is the only legal way to begin a Task 1 run, so
# the string exists whenever the app is running -- including in checklist
# demos, when this plain bridge is what is listening. It must not come back
# as ERR: the tablet paints every ERR as a red warning.
to_android, to_stm = run(["START"])
check("START is not refused", to_android[-1:], ["MSG,Bridge only. Start Task 1 from run_task1.py."])
check("START never reaches the board", to_stm, [])

# Task 2's trigger has no runner yet, but it must still not come back as an
# error -- the tablet paints ERR red, and the button is not the thing at fault.
to_android, to_stm = run(["START2"])
check("START2 is not refused either", to_android[-1:], ["MSG,Bridge only. No Task 2 runner yet."])
check("START2 never reaches the board", to_stm, [])

# --- things that should still be refused --------------------------------
for cmd in ["ROBOT,7,2,W", "NONSENSE", "FW10", "FWABC", "FW0100"]:
    to_android, to_stm = run([cmd])
    check(f"{cmd} is refused", to_android[-1:], ["ERR,INVALID_COMMAND"])
    check(f"{cmd} never reaches the board", to_stm, [])

# --- board goes quiet ----------------------------------------------------
to_android, _ = run(["FW010"], [])  # board never answers at all
check("no reply from the board is reported", to_android[-1:], ["STM,NO_REPLY"])

# --- board aborts --------------------------------------------------------
for reply in ["STALL", "TIMEOUT", "BUSY", "ERR"]:
    to_android, _ = run(["FW010"], [reply])
    check(f"board reply {reply} is relayed", to_android[-1:], [f"STM,{reply}"])

# ACK is a STOP acknowledgement, not a movement completion. A delayed ACK
# from a previous STOP must not make the next movement look complete before
# its DONE arrives.
to_android, to_stm = run(
    ["FW010", "FL090"],
    ["DONE", "ACK", "DONE"],
)
check(
    "delayed STOP ACK does not finish the next move",
    to_stm,
    ["FW010", "FL090"],
)
check(
    "next move waits for DONE after delayed STOP ACK",
    [item for item in to_android if item.startswith("STM,")],
    ["STM,DONE", "STM,ACK", "STM,DONE"],
)

# STOP clears commands already queued in the bridge and commands still
# buffered on the Android link. Neither command after STOP may reach STM32.
to_android, to_stm = run(
    ["FW010", "FL090", "STOP", "BR090"],
    [None, "ACK"],
)
check(
    "STOP clears queued and buffered commands",
    to_stm,
    ["FW010", "STOP"],
)
check(
    "STOP queue clear still returns its acknowledgement",
    to_android[-2:],
    ["STATUS,SENT,STOP", "STM,ACK"],
)

# --- board stops short of an obstacle -----------------------------------
# Since the IR sensors went in, control.c cuts a move short rather than hit
# something -- and reports it as a [WARN] line *then* DONE. Both must reach
# the tablet, in that order: Android reads the warning as "position lost"
# and uses it to talk down the DONE that follows.
to_android, _ = run(["FW010"], ["", "[WARN] COLLISION AVOIDED! Stopping early.", "DONE"])
check(
    "collision warning and its DONE are both relayed, in order",
    [item for item in to_android if item.startswith("STM,")],
    ["STM,[WARN] COLLISION AVOIDED! Stopping early.", "STM,DONE"],
)

# With the command.c fix the same stop reports BLOCKED instead of DONE. It
# must end the wait: if the bridge did not know BLOCKED, it would sit until
# STM_TIMEOUT_SECONDS and then tack on a spurious NO_REPLY.
to_android, _ = run(["FW010"], ["", "[WARN] COLLISION AVOIDED! Stopping early.", "BLOCKED"])
check(
    "BLOCKED is relayed and ends the wait",
    to_android[-2:],
    ["STM,[WARN] COLLISION AVOIDED! Stopping early.", "STM,BLOCKED"],
)

# --- STOP while a move is running -----------------------------------------
# The board is silent for one poll (None), during which the tablet's STOP
# arrives. It must reach the board *before* the board's reply, not after.
to_android, to_stm = run(["FW010", "STOP"], [None, "ACK"])
check("STOP mid-move reaches the board at once", to_stm, ["FW010", "STOP"])
check(
    "STOP mid-move is receipted before the board answers",
    to_android,
    ["STATUS,RPi bridge ready", "STATUS,SENT,FW010", "STATUS,SENT,STOP", "STM,ACK"],
)

# --- anything else mid-move waits its turn --------------------------------
to_android, to_stm = run(["FW010", "FL090"], [None, "DONE", "DONE"])
check("a second move sent mid-move is held, not lost", to_stm, ["FW010", "FL090"])
check(
    "and is forwarded only after the first move ends",
    to_android,
    [
        "STATUS,RPi bridge ready",
        "STATUS,SENT,FW010", "STM,DONE",
        "STATUS,SENT,FL090", "STM,DONE",
    ],
)

to_android, to_stm = run(["FW010", "ADD,B1,(10,6)"], [None, "DONE"])
check("a map edit sent mid-move never reaches the board", to_stm, ["FW010"])
check(
    "and is acknowledged once the move ends",
    [item for item in to_android if item.startswith("STM,") or item.startswith("STATUS,MAP,")],
    ["STM,DONE", "STATUS,MAP,ADD,B1,(10,6)"],
)

# --- blank input is ignored ---------------------------------------------
to_android, to_stm = run(["", "   "])
check("blank lines are ignored", to_android, ["STATUS,RPi bridge ready"])
check("blank lines never reach the board", to_stm, [])

print()
if FAILURES:
    print(f"{len(FAILURES)} FAILED: {FAILURES}")
    sys.exit(1)
print("all checks passed")
