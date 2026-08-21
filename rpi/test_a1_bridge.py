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
    ok = actual == expected
    print(f"  {'PASS' if ok else 'FAIL'}  {label}")
    if not ok:
        print(f"          expected: {expected}")
        print(f"          actual:   {actual}")
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

# --- things that should still be refused --------------------------------
for cmd in ["ROBOT,7,2,W", "NONSENSE", "FW10", "FWABC", "FW0100"]:
    to_android, to_stm = run([cmd])
    check(f"{cmd} is refused", to_android[-1:], ["ERR,INVALID_COMMAND"])
    check(f"{cmd} never reaches the board", to_stm, [])

# --- board goes quiet ----------------------------------------------------
to_android, _ = run(["FW010"], [])  # board never answers at all
check("no reply from the board is reported", to_android[-1:], ["STM,NO_REPLY"])

# --- board aborts --------------------------------------------------------
for reply in ["STALL", "TIMEOUT", "BUSY", "ERR", "ACK"]:
    to_android, _ = run(["FW010"], [reply])
    check(f"board reply {reply} is relayed", to_android[-1:], [f"STM,{reply}"])

# --- blank input is ignored ---------------------------------------------
to_android, to_stm = run(["", "   "])
check("blank lines are ignored", to_android, ["STATUS,RPi bridge ready"])
check("blank lines never reach the board", to_stm, [])

print()
if FAILURES:
    print(f"{len(FAILURES)} FAILED: {FAILURES}")
    sys.exit(1)
print("all checks passed")
