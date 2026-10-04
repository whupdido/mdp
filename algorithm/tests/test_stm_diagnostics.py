import importlib.util
import json
import sys
from pathlib import Path
from types import ModuleType


def load_task1_runner(monkeypatch):
    serial = ModuleType("serial")
    serial.SerialException = type("SerialException", (Exception,), {})
    monkeypatch.setitem(sys.modules, "serial", serial)
    bridge = ModuleType("a1_bridge")
    bridge.STM_TIMEOUT_SECONDS = 0.1
    bridge.FINAL_REPLIES = {"DONE", "STALL", "TIMEOUT", "BUSY", "ERR", "BLOCKED"}
    sent = []
    bridge.forward_stop_if_pending = lambda *_args: None
    bridge.send_line = lambda _port, line: sent.append(line)
    bridge.send_pose = lambda *_args: None
    class PoseTracker:
        def apply(self, _command):
            pass
    bridge.PoseTracker = PoseTracker
    monkeypatch.setitem(sys.modules, "a1_bridge", bridge)
    client = ModuleType("algo_client")
    client.plan_route = lambda *_args: None
    monkeypatch.setitem(sys.modules, "algo_client", client)
    capture = ModuleType("capture_and_report")
    capture.report_obstacle = lambda *_args, **_kwargs: None
    monkeypatch.setitem(sys.modules, "capture_and_report", capture)
    path = Path(__file__).parents[2] / "rpi" / "run_task1.py"
    spec = importlib.util.spec_from_file_location("task1_diagnostics_under_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, sent


def test_unexpected_ready_is_reported_as_possible_reset_and_kept_in_log(monkeypatch, capsys):
    runner, sent = load_task1_runner(monkeypatch)

    class STM:
        def readline(self):
            return b"READY\n"

    class Android:
        pass

    lines = []
    first = []
    reply = runner.wait_for_stm_reply(STM(), Android(), lines=lines,
                                      on_first_response=lambda: first.append(True),
                                      execution_started=True)
    assert reply == "POSSIBLE_STM_RESET"
    assert lines == ["READY"]
    assert first == [True]
    assert "MSG,POSSIBLE STM RESET" in sent
    runner._log_movement(4, 123.5, "FW030", lines, 0.01, reply, 0.02, None)
    payload = json.loads(capsys.readouterr().out.split("[TASK1 STM DIAGNOSTIC] ", 1)[1].splitlines()[-1])
    assert payload["sequence"] == 4
    assert payload["monotonic_send_timestamp"] == 123.5
    assert payload["raw_command"] == "FW030"
    assert payload["stm_lines"] == ["READY"]
    assert payload["unexpected_ready"] is True
    assert payload["terminal_response"] == "POSSIBLE_STM_RESET"


def test_unexpected_ready_does_not_turn_into_no_reply_after_reset_warning(monkeypatch, capsys):
    runner, _sent = load_task1_runner(monkeypatch)

    class STM:
        def __init__(self):
            self.calls = 0

        def readline(self):
            self.calls += 1
            return b"READY\n" if self.calls == 1 else b""

    lines = []
    reply = runner.wait_for_stm_reply(STM(), object(), lines=lines, execution_started=True)
    output = capsys.readouterr().out
    assert reply == "POSSIBLE_STM_RESET"
    assert lines == ["READY"]
    assert "POSSIBLE STM RESET" in output
    assert "NO_REPLY" not in output


def test_route_logs_transmitted_command_and_stops_without_retry_after_unexpected_ready(monkeypatch, capsys):
    runner, _sent = load_task1_runner(monkeypatch)

    class STM:
        sent = []
        def write(self, payload):
            self.sent.append(payload)
        def readline(self):
            return b"READY\n"

    stm = STM()
    runner.a1_bridge.send_line = lambda port, line: port.write(line.encode()) if port is stm else None
    runner.run_route([{"type": "move", "command": "FW030"},
                      {"type": "move", "command": "FW030"}], stm, object())
    output = capsys.readouterr().out
    diagnostic = json.loads(output.split("[TASK1 STM DIAGNOSTIC] ", 1)[1].splitlines()[0])
    assert stm.sent.count(b"FW030") == 1
    assert diagnostic["raw_command"] == "FW030"
    assert diagnostic["terminal_response"] == "POSSIBLE_STM_RESET"
    assert "POSSIBLE STM RESET" in output
