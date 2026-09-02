"""
Capture a photo of an obstacle, send it to the laptop's detection server
(server/yolo_task1.py), and report the result to Android as the TARGET
string it expects (see Android/PROTOCOL.md -- "Image recognition (Denzel)").

This runs ON THE RPI (needs the physical camera). It's a building block, not
a standalone program: something else -- wherever "we've arrived at obstacle
B<n>, face <D>" gets decided in the real movement/pathfinding loop -- needs
to call report_obstacle() with the obstacle number and the open Android
serial connection (the same one a1_bridge.py holds open).

Example, once wired into the real loop:

    import serial
    from capture_and_report import report_obstacle

    with serial.Serial(STM_DEVICE, BAUD_RATE, timeout=1) as stm:
        with serial.Serial(BT_DEVICE, BAUD_RATE, timeout=1) as android:
            report_obstacle(obstacle_number=2, android_serial=android, stm_serial=stm)
"""

# Defers type-hint evaluation so `str | None`-style hints below don't crash
# on the Pi's older Python (that union syntax needs 3.10+ without this).
from __future__ import annotations

import pickle
import socket
import struct

import cv2

# Laptop's IP on the shared WiFi, running `python -m server.yolo_task1`.
# TODO: set this before running -- find it with `ipconfig getifaddr en0` on the Mac.
DETECTION_SERVER_IP = "SET_ME_TO_YOUR_LAPTOP_IP"
DETECTION_SERVER_PORT = 5001
CAMERA_INDEX = 0  # matches /dev/video0, same as rpi_camera_server.py

# The bullseye/marker class -- confirms the camera is facing the obstacle,
# but is NOT a real face image and must never be reported as one.
BULLSEYE_ID = 41


def _recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("Detection server closed the connection")
        buf += chunk
    return buf


def _recv_json(sock: socket.socket) -> dict:
    (length,) = struct.unpack(">I", _recv_exact(sock, 4))
    import json
    return json.loads(_recv_exact(sock, length).decode("utf-8"))


def _send_pickle(sock: socket.socket, obj) -> None:
    # Pinned to protocol 4, not HIGHEST_PROTOCOL -- keeps this compatible
    # with the server's Python even if this script ever runs somewhere with
    # a different Python version than expected.
    blob = pickle.dumps(obj, protocol=4)
    sock.sendall(struct.pack(">Q", len(blob)))
    sock.sendall(blob)


def _recv_pickle(sock: socket.socket):
    (length,) = struct.unpack(">Q", _recv_exact(sock, 8))
    return pickle.loads(_recv_exact(sock, length))


def capture_frame():
    """Grab one frame from the RPi camera."""
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        raise RuntimeError(
            "Could not open camera. Check `ls /dev/video*` shows a device, "
            "and that Legacy Camera is enabled in raspi-config."
        )
    try:
        ok, frame = cap.read()
    finally:
        cap.release()
    if not ok:
        raise RuntimeError("Camera opened but failed to capture a frame")
    return frame


def detect(frame, conf: float = 0.25):
    """Send one frame to the detection server. Returns the official Image ID
    (11-40), or None if nothing was confidently detected."""
    height, width = frame.shape[:2]

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((DETECTION_SERVER_IP, DETECTION_SERVER_PORT))
    try:
        _recv_json(sock)  # class-name metadata, sent on connect -- not needed here
        _send_pickle(sock, {
            "frame": frame,
            "height": height,
            "width": width,
            "conf": conf,
        })
        response = _recv_pickle(sock)
        return response["class_id"]
    finally:
        sock.close()


def send_to_stm(class_id: int, stm_serial):
    """Send IMxxx (xxx = the Image ID, zero-padded to 3 digits) to the STM
    over the same serial connection a1_bridge.py uses.

    NOTE: the current STM firmware (stm32/Core/Src/command.c) doesn't have an
    "IM" branch in its command dispatcher yet -- it'll reply ERR until that's
    added on the firmware side. That's expected for now, not a bug here;
    this function's job is just to send the signal, not to process the reply.
    """
    message = f"IM{class_id:03d}"
    stm_serial.write((message + "\n").encode("ascii"))
    stm_serial.flush()
    print(f"[CAPTURE] Sent to STM: {message}")


def signal_search_result(class_id, stm_serial):
    """A.5 search protocol: IM000 means "not found yet" (covers both no
    detection at all and seeing only the marker) -- the STM keeps rotating.
    Any other IMxxx means "found it, stop" -- xxx is the real Image ID.

    Unlike send_to_stm(), this always sends something (never skips), since
    the STM needs a signal every cycle to know whether to keep searching.
    Returns the message that was sent.
    """
    if class_id is None or class_id == BULLSEYE_ID:
        message = "IM000"
    else:
        message = f"IM{class_id:03d}"

    stm_serial.write((message + "\n").encode("ascii"))
    stm_serial.flush()
    print(f"[CAPTURE] Sent to STM: {message}")
    return message


def report_obstacle(obstacle_number: int, android_serial, stm_serial=None, face: str | None = None):
    """Capture a frame, detect it, and:
      - send TARGET,<obstacle>,<id> (optionally ,<face>) to Android
      - if stm_serial is given, also send IMxxx to the STM

    Returns the detected Image ID, or None if nothing was found (including
    seeing only the bullseye/marker) -- in that case nothing is sent
    anywhere. For the marker case that's a deliberate choice, not just
    relying on Android to drop an out-of-range ID: the marker means "facing
    the obstacle," not "found the face," so it isn't a real result yet."""
    frame = capture_frame()
    class_id = detect(frame)

    if class_id is None:
        print(f"[CAPTURE] No confident detection for obstacle {obstacle_number}")
        return None

    if class_id == BULLSEYE_ID:
        print(f"[CAPTURE] Saw the marker, not a face yet, for obstacle {obstacle_number}")
        return None

    message = f"TARGET,{obstacle_number},{class_id}"
    if face is not None:
        message += f",{face}"

    android_serial.write((message + "\n").encode("ascii"))
    android_serial.flush()
    print(f"[CAPTURE] Sent: {message}")

    if stm_serial is not None:
        send_to_stm(class_id, stm_serial)

    return class_id


if __name__ == "__main__":
    # Manual test: capture + detect only, no Android connection needed.
    # Confirms the camera and the detection server both work before this
    # is wired into the real movement loop.
    frame = capture_frame()
    class_id = detect(frame)
    print("Detected Image ID:", class_id)
