import argparse
import json
import socket
import struct
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

HERE = Path(__file__).parent
MODEL_PATH = HERE / "best.pt"
MAPPING_PATH = HERE / "mapping.json"
NAMES_PATH = HERE / "image_names.json"
PORT = 6000

# Placeholder ID for the bullseye/marker class -- not a scorable image.
BULLSEYE_PLACEHOLDER = 41


def load_mapping() -> dict[int, int]:
    with open(MAPPING_PATH) as f:
        raw = json.load(f)
    return {int(k): v for k, v in raw.items()}


def load_names() -> dict[int, str]:
    with open(NAMES_PATH) as f:
        raw = json.load(f)
    return {int(k): v for k, v in raw.items()}


def recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("RPi closed the connection")
        buf += chunk
    return buf


def get_frame(sock: socket.socket) -> np.ndarray:
    sock.sendall(b"\x01")  # request signal
    (length,) = struct.unpack(">I", recv_exact(sock, 4))
    data = recv_exact(sock, length)
    arr = np.frombuffer(data, dtype=np.uint8)
    return cv2.imdecode(arr, cv2.IMREAD_COLOR)


def draw_detections(frame, results, mapping, names):
    for box in results.boxes:
        raw_class_id = int(box.cls[0])
        conf = float(box.conf[0])
        image_id = mapping.get(raw_class_id)
        name = names.get(image_id, "?")
        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())

        if image_id == BULLSEYE_PLACEHOLDER:
            color = (0, 165, 255)  # orange -- marker, not a scorable image
            label = f"MARKER (raw {raw_class_id})"
        else:
            color = (0, 220, 0)    # green -- real recognized image
            label = f"Image ID: {image_id} ({name})  (raw {raw_class_id}, {conf:.2f})"

        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(frame, label, (x1, max(0, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    return frame


def main(rpi_ip: str, conf: float):
    model = YOLO(str(MODEL_PATH))
    mapping = load_mapping()
    names = load_names()

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((rpi_ip, PORT))
    print(f"[PC] Connected to RPi camera at {rpi_ip}:{PORT}")

    try:
        while True:
            frame = get_frame(sock)
            results = model.predict(frame, conf=conf, verbose=False)[0]
            frame = draw_detections(frame, results, mapping, names)

            cv2.imshow("A.2 -- RPi detection demo (q to quit)", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        sock.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--rpi-ip", required=True, help="RPi's IP on the shared WiFi")
    parser.add_argument("--conf", type=float, default=0.25)
    args = parser.parse_args()
    main(args.rpi_ip, args.conf)
