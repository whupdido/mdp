import argparse
import json
import socket
import struct
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

from server.detection_labels import draw_detection_annotations

HERE = Path(__file__).parent
MODEL_PATH = HERE / "best.pt"
MAPPING_PATH = HERE / "mapping.json"
NAMES_PATH = HERE / "image_names.json"
PORT = 6000

# Placeholder ID for the bullseye/marker class -- not a scorable image.
BULLSEYE_PLACEHOLDER = 0


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
    boxes = []
    image_ids = []
    for box in results.boxes:
        raw_class_id = int(box.cls[0])
        image_ids.append(mapping.get(raw_class_id, raw_class_id))
        boxes.append(box.xyxy[0].tolist())

    return draw_detection_annotations(frame, boxes, image_ids, names)


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
