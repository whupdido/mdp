"""
Reference client for server/yolo_task1.py.

This is NOT part of the RPi's real code -- it's a working, minimal example
showing exactly how to talk to the detection server, for whoever wires the
real RPi-side capture logic (take a photo -> call this -> get an ID -> send
TARGET,<obstacle>,<id> to Android).

Run this on any machine that can reach the laptop running yolo_task1.py,
pointed at a single test image, to see the full request/response shape.

Usage:
    python example_client.py <server_ip> <path_to_image.jpg>
"""

import socket
import sys

import cv2

from utils import recv_json, recv_pickle, send_pickle

PORT = 5001


def main(server_ip: str, image_path: str):
    frame = cv2.imread(image_path)
    if frame is None:
        raise SystemExit(f"Could not read image: {image_path}")
    height, width = frame.shape[:2]

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((server_ip, PORT))
    print(f"Connected to {server_ip}:{PORT}")

    # The server sends class-name metadata immediately on connect.
    metadata = recv_json(sock)
    print("Server class names:", metadata)

    # Send one frame. This exact shape is what yolo_task1.py expects:
    #   "frame": a single BGR image (numpy array from cv2.imread/cv2.imdecode)
    #   "height"/"width": the image dimensions
    #   "conf": confidence threshold to use for this prediction
    send_pickle(sock, {
        "frame": frame,
        "height": height,
        "width": width,
        "conf": 0.25,
    })

    # Response shape:
    #   "class_id": int or None -- the official Image ID (11-40), already
    #               mapped and already filtered against the "unwanted" list
    #               in task1_config.yaml. None means no valid detection.
    #   "bboxes", "confs", "cids": raw per-box detail, if you need it
    #   "annotated_jpeg": JPEG bytes of the frame with boxes drawn, or None
    response = recv_pickle(sock)
    print("class_id:", response["class_id"])
    print("bboxes:", response["bboxes"])
    print("confs:", response["confs"])

    sock.close()


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit(f"Usage: python {sys.argv[0]} <server_ip> <path_to_image.jpg>")
    main(sys.argv[1], sys.argv[2])
