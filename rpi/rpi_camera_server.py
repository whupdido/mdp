import socket
import struct

import cv2

HOST = "0.0.0.0"
PORT = 6000
WIDTH, HEIGHT = 640, 480
CAMERA_INDEX = 0  # matches /dev/video0


def main():
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)

    if not cap.isOpened():
        raise RuntimeError(
            "Could not open camera. Check `ls /dev/video*` shows a device, "
            "and that Legacy Camera is enabled in raspi-config."
        )
    print("[RPI] Camera opened")

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((HOST, PORT))
    sock.listen(1)
    print(f"[RPI] Listening on {HOST}:{PORT}")

    try:
        while True:
            conn, addr = sock.accept()
            print(f"[RPI] Laptop connected from {addr}")
            try:
                while True:
                    req = conn.recv(1)
                    if not req:
                        break

                    ok, frame = cap.read()
                    if not ok:
                        print("[RPI] Frame capture failed, skipping")
                        continue

                    ok, buf = cv2.imencode(".jpg", frame,
                                            [int(cv2.IMWRITE_JPEG_QUALITY), 85])
                    if not ok:
                        continue
                    data = buf.tobytes()

                    conn.sendall(struct.pack(">I", len(data)))
                    conn.sendall(data)
            except (ConnectionResetError, BrokenPipeError):
                print("[RPI] Laptop disconnected")
            finally:
                conn.close()
    except KeyboardInterrupt:
        print("\n[RPI] Stopped")
    finally:
        sock.close()
        cap.release()


if __name__ == "__main__":
    main()