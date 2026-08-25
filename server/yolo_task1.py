# yolo.py
import socket
import traceback
import yaml
import os
import time
from ultralytics import YOLO
import cv2
import threading
import numpy as np
import json

from server.utils import recv_pickle, send_json, send_pickle

class Server:
    def __init__(self, config):

        host = "0.0.0.0"
        port = int(config["port"])
        stream = config["stream"]
        unwanted = config["unwanted"]
        num_images = config["num_images"]
        fail_threshold = config["fail_threshold"]
        model_configs = list(config["models"].values())

        self.models = []
        self.weights = []
        self.mappings = []
        self.stream = stream
        self.unwanted = unwanted
        self.num_images_to_save = num_images
        self.fail_threshold = fail_threshold

        self.main_model_idx = None

        # =========================
        # Load models and mappings
        # =========================
        # Get script directory for resolving relative paths
        script_dir = os.path.dirname(os.path.abspath(__file__))
        # NOTE: in the old repo this script lived at backend/server/ (two levels
        # deep from repo root); here it's server/ (one level deep), so project
        # root is only one dirname() up, not two.
        project_root = os.path.dirname(script_dir)

        for i, cfg in enumerate(model_configs):
            path = cfg["path"]
            weight = cfg.get("weight", 1.0)
            mapping_path = cfg.get("mapping", None)
            is_main = cfg.get("is_main", False)

            # Track which one is main
            if is_main:
                if self.main_model_idx is not None:
                    raise ValueError(f"[SERVER] Multiple main models specified! Already had {self.main_model_idx}, but model {i} also marked is_main=True")
                self.main_model_idx = i

            # Resolve relative paths from project root
            if not os.path.isabs(path):
                path = os.path.join(project_root, path)

            # print(path)
            model = YOLO(path)
            self.models.append(model)
            self.weights.append(weight)

            # Load mapping if exists
            if mapping_path:
                # Resolve relative paths from project root
                if not os.path.isabs(mapping_path):
                    mapping_path = os.path.join(project_root, mapping_path)

                if os.path.exists(mapping_path):
                    with open(mapping_path, "r") as f:
                        mapping = json.load(f)
                    print(f"[SERVER] Loaded mapping for model {i}: {mapping_path}")
                else:
                    mapping = None
            else:
                mapping = None

            self.mappings.append(mapping)

        # Must have exactly one main model
        if self.main_model_idx is None:
            raise ValueError("[SERVER] No main model specified! Set exactly one model with 'is_main: true' in config.yaml")

        self.main_model = self.models[self.main_model_idx]
        print(f"[SERVER] Main model = index {self.main_model_idx}: {model_configs[self.main_model_idx]['path']}")

        # =========================
        # Socket setup
        # =========================
        print("[SERVER] Loaded models:")
        for i, cfg in enumerate(model_configs):
            print(f"   Model {i}: {cfg['path']} (weight={self.weights[i]}, mapping={self.mappings[i] is not None}, is_main={cfg.get('is_main', False)})")

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, port))
        self.sock.listen(1)
        self.sock.settimeout(1)
        print(f"[SERVER] Listening on {(host, port)}")

    # ==========================================================
    # Threaded YOLO inference for one model
    # ==========================================================
    def _run_model(self, model, obj, results_list, idx):
        try:
            print(obj)
            if "frames" in obj:
                frames = obj["frames"]
            elif "frame" in obj:
                frames = [obj["frame"]]
            else:
                raise ValueError("No 'frame' or 'frames' key found in object.")

            for i, frame in enumerate(frames):
                result = model.predict(
                    frame,
                    save=False,
                    imgsz=(obj["height"], obj["width"]),
                    conf=obj["conf"],
                    verbose=False,
                )[0]

                ts = time.strftime("%Y%m%d-%H%M%S")
                filename = f"yolo_logs/frame_{ts}_{idx}_{i}.jpg"
                cv2.imwrite(filename, result)

                # Check if this frame produced a detection
                if len(result.boxes) > 0:
                    print(f"[SERVER] Model {idx} found detection, stopping early.")
                    results_list[idx] = result
                    break

        except Exception as e:
            print(f"[SERVER] Model {idx} failed:", e)
            traceback.print_exc()
            results_list[idx] = None

    # ==========================================================
    # Handle a single client connection
    # ==========================================================
    def handle_client(self, conn: socket.socket):
        try:
            # Send class names metadata immediately upon connection
            metadata = {"names": self.main_model.names}
            send_json(conn, metadata)
            print(f"[SERVER] Sent class names metadata: {list(self.main_model.names.values())}")

            os.makedirs("yolo_logs", exist_ok=True)
            frame_counter = 0
            saved_files = []

            failed = 0

            while True:
                obj = recv_pickle(conn)
                if obj is None:
                    break

                # ---- Run models in parallel ----
                results = [None] * len(self.models)
                threads = []
                for i, model in enumerate(self.models):
                    t = threading.Thread(target=self._run_model, args=(model, obj, results, i))
                    t.start()
                    threads.append(t)
                for t in threads:
                    t.join()

                # ---- Weighted fusion ----
                class_scores = {}
                class_model_map = {}  # remember which model saw this class

                for model_idx, result in enumerate(results):
                    if result is None or result.boxes is None or len(result.boxes) == 0:
                        continue

                    boxes = result.boxes.xyxy.cpu().numpy()
                    cids = result.boxes.cls.cpu().numpy().astype(int)

                    print(cids)

                    # Apply mapping if exists
                    mapping = self.mappings[model_idx]
                    if mapping is not None:
                        cids = np.array([mapping.get(str(cid), cid) for cid in cids])

                    mask = ~np.isin(cids, self.unwanted)
                    boxes = boxes[mask]
                    cids = cids[mask]

                    for i, cid in enumerate(cids):
                        x1, y1, x2, y2 = boxes[i]
                        length = abs(y2 - y1)
                        weighted_val = self.weights[model_idx] * length
                        class_scores[cid] = class_scores.get(cid, 0.0) + weighted_val
                        class_model_map[cid] = class_model_map.get(cid, model_idx)

                # -------------------------------------------------------------------
                # Always save once — fallback to main model if no detections
                # -------------------------------------------------------------------
                if not class_scores:
                    # No valid detections → use main model for saving
                    frames = obj.get("frames", [obj["frame"]])
                    for f in frames:
                        annotated = f.copy()
                        ts = time.strftime("%Y%m%d-%H%M%S")
                        filename = f"yolo_logs/frame_{ts}_{frame_counter}.jpg"
                        cv2.imwrite(filename, annotated)
                        print(f"[SERVER] No detections. Saved main model frame: {filename}")
                        frame_counter += 1
                    if failed == self.fail_threshold:
                        saved_files.append(filename)
                        failed = 0
                    else:
                        failed += 1
                    send_pickle(conn, {
                        "bboxes": [],
                        "confs": [],
                        "cids": [],
                        "annotated_jpeg": None,
                        "class_id": None,
                    })

                else:
                    failed = 0
                    # ---- Find top class ----
                    best_class = max(class_scores, key=class_scores.get)
                    best_model_idx = class_model_map.get(best_class, 0)
                    best_result = results[best_model_idx]

                    # ---- Save once with chosen model ----
                    annotated = best_result.plot()

                    ts = time.strftime("%Y%m%d-%H%M%S")
                    filename = f"yolo_logs/frame_{ts}_{frame_counter}.jpg"
                    cv2.imwrite(filename, annotated)
                    frame_counter += 1
                    saved_files.append(filename)

                    print(f"[SERVER] Saved annotated image (model {best_model_idx}, class {best_class}): {filename}")
                    print(f"[SERVER] Best class ID: {best_class}")

                    boxes = best_result.boxes.xyxy.cpu().numpy()
                    confs = best_result.boxes.conf.cpu().numpy()
                    cids = best_result.boxes.cls.cpu().numpy().astype(int)

                    # Apply mapping/unwanted to payload for consistency
                    mapping = self.mappings[best_model_idx]
                    if mapping is not None:
                        cids = np.array([mapping.get(str(cid), cid) for cid in cids])
                    mask = ~np.isin(cids, self.unwanted)
                    boxes = boxes[mask]
                    confs = confs[mask]
                    cids = cids[mask]

                    ok, buf = cv2.imencode(".jpg", annotated, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
                    annotated_jpeg = buf.tobytes() if ok else None

                    send_pickle(conn, {
                        "bboxes": boxes.tolist(),
                        "confs": confs.tolist(),
                        "cids": cids.tolist(),
                        "annotated_jpeg": annotated_jpeg,
                        "class_id": int(best_class),
                    })

                if len(saved_files) == self.num_images_to_save:
                    stitched = self._stitch_frames(saved_files, rows=2, cols=4, width=640, height=480)
                    out_path = f"yolo_logs/stitched_{time.strftime('%Y%m%d-%H%M%S')}.jpg"
                    cv2.imwrite(out_path, stitched)
                    print(f"[SERVER] Stitched 8 frames → {out_path}")
                    saved_files.clear()

        except Exception as e:
            print("[SERVER] Error:", e)

    # ==========================================================
    # Main server loop
    # ==========================================================
    def serve(self):
        print("[SERVER] Waiting for clients (Ctrl+C to stop)…")
        while True:
            conn = None
            try:
                try:
                    conn, addr = self.sock.accept()
                except socket.timeout:
                    continue
                print(f"[SERVER] Connected from {addr}")

                self.handle_client(conn)

            except KeyboardInterrupt:
                raise
            except Exception as e:
                print("[SERVER] Connection error:", e)
            finally:
                if conn:
                    try:
                        conn.close()
                    except Exception:
                        pass
                    conn = None

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            finally:
                self.sock = None

    def _stitch_frames(self, file_list, rows=2, cols=4, width=640, height=480):

        imgs = []
        for path in file_list:
            img = cv2.imread(path)
            if img is None:
                img = np.zeros((height, width, 3), dtype=np.uint8)
            else:
                img = cv2.resize(img, (width, height))
            imgs.append(img)

        # Fill grid with black frames if fewer than expected
        while len(imgs) < rows * cols:
            imgs.append(np.zeros((height, width, 3), dtype=np.uint8))

        # Stack into grid
        grid = []
        for r in range(rows):
            row_imgs = imgs[r * cols:(r + 1) * cols]
            grid.append(np.hstack(row_imgs))
        return np.vstack(grid)


# ==========================================================
# Entry Point
# ==========================================================
if __name__ == "__main__":
    # Get the directory where this script is located
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, "task1_config.yaml")

    with open(config_path, "r") as f:
        config = yaml.safe_load(f)["yolo"]

    server = Server(config)

    try:
        server.serve()
    except KeyboardInterrupt:
        print("\n[!] Server stopped by user (Ctrl+C)")
    finally:
        server.close()
