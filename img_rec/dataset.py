import os

from roboflow import Roboflow

api_key = os.environ["ROBOFLOW_API_KEY"]  # set this in your shell, never hardcode it here
rf = Roboflow(api_key=api_key)
project = rf.workspace("alex-khoo").project("detection-symbols-updated-3-z4qhn")
dataset = project.version(4).download("yolov8")  # or "coco", "voc", etc.
