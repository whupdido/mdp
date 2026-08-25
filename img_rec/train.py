from ultralytics import YOLO

# Load model (YOLOv8n pretrained weights)
model = YOLO("yolov8n.pt")

# Train
model.train(
    data="Detection-Symbols-Updated-3-4/data.yaml",  # swap to your own data.yaml once the real dataset is back
    epochs=100,          # matches the old repo's real setting
    patience=20,         # auto-stop if mAP hasn't improved in 20 epochs, so it doesn't run to 100 past convergence
    imgsz=(640, 480),
    rect=True,
    batch=16,
    device="mps",        # Apple Silicon GPU backend -- was "device=3" (CUDA) in the original, which errors on this Mac
    mosaic=1.0,
    mixup=0.2,
    # blur=0.2,
    # noise=0.1,
    hsv_h=0.015,       # hue
    hsv_s=0.7,         # saturation
    hsv_v=0.4,         # brightness
    fliplr=0.0,        # disable left-right flip
    flipud=0.0,        # disable up-down flip
    name="full_train_v1"
)
