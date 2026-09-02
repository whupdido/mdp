from ultralytics import YOLO

# Load your trained model
model = YOLO("best.pt")
model.predict(source=0, show=True)
