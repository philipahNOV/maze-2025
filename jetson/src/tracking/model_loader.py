import numpy as np
from ultralytics import YOLO

class YOLOModel:
    def __init__(self, model_path="v8-281.pt"):
        print("Loading tracking model...")
        self.model = YOLO(model_path)
        self.model.fuse()
        self.model.eval()
        dummy = np.zeros((720, 1280, 3), dtype=np.uint8)
        self.model.predict(dummy, verbose=False)
        print("Tracking model ready.")

    def predict(self, image, conf=0.7):
        return self.model.predict(source=image, conf=conf)[0]
    
    def get_label(self, cls_id):
        return self.model.names[int(cls_id)]