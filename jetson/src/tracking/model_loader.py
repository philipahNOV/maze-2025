import numpy as np
from ultralytics import YOLO

class YOLOModel:
    def __init__(self, model_path="v8-291.onnx"):
        print(f"[YOLOModel] Loading model from: {model_path}")
        self.model = YOLO(model_path)

        if not hasattr(self.model, "predict"):
            raise RuntimeError(f"[YOLOModel] Failed to load model from: {model_path}")

        try:
            dummy = np.zeros((640, 640, 3), dtype=np.uint8)
            self.model.predict(dummy, verbose=False)
        except Exception as e:
            raise RuntimeError(f"[YOLOModel] Failed during warmup: {e}")

        self.names = self.model.names

    def predict(self, image, conf=0.35):
        return self.model.predict(source=image, conf=conf, verbose=False)[0]

    def get_label(self, cls_id):
        return self.names[int(cls_id)]
