import numpy as np
import torch
from ultralytics import YOLO

class YOLOModel:
    def __init__(self, model_path="new-v8.engine"):
        print(f"[YOLOModel] Loading model from: {model_path}")

        self.device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
        print(f"[YOLOModel] Using device: {self.device}")

        self.model = None
        self.names = []
        self.is_trt = model_path.endswith('.engine')

        try:
            self.model = YOLO(model_path)
            print("[YOLOModel] TensorRT model loaded")

            dummy = np.zeros((640, 640, 3), dtype=np.uint8)
            with torch.no_grad():
                self.model.predict(source=dummy, device=0, verbose=False)  # TensorRT requires device=0
            print(f"[YOLOModel] Warmup successful on TensorRT (device=0)")

        except Exception as e:
            print(f"[YOLOModel] Failed to load or warm up TensorRT model: {e}")
            print("[YOLOModel] Falling back to PyTorch (.pt) model...")

            pt_path = model_path.replace(".engine", ".pt")
            try:
                self.model = YOLO(pt_path)
                self.device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
                print(f"[YOLOModel] Fallback PyTorch model loaded from: {pt_path}")
            except Exception as e2:
                raise RuntimeError(f"[YOLOModel] Failed to load fallback model: {e2}")

        if not hasattr(self.model, "predict"):
            raise RuntimeError(f"[YOLOModel] Model from {model_path} does not support prediction")

        self.names = self.model.names

    def predict(self, image, conf=0.35):
        try:
            with torch.no_grad():
                if self.is_trt:
                    results = self.model.predict(
                        source=image,
                        conf=conf,
                        device=0,
                        imgsz=640,
                        verbose=False
                    )
                else:
                    results = self.model.predict(
                        source=image,
                        conf=conf,
                        device=self.device,
                        imgsz=640,
                        verbose=False
                    )
                return results[0]
        except Exception as e:
            print(f"[YOLOModel] Prediction failed: {e}")
            class EmptyResult:
                def __init__(self):
                    self.boxes = []
            return EmptyResult()

    def get_label(self, cls_id):
        return self.names[int(cls_id)]