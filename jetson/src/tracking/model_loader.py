import numpy as np
import torch
from ultralytics import YOLO

class YOLOModel:
    def __init__(self, model_path="v8-291.trt"):
        print(f"[YOLOModel] Loading model from: {model_path}")
        
        # Check CUDA availability and set device appropriately for TensorRT
        self.device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
        print(f"[YOLOModel] Using device: {self.device}")
        
        try:
            # Load model with explicit device setting for TensorRT
            self.model = YOLO(model_path)
            
            # For TensorRT models, ensure proper device placement
            if model_path.endswith('.trt') and torch.cuda.is_available():
                self.model.to(self.device)
                
        except Exception as e:
            print(f"[YOLOModel] Failed to load TensorRT model: {e}")
            print(f"[YOLOModel] Falling back to ONNX model...")
            # Fallback to ONNX if TensorRT fails
            fallback_path = model_path.replace('.trt', '.onnx')
            self.model = YOLO(fallback_path)
            self.device = 'cpu'  # Use CPU for ONNX fallback to avoid CUDA issues

        if not hasattr(self.model, "predict"):
            raise RuntimeError(f"[YOLOModel] Failed to load model from: {model_path}")

        try:
            # Warmup with proper device handling
            dummy = np.zeros((640, 640, 3), dtype=np.uint8)
            with torch.no_grad():  # Prevent CUDA memory issues during warmup
                self.model.predict(dummy, verbose=False, device=self.device)
            print(f"[YOLOModel] Warmup successful on {self.device}")
        except Exception as e:
            print(f"[YOLOModel] Warmup failed: {e}")
            # Don't raise error for warmup failure, model might still work
            
        self.names = self.model.names

    def predict(self, image, conf=0.35):
        try:
            with torch.no_grad():  # Prevent CUDA memory accumulation
                results = self.model.predict(
                    source=image, 
                    conf=conf, 
                    verbose=False, 
                    device=self.device,
                    imgsz=640  # Explicit image size for TensorRT
                )
                return results[0]
        except Exception as e:
            print(f"[YOLOModel] Prediction failed: {e}")
            # Return empty result to prevent crashes
            class EmptyResult:
                def __init__(self):
                    self.boxes = []
            return EmptyResult()

    def get_label(self, cls_id):
        return self.names[int(cls_id)]
