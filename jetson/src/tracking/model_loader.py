import numpy as np
import cv2
import tensorrt as trt
import pycuda.driver as cuda
import torch
import threading
import time
from functools import wraps
from ultralytics import YOLO

def ensure_context(fn):
    @wraps(fn)
    def wrapper(self, *args, **kwargs):
        if self.engine_type != "tensorrt" or self.is_shutdown:
            raise RuntimeError("Cannot run inference after shutdown.")
        with self.lock:
            return fn(self, *args, **kwargs)
    return wrapper


def log_exception(fn):
    @wraps(fn)
    def wrapper(*args, **kwargs):
        try:
            return fn(*args, **kwargs)
        except Exception as e:
            print(f"[{fn.__qualname__}] Exception: {e}")
            return args[0]._empty_result()
    return wrapper


class YOLOModel:
    def __init__(self, model_path="v8-512.engine", input_shape=(512, 512), conf=0.3):
        self.input_shape = input_shape
        self.conf_thres = conf
        self.names = ['ball']
        self.original_h, self.original_w = input_shape
        self.is_shutdown = False
        self.lock = threading.Lock()
        self._last_print_time = 0

        try:
            self._init_tensorrt(model_path)
            self.engine_type = "tensorrt"
        except Exception as e:
            print(f"[YOLOModel] TensorRT init failed: {e}, falling back to PyTorch")
            self._init_pytorch_fallback(model_path)
            self.engine_type = "pytorch"

    def _init_tensorrt(self, model_path):
        self.logger = trt.Logger(trt.Logger.WARNING)
        self.runtime = trt.Runtime(self.logger)

        with open(model_path, "rb") as f:
            engine_data = f.read()

        self.engine = self.runtime.deserialize_cuda_engine(engine_data)
        self.context = self.engine.create_execution_context()
        self.input_binding_idx = self.engine.get_binding_index("images")
        self.output_binding_idx = self.engine.get_binding_index(self.engine[1])
        self.output_shape = self.engine.get_binding_shape(self.output_binding_idx)
        self.input_dtype = trt.nptype(self.engine.get_binding_dtype(self.input_binding_idx))
        self.output_dtype = trt.nptype(self.engine.get_binding_dtype(self.output_binding_idx))
        self.input_host = cuda.pagelocked_empty(trt.volume((1, 3, *self.input_shape)), dtype=np.float16)
        self.output_host = cuda.pagelocked_empty(trt.volume(self.output_shape), dtype=self.output_dtype)
        self.input_device = cuda.mem_alloc(self.input_host.nbytes)
        self.output_device = cuda.mem_alloc(self.output_host.nbytes)
        self.stream = cuda.Stream()

    def _init_pytorch_fallback(self, model_path):
        pt_path = model_path.replace(".engine", ".pt")
        self.model = YOLO(pt_path)
        self.device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
        self.names = self.model.names
        print(f"[YOLOModel] Loaded PyTorch model from: {pt_path}")

    def __del__(self):
        # No explicit CUDA context management needed
        pass

    def get_label(self, cls_id):
        return self.names[int(cls_id)]

    def preprocess(self, image):
        img = cv2.resize(image, self.input_shape[::-1])
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))[None].astype(np.float16)
        np.copyto(self.input_host, img.ravel())

    def predict(self, image):
        if self.engine_type == "tensorrt":
            return self._predict_tensorrt(image)
        return self._predict_pytorch(image)

    @ensure_context
    @log_exception
    def _predict_tensorrt(self, image):
        self.original_h, self.original_w = image.shape[:2]
        self.preprocess(image)

        cuda.memcpy_htod_async(self.input_device, self.input_host, self.stream)
        self.context.execute_async_v2(
            bindings=[int(self.input_device), int(self.output_device)],
            stream_handle=self.stream.handle
        )
        cuda.memcpy_dtoh_async(self.output_host, self.output_device, self.stream)
        self.stream.synchronize()

        raw_output = self.output_host.reshape(self.output_shape)
        return self.postprocess(raw_output)

    @log_exception
    def _predict_pytorch(self, image):
        with torch.no_grad():
            results = self.model.predict(
                source=image, conf=self.conf_thres, device=self.device, imgsz=512, verbose=False
            )
            return results[0]

    def postprocess(self, output, iou_thres=0.5):
        output = output.squeeze()
        if output.shape[0] != 5:
            return self._empty_result()

        x_center, y_center, width, height, conf = output
        mask = conf >= self.conf_thres
        if not np.any(mask):
            return self._empty_result()

        x1 = x_center[mask] - width[mask] / 2
        y1 = y_center[mask] - height[mask] / 2
        x2 = x_center[mask] + width[mask] / 2
        y2 = y_center[mask] + height[mask] / 2

        boxes = np.stack([
            x1 * self.original_w / self.input_shape[0],
            y1 * self.original_h / self.input_shape[1],
            x2 * self.original_w / self.input_shape[0],
            y2 * self.original_h / self.input_shape[1],
        ], axis=1)

        scores = conf[mask]

        nms_indices = self.nms(boxes, scores, iou_thres)
        if not nms_indices:
            return self._empty_result()
        best_idx = max(nms_indices, key=lambda i: scores[i])
        indices = [best_idx]

        def make_box(i):
            b = boxes[i]
            return Box(b[0], b[1], b[2], b[3], scores[i], 0)

        return Result([make_box(i) for i in indices])

    def nms(self, boxes, scores, iou_threshold=0.5):
        x1, y1, x2, y2 = boxes.T
        areas = (x2 - x1) * (y2 - y1)
        order = scores.argsort()[::-1]

        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)
            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])

            inter = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
            union = areas[i] + areas[order[1:]] - inter
            iou = inter / (union + 1e-6)
            order = order[1:][iou <= iou_threshold]

        return keep

    def _empty_result(self):
        class Empty:
            boxes = []
        return Empty()

    def shutdown(self):
        print("[YOLOModel] Starting shutdown...")
        with self.lock:
            if self.is_shutdown:
                return
            self.is_shutdown = True

        try:
            for attr in ('input_device', 'output_device'):
                dev = getattr(self, attr, None)
                if dev:
                    dev.free()

            for attr in ('context', 'engine', 'runtime', 'stream'):
                obj = getattr(self, attr, None)
                if obj:
                    del obj

            print("[YOLOModel] Shutdown completed")

        except Exception as e:
            print(f"[YOLOModel] Error: {e}")

class Box:
    def __init__(self, x1, y1, x2, y2, conf, cls):
        self.xyxy = np.array([x1, y1, x2, y2])
        self.conf = np.array([conf])
        self.cls = np.array([cls])
        self.xywh = np.array([
            (x1 + x2) / 2,
            (y1 + y2) / 2,
            x2 - x1,
            y2 - y1
        ]).reshape(1, 4)


class Result:
    def __init__(self, boxes):
        self.boxes = boxes