import numpy as np
import cv2
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
import torch

class YOLOModel:
    def __init__(self, model_path="new-v8-fp16.engine", input_shape=(640, 640)):
        print(f"[YOLOModel] Loading TensorRT engine from: {model_path}")
        
        # Try TensorRT first, fallback to PyTorch if it fails
        try:
            self._init_tensorrt(model_path, input_shape)
            self.engine_type = "tensorrt"
            print(f"[YOLOModel] TensorRT engine loaded successfully")
        except Exception as e:
            print(f"[YOLOModel] TensorRT initialization failed: {e}")
            print(f"[YOLOModel] Falling back to PyTorch YOLO...")
            self._init_pytorch_fallback(model_path)
            self.engine_type = "pytorch"
    
    def _init_tensorrt(self, model_path, input_shape):
        """Initialize TensorRT engine with proper error handling"""
        self.input_shape = input_shape
        self.names = ['ball']
        self.original_h, self.original_w = 640, 640
        
        cuda.init()
        self.cuda_ctx = cuda.Device(0).make_context()
        
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
        self.input_host = cuda.pagelocked_empty(trt.volume((1, 3, *input_shape)), dtype=np.float32)
        self.output_host = cuda.pagelocked_empty(trt.volume(self.output_shape), dtype=self.output_dtype)
        self.input_device = cuda.mem_alloc(self.input_host.nbytes)
        self.output_device = cuda.mem_alloc(self.output_host.nbytes)
        self.stream = cuda.Stream()

    def _init_pytorch_fallback(self, model_path):
        from ultralytics import YOLO
        
        pt_path = model_path.replace(".engine", ".pt")
        try:
            self.model = YOLO(pt_path)
            self.device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
            self.names = self.model.names
            print(f"[YOLOModel] PyTorch fallback model loaded from: {pt_path}")
        except Exception as e:
            raise RuntimeError(f"[YOLOModel] Both TensorRT and PyTorch models failed: {e}")

    def __del__(self):
        if hasattr(self, 'cuda_ctx') and self.cuda_ctx:
            self.cuda_ctx.pop()
            self.cuda_ctx = None

    def preprocess(self, image):
        if self.engine_type != "tensorrt":
            return

        # input shape is (720, 1280, 3)
        input_w, input_h = self.input_shape
        img = cv2.resize(image, (input_w, input_h))  # (640, 640)
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))  # HWC to CHW
        img = np.expand_dims(img, axis=0)
        np.copyto(self.input_host, img.ravel())


    def predict(self, image, conf=0.35):
        if self.engine_type == "tensorrt":
            return self._predict_tensorrt(image, conf)
        else:
            return self._predict_pytorch(image, conf)
    
    def _predict_tensorrt(self, image, conf=0.35):
        if not self.cuda_ctx:
            raise RuntimeError("CUDA context not initialized properly")
        
        try:
            self.cuda_ctx.push()
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
            print(f"[YOLOModel] Raw TensorRT output shape: {raw_output.shape}")
            return self.postprocess(raw_output, conf)
        except Exception as e:
            print(f"[YOLOModel] TensorRT inference failed: {e}")
            return self._empty_result()
        finally:
            self.cuda_ctx.pop()
    
    def _predict_pytorch(self, image, conf=0.35):
        try:
            with torch.no_grad():
                results = self.model.predict(
                    source=image,
                    conf=conf,
                    device=self.device,
                    imgsz=640,
                    verbose=False
                )
                return results[0]
        except Exception as e:
            print(f"[YOLOModel] PyTorch inference failed: {e}")
            return self._empty_result()
    
    def _empty_result(self):
        class EmptyResult:
            def __init__(self):
                self.boxes = []
        return EmptyResult()

    def postprocess(self, output, conf_thres=0.35):
        # Expecting [1, num_preds, 6] -> (x1, y1, x2, y2, conf, cls)
        if output.ndim == 3:
            output = output[0]  # Remove batch dimension

        if output.shape[-1] == 6:
            # Already in [x1, y1, x2, y2, conf, cls] format
            predictions = output[output[:, 4] >= conf_thres]
        elif output.shape[-1] == 5:
            print("[YOLOModel] ERROR: Output has no class index. Postprocess requires [x1,y1,x2,y2,conf,class]")
            return self._empty_result()
        else:
            print(f"[YOLOModel] Unsupported output shape: {output.shape}")
            return self._empty_result()

        if predictions.shape[0] == 0:
            return self._empty_result()

        
        # Convert from center-scaled 640x640 back to original shape 720x1280
        scale_x = self.original_w / self.input_shape[0]  # 1280 / 640 = 2.0
        scale_y = self.original_h / self.input_shape[1]  # 720 / 640 = 1.125

        predictions[:, 0] *= scale_x  # x1
        predictions[:, 1] *= scale_y  # y1
        predictions[:, 2] *= scale_x  # x2
        predictions[:, 3] *= scale_y  # y2

        class Box:
            def __init__(self, box_array):
                self.xyxy = box_array[:4]
                self.conf = box_array[4:5]
                self.cls = box_array[5:6]
                self.xywh = np.array([  # Convert to center format
                    (self.xyxy[0] + self.xyxy[2]) / 2,
                    (self.xyxy[1] + self.xyxy[3]) / 2,
                    self.xyxy[2] - self.xyxy[0],
                    self.xyxy[3] - self.xyxy[1]
                ]).reshape(1, 4)

        class Result:
            def __init__(self, boxes):
                self.boxes = boxes

        boxes = [Box(pred) for pred in predictions]
        return Result(boxes)

    def get_label(self, cls_id):
        if self.engine_type == "tensorrt":
            return self.names[int(cls_id)]
        else:
            return self.names[int(cls_id)]