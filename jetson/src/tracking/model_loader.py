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
        self.original_h, self.original_w = 640, 640  # Default values
        
        # Initialize CUDA context properly
        cuda.init()
        self.cuda_ctx = cuda.Device(0).make_context()
        
        # TensorRT initialization
        self.logger = trt.Logger(trt.Logger.WARNING)
        self.runtime = trt.Runtime(self.logger)

        with open(model_path, "rb") as f:
            engine_data = f.read()
        self.engine = self.runtime.deserialize_cuda_engine(engine_data)
        self.context = self.engine.create_execution_context()

        # Bindings
        self.input_binding_idx = self.engine.get_binding_index("images")
        self.output_binding_idx = self.engine.get_binding_index(self.engine[1])
        self.output_shape = self.engine.get_binding_shape(self.output_binding_idx)

        self.input_dtype = trt.nptype(self.engine.get_binding_dtype(self.input_binding_idx))
        self.output_dtype = trt.nptype(self.engine.get_binding_dtype(self.output_binding_idx))

        # Allocate memory with proper context
        self.input_host = cuda.pagelocked_empty(trt.volume((1, 3, *input_shape)), dtype=np.float32)
        self.output_host = cuda.pagelocked_empty(trt.volume(self.output_shape), dtype=self.output_dtype)
        self.input_device = cuda.mem_alloc(self.input_host.nbytes)
        self.output_device = cuda.mem_alloc(self.output_host.nbytes)
        self.stream = cuda.Stream()

    def _init_pytorch_fallback(self, model_path):
        """Fallback to PyTorch YOLO if TensorRT fails"""
        from ultralytics import YOLO
        
        # Try to load PyTorch version
        pt_path = model_path.replace(".engine", ".pt")
        try:
            self.model = YOLO(pt_path)
            self.device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
            self.names = self.model.names
            print(f"[YOLOModel] PyTorch fallback model loaded from: {pt_path}")
        except Exception as e:
            raise RuntimeError(f"[YOLOModel] Both TensorRT and PyTorch models failed: {e}")

    def __del__(self):
        """Cleanup CUDA context"""
        if hasattr(self, 'cuda_ctx') and self.cuda_ctx:
            self.cuda_ctx.pop()
            self.cuda_ctx = None

    def preprocess(self, image):
        """Preprocess image for TensorRT inference"""
        if self.engine_type != "tensorrt":
            return  # PyTorch handles preprocessing internally
            
        img = cv2.resize(image, self.input_shape)
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))  # HWC to CHW
        img = np.expand_dims(img, axis=0)  # Add batch dimension
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
        detections = []
        preds = output[0] if len(output.shape) == 3 else output
        
        # Use original image dimensions for coordinate conversion (not input_shape)
        img_h, img_w = self.original_h, self.original_w
        input_h, input_w = self.input_shape[1], self.input_shape[0]  # Model input size (640x640)
        
        print(f"[DEBUG] TensorRT postprocess - Image: {img_w}x{img_h}, Model input: {input_w}x{input_h}")

        for pred in preds:
            if len(pred) >= 5:
                conf = pred[4]
                if conf > conf_thres:
                    # TensorRT output coordinates
                    x_center_raw, y_center_raw, w_raw, h_raw = pred[0:4]

                    # Step 1: From model output (640x640) → original image (1280x720)
                    scale_x = img_w / self.input_shape[0]  # 1280 / 640 = 2.0
                    scale_y = img_h / self.input_shape[1]  # 720 / 640 = 1.125

                    x_center_img = x_center_raw * scale_x
                    y_center_img = y_center_raw * scale_y
                    width_img = w_raw * scale_x
                    height_img = h_raw * scale_y

                    # Step 2: Maze boundary check
                    if not (430 <= x_center_img <= 1085 and 27 <= y_center_img <= 682):
                        print(f"[DEBUG] ✗ Outside maze bounds: x={x_center_img:.1f}, y={y_center_img:.1f}")
                    else:
                        print(f"[DEBUG] ✓ Inside maze bounds")

                    # Step 3: Optional maze-relative position (for logic use, not bounding boxes)
                    maze_x = x_center_img - 430
                    maze_y = y_center_img - 27
                    print(f"[DEBUG] Maze-relative coords: x={maze_x:.1f}, y={maze_y:.1f}")

                    # Step 4: Bounding box corners in image coordinates
                    x1 = x_center_img - width_img / 2
                    y1 = y_center_img - height_img / 2
                    x2 = x_center_img + width_img / 2
                    y2 = y_center_img + height_img / 2

                    detections.append({
                        "bbox_xyxy": [x1, y1, x2, y2],
                        "bbox_xywh": [x_center_img, y_center_img, width_img, height_img],
                        "confidence": float(conf),
                        "class_id": 0
                    })


        class Result:
            def __init__(self, detections):
                self.boxes = []
                for det in detections:
                    box = type('Box', (), {})()
                    # Add both formats for compatibility
                    box.xyxy = np.array([[*det["bbox_xyxy"]]])
                    box.xywh = np.array([[*det["bbox_xywh"]]])  # Now in pixel coordinates relative to original image
                    box.conf = np.array([det["confidence"]])
                    box.cls = np.array([det["class_id"]])
                    self.boxes.append(box)

        return Result(detections)

    def get_label(self, cls_id):
        if self.engine_type == "tensorrt":
            return self.names[int(cls_id)]
        else:
            return self.names[int(cls_id)]