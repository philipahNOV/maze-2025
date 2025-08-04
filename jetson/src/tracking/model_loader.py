import numpy as np
import cv2
import tensorrt as trt
import pycuda.driver as cuda
import torch

class YOLOModel:
    def __init__(self, model_path="new-v8-fp16.engine", input_shape=(640, 640)):
        self.input_shape = input_shape
        self.names = ['ball']
        self.original_h, self.original_w = 640, 640

        cuda.init()  # manual init, no pycuda.autoinit
        self.cuda_ctx = cuda.Device(0).make_context()

        self.logger = trt.Logger(trt.Logger.WARNING)
        self.runtime = trt.Runtime(self.logger)

        # Load TensorRT engine
        with open(model_path, "rb") as f:
            engine_data = f.read()
        self.engine = self.runtime.deserialize_cuda_engine(engine_data)
        self.context = self.engine.create_execution_context()

        # Bindings
        self.input_binding_idx = self.engine.get_binding_index("images")
        self.output_binding_idx = self.engine.get_binding_index(self.engine[1])
        self.output_shape = self.engine.get_binding_shape(self.output_binding_idx)

        # Data types
        self.input_dtype = np.float16
        self.output_dtype = trt.nptype(self.engine.get_binding_dtype(self.output_binding_idx))

        # Memory buffers
        self.input_host = cuda.pagelocked_empty(trt.volume((1, 3, *input_shape)), dtype=self.input_dtype)
        self.output_host = cuda.pagelocked_empty(trt.volume(self.output_shape), dtype=self.output_dtype)
        self.input_device = cuda.mem_alloc(self.input_host.nbytes)
        self.output_device = cuda.mem_alloc(self.output_host.nbytes)
        self.stream = cuda.Stream()

    def __del__(self):
        if hasattr(self, 'cuda_ctx') and self.cuda_ctx:
            self.cuda_ctx.pop()
            self.cuda_ctx = None

    def preprocess(self, image):
        input_w, input_h = self.input_shape
        img = cv2.resize(image, (input_w, input_h))
        img = img.astype(np.float16) / 255.0
        img = np.transpose(img, (2, 0, 1))  # HWC to CHW
        img = np.expand_dims(img, axis=0)  # Add batch dimension
        assert self.input_host.size == img.size, "Input size mismatch"
        np.copyto(self.input_host, img.ravel())

    def predict(self, image, conf=0.35):
        if not self.cuda_ctx:
            raise RuntimeError("CUDA context not initialized")
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

    def postprocess(self, output, conf_thres=0.35):
        output = output.squeeze()

        if output.shape[0] != 5:
            return self._empty_result()

        x_center, y_center, width, height, conf = output

        mask = conf >= conf_thres
        if not np.any(mask):
            return self._empty_result()

        x_center = x_center[mask]
        y_center = y_center[mask]
        width = width[mask]
        height = height[mask]
        conf = conf[mask]

        x1 = x_center - width / 2
        y1 = y_center - height / 2
        x2 = x_center + width / 2
        y2 = y_center + height / 2

        scale_x = self.original_w / self.input_shape[0]
        scale_y = self.original_h / self.input_shape[1]

        x1 *= scale_x
        x2 *= scale_x
        y1 *= scale_y
        y2 *= scale_y

        cls = np.zeros_like(conf)

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

        boxes = [Box(x1[i], y1[i], x2[i], y2[i], conf[i], cls[i]) for i in range(len(conf))]
        return Result(boxes)

    def _empty_result(self):
        class EmptyResult:
            def __init__(self):
                self.boxes = []
        return EmptyResult()

    def get_label(self, cls_id):
        return self.names[int(cls_id)]
    
    def shutdown(self):
        print("[YOLOModel] Starting shutdown...")
        try:
            # Free device memory
            if hasattr(self, 'input_device') and self.input_device is not None:
                self.input_device.free()  # Free via object method
                self.input_device = None

            if hasattr(self, 'output_device') and self.output_device is not None:
                self.output_device.free()
                self.output_device = None

            # Delete TensorRT objects
            if hasattr(self, 'context') and self.context is not None:
                del self.context
                self.context = None

            if hasattr(self, 'engine') and self.engine is not None:
                del self.engine
                self.engine = None

            if hasattr(self, 'runtime') and self.runtime is not None:
                del self.runtime
                self.runtime = None

            # Delete CUDA stream
            if hasattr(self, 'stream') and self.stream is not None:
                del self.stream
                self.stream = None

            # Detach CUDA context
            if hasattr(self, 'cuda_ctx') and self.cuda_ctx is not None:
                self.cuda_ctx.pop()
                self.cuda_ctx = None

            print("[YOLOModel] Shutdown completed successfully")

        except Exception as e:
            print(f"[YOLOModel] Error during shutdown: {e}")

