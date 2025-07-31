import numpy as np
import cv2
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit

class YOLOModel:
    def __init__(self, model_path="new-v8-fp16.engine", input_shape=(640, 640)):
        print(f"[YOLOModel] Loading raw TensorRT engine from: {model_path}")
        self.input_shape = input_shape  # (W, H)
        self.names = ['ball']  # Manually set class names

        # TensorRT boilerplate
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

        self.input_host = cuda.pagelocked_empty(trt.volume((1, 3, *input_shape)), dtype=np.float32)
        self.output_host = cuda.pagelocked_empty(trt.volume(self.output_shape), dtype=self.output_dtype)
        self.input_device = cuda.mem_alloc(self.input_host.nbytes)
        self.output_device = cuda.mem_alloc(self.output_host.nbytes)
        self.stream = cuda.Stream()

        print(f"[YOLOModel] TensorRT engine initialized successfully.")

    def preprocess(self, image):
        img = cv2.resize(image, self.input_shape)
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))  # HWC to CHW
        img = np.expand_dims(img, axis=0)  # Add batch dimension
        np.copyto(self.input_host, img.ravel())

    def predict(self, image, conf=0.35):
        try:
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
            print(f"[YOLOModel] Inference failed: {e}")
            class EmptyResult:
                def __init__(self):
                    self.boxes = []
            return EmptyResult()

    def postprocess(self, output, conf_thres=0.35):
        detections = []
        preds = output[0] if len(output.shape) == 3 else output

        for pred in preds:
            if len(pred) >= 5:
                conf = pred[4]
                if conf > conf_thres:
                    x, y, w, h = pred[0:4]
                    x1 = x - w / 2
                    y1 = y - h / 2
                    x2 = x + w / 2
                    y2 = y + h / 2

                    detections.append({
                        "bbox": [x1, y1, x2, y2],
                        "confidence": float(conf),
                        "class_id": 0
                    })

        class Result:
            def __init__(self, detections):
                self.boxes = []
                for det in detections:
                    box = type('Box', (), {})()
                    box.xyxy = np.array([[*det["bbox"]]])
                    box.conf = np.array([det["confidence"]])
                    box.cls = np.array([det["class_id"]])
                    self.boxes.append(box)

        return Result(detections)

    def get_label(self, cls_id):
        return self.names[int(cls_id)]