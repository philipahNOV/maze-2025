import cv2
import numpy as np
import pyzed.sl as sl
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit  # This automatically initializes CUDA driver
import ctypes

class YOLOv8TRT:
    def __init__(self, engine_path, input_shape=(640, 640)):
        self.input_shape = input_shape
        self.logger = trt.Logger(trt.Logger.INFO)
        self.runtime = trt.Runtime(self.logger)

        # Load engine
        with open(engine_path, "rb") as f:
            engine_data = f.read()
        self.engine = self.runtime.deserialize_cuda_engine(engine_data)
        self.context = self.engine.create_execution_context()

        self.input_binding_idx = self.engine.get_binding_index("images")  # Change if your input name is different
        self.output_binding_idx = self.engine.get_binding_index(self.engine[1])  # Assuming one output
        self.allocate_buffers()

    def allocate_buffers(self):
        input_shape = (1, 3, *self.input_shape)
        output_shape = (1, 84, 8400)  # Change this based on your model output

        self.input_host = cuda.pagelocked_empty(np.prod(input_shape), dtype=np.float32)
        self.output_host = cuda.pagelocked_empty(np.prod(output_shape), dtype=np.float32)

        self.input_device = cuda.mem_alloc(self.input_host.nbytes)
        self.output_device = cuda.mem_alloc(self.output_host.nbytes)

        self.bindings = [int(self.input_device), int(self.output_device)]
        self.stream = cuda.Stream()

    def preprocess(self, image):
        img_resized = cv2.resize(image, self.input_shape)
        self.scale_w = image.shape[1] / self.input_shape[0]
        self.scale_h = image.shape[0] / self.input_shape[1]

        img = img_resized.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))
        img = np.expand_dims(img, axis=0)
        return img

    def postprocess(self, outputs, conf_thres=0.4):
        outputs = outputs.reshape(-1, outputs.shape[-1])
        detections = []
        for pred in outputs:
            if len(pred) >= 5:
                conf = pred[4]
                if conf > conf_thres:
                    x, y, w, h = pred[0:4]
                    x1 = int((x - w / 2) * self.scale_w)
                    y1 = int((y - h / 2) * self.scale_h)
                    x2 = int((x + w / 2) * self.scale_w)
                    y2 = int((y + h / 2) * self.scale_h)

                    detections.append({
                        "bbox": [max(0, x1), max(0, y1), min(int(self.input_shape[0] * self.scale_w), x2), min(int(self.input_shape[1] * self.scale_h), y2)],
                        "confidence": float(conf),
                        "class_id": 0
                    })
        return detections

    def predict(self, image):
        if image.shape[2] == 4:
            image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

        input_tensor = self.preprocess(image)
        np.copyto(self.input_host, input_tensor.ravel())

        cuda.memcpy_htod_async(self.input_device, self.input_host, self.stream)
        self.context.execute_async_v2(self.bindings, self.stream.handle, None)
        cuda.memcpy_dtoh_async(self.output_host, self.output_device, self.stream)
        self.stream.synchronize()

        return self.postprocess(self.output_host)


# ----------------------------
# Initialize ZED in CustomBox Mode
# ----------------------------
def init_zed():
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.depth_mode = sl.DEPTH_MODE.ULTRA
    init_params.coordinate_units = sl.UNIT.METER

    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("Failed to open ZED")
        exit(1)

    tracking_params = sl.PositionalTrackingParameters()
    zed.enable_positional_tracking(tracking_params)

    obj_params = sl.ObjectDetectionParameters()
    obj_params.detection_model = sl.OBJECT_DETECTION_MODEL.CUSTOM_BOX_OBJECTS
    obj_params.enable_tracking = True
    zed.enable_object_detection(obj_params)

    return zed

# ----------------------------
# Main Tracking Loop
# ----------------------------
def main():
    zed = init_zed()
    runtime_params = sl.ObjectDetectionRuntimeParameters()
    objects = sl.Objects()

    # IMPORTANT: Use the correct model and input size for ball detection
    MODEL_INPUT_SHAPE = (1280, 1280)  # Much faster than 1280x1280
    MODEL_PATH = "zed_test/v8-291.trt"  # Back to the ball-trained model

    yolo = YOLOv8TRT(MODEL_PATH, input_shape=MODEL_INPUT_SHAPE)

    cv2.namedWindow("ZED Ball Tracker", cv2.WINDOW_NORMAL)

    while True:
        if zed.grab() != sl.ERROR_CODE.SUCCESS:
            continue

        zed_image = sl.Mat()
        zed.retrieve_image(zed_image, sl.VIEW.LEFT)
        image = zed_image.get_data()
        display = image.copy()
        h, w = image.shape[:2]

        detections = yolo.predict(image)

        objects_in = []
        for det in detections:
            x1, y1, x2, y2 = [max(0, min(v, w if i % 2 == 0 else h)) for i, v in enumerate(det["bbox"])]
            obj = sl.CustomBoxObjectData()
            obj.bounding_box_2d = np.array([
                    [x1, y1],
                    [x2, y1],
                    [x2, y2],
                    [x1, y2]
                ], dtype=np.float32)

            obj.label = 0
            obj.probability = det["confidence"]
            obj.unique_object_id = sl.generate_unique_id()
            obj.is_grounded = True
            objects_in.append(obj)

        zed.ingest_custom_box_objects(objects_in)

        zed.retrieve_objects(objects, runtime_params)
        for obj in objects.object_list:
            if obj.tracking_state == sl.OBJECT_TRACKING_STATE.OK:
                pos = obj.position
                x, y, z = pos
                label = f"ID:{obj.id} X:{x:.2f} Y:{y:.2f} Z:{z:.2f}m"

                tl = obj.bounding_box_2d[0]
                br = obj.bounding_box_2d[2]
                cv2.rectangle(display, (int(tl[0]), int(tl[1])), (int(br[0]), int(br[1])), (0, 255, 0), 2)
                cv2.putText(display, label, (int(tl[0]), int(tl[1]) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        cv2.imshow("ZED Ball Tracker", display)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    zed.disable_object_detection()
    zed.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
