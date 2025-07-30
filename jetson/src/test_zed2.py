import cv2
import numpy as np
import pyzed.sl as sl
import onnxruntime as ort

# ----------------------------
# YOLOv8 ONNX Loader (1-Class)
# ----------------------------
class YOLOv8ONNX:
    def __init__(self, model_path, input_shape=(640, 640)):
        self.session = ort.InferenceSession(model_path)
        self.input_name = self.session.get_inputs()[0].name
        self.input_shape = input_shape  # (width, height)

    def preprocess(self, image):
        # Resize to model input shape
        img_resized = cv2.resize(image, self.input_shape)

        # Save scale for post-processing
        self.scale_w = image.shape[1] / self.input_shape[0]
        self.scale_h = image.shape[0] / self.input_shape[1]

        # Normalize and format
        img = img_resized.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))  # HWC to CHW
        img = np.expand_dims(img, axis=0)  # Add batch dim
        return img

    def postprocess(self, outputs, conf_thres=0.4):
        detections = []
        preds = outputs[0]
        if len(preds.shape) == 3:
            preds = preds[0]

        for pred in preds:
            if len(pred) >= 5:
                conf = pred[4]
                if conf > conf_thres:
                    x, y, w, h = pred[0:4]
                    x1 = int((x - w / 2) * self.scale_w)
                    y1 = int((y - h / 2) * self.scale_h)
                    x2 = int((x + w / 2) * self.scale_w)
                    y2 = int((y + h / 2) * self.scale_h)

                    x1 = max(0, x1)
                    y1 = max(0, y1)
                    x2 = min(int(self.input_shape[0] * self.scale_w), x2)
                    y2 = min(int(self.input_shape[1] * self.scale_h), y2)

                    detections.append({
                        "bbox": [x1, y1, x2, y2],
                        "confidence": float(conf),
                        "class_id": 0
                    })
        return detections

    def predict(self, image):
        # Convert RGBA to RGB if needed
        if image.shape[2] == 4:
            image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

        input_tensor = self.preprocess(image)
        outputs = self.session.run(None, {self.input_name: input_tensor})
        return self.postprocess(outputs)

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

    # IMPORTANT: Set this to match your ONNX input size!
    MODEL_INPUT_SHAPE = (1280, 1280)  # or (1280, 1280) if that's your model's export size
    MODEL_PATH = "tracking/simple.onnx"

    yolo = YOLOv8ONNX(MODEL_PATH, input_shape=MODEL_INPUT_SHAPE)
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
            obj.bounding_box_2d = [
                sl.PySL_Pixel(x1, y1),
                sl.PySL_Pixel(x2, y1),
                sl.PySL_Pixel(x2, y2),
                sl.PySL_Pixel(x1, y2)
            ]
            obj.label = 0
            obj.raw_label = 0
            obj.probability = det["confidence"]
            obj.unique_object_id = sl.generate_unique_id()
            obj.is_grounded = True
            objects_in.append(obj)

        zed.ingest_custom_box_objects(objects_in)

        zed.retrieve_objects(objects, runtime_params)
        for obj in objects.object_list:
            if obj.tracking_state == sl.OBJECT_TRACKING_STATE.OK:
                pos = obj.position
                x, y, z = pos.get()
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
