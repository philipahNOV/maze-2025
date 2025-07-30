import cv2
import numpy as np
import pyzed.sl as sl
import onnxruntime as ort

# ----------------------------
# YOLOv8 ONNX: 1-class model
# ----------------------------
class YOLOv8ONNX:
    def __init__(self, model_path, input_shape=(640, 640)):
        self.session = ort.InferenceSession(model_path)
        self.input_name = self.session.get_inputs()[0].name
        self.input_shape = input_shape

    def preprocess(self, image):
        img = cv2.resize(image, self.input_shape)
        self.scale_w = image.shape[1] / self.input_shape[0]
        self.scale_h = image.shape[0] / self.input_shape[1]
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))[np.newaxis, ...]  # CHW
        return img

    def postprocess(self, outputs, conf_thres=0.4):
        detections = []
        for pred in outputs[0]:
            conf = pred[4]
            if conf > conf_thres:
                class_id = int(np.argmax(pred[5:]))
                if class_id != 0:
                    continue  # ONLY CLASS 0 (ball)
                x, y, w, h = pred[0:4]
                x1 = int((x - w / 2) * self.scale_w)
                y1 = int((y - h / 2) * self.scale_h)
                x2 = int((x + w / 2) * self.scale_w)
                y2 = int((y + h / 2) * self.scale_h)
                detections.append({
                    "bbox": [x1, y1, x2, y2],
                    "confidence": float(conf),
                    "class_id": class_id
                })
        return detections

    def predict(self, image):
        img_input = self.preprocess(image)
        outputs = self.session.run(None, {self.input_name: img_input})
        return self.postprocess(outputs)

# ----------------------------
# Initialize ZED
# ----------------------------
def init_zed():
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.depth_mode = sl.DEPTH_MODE.ULTRA
    init_params.coordinate_units = sl.UNIT.METER
    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("ZED failed to open.")
        exit(1)

    # Enable tracking
    tracking_params = sl.PositionalTrackingParameters()
    zed.enable_positional_tracking(tracking_params)

    # Enable custom detection
    obj_params = sl.ObjectDetectionParameters()
    obj_params.detection_model = sl.OBJECT_DETECTION_MODEL.CUSTOM_BOX_OBJECTS
    obj_params.enable_tracking = True
    zed.enable_object_detection(obj_params)

    return zed

# ----------------------------
# Main Loop
# ----------------------------
def main():
    zed = init_zed()
    runtime_params = sl.ObjectDetectionRuntimeParameters()
    objects = sl.Objects()

    # Load YOLOv8 ONNX
    yolo = YOLOv8ONNX("tracking/v8-291.onnx", input_shape=(640, 640))

    cv2.namedWindow("ZED Ball Tracking", cv2.WINDOW_NORMAL)

    while True:
        if zed.grab() != sl.ERROR_CODE.SUCCESS:
            continue

        # Get image from ZED
        image_zed = sl.Mat()
        zed.retrieve_image(image_zed, sl.VIEW.LEFT)
        image_np = image_zed.get_data()
        image_disp = image_np.copy()

        # Run YOLOv8
        detections = yolo.predict(image_np)

        # Ingest custom boxes to ZED
        custom_objects = []
        for det in detections:
            x1, y1, x2, y2 = det["bbox"]
            h, w = image_np.shape[:2]
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w - 1, x2), min(h - 1, y2)

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
            custom_objects.append(obj)

        zed.ingest_custom_box_objects(custom_objects)

        # Retrieve tracked objects
        zed.retrieve_objects(objects, runtime_params)
        for obj in objects.object_list:
            if obj.tracking_state == sl.OBJECT_TRACKING_STATE.OK:
                pos = obj.position
                x, y, z = pos.get()
                label = f"ID:{obj.id} X:{x:.2f} Y:{y:.2f} Z:{z:.2f}m"
                print(label)

                # Draw box
                tl = obj.bounding_box_2d[0]
                br = obj.bounding_box_2d[2]
                cv2.rectangle(image_disp, (int(tl[0]), int(tl[1])), (int(br[0]), int(br[1])), (0, 255, 0), 2)
                cv2.putText(image_disp, label, (int(tl[0]), int(tl[1] - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        cv2.imshow("ZED Ball Tracking", image_disp)
        key = cv2.waitKey(1)
        if key == ord('q'):
            break

    zed.disable_object_detection()
    zed.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
