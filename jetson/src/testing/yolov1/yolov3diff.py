import cv2
import numpy as np
from ultralytics import YOLO

# === CONFIGURATION ===
CONF_THRESHOLD = 0.6          # Only show detections with confidence >= this
COLOR_FILTER = 'red'           # Choose: 'red', 'green', 'blue', 'yellow'
RED_THRESH = 0.5          # % of pixels in box that must be the chosen color

# === COLOR FILTERING ===
def is_color_dominant(frame, box, color='red', thresh=0.25):
    x1, y1, x2, y2 = map(int, box)
    roi = frame[y1:y2, x1:x2]
    if roi.size == 0:
        return False

    roi = roi.astype(np.float32)
    B, G, R = cv2.split(roi)
    total = R + G + B + 1e-6

    red_ratio = R / total
    green_ratio = G / total
    blue_ratio = B / total

    if color == 'red':
        color_mask = (red_ratio > 0.45) & (red_ratio > green_ratio) & (red_ratio > blue_ratio)
    elif color == 'green':
        color_mask = (green_ratio > 0.45) & (green_ratio > red_ratio) & (green_ratio > blue_ratio)
    elif color == 'blue':
        color_mask = (blue_ratio > 0.45) & (blue_ratio > red_ratio) & (blue_ratio > green_ratio)
    elif color == 'yellow':
        color_mask = (red_ratio > 0.35) & (green_ratio > 0.35) & (blue_ratio < 0.3)
    else:
        return False  # Unsupported color

    fraction = np.sum(color_mask) / roi.size * 3  # 3 channels
    return fraction > thresh

# === YOLO INFERENCE + FILTERING ===
def detect_colored_objects(model, frame, color='red', conf_thresh=0.3):
    results = model.predict(frame, verbose=False, imgsz=640)[0]
    detections = []

    for box in results.boxes:
        conf = float(box.conf[0].item())
        if conf < conf_thresh:
            continue

        x1, y1, x2, y2 = map(int, box.xyxy[0])
        if is_color_dominant(frame, (x1, y1, x2, y2), color=color, thresh=RED_THRESH):
            center = ((x1 + x2) // 2, (y1 + y2) // 2)
            detections.append((x1, y1, x2, y2, center, conf))

    return detections

# === MAIN LOOP ===
def main():
    model = YOLO("best.pt")
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Cannot open camera.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error reading frame.")
            break

        detections = detect_colored_objects(model, frame, color=COLOR_FILTER, conf_thresh=CONF_THRESHOLD)

        for (x1, y1, x2, y2, center, conf) in detections:
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
            cv2.putText(frame, f"{COLOR_FILTER.capitalize()}: {conf:.2f}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        cv2.imshow("YOLOv8 Color Filtered Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
