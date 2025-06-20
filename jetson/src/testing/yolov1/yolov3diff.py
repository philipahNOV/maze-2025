import cv2
import numpy as np
from ultralytics import YOLO

# === CONFIGURATION ===
TARGET_COLOR = 'red'     #'red', 'green'
CONF_THRESHOLD = 0.75
COLOR_THRESH = 0.2

COLOR_THRESHOLDS = {
    'red': {'r': 0.45, 'g': 0.0, 'b': 0.0},
    'green': {'g': 0.25, 'r': 0.0, 'b': 0.0},
    'blue': {'b': 0.25, 'r': 0.0, 'g': 0.0},
    'yellow': {'r': 0.35, 'g': 0.35, 'b': 0.3},
}

# === COLOR FILTERING ===
def is_color_dominant(frame, box, color, thresh):
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

    t = COLOR_THRESHOLDS[color]
    if color == 'yellow':
        mask = (red_ratio > t['r']) & (green_ratio > t['g']) & (blue_ratio < t['b'])
    else:
        mask = True
        if 'r' in t: mask &= red_ratio > t['r']
        if 'g' in t: mask &= green_ratio > t['g']
        if 'b' in t: mask &= blue_ratio > t['b']
        if color == 'red':   mask &= (red_ratio > green_ratio) & (red_ratio > blue_ratio)
        if color == 'green': mask &= (green_ratio > red_ratio) & (green_ratio > blue_ratio)
        if color == 'blue':  mask &= (blue_ratio > red_ratio) & (blue_ratio > green_ratio)

    dominant_fraction = np.sum(mask) / (roi.shape[0] * roi.shape[1])
    return dominant_fraction > thresh

# === DETECTION ===
def detect_colored_objects(model, frame, color, conf_thresh, color_thresh):
    results = model.predict(frame, verbose=False, imgsz=640)[0]
    detections = []

    for box in results.boxes:
        conf = float(box.conf[0].item())
        if conf < conf_thresh:
            continue

        x1, y1, x2, y2 = map(int, box.xyxy[0])
        if is_color_dominant(frame, (x1, y1, x2, y2), color=color, thresh=color_thresh):
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

        detections = detect_colored_objects(model, frame, TARGET_COLOR, CONF_THRESHOLD, COLOR_THRESH)

        for (x1, y1, x2, y2, center, conf) in detections:
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
            cv2.putText(frame, f"{TARGET_COLOR}: {conf:.2f}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        cv2.imshow("some shit", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()