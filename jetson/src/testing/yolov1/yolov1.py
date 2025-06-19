import cv2
import numpy as np
from ultralytics import YOLO
import time

def center_difference(prev_center, current_center):
    if prev_center is None or current_center is None:
        return None
    return np.linalg.norm(np.array(current_center) - np.array(prev_center))

def detect_balls_yolo(model, frame, prev_centers=None, max_diff=300, conf_thresh=0.4):
    results = model.predict(frame, verbose=False, imgsz=640)[0]
    detections = []

    for box in results.boxes:
        cls = int(box.cls[0].item())
        conf = float(box.conf[0].item())
        if conf < conf_thresh:
            continue

        x1, y1, x2, y2 = map(int, box.xyxy[0])
        center = ((x1 + x2) // 2, (y1 + y2) // 2)

        if prev_centers:
            if all(center_difference(prev, center) > max_diff for prev in prev_centers):
                continue

        detections.append((x1, y1, x2, y2, center, conf))

    return detections

def main():
    model = YOLO("best.pt")
    cap = cv2.VideoCapture(0)  # 0 for default webcam

    if not cap.isOpened():
        print("Error: Cannot open camera.")
        return

    prev_centers = []

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error reading frame.")
            break

        detections = detect_balls_yolo(model, frame, prev_centers)

        prev_centers = []
        for x1, y1, x2, y2, center, conf in detections:
            prev_centers.append(center)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
            label = f"Ball: {conf:.2f}"
            cv2.putText(frame, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        cv2.imshow("YOLO Ball Tracker", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
