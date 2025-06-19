import cv2
import numpy as np
from ultralytics import YOLO
import time

def center_difference(prev_center, current_center):
    if prev_center is None or current_center is None:
        return None
    return np.linalg.norm(np.array(current_center) - np.array(prev_center))

def detect_ball_yolo(model, frame, prev_center=None, max_diff=300):
    results = model.predict(frame, verbose=False, imgsz=640)[0]
    best_box = None
    best_conf = 0

    for box in results.boxes:
        cls_id = int(box.cls[0].item())
        conf = float(box.conf[0].item())
        if conf < 0.3:
            continue

        x1, y1, x2, y2 = map(int, box.xyxy[0])
        center = ((x1 + x2) // 2, (y1 + y2) // 2)

        if prev_center is not None:
            if center_difference(prev_center, center) > max_diff:
                continue

        if conf > best_conf:
            best_box = (x1, y1, x2, y2, center)
            best_conf = conf

    return best_box

def main():
    model = YOLO("best.pt")
    cap = cv2.VideoCapture(0)  # Use webcam; change to 1 or 2 for different camera

    if not cap.isOpened():
        print("Error: Cannot open camera.")
        return

    prev_center = None

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error reading frame.")
            break

        result = detect_ball_yolo(model, frame, prev_center)

        if result:
            x1, y1, x2, y2, center = result
            prev_center = center
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
            cv2.putText(frame, f"Ball", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.imshow("YOLO Ball Tracker", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()