import cv2
import numpy as np
from ultralytics import YOLO

def is_red_dominant(frame, box, red_thresh=0.6):
    x1, y1, x2, y2 = map(int, box)
    roi = frame[y1:y2, x1:x2]
    if roi.size == 0:
        return False

    # Convert to float for ratio calculation
    roi = roi.astype(np.float32)
    B, G, R = cv2.split(roi)
    total = R + G + B + 1e-6  # prevent divide-by-zero
    red_ratio = R / total
    red_pixels = np.sum(red_ratio > 0.5)  # red-dominant pixels
    total_pixels = roi.shape[0] * roi.shape[1]
    fraction_red = red_pixels / total_pixels

    return fraction_red > red_thresh

def detect_red_marbles(model, frame):
    results = model.predict(frame, verbose=False, imgsz=640)[0]
    detections = []

    for box in results.boxes:
        conf = float(box.conf[0].item())
        if conf < 0.2:
            continue

        x1, y1, x2, y2 = map(int, box.xyxy[0])

        if is_red_dominant(frame, (x1, y1, x2, y2)):
            center = ((x1 + x2) // 2, (y1 + y2) // 2)
            detections.append((x1, y1, x2, y2, center, conf))

    return detections

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

        detections = detect_red_marbles(model, frame)

        for (x1, y1, x2, y2, center, conf) in detections:
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
            cv2.putText(frame, f"Ball: {conf:.2f}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        cv2.imshow("YOLOv8 Red Marble Detector", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
