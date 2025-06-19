import cv2
from ultralytics import YOLO
import numpy as np

def center_difference(p1, p2):
    if p1 is None or p2 is None:
        return None
    return np.linalg.norm(np.array(p1) - np.array(p2))

def detect_ball_with_yolo(video_path='output.mp4', model_path='best.pt'):
    model = YOLO(model_path)

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"Error opening video file: {video_path}")
        return

    prev_center = None
    frame_num = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("End of video or error reading frame.")
            break

        results = model(frame)[0]
        detections = results.boxes

        best_center = None
        best_radius = 0

        for box in detections:
            cls_id = int(box.cls[0].item())
            conf = float(box.conf[0].item())

            if conf < 0.4:  # conf
                continue

            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)
            w = int(x2 - x1)
            h = int(y2 - y1)
            radius = int((w + h) / 4)

            if prev_center is None or center_difference(prev_center, (cx, cy)) < 300:
                best_center = (cx, cy)
                best_radius = radius

        if best_center:
            prev_center = best_center
            cv2.circle(frame, best_center, best_radius, (0, 255, 0), 3)
            cv2.putText(frame, f"Ball", (best_center[0] - 10, best_center[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow("YOLO Ball Tracker", frame)
        frame_num += 1
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    detect_ball_with_yolo("output.mp4", "best.pt")
