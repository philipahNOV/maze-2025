import cv2

video = cv2.VideoCapture("output.mp4")
video.set(cv2.CAP_PROP_POS_FRAMES, 15)
ret, frame = video.read()
print(cv2.__version__)

# Initialize a bounding box around the object manually or from detection
bbox = cv2.selectROI("Frame", frame, False)
tracker = cv2.TrackerCSRT_create()
tracker.init(frame, bbox)

while True:
    ret, frame = video.read()
    if not ret:
        break

    success, bbox = tracker.update(frame)

    if success:
        (x, y, w, h) = [int(v) for v in bbox]
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
    else:
        cv2.putText(frame, "Tracking failed", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    cv2.imshow("Tracking", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

video.release()
cv2.destroyAllWindows()