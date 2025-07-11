# Ball Detection

This part provides an explanation of the ball tracking system which combines object detection via YOLOv8 with real-time color-based tracking using the HSV color space. It is designed for fast visual feedback, and is implemented using modular Python components with threaded architecture for performance. This pipeline is modular and highly tunable, meaning if there are any issues then variables can easily be changed.

---

## 1. System Architecture

The pipeline consists of these components:

- `CameraManager`: Interfaces with the ZED stereo camera to acquire RGB/BGR frames and retrieve orientation data.
- `YOLOModel`: Loads and runs YOLOv8 for object detection.
- `BallTracker`: The core of the tracking system, responsible for combining YOLO and HSV tracking.
- `vision_utils`: Contains utility functions for color tracking using OpenCV.
- `TrackerService`: A unified interface for the rest of the application to interact with tracking functionality.

All components are developed to maintain high responsiveness and frame stability using a producer-consumer threading pattern.

---

## 2. Camera Acquisition (`CameraManager`)

The ZED camera is initialized with fixed parameters for resolution and frame rate:

```python
init_params.camera_resolution = sl.RESOLUTION.HD720
init_params.camera_fps = 60
init_params.depth_mode = sl.DEPTH_MODE.NONE
```

Frame acquisition is handled by:

```python
def grab_frame(self):
    if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
        self.zed.retrieve_image(image, sl.VIEW.LEFT)
        bgr = image.get_data()
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        return rgb, bgr
    return None, None
```

Orientation data is retrieved from the IMU:

```python
def get_orientation(self):
    imu_data = sensors_data.get_imu_data()
    orientation = imu_data.get_pose(zed_imu_pose).get_orientation().get()
    return [-orientation[1] + orientation[2], orientation[0] + orientation[3]]
```

---

## 3. Object Detection (`YOLOModel`)

The YOLOv8 model is loaded and prepared at runtime:

```python
self.model = YOLO(model_path)
self.model.fuse()
self.model.eval()
```

Before inference begins, the model is pre-warmed using a dummy frame:

```python
dummy = np.zeros((720, 1280, 3), dtype=np.uint8)
self.model.predict(dummy, verbose=False)
```

Predictions are returned as:

```python
results = self.model.predict(image, conf=0.6)[0]
```

Each bounding box can be interpreted via:

```python
label = self.model.get_label(box.cls[0])
```

---

## 4. Ball Tracking (`BallTracker`)

The core tracking class runs two threads:
- `producer_loop`: Continuously updates the latest RGB/BGR frames from the camera.
- `consumer_loop`: Processes those frames to track the ball.

### Initialization Phase

When the system starts, the ball must be detected by YOLO within a predefined region:

```python
if self.INIT_BALL_REGION[0][0] <= cx <= self.INIT_BALL_REGION[1][0] and ...
```

This makes sure that the ball is only initialized if it appears in a valid location, avoiding false starts caused by noise outside of the maze.

### HSV Tracking Phase

Once the ball is initialized, frame-to-frame tracking switches to HSV. A region around the last known position is used:

```python
def hsv_tracking(frame, prev_pos, hsv_lower, hsv_upper, window_size=80):
    roi = frame[y_min:y_max, x_min:x_max]
    mask = cv2.inRange(cv2.cvtColor(roi, cv2.COLOR_BGR2HSV), hsv_lower, hsv_upper)
    return get_center_of_mass(mask)
```

To improve robustness, position smoothing is applied:

```python
alpha = 0.5
x = int(alpha * new_pos[0] + (1 - alpha) * self.ball_position[0])
y = int(alpha * new_pos[1] + (1 - alpha) * self.ball_position[1])
self.ball_position = (x, y)
```

---

## 5. Fallback and Recovery

If HSV fails for several consecutive frames (defined by `self.hsv_fail_threshold`), the system triggers a fallback process:

1. A global HSV scan (`global_hsv_search`) checks for color presence in the full frame.
2. If a candidate is found, YOLO is re-run to validate the detection.
3. A cooldown (`self.yolo_cooldown`) prevents YOLO from running every frame.

Example fallback logic:

```python
if self.hsv_fail_counter >= self.hsv_fail_threshold and self.yolo_cooldown == 0:
    global_pos = global_hsv_search(bgr, *self.HSV_RANGE)
    if global_pos:
        results = self.model.predict(rgb)
        # updates the ball position if confirmed
```

---

## 6. TrackerService (`tracker_service`)

The `TrackerService` class simplifies external control and access:

```python
service = TrackerService()
service.start_tracker()
position = service.get_ball_position()
frame = service.get_stable_frame()
orientation = service.get_orientation()
service.retrack()  # force re-detection
```

---

## 7. Vision Utilities (`vision_utils`)

These functions are used internally for HSV-based tracking and position estimation.

### Center of Mass

```python
def get_center_of_mass(mask):
    contours = cv2.findContours(...)[0]
    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < min_area:
        return None
    return (cx, cy)
```

### Global Search

```python
def global_hsv_search(frame, hsv_lower, hsv_upper):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
    return get_center_of_mass(mask)
```

### Local Windowed Search

```python
def hsv_tracking(frame, prev_pos, hsv_lower, hsv_upper, window_size=80):
    roi = extract around prev_pos
    mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
    return get_center_of_mass(mask)
```

---

## 8. Known Issues and Tuning Guide

### hsv_fail_threshold

Controls how many consecutive HSV tracking failures must occur before falling back to global search and YOLO detection.

```python
self.hsv_fail_threshold = 30  # default as it shows a good balance (0.5s at 60 FPS)
```

Increase if you’re experiencing too frequent fallback, decrease for faster recovery.

### yolo_cooldown_period

Defines how many frames to wait after a YOLO attempt before allowing another.

```python
self.yolo_cooldown_period = 15  # ~0.25s cooldown
```

Lowering this increases responsiveness at the cost of more frequent inference.

### min_area

Used in `get_center_of_mass()` to ignore noise in the HSV mask.

```python
min_area = 150  # or higher for higher-resolution cameras
```

Raise this to avoid tracking small irrelevant blobs.

### smoothing factor (alpha)

Applies exponential smoothing to reduce jumpiness:

```python
alpha = 0.5  # lower = smoother, higher = faster reaction
```

Tune this based on the expected ball motion and latency tolerance.

### Lighting sensitivity

The HSV thresholds are susceptible to changing light conditions. Tune these empirically:

```python
self.HSV_RANGE = (
    np.array([35, 80, 80]),  # HSV lower bound for green
    np.array([85, 255, 255]) # HSV upper bound for green
)
```

These can be changed for different ball colors, but it is important to remember that the YOLOv8 model is mostly trained on green balls, and green is what works the best from all values in the HSV space.