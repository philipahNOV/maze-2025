# Autonomous Maze Solver – NOV 2025

This project is the final version of NOV’s autonomous maze-solving system, developed during Summer 2025. Building on earlier prototypes from previous years, the system is now complete and fully functional.

The goal is to balance a steel ball through a physical maze using camera-based tracking, real-time control, and path planning without falling into any holes. The system is designed to operate reliably and autonomously from end to end.

Key features:
- Real-time ball tracking using YOLOv8 and HSV-based refinement
- Path planning with A* using dynamic obstacle maps
- PID-based position control with Arduino-driven actuation
- MQTT communication between Jetson (control) and Raspberry Pi (HMI)

The system runs on NVIDIA Jetson and integrates with a ZED camera and Arduino-based tilt platform. It is configured through a single `config.yaml` file and is structured for maintainability and testing.

---

<summary><h3>Table of Contents</h3></summary>

- [Introduction](#introduction)
- [System Overview](#system-overview)
- [Setup & Configuration](#setup--configuration)
  - [3.1 Dependencies](#31-dependencies)
- [Run Controller](#run-controller)
  - [1. System Components](#1-system-components)
  - [2. Control Execution Flow](#2-control-execution-flow)
  - [3. Controller Configuration](#3-controller-configuration)
  - [4. Path Execution](#4-path-execution)
  - [5. Horizontal Calibration](#5-horizontal-calibration)
  - [6. MQTT Command Integration](#6-mqtt-command-integration)
  - [7. Troubleshooting](#7-troubleshooting)
  - [8. Tuning Tips](#8-tuning-tips)
- [Ball Detection](#ball-detection)
  - [1. System Architecture](#1-system-architecture)
  - [2. Camera Acquisition](#2-camera-acquisition-cameramanager)
  - [3. Object Detection](#3-object-detection-yolomodel)
  - [4. Ball Tracking](#4-ball-tracking-balltracker)
  - [5. Fallback and Recovery](#5-fallback-and-recovery)
  - [6. Tracker Service](#6-trackerservice-trackerservice)
  - [7. Vision Utilities](#7-vision-utilities-vision_utils)
  - [8. Troubleshooting and Tuning](#8-troubleshooting-and-tuning)
- [A* Pathfinding](#a-pathfinding)
  - [1. A* Algorithm with Repulsion Field](#1-a-algorithm-with-repulsion-field)
  - [2. Downscaled Pathfinding](#2-downscaled-pathfinding)
  - [3. Binary Mask Preprocessing](#3-binary-mask-preprocessing)
  - [4. Waypoint Sampling](#4-waypoint-sampling)
  - [5. Path Drawing](#5-path-drawing)
  - [6. Path Memory Caching](#6-path-memory-caching)
  - [7. Nearest Walkable Point](#7-nearest-walkable-point)
  - [8. A* Troubleshooting and Tuning](#8-troubleshooting-and-tuning-1)
- [Troubleshooting](#troubleshooting)
- [License / Authors / Acknowledgements](#license--authors--acknowledgements)
- [Password](#password)

## Introduction

This project integrates multiple subsystems including camera-based ball tracking, a PID controller, MQTT communication, pathfinding via A*, and offline reinforcement learning potential. It runs on Jetson (for vision and control) and communicates with a Raspberry Pi interface for external monitoring and feedback.

---

## System Overview

- **Jetson**: Main control unit; runs YOLO + HSV ball tracking and PID or RL controller.
- **Raspberry Pi**: Displays HMI; sends goals and receives status via MQTT.
- **Arduino**: Drives the actuators to tilt the maze.
- **ZED Camera**: Captures real-time video of the maze and ball for tracking.
- **Software Components**:
  - Ball detection using YOLOv8 + HSV
  - A* path planning using binary mask maps
  - MQTT communication between Jetson and Pi
  - PID-based and RL-based position control
  - Visualization + troubleshooting tools

---

## Setup & Configuration

### 3.1 Dependencies

Install via `requirements.txt`:

`pip install -r requirements.txt`

## Run Controller

This part of the system is responsible for coordinating ball tracking, control execution, and path-following logic. It serves as the core runtime loop of the Jetson system and interfaces directly with the tracker, controller, image pipeline, and the HMI via MQTT.

---

### 1. System Components

- **HMIController** (`fsm2.py`): A central FSM (finite state machine) that reacts to commands from the Pi (via MQTT) and manages system transitions (e.g., from navigation to control).
- **Controller** (`pos2.py`): Implements PID-based position control and axis-level motor commands, driven by camera tracking and waypoint reference signals.
- **Main Loop** (`run_controller_main.py`): Continuously reads the ball position, smooths it, and invokes control logic to reach the next target in the path.
- **ImageController**: Updates HMI visuals with the latest camera frame and path overlay.
- **MQTTClientJetson**: Handles all MQTT communication and command/event publishing.
- **utility_threads**: Provides supporting threads for blinking LEDs, elevator escape, and pathfinding.

---

### 2. Control Execution Flow

1. On boot, the system enters the `BOOTING` state.
2. The Pi sends a `"booted"` MQTT message to unlock the main screen.
3. User sends `"Locate"` → triggers `start_tracker()` and begins ball tracking.
4. After the ball is found, the system can either:
   - Auto-calculate a path (`"AutoPath"`)
   - Wait for a custom goal from the HMI
5. When `"Start"` is received, control enters the `CONTROLLING` state:
   - The PID controller is run in a 60Hz loop
   - The controller calculates tilt angles and sends velocity commands to the Arduino
   - Camera feedback updates ball position

---

### 3. Controller Configuration

All controller gains, tolerances, and hardware offsets are defined in `config.yaml` under the `controller:` section.

Example:

```yaml
controller:
  stuck_wiggle_amplitude: 0.8
  stuck_wiggle_frequency: 10
  angular_controller:
    kp: 6500
    max_angle: 1.8
    command_delay: 0.0001
  arduino:
    x_offset: 0.01
    y_offset: 0.0
    minimum_velocity: 22
    maximum_velocity: 100
    minimum_velocity_difference: 10
```

The controller supports both standard and lookahead PID gains (configurable via the `lookahead` toggle).

---

### 4. Path Execution

- The `PathFollower` classes take a list of waypoints and automatically handle:
  - Velocity estimation
  - Waypoint switching
  - Stuck detection
  - Looping and dwell times
- The control loop (`run_controller_main.py`) executes path-following in real time, keeping to a target frame rate defined by `TARGET_HZ` (60Hz by default).

---

### 5. Horizontal Calibration

Before control begins, `controller.horizontal()` is called to zero the maze and make sure that tilt angles are neutral, for consistent starting conditions.

```python
controller.horizontal()  # waits until within tolerance or timeout
```

Calibration parameters (time limit, tolerance, gain) are also set in `config.yaml`.

---

### 6. MQTT Command Integration

Commands from the Pi HMI are parsed inside the `HMIController.on_command()` method. Available commands include:

- `Locate`, `AutoPath`, `CustomPath`, `Start`, `Back`, `Restart`, `Exit`
- `Elevator` to retrieve the ball
- `Horizontal` for flat calibration
- `LoopOn` / `LoopOff` to toggle waypoint looping
- `Goal_set:x,y` and `CalculatePath` for custom pathfinding

All command triggers are mapped to state transitions in a finite-state machine.

---

### 7. Troubleshooting

| Issue | Resolution |
|-------|------------|
| Ball not found | Can happen along the edge opposite of where the camera is mounted. See if the camera is properly angled and lighting is sufficient. Check HSV range. |
| Motors not responding | Check Arduino connection and serial port. Try rebooting the Arduino. |
| Control loop unresponsive | Check if the tracker is initialized and goal path is valid. |
| Path not followed | Confirm waypoints are outside elevator zone and control loop is active. |
| MQTT command ignored | Verify Pi is sending valid command strings to expected topics. |

---

### 8. Tuning Tips

- Increase `stuck_wiggle_amplitude` or `stuck_wiggle_frequency` to help escape dead zones.
- Adjust `position_tolerance` and `velocity_tolerance` for tighter or looser control.
- Use `controller.lookahead = True` for faster, more aggressive control.
- Enable or disable looping behavior with the `"LoopOn"` / `"LoopOff"` MQTT commands.
- Tune `feedforward_t` values for more/less anticipatory movement based on distance to next waypoint.

---


## Ball Detection

This part provides an explanation of the ball tracking system which combines object detection via YOLOv8 with real-time color-based tracking using the HSV color space. It is designed for fast visual feedback, and is implemented using modular Python components with threaded architecture for performance. This pipeline is modular and highly tunable, meaning if there are any issues then variables can easily be changed via the `config.yaml` file.

---

### 1. System Architecture

The pipeline consists of these components:

- `CameraManager`: Interfaces with the ZED stereo camera to acquire RGB/BGR frames and retrieve orientation data.
- `YOLOModel`: Loads and runs YOLOv8 for object detection.
- `BallTracker`: The core of the tracking system, responsible for combining YOLO and HSV tracking.
- `vision_utils`: Contains utility functions for color tracking using OpenCV.
- `TrackerService`: A unified interface for the rest of the application to interact with tracking functionality.

All components are developed to maintain high responsiveness and frame stability using a producer-consumer threading pattern.

---

### 2. Camera Acquisition (`CameraManager`)

The ZED camera is initialized with fixed parameters for resolution and frame rate, which can be adjusted in `config.yaml` under the `camera:` section:

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

### 3. Object Detection (`YOLOModel`)

The YOLOv8 nano model is loaded and initialized at runtime using the Ultralytics API. It was trained for 40 epochs using 400+ images with manually annotated labels of the class `ball`. The model path is defined in `config.yaml` under `tracking.model_path`.

After loading, the `.fuse()` method is called to combine convolution and batch normalization layers, optimizing the model for faster inference during deployment. This step improves runtime efficiency without affecting accuracy. The model is then set to evaluation mode using `.eval()`, which disables training-specific behaviors such as dropout and batch norm updates in order to give deterministic and consistent outputs.


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

### 4. Ball Tracking (`BallTracker`)

The core tracking class runs two threads:
- `producer_loop`: Continuously updates the latest RGB/BGR frames from the camera.
- `consumer_loop`: Processes those frames to track the ball.

#### Initialization Phase

When the system starts, the ball must be detected by YOLO within a predefined region defined in `config.yaml` as `tracking.init_ball_region`:

```python
if self.INIT_BALL_REGION[0][0] <= cx <= self.INIT_BALL_REGION[1][0] and ...
```

This makes sure that the ball is only initialized if it appears in a valid location, avoiding false starts caused by noise outside of the maze.

#### HSV Tracking Phase

Once the ball is initialized, frame-to-frame tracking switches to HSV. A region around the last known position is used, and the HSV color range and window size are defined in `config.yaml` under `tracking.hsv_range` and `tracking.hsv_window_size`:

```python
def hsv_tracking(frame, prev_pos, hsv_lower, hsv_upper, window_size=80):
    roi = frame[y_min:y_max, x_min:x_max]
    mask = cv2.inRange(cv2.cvtColor(roi, cv2.COLOR_BGR2HSV), hsv_lower, hsv_upper)
    return get_center_of_mass(mask)
```

To improve robustness, position smoothing is applied using `tracking.smoothing_alpha`:

```python
alpha = 0.5
x = int(alpha * new_pos[0] + (1 - alpha) * self.ball_position[0])
y = int(alpha * new_pos[1] + (1 - alpha) * self.ball_position[1])
self.ball_position = (x, y)
```

---

### 5. Fallback and Recovery

If HSV fails for several consecutive frames (defined by `self.hsv_fail_threshold` in `config.yaml` under `tracking.hsv_fail_threshold`), the system triggers a fallback process:

1. A global HSV scan (`global_hsv_search`) checks for color presence in the full frame.
2. If a candidate is found, YOLO is re-run to validate the detection.
3. A cooldown (`self.yolo_cooldown`) prevents YOLO from running every frame, controlled by `tracking.yolo_cooldown_period`.

Example fallback logic:

```python
if self.hsv_fail_counter >= self.hsv_fail_threshold and self.yolo_cooldown == 0:
    global_pos = global_hsv_search(bgr, *self.HSV_RANGE)
    if global_pos:
        results = self.model.predict(rgb)
        # updates the ball position if confirmed
```

---

### 6. TrackerService (`tracker_service`)

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

### 7. Vision Utilities (`vision_utils`)

These functions are used internally for HSV-based tracking and position estimation.

#### Center of Mass

```python
def get_center_of_mass(mask):
    contours = cv2.findContours(...)[0]
    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < min_area:
        return None
    return (cx, cy)
```

The minimum contour area used for filtering can be set in `config.yaml` under `tracking.hsv_min_contour_area`.

#### Global Search

```python
def global_hsv_search(frame, hsv_lower, hsv_upper):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
    return get_center_of_mass(mask)
```

#### Local Windowed Search

```python
def hsv_tracking(frame, prev_pos, hsv_lower, hsv_upper, window_size=80):
    roi = extract around prev_pos
    mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
    return get_center_of_mass(mask)
```

---

### 8. Troubleshooting and Tuning

### hsv_fail_threshold

Controls how many consecutive HSV tracking failures must occur before falling back to global search and YOLO detection. This can be modified in `config.yaml`:

```python
self.hsv_fail_threshold = 30  # default as it shows a good balance (0.5s at 60 FPS)
```

Increase if you’re experiencing too frequent fallback, decrease for faster recovery.

#### yolo_cooldown_period

Defines how many frames to wait after a YOLO attempt before allowing another. Configurable in `config.yaml`.

```python
self.yolo_cooldown_period = 15  # ~0.25s cooldown
```

Lowering this increases responsiveness at the cost of more frequent inference.

#### min_area

Used in `get_center_of_mass()` to ignore noise in the HSV mask. Change this via `tracking.hsv_min_contour_area` in `config.yaml`.

```python
min_area = 150  # or higher for higher-resolution cameras
```

Raise this to avoid tracking small irrelevant blobs.

#### smoothing factor (alpha)

Applies exponential smoothing to reduce jumpiness. Set via `tracking.smoothing_alpha`:

```python
alpha = 0.5  # lower = smoother, higher = faster reaction
```

Tune this based on the expected ball motion and latency tolerance.

#### Lighting sensitivity

The HSV thresholds are susceptible to changing light conditions. Tune these empirically via `tracking.hsv_range` in `config.yaml`:

```python
self.HSV_RANGE = (
    np.array([35, 80, 80]),  # HSV lower bound for green
    np.array([85, 255, 255]) # HSV upper bound for green
)
```

These can be changed for different ball colors, but it is important to remember that the YOLOv8 model is mostly trained on green balls, and green is what works the best from all values in the HSV space.

## A* Pathfinding

This part explains the architecture and implementation of a complete A* pathfinding system built for navigating the maze using binary obstacle maps. The system includes repulsion-aware path planning, mask preprocessing, waypoint smoothing, path memory caching, and several geometric enhancements. It is modular and can generalize for any type of maze with darker walls and a light background. Configuration for key parameters can be adjusted via the `config.yaml` file.

---

### 1. A* Algorithm with Repulsion Field

The core of the system is a modified A* algorithm (`astar.py`) that adds a **repulsion term** to avoid planning paths too close to obstacles. The algorithm uses **Manhattan distance** as its heuristic and introduces a **cost penalty** that increases near obstacles using a distance transform. The repulsion weight and minimum safe distance are configurable in `config.yaml` under `path_finding.repulsion_weight` and `path_finding.min_safe_distance`.

#### Heuristic

The Manhattan distance works well for grid-based maps:

```python
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])
```

#### Repulsion Map

Repulsion cost is calculated by a distance transform on the binary obstacle map:

```python
def compute_repulsion_cost(array, min_safe_dist=14):
    dist_transform = cv2.distanceTransform((array * 255).astype(np.uint8), cv2.DIST_L2, 3)
    repulsion = (1.0 - (dist_transform / max_dist)) ** 3.0
    return repulsion, mask_safe
```

This produces a smooth cost field where grid points near obstacles are heavily penalized. The `min_safe_dist` value is set through `config.yaml`.

#### Path Search

The `astar()` function integrates the repulsion field into the node cost:

```python
cost = 1.0 + repulsion_weight * repulsion_map[neighbor[0], neighbor[1]]
tentative_g = gscore[current] + cost
```

This guides the A* search away from narrow gaps and obstacle boundaries.

---

### 2. Downscaled Pathfinding

The `astar_downscaled()` function provides a mechanism to run A* on a lower-resolution version of the map for performance or smoothing purposes. The scale factor can be configured through `config.yaml` under `path_finding.astar_downscale`.

```python
small_array = cv2.resize(array, (0, 0), fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
path_small = astar(small_array, start_small, goal_small, repulsion_weight)
```

Once a path is found, it is scaled back up to the original resolution.

---

### 3. Binary Mask Preprocessing

Binary masks are generated from RGB camera input and refined through a series of transformations in `mask_utils.py`. The goal is to extract a reliable navigable area that excludes noise, green-field artifacts, and subtle edges. CLAHE, dynamic thresholding, and edge suppression are used. Preprocessing parameters such as kernel sizes and clip limits can be adjusted in `config.yaml` under `path_finding`.

#### CLAHE and Thresholding

The grayscale image is equalized with CLAHE and then binarized using Otsu’s threshold:

```python
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
enhanced = apply_clahe(gray)
_, base_mask = cv2.threshold(enhanced, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
```

#### Edge Removal and Morphology

Edge noise is removed and the mask is cleaned:

```python
edges = cv2.Canny(enhanced, 100, 200)
edges_inv = cv2.bitwise_not(cv2.dilate(edges, kernel))
final_mask = cv2.bitwise_and(cleaned, edges_inv)
```

#### Green Area Suppression

Green pixels from the ball are removed. HSV threshold values used for this suppression are tuned to avoid corrupting the mask with ball-colored areas:

```python
hsv = cv2.cvtColor(gray_to_bgr, cv2.COLOR_BGR2HSV)
green_mask = cv2.inRange(hsv, lower_green, upper_green)
final_mask = cv2.bitwise_or(final_mask, green_mask)
```

---

### 4. Waypoint Sampling

The raw A* path is often jagged or too dense for real-time movement. The `waypoint_sampling.py` module extracts smooth and valid waypoints with spacing and angular constraints. These values are configured in `config.yaml` under `path_finding.waypoint_spacing` and `path_finding.angle_threshold`.

#### Spacing and Angles

Waypoints are selected based on distance and turning angles:

```python
angle = angle_between(path[last_wp_idx], path[i], path[i + 1])
if angle < angle_threshold and accumulated >= spacing / 2:
    if is_clear_path(mask, path[last_wp_idx], path[i]):
        waypoints.append(path[i])
```

#### Visibility Checks

Every candidate waypoint is validated against a morphological corridor in the mask:

```python
def is_clear_path(mask, p1, p2, kernel_size=6):
    cv2.line(line_img, p1, p2, 1, 1)
    corridor = cv2.dilate(line_img, kernel)
    return np.all(mask[corridor.astype(bool)] > 0)
```

The visibility kernel size can be configured via `path_finding.clear_path_kernel_size`.

---

### 5. Path Drawing

The `draw_path.py` module overlays paths on images for debugging or visualization:

```python
def draw_path(image, waypoints, start, goal):
    for x, y in waypoints:
        cv2.circle(out, (x, y), 8, (0, 0, 255), -1)
```

The output supports grayscale or color images, and annotates start/goal locations.

---

### 6. Path Memory Caching

To avoid recomputation, `path_memory.py` stores previously computed paths using a fuzzy match based on Euclidean distance. Caching parameters like size and tolerance are defined in `config.yaml` under `path_finding.path_cache_size` and `path_finding.path_cache_tolerance`.

```python
def get_cached_path(self, start, goal):
    if _within_tolerance(start, cached_start) and _within_tolerance(goal, cached_goal):
        return cached_path
```

Paths are serialized to JSON and persisted across runs:

```python
with open(self.cache_file, "w") as f:
    json.dump(self.paths, f)
```

---

### 7. Nearest Walkable Point

The function `find_nearest_walkable()` makes sure that the start and goal positions fall inside valid areas:

```python
if mask[y, x] != 0:
    return point
```

If not, it searches in a radius using a distance transform to locate the nearest walkable pixel.

---

### 8. Troubleshooting and Tuning

#### repulsion_weight

Controls how strongly the path avoids obstacles. Set in `config.yaml` under `path_finding.repulsion_weight`.

```python
astar(..., repulsion_weight=5.0)
```

Try values between 2.0 and 6.0 for balance.

#### min_safe_dist in repulsion

Defines how far from an obstacle a point must be to be considered "safe." Set in `config.yaml` as `path_finding.min_safe_distance`.

```python
compute_repulsion_cost(..., min_safe_dist=14)
```

#### scale in `astar_downscaled()`

The downscale factor can be tuned in `config.yaml` using `path_finding.astar_downscale`. The default value of 1.0 means no scaling, but this can be changed to 0.5-0.8 for faster processing. Be careful as downscaling can result in `A* failed in downscaled space` error as downscaling the pixels can interfere with the threshold masking.

```python
astar_downscaled(..., scale=1.0)
```

Basically, lower values improve speed but reduce precision.

#### waypoint_spacing and angle_threshold

Configured in `config.yaml` as `path_finding.waypoint_spacing` and `path_finding.angle_threshold`.

```python
waypoint_spacing=160, angle_threshold=135
```

Higher spacing reduces path density; lower angle threshold improves flexibility.

#### cache tolerance

The threshold for cache re-use is defined in `path_finding.path_cache_tolerance` in `config.yaml`.

```python
PathMemory(tolerance=10)
```

Increase to enable more cache hits, but at the cost of re-using less precise paths.

## Troubleshooting

## Authors

## Password

Login for raspberrypi is: login: raspberrypi, password: raspberry
Login for Jetson is: login: student, password: student 