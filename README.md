# <a id="autonomous-maze-solver"></a>Autonomous Maze Solver - NOV 2025

This project is an autonomous maze-solving system that uses computer vision and robotics to guide a steel ball through physical mazes. The system tilts a maze board with two motors to navigate the ball from start to goal while avoiding holes and obstacles.

The core architecture consists of three main devices: a Jetson Orin Nano for computer vision and control logic, a Raspberry Pi for the human-machine interface, and an Arduino for motor control. A ZED 2i stereo camera mounted above the maze provides real-time ball tracking using a TensorRT-optimized YOLOv8 model. The system calculates optimal paths with A* pathfinding and executes precise movements through PID control.

Users can choose between autonomous mode where the system solves the maze automatically, manual mode for joystick control with competitive timing, and human vs AI mode for head-to-head competition. A local leaderboard tracks the fastest completion times across different maze configurations.

Most system parameters are configured through `config.yaml`, making it easy to adapt to different maze layouts, adjust control parameters, and modify tracking settings without code changes.

---

## <a id="table-of-contents"></a>Table of Contents

- [System Overview](#system-overview)  
- [Physical Setup](#physical-setup)
- [Setup & Configuration](#setup--configuration)
  - [Login Information](#login--information)
  - [Dependencies](#dependencies)
- [Control System](#control-system)
  - [System Components](#system-components)
  - [Control Execution Flow](#control-execution-flow)
  - [Controller Configuration](#controller-configuration)
  - [Path Execution](#path-execution)
  - [Horizontal Calibration](#horizontal-calibration)
  - [MQTT Command Integration](#mqtt-command-integration)
  - [Troubleshooting](#troubleshooting)
  - [Tuning Tips](#tuning-tips)
  - [Electrical Setup](#electrical-setup)
- [MQTT Communication](#mqtt-communication)
  - [Overview](#overview)
  - [Topic Structure](#topic-structure)
  - [Handshake Protocol](#handshake-protocol)
  - [Jetson MQTT Client](#jetson-mqtt-client)
  - [Pi MQTT Client](#pi-mqtt-client)
  - [MQTT Troubleshooting](#mqtt-troubleshooting)
- [Ball Detection](#ball-detection)
  - [System Architecture](#ball-detection-architecture)
  - [Camera Acquisition](#camera-acquisition)
  - [TensorRT YOLO Inference](#tensorrt-yolo-inference)
  - [Ball Tracking](#ball-tracking)
  - [Tracker Service](#tracker-service)
  - [Detection Troubleshooting](#detection-troubleshooting)
- [A* Pathfinding](#a-pathfinding)
  - [A* Algorithm with Repulsion Field](#a-algorithm-with-repulsion-field)
  - [Binary Mask Preprocessing](#binary-mask-preprocessing)
  - [Waypoint Sampling](#waypoint-sampling)
  - [Path Drawing](#path-drawing)
  - [Path Memory Caching](#path-memory-caching)
  - [Nearest Walkable Point](#nearest-walkable-point)
  - [Pathfinding Troubleshooting](#pathfinding-troubleshooting)
- [Authors](#authors)


---

## <a id="system-overview"></a>System Overview

### Core Architecture
- **Jetson Orin Nano**: Primary control unit running TensorRT-optimized YOLOv8 inference, A* pathfinding, and PID control logic
- **Raspberry Pi**: Touchscreen HMI providing user interaction, mode selection, and real-time system status
- **Arduino Mega**: Motor controller executing precise tilt commands via dual-axis PID control
- **ZED 2i Camera**: Stereo camera providing real-time ball position tracking and maze orientation detection

### Software Components
- **Ball Detection**: TensorRT-accelerated YOLOv8 model with boundary validation and position smoothing
- **Path Planning**: A* algorithm with repulsion fields and optimized waypoint sampling
- **Control System**: PID-based position control with adaptive lookahead and error correction
- **Communication**: MQTT-based messaging between all system components
- **User Interface**: Multi-mode touchscreen interface with competitive leaderboards

### Operating Modes
- **Navigation Mode**: Automatic maze solving with real-time path visualization
- **Practice Mode**: Manual joystick control for practicing with a joystick
- **Play Alone**: Timed single-player challenges with leaderboard integration
- **Human vs Robot**: Competitive mode alternating between human and robot turns
- **Admin Tools**: Calibration, and access to shutdown and other tools

---

## <a id="physical-setup"></a>Physical Setup

### Transport Configuration
Before operation, disconnect the transport rod by unscrewing with a 3mm hex key, then reinstall the screw at the bottom position.

![Transport Rod Up](/pictures/transport_stag_nede.png)

For transportation, move the transport rod to the upper position to secure the platform.

![Transport Rod Down](/pictures/transport_stag_oppe.png)

### Access Panels
Use a 2mm hex key to remove the electronics panel covers for maintenance access.

### Camera Positioning
Mount the ZED 2i camera directly above the maze center with clear line-of-sight to the entire playing surface.

---

## <a id="setup--configuration"></a>Setup & Configuration

### <a id="login--information"></a>Login Information

**Raspberry Pi:**
- Username: raspberrypi
- Password: raspberry

**Jetson:**
- Username: student
- Password: student

### <a id="dependencies"></a>Dependencies

Install all required packages via requirements file:

```bash
pip install -r requirements.txt
```
---

## <a id="Flow of the FSM"></a>Flow of the FSM

The system operates through a well-defined state machine `finite_state_machine.py` with the following states:

### BOOTING
Initial state. Transitions to `MAIN_SCREEN` after the Raspberry Pi and Jetson perform a MQTT handshake.

### MAIN_SCREEN
Functions as a home-screen. May branch into several states.

### ADMIN_TOOLS
Requires a code to enter. From here, the admin can calibrate, clear leaderboards and path cache, restart, reboot or shutdown the robot.

### INFO_SCREEN
Displays information about the project.

### LOCATING
Transitioned into from `MAIN_SCREEN` when autonomous solving is chosen. Starts tracker and other threads required to locate the ball. When the ball is found, the state transitions to `CUSTOM_PATH`. 

### CUSTOM_PATH
In this state/screen, the user can choose a path goal by touching an image of the maze. By pressing 'Calculate path', finding a path will be attempted. If a path is found, start-buttons are enabled. The user can choose 'safe-control' or 'fast-control', which are two modes of the controller. When one of the buttons is pressed, the state transitions into `CONTROLLING`

### CONTROLLING
In this state, the control loop is running, letting the robot autonomously balance the ball through the maze. On the screen, a large live camera feed of the maze is displayed for the user to examin while executing. 

### HUMAN_CONTROLLER
Transitioned into from `MAIN_SCREEN` if the user chooses 'Game modes'. The main property of this state and states branching from it, is that the user may control the maze using and Xbox controller.

### PRACTICE
Playground for the user to attempt the maze without being able to loose and without being timed by a stop-watch.

### PLAYALONE
Start of the sequence of states that let the player attempt the maze while having their time recorded and, if succeeding, added to the leaderboard. In this state, the user may enter their name on the screen. Transitions into `PLAYALONE_START` when the user confirms their name

### PLAYALONE_START
In this state, the ball is continuously tracked. If the ball is located withing the elevator, the user may start their attempt. After pressing start, the user attempt is began, letting the user control the ball using the Xbox controller. If the ball falls through a hole, the state transitions into `PLAYALONE_FAILED`. If the ball reaches the goal, the state transitions into `PLAYALONE VICTORY`.

### PLAYALONE_FAILED
State where the user may either exit from the game mode, or choose to try again, returning to `PLAYALONE_START`

### PLAYALONE_VICTORY
The users maze solving time is saved. Their rank is computed by comparing the time to the leaderboard of the corresponding maze. The maze type (easy or hard) is automatically determined by the camera. The users time is added to the leaderboard.

### LEADERBOARD
Transitioned into using the corresponding button in either `HUMAN_CONTROLLER` or `PLAYALONE_VICTORY`. Displays the leaderboards of the easy and hard maze. The user may use a button to swap between the two displays.

### PLAYVSAI
Start of the sequence for the game mode 'Human vs Robot'. In this state, the user may choose a goal by touching an image of the maze. Then the user may press 'Start robot'.

### PLAYVSAI_PID
Transitioned into when the user presses 'Start robot' in `PLAYVSAI`. The robot computes a path to the goal, and attempts to balance the ball through it. If success, the time is saved.

### PLAYVSAI_HUMAN
Transitioned into after the robot has either failed or succeeded in `PLAYVSAI_PID`. The user may press 'Start turn' and then control the ball through the maze using the xBox controller. If success, the time is saved. The times of the robot and the user are compared, and the winner is determined. 

### **Sate transition logic**
Each state handles specific commands and maintains system integrity:
- **Back Commands**: Clean state transitions with resource cleanup
- **Error Handling**: Automatic fallback to safe states on failures
- **Resource Management**: Thread lifecycle and hardware control coordination

### **MQTT Command integration**

The control system integrates tightly with MQTT communication for coordinated operation:

**Command Processing**: The finite state machine subscribes to `jetson/command` topic and processes:
- Mode transitions (Practice, PlayAlone, Navigate, etc.)
- Game controls (StartGame, Retry, Back)
- System commands (Locate, AdminTools, Disco)

**Status Broadcasting**: Control states are published to relevant topics:
- `pi/command`: UI state updates and game results
- `pi/tracking_status`: Ball detection status and game events
- `arduino/speed`: Real-time motor control commands

## <a id="control-system"></a>Control System

### <a id="system-components"></a>System Components

The control system integrates multiple subsystems through a centralized finite state machine:

**Position Controller** (`position_controller.py`): 
- `posControl`: PID-based control system computing tilt angles in both X and Y axis required for the ball to reach the target waypoint.
- `axisControl`: P-controller that takes in reference tilt angles from the position controller, computes motor velocities, and sends the velocities to the Arduino

**Arduino Connection** (`arduino_connection.py`): Serial communication interface handling motor commands, elevator control, and system status feedback.

**Joystick Controller**: Manual control interface for practice mode and human vs AI gameplay.

### <a id="controller-configuration"></a>Controller Configuration

The PID controller accepts configuration through `config.yaml`:

```yaml
position_controller_normal:
    feedforward_t: 7.8
    kd_x: 0.000085
    kd_y: 0.000085
    ki_x: 0.0004
    ki_y: 0.0004
    kp_x: 0.00004
    kp_y: 0.00004
    position_tolerance: 25
    velocity_tolerance: 20
  position_smoothing_alpha: 0.1
  stuck_time_threshold: 0.5
  stuck_unstuck_hold_time: 0.2
  stuck_upper_position_threshold: 70
  stuck_velocity_threshold: 30
  stuck_wiggle_amplitude: 0.4
  stuck_wiggle_frequency: 20
  velocity_smoothing_alpha: 0.99
  wiggle_direction_bias: 0.007
```

**Key Parameters:**
- **feedforward_t**: Tuning parameter for feedforward. Increasing this reduces the responsiveness of the system. Decreasing may lead to fast but unstable responses.
- **kp**: Proportional gain. Increase to punish position error deviations.
- **ki**: Integral gain. Increase to punish position error deviations over time. Prevents steady-state errors.
- **kd**: Derivative gain. Increase for more damping. Increase to punish large velocities and to 'break' when approaching waypoints.
- **position_tolerance**: Acceptance radius for waypoint navigation.

### <a id="path-execution"></a>Path Execution

Path execution occurs in the `run_controller_main.main()` function with the following sequence:

1. **Path Validation**: Verify path waypoints are within maze boundaries
2. **Target Assignment**: Set initial target to first path waypoint
3. **Position Control Loop**:
   - Get current ball position from tracking service
   - Calculate PID control output for X/Y axes
   - Apply velocity limits and send motor commands
   - Check target proximity and advance to next waypoint
4. **Completion Detection**: Monitor goal proximity and handle success/failure states

### <a id="path-following scheme"></a>Path-following scheme
The controller may use one of two path-following schemes. Both schemes have advantages and dissadvantages, and the user may choose what scheme to use when starting the control execution after path-finding.
The member variable `lookahead` in `position_controller.py` can be set to `True` or `False` corresponding to waypoint navigation and pure pursuit respectively. This should be set before starting `run_controller_main.py`.

**Waypoint Navigation**

*In the HMI, this is referred to as "Safe Control"*
- Setpoint control toward the next waypoint
- Advances waypoint when the ball is within an acceptance radius
- Uses dwell/advance logic at each waypoint. This ensures the ball rests at waypoints and helps prevent prematurely turning around corners.
- Timeout based waypoint reverting. When a certain amount of time passes with no waypoint advancements, the target waypoint is reverted to the previous in the path.
- Pros:
    - Accurately controls to each waypoint
    - Ball velocity stays controllable
- Cons:
    - The scheme is optimal when the path consists of few waypoints. This large reduction with waypoint sampling may lead to important waypoints being removed, resulting in a bad path.
    - Slow. Stops at every waypoint.
    - Low velocities may result in the ball stopping just before a waypoint, then advancing. This leads to cutting of turns. This is attempted prevented using dwell/advance.

**Pure Pursuit**

*In the HMI, this is referred to as "Fast Control"*
- Projects the ball onto the nearest segment between two waypoints. Then walks along the path from the projected point a total distance of `lookahead distance`. The resulting point is chosen as the target reference for position control.
- The projection is continuous, resulting in a smoother path trajectory.
- Pros:
    - Continuous path following
    - Fast control due to not stopping at waypoints
    - Smooth path/turns
    - Natural ball movement
    - Does not rely on optimal waypoint sampling
- Cons:
    - Smaller reduction of waypoints through sampling. This results in a more computaionally expensive control loop.
    - Cuts turns due to the lookahead.
    - Velocity may build up enough to become difficult to stop quickly using damping.

### <a id="horizontal-calibration"></a>Horizontal Calibration

The system includes automatic horizontal calibration to establish a neutral maze position.
Calibration occurs automatically during state transitions and can be manually triggered through the admin interface.

### <a id="tuning-tips"></a>Tuning Tips

- Use current values as a base line
- More responsiveness -> Increase `feedforward_t`
- Smaller max velocities -> Increase `k_d`
- More accurately stopping at waypoints -> Increase `k_d`
- Prevent ball from getting stuck close to waypoints -> Increase `k_i`
- Improve path accuracy (waypoint navigation) -> Decrease `position_tolerance`
    - Consequence: Results in ball stopping more often and struggling to approach waypoint.
- Improve path accuracy (pure pursuit) -> Decrease `lookahead_distance`
    - Consequence: Reacts more slowly to path turns. Less predictive behaviour.

## <a id="electrical-setup"></a>Electrical Setup

The electrical system connects multiple components through a central power distribution and communication hub:

**Power Requirements:**
- Main Power Supply: 24V/5A for motor control system
- Jetson Power: 5V/4A via USB-C or barrel connector
- Raspberry Pi: 5V/3A via USB-C
- Arduino: 5V via USB (powered by Jetson)

**Communication Wiring:**
- Jetson ↔ Arduino: USB serial connection
- Jetson ↔ Pi: Ethernet (MQTT over WiFi backup)
- Arduino ↔ Motors: PWM control signals with current feedback

**Safety Features:**
- Emergency stop circuitry for immediate motor shutdown
- Current limiting on motor drivers to prevent damage
- Voltage monitoring with automatic shutdown on power anomalies

---

## <a id="mqtt-communication"></a>MQTT Communication

### <a id="overview"></a>Overview

The system uses MQTT as the primary communication protocol between the Jetson control unit and Raspberry Pi interface. This lightweight publish-subscribe protocol gives a reliable message delivery with low latency.

Raspberry Pi IP: `192.168.1.2`

Jetson IP: `192.168.1.3`

The broker runs locally on the Jetson device.

### <a id="topic-structure"></a>Topic Structure

The topic hierarchy follows a device-based naming convention for clear message routing:

#### **Jetson Publishes:**
- `handshake/response`: Handshake response to Pi.
- `pi/command`: General commands.
- `pi/camera`: Camera feed.
- `pi/info`: General information flow from Jetson to Pi.
- `pi/tracking_status`: Information about wether the tracker is started and wether or not the ball is found.
- `pi/leaderboard_data`: Sending of leaderboard data. 

#### **Pi publishes:**
- `hadshake/request`: Initiate handshake process.
- `jetson/command`: General commands to transition states in the FSM.
- `jetson/player_name`: Information about player name when using joystick.

### <a id="handshake-protocol"></a>Handshake Protocol

Device initialization follows a structured handshake sequence to ensure reliable connection establishment:

1. **Pi Startup**: Publishes connection request to `jetson/handshake`
2. **Jetson Response**: Acknowledges connection and sends initial state
3. **State Synchronization**: Exchange current system mode and configuration
4. **Heartbeat Establishment**: Begin periodic status updates

### <a id="jetson-mqtt-client"></a>Jetson MQTT Client

The Jetson MQTT client (`MQTTClientJetson`) manages all communication from the control system:

```python
class MQTTClientJetson(threading.Thread):
    def __init__(self, arduino_connection: ArduinoConnection = None, fsm = None, broker_address="192.168.1.3", port=1883):
        super().__init__()
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)  # type: ignore
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_disconnect = self.on_disconnect

    def on_message(self, client, userdata, msg):
        payload = msg.payload.decode()
        topic = msg.topic
        print(f"Message received on topic '{topic}': {payload}")

        if topic == "handshake/request":
            if payload == "pi":
                self.client.publish("handshake/response", "ack", qos=1)
                self.client.publish("pi/command", "booted", qos=1)
                self.handshake_complete = True
```
### <a id="pi-mqtt-client"></a>Pi MQTT Client

The Pi MQTT client handles the touchscreen interface and user interaction:

```python
class MQTTClientPi(threading.Thread):
    def __init__(self, broker_address='192.168.1.3', port=1883):
        super().__init__()
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        
    def on_message(self, client, userdata, msg):
        if msg.topic == "pi/command":
            payload = msg.payload.decode()
            if payload.startswith("playalone_success:"):
                duration = payload.split(":")[1]
                rank = payload.split(":")[2]
                if self.app and hasattr(self.app, 'frames'):
```

### <a id="mqtt-troubleshooting"></a>MQTT Troubleshooting

**Connection Issues:**
- Verify broker is running: `sudo systemctl status mosquitto`
- Check connectivity between devices
- Monitor broker logs: `sudo tail -f /var/log/mosquitto/mosquitto.log`

---

## <a id="ball-detection"></a>Ball Detection

### <a id="ball-detection-architecture"></a>System Architecture

The ball detection system uses a TensorRT-optimized YOLOv8 model for robust real-time ball tracking. The detection pipeline operates at 50 FPS with the following components:
- **Camera Manager**: ZED 2i stereo camera interface with hardware-accelerated frame capture
- **YOLO Model**: TensorRT-optimized object detection with CUDA context management
- **Ball Tracker**: Producer-consumer threading with boundary validation and position smoothing
- **Tracker Service**: High-level coordination and state management

### <a id="camera-acquisition"></a>Camera Acquisition

The `CameraManager` class handles ZED 2i camera initialization and frame acquisition:

```python
CLASS CameraManager:
    INIT():
        Create ZED camera object
        Mark as not initialized

    FUNCTION init_camera():
        IF already initialized:
            RETURN
        Create camera initialization parameters:
            - Resolution: HD720
            - FPS: 60
            - Depth mode: Neural
            - Units: Millimeters
        TRY to open ZED camera with parameters
        IF open fails:
            RAISE error "Camera failed to open"
        Mark as initialized

    FUNCTION grab_frame():
        Create empty image container
        IF ZED grab() succeeds:
            Retrieve LEFT view image into container
            Convert image to BGR format
            Convert BGR to RGB format
            RETURN (rgb_frame, bgr_frame)
        ELSE:
            RETURN (None, None)

    FUNCTION get_orientation():
        Create sensor data container
        IF retrieving sensor data fails:
            RETURN None
        Get IMU data from sensor data
        Create transform container
        Get orientation quaternion from IMU pose
        Round quaternion values to 3 decimal places
        Calculate:
            dir1 = ox + ow
            dir2 = oy - oz
        RETURN orientation vector [-dir2, dir1]

    FUNCTION close():
        IF camera is initialized:
            Close ZED camera
            Mark as not initialized
```

### <a id="tensorrt-yolo-inference"></a>TensorRT YOLO Inference

The `YOLOModel` class implements optimized inference with TensorRT acceleration:

```python
CLASS YOLOModel:
    INIT(model_path, input_shape, conf):
        Store input shape and confidence threshold
        Initialize variables and lock
        TRY:
            Initialize TensorRT engine (preferred)
            Set engine_type = "tensorrt"
        EXCEPT:
            Print warning and fall back to PyTorch
            Set engine_type = "pytorch"

    FUNCTION _init_tensorrt(model_path):
        Initialize CUDA context
        Create TensorRT runtime
        Load engine from file
        Create execution context
        Get input/output bindings and data types
        Allocate host and device memory
        Create CUDA stream

    FUNCTION _init_pytorch_fallback(model_path):
        Convert engine path to .pt file path
        Load YOLO model from PyTorch
        Detect available device (GPU/CPU)
        Set class names from model

    DESTRUCTOR:
        If CUDA context exists, pop it

    FUNCTION get_label(cls_id):
        RETURN class name from ID

    FUNCTION preprocess(image):
        Resize image to model input size
        Normalize pixel values to [0,1]
        Rearrange channels (CHW)
        Convert to float16
        Copy into host input buffer

    FUNCTION predict(image):
        IF engine_type == "tensorrt":
            RETURN _predict_tensorrt(image)
        ELSE:
            RETURN _predict_pytorch(image)

    FUNCTION _predict_tensorrt(image) [ensured CUDA context]:
        Store original image dimensions
        Preprocess image
        Copy input data to GPU
        Execute inference asynchronously
        Copy output data from GPU
        Synchronize stream
        Postprocess and return results

    FUNCTION _predict_pytorch(image):
        Run PyTorch YOLO inference
        Return the first result object

    FUNCTION postprocess(output):
        If output shape is invalid → return empty result
        Filter detections by confidence threshold
        Convert normalized coords to original image coords
        Run Non-Maximum Suppression (NMS)
        Select best detection
        RETURN Result object with Box data

    FUNCTION nms(boxes, scores, iou_threshold):
        Sort boxes by score (descending)
        Iteratively keep highest-scoring boxes that don't overlap too much
        RETURN kept box indices

    FUNCTION _empty_result():
        RETURN object with empty boxes list

    FUNCTION shutdown():
        Acquire lock
        If already shutdown → return
        Mark as shutdown
        Free GPU memory
        Delete TensorRT resources
        Pop CUDA context if still active
        Print shutdown complete

CLASS Box:
    INIT(x1, y1, x2, y2, conf, cls):
        Store coordinates in xyxy format
        Store confidence and class
        Compute xywh format

CLASS Result:
    INIT(boxes):
        Store list of Box objects
```

**Performance Benefits:**
- **Fast**: TensorRT optimization significantly reduces inference time
- **Memory**: Optimized memory allocation and reuse
- **Latency**: Predictable inference times for real-time control
- **Fallback**: Automatic PyTorch fallback if TensorRT unavailable

### <a id="ball-tracking"></a>Ball Tracking

The `BallTracker` class implements a producer-consumer pattern for continuous ball tracking:

```python
CLASS BallTracker:
    INIT(camera, tracking_config, model_path):
        Load YOLO model from model_path
        Store camera reference
        Set configuration parameters (or defaults)
        Initialize runtime variables:
            - running state
            - initialized state
            - frame queue (small size buffer)
            - latest BGR frame
            - current ball position
            - bounding box buffer
            - maze boundaries (top-left, top-right, bottom-left, bottom-right)
    
    FUNCTION is_point_in_maze(x, y):
        RETURN True if point is inside maze bounds
    
    FUNCTION producer_loop():
        WHILE running:
            Record start time
            Capture RGB and BGR frames from camera
            IF frames are valid:
                Add frames to queue
            Sleep to maintain ~60 FPS

    FUNCTION consumer_loop():
        WHILE running:
            IF no frames in queue:
                Short sleep
                CONTINUE
            Remove older frames so only latest remains
            Take most recent RGB and BGR frames
            Store latest BGR for later retrieval

            Run YOLO inference on RGB frame
            Clear current ball position

            Filter predicted boxes for label "ball" (max 1)
            FOR each detected ball:
                Compute center coordinates
                IF ball center is inside maze:
                    Save ball position
                    Normalize coordinates relative to frame size
                    Update bounding box buffer
                    BREAK (only first ball used)

            Sleep to maintain ~50 FPS

    FUNCTION start():
        Set running = True and initialized = True
        Start producer thread (frame grabbing)
        Start consumer thread (detection/tracking)

    FUNCTION stop():
        Set running = False
        Set initialized = False
        IF model has shutdown method:
            Call shutdown

    FUNCTION get_position():
        RETURN current ball position

    FUNCTION get_frame():
        RETURN latest BGR frame (if available)

    FUNCTION retrack():
        Reset ball position
```

**Key Features:**
- **Threading**: Separate detection thread prevents blocking the control loop
- **Boundary Validation**: Rejects detections outside valid maze area
- **Position Smoothing**: Exponential moving average reduces noise
- **Thread Safety**: Proper locking for concurrent position access

### <a id="tracker-service"></a>Tracker Service

The `TrackerService` provides a high-level interface for the tracking system:

```python
CLASS TrackerService:
    INIT(model_path, tracking_config):
        Create CameraManager instance
        Initialize camera
        Set tracker to None
        Mark as not started
        Store model path
        Store tracking config (or default)

    FUNCTION start_tracker():
        IF already started:
            RETURN
        Create BallTracker instance (pass camera, config, model_path)
        Start BallTracker threads (producer & consumer)
        Mark as started

    FUNCTION stop_tracker():
        IF started:
            Stop BallTracker
            Mark as not started

    FUNCTION get_ball_position():
        IF started:
            RETURN ball position from tracker
        ELSE:
            RETURN None

    FUNCTION get_stable_frame():
        IF started:
            RETURN latest BGR frame from tracker
        ELSE:
            RETURN None

    FUNCTION get_orientation():
        RETURN orientation vector from camera IMU

    PROPERTY is_initialized:
        RETURN tracker.initialized if tracker exists, else False

    FUNCTION retrack():
        IF started AND tracker exists:
            Call retrack() on BallTracker
```

### <a id="detection-troubleshooting"></a>Detection Troubleshooting

**Common Issues and Solutions:**

**Ball Detection Failures:**
- Check lighting conditions, even illumination without shadows is ideal
- Verify camera focus and positioning for clear ball visibility
- Check GPU memory usage as insufficient VRAM can cause inference failures

**Position Accuracy Problems:**
- Calibrate the maze
- Check camera mounting stability

---

## <a id="a-pathfinding"></a>A* Pathfinding

### <a id="a-algorithm-with-repulsion-field"></a>A* Algorithm with Repulsion Field

The pathfinding system uses an enhanced A* algorithm with repulsion fields to generate safe, efficient paths through the maze. Unlike standard A*, this implementation adds artificial potential fields around obstacles to create paths that maintain safe distances from walls and holes.

#### **Heuristic Function**
The heuristic uses Manhattan distance:

```python
FUNCTION heuristic(a, b):
    RETURN Manhattan distance between a and b
    # (Alternatively: Euclidean distance for fewer node expansions)
```

#### **Repulsion Map Generation**
Repulsion fields are precomputed using distance transforms, in order to not make the path hug the walls:

```python
FUNCTION compute_repulsion_cost(array, min_safe_dist):
    Compute distance transform of obstacle map
    Create mask marking cells >= min_safe_dist as walkable
    Find maximum distance value
    IF max distance is zero:
        RETURN ones array (no repulsion), mask
    ELSE:
        Compute repulsion cost as (1 - (dist / max_dist))^3
        RETURN repulsion cost map, mask
```

#### **Path Search Implementation**
The core A* search integrates repulsion costs:

```python
FUNCTION astar(array, start, goal, repulsion_weight):
    rows, cols = dimensions of array
    repulsion_map, walkable_mask = compute_repulsion_cost(array)

    Define neighbor offsets (up, down, left, right)

    Create min-heap for open set
    Push start node with f = g + h

    Initialize:
        came_from = empty dict
        g_score = array filled with infinity
        g_score[start] = 0
        in_open = set containing start
        closed = boolean array (all False)

    WHILE open_heap not empty:
        Pop node with lowest fscore
        Remove from in_open

        IF current node == goal:
            Reconstruct path by backtracking via came_from
            RETURN path (start → goal)

        Mark current as closed

        FOR each neighbor offset (dx, dy):
            nx, ny = current.x + dx, current.y + dy

            IF neighbor out of bounds → skip
            IF not walkable or already closed → skip

            move_cost = 1 + repulsion_weight * repulsion_map[nx, ny]
            tentative_g = cost_so_far + move_cost

            IF tentative_g < g_score[neighbor]:
                Update came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                fscore = tentative_g + heuristic(neighbor, goal)
                IF neighbor not in in_open:
                    Push (fscore, tentative_g, neighbor) to heap
                    Add neighbor to in_open

    RETURN empty list (no path found)
```

### <a id="binary-mask-preprocessing"></a>Binary Mask Preprocessing

High-quality binary masks separating clear areas for the ball and the walls/holes are important for reliable pathfinding:

#### **Dilation and CLAHE**
```python
FUNCTION dilate_mask(mask, kernel_size, iterations):
    Create kernel of ones (uint8) with given size
    RETURN result of morphological dilation on mask using kernel for given iterations

FUNCTION get_dynamic_threshold(image, target_brightness):
    Convert image to grayscale
    Compute current average brightness
    Calculate adjustment = target_brightness - current_brightness
    Adjust grayscale pixel values by this amount (clip to range 0–255)
    RETURN adjusted grayscale image

FUNCTION apply_clahe(gray):
    Create CLAHE object (contrast-limited adaptive histogram equalization)
    Apply CLAHE to grayscale image
    RETURN enhanced grayscale image
```

#### **Edge Removal and Morphology**
```python
FUNCTION create_binary_mask(gray):
    enhanced = apply_clahe(gray)
    base_mask = Otsu binary threshold on enhanced image
    cleaned = Morphological closing (kernel 5x5) on base_mask, 2 iterations

    edges = Canny edge detection on enhanced image (thresholds 100, 200)
    edges_dilated = Dilate edges with 3x3 kernel, 1 iteration
    edges_inv = Invert edges_dilated

    final_mask = Logical AND between cleaned mask and edges_inv
    final_mask = Morphological closing (kernel 7x7) on final_mask, 1 iteration

    RETURN final_mask
```

### <a id="waypoint-sampling"></a>Waypoint Sampling

The waypoint sampling system transforms raw A* paths into optimized trajectories suitable for physical ball control. The process combines multiple geometric algorithms to balance path smoothness, control accuracy, and computational efficiency.

#### **Theoretical Foundation**

The system uses a multi-stage optimization pipeline:

1. **Raw path densification**: Continuous coverage along path segments by adding intermediate points at regular intervals
2. **Direction change**: Identifies course changes requiring explicit waypoints for accurate navigation
3. **L-Pivot smoothing**: Inserts intermediate waypoints at corners to for 90 degrees turns
4. **Douglas-Peucker**: Reduces redundant waypoints while preserving path geometry within specified tolerance
5. **Statistical refinement**: Segment length analysis to maintain consistent waypoint spacing

#### **Implementation**

The sampling algorithm processes raw A* output through several transformation stages:

```python
FUNCTION sample_waypoints(path, mask):
    raw = [first point in path]
    y, x = first point
    FOR each (ty, tx) in remaining path points:
        dx = tx - x
        sx = 1 if tx > x else -1
        FOR xx from x+sx to tx (inclusive) step sx:
            IF mask[y][xx] is walkable:
                append (y, xx) to raw
        x = tx

        dy = ty - y
        sy = 1 if ty > y else -1
        FOR yy from y+sy to ty (inclusive) step sy:
            IF mask[yy][x] is walkable:
                append (yy, x) to raw
        y = ty

    pts = [raw[0]]
    FOR (prev, curr, nxt) sliding through raw:
        dir1 = vector from prev → curr
        dir2 = vector from curr → nxt
        IF direction changes:
            append curr to pts
    append last point of raw to pts

    full = [pts[0]]
    FOR (a, b, c) sliding through pts:
        IF straight in one axis but corner in next:
            Generate intermediate corner points:
                (a.y, c.x), (c.y, c.x)
            IF not duplicate AND clear_path from last full point to new point:
                append new point to full
        IF b not already last in full:
            append b
    Append last point of pts if missing

    seg_dists = distances between consecutive points in full
    mean_seg = average segment length
    eps = mean_seg * log(raw_length + 1)
    waypoints = Douglas-Peucker simplify(full, eps)

    IF at least two waypoints:
        Compute average and std deviation of segment lengths
        threshold = avg + std
        new_wps = [first waypoint]
        FOR each segment (a, b):
            IF length > threshold:
                mid = midpoint between a and b
                append mid
            append b
        waypoints = new_wps

    RETURN waypoints

```

#### **Douglas-Peucker Algorithm**

The system uses the Douglas-Peucker algorithm for path simplification:

```python
FUNCTION _douglas_peucker(pts, eps):
    IF fewer than 3 points:
        RETURN pts

    a = first point in pts
    b = last point in pts
    idx = 0
    dmax = 0.0

    FOR i from 1 to len(pts)-2:
        d = _perp_dist(pts[i], a, b)
        IF d > dmax:
            idx = i
            dmax = d

    IF dmax <= eps:
        RETURN [a, b]  # all points lie close to straight line

    left = _douglas_peucker(points from a to pts[idx], eps)
    right = _douglas_peucker(points from pts[idx] to b, eps)

    RETURN left[:-1] + right


FUNCTION _perp_dist(pt, a, b):
    y0, x0 = pt
    y1, x1 = a
    y2, x2 = b

    num = ABS((x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0))
    den = Euclidean distance between a and b (avoid division by zero)
    RETURN num / den

```

### <a id="path-drawing"></a>Path Drawing

Visual path representation for debugging and user feedback:

```python
FUNCTION draw_path(image, waypoints, start, goal):
    out = copy of image

    IF image is grayscale (2D) or has 1 channel:
        Convert to BGR
    ELSE IF image has 4 channels (BGRA):
        Convert to BGR

    h, w = image height and width

    FOR each (x, y) in waypoints (if any):
        IF y within [0, h) AND x within [0, w):
            Draw filled circle at (x, y) with radius 4 and color (128, 0, 0)  # dark red

    IF start exists:
        Draw filled green circle (0, 255, 0) at (start col, start row), radius 5
    IF goal exists:
        Draw filled blue circle (255, 0, 0) at (goal col, goal row), radius 5

    RETURN out

```

### <a id="path-memory-caching"></a>Path Memory Caching

Caching reduces computation for repeated queries, skipping the A* loading time:

```python
FUNCTION get_cached_path(start, goal):
    FOR each entry in self.paths:
        s = entry["start"]
        g = entry["goal"]
        path = entry["path"]
        IF start is within tolerance of s AND goal is within tolerance of g:
            RETURN path
    RETURN None


FUNCTION cache_path(start, goal, path):
    IF number of cached paths >= max_paths:
        Remove the oldest entry from self.paths

    Append to self.paths:
        {
            "start": start as list,
            "goal": goal as list,
            "path": each point in path as list
        }

    Call self.save_cache()
```

### <a id="nearest-walkable-point"></a>Nearest Walkable Point

Handles cases where start/goal points are in invalid areas:

```python
FUNCTION find_nearest_walkable(mask, point, max_radius):
    y, x = point

    IF mask[y, x] != 0:
        RETURN point

    dist = DistanceTransform of mask (L2 metric, kernel size 5)

    min_val = infinity
    nearest = point
    h, w = mask dimensions

    FOR dy from -max_radius to max_radius:
        FOR dx from -max_radius to max_radius:
            ny = y + dy
            nx = x + dx

            IF ny, nx inside mask bounds AND mask[ny, nx] > 0:
                IF dist[ny, nx] < min_val:
                    min_val = dist[ny, nx]
                    nearest = (ny, nx)

    RETURN nearest
```

### <a id="pathfinding-troubleshooting"></a>Pathfinding Troubleshooting

#### **repulsion_weight**
Controls how strongly paths avoid obstacles:
- **Too Low (<5)**: Paths may pass too close to walls, risking collisions
- **Too High (>20)**: Paths become overly conservative and inefficient
- **Optimal Range**: 8-15 for most maze configurations

#### **min_safe_dist in repulsion**  
Minimum distance maintained from obstacles:
- **Too Small (<10 pixels)**: Insufficient safety margin for control errors
- **Too Large (>30 pixels)**: May make narrow passages impassable
- **Recommended**: 15-25 pixels based on ball size and control precision

---

## <a id="authors"></a>Authors

