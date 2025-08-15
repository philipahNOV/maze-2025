# <a id="autonomous-maze-solver"></a>Autonomous Maze Solver - NOV 2025

This project is an autonomous maze-solving system that uses computer vision and robotics to guide a steel ball through physical mazes. The system tilts a maze board with two motors to navigate the ball from start to goal while avoiding holes and obstacles.

The core architecture consists of three main devices: a Jetson Orin Nano for computer vision and control logic, a Raspberry Pi for the human-machine interface, and an Arduino for motor control. A ZED 2i stereo camera mounted above the maze provides real-time ball tracking using a TensorRT-optimized YOLOv8 model. The system calculates optimal paths with A* pathfinding and executes precise movements through PID control.

Users can choose between autonomous mode where the system solves the maze automatically, manual mode for joystick control with competitive timing, and human vs AI mode for head-to-head competition. A local leaderboard tracks the fastest completion times across different maze configurations.

Most system parameters are configured through `config.yaml`, making it easy to adapt to different maze layouts, adjust control parameters, and modify tracking settings without code changes.

---

## <a id="table-of-contents"></a>Table of Contents

- [Introduction](#introduction)
- [System Overview](#system-overview)  
- [Physical Setup](#physical-setup)
- [Setup & Configuration](#setup--configuration)
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
  - [Message Formats](#message-formats)
  - [Command Flow](#command-flow)
  - [MQTT Troubleshooting](#mqtt-troubleshooting)
- [Ball Detection](#ball-detection)
  - [System Architecture](#ball-detection-architecture)
  - [Camera Acquisition](#camera-acquisition)
  - [TensorRT YOLO Inference](#tensorrt-yolo-inference)
  - [Ball Tracking](#ball-tracking)
  - [Tracker Service](#tracker-service)
  - [Vision Utilities](#vision-utilities)
  - [Detection Troubleshooting](#detection-troubleshooting)
- [A* Pathfinding](#a-pathfinding)
  - [A* Algorithm with Repulsion Field](#a-algorithm-with-repulsion-field)
  - [Downscaled Pathfinding](#downscaled-pathfinding)
  - [Binary Mask Preprocessing](#binary-mask-preprocessing)
  - [Waypoint Sampling](#waypoint-sampling)
  - [Path Drawing](#path-drawing)
  - [Path Memory Caching](#path-memory-caching)
  - [Nearest Walkable Point](#nearest-walkable-point)
  - [Pathfinding Troubleshooting](#pathfinding-troubleshooting)
- [Authors](#authors)

---

## <a id="introduction"></a>Introduction

This autonomous maze solver represents NOV's 2025 robotics project, integrating computer vision, pathfinding algorithms, and precision motor control into a complete system. The project demonstrates practical applications of machine learning inference, real-time control systems, and distributed computing across multiple embedded devices. The system processes live camera feeds at high framerates using GPU-accelerated deep learning models, computes optimal navigation paths in real-time, and executes precise mechanical movements to achieve sub-second maze completion times. The modular architecture allows for easy adaptation to different maze configurations and provides both autonomous operation and interactive gameplay modes.

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
- **Practice Mode**: Manual joystick control for user skill development  
- **Play Alone**: Timed single-player challenges with leaderboard integration
- **Human vs AI**: Competitive mode alternating between human and AI turns
- **Admin Tools**: System diagnostics, calibration, and configuration access

---

## <a id="physical-setup"></a>Physical Setup

### Transport Configuration
Before operation, disconnect the transport rod by unscrewing with a 3mm hex key, then reinstall the screw at the bottom position.

![Transport Rod Up](/pictures/transport_rod_up.png)

For transportation, move the transport rod to the upper position to secure the platform.

![Transport Rod Down](/pictures/transport_rod_down.png)

### Access Panels
Use a 2mm hex key to remove the electronics panel covers for maintenance access.

### Camera Positioning
Mount the ZED 2i camera directly above the maze center with clear line-of-sight to the entire playing surface. Ensure stable mounting to prevent vibration-induced tracking errors.

---

## <a id="setup--configuration"></a>Setup & Configuration

### <a id="dependencies"></a>Dependencies

Install all required packages via requirements file:

```bash
pip install -r requirements.txt
```

Core dependencies include:
- **PyTorch**: Deep learning framework for YOLO model inference
- **TensorRT**: GPU acceleration for optimized model execution  
- **OpenCV**: Computer vision operations and camera interface
- **NumPy**: Numerical computations for pathfinding and control
- **Paho-MQTT**: Inter-device communication protocol
- **PyCUDA**: GPU memory management and CUDA context handling

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

**Pure Pursuit**

*In the HMI, this is referred to as "Fast Control"*
- Projects the ball onto the nearest segment between two waypoints. Then walks along the path from the projected point a total distance of `lookahead distance`.
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

The system uses MQTT (Message Queuing Telemetry Transport) as the primary communication protocol between the Jetson control unit and Raspberry Pi interface. This lightweight publish-subscribe protocol ensures reliable message delivery with minimal latency for real-time control applications.

MQTT provides several advantages for this distributed robotics system:
- **Low Latency**: Sub-millisecond message delivery for time-critical control commands
- **Reliability**: Quality of Service (QoS) levels ensure message delivery guarantees  
- **Scalability**: Easy addition of new devices without protocol changes
- **Debugging**: All messages can be monitored and logged for troubleshooting

The broker runs locally on the Jetson device, eliminating external network dependencies and ensuring consistent performance.

### <a id="topic-structure"></a>Topic Structure

The topic hierarchy follows a device-based naming convention for clear message routing:

#### **Jetson Publishes:**
- `pi/command`: UI state changes and game mode transitions
- `pi/tracking_status`: Ball detection events and system status
- `pi/image_feed`: Real-time camera feed with path overlays
- `pi/ball_info`: Ball position coordinates and tracking confidence
- `arduino/speed`: Motor control commands (X/Y velocities)

#### **Jetson Subscribes:**
- `jetson/command`: User interface commands and mode requests
- `jetson/handshake`: Device connection and status verification

#### **Pi Subscribes:**
- `pi/command`: System state updates and display instructions
- `pi/tracking_status`: Ball detection status for UI indicators
- `pi/image_feed`: Camera feed for real-time maze visualization
- `pi/ball_info`: Ball position for overlay graphics

### <a id="handshake-protocol"></a>Handshake Protocol

Device initialization follows a structured handshake sequence to ensure reliable connection establishment:

1. **Pi Startup**: Publishes connection request to `jetson/handshake`
2. **Jetson Response**: Acknowledges connection and sends initial state
3. **State Synchronization**: Exchange current system mode and configuration
4. **Heartbeat Establishment**: Begin periodic status updates

Example handshake sequence:
```python
# Pi initiates connection
mqtt_client.publish("jetson/handshake", "pi_connected")

# Jetson acknowledges and syncs state  
mqtt_client.publish("pi/command", "show_main_screen")
mqtt_client.publish("pi/tracking_status", "system_ready")
```

### <a id="jetson-mqtt-client"></a>Jetson MQTT Client

The Jetson MQTT client (`MQTTClientJetson`) manages all communication from the control system:

```python
class MQTTClientJetson:
    def __init__(self, broker_host="localhost", broker_port=1883):
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.image_buffer = []
        
    def on_connect(self, client, userdata, flags, rc):
        """Subscribe to command topics on connection"""
        client.subscribe("jetson/command")
        client.subscribe("jetson/handshake")
        
    def on_message(self, client, userdata, msg):
        """Route messages to finite state machine"""
        topic = msg.topic
        payload = msg.payload.decode()
        
        if topic == "jetson/command":
            self.hmi_controller.on_command(payload)
```

**Key Features:**
- **Image Buffering**: Manages camera feed transmission with compression
- **Command Routing**: Forwards UI commands to the finite state machine
- **Connection Management**: Handles reconnection and error recovery
- **Message Queuing**: Buffers messages during temporary disconnections

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

**Responsibilities:**
- **UI State Management**: Updates interface based on system state changes
- **User Input Processing**: Captures touchscreen interactions and forwards commands
- **Visual Feedback**: Displays ball tracking status and game progress
- **Image Display**: Renders real-time camera feed with path overlays

### <a id="message-formats"></a>Message Formats

#### **Ball Info Message**
```json
{
  "x": 145.7,
  "y": 203.2,
  "confidence": 0.89,
  "timestamp": 1677123456.789
}
```

#### **Camera Feed**
Images are transmitted as base64-encoded JPEG data with metadata:
```json
{
  "image": "base64_encoded_jpeg_data",
  "timestamp": 1677123456.789,
  "path_overlay": true,
  "ball_position": [145.7, 203.2]
}
```

#### **Command Messages**
Simple string commands for state transitions:
- `"Practice"` - Enter manual control mode
- `"PlayAlone"` - Start timed single-player game
- `"StartGame"` - Begin timer for current game mode
- `"Back"` - Return to previous state
- `"Locate"` - Initiate ball detection and tracking

### <a id="command-flow"></a>Command Flow

Typical command sequences for different operations:

**Starting a Navigation Session:**
```
Pi → jetson/command: "Navigate"
Jetson → pi/command: "show_navigation_screen" 
Pi → jetson/command: "Locate"
Jetson → pi/tracking_status: "tracking_started"
Jetson → pi/image_feed: [camera_feed_with_ball_detection]
```

**Autonomous Path Execution:**
```
Pi → jetson/command: "AutoPath"
Jetson → pi/command: "show_path_planning"
Jetson → pi/image_feed: [feed_with_path_overlay]
Jetson → arduino/speed: "120,85"  # X,Y motor commands
Jetson → pi/tracking_status: "goal_reached"
```

### <a id="mqtt-troubleshooting"></a>MQTT Troubleshooting

**Connection Issues:**
- Verify broker is running: `sudo systemctl status mosquitto`
- Check network connectivity between devices
- Monitor broker logs: `sudo tail -f /var/log/mosquitto/mosquitto.log`

**Message Delivery Problems:**
- Use MQTT client tools to verify topic publishing: `mosquitto_pub -h localhost -t test -m "hello"`
- Monitor all traffic: `mosquitto_sub -h localhost -t "#"`
- Check QoS settings for critical messages

**Performance Optimization:**
- Adjust keep-alive intervals for faster connection detection
- Use message compression for large image data
- Implement message priority queuing for time-critical commands

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
class CameraManager:
    def __init__(self, config):
        self.zed = sl.Camera()
        self.init_params = sl.InitParameters()
        self.init_params.camera_resolution = sl.RESOLUTION.HD720
        self.init_params.camera_fps = 60
        self.runtime_params = sl.RuntimeParameters()
        
    def get_frame(self):
        """Capture frame with hardware acceleration"""
        if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
            return self.image.get_data()[:, :, :3]  # BGR format
        return None
```

**Key Features:**
- **High Framerate**: 60 FPS capture for smooth tracking
- **Hardware Acceleration**: GPU-accelerated frame processing
- **Stereo Capability**: Depth information available for future enhancements
- **Auto-exposure**: Adaptive exposure for varying lighting conditions

### <a id="tensorrt-yolo-inference"></a>TensorRT YOLO Inference

The `YOLOModel` class implements optimized inference with TensorRT acceleration:

```python
class YOLOModel:
    def __init__(self, model_path, device='cuda'):
        self.device = device
        self.model_path = model_path
        self.trt_model = None
        self.pytorch_model = None
        self.cuda_context = None
        
    def _init_tensorrt(self):
        try:
            import tensorrt as trt
            import pycuda.driver as cuda
            
            cuda.init()
            self.cuda_context = cuda.Device(0).make_context()
            
            with open(self.trt_model_path, 'rb') as f:
                self.trt_engine = trt.Runtime(trt.Logger()).deserialize_cuda_engine(f.read())
                
        except Exception as e:
            print(f"TensorRT initialization failed: {e}")
            self._fallback_to_pytorch()
    
    def _predict_tensorrt(self, image):
        input_tensor = self._preprocess_image(image)
        
        with self.trt_engine.create_execution_context() as context:
            outputs = self._execute_inference(context, input_tensor)
            
        return self._postprocess_detections(outputs)
```

**Performance Benefits:**
- **5-10x Faster**: TensorRT optimization significantly reduces inference time
- **Lower Memory Usage**: Optimized memory allocation and reuse
- **Consistent Latency**: Predictable inference times for real-time control
- **Graceful Fallback**: Automatic PyTorch fallback if TensorRT unavailable

### <a id="ball-tracking"></a>Ball Tracking

The `BallTracker` class implements a producer-consumer pattern for continuous ball tracking:

```python
class BallTracker:
    def __init__(self, camera_manager, yolo_model, config):
        self.camera_manager = camera_manager
        self.yolo_model = yolo_model
        self.config = config
        self.running = False
        self.current_position = None
        self.position_lock = threading.Lock()
        
    def consumer_loop(self):
        while self.running:
            frame = self.camera_manager.get_frame()
            if frame is None:
                continue
                
            detections = self.yolo_model.predict(frame)
            ball_position = self._process_detections(detections)
            
            if ball_position and self._is_point_in_maze(ball_position):
                with self.position_lock:
                    self.current_position = self._smooth_position(ball_position)
                    
    def _is_point_in_maze(self, point):
        x, y = point
        maze_bounds = self.config['maze']['boundaries']
        return (maze_bounds['x_min'] <= x <= maze_bounds['x_max'] and
                maze_bounds['y_min'] <= y <= maze_bounds['y_max'])
```

**Key Features:**
- **Threading**: Separate detection thread prevents blocking the control loop
- **Boundary Validation**: Rejects detections outside valid maze area
- **Position Smoothing**: Exponential moving average reduces noise
- **Thread Safety**: Proper locking for concurrent position access

### <a id="tracker-service"></a>Tracker Service

The `TrackerService` provides a high-level interface for the tracking system:

```python
class TrackerService:
    def __init__(self, config):
        self.config = config
        self.camera_manager = CameraManager(config)
        self.yolo_model = YOLOModel(config['model']['path'])
        self.ball_tracker = None
        self.started = False
        
    def start_tracker(self):
        if not self.started:
            self.ball_tracker = BallTracker(
                self.camera_manager, 
                self.yolo_model, 
                self.config
            )
            self.ball_tracker.start()
            self.started = True
            
    def get_ball_position(self):
        if self.ball_tracker:
            return self.ball_tracker.get_position()
        return None
```

### <a id="vision-utilities"></a>Vision Utilities

Supporting utilities provide additional functionality:

**Position Smoothing**: Exponential moving average reduces tracking noise
```python
def smooth_position(current, new_pos, alpha=0.7):
    """Apply exponential smoothing to reduce noise"""
    if current is None:
        return new_pos
    return (alpha * new_pos[0] + (1-alpha) * current[0],
            alpha * new_pos[1] + (1-alpha) * current[1])
```

**Confidence Filtering**: Reject low-confidence detections to improve reliability
```python
def filter_by_confidence(detections, min_confidence=0.7):
    """Filter detections below confidence threshold"""
    return [det for det in detections if det['confidence'] >= min_confidence]
```

### <a id="detection-troubleshooting"></a>Detection Troubleshooting

**Common Issues and Solutions:**

**Ball Detection Failures:**
- Check lighting conditions, even illumination without shadows is ideal
- Verify camera focus and positioning for clear ball visibility
- Adjust confidence threshold
- Monitor GPU memory usage as insufficient VRAM can cause inference failures

**Position Accuracy Problems:**
- Calibrate maze boundary settings to match physical layout
- Increase position smoothing factor for noisy environments
- Check camera mounting stability as vibration causes tracking errors

**Configuration Parameters:**
```yaml
tracking:
  confidence_threshold: 0.7    # Minimum detection confidence
  smoothing_factor: 0.8        # Position smoothing (0-1)
  boundary_tolerance: 10       # Pixels outside boundary to allow
  max_detection_age: 0.5       # Seconds before considering ball lost
```

---

## <a id="a-pathfinding"></a>A* Pathfinding

### <a id="a-algorithm-with-repulsion-field"></a>A* Algorithm with Repulsion Field

The pathfinding system uses an enhanced A* algorithm with repulsion fields to generate safe, efficient paths through the maze. Unlike standard A*, this implementation adds artificial potential fields around obstacles to create paths that maintain safe distances from walls and holes.

#### **Heuristic Function**
The heuristic combines Euclidean distance with repulsion potential:

```python
def heuristic_with_repulsion(current, goal, repulsion_map, weight=10):
    euclidean_dist = np.sqrt((current[0] - goal[0])**2 + (current[1] - goal[1])**2)
    repulsion_penalty = repulsion_map[current[1], current[0]] * weight
    
    return euclidean_dist + repulsion_penalty
```

#### **Repulsion Map Generation**
Repulsion fields are precomputed using distance transforms:

```python
def create_repulsion_map(binary_mask, min_safe_dist=20):
    distance_map = cv2.distanceTransform(binary_mask, cv2.DIST_L2, 5)
    repulsion_map = np.zeros_like(distance_map)
    repulsion_map[distance_map < min_safe_dist] = (min_safe_dist - distance_map[distance_map < min_safe_dist]) / min_safe_dist
    
    return repulsion_map
```

#### **Path Search Implementation**
The core A* search integrates repulsion costs:

```python
def astar_with_repulsion(start, goal, binary_mask, repulsion_map):
    """A* pathfinding with obstacle repulsion"""
    open_set = [(0, start)]
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic_with_repulsion(start, goal, repulsion_map)}
    
    while open_set:
        current = heapq.heappop(open_set)[1]
        
        if current == goal:
            return reconstruct_path(came_from, current)
            
        for neighbor in get_neighbors(current, binary_mask):
            tentative_g = g_score[current] + distance(current, neighbor)
            
            # Add repulsion cost to movement
            tentative_g += repulsion_map[neighbor[1], neighbor[0]]
            
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic_with_repulsion(neighbor, goal, repulsion_map)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
```

### <a id="downscaled-pathfinding"></a>Downscaled Pathfinding

For computational efficiency, pathfinding operates on downscaled maze representations:

```python
def astar_downscaled(start, goal, binary_mask, scale=4):
    """Compute path on downscaled maze for performance"""
    h, w = binary_mask.shape
    downscaled_mask = cv2.resize(binary_mask, (w//scale, h//scale), interpolation=cv2.INTER_NEAREST)
    
    # Scale coordinates
    scaled_start = (start[0]//scale, start[1]//scale)
    scaled_goal = (goal[0]//scale, goal[1]//scale)
    
    # Find path on downscaled maze
    scaled_path = astar_with_repulsion(scaled_start, scaled_goal, downscaled_mask, create_repulsion_map(downscaled_mask))
    
    # Upscale path coordinates
    full_path = [(x*scale, y*scale) for x, y in scaled_path]
    
    return full_path
```

**Benefits:**
- **4-16x Faster**: Reduced search space dramatically improves performance
- **Memory Efficient**: Lower memory usage for large maze representations
- **Quality Preservation**: Careful scaling maintains path quality

### <a id="binary-mask-preprocessing"></a>Binary Mask Preprocessing

High-quality binary masks are essential for reliable pathfinding:

#### **CLAHE and Thresholding**
```python
def preprocess_maze_image(image):
    """Convert camera image to binary walkable mask"""
    # Convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # CLAHE for contrast enhancement
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    enhanced = clahe.apply(gray)
    
    # Adaptive thresholding for varying lighting
    binary = cv2.adaptiveThreshold(enhanced, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
    
    return binary
```

#### **Edge Removal and Morphology**
```python
def clean_binary_mask(binary_mask):
    """Remove artifacts and clean up mask"""
    # Remove edge artifacts
    binary_mask[:10, :] = 0  # Top edge
    binary_mask[-10:, :] = 0  # Bottom edge
    binary_mask[:, :10] = 0  # Left edge  
    binary_mask[:, -10:] = 0  # Right edge
    
    # Morphological operations
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    cleaned = cv2.morphologyEx(binary_mask, cv2.MORPH_CLOSE, kernel)
    cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_OPEN, kernel)
    
    return cleaned
```

#### **Green Area Suppression**
Remove green goal areas from pathfinding masks:

```python
def suppress_green_areas(image, binary_mask):
    """Remove green goal regions from walkable area"""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Green color range
    lower_green = np.array([40, 50, 50])
    upper_green = np.array([80, 255, 255])
    
    green_mask = cv2.inRange(hsv, lower_green, upper_green)
    
    # Remove green areas from walkable mask
    binary_mask[green_mask > 0] = 0
    
    return binary_mask
```

### <a id="waypoint-sampling"></a>Waypoint Sampling

The waypoint sampling system transforms raw A* paths into optimized trajectories suitable for physical ball control. The process combines multiple geometric algorithms to balance path smoothness, control accuracy, and computational efficiency.

#### **Theoretical Foundation**

The system employs a multi-stage optimization pipeline:

1. **Raw Path Densification**: Ensures continuous coverage along path segments by adding intermediate points at regular intervals
2. **Direction Change Detection**: Identifies significant course changes requiring explicit waypoints for accurate navigation
3. **L-Pivot Smoothing**: Inserts intermediate waypoints at corners to enable smooth physical transitions
4. **Douglas-Peucker Simplification**: Reduces redundant waypoints while preserving path geometry within specified tolerance
5. **Statistical Refinement**: Applies segment length analysis to maintain consistent waypoint spacing

#### **Implementation**

The core sampling algorithm processes raw A* output through several transformation stages:

```python
def sample_waypoints(path, mask):
    raw = [path[0]]
    y, x = path[0]
    for ty, tx in path[1:]:
        dx, sx = tx-x, 1 if tx>x else -1
        for xx in range(x+sx, tx+sx, sx):
            if mask[y][xx]:
                raw.append((y, xx))
        x = tx
        dy, sy = ty-y, 1 if ty>y else -1
        for yy in range(y+sy, ty+sy, sy):
            if mask[yy][x]: 
                raw.append((yy, x))
        y = ty

    pts = [raw[0]]
    for i, (prev, curr, nxt) in enumerate(zip(raw, raw[1:], raw[2:])):
        dir1 = (curr[0] - prev[0], curr[1] - prev[1])
        dir2 = (nxt[0] - curr[0], nxt[1] - curr[1])
        
        if dir1 != dir2 and i % 1 == 0:
            pts.append(curr)
    pts.append(raw[-1])

    full = [pts[0]]
    for a, b, c in zip(pts, pts[1:], pts[2:]):
        if (a[0]==b[0] and b[1]!=c[1]) or (a[1]==b[1] and b[0]!=c[0]):
            for p in ((a[0], c[1]), (c[0], c[1])):
                if p != full[-1] and clear_path(full[-1], p, mask):
                    full.append(p)
        if b != full[-1]:
            full.append(b)
    if pts[-1] != full[-1]:
        full.append(pts[-1])

    seg_dists = [math.hypot(y2-y1, x2-x1) for (y1,x1),(y2,x2) in zip(full, full[1:])]
    mean_seg = sum(seg_dists)/len(seg_dists) if seg_dists else 0.0
    raw_len = len(raw)
    eps = mean_seg * math.log(raw_len + 1)
    
    waypoints = _douglas_peucker(full, eps)

    if len(waypoints) >= 2:
        seg_lens = [
            math.hypot(b[0] - a[0], b[1] - a[1])
            for a, b in zip(waypoints[:-1], waypoints[1:])
        ]
        
        avg = statistics.mean(seg_lens)
        std = statistics.stdev(seg_lens) if len(seg_lens) > 1 else 0.0
        threshold = avg + std
        
        new_wps = [waypoints[0]]
        for i in range(1, len(waypoints)):
            a, b = waypoints[i - 1], waypoints[i]
            dist = math.hypot(b[0] - a[0], b[1] - a[1])
            if dist > threshold:
                mid = ((a[0] + b[0]) // 2, (a[1] + b[1]) // 2)
                new_wps.append(mid)
            new_wps.append(b)
        waypoints = new_wps
        
    return waypoints
```

#### **Douglas-Peucker Algorithm**

The system uses the Douglas-Peucker algorithm for intelligent path simplification:

```python
def _douglas_peucker(pts, eps):
    if len(pts) < 3:
        return pts
        
    a, b = pts[0], pts[-1]
    idx, dmax = 0, 0.0
    for i in range(1, len(pts)-1):
        d = _perp_dist(pts[i], a, b)
        if d > dmax:
            idx, dmax = i, d
            
    if dmax <= eps:
        return [a, b]
    
    left = _douglas_peucker(pts[:idx+1], eps)
    right = _douglas_peucker(pts[idx:], eps)
    return left[:-1] + right

def _perp_dist(pt, a, b):
    (y0, x0), (y1, x1), (y2, x2) = pt, a, b
    num = abs((x1-x0)*(y2-y0) - (x2-x0)*(y1-y0))
    den = math.hypot(y2-y1, x2-x1) or 1.0
    return num/den
```

### <a id="path-drawing"></a>Path Drawing

Visual path representation for debugging and user feedback:

```python
def draw_path(image, waypoints, start, goal):
    out = image.copy()

    if len(out.shape) == 2 or out.shape[2] == 1:
        out = cv2.cvtColor(out, cv2.COLOR_GRAY2BGR)
    elif out.shape[2] == 4:
        out = cv2.cvtColor(out, cv2.COLOR_BGRA2BGR)

    h, w = out.shape[:2]

    for x, y in waypoints or []:
        if 0 <= y < h and 0 <= x < w:
            cv2.circle(out, (x, y), 4, (128, 0, 0), -1)

    if start:
        cv2.circle(out, (start[1], start[0]), 5, (0, 255, 0), -1)
    if goal:
        cv2.circle(out, (goal[1], goal[0]), 5, (255, 0, 0), -1)

    return out
```

### <a id="path-memory-caching"></a>Path Memory Caching

Caching reduces computation for repeated queries:

```python
def get_cached_path(self, start, goal):
        for entry in self.paths:
            s, g, path = entry["start"], entry["goal"], entry["path"]
            if self._within_tolerance(start, s) and self._within_tolerance(goal, g):
                return path
        return None

def cache_path(self, start, goal, path):
    if len(self.paths) >= self.max_paths:
        self.paths.pop(0)

    self.paths.append({
        "start": list(start),
        "goal": list(goal),
        "path": [list(p) for p in path]
    })
    print(f"[PathMemory] Cached path. Total stored: {len(self.paths)}")
    self.save_cache()
```

### <a id="nearest-walkable-point"></a>Nearest Walkable Point

Handles cases where start/goal points are in invalid areas:

```python
def find_nearest_walkable(mask, point, max_radius=10):
    y, x = point
    if mask[y, x] != 0:
        return point

    dist = cv2.distanceTransform(mask.astype(np.uint8), cv2.DIST_L2, 5)
    min_val = float('inf')
    nearest = point

    h, w = mask.shape
    for dy in range(-max_radius, max_radius + 1):
        for dx in range(-max_radius, max_radius + 1):
            ny, nx = y + dy, x + dx
            if 0 <= ny < h and 0 <= nx < w and mask[ny, nx] > 0:
                if dist[ny, nx] < min_val:
                    min_val = dist[ny, nx]
                    nearest = (ny, nx)
    return nearest
```

### <a id="pathfinding-troubleshooting"></a>Pathfinding Troubleshooting

**Common Issues and Solutions:**

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

#### **scale in astar_downscaled()**
Downscaling factor for performance optimization:
- **Scale=2**: High accuracy, slower computation
- **Scale=4**: Good balance of speed and quality
- **Scale=8**: Fast computation, may miss narrow passages

#### **cache tolerance**
Distance threshold for path cache hits:
- **Too Strict (<5 pixels)**: Frequent cache misses, poor performance
- **Too Loose (>20 pixels)**: Inaccurate cached paths, navigation errors
- **Sweet Spot**: 8-12 pixels for typical maze scales

---

## <a id="authors"></a>Authors

