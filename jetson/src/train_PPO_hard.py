import testing.yolov1.hsv3 as tracking
import time
import cv2
import positionController
import arduino_connection
import lowPassFilter
import PPOfile_hardware
import numpy as np

def main():

    def plot_waypoints(frame, pathFollower):
        """Plot waypoints on the frame with current target highlighted."""
        for n in range(pathFollower.path_length):
            # Use different colors for current target vs other waypoints
            if n == pathFollower.current_waypoint_idx:
                color = (0, 255, 255)  # Yellow for current target
                radius = 8
            elif n < pathFollower.current_waypoint_idx:
                color = (0, 255, 0)    # Green for completed waypoints
                radius = 5
            else:
                color = (0, 0, 255)    # Red for future waypoints
                radius = 5
            
            # Convert to integer coordinates for cv2.circle
            waypoint = (int(pathFollower.path[n][0]), int(pathFollower.path[n][1]))
            cv2.circle(frame, waypoint, radius, color, -1)
            
            # Add waypoint number
            cv2.putText(frame, str(n), (waypoint[0] + 10, waypoint[1] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

    def plot_progress_info(frame, pathFollower):
        """Display progress information on the frame."""
        progress = pathFollower.get_progress()
        info_text = [
            f"Waypoint: {progress['current_waypoint']}/{progress['total_waypoints']}",
            f"Progress: {progress['progress_percentage']:.1f}%",
            f"AI Control: {'ON' if progress.get('ai_control_active', False) else 'OFF'}",
            f"Training: {'ACTIVE' if progress.get('training_active', False) else 'COMPLETE'}"
        ]
        
        y_offset = 30
        for i, text in enumerate(info_text):
            color = (0, 255, 0) if 'ON' in text or 'ACTIVE' in text else (255, 255, 255)
            cv2.putText(frame, text, (10, y_offset + i * 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    def initialize_component(component, name, retries=5, delay=2):
        for attempt in range(retries):
            try:
                comp_instance = component()
                print(f"{name} initialized on attempt {attempt + 1}")
                return comp_instance
            except Exception as e:
                print(f"Failed to initialize {name} on attempt {attempt + 1}: {e}")
                time.sleep(delay)
        raise Exception(f"Failed to initialize {name} after {retries} attempts")

    # Initialize Arduino connection
    try:
        arduino_thread = initialize_component(arduino_connection.ArduinoConnection, "ArduinoConnection")
        time.sleep(10)
    except Exception as e:
        print(e)
        exit(1)

    # Initialize tracking components
    tracker = tracking.BallTracker(model_path="testing/yolov1/best.pt")
    tracker.start()
    smoother = lowPassFilter.SmoothedTracker(alpha=0.5)

    print("[INFO] Waiting for YOLO initialization...")
    while not tracker.initialized:
        time.sleep(0.1)

    print("[INFO] Tracking started. Press 'q' to quit.")

    # Initialize controller
    controller = positionController.Controller(arduino_thread, tracker)
    
    # Define path
    path_array = [
        (50, 50), (150, 50), (250, 50), (350, 50), (450, 50), 
        (450, 150), (450, 250), (450, 350), (450, 450)
    ]
    
    # Initialize path follower with real-time training
    try:
        print("[INFO] Initializing Real-time PPO Path Follower...")
        print("[INFO] Training will happen while controlling the hardware!")
        
        # Use the simple real-time path follower that trains in background
        pathFollower = PPOfile_hardware.SimpleRealTimePathFollower(
            path_array=path_array, 
            controller=controller,
            quick_training_steps=5000  # Start with quick training
        )
        
        print("[INFO] Real-time path follower initialized!")
        print("[INFO] Basic control active, AI training in background...")
        
    except Exception as e:
        print(f"[ERROR] Failed to initialize path follower: {e}")
        print("[INFO] Falling back to basic controller...")
        pathFollower = None

    # Platform initialization
    time.sleep(1)
    controller.horizontal()
    time.sleep(2)
    
    # Main control loop
    try:
        start_time = time.time()
        path_complete = False
        
        while True:
            frame = tracker.frame
            if frame is None:
                continue

            # Plot waypoints and progress
            if pathFollower:
                plot_waypoints(frame, pathFollower)
                plot_progress_info(frame, pathFollower)

            # Get and smooth ball position
            ball_pos = tracker.get_position()
            if not ball_pos:
                print("No ball found", end="\r")
                cv2.imshow("Ball & Marker Tracking", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                continue
                
            ball_pos = smoother.update(ball_pos)

            # Draw ball position
            cv2.circle(frame, ball_pos, 8, (0, 255, 0), -1)
            cv2.putText(frame, "Ball", (ball_pos[0]+10, ball_pos[1]), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Display frame
            cv2.imshow("Ball & Marker Tracking", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # Control logic
            if pathFollower and not path_complete:
                try:
                    action = pathFollower.follow_path(ball_pos)
                    
                    if action is None:
                        print("[INFO] Path following complete!")
                        path_complete = True
                        # Optionally restart path
                        # pathFollower.reset_path_following()
                        # path_complete = False
                    else:
                        # Optional: Print progress occasionally
                        if int(time.time()) % 5 == 0:  # Every 5 seconds
                            progress = pathFollower.get_progress()
                            print(f"[INFO] Progress: {progress['progress_percentage']:.1f}%")
                            
                except Exception as e:
                    print(f"[ERROR] Path following error: {e}")
                    # Fall back to simple position control or stop
                    controller.horizontal()
            else:
                # Fallback control or manual control
                # Example: Simple position control to center
                # controller.posControl((400, 300))
                pass

    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user.")
    finally:
        tracker.stop()
        cv2.destroyAllWindows()
        print("[INFO] Tracker stopped.")

if __name__ == "__main__":
    main()