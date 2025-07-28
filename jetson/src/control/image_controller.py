from control.paths.path_following import PathFollower
from control.paths.path_following_lookahead import PathFollower as PathFollowerLookahead
import numpy as np
import cv2
import time
import base64
from mqtt.mqtt_client import MQTTClientJetson
import threading
from control.astar.draw_path import draw_path

class ImageController:
    """
    Handles drawing and formatting of video frames for display and transmission.

    Responsibilities:
    - Drawing ball and waypoint overlays
    - Cropping and rotating frames
    - Resizing and sending frames to Raspberry Pi over MQTT
    """

    def __init__(self, config):
        # Crop coordinates: [(top-left, top-right), (bottom-left, bottom-right)]
        self.padding = config['camera'].get('padding', 10)  # Padding around the maze image
        self.top_left = config['camera'].get('top_left', (430, 27))
        self.top_right = config['camera'].get('top_right', (1085, 27))
        self.bottom_left = config['camera'].get('bottom_left', (430, 682))
        self.bottom_right = config['camera'].get('bottom_right', (1085, 682))

        # Apply padding outward from each corner
        self.top_left = (self.top_left[0] - self.padding, self.top_left[1] - self.padding)
        self.top_right = (self.top_right[0] + self.padding, self.top_right[1] - self.padding)
        self.bottom_left = (self.bottom_left[0] - self.padding, self.bottom_left[1] + self.padding)
        self.bottom_right = (self.bottom_right[0] + self.padding, self.bottom_right[1] + self.padding)
        self.frame_corners = (self.top_left, self.top_right, self.bottom_left, self.bottom_right)

        self.frame = None
        self.cropped_frame = None
        self.last_sent_frame_time = time.time()
        self.frame_send_hz = 5  # Number of frames to send per second
        self.current_path = None  # Holds the active path
        self.new_path_available = False  # Flag to track if we need to update the drawing
        self.maze_angle = config['camera'].get('maze_relative_angle', 1.0)  # Angle of the maze relative to the camera frame

    def draw_waypoints(self, pathFollower: PathFollower):
        """Draw path waypoints on the current frame with color coding."""
        if self.frame is None or self.current_path is None:
            return

        for i in range(pathFollower.length):
            if i < pathFollower.next_waypoint:
                color = (0, 200, 0)  # Green for past waypoints
            elif i == pathFollower.next_waypoint:
                color = (0, 255, 255)  # Yellow for current target
            else:
                color = (0, 0, 255)  # Red for future waypoints
            cv2.circle(self.frame, pathFollower.path[i], 5, color, -1)

    def set_new_path(self, path):
        if path and len(path) > 0:
            self.current_path = path
            self.new_path_available = True
        else:
            self.current_path = None

    def draw_waypoints_lookahead(self, pathFollower: PathFollowerLookahead):
        #for i in range(pathFollower.length):
        #    if i < pathFollower.last_closest_index:
        #        color = (0, 200, 0)  # Green for past waypoints
        #    elif i == pathFollower.last_closest_index:
        #        color = (0, 255, 255)  # Yellow for current target
        #    else:
        #        color = (0, 0, 255)  # Red for future waypoints
        #    cv2.circle(self.frame, pathFollower.path[i], 5, color, -1)
        if pathFollower.lookahead_point is not None:
            pt = pathFollower.lookahead_point
            if pt is not None and all(np.isfinite(pt)):
                cv2.circle(self.frame, tuple(map(int, pt)), 7, (255, 0, 0), -1)

    def draw_waypoints_simple(self, path):
        if self.frame is None or path is None:
            return

        for point in path:
            cv2.circle(self.frame, point, 5, (255, 0, 0), -1)

    def draw_ball(self, ball_pos):
        """Draw the ball position on the frame."""
        if self.frame is None or ball_pos is None:
            return
        cv2.circle(self.frame, ball_pos, 8, (255, 165, 0), -1)

    def crop_and_rotate_frame(self):
        if self.frame is None:
            return None

        x1, y1 = self.frame_corners[0]
        x2, y2 = self.frame_corners[3]
        cropped = self.frame[y1:y2, x1:x2]
        rotated = cv2.rotate(cropped, cv2.ROTATE_180)
        angle = self.maze_angle  # degrees, negative = clockwise
        (h, w) = rotated.shape[:2]
        center = (w // 2, h // 2)

        M = cv2.getRotationMatrix2D(center, angle, 1.0)
        rotated_corrected = cv2.warpAffine(rotated, M, (w, h), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT)
        self.cropped_frame = rotated_corrected

    def send_frame_to_pi(self, mqtt_client: MQTTClientJetson):
        if self.cropped_frame is None:
            return

        if time.time() > self.last_sent_frame_time + 1 / self.frame_send_hz:
            scale = 0.5
            height, width = self.cropped_frame.shape[:2]
            new_size = (int(width * scale), int(height * scale))

            resized = cv2.resize(self.cropped_frame, new_size, interpolation=cv2.INTER_AREA)
            _, buffer = cv2.imencode('.jpg', resized, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
            jpg_as_text = base64.b64encode(buffer).decode('utf-8')

            mqtt_client.client.publish("pi/camera", jpg_as_text)
            self.last_sent_frame_time = time.time()

    def update(self, ballPos, pathFollower: PathFollower = None, mqtt_client: MQTTClientJetson = None, path=None):
        if pathFollower is not None:
            if isinstance(pathFollower, PathFollowerLookahead):
                self.draw_waypoints_lookahead(pathFollower)
            else:
                self.draw_waypoints(pathFollower)
        elif self.current_path:
            if self.new_path_available:
                self.frame = draw_path(self.frame, self.current_path, self.current_path[0], self.current_path[-1])
                self.new_path_available = False
            else:
                self.frame = draw_path(self.frame, self.current_path, None, None)
        self.draw_ball(ballPos)
        self.crop_and_rotate_frame()
        self.send_frame_to_pi(mqtt_client)
        return self.cropped_frame
    
class ImageSenderThread(threading.Thread):
    def __init__(self, image_controller: ImageController, mqtt_client: MQTTClientJetson, tracker_service, path, path_follower=None, stop_event=None):
        super().__init__(daemon=True)
        self.image_controller = image_controller
        self.mqtt_client = mqtt_client
        self.tracker_service = tracker_service
        self.path = path
        self.path_follower = path_follower
        self.running = False
        self.sleep_interval = 0.20  # 5 fps update loop; actual send rate is limited by image_controller
        self.stop_event = stop_event

    def run(self):
        self.running = True
        print("[ImageSenderThread] Started")
        while self.running and (self.stop_event is None or not self.stop_event.is_set()):
            frame = self.tracker_service.get_stable_frame()
            if frame is not None:
                self.image_controller.frame = frame.copy()
                self.image_controller.update(
                    ballPos=self.tracker_service.get_ball_position() if self.path_follower is not None else None,
                    pathFollower=self.path_follower,
                    mqtt_client=self.mqtt_client,
                    path=self.path
                )
            time.sleep(self.sleep_interval)

    def stop(self):
        self.running = False
        print("[ImageSenderThread] Stopped")