from path_following import PathFollower
from path_following_lookahead import PathFollower as PathFollowerLookahead
import numpy as np
import cv2
import time
import base64
from mqtt_client import MQTTClientJetson
import threading
from astar.draw_path import draw_path

class ImageController:
    """
    Handles drawing and formatting of video frames for display and transmission.

    Responsibilities:
    - Drawing ball and waypoint overlays
    - Cropping and rotating frames
    - Resizing and sending frames to Raspberry Pi over MQTT
    """

    def __init__(self):
        # Crop coordinates: [(top-left, top-right), (bottom-left, bottom-right)]
        self.frame_corners = [[(390, 10), (1120, 10)], [(390, 720), (1120, 720)]]
        self.frame = None
        self.cropped_frame = None
        self.last_sent_frame_time = time.time()
        self.frame_send_hz = 5  # Number of frames to send per second

    def draw_waypoints(self, pathFollower: PathFollower):
        """Draw path waypoints on the current frame with color coding."""
        if self.frame is None:
            return

        for i in range(pathFollower.length):
            if i < pathFollower.next_waypoint:
                color = (0, 200, 0)  # Green for past waypoints
            elif i == pathFollower.next_waypoint:
                color = (0, 255, 255)  # Yellow for current target
            else:
                color = (0, 0, 255)  # Red for future waypoints
            cv2.circle(self.frame, pathFollower.path[i], 5, color, -1)

    def draw_waypoints_lookahead(self, pathFollower: PathFollowerLookahead):
        cv2.circle(self.frame, tuple(map(int, pathFollower.lookahead_point)), 5, (100, 200, 100), -1)
        #for i in range(pathFollower.length):
        #    cv2.circle(self.frame, pathFollower.path[i], 5, (255, 0, 0), -1)

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
        """Crop and rotate the frame based on predefined coordinates."""
        if self.frame is None:
            return None

        x1, y1 = self.frame_corners[0][0]
        x2, y2 = self.frame_corners[1][1]
        self.cropped_frame = self.frame[y1:y2, x1:x2]
        self.cropped_frame = cv2.rotate(self.cropped_frame, cv2.ROTATE_180)

    def send_frame_to_pi(self, mqtt_client: MQTTClientJetson):
        """Encode and send the cropped frame to the Raspberry Pi over MQTT."""
        if self.cropped_frame is None:
            return

        if time.time() > self.last_sent_frame_time + 1 / self.frame_send_hz:
            scale = 0.5  # Scale down for bandwidth efficiency
            height, width = self.cropped_frame.shape[:2]
            new_size = (int(width * scale), int(height * scale))

            resized = cv2.resize(self.cropped_frame, new_size, interpolation=cv2.INTER_AREA)
            _, buffer = cv2.imencode('.jpg', resized, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
            jpg_as_text = base64.b64encode(buffer).decode('utf-8')

            mqtt_client.client.publish("pi/camera", jpg_as_text)
            self.last_sent_frame_time = time.time()

    def update(self, ballPos, pathFollower: PathFollower = None, mqtt_client: MQTTClientJetson = None, path=None):
        """
        Main update routine: draw overlays, crop, rotate, and send frame.

        Args:
            ballPos (tuple): Ball position in pixel coordinates.
            pathFollower (PathFollower): Path planning module.
            mqtt_client (MQTTClientJetson): Communication module.

        Returns:
            np.ndarray: The final processed frame ready for display.
        """
        if pathFollower is not None:
            if isinstance(pathFollower, PathFollowerLookahead):
                self.draw_waypoints_lookahead(pathFollower)
            else:
                self.draw_waypoints(pathFollower)
        elif path is not None:
            if len(path) > 0:
                self.frame = draw_path(self.frame, path, path[0], path[-1])
        self.draw_ball(ballPos)
        self.crop_and_rotate_frame()
        self.send_frame_to_pi(mqtt_client)
        return self.cropped_frame
    
class ImageSenderThread(threading.Thread):
    def __init__(self, image_controller: ImageController, mqtt_client: MQTTClientJetson, tracker_service, path):
        super().__init__(daemon=True)
        self.image_controller = image_controller
        self.mqtt_client = mqtt_client
        self.tracker_service = tracker_service
        self.path = path
        self.running = False
        self.sleep_interval = 0.20  # 5 FPS update loop; actual send rate is limited by image_controller

    def run(self):
        self.running = True
        print("[ImageSenderThread] Started")
        while self.running:
            frame = self.tracker_service.get_stable_frame()
            if frame is not None:
                self.image_controller.frame = frame.copy()
                self.image_controller.update(
                    ball_pos=None,
                    pathFollower=None,
                    mqtt_client=self.mqtt_client,
                    path=self.path
                )
            time.sleep(self.sleep_interval)

    def stop(self):
        self.running = False
        print("[ImageSenderThread] Stopped")

