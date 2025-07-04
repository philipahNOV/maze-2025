from path_following import PathFollower
import cv2
import time
import base64
from mqtt_client import MQTTClientJetson

class ImageController():
    def __init__(self):
        self.frame_corners = [[(390, 10), (1120, 10)], [(390, 720), (1120, 720)]]
        self.frame = None
        self.cropped_frame = None
        self.last_sent_frame_time = time.time()
        self.frame_send_hz = 5

    def draw_waypoints(self, pathFollower: PathFollower):
        if self.frame is None: return

        for i in range(pathFollower.length):
                if i < pathFollower.next_waypoint:
                    cv2.circle(self.frame, pathFollower.path[i], 5, (0, 200, 0), -1)
                    continue
                elif i == pathFollower.next_waypoint:
                    cv2.circle(self.frame, pathFollower.path[i], 5, (0, 255, 255), -1)
                else:
                    cv2.circle(self.frame, pathFollower.path[i], 5, (0, 0, 255), -1)

    def draw_ball(self, ball_pos):
        if self.frame is None or ball_pos is None: return
        cv2.circle(self.frame, ball_pos, 8, (255, 165, 0), -1)

    def fix_frame(self):
        if self.frame is None: return
        x1, y1 = self.frame_corners[0][0]
        x2, y2 = self.frame_corners[1][1]
        self.cropped_frame = self.frame[y1:y2, x1:x2]
        self.cropped_frame = cv2.rotate(self.cropped_frame, cv2.ROTATE_180)
        return self.cropped_frame

    def send_frame_to_pi(self, mqtt_client: MQTTClientJetson):
        if time.time() > self.last_sent_frame_time + 1/self.frame_send_hz:
            scale = 0.5  # 50% of original size
            height, width = self.cropped_frame.shape[:2]

            new_size = (int(width * scale), int(height * scale))
            resized = cv2.resize(self.cropped_frame, new_size, interpolation=cv2.INTER_AREA)
            _, buffer = cv2.imencode('.jpg', resized, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
            jpg_as_text = base64.b64encode(buffer).decode('utf-8')

            mqtt_client.client.publish("pi/camera", jpg_as_text)
            self.last_sent_frame_time = time.time()

    def update(self, ballPos, pathFollower: PathFollower, mqtt_client: MQTTClientJetson):
        self.draw_waypoints(pathFollower)
        self.draw_ball(ballPos)
        self.fix_frame()
        self.send_frame_to_pi(mqtt_client)
        return self.cropped_frame
