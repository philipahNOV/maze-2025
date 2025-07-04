from path_following import PathFollower
import cv2

class ImageController():
    def __init__(self):
        self.frame_corners = [[(390, 10), (1120, 10)], [(390, 720), (1120, 720)]]
        self.frame = None

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

    def get_fixed_frame(self):
        if self.frame is None: return
        x1, y1 = self.frame_corners[0][0]
        x2, y2 = self.frame_corners[1][1]
        cropped_frame = self.frame[y1:y2, x1:x2]
        cropped_frame = cv2.rotate(cropped_frame, cv2.ROTATE_180)
        return cropped_frame
