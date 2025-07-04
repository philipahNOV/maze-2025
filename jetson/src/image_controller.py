from path_following import PathFollower
import cv2

class ImageController():
    def __init__(self):
        self.frame_corners = [[(390, 10), (1120, 10)], [(390, 720), (390, 1120)]]
        self.frame = None

    def draw_waypoints(self, pathFollower: PathFollower):
        if self.frame == None: return

        for i in range(pathFollower.length):
                if i < pathFollower.next_waypoint:
                    cv2.circle(self.frame, pathFollower.path[i], 5, (0, 200, 0), -1)
                    continue
                elif i == pathFollower.next_waypoint:
                    cv2.circle(self.frame, pathFollower.path[i], 5, (0, 255, 255), -1)
                else:
                    cv2.circle(self.frame, pathFollower.path[i], 5, (0, 0, 255), -1)
