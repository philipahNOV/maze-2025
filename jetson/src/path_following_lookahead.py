import position_controller
import numpy as np
import time

class PathFollower:
    def __init__(self, path_array, controller: position_controller.Controller):
        self.path = [(x, y) for x, y in path_array]
        self.controller = controller
        self.length = len(self.path)

        self.lookahead_distance = 90  # pixels
        self.acceptance_radius = controller.pos_tol

        self.prev_time = None
        self.prev_ball_pos = None
        self.vel_threshold = 50
        self.prev_progress_time = None
        self.stuck_retry_time = 3
        self.lookahead_point = None

        # Optimization: cache the last closest segment index
        self.last_closest_index = 0

        # Limit how far ahead we can skip (in waypoints)
        self.max_skip_ahead = int(self.length / 20)  # How far ahead we can search
        self.max_skip_behind = int(self.length / 10)  # How far back we can search

    def follow_path(self, ballPos):
        if not ballPos:
            return

        # Compute velocity
        vel = None
        if self.prev_time and self.prev_ball_pos:
            dt = time.time() - self.prev_time
            if dt > 0:
                vel = np.linalg.norm(np.array(ballPos) - np.array(self.prev_ball_pos)) / dt

        self.prev_time = time.time()
        self.prev_ball_pos = ballPos

        # Handle stuck detection
        if not self.prev_progress_time and vel:
            if vel < self.vel_threshold:
                self.prev_progress_time = time.time()
        elif self.prev_progress_time:
            if vel and vel > self.vel_threshold:
                self.prev_progress_time = None
            elif time.time() - self.prev_progress_time > self.stuck_retry_time:
                print("[STUCK] Ball not progressing, stopping control temporarily.")
                self.prev_progress_time = None
                #self.controller.arduinoThread.send_speed(0, 0)
                return

        # Compute lookahead target
        lookahead_point = self.get_lookahead_point(ballPos, self.lookahead_distance)

        # Feedforward direction
        dx = lookahead_point[0] - ballPos[0]
        dy = lookahead_point[1] - ballPos[1]
        norm = np.linalg.norm([dx, dy])
        if norm > 0:
            self.controller.feedforward_vector = (dx / norm, dy / norm)
        else:
            self.controller.feedforward_vector = (0, 0)

        # Send position command to controller
        self.lookahead_point = lookahead_point
        self.controller.posControl(lookahead_point)

    def get_lookahead_point(self, ball_pos, lookahead_dist=100):
        min_dist = float('inf')
        closest_index = self.last_closest_index
        closest_proj = None
        ball_pos_np = np.array(ball_pos)

        # Allow limited backward and forward search around cached index
        start_index = max(0, self.last_closest_index - self.max_skip_behind)
        end_index = min(self.length - 1, self.last_closest_index + self.max_skip_ahead)
        search_range = range(start_index, end_index)

        for i in search_range:
            a = np.array(self.path[i])
            b = np.array(self.path[i + 1])
            proj = self._project_point_onto_segment(ball_pos_np, a, b)
            dist = np.linalg.norm(ball_pos_np - proj)
            if dist < min_dist:
                min_dist = dist
                closest_index = i
                closest_proj = proj

        self.last_closest_index = closest_index  # Update cache

        # Traverse forward from closest point
        dist_acc = 0
        for j in range(closest_index, self.length - 1):
            a = np.array(self.path[j])
            b = np.array(self.path[j + 1])
            seg_len = np.linalg.norm(b - a)
            if dist_acc + seg_len >= lookahead_dist:
                ratio = (lookahead_dist - dist_acc) / seg_len
                lookahead_point = a + (b - a) * ratio
                return tuple(lookahead_point)
            dist_acc += seg_len

        # Reached end of path — reset
        print("[LOOP] End of path reached. Resetting to start.")
        self.last_closest_index = 0
        return self.path[0]

    def _project_point_onto_segment(self, p, a, b):
        ap = p - a
        ab = b - a
        t = np.dot(ap, ab) / np.dot(ab, ab)
        t = np.clip(t, 0, 1)
        return a + t * ab