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

        self.looping = False
        self.forward = True

        self.filtered_lookahead = 80  # initial default
        self.alpha = 0.03             # low-pass factor, smaller = smoother
        self.min_lookahead = 60          # minimum lookahead distance
        self.max_lookahead = 100         # maximum lookahead distance

        self.last_reverse_time = None
        self.reverse_cooldown = 3  # seconds before another reversal is allowed

        # Optimization: cache the last closest segment index
        self.last_closest_index = 0

        # Limit how far ahead we can skip (in waypoints)
        self.max_skip_ahead = max(5, int(self.length / 5))
        self.max_skip_behind = max(5, int(self.length / 10))

    def follow_path(self, ballPos):
        if not ballPos:
            return

        self.looping = self.controller.looping

        # Compute velocity
        vel = None
        if self.prev_time and self.prev_ball_pos:
            dt = time.time() - self.prev_time
            if dt > 0:
                vel = np.linalg.norm(np.array(ballPos) - np.array(self.prev_ball_pos)) / dt

        if vel is not None:
            # Choose dynamic target between min and max
            k = 100  # Controls how steeply it ramps up — tweak as needed
            target_lookahead = self.min_lookahead + (
                (self.max_lookahead - self.min_lookahead) * (vel / (vel + k))
            )
            self.filtered_lookahead = (
                self.alpha * target_lookahead + (1 - self.alpha) * self.filtered_lookahead
            )
            self.filtered_lookahead = np.clip(self.filtered_lookahead, self.min_lookahead, self.max_lookahead)
            self.lookahead_distance = self.filtered_lookahead

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
        self.lookahead_point = lookahead_point

        # Feedforward direction
        dx = lookahead_point[0] - ballPos[0]
        dy = lookahead_point[1] - ballPos[1]
        norm = np.linalg.norm([dx, dy])
        if norm > 0:
            self.controller.feedforward_vector = (dx / norm, dy / norm)
        else:
            self.controller.feedforward_vector = (0, 0)

        # Send position command to controller
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

        total_dist = np.linalg.norm(ball_pos_np - closest_proj)
        a = closest_proj

        if self.forward:
            range_iter = range(closest_index, self.length - 1)
        else:
            range_iter = range(closest_index, 0, -1)

        for j in range_iter:
            b = np.array(self.path[j + 1] if self.forward else self.path[j - 1])

            seg_len = np.linalg.norm(b - a)
            if seg_len < 1e-6:
                continue  # skip zero-length segment

            if total_dist + seg_len >= lookahead_dist:
                ratio = (lookahead_dist - total_dist) / seg_len
                lookahead_point = a + (b - a) * ratio
                return tuple(lookahead_point)

            total_dist += seg_len
            a = b  # Move to next segment

        # Reached end/start of path — reverse if looping
        now = time.time()
        if self.looping and (self.last_reverse_time is None or now - self.last_reverse_time > self.reverse_cooldown):
            self.forward = not self.forward
            self.last_reverse_time = now
            print(f"[LOOKAHEAD] Reversed direction: {'→' if self.forward else '←'}")
            self.last_closest_index = self.length - 2 if self.forward else 1
            return self.path[self.last_closest_index]
        else:
            print("[LOOKAHEAD] No lookahead segment in range — using closest projection.")
            return tuple(closest_proj)

    def _project_point_onto_segment(self, p, a, b):
        ap = p - a
        ab = b - a

        if np.allclose(ab, 0):
            return a

        t = np.dot(ap, ab) / np.dot(ab, ab)
        t = np.clip(t, 0, 1)
        return a + t * ab