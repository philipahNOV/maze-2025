import control.position_controller as position_controller
import numpy as np
import time

class PathFollower:
    def __init__(self, path_array, controller: position_controller.Controller, config):
        self.path = [(x, y) for x, y in path_array]
        self.controller = controller
        self.length = len(self.path)
        self.config = config

        self.lookahead_distance = self.config["path_following"]["lookahead"].get("lookahead_distance", 80)  # pixels
        self.acceptance_radius = controller.pos_tol
        self.prev_time = None
        self.prev_ball_pos = None
        self.vel_threshold = 50
        self.prev_progress_time = None
        self.stuck_retry_time = 3
        self.lookahead_point = None
        self.lookahead_index = 0
        self.adaptive_lookahead = self.config["path_following"]["lookahead"].get("adaptive", False)
        self.curvature_gain = self.config["path_following"]["lookahead"].get("curvature_gain", 0.99)
        self.velocity_gain = self.config["path_following"]["lookahead"].get("velocity_gain", 100)

        self.looping = False
        self.forward = True

        self.path_np = np.array(path_array, dtype=np.float64)

        self.filtered_lookahead = self.lookahead_distance
        self.alpha = self.config["path_following"]["lookahead"].get("smoothing_alpha", 0.03)
        self.min_lookahead = self.config["path_following"]["lookahead"].get("lookahead_distance_min", 50)
        self.max_lookahead = self.config["path_following"]["lookahead"].get("lookahead_distance_max", 80)

        self.last_reverse_time = None
        self.reverse_cooldown = 3  # seconds before another reversal is allowed

        self.last_closest_index = 0

        self.max_skip_ahead = max(5, int(self.length / 10))
        self.max_skip_behind = max(5, int(self.length / 25))
        #self.max_skip_behind = 1

    def get_path_curvature_at_index(self, idx):
        if idx <= 0 or idx >= self.length - 2:
            return 0  # assume straight at ends

        p0 = np.array(self.path[idx - 1])
        p1 = np.array(self.path[idx])
        p2 = np.array(self.path[idx + 1])

        v1 = (p1 - p0).astype(np.float64)
        v2 = (p2 - p1).astype(np.float64)

        if np.linalg.norm(v1) < 1e-3 or np.linalg.norm(v2) < 1e-3:
            return 0

        v1 /= np.linalg.norm(v1)
        v2 /= np.linalg.norm(v2)

        dot = np.clip(np.dot(v1, v2), -1.0, 1.0)
        angle = np.arccos(dot)  # in radians

        return angle / np.pi

    def follow_path(self, ballPos):
        if not ballPos:
            return

        self.looping = self.controller.looping

        vel = None
        if self.prev_time and self.prev_ball_pos:
            dt = time.time() - self.prev_time
            if dt > 0:
                vel = np.linalg.norm(np.array(ballPos) - np.array(self.prev_ball_pos)) / dt

        if self.adaptive_lookahead:
            target_lookahead = self.lookahead_distance
            if vel is not None:
                target_lookahead = self.min_lookahead + (
                    (self.max_lookahead - self.min_lookahead) * (vel / (vel + self.velocity_gain))
                )

            curvature_scale = 1.0 - self.get_path_curvature_at_index(self.lookahead_index) * self.curvature_gain
            target_lookahead *= curvature_scale

            self.filtered_lookahead = (
                    self.alpha * target_lookahead + (1 - self.alpha) * self.filtered_lookahead
                )
            self.filtered_lookahead = np.clip(self.filtered_lookahead, self.min_lookahead, self.max_lookahead)
            self.lookahead_distance = self.filtered_lookahead
        
        self.prev_time = time.time()
        self.prev_ball_pos = ballPos

        if not self.prev_progress_time and vel:
            if vel < self.vel_threshold:
                self.prev_progress_time = time.time()
        elif self.prev_progress_time:
            if vel and vel > self.vel_threshold:
                self.prev_progress_time = None
            elif time.time() - self.prev_progress_time > self.stuck_retry_time:
                self.prev_progress_time = None
                return

        lookahead_point, self.lookahead_index = self.get_lookahead_point(ballPos, self.lookahead_distance)
        self.lookahead_point = lookahead_point

        dx = lookahead_point[0] - ballPos[0]
        dy = lookahead_point[1] - ballPos[1]
        norm = np.linalg.norm([dx, dy])
        if norm > 0:
            self.controller.feedforward_vector = (dx / norm, dy / norm)
        else:
            self.controller.feedforward_vector = (0, 0)

        self.controller.posControl(lookahead_point)

    def get_lookahead_point(self, ball_pos, lookahead_dist=100):
        min_dist = float('inf')
        closest_index = self.last_closest_index
        closest_proj = None
        ball_pos_np = np.array(ball_pos)

        start_index = max(0, self.last_closest_index - self.max_skip_behind)
        end_index = min(self.length - 1, self.last_closest_index + self.max_skip_ahead)
        search_range = range(start_index, end_index)

        v_ball = None
        if self.prev_ball_pos is not None:
            v_ball = ball_pos_np - np.array(self.prev_ball_pos)
            if np.linalg.norm(v_ball) > 1e-6:
                v_ball = v_ball / np.linalg.norm(v_ball)

        min_score = float("inf")
        for i in search_range:
            a = self.path_np[i]
            b = self.path_np[i + 1]
            proj = self._project_point_onto_segment(ball_pos_np, a, b)
            dist = np.linalg.norm(ball_pos_np - proj)

            angle_penalty = 1.0
            if v_ball is not None:
                v_path = b - a
                if np.linalg.norm(v_path) > 1e-6:
                    v_path = v_path / np.linalg.norm(v_path)
                    alignment = np.dot(v_ball, v_path)  # +1 is same dir, -1 is opposite
                    angle_penalty = 1.5 - alignment  # So backward = 2.5, forward = 0.5

            score = dist * angle_penalty

            if score < min_score:
                min_score = score
                closest_index = i
                closest_proj = proj


        self.last_closest_index = closest_index

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
                return tuple(lookahead_point), j

            total_dist += seg_len
            a = b

        # now = time.time()
        # if self.looping and (self.last_reverse_time is None or now - self.last_reverse_time > self.reverse_cooldown):
        #     self.forward = not self.forward
        #     self.last_reverse_time = now
        #     self.last_closest_index = self.length - 2 if self.forward else 1
        #     return self.path[self.last_closest_index]
        # else:
        #     return tuple(closest_proj), closest_index

        now = time.time()
        can_reverse = self.last_reverse_time is None or (now - self.last_reverse_time > self.reverse_cooldown)

        if self.looping and can_reverse:
            self.forward = not self.forward
            self.last_reverse_time = now
            self.last_closest_index = self.length - 2 if not self.forward else 1
            return self.path[self.last_closest_index], self.last_closest_index

        return tuple(closest_proj), closest_index

    def _project_point_onto_segment(self, p, a, b):
        ap = p - a
        ab = b - a

        if np.allclose(ab, 0):
            return a

        t = np.dot(ap, ab) / np.dot(ab, ab)
        t = np.clip(t, 0, 1)
        return a + t * ab