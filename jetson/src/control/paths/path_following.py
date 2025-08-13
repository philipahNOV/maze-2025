import control.position_controller as position_controller
import numpy as np
import time

class PathFollower:
    def __init__(self, path_array, controller: position_controller.Controller, config):
        self.path = []
        self.path_done = False
        self.length = 0
        self.config = config
        self.camera_offset_x = 0
        self.camera_offset_y = 0
        self.controller = controller
        self.prev_waypoint = 0
        self.next_waypoint = 0
        self.looping = False
        self.time_started = time.time()
        self.allow_waypoint_advancement_after = 1

        self.prev_time = None
        self.prev_ball_pos = None
        self.last_checkpoint_change_time = time.time()
        self.checkpoint_timeout = self.config["path_following"]["normal"].get("checkpoint_timeout", 10)
        self.prev_checkpoint = self.next_waypoint

        self.vel_threshold = self.config["path_following"]["normal"].get("velocity_threshold", 50)

        self.prev_progress_time = None
        self.stuck_retry_time = self.config["path_following"]["normal"].get("stuck_retry_time", 3)
        self.forward = True

        self.inside_target_radius = False
        self.time_entered_radius = None
        self.radius_dwell_time = self.config["path_following"]["normal"].get("dwell_time", 0.7)

        if path_array:
            self.path = []
            self.length = len(path_array)
            for n in range(self.length):
                self.path.append((path_array[n][0], path_array[n][1]))


        self.acceptance_radius = self.controller.pos_tol
        self.acceptance_radius_out = self.controller.pos_tol + 10

    def _advance_waypoint(self):

        if time.time() - self.time_started < self.allow_waypoint_advancement_after:
            # Don't advance if minimum time hasn't passed
            return

        if self.forward:
            if self.next_waypoint >= self.length - 1:
                if self.looping:
                    self.forward = False
                    self.next_waypoint = self.length - 2
                    self.prev_waypoint = self.length - 1
                else:
                    self.path_done = True
            else:
                self.prev_waypoint = self.next_waypoint
                self.next_waypoint += 1
        else:
            if self.next_waypoint <= 0:
                if self.looping:
                    self.forward = True
                    self.next_waypoint = 1
                    self.prev_waypoint = 0
                else:
                    self.path_done = True
            else:
                self.prev_waypoint = self.next_waypoint
                self.next_waypoint -= 1

        self.inside_target_radius = False
        self.time_entered_radius = None

    def follow_path(self, ballPos):
        vel = None
        self.looping = self.controller.looping

        if self.next_waypoint != self.prev_checkpoint:
            self.last_checkpoint_change_time = time.time()
            self.prev_checkpoint = self.next_waypoint

        if self.prev_time and self.prev_ball_pos:
            dt = time.time() - self.prev_time
            if dt > 0.000001:
                vel = np.linalg.norm(np.array(ballPos) - np.array(self.prev_ball_pos)) / dt
        self.prev_time = time.time()
        self.prev_ball_pos = ballPos
            

        if not self.prev_progress_time and vel:
            if vel < self.vel_threshold:
                self.prev_progress_time = time.time()
        elif self.prev_progress_time:
            if vel and vel > self.vel_threshold:
                self.prev_progress_time = None
            elif time.time() - self.prev_progress_time > self.stuck_retry_time:
                stuck_within_waypoint = np.linalg.norm(np.array(ballPos) - np.array(self.path[self.next_waypoint])) < self.acceptance_radius

                if self.forward:
                    self.next_waypoint = max(self.next_waypoint - 1, 0)
                    self.prev_waypoint = max(self.next_waypoint - 1, 0)
                else:
                    self.next_waypoint = min(self.next_waypoint + 1, self.length - 1)
                    self.prev_waypoint = min(self.next_waypoint + 1, self.length - 1)

                self.prev_progress_time = None

        if time.time() - self.last_checkpoint_change_time > self.checkpoint_timeout:
            self.last_checkpoint_change_time = time.time()

            if self.forward:
                self.next_waypoint = max(self.next_waypoint - 1, 0)
                self.prev_waypoint = max(self.next_waypoint - 1, 0)
            else:
                self.next_waypoint = min(self.next_waypoint + 1, self.length - 1)
                self.prev_waypoint = min(self.next_waypoint + 1, self.length - 1)

        
        self.controller.posControl(self.path[self.next_waypoint])

        
        current_time = time.time()

        dist_to_next = np.linalg.norm(np.array(ballPos) - np.array(self.path[self.next_waypoint]))
        # dist_to_prev = (
        #     np.linalg.norm(np.array(ballPos) - np.array(self.path[self.prev_waypoint]))
        #     if self.prev_waypoint is not None else float('inf')
        # )

        for i in reversed(range(self.length)):
            wpt = self.path[i]
            dist = np.linalg.norm(np.array(ballPos) - np.array(wpt))
            
            if dist < self.acceptance_radius and dist_to_next > self.acceptance_radius_out:
                self.prev_waypoint = i
                if self.forward:
                    self.next_waypoint = min(i + 1, self.length - 1)
                else:
                    self.next_waypoint = max(i - 1, 0)
                self.inside_target_radius = False
                self.time_entered_radius = None
                break

        if dist_to_next < self.acceptance_radius:
            if not self.inside_target_radius:
                self.inside_target_radius = True
                self.time_entered_radius = current_time
            elif self.time_entered_radius and (current_time - self.time_entered_radius > self.radius_dwell_time):
                self._advance_waypoint()
        else:
            if self.inside_target_radius:
                self._advance_waypoint()

        if dist_to_next >= self.acceptance_radius:
            self.inside_target_radius = False
            self.time_entered_radius = None

        dx = self.path[self.next_waypoint][0] - ballPos[0]
        dy = self.path[self.next_waypoint][1] - ballPos[1]

        norm = np.linalg.norm((dx, dy))
        if norm > 0:
            self.controller.feedforward_vector = (dx / norm, dy / norm)
        else:
            self.controller.feedforward_vector = (0, 0)
    
    def get_current_waypoint(self):
        return self.next_waypoint