import positionController_2
import numpy as np
import time

class PathFollower:

    """
    PathFollower handles navigation along a predefined path of 2D waypoints using a position controller.

    Attributes:
        path (list of tuple): A list of (x, y) waypoints defining the navigation path.
        controller (positionController.Controller): The external controller used for position control.
        prev_waypoint (int): Index of the last successfully reached waypoint.
        next_waypoint (int): Index of the current target waypoint.
        looping (bool): If True, the path loops back to the start upon completion.
        prev_time (float): Timestamp of the previous position update (used to compute velocity).
        prev_ball_pos (tuple): Last known position of the ball.
        vel_threshold (float): Minimum velocity considered as progress; below this may trigger "stuck" detection.
        prev_progress_time (float): Timestamp of when the ball last showed insufficient progress.
        stuck_retry_time (float): Time (in seconds) before triggering a step back to a previous waypoint if stuck.
        acceptance_radius (float): Distance threshold to consider a waypoint as "reached".

    Methods:
        follow_path(ballPos):
            Drives the controller toward the next waypoint. Handles path following, progress tracking, and 
            automatic recovery if the object is stuck or goes behind a previously visited waypoint.
    """

    def __init__(self, path_array, controller: positionController_2.Controller):

        self.path = []
        self.length = 0

        # self.camera_offset_x = 420
        # self.camera_offset_y = 10

        self.camera_offset_x = 0
        self.camera_offset_y = 0

        self.controller = controller

        self.prev_waypoint = 0
        self.next_waypoint = 0

        self.looping = False

        self.prev_time = None
        self.prev_ball_pos = None

        self.vel_threshold = 50

        self.prev_progress_time = None
        self.stuck_retry_time = 3

        self.inside_target_radius = False
        self.time_entered_radius = None
        self.radius_dwell_time = 0.8  # seconds before allowing progress while stationary

        if path_array:
            self.path = []
            self.length = len(path_array)
            for n in range(self.length):
                self.path.append((path_array[n][0], path_array[n][1]))


        self.acceptance_radius = self.controller.pos_tol
        self.acceptance_radius_out = self.controller.pos_tol + 10

    def _advance_waypoint(self):
        self.prev_waypoint = self.next_waypoint
        if self.next_waypoint == self.length - 1:
            if self.looping:
                print("Loop completed, restarting.")
                self.prev_waypoint = None
                self.next_waypoint = 0
            else:
                print("Done following path")
        else:
            self.next_waypoint = min(self.next_waypoint + 1, self.length - 1)
        self.inside_target_radius = False
        self.time_entered_radius = None

    def follow_path(self, ballPos):
        vel = None

        if self.prev_time and self.prev_ball_pos:
            dt = time.time() - self.prev_time
            if dt > 0.000001:
                vel = np.linalg.norm(np.array(ballPos) - np.array(self.prev_ball_pos)) / dt
        self.prev_time = time.time()
        self.prev_ball_pos = ballPos
            

        if not self.prev_progress_time and vel:
            if vel < self.vel_threshold:
                self.prev_progress_time = time.time()
                print(f"No progression, starting timer. Velocity: {vel}")
        elif self.prev_progress_time:
            if vel and vel > self.vel_threshold:
                self.prev_progress_time = None
                print(f"Progressing again. Velocity: {vel}")
            elif time.time() - self.prev_progress_time > self.stuck_retry_time:
                # Check if we're stuck within the current waypoint's acceptance radius
                stuck_within_waypoint = np.linalg.norm(np.array(ballPos) - np.array(self.path[self.next_waypoint])) < self.acceptance_radius

                # Normal backstep
                self.next_waypoint = max(self.next_waypoint - 1, 0)
                self.prev_waypoint = max(self.next_waypoint - 1, 0)
                print(f"Stuck — reverting to waypoint {self.next_waypoint}")

                self.prev_progress_time = None

        
        self.controller.posControl(self.path[self.next_waypoint])

        
        current_time = time.time()

        # Distance to relevant waypoints
        dist_to_next = np.linalg.norm(np.array(ballPos) - np.array(self.path[self.next_waypoint]))
        dist_to_prev = (
            np.linalg.norm(np.array(ballPos) - np.array(self.path[self.prev_waypoint]))
            if self.prev_waypoint is not None else float('inf')
        )

        # === Backward skipping (keep this first!) ===
        for i in reversed(range(self.length)):
            wpt = self.path[i]
            dist = np.linalg.norm(np.array(ballPos) - np.array(wpt))
            
            if dist < self.acceptance_radius and dist_to_next > self.acceptance_radius_out:
                print(f"[SKIP] Ball near waypoint {i} → skipping back")
                self.prev_waypoint = i
                self.next_waypoint = min(i + 1, self.length - 1)
                self.inside_target_radius = False
                self.time_entered_radius = None
                break

        # === Latch + dwell logic ===
        if dist_to_next < self.acceptance_radius:
            if not self.inside_target_radius:
                self.inside_target_radius = True
                self.time_entered_radius = current_time
                print(f"[LATCH] Ball entered waypoint {self.next_waypoint}")
            elif self.time_entered_radius and (current_time - self.time_entered_radius > self.radius_dwell_time):
                print(f"[DWELL ADVANCE] Stayed on waypoint {self.next_waypoint} → advancing")
                self._advance_waypoint()
        else:
            if self.inside_target_radius:
                print(f"[EXIT ADVANCE] Ball exited waypoint {self.next_waypoint} → advancing")
                self._advance_waypoint()

        # Reset latch if outside
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

    
