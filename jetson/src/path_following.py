import positionController
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

    def __init__(self, path_array, controller: positionController.Controller):

        self.path = None
        self.length = 0

        # self.camera_offset_x = 420
        # self.camera_offset_y = 10

        self.camera_offset_x = 0
        self.camera_offset_y = 0

        self.controller = controller

        self.prev_waypoint = None
        self.next_waypoint = 0

        self.looping = False

        self.prev_time = None
        self.prev_ball_pos = None

        self.vel_threshold = 10
        self.prev_progress_time = None
        self.stuck_retry_time = 3

        if path_array:
            self.path = []
            self.length = len(path_array)
            for n in range(self.length):
                self.path.append((path_array[n][0] + self.camera_offset_x, path_array[n][1] + self.camera_offset_y))
                #self.path.append((path_array[n][0], path_array[n][1]))


        self.acceptance_radius = self.controller.pos_tol

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
        elif vel:
            if vel > self.vel_threshold:
                self.prev_progress_time = None
            if time.time() - self.prev_progress_time > self.stuck_retry_time:
                print(f"Stuck detected. Reversing to waypoint {self.next_waypoint}")
                self.next_waypoint = max(self.next_waypoint - 1, 0)
                self.prev_waypoint = max(self.prev_waypoint - 1, 0)
                self.prev_progress_time = None

        
        self.controller.posControl(self.path[self.next_waypoint])

        if np.linalg.norm(np.array(ballPos) - np.array(self.path[self.next_waypoint])) < self.acceptance_radius:
            self.prev_waypoint = self.next_waypoint

            if self.next_waypoint == self.length-1:
                if self.looping:
                    self.prev_waypoint = None
                    self.next_waypoint = 0
                    print("Loop completed, starting from first waypoint")
                    return
                print("Done following path")
                return
            
            self.next_waypoint = self.next_waypoint + 1
        else:
            for i in reversed(range(len(self.path))):
                wpt = self.path[i]
                if np.linalg.norm(np.array(ballPos) - np.array(wpt)) < self.acceptance_radius:

                    self.prev_waypoint = i
                    self.next_waypoint = i+1
                    return

    
