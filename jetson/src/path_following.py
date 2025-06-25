import positionController
import numpy as np
import time

class PathFollower:
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
                # self.path.append((path_array[n][0] + self.camera_offset_x, path_array[n][1] + self.camera_offset_y))
                self.path.append((path_array[n][0], path_array[n][1]))


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

    
