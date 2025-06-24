import positionController
import numpy as np

class PathFollower:
    def __init__(self, path_array, controller: positionController.Controller):

        self.path = None
        self.length = 0

        self.camera_offset_x = 420
        self.camera_offset_y = 10

        self.controller = controller

        self.prev_waypoint = None
        self.next_waypoint = 0

        self.looping = False

        if path_array:
            self.path = []
            self.length = len(path_array)
            for n in range(self.length):
                self.path.append((path_array[n][0] + self.camera_offset_x, path_array[n][1] + self.camera_offset_y))

        self.acceptance_radius = self.controller.pos_tol

    def follow_path(self, ballPos):
        
        self.controller.posControl(self.path[self.next_waypoint])

        if np.linalg.norm(np.array(ballPos) - np.array(self.path[self.next_waypoint])) < self.acceptance_radius:
            self.prev_waypoint = self.next_waypoint

            if self.next_waypoint == self.length - 1:
                if self.looping:
                    self.prev_waypoint = None
                    self.next_waypoint = 0
                    print("Loop completed, starting from first waypoint")
                    return
                print("Done following path")
                return
            
            self.next_waypoint = self.next_waypoint + 1

    
