
class Path:
    def __init__(self, path_array):

        self.path = None
        self.length = 0

        self.camera_offset_x = 0
        self.camera_offset_y = 0

        if path_array:
            self.path = []
            self.length = len(path_array)
            for n in range(self.length):
                self.path.append((path_array[n][0] + self.camera_offset_x, path_array[n][1] + self.camera_offset_y))

        self.acceptance_radius = 20