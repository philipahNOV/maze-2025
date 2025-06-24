
class Path:
    def __init__(self, path_array):

        self.path = None
        self.length = 0

        if path_array:
            self.path = path_array
            self.length = len(path_array)

        self.acceptance_radius = 20