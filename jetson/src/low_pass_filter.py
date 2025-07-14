class SmoothedTracker:
    def __init__(self, alpha=0.5):
        self.alpha = alpha
        self.smoothed_pos = None

    def update(self, new_pos):
        if new_pos is None:
            return self.smoothed_pos

        if self.smoothed_pos is None:
            self.smoothed_pos = new_pos
        else:
            x = self.alpha * new_pos[0] + (1 - self.alpha) * self.smoothed_pos[0]
            y = self.alpha * new_pos[1] + (1 - self.alpha) * self.smoothed_pos[1]
            self.smoothed_pos = (int(x), int(y))

        return self.smoothed_pos