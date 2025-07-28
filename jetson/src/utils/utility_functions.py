import numpy as np

def densify_path(self, path, factor=6):
        new_path = []
        for i in range(len(path) - 1):
            p1 = np.array(path[i])
            p2 = np.array(path[i + 1])
            new_path.append(tuple(p1))
            for j in range(1, factor):
                interp = p1 + (p2 - p1) * (j / factor)
                new_path.append(tuple(interp.astype(int)))
        new_path.append(path[-1])
        return new_path

def is_in_elevator(config, ball_pos):
        center_x = config['camera'].get('elevator_center_x', 1030)
        center_y = config['camera'].get('elevator_center_y', 630)
        radius = config['camera'].get('elevator_radius', 60)
        if ball_pos is None:
            return False
        x, y = ball_pos
        dx = x - center_x
        dy = y - center_y
        distance_squared = dx * dx + dy * dy
        return distance_squared <= radius * radius

def remove_withing_elevator(config, path, radius: int = 80):
        center_x = config['camera'].get('elevator_center_x', 1030)
        center_y = config['camera'].get('elevator_center_y', 630)
        within_indexes = []
        new_path = path.copy()
        if path is None:
            return
        for i in range(len(new_path)):
            x, y = new_path[i]
            dx = x - center_x
            dy = y - center_y
            distance_squared = dx * dx + dy * dy
            if distance_squared <= radius * radius:
                within_indexes.append(i)
        for i in reversed(within_indexes):
            new_path.pop(i)
        return new_path