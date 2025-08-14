import numpy as np
import cv2
from control.astar.board_masking import get_dynamic_threshold, create_binary_mask, dilate_mask
import os

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

def determine_maze(tracking_service, center=(766, 341), box_size=(300, 300), threshold_ratio=0.15):
    frame = tracking_service.get_stable_frame()
    if frame is None:
        return None

    gray = get_dynamic_threshold(frame)
    binary_mask = create_binary_mask(gray)
    safe_mask = dilate_mask(binary_mask)

    h, w = safe_mask.shape[:2]
    x, y = center
    box_w, box_h = box_size

    # Define bounding box coordinates and clip to image boundaries
    x1 = max(0, x - box_w // 2)
    y1 = max(0, y - box_h // 2)
    x2 = min(w, x + box_w // 2)
    y2 = min(h, y + box_h // 2)

    # Extract the region of interest
    roi = safe_mask[y1:y2, x1:x2]

    # Normalize black pixel count within ROI
    total_pixels = roi.size
    black_pixels = np.sum(roi == 0)
    black_ratio = black_pixels / total_pixels
    if black_ratio >= threshold_ratio:
        return "Hard"
    else:
        return "Easy"

        
def is_within_goal(maze, position, custom_goal=None):
    if custom_goal is not None:
        return np.linalg.norm(np.array(position) - np.array(custom_goal)) < 50

    if maze == "Hard":
        corners = [(720, 32), (890, 32), (890, 110), (720, 110)]
    else:
        corners = [(760, 32), (904, 32), (904, 72), (760, 72)]
    contour = np.array(corners, dtype=np.int32)
    result = cv2.pointPolygonTest(contour, position, measureDist=False)
    return result >= 0

def load_image(config, frame=None, path=None):
    padding = config['camera'].get('padding', 10)
    top_left = config['camera'].get('top_left', (430, 27))
    top_right = config['camera'].get('top_right', (1085, 27))
    bottom_left = config['camera'].get('bottom_left', (430, 682))
    bottom_right = config['camera'].get('bottom_right', (1085, 682))
    top_left = (top_left[0] - padding, top_left[1] - padding)
    top_right = (top_right[0] + padding, top_right[1] - padding)
    bottom_left = (bottom_left[0] - padding, bottom_left[1] + padding)
    bottom_right = (bottom_right[0] + padding, bottom_right[1] + padding)
    frame_corners = (top_left, top_right, bottom_left, bottom_right)

    if frame is not None:
        img = frame.copy()
    elif path is not None:
        img = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if img is None:
            raise FileNotFoundError(f"Could not read image: {path}")
    else:
        raise ValueError("Either 'frame' or 'path' must be provided to load_image().")

    x1, y1 = frame_corners[0]
    x2, y2 = frame_corners[3]
    cropped = img[y1:y2, x1:x2]
    return cropped

def normalize_view(img, roi=None, resize_to=(512, 512), clahe=True):
    """
    - Optional ROI crop
    - Resize to a consistent size
    - Optional CLAHE on L channel to reduce lighting variability
    """
    if roi is not None:
        x, y, w, h = roi
        img = img[y:y+h, x:x+w]

    img = cv2.resize(img, resize_to, interpolation=cv2.INTER_AREA)

    if clahe:
        lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        L, A, B = cv2.split(lab)
        c = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        L = c.apply(L)
        lab = cv2.merge([L, A, B])
        img = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

    return img

def absdiff_distance(img_a, img_b, use_gray=True, blur_ksize=3):
    """
    Compute a scalar difference using pixel-wise absolute difference.
    Lower = more similar.
    - Converts to grayscale (optional)
    - Light Gaussian blur to reduce noise
    - Returns the mean absolute difference (float)
    """
    if use_gray:
        img_a = cv2.cvtColor(img_a, cv2.COLOR_BGR2GRAY)
        img_b = cv2.cvtColor(img_b, cv2.COLOR_BGR2GRAY)

    if blur_ksize and blur_ksize > 1:
        img_a = cv2.GaussianBlur(img_a, (blur_ksize, blur_ksize), 0)
        img_b = cv2.GaussianBlur(img_b, (blur_ksize, blur_ksize), 0)

    diff = cv2.absdiff(img_a, img_b)
    return float(np.mean(diff))

def identify_maze(frame, config):
    """
    Returns 'Hard' or 'Easy'.
    Uses normalized view + absdiff to pick the closest reference board.
    """
    ref_a_path = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "MAZE_PICS", "Hard.jpg")
    )
    ref_b_path = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "MAZE_PICS", "Easy.jpg")
    )

    img_cap = normalize_view(load_image(config, frame=frame))
    img_a   = normalize_view(load_image(config, path=ref_a_path))
    img_b   = normalize_view(load_image(config, path=ref_b_path))

    d_a = absdiff_distance(img_cap, img_a)
    d_b = absdiff_distance(img_cap, img_b)

    choice = 'Hard' if d_a <= d_b else 'Easy'
    return choice