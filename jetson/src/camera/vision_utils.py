import cv2

def get_center_of_mass(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        return None
    largest = max(contours, key=cv2.contourArea)
    M = cv2.moments(largest)

    if M["m00"] == 0:
        return None
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    return (cx, cy)

def global_hsv_search(frame, hsv_lower, hsv_upper):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
    return get_center_of_mass(mask)

def hsv_tracking(frame, prev_pos, hsv_lower, hsv_upper, window_size=80):
    if prev_pos is None:
        return None
    
    h, w = frame.shape[:2]
    x, y = prev_pos
    x_min = max(0, x - window_size)
    x_max = min(w, x + window_size)
    y_min = max(0, y - window_size)
    y_max = min(h, y + window_size)
    roi = frame[y_min:y_max, x_min:x_max]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
    local_center = get_center_of_mass(mask)
    if local_center:
        cx, cy = local_center
        return (x_min + cx, y_min + cy)
    return None