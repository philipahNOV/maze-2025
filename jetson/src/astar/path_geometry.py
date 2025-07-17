import numpy as np
import cv2

def angle_between(p1, p2, p3):
    a = np.array(p1)
    b = np.array(p2)
    c = np.array(p3)
    ba = a - b
    bc = c - b
    cos_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
    return np.degrees(np.arccos(np.clip(cos_angle, -1.0, 1.0)))

def is_clear_path(mask, p1, p2, kernel_size=6):
    line_img = np.zeros_like(mask, dtype=np.uint8)
    cv2.line(line_img, (p1[1], p1[0]), (p2[1], p2[0]), 1, 1)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_size, kernel_size))
    corridor = cv2.dilate(line_img, kernel)
    return np.all(mask[corridor.astype(bool)] > 0)

def clear_path(p0, p1, mask):
    y0, x0 = p0; y1, x1 = p1
    dy, dx = abs(y1 - y0), abs(x1 - x0)
    sy, sx = (1 if y0 < y1 else -1), (1 if x0 < x1 else -1)
    err = dx - dy; y, x = y0, x0
    while True:
        if not mask[y][x]:
            return False
        if (y, x) == (y1, x1):
            return True
        e2 = err*2
        if e2 > -dy:
            err -= dy; x += sx
        if e2 <  dx:
            err += dx; y += sy