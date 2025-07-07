import cv2
import numpy as np

def dilate_mask(mask, kernel_size=(3, 3), iterations=2):
    kernel = np.ones(kernel_size, np.uint8)
    return cv2.dilate(mask, kernel, iterations=iterations)

def get_dynamic_threshold(image, target_brightness=130):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    current_brightness = np.mean(gray)
    adjustment = target_brightness - current_brightness
    adjusted = np.clip(gray + adjustment, 0, 255).astype(np.uint8)
    return adjusted

def apply_clahe(gray):
    clahe = cv2.createCLAHE(clipLimit=1.0, tileGridSize=(8, 8))
    return clahe.apply(gray)

def create_binary_mask(gray, color_frame=None):
    enhanced = apply_clahe(gray)
    _, base_mask = cv2.threshold(enhanced, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    cleaned = cv2.morphologyEx(base_mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8), iterations=2)

    edges = cv2.Canny(enhanced, 100, 200)
    edges_dilated = cv2.dilate(edges, np.ones((3, 3), np.uint8), iterations=1)
    edges_inv = cv2.bitwise_not(edges_dilated)

    final_mask = cv2.bitwise_and(cleaned, edges_inv)
    final_mask = cv2.morphologyEx(final_mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8), iterations=1)

    if color_frame is not None:
        hsv = cv2.cvtColor(color_frame, cv2.COLOR_BGR2HSV)
        ball_lower = np.array([30, 50, 50])
        ball_upper = np.array([90, 255, 255])
        ball_mask = cv2.inRange(hsv, ball_lower, ball_upper)

        kernel = np.ones((9, 9), np.uint8)
        ball_mask = cv2.morphologyEx(ball_mask, cv2.MORPH_CLOSE, kernel, iterations=5)
        ball_mask = cv2.dilate(ball_mask, kernel, iterations=2)

        final_mask[ball_mask > 0] = 255

        launch_pad_center = (1030, 630)
        launch_pad_radius = 70
        cv2.circle(final_mask, launch_pad_center, launch_pad_radius, 255, -1)

    return final_mask