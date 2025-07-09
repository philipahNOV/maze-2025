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

def create_binary_mask(gray):
    enhanced = apply_clahe(gray)
    _, base_mask = cv2.threshold(enhanced, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    cleaned = cv2.morphologyEx(base_mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8), iterations=2)

    edges = cv2.Canny(enhanced, 100, 200)
    edges_dilated = cv2.dilate(edges, np.ones((3, 3), np.uint8), iterations=1)
    edges_inv = cv2.bitwise_not(edges_dilated)

    final_mask = cv2.bitwise_and(cleaned, edges_inv)
    final_mask = cv2.morphologyEx(final_mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8), iterations=1)

    # needs to filter green hsv values as white
    hsv = cv2.cvtColor(cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR), cv2.COLOR_BGR2HSV)
    lower_green = np.array([35, 20, 20])
    upper_green = np.array([85, 255, 255])
    green_mask = cv2.inRange(hsv, lower_green, upper_green)
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8), iterations=2)
    green_dilated = cv2.dilate(green_mask, np.ones((9, 9), np.uint8), iterations=2)

    flood_fill_mask = green_dilated.copy()
    contours, _ = cv2.findContours(green_dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        M = cv2.moments(cnt)
        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            cv2.floodFill(flood_fill_mask, None, (cx, cy), 255)

    safe_mask = cv2.bitwise_or(safe_mask, flood_fill_mask)

    return final_mask