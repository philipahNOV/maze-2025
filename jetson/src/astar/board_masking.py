import cv2
import numpy as np

def dilate_mask(mask, kernel_size=(3, 3), iterations=2):
    kernel = np.ones(kernel_size, np.uint8)
    return cv2.dilate(mask, kernel, iterations=iterations)

def adaptive_green_mask(hsv, relaxed=True):
    if relaxed:
        lower = np.array([35, 30, 30])
        upper = np.array([85, 255, 255])
    else:
        lower = np.array([40, 80, 80])
        upper = np.array([80, 255, 255])

    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.medianBlur(mask, 5)
    mask = cv2.dilate(mask, np.ones((5, 5), np.uint8), iterations=1)
    return mask

def get_dynamic_threshold(image, target_brightness=130):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    current_brightness = np.mean(gray)
    adjustment = target_brightness - current_brightness
    adjusted = np.clip(gray + adjustment, 0, 255).astype(np.uint8)
    return adjusted

def apply_clahe(gray: np.ndarray) -> np.ndarray:
    clahe = cv2.createCLAHE(clipLimit=1.0, tileGridSize=(8, 8))
    return clahe.apply(gray)

def create_binary_mask(image: np.ndarray) -> np.ndarray:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    enhanced = apply_clahe(gray)
    _, base_mask = cv2.threshold(enhanced, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    cleaned = cv2.morphologyEx(base_mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8), iterations=2)

    edges = cv2.Canny(enhanced, 100, 200)
    edges_dilated = cv2.dilate(edges, np.ones((3, 3), np.uint8), iterations=1)
    edges_inv = cv2.bitwise_not(edges_dilated)
    final_mask = cv2.bitwise_and(cleaned, edges_inv)
    final_mask = cv2.morphologyEx(final_mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8), iterations=1)

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    green_mask = adaptive_green_mask(hsv, relaxed=True)
    _, green_mask = cv2.threshold(green_mask, 30, 255, cv2.THRESH_BINARY)

    final_mask = cv2.bitwise_or(final_mask, green_mask)
    return final_mask