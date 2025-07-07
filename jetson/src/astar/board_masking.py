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

    return final_mask