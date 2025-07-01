import cv2
import numpy as np

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

    edges = cv2.Canny(enhanced, threshold1=50, threshold2=150)
    edges_dilated = cv2.dilate(edges, np.ones((3, 3), np.uint8), iterations=2)
    edges_inv = cv2.bitwise_not(edges_dilated)

    final_mask = cv2.bitwise_and(base_mask, edges_inv)

    return final_mask