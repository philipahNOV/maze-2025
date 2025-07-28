#!/usr/bin/env python3
"""
LAB Color Space Calibration Tool for Gray Ball Detection

This tool helps you find the optimal LAB color range for your gray ball.
LAB color space is more perceptually uniform than HSV and works better
with different lighting conditions.
"""

import cv2
import numpy as np
import sys
import os

def nothing(x):
    pass

def lab_color_picker():
    """Interactive tool to pick LAB color ranges"""
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Error: Cannot open camera")
        return
    
    # Create trackbars for LAB values
    cv2.namedWindow('LAB Controls')
    cv2.createTrackbar('L Min', 'LAB Controls', 0, 255, nothing)
    cv2.createTrackbar('L Max', 'LAB Controls', 255, 255, nothing)
    cv2.createTrackbar('A Min', 'LAB Controls', 120, 255, nothing)
    cv2.createTrackbar('A Max', 'LAB Controls', 135, 255, nothing)
    cv2.createTrackbar('B Min', 'LAB Controls', 120, 255, nothing)
    cv2.createTrackbar('B Max', 'LAB Controls', 135, 255, nothing)
    
    print("LAB Color Picker")
    print("================")
    print("Adjust the sliders to isolate your gray ball")
    print("L = Lightness (0-255)")
    print("A = Green-Red axis (0-255, 128 is neutral)")
    print("B = Blue-Yellow axis (0-255, 128 is neutral)")
    print("Press 'q' to quit, 's' to save current values")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Convert to LAB
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        
        # Get trackbar values
        l_min = cv2.getTrackbarPos('L Min', 'LAB Controls')
        l_max = cv2.getTrackbarPos('L Max', 'LAB Controls')
        a_min = cv2.getTrackbarPos('A Min', 'LAB Controls')
        a_max = cv2.getTrackbarPos('A Max', 'LAB Controls')
        b_min = cv2.getTrackbarPos('B Min', 'LAB Controls')
        b_max = cv2.getTrackbarPos('B Max', 'LAB Controls')
        
        # Create mask
        lower_lab = np.array([l_min, a_min, b_min])
        upper_lab = np.array([l_max, a_max, b_max])
        mask = cv2.inRange(lab, lower_lab, upper_lab)
        
        # Apply mask to original image
        result = cv2.bitwise_and(frame, frame, mask=mask)
        
        # Stack images for display
        mask_3ch = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        combined = np.hstack([frame, mask_3ch, result])
        combined = cv2.resize(combined, (1200, 400))  # Resize for display
        
        # Add text
        cv2.putText(combined, 'Original', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(combined, 'Mask', (410, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(combined, 'Result', (810, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        # Show current LAB values
        lab_text = f"LAB: L({l_min}-{l_max}) A({a_min}-{a_max}) B({b_min}-{b_max})"
        cv2.putText(combined, lab_text, (10, 380), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        cv2.imshow('LAB Color Picker', combined)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            print(f"\nCurrent LAB values:")
            print(f"lower: [{l_min}, {a_min}, {b_min}]")
            print(f"upper: [{l_max}, {a_max}, {b_max}]")
            print(f"\nAdd this to your config.yaml:")
            print(f"lab_range:")
            print(f"  lower: [{l_min}, {a_min}, {b_min}]")
            print(f"  upper: [{l_max}, {a_max}, {b_max}]")
    
    cap.release()
    cv2.destroyAllWindows()

def compare_hsv_vs_lab():
    """Compare HSV and LAB color spaces side by side"""
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Error: Cannot open camera")
        return
    
    # Default values for gray ball
    hsv_lower = np.array([0, 0, 50])    # Gray in HSV
    hsv_upper = np.array([180, 50, 200])
    
    lab_lower = np.array([0, 120, 120])  # Gray in LAB
    lab_upper = np.array([200, 135, 135])
    
    print("HSV vs LAB Comparison")
    print("Press 'q' to quit")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # HSV processing
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hsv_mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
        hsv_result = cv2.bitwise_and(frame, frame, mask=hsv_mask)
        
        # LAB processing
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        lab_mask = cv2.inRange(lab, lab_lower, lab_upper)
        lab_result = cv2.bitwise_and(frame, frame, mask=lab_mask)
        
        # Combine for display
        hsv_combined = np.hstack([frame, cv2.cvtColor(hsv_mask, cv2.COLOR_GRAY2BGR), hsv_result])
        lab_combined = np.hstack([frame, cv2.cvtColor(lab_mask, cv2.COLOR_GRAY2BGR), lab_result])
        
        # Resize and stack vertically
        hsv_combined = cv2.resize(hsv_combined, (900, 300))
        lab_combined = cv2.resize(lab_combined, (900, 300))
        
        cv2.putText(hsv_combined, 'HSV Detection', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(lab_combined, 'LAB Detection', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        
        final = np.vstack([hsv_combined, lab_combined])
        cv2.imshow('HSV vs LAB Comparison', final)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    print("LAB Color Space Calibration Tool")
    print("================================")
    print("1. Interactive LAB color picker")
    print("2. HSV vs LAB comparison")
    print("3. Exit")
    
    while True:
        choice = input("\nSelect option (1-3): ").strip()
        
        if choice == '1':
            lab_color_picker()
        elif choice == '2':
            compare_hsv_vs_lab()
        elif choice == '3':
            break
        else:
            print("Invalid choice")
    
    print("Calibration complete!")
