import cv2
import os
import numpy as np
import time

def red_threshold(frame, frac):
    """
    Apply color thresholding to the frame.
    :param frame: Input image frame.
    :param frac: Fraction of the maximum red value to use as the upper bound.
    :return: Masked image after applying color thresholding.
    """
    frame_float = frame.astype(np.float32)
    B, G, R = cv2.split(frame_float)
    total = R + G + B + 1e-6
    red_ratio = R / total
    max_red_ratio = np.max(red_ratio)
    mask = red_ratio > (frac * max_red_ratio)


    # Apply mask to original frame
    result = np.zeros_like(frame)
    result[mask] = frame[mask]
    return result

vid = cv2.VideoCapture('output.mp4')
if not vid.isOpened():
    print("Error: Cannot open video file.")
    exit()

frame_num = 0
while True:
    ret, frame = vid.read()
    if not ret:
        print("End of video or error reading frame.")
        break
    frame = red_threshold(frame, 0.5)

    cv2.imshow('Video Frame', frame)
    time.sleep(1 / 15)
    frame_num += 1
    print(f"Displaying frame {frame_num}")
    if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to exit
        break