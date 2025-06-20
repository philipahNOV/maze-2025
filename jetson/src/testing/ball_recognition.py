import cv2
import os
import numpy as np
import time

def red_threshold(frame, frac, min_intensity, prev_center=None, roi_radius=100):
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

    # Find all pixels that satisfy both red ratio and intensity threshold
    
    spatial_mask = np.ones_like(total, dtype=bool)
    if prev_center is not None:
        h, w = total.shape
        Y, X = np.ogrid[:h, :w]
        dist_sq = (X - prev_center[0]) ** 2 + (Y - prev_center[1]) ** 2
        spatial_mask = dist_sq <= roi_radius ** 2

    #valid_pixels = np.where((red_ratio > max_red_ratio) & (total > min_intensity) & spatial_mask)
    valid_pixels = np.where(spatial_mask)
    if valid_pixels[0].size > 0 and prev_center is not None:
        # Compute max red ratio among valid pixels
        max_red_ratio = np.max(red_ratio[valid_pixels])
    else:
        max_red_ratio = np.max(red_ratio)

    max_red_ratio = max(max_red_ratio, 0.3)
    bright_mask = total > min_intensity
    red_mask = red_ratio > (frac * max_red_ratio)
    mask = (np.logical_and(bright_mask, red_mask)).astype(np.uint8) * 255

    #hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Rough skin color range (tweak as needed)
    #lower_skin = np.array([0, 20, 70])
    #upper_skin = np.array([20, 255, 255])
    #skin_mask = cv2.inRange(hsv, lower_skin, upper_skin)

    #mask = cv2.bitwise_and(mask, cv2.bitwise_not(skin_mask))

    # Morphological cleaning
    #kernel = np.ones((3, 3), np.uint8)
    #mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    #mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Apply mask to original frame
    result = np.zeros_like(frame)
    result[mask == 255] = 255
    return result, (mask > 0).astype(np.uint8) * 255

def find_most_circular_contour(mask, min_area=100, max_area = 5000, min_circularity=0.6, prev_center= None, max_diff=500):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        #print("No contours found.")
        return None
    
    best_contour = None
    best_circularity = 0
    
    for cnt in contours:
        area = cv2.contourArea(cnt)

        if not (min_area <= area <= max_area):
            continue
        
        perimeter = cv2.arcLength(cnt, True)
        if perimeter == 0:
            continue
        
        circularity = 4 * np.pi * (area / (perimeter * perimeter))

        (x, y), radius = cv2.minEnclosingCircle(cnt)
        center = (int(x), int(y))
        diff = center_difference(prev_center, center)
        if diff is not None and center_difference(prev_center, center) > max_diff:
            print(f"Large center difference detected: {center_difference(prev_center, center)} pixels")
            continue
        
        if circularity > best_circularity and circularity >= min_circularity:
            best_circularity = circularity
            best_contour = cnt
    
    if best_contour is None:
        pass
        #print("No contour passed the circularity filter.")
    
    return best_contour

def center_difference(prev_center, current_center):
    if prev_center is None or current_center is None:
        return None
    return np.linalg.norm(np.array(current_center) - np.array(prev_center))

def detect_red_ball(video_name):
    vid = cv2.VideoCapture(video_name)
    if not vid.isOpened():
        print("Error; Cannot open video file.")
        exit()

    frame_num = 0
    center = None
    prev_center = None
    while True:
        ret, frame = vid.read()
        if not ret:
            print("End of video or error reading frame.")
            break
        masked_frame, mask = red_threshold(frame, 0.6, 40, prev_center, 500)

        #best_contour = find_most_circular_contour(mask, min_area=1, max_area=800, min_circularity=0.4, prev_center=prev_center, max_diff=200)
        M = cv2.moments(mask)

        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            center = (cX, cY)
        else:
            center = None  # No object detected
        if center is not None:
            radius = 5
            cv2.circle(frame, center, radius, (0, 255, 0), 4)
            prev_center = center

        best_contour = None
        if best_contour is not None:
            #cv2.drawContours(masked_frame, [best_contour], -1, (0, 255, 0), thickness=cv2.FILLED)
            (x, y), radius = cv2.minEnclosingCircle(best_contour)
            center = (int(x), int(y))
            diff = center_difference(prev_center, center)
            if diff is not None and center_difference(prev_center, center) > 10000:
                print(f"Large center difference detected; {center_difference(prev_center, center)} pixels")
                center = prev_center
            prev_center = center
            radius = int(radius)
            cv2.circle(frame, center, radius, (0, 255, 0), 4)

        cv2.imshow('Video frame', frame)
        time.sleep(1 / 15)
        frame_num += 1
        print(f"Displaying frame {frame_num}")
        if cv2.waitKey(1) & 0xFF == ord('q'):  # Pes 'q' to exit
            break
    vid.release()
    cv2.destroyAllWindows()

def detect_red_ball_frame(frame, prev_center=None):
    masked_frame, mask = red_threshold(frame, 0.6, 50, prev_center, 500)
    if cv2.countNonZero(mask) == 0:
        return None, 0, masked_frame
    #cv2.circle(frame, brightest_pixel, 10, (0, 0, 255), 4)

    #best_contour = find_most_circular_contour(mask, min_area=1, max_area=800, min_circularity=0.5, prev_center=prev_center, max_diff=10000)
    #gray_mask = cv2.cvtColor(masked_frame, cv2.COLOR_BGR2GRAY)
    M = cv2.moments(mask)
    center = None
    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        center = (cX, cY)
    else:
        center = None  # No object detected
    if center is not None:
        prev_center = center
        print("TEST")
        #center = (center[1], center[0])

    #best_contour = None
    #center = prev_center
    radius = 0
    #if best_contour is not None:
        #cv2.drawContours(masked_frame, [best_contour], -1, (0, 255, 0), thickness=cv2.FILLED)
        #he
    #    (x, y), radius = cv2.minEnclosingCircle(best_contour)
    #    center = (int(x), int(y))
    #    diff = center_difference(prev_center, center)
    #    if diff is not None and center_difference(prev_center, center) > 10000:
    #        #print(f"Large center difference detected: {center_difference(prev_center, center)} pixels")
    #        center = prev_center
    #    radius = int(radius)
    return center, radius, masked_frame
    
def main():
    vid = cv2.VideoCapture("output.mp4")
    if not vid.isOpened():
        print("Error: Cannot open video file.")
        exit()
    frame_num = 0

    center = None
    while True:
        ret, frame = vid.read()
        if not ret:
            print("End of video or error reading frame.")
            break
        center, radius, masked_frame = detect_red_ball_frame(frame, center)
        cv2.circle(frame, center, 5, (0, 255, 0), 4)
        cv2.imshow('Video Frame', frame)
        #time.sleep(1 / 5)
        frame_num += 1
        #print(f"Displaying frame {frame_num}")
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    vid.release()
