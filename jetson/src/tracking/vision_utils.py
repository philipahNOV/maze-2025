import cv2
import numpy as np

def extract_ball_template(frame, bbox, padding=10):
    """Extract a template of the ball from a bounding box"""
    x1, y1, x2, y2 = bbox
    h, w = frame.shape[:2]
    
    # Add padding around the ball
    x1 = max(0, x1 - padding)
    y1 = max(0, y1 - padding)
    x2 = min(w, x2 + padding)
    y2 = min(h, y2 + padding)
    
    template = frame[y1:y2, x1:x2]
    return template, (x1, y1, x2, y2)

def calculate_histogram(roi, mask=None):
    """Calculate histogram for a region of interest"""
    # Use HSV for better color representation
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    
    # Calculate histogram for all channels
    hist_h = cv2.calcHist([hsv], [0], mask, [180], [0, 180])
    hist_s = cv2.calcHist([hsv], [1], mask, [256], [0, 256])
    hist_v = cv2.calcHist([hsv], [2], mask, [256], [0, 256])
    
    # Normalize histograms
    cv2.normalize(hist_h, hist_h, 0, 1, cv2.NORM_MINMAX)
    cv2.normalize(hist_s, hist_s, 0, 1, cv2.NORM_MINMAX)
    cv2.normalize(hist_v, hist_v, 0, 1, cv2.NORM_MINMAX)
    
    return hist_h, hist_s, hist_v

def histogram_matching_tracking(frame, prev_pos, reference_hists, search_radius=100):
    """Track ball using histogram matching"""
    if prev_pos is None or reference_hists is None:
        return None
    
    h, w = frame.shape[:2]
    x, y = prev_pos
    
    # Define search region
    x_min = max(0, x - search_radius)
    x_max = min(w, x + search_radius)
    y_min = max(0, y - search_radius)
    y_max = min(h, y + search_radius)
    
    best_score = -1
    best_pos = None
    
    # Search in a grid pattern
    step_size = 5
    for search_x in range(x_min, x_max, step_size):
        for search_y in range(y_min, y_max, step_size):
            # Extract candidate region (assume ball size ~40x40)
            roi_size = 20
            roi_x1 = max(0, search_x - roi_size)
            roi_y1 = max(0, search_y - roi_size)
            roi_x2 = min(w, search_x + roi_size)
            roi_y2 = min(h, search_y + roi_size)
            
            if roi_x2 - roi_x1 < 10 or roi_y2 - roi_y1 < 10:
                continue
                
            roi = frame[roi_y1:roi_y2, roi_x1:roi_x2]
            
            try:
                # Calculate histograms for this ROI
                roi_hist_h, roi_hist_s, roi_hist_v = calculate_histogram(roi)
                
                # Compare with reference histograms
                score_h = cv2.compareHist(reference_hists[0], roi_hist_h, cv2.HISTCMP_CORREL)
                score_s = cv2.compareHist(reference_hists[1], roi_hist_s, cv2.HISTCMP_CORREL)
                score_v = cv2.compareHist(reference_hists[2], roi_hist_v, cv2.HISTCMP_CORREL)
                
                # Weighted average (V channel is most important for gray balls)
                combined_score = 0.2 * score_h + 0.3 * score_s + 0.5 * score_v
                
                if combined_score > best_score:
                    best_score = combined_score
                    best_pos = (search_x, search_y)
                    
            except Exception:
                continue
    
    # Only return position if score is above threshold
    if best_score > 0.6:
        return best_pos
    return None

def template_matching_tracking(frame, template, prev_pos, search_radius=80):
    """Track ball using template matching"""
    if prev_pos is None or template is None:
        return None
        
    h, w = frame.shape[:2]
    x, y = prev_pos
    
    # Define search region
    x_min = max(0, x - search_radius)
    x_max = min(w, x + search_radius)
    y_min = max(0, y - search_radius)
    y_max = min(h, y + search_radius)
    
    search_region = frame[y_min:y_max, x_min:x_max]
    
    if search_region.shape[0] < template.shape[0] or search_region.shape[1] < template.shape[1]:
        return None
    
    # Convert to grayscale for template matching
    gray_search = cv2.cvtColor(search_region, cv2.COLOR_BGR2GRAY)
    gray_template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
    
    # Perform template matching
    result = cv2.matchTemplate(gray_search, gray_template, cv2.TM_CCOEFF_NORMED)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
    
    # Only accept if confidence is high enough
    if max_val > 0.7:
        # Calculate center of matched region
        template_h, template_w = gray_template.shape
        center_x = x_min + max_loc[0] + template_w // 2
        center_y = y_min + max_loc[1] + template_h // 2
        return (center_x, center_y)
    
    return None

def optical_flow_tracking(prev_frame, curr_frame, prev_pos):
    """Track ball using optical flow"""
    if prev_pos is None or prev_frame is None:
        return None
    
    # Convert to grayscale
    prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)
    curr_gray = cv2.cvtColor(curr_frame, cv2.COLOR_BGR2GRAY)
    
    # Create points around the ball position
    x, y = prev_pos
    points = np.array([[x, y]], dtype=np.float32).reshape(-1, 1, 2)
    
    # Parameters for Lucas-Kanade optical flow
    lk_params = dict(winSize=(15, 15),
                     maxLevel=2,
                     criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
    
    # Calculate optical flow
    new_points, status, error = cv2.calcOpticalFlowPyrLK(prev_gray, curr_gray, points, None, **lk_params)
    
    # Check if tracking was successful
    if status[0][0] == 1 and error[0][0] < 50:
        return tuple(new_points[0][0].astype(int))
    
    return None

def create_circular_mask(shape, center, radius):
    """Create a circular mask for the ball region"""
    mask = np.zeros(shape[:2], dtype=np.uint8)
    cv2.circle(mask, center, radius, 255, -1)
    return mask

def adaptive_ball_detection(frame, prev_pos=None, search_radius=100):
    """Advanced ball detection using multiple OpenCV techniques"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Use HoughCircles for circular object detection
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=1, minDist=50,
                              param1=50, param2=30, minRadius=10, maxRadius=50)
    
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        
        # If we have a previous position, find the closest circle
        if prev_pos is not None:
            min_dist = float('inf')
            best_circle = None
            
            for (x, y, r) in circles:
                dist = np.sqrt((x - prev_pos[0])**2 + (y - prev_pos[1])**2)
                if dist < min_dist and dist < search_radius:
                    min_dist = dist
                    best_circle = (x, y)
            
            return best_circle
        else:
            # Return the first detected circle
            return (circles[0][0], circles[0][1])
    
    return None