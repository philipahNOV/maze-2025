import cv2
import numpy as np
from math import sqrt
import time
import random

def point_line_distance(point, start, end):
    point = np.array(point)
    start = np.array(start)
    end = np.array(end)
    if np.array_equal(start, end):
        return np.linalg.norm(point - start)
    else:
        return np.linalg.norm(np.cross(end - start, start - point)) / np.linalg.norm(end - start)

def rdp(points, epsilon):
    """
    Applies the Ramer-Douglas-Peucker algorithm to simplify a polyline represented by a list of points.

    Args:
        points (list): A list of points representing the polyline.
        epsilon (float): The maximum distance threshold for simplification.

    Returns:
        list: A simplified polyline represented by a list of points.

    """
    if len(points) < 3:
        return points

    start = np.array(points[0])
    end = np.array(points[-1])
    distances = [point_line_distance(np.array(points[i]), start, end) for i in range(1, len(points) - 1)]
    max_distance = max(distances)
    index = distances.index(max_distance) + 1

    if max_distance > epsilon:
        result1 = rdp(points[:index + 1], epsilon)
        result2 = rdp(points[index:], epsilon)
        return result1[:-1] + result2
    else:
        return [tuple(start), tuple(end)]

def euclidean(p1, p2):
    return sqrt((p1[0] - p2[0]) ** 2 + (p2[1] - p1[1]) ** 2)

def sort_points(points):
    points = np.array(points)
    sorted_points = [points[0]]
    points = points[1:]
    
    while len(points) > 0:
        last_point = sorted_points[-1]
        distances = np.linalg.norm(points - last_point, axis=1)
        closest_index = np.argmin(distances)
        closest_point = points[closest_index]
        sorted_points.append(closest_point)
        points = np.delete(points, closest_index, axis=0)
    
    return sorted_points

def lines_(frame):
    """
    Detects and connects lines in an input frame.

    Args:
        frame (numpy.ndarray): The input frame.

    Returns:
        numpy.ndarray: The processed frame with connected lines.
    Info: This fuction is slow and should be tried to be optimized, or switched to a faster implementation.
    """
    print(f'Frameshape{frame.shape}')
    fx = 3
    fy = 3
    img = cv2.resize(frame, (0, 0), fx=fx, fy=fy)
    if img is None:
        print("Error: Could not read image.")
        return None

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (9, 9), 0)
    edges = cv2.Canny(gray, 10, 150, apertureSize=3)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=1, minLineLength=1, maxLineGap=1)
    
    new_lines = []
    for l in lines:
        if random.random() < 0.15:
            new_lines.append(l)
    lines = new_lines
    if lines is not None:
        lines = sorted(lines, key=lambda line: (line[0][0], line[0][1]))
        points = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            points.append((x1, y1))
            points.append((x2, y2))
        sorted_points = sort_points(points)
        max_length = 70
        blank_image = np.zeros_like(img)
        connected_lines = []
        current_line = [sorted_points[0]]
        for i in range(1, len(sorted_points)):
            if euclidean(sorted_points[i - 1], sorted_points[i]) <= max_length:
                current_line.append(sorted_points[i])
            else:
                connected_lines.append(current_line)
                current_line = [sorted_points[i]]
        
        if current_line:
            connected_lines.append(current_line)
        filtered_lines = []
        for line in connected_lines:
            x_coords = [point[0] for point in line]
            y_coords = [point[1] for point in line]
            min_x, max_x = min(x_coords), max(x_coords)
            min_y, max_y = min(y_coords), max(y_coords)
            
            if abs(max_x - min_x) <= 1.25 * abs(max_y - min_y) and abs(max_y - min_y) <= 1.25 * abs(max_x - min_x):
                filtered_lines.append(line)
        pathts = []
        epsilon = 7
        for path in filtered_lines:
            simplified_path = rdp(path, epsilon)
            pathts.append(simplified_path)
        
        filtered_lines = pathts
        sorted_filtered_lines = [sort_points(line) for line in filtered_lines]
        filtered_lines = sorted_filtered_lines
        for i in range(len(filtered_lines)):
            filtered_lines[i].append(filtered_lines[i][0])
            
        for line in filtered_lines:
            for i in range(len(line) - 1):
                cv2.line(blank_image, line[i], line[i + 1], (255, 255, 255), 2)
        for i in range(len(filtered_lines)):
            for j in range(len(filtered_lines)):
                if i != j or (i == j and len(filtered_lines[i]) > 1):
                    for end_point in [filtered_lines[i][0], filtered_lines[i][-1]]:
                        for start_point in [filtered_lines[j][0], filtered_lines[j][-1]]:
                            if euclidean(end_point, start_point) <= max_length:
                                cv2.line(blank_image, end_point, start_point, (255, 255, 255), 2)
        blank_image = cv2.resize(blank_image, (frame.shape[1], frame.shape[0]))
        return blank_image
    return None


def detect_shapes(img):
    """
    Detects shapes in an image and returns the positions of the start and end points.

    Parameters:
    - img: A numpy array representing the input image.

    Returns:
    - start_position: A tuple representing the (x, y) coordinates of the detected start point.
    - end_position: A tuple representing the (x, y) coordinates of the detected end point.
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    contours, _ = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    img_copy = img.copy()
    cv2.drawContours(img_copy, contours, -1, (255, 255, 255), cv2.FILLED)
    contours, _ = cv2.findContours(cv2.cvtColor(img_copy, cv2.COLOR_BGR2GRAY), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    start_position = None
    end_position = None
    for contour in contours:
        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
        
        if len(approx) == 3:
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                start_position = (cX, cY)
        elif len(approx) == 5:
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                end_position = (cX, cY)
    if start_position != None and end_position == None:
        print("found start, but no end")
    if start_position == None and end_position != None:
        print("found end, but no start")
    return start_position, end_position
