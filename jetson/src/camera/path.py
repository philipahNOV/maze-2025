import heapq
import numpy as np
import cv2
import time

from camera.board import get_board
from scipy.ndimage import distance_transform_edt
import matplotlib.pyplot as plt


def heuristic(a, distance_map):
    """
    This function calculates the heuristic value for the A* algorithm. It uses the distance map to calculate the penalty
    """
    distance_penalty = distance_map[a[0], a[1]] * 5000  # Distance from the nearest red pixel
    return -distance_penalty  # euclidean_distance -


# Check if a pixel is walkable
def is_walkable(pixel, white_pixel):
    return (pixel == white_pixel).all()


# Check if a pixel is the goal
def is_goal(current, end):
    return current == end


# A* algorithm implementation
def a_star_search(image_array, start, end, distance_map, white_pixel):
    """
    Performs A* search algorithm to find the shortest path from the start point to the end point in a maze.

    Args:
        image_array (numpy.ndarray): The maze represented as a 2D array.
        start (tuple): The starting point coordinates (row, column).
        end (tuple): The ending point coordinates (row, column).
        distance_map (numpy.ndarray): The distance map used for heuristic calculation.
        white_pixel (int): The value representing a walkable path in the maze.

    Returns:
        list or None: The list of coordinates representing the shortest path from start to end, or None if no path is found.
    """
    neighbors = [(1, 1), (1, -1), (-1, 1), (-1, -1), (0, 1), (0, -1), (1, 0), (-1, 0)]  # 8 possible movements
    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic(start, distance_map)}
    oheap = []

    heapq.heappush(oheap, (fscore[start], start))

    while oheap:
        current = heapq.heappop(oheap)[1]

        if is_goal(current, end):
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            return path[::-1]  # Return reversed path

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + 1
            if 0 <= neighbor[0] < image_array.shape[0]:
                if 0 <= neighbor[1] < image_array.shape[1]:
                    if neighbor in close_set:
                        continue
                    if not is_walkable(image_array[neighbor], white_pixel):
                        continue

                    if neighbor not in [i[1] for i in oheap] or tentative_g_score < gscore.get(neighbor, 0):
                        came_from[neighbor] = current
                        gscore[neighbor] = tentative_g_score
                        fscore[neighbor] = tentative_g_score + heuristic(neighbor, distance_map)
                        heapq.heappush(oheap, (fscore[neighbor], neighbor))

    return None

def resize_image(img, fx, fy):
    """
    Resizes the image by scaling factors fx and fy.
    """
    height, width = img.shape[:2]
    new_size = (int(width * fx), int(height * fy))
    resized_img = cv2.resize(img, new_size, interpolation=cv2.INTER_AREA)
    return resized_img, (fx, fy)


def scale_path(path, scale_factors):
    """
    Scales the path coordinates back to the original image size.
    """
    fx, fy = scale_factors
    scaled_path = [(int(x / fx), int(y / fy)) for (x, y) in path]
    return scaled_path

def get_path(img, start_position, end_position, fx=0.5, fy=0.5):
    """
    Finds a path from the start position to the end position on the given image.

    Args:
        img (numpy.ndarray): The input image.
        start_position (tuple): The starting position (x, y) on the image.
        end_position (tuple): The ending position (x, y) on the image.
        fx (float, optional): The scale factor for resizing the image in the x-axis. Defaults to 0.5.
        fy (float, optional): The scale factor for resizing the image in the y-axis. Defaults to 0.5.

    Returns:
        tuple: A tuple containing the path (list of coordinates) and the original image with the path drawn on it.

    """
    big_board = get_board(img)
    print(f'Big board shape: {big_board.shape}')

    img, scale_factors = resize_image(img, fx, fy)
    board = get_board(img)
    
    
    if start_position == None or end_position == None:
        return None, big_board

    start_position = (int(start_position[0] * fx), int(start_position[1] * fy))
    end_position = (int(end_position[0] * fx), int(end_position[1] * fy))
    
    
    # Define pixel colors
    black_pixel = [0, 0, 0]
    white_pixel = [255, 255, 255]

    if board.shape[2] == 4:
        board = board[:, :, :3]

    if big_board.shape[2] == 4:
        big_board = big_board[:, :, :3]

    # Define the heuristic function (Manhattan distance)
    black_mask = np.all(board == black_pixel, axis=-1)
    distance_map = distance_transform_edt(~black_mask)
    
    if end_position:
        print(f"End position at: {end_position}")
    else:
        print("No end blue pixel found.")

    if start_position:
        print(f"Starting position at: {start_position}")
    else:
        print("No starting green pixel found.")

    # Find path
    start_time = time.time()
    # Display board
    
    
    path = a_star_search(board, start_position, end_position, distance_map, white_pixel)
    end_time = time.time()
    print(f'Time taken: {end_time - start_time} seconds')
    if path:
        print("Path found!")
        path = scale_path(path, scale_factors)
    else:
        print("No path found.")

    return path, big_board