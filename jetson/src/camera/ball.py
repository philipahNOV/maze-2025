import cv2
import numpy as np


def find_ball_contours(mask, board):
    """
    Finds and returns the valid ball contours in the given mask.

    Parameters:
    - mask: A binary image mask where white pixels represent the ball.
    - board: The image board where the ball contours are searched.

    Returns:
    - valid_centers: A list of valid ball centers, radii, and confidence values.

    Valid ball centers are determined based on the following criteria:
    - The radius of the ball is between 5 and 20 pixels.
    - The center of the ball is located on a white pixel in the board image.
    - The confidence value is calculated based on the circularity of the contour.

    Circular contours with higher confidence values are considered more valid.

    Note: This function assumes that the OpenCV library is imported as cv2 and the NumPy library is imported as np.
    """

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    valid_centers = []
    for contour in contours:
        (x, y), radius = cv2.minEnclosingCircle(contour)
        center = [int(x), int(y)]
        radius = int(radius)
        if 5 <= radius <= 20:
            #if np.all(board[center[1], center[0]] == [255, 255, 255]):
            area = cv2.contourArea(contour)
            circularity = 4 * np.pi * (area / (cv2.arcLength(contour, True) ** 2))
            confidence = min(1.0, max(0.0, circularity))  # Confidence based on circularity
            valid_centers.append([center, radius, confidence])
    return valid_centers

def get_red_ball(img, board):
    """
    Detects and returns the position of a red ball in the given image.

    Parameters:
    img (numpy.ndarray): The input image.
    board (Board): The board object representing the game board.

    Returns:
    list or None: If a red ball is found, returns the position of the ball as a list [x, y, confidence].
                 If no ball is found, returns None.
    """
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_red1 = np.array([0, 69, 108])
    upper_red1 = np.array([66, 255, 255])
    lower_red2 = np.array([180, 69, 109])
    upper_red2 = np.array([180, 255, 255])
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask1 + mask2
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)

    valid_centers = find_ball_contours(mask, board)
    if valid_centers:
        # Return the center with the highest confidence
        best_center = max(valid_centers, key=lambda x: x[2])
        return best_center[0] + [best_center[2]]  # Return position as a list [x, y, confidence]
    return None  # No ball found

def ball_tracking(ball_dict: dict):
    """
    Calculate the velocity of a tracked ball based on its position observations.

    Args:
        ball_dict (dict): A dictionary containing the position observations of the ball.

    Returns:
        list: A list containing the x and y velocity of the ball.

    Raises:
        None

    """

    found_keys = [(key, ball_dict[key]) for key in sorted(ball_dict.keys(), reverse=True)
                  if ball_dict[key]['found']]

    if len(found_keys) < 2:
        return [0, 0]  # Not enough data to calculate velocity

    # Latest observation
    _, latest_observation = found_keys[0]
    latest_x = latest_observation['position'][0]
    latest_y = latest_observation['position'][1]
    latest_time = latest_observation['time']

    # Second latest observation
    _, previous_observation = found_keys[1]
    previous_x = previous_observation['position'][0]
    previous_y = previous_observation['position'][1]
    previous_time = previous_observation['time']

    # Calculate velocity
    time_diff = latest_time - previous_time
    if time_diff == 0:
        return [0, 0]

    x_velocity = (latest_x - previous_x) / time_diff
    y_velocity = (latest_y - previous_y) / time_diff

    return [x_velocity, y_velocity]