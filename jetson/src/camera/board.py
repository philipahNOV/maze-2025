import cv2
import numpy as np

def get_board(img):
    """
    Applies image processing techniques to the input image and returns a masked image.

    Parameters:
    img (numpy.ndarray): The input image.

    Returns:
    numpy.ndarray: The masked image.

    """
    width = int(img.shape[1])
    height = int(img.shape[0])
    dim = (width, height)

    # Resize the image
    resized_img = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)
    img_gray = cv2.cvtColor(resized_img, cv2.COLOR_BGR2GRAY)

    # Apply median blur
    img_blurred = cv2.medianBlur(img_gray, 5)

    # Initial bounds
    lower_bound_init = 0
    upper_bound_init = 110

    # Create the mask based on the initial bounds
    mask = np.ones_like(img_blurred) * 255
    mask[(img_blurred >= lower_bound_init) & (img_blurred <= upper_bound_init)] = 0

    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

    return mask_bgr


def get_mm_per_pixel(img):
    """
    Returns the size of the board in the input image.

    Returns:
    list: The size of the board.

    """
    if img is None:
        return [0, 0]
    board_size = 280 # mm
    img_size_x = img.shape[0] # pixels
    img_size_y = img.shape[1] # pixels
    mm_per_pixel_x = board_size / img_size_x
    mm_per_pixel_y = board_size / img_size_y
    return [mm_per_pixel_x, mm_per_pixel_y]