import numpy as np
import matplotlib.pyplot as plt
from matplotlib.path import Path

def polygon_calculate_bounding_box(polygon):
    """
    Calculate the bounding box of a polygon.
    
    :param polygon: A numpy array of shape (N, 2) representing the vertices of the polygon.
    :return: A tuple (min_x, min_y, max_x, max_y) representing the bounding box.
    """
    min_x, min_y = np.min(polygon, axis=0)
    max_x, max_y = np.max(polygon, axis=0)
    return min_x, min_y, max_x, max_y

import numpy as np

def calculate_bounding_box(start, goal, margin):
    """
    Calculate the bounding box that includes the start location, goal location,
    and a margin around the start location.

    Parameters:
    - start: tuple of (x, y) coordinates for the start location.
    - goal: tuple of (x, y) coordinates for the goal location.
    - margin: integer, margin in pixels around the start location.

    Returns:
    - bbox: tuple of (min_x, max_x, min_y, max_y) defining the bounding box.
    """
    # Calculate the bounding box with margin around the start location
    min_x = min(start[0] - margin, goal[0])
    max_x = max(start[0] + margin, goal[0])
    min_y = min(start[1] - margin, goal[1])
    max_y = max(start[1] + margin, goal[1])
    
    return min_x, max_x, min_y, max_y


def clip_bounding_box(min_x, min_y, max_x, max_y, image_shape):
    """
    Clip the bounding box to the image dimensions.
    
    :param min_x: Minimum x-coordinate of the bounding box.
    :param min_y: Minimum y-coordinate of the bounding box.
    :param max_x: Maximum x-coordinate of the bounding box.
    :param max_y: Maximum y-coordinate of the bounding box.
    :param image_shape: Shape of the image (height, width).
    :return: Clipped bounding box as (min_x, min_y, max_x, max_y).
    """
    min_x = max(0, min_x)
    min_y = max(0, min_y)
    max_x = min(image_shape[1], max_x)
    max_y = min(image_shape[0], max_y)
    return min_x, min_y, max_x, max_y


def extract_roi(image, min_x, min_y, max_x, max_y):
    """
    Extract the region of interest (ROI) from the image.
    
    :param image: The image as a numpy array.
    :param min_x: Minimum x-coordinate of the bounding box.
    :param min_y: Minimum y-coordinate of the bounding box.
    :param max_x: Maximum x-coordinate of the bounding box.
    :param max_y: Maximum y-coordinate of the bounding box.
    :return: The extracted ROI as a numpy array.
    """
    return image[int(min_y):int(max_y), int(min_x):int(max_x)]


def adjust_polygon_to_roi(polygon, min_x, min_y):
    """
    Adjust polygon coordinates relative to the top-left corner of the ROI.
    
    :param polygon: A numpy array of shape (N, 2) representing the vertices of the polygon.
    :param min_x: Minimum x-coordinate of the bounding box.
    :param min_y: Minimum y-coordinate of the bounding box.
    :return: Adjusted polygon coordinates as a numpy array.
    """
    return polygon - np.array([min_x, min_y])

def check_polygon_intersection(roi, polygon, margin=0):
    """
    Check if the polygon intersects with any non-zero pixel in the ROI.
    
    :param roi: The region of interest as a numpy array.
    :param polygon: A numpy array of shape (N, 2) representing the vertices of the polygon.
    :return: Boolean indicating whether there is an intersection with non-zero pixels.
    """
    path = Path(polygon)

    # Generate the points in the ROI
    y, x = np.mgrid[:roi.shape[0], :roi.shape[1]]
    points = np.vstack((x.ravel(), y.ravel())).T

    # Check which points in the ROI are inside the polygon
    inside = path.contains_points(points, radius=-margin)

    # Check if any pixel inside the polygon is non-zero in the ROI
    return np.any(roi.ravel()[inside])
