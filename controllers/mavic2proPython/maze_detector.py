import pdb
import pickle
import random
import copy
import cv2  
import numpy as np  

color_ranges = []

'''

NOTE:

THIS MAZE DETECTOR USES ELEMENTS FROM HOMEWORK 3 AS WELL AS:
https://stackoverflow.com/questions/63954772/perspective-transform-in-opencv-python

'''


def add_color_range_to_detect(lower_bound, upper_bound):
    '''
  @param lower_bound: Tuple of BGR values
  @param upper_bound: Tuple of BGR values
  '''
    global color_ranges
    color_ranges.append([lower_bound, upper_bound])  # Add color range to global list of color ranges to detect


def check_if_color_in_range(bgr_tuple):
    '''
  @param bgr_tuple: Tuple of BGR values
  @returns Boolean: True if bgr_tuple is in any of the color ranges specified in color_ranges
  '''
    global color_ranges
    for entry in color_ranges:
        lower, upper = entry[0], entry[1]
        in_range = True
        for i in range(len(bgr_tuple)):
            if bgr_tuple[i] < lower[i] or bgr_tuple[i] > upper[i]:
                in_range = False
                break
        if in_range:
            return True

    return False


## Creates a mask of the given image depending on whether the pixel value is in the color mask
def do_color_filtering(img):
    img_height = img.shape[0]
    img_width = img.shape[1]

    mask = np.zeros((img_height, img_width))  # Index mask as [height, width] (e.g.,: mask[y,x])

    for i in range(0, img_height):
        for j in range(0, img_width):
            pixel = img[i][j]

            if check_if_color_in_range((pixel[0], pixel[1], pixel[2])):
                mask[i][j] = 1
            else:
                mask[i][j] = 0

    return mask


# Gets centerpoint of coordinates
def get_coordinates(list_of_coords):
    centre = np.mean(list_of_coords, axis=0)
    return [int(centre[0]), int(centre[1])]


# Identifies which coordinates are 1 in a mask
def get_coords_in_mask(mask):
    coords = []

    img_height = len(mask)
    img_width = len(mask[0])

    for i in range(0, img_height):
        for j in range(0, img_width):
            if mask[i][j] == 1:
                coords.append((i, j))

    return coords


# Get a dictionary with the min/max x/y values in a given mask (works for corner maps by identifying blobs)
def get_crop_bounds_corners(mask):
    coords = get_coords_in_mask(mask)
    #print(coords)

    blob_coords = get_blobs(mask)
    blob_centres = get_blob_centroids(blob_coords)

    min_y = int(min(blob_centres, key=lambda t: t[0])[0])
    max_y = int(max(blob_centres, key=lambda t: t[0])[0])

    min_x = int(min(blob_centres, key=lambda t: t[1])[1])
    max_x = int(max(blob_centres, key=lambda t: t[1])[1])

    return {
        "min_y": min_y,
        "max_y": max_y,
        "min_x": min_x,
        "max_x": max_x
    }


# Take a mask of corners as input and crop it based on the min/max x/y values of the corners
def get_cropped_mask_corners(mask):
    cb = get_crop_bounds_corners(mask)

    #print("cb", cb)

    min_x = cb["min_x"]
    max_x = cb["max_x"]
    min_y = cb["min_y"]
    max_y = cb["max_y"]

    img_height = len(mask)
    img_width = len(mask[0])

    mask_new = np.zeros((max_y - min_y, max_x - min_x))

    for i in range(0, max_y - min_y):
        for j in range(0, max_x - min_x):
            mask_new[i][j] = mask[i + min_y][j + min_x]

    #print("vals y, x", len(mask_new), len(mask_new[0]))

    return mask_new


# Get bounds for a cropping mask (does not work on corners as it looks for all pixel values, not just blobs
def get_crop_bounds(mask):
    coords = get_coords_in_mask(mask)
    # print(coords)

    min_y = int(min(coords, key=lambda t: t[0])[0])
    max_y = int(max(coords, key=lambda t: t[0])[0])

    min_x = int(min(coords, key=lambda t: t[1])[1])
    max_x = int(max(coords, key=lambda t: t[1])[1])

    return {
        "min_y": min_y,
        "max_y": max_y,
        "min_x": min_x,
        "max_x": max_x
    }


# Take a mask as input and output a cropped mask based on the min/max x/y values of that mask
def get_cropped_mask(mask):
    cb = get_crop_bounds(mask)

    min_x = cb["min_x"]
    max_x = cb["max_x"]
    min_y = cb["min_y"]
    max_y = cb["max_y"]

    img_height = len(mask)
    img_width = len(mask[0])

    mask_new = np.zeros((max_y - min_y, max_x - min_x))

    for i in range(0, max_y - min_y):
        for j in range(0, max_x - min_x):
            mask_new[i][j] = mask[i + min_y][j + min_x]

    #print("vals y, x", len(mask_new), len(mask_new[0]))

    return mask_new


def crop_main_image(img, cb):
    min_x = cb["min_x"]
    max_x = cb["max_x"]
    min_y = cb["min_y"]
    max_y = cb["max_y"]

    new_img = img[min_y: max_y, min_x: max_x]

    return new_img


# Overlay masks (or operation on all 1s and 0s of the two masks)
def combine_masks(mask1, mask2):
    img_height = len(mask1)
    img_width = len(mask1[0])

    mask_new = np.zeros((img_height, img_width))

    for i in range(0, img_height):
        for j in range(0, img_width):
            if mask1[i][j] == 1 or mask2[i][j] == 1:
                mask_new[i][j] = 1

    return mask_new


# Expand function from homework 3 for identifying blobs
def expand_nr(img_mask, cur_coord, coordinates_in_blob):
    coordinates_in_blob = []
    coordinate_list = [cur_coord]
    while len(coordinate_list) > 0:
        cur_coordinate = coordinate_list.pop()

        if cur_coordinate[0] < 0 or cur_coordinate[1] < 0:
            continue

        if cur_coordinate[0] >= img_mask.shape[0] or cur_coordinate[1] >= img_mask.shape[1]:
            continue

        if img_mask[cur_coordinate[0], cur_coordinate[1]] == 0:
            continue

        coordinates_in_blob.append([cur_coordinate[0], cur_coordinate[1]])

        coordinate_list.append([cur_coordinate[0], cur_coordinate[1] + 1])
        coordinate_list.append([cur_coordinate[0], cur_coordinate[1] - 1])
        coordinate_list.append([cur_coordinate[0] + 1, cur_coordinate[1]])
        coordinate_list.append([cur_coordinate[0] - 1, cur_coordinate[1]])

        img_mask[cur_coordinate[0], cur_coordinate[1]] = 0

    return coordinates_in_blob


# Function from homework 3 for identifying blobs on a mask
def get_blobs(img_mask):
    img_mask_height = img_mask.shape[0]
    img_mask_width = img_mask.shape[1]

    mask = copy.copy(img_mask)
    blobs_list = []

    for i in range(0, img_mask_height):
        for j in range(0, img_mask_width):
            if img_mask[i, j] == 1:
                blob_coords = expand_nr(mask, [i, j], [])
                # print("b", len(blob_coords))
                if len(blob_coords) >= 1:
                    blobs_list.append(blob_coords)

    # print(blobs_list)
    return blobs_list


# Function from homework 3 for getting the center of blobs in a list of blobs
def get_blob_centroids(blobs_list):
    object_positions_list = []

    for blob in blobs_list:
        if len(blob) < 10:
            continue

        centroid = np.mean(blob, axis=0)
        object_positions_list.append(centroid)

    # print("Blob Centroids", object_positions_list)
    return object_positions_list


def detect_maze(filename):
    img = cv2.imread(filename)

    global color_ranges

    # Make a mask of the maze based on black values in our image
    color_ranges
    add_color_range_to_detect([0, 0, 0], [20, 20, 20])
    img_mask_maze = do_color_filtering(img)

    # Make a mask of the maze corners by identifying the green pixels in our image
    color_ranges = []
    add_color_range_to_detect([0, 40, 0], [70, 256, 70])  # green (corners)
    img_mask_corners = do_color_filtering(img)

    # Crop the corners mask, find out where the corner centroids are, and store those values
    corners_cropped = get_cropped_mask_corners(img_mask_corners)
    corner_blobs = get_blobs(corners_cropped)
    corner_centroids = get_blob_centroids(corner_blobs)
    corner_points = [[cp[0], cp[1]] for cp in corner_centroids]
    # print("corners", corner_points)

    img_mask_cropped = get_cropped_mask(combine_masks(img_mask_maze, img_mask_corners))


    ##### CROP IMAGE

    cb = get_crop_bounds_corners(img_mask_corners)
    c_img = crop_main_image(img, cb)

    #### CROP IMAGe

    # Make a mask of the beginning start point by identifying the red pixels in our image
    color_ranges = []
    add_color_range_to_detect([0, 0, 100], [60, 60, 256])  # red (beginning)
    img_mask_start = do_color_filtering(c_img)

    # Identify the center of the red blob to tell our robot where to start at.
    red_coords = get_coords_in_mask(img_mask_start)
    red_centre = get_coordinates(red_coords)

    # Make a mask of the endpoint by identifying the blue pixels in our image
    color_ranges = []
    add_color_range_to_detect([40, 0, 0], [255, 60, 60])  # blue (end)
    img_mask_end = do_color_filtering(c_img)

    # Identify the center of the blue blob to tell our robot where to end at.
    blue_coords = get_coords_in_mask(img_mask_end)
    blue_centre = get_coordinates(blue_coords)

    # cv2.imshow('orig', img)
    # cv2.imshow('cropped', c_img)
    # cv2.imshow('mask_maze', img_mask_maze)
    # cv2.imshow('mask_red', img_mask_start)
    # cv2.imshow('mask_blue', img_mask_end)
    # cv2.imshow('mask-corners', img_mask_corners)
    # cv2.imshow('corners_cropped', corners_cropped)
    # cv2.imshow('corners_transformed', corner_perspective_transform)
    cv2.imshow('maze croppped', img_mask_cropped)
    # cv2.imshow('maze tranform', maze_perspective_transform)
    # print("Done!")
    cv2.waitKey(-1)  # Wait until a key is pressed to exit the program
    cv2.destroyAllWindows()  # Close all the windows

    np.save("map.npy", img_mask_cropped)

    return red_centre, blue_centre