import cv2
import os
from numpy.lib.function_base import append
import torch
import time
import math
from skimage.feature import peak_local_max
import numpy as np
from ggcnn.models.common import post_process_output
from ggcnn.models.loss import get_pred
from ggcnn.models.ggcnn2 import GGCNN2
from skimage.draw import line
from skimage.feature import peak_local_max

def ptsOnRect(pts):
    """
    Get points on five lines of a rectangle
    Five lines are: four edge lines and one diagonal line
    pts: np.array, shape=(4, 2) (row, col)
    """
    rows1, cols1 = line(int(pts[0, 0]), int(pts[0, 1]), int(pts[1, 0]), int(pts[1, 1]))
    rows2, cols2 = line(int(pts[1, 0]), int(pts[1, 1]), int(pts[2, 0]), int(pts[2, 1]))
    rows3, cols3 = line(int(pts[2, 0]), int(pts[2, 1]), int(pts[3, 0]), int(pts[3, 1]))
    rows4, cols4 = line(int(pts[3, 0]), int(pts[3, 1]), int(pts[0, 0]), int(pts[0, 1]))
    rows5, cols5 = line(int(pts[0, 0]), int(pts[0, 1]), int(pts[2, 0]), int(pts[2, 1]))

    rows = np.concatenate((rows1, rows2, rows3, rows4, rows5), axis=0)
    cols = np.concatenate((cols1, cols2, cols3, cols4, cols5), axis=0)
    return rows, cols

def ptsOnRotateRect(pt1, pt2, w):
    """
    Draw a rectangle
    Given two points in the image (x1, y1) and (x2, y2), draw a line segment with these points as endpoints,
    with width w. This creates a rectangle in the image.
    pt1: [row, col] 
    w: width in pixels
    img: single channel image to draw rectangle on
    """
    y1, x1 = pt1
    y2, x2 = pt2

    if x2 == x1:
        if y1 > y2:
            angle = math.pi / 2
        else:
            angle = 3 * math.pi / 2
    else:
        tan = (y1 - y2) / (x2 - x1)
        angle = np.arctan(tan)

    points = []
    points.append([y1 - w / 2 * np.cos(angle), x1 - w / 2 * np.sin(angle)])
    points.append([y2 - w / 2 * np.cos(angle), x2 - w / 2 * np.sin(angle)])
    points.append([y2 + w / 2 * np.cos(angle), x2 + w / 2 * np.sin(angle)])
    points.append([y1 + w / 2 * np.cos(angle), x1 + w / 2 * np.sin(angle)])
    points = np.array(points)

    # Method 1: More precise but time-consuming
    # rows, cols = polygon(points[:, 0], points[:, 1], (10000, 10000))	# Get rows and columns of all points in rectangle

    # Method 2: Faster
    return ptsOnRect(points)	# Get rows and columns of all points in rectangle
def calcAngle2(angle):
    """
    Calculate the opposite angle for a given angle
    :param angle: in radians
    :return: angle in radians
    """
    return angle + math.pi - int((angle + math.pi) // (2 * math.pi)) * 2 * math.pi

def drawGrasps(img, grasps, mode='line'):
    """
    Draw grasps
    img:    RGB image
    grasps: list()	elements are [row, col, angle, width]
    mode:   line or region
    """
    assert mode in ['line', 'region']

    num = len(grasps)
    for i, grasp in enumerate(grasps):
        row, col, angle, width = grasp

        color_b = 255 / num * i
        color_r = 0
        color_g = -255 / num * i + 255

        if mode == 'line':
            width = width / 2

            angle2 = calcAngle2(angle)
            k = math.tan(angle)

            if k == 0:
                dx = width
                dy = 0
            else:
                dx = k / abs(k) * width / pow(k ** 2 + 1, 0.5)
                dy = k * dx

            if angle < math.pi:
                cv2.line(img, (col, row), (int(col + dx), int(row - dy)), (0, 0, 255), 1)
            else:
                cv2.line(img, (col, row), (int(col - dx), int(row + dy)), (0, 0, 255), 1)

            if angle2 < math.pi:
                cv2.line(img, (col, row), (int(col + dx), int(row - dy)), (0, 0, 255), 1)
            else:
                cv2.line(img, (col, row), (int(col - dx), int(row + dy)), (0, 0, 255), 1)

            cv2.circle(img, (col, row), 2, (color_b, color_g, color_r), -1)

        else:
            img[row, col] = [color_b, color_g, color_r]
    return img

def drawRect(img, rect):
    """
    Draw rectangle
    rect: [x1, y1, x2, y2]
    """
    print(rect)
    cv2.rectangle(img, (rect[0], rect[1]), (rect[2], rect[3]), (0, 255, 0), 1)


def depth2Gray(im_depth):
    """
    Convert depth image to 8-bit grayscale image
    """
    # Convert 16-bit to 8-bit
    x_max = np.max(im_depth)
    x_min = np.min(im_depth)
    if x_max == x_min:
        print('Image rendering error ...')
        raise EOFError
    
    k = 255 / (x_max - x_min)
    b = 255 - k * x_max
    return (im_depth * k + b).astype(np.uint8)


def inpaint(img, missing_value=0):
    """
    Inpaint missing values in depth image.
    :param missing_value: Value to fill in the depth image.
    """
    img = cv2.copyMakeBorder(img, 1, 1, 1, 1, cv2.BORDER_DEFAULT)
    mask = (img == missing_value).astype(np.uint8)

    # Scale to keep as float, but has to be in bounds -1:1 to keep opencv happy.
    scale = np.abs(img).max()
    img = img.astype(np.float32) / scale  # Has to be float32, 64 not supported.
    img = cv2.inpaint(img, mask, 1, cv2.INPAINT_NS)

    # Back to original size and value range.
    img = img[1:-1, 1:-1]
    img = img * scale

    return img


def input_img(img, out_size=300):
    """
    Crop the image, keeping the middle (320, 320) portion
    :param file: rgb file
    :return: tensor ready for network input, coordinates of top-left corner of cropped area
    """

    assert img.shape[0] >= out_size and img.shape[1] >= out_size, 'Input depth image must be greater than or equal to (320, 320)'

    # Crop middle image block
    crop_x1 = int((img.shape[1] - out_size) / 2)
    crop_y1 = int((img.shape[0] - out_size) / 2)
    crop_x2 = crop_x1 + out_size
    crop_y2 = crop_y1 + out_size
    img = img[crop_y1:crop_y2, crop_x1:crop_x2]

    # Normalize
    img = np.clip(img - img.mean(), -1., 1.).astype(np.float32)

    # Adjust order to match network input
    tensor = torch.from_numpy(img[np.newaxis, np.newaxis, :, :])  # numpy to tensor

    return tensor, crop_x1, crop_y1


def arg_thresh(array, thresh):
    """
    Get 2D indices of array elements greater than thresh
    :param array: 2D array
    :param thresh: float threshold
    :return: array shape=(n, 2)
    """
    res = np.where(array > thresh)
    rows = np.reshape(res[0], (-1, 1))
    cols = np.reshape(res[1], (-1, 1))
    locs = np.hstack((rows, cols))
    for i in range(locs.shape[0]):
        for j in range(locs.shape[0])[i+1:]:
            if array[locs[i, 0], locs[i, 1]] < array[locs[j, 0], locs[j, 1]]:
                locs[[i, j], :] = locs[[j, i], :]

    return locs


def collision_detection(pt, dep, angle, depth_map, finger_l1, finger_l2):
    """
    Collision detection
    pt: (row, col)
    angle: grasp angle in radians
    depth_map: depth image
    finger_l1 l2: length in pixels

    return:
        True: No collision
        False: Collision detected
    """
    row, col = pt

    # Two points
    row1 = int(row - finger_l2 * math.sin(angle))
    col1 = int(col + finger_l2 * math.cos(angle))
    
    # Draw gripper rectangle on cross-section
    # Check if there are any 1s in the rectangular area of the cross-section
    rows, cols = ptsOnRotateRect([row, col], [row1, col1], finger_l1)

    if np.min(depth_map[rows, cols]) > dep:   # No collision
        return True
    return False    # Collision detected

def getGraspDepth(camera_depth, grasp_row, grasp_col, grasp_angle, grasp_width, finger_l1, finger_l2):
    """
    Calculate maximum collision-free grasp depth (descent depth relative to object surface)
    based on depth image, grasp angle, and grasp width
    The grasp point is at the center of the depth image
    camera_depth: camera depth image from directly above grasp point
    grasp_angle: grasp angle in radians
    grasp_width: grasp width in pixels
    finger_l1 l2: gripper dimensions in pixels

    return: grasp depth relative to camera
    """
    # grasp_row = int(camera_depth.shape[0] / 2)
    # grasp_col = int(camera_depth.shape[1] / 2)
    # First calculate endpoints of gripper's two fingers
    k = math.tan(grasp_angle)

    grasp_width /= 2
    if k == 0:
        dx = grasp_width
        dy = 0
    else:
        dx = k / abs(k) * grasp_width / pow(k ** 2 + 1, 0.5)
        dy = k * dx
    
    pt1 = (int(grasp_row - dy), int(grasp_col + dx))
    pt2 = (int(grasp_row + dy), int(grasp_col - dx))

    # Changed to: start from highest point on grasp line and calculate grasp depth downward
    # until collision or maximum depth is reached
    rr, cc = line(pt1[0], pt1[1], pt2[0], pt2[1])   # Get coordinates of points along grasp line
    min_depth = np.min(camera_depth[rr, cc])
    # print('camera_depth[grasp_row, grasp_col] = ', camera_depth[grasp_row, grasp_col])

    grasp_depth = min_depth + 0.003
    while grasp_depth < min_depth + 0.05:
        if not collision_detection(pt1, grasp_depth, grasp_angle, camera_depth, finger_l1, finger_l2):
            return grasp_depth - 0.003
        if not collision_detection(pt2, grasp_depth, grasp_angle + math.pi, camera_depth, finger_l1, finger_l2):
            return grasp_depth - 0.003
        grasp_depth += 0.003

    return grasp_depth


class GGCNNNet:
    def __init__(self, model, device="cpu"):
        """
        Initialize the GGCNNNet class.
        :param model: Path to the model file.
        :param device: Device to load the model on ("cuda" or "cpu").
        """
        self.device = device
        # Load model
        print(f">> Loading GGCNN model on {self.device.upper()}...")
        self.net = GGCNN2()  # Initialize the GGCNN2 model
        self.net.load_state_dict(torch.load(model, map_location=self.device), strict=True)  # Load model weights
        self.net.to(self.device)  # Move the model to the specified device
        self.net.eval()  # Set the model to evaluation mode
        print(">> Model loaded successfully.")

    def predict(self, img, input_size=300):
        """
        Predict grasp parameters using the GGCNN model.
        :param img: Input depth image as a numpy array (h, w).
        :param input_size: Size of the input image for the model.
        :return: Grasp parameters (row, col, angle, width).
        """
        # Preprocess the input image
        input, self.crop_x1, self.crop_y1 = input_img(img, input_size)
        input = input.to(self.device)  # Move the input tensor to the same device as the model

        # Forward pass through the network
        with torch.no_grad():  # Disable gradient computation for inference
            self.pos_out, self.cos_out, self.sin_out, self.wid_out = get_pred(self.net, input)

        # Post-process the output
        pos_pred, ang_pred, wid_pred = post_process_output(self.pos_out, self.cos_out, self.sin_out, self.wid_out)

        # Find the point with the maximum confidence
        
        # Filter out bad grasp regions by setting a confidence threshold
        conf_thresh = 0.5
        peaks = peak_local_max(pos_pred, min_distance=10, threshold_abs=conf_thresh)

        if len(peaks) == 0:
            # If nothing passes the threshold, fallback to the best one
            loc = np.argmax(pos_pred)
            row = loc // pos_pred.shape[0]
            col = loc % pos_pred.shape[0]
        else:
            # Choose the first peak found (you could also sort by confidence or center distance)
            row, col = peaks[0]

        angle = (ang_pred[row, col] + 2 * math.pi) % math.pi
        width = wid_pred[row, col]  # Grasp width in pixels
        row += self.crop_y1
        col += self.crop_x1

        return row, col, angle, width
