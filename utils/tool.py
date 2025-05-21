import math
import numpy as np


def quaternion_to_rotation_matrix(q):  # x, y, z, w
    """
    Convert quaternion to rotation matrix
    
    Args:
        q: Quaternion in [x, y, z, w] format
        
    Returns:
        numpy.ndarray: 3x3 rotation matrix
    """
    rot_matrix = np.array(
        [[1.0 - 2 * (q[1] * q[1] + q[2] * q[2]), 2 * (q[0] * q[1] - q[3] * q[2]), 2 * (q[3] * q[1] + q[0] * q[2])],
         [2 * (q[0] * q[1] + q[3] * q[2]), 1.0 - 2 * (q[0] * q[0] + q[2] * q[2]), 2 * (q[1] * q[2] - q[3] * q[0])],
         [2 * (q[0] * q[2] - q[3] * q[1]), 2 * (q[1] * q[2] + q[3] * q[0]), 1.0 - 2 * (q[0] * q[0] + q[1] * q[1])]],
        dtype=np.float)
    return rot_matrix


def getTransfMat(offset, rotate):
    """
    Combine translation vector and rotation matrix into transformation matrix
    
    Args:
        offset: Translation vector (x, y, z)
        rotate: 3x3 rotation matrix
        
    Returns:
        numpy.ndarray: 4x4 transformation matrix
    """
    mat = np.array([
        [rotate[0, 0], rotate[0, 1], rotate[0, 2], offset[0]], 
        [rotate[1, 0], rotate[1, 1], rotate[1, 2], offset[1]], 
        [rotate[2, 0], rotate[2, 1], rotate[2, 2], offset[2]],
        [0, 0, 0, 1.] 
    ])
    return mat


def depth2Gray(im_depth):
    """
    Convert depth image to 8-bit grayscale image
    
    Args:
        im_depth: Input depth image
        
    Returns:
        numpy.ndarray: 8-bit grayscale image
        
    Raises:
        EOFError: If image rendering fails (max depth equals min depth)
    """
    x_max = np.max(im_depth)
    x_min = np.min(im_depth)
    if x_max == x_min:
        print('Image rendering error...')
        raise EOFError
    
    k = 255 / (x_max - x_min)
    b = 255 - k * x_max
    return (im_depth * k + b).astype(np.uint8)


def depth2Gray3(im_depth):
    """
    Convert depth image to 3-channel 8-bit grayscale image
    
    Args:
        im_depth: Input depth image
        
    Returns:
        numpy.ndarray: 3-channel 8-bit grayscale image of shape (h, w, 3)
        
    Raises:
        EOFError: If image rendering fails (max depth equals min depth)
    """
    x_max = np.max(im_depth)
    x_min = np.min(im_depth)
    if x_max == x_min:
        print('Image rendering error...')
        raise EOFError
    
    k = 255 / (x_max - x_min)
    b = 255 - k * x_max

    ret = (im_depth * k + b).astype(np.uint8)
    ret = np.expand_dims(ret, 2).repeat(3, axis=2)
    return ret


def distancePt(pt1, pt2):
    """
    Calculate Euclidean distance between two 2D points
    
    Args:
        pt1: First point [row, col] or [x, y]
        pt2: Second point [row, col] or [x, y]
        
    Returns:
        float: Euclidean distance
    """
    return ((pt1[0] - pt2[0]) ** 2 + (pt1[1] - pt2[1]) ** 2) ** 0.5


def distancePt3d(pt1, pt2):
    """
    Calculate Euclidean distance between two 3D points
    
    Args:
        pt1: First point [x, y, z]
        pt2: Second point [x, y, z]
        
    Returns:
        float: Euclidean distance
    """
    return ((pt1[0] - pt2[0]) ** 2 + (pt1[1] - pt2[1]) ** 2 + (pt1[2] - pt2[2]) ** 2) ** 0.5


def calcAngleOfPts(pt1, pt2):
    """
    Calculate counterclockwise angle from pt1 to pt2 in range [0, 2pi)
    
    Args:
        pt1: First point [x, y] in Cartesian coordinates (not image coordinates)
        pt2: Second point [x, y] in Cartesian coordinates
        
    Returns:
        float: Angle in radians
    """
    dy = pt2[1] - pt1[1]
    dx = pt2[0] - pt1[0]
    return (math.atan2(dy, dx) + 2*math.pi) % (2*math.pi)
    

def radians_TO_angle(radians):
    """
    Convert radians to degrees
    
    Args:
        radians: Angle in radians
        
    Returns:
        float: Angle in degrees
    """
    return 180 * radians / math.pi


def angle_TO_radians(angle):
    """
    Convert degrees to radians
    
    Args:
        angle: Angle in degrees
        
    Returns:
        float: Angle in radians
    """
    return math.pi * angle / 180


def depth3C(depth):
    """
    Convert depth image to 3-channel uint8 image
    
    Args:
        depth: Input depth image
        
    Returns:
        numpy.ndarray: 3-channel uint8 image
    """
    depth_3c = depth[..., np.newaxis]
    depth_3c = np.concatenate((depth_3c, depth_3c, depth_3c), axis=2)
    return depth_3c.astype(np.uint8)
