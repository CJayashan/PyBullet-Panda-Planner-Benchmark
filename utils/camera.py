import math
import numpy as np
import scipy.io as scio

# Image parameters
HEIGHT = 480  # Image height in pixels
WIDTH = 640   # Image width in pixels


def radians_TO_angle(radians):
    """
    Convert radians to degrees
    
    Args:
        radians (float): Angle in radians
        
    Returns:
        float: Angle in degrees
    """
    return 180 * radians / math.pi

def angle_TO_radians(angle):
    """
    Convert degrees to radians
    
    Args:
        angle (float): Angle in degrees
        
    Returns:
        float: Angle in radians
    """
    return math.pi * angle / 180

def eulerAnglesToRotationMatrix(theta):
    """
    Convert euler angles to rotation matrix using ZYX convention
    
    Args:
        theta (list): Euler angles [roll, pitch, yaw] in radians
        
    Returns:
        numpy.ndarray: 3x3 rotation matrix
    """
    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                    ])
        
    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                    ])
                
    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])
                    
    # Combine rotations: R = R_z * R_y * R_x
    R = np.dot(R_z, np.dot( R_y, R_x ))

    return R

def getTransfMat(offset, rotate):
    """
    Combine translation vector and rotation matrix into transformation matrix
    
    Args:
        offset (tuple): Translation vector (x, y, z)
        rotate (numpy.ndarray): 3x3 rotation matrix
        
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


class Camera:
    def __init__(self):
        """
        Initialize camera parameters and calculate intrinsic matrix
        
        Camera Parameters:
        - fov: Vertical field of view in degrees
        - length: Camera height from ground in meters
        - H: Actual distance from image center to first row in meters
        - W: Actual distance from image center to right edge in meters
        - A: Focal length in pixels
        - InMatrix: Camera intrinsic matrix
        - transMat: World to camera transformation matrix
        """
        self.fov = 60   # Vertical field of view
        self.length = 0.7   # Camera height
        self.H = self.length * math.tan(angle_TO_radians(self.fov/2))   # Distance from center to first row
        self.W = WIDTH * self.H / HEIGHT     # Distance from center to right edge
        # Calculate focal length in pixels
        self.A = (HEIGHT / 2) * self.length / self.H
        # Calculate intrinsic matrix
        self.InMatrix = np.array([[self.A, 0, WIDTH/2 - 0.5], [0, self.A, HEIGHT/2 - 0.5], [0, 0, 1]], dtype=float)
        # Calculate world to camera transformation matrix (4x4)
        # Euler angles: (pi, 0, 0), Translation: (0, 0, 0.7)
        rotMat = eulerAnglesToRotationMatrix([math.pi, 0, 0])
        self.transMat = getTransfMat([0, 0, 0.7], rotMat)

    def camera_height(self):
        """Returns camera height from ground in meters"""
        return self.length
    
    def img2camera(self, pt, dep):
        """
        Convert image coordinates to camera coordinates
        
        Args:
            pt (list): Image coordinates [x, y] in pixels
            dep (float): Depth value in meters
            
        Returns:
            list: Camera coordinates [x, y, z] in meters
        """
        pt_in_img = np.array([[pt[0]], [pt[1]], [1]], dtype=float)
        ret = np.matmul(np.linalg.inv(self.InMatrix), pt_in_img) * dep
        return list(ret.reshape((3,)))
    
    def camera2img(self, coord):
        """
        Convert camera coordinates to image coordinates
        
        Args:
            coord (list): Camera coordinates [x, y, z] in meters
            
        Returns:
            list: Image coordinates [row, col] in pixels
        """
        z = coord[2]
        coord = np.array(coord).reshape((3, 1))
        rc = (np.matmul(self.InMatrix, coord) / z).reshape((3,))
        return list(rc)[:-1]

    def length_TO_pixels(self, l, dep):
        """
        Convert real-world length to pixel length at given depth
        
        Args:
            l (float): Length in meters
            dep (float): Depth in meters
            
        Returns:
            float: Length in pixels
        """
        return l * self.A / dep
    
    def pixels_TO_length(self, p, dep):
        """
        Convert pixel length to real-world length at given depth
        
        Args:
            p (float): Length in pixels
            dep (float): Depth in meters
            
        Returns:
            float: Length in meters
        """
        return p * dep / self.A
    
    def camera2world(self, coord):
        """
        Convert camera coordinates to world coordinates
        
        Args:
            coord (list): Camera coordinates [x, y, z] in meters
            
        Returns:
            list: World coordinates [x, y, z] in meters
        """
        coord.append(1.)
        coord = np.array(coord).reshape((4, 1))
        coord_new = np.matmul(self.transMat, coord).reshape((4,))
        return list(coord_new)[:-1]
    
    def world2camera(self, coord):
        """
        Convert world coordinates to camera coordinates
        
        Args:
            coord (list): World coordinates [x, y, z] in meters
            
        Returns:
            list: Camera coordinates [x, y, z] in meters
        """
        coord.append(1.)
        coord = np.array(coord).reshape((4, 1))
        coord_new = np.matmul(np.linalg.inv(self.transMat), coord).reshape((4,))
        return list(coord_new)[:-1]

    def world2img(self, coord):
        """
        Convert world coordinates to image coordinates
        
        Args:
            coord (list): World coordinates [x, y, z] in meters
            
        Returns:
            list: Image coordinates [row, col] in pixels
        """
        # Convert to camera coordinates
        coord = self.world2camera(coord)
        # Convert to image coordinates
        pt = self.camera2img(coord)
        return [int(pt[1]), int(pt[0])]
    
    def img2world(self, pt, dep):
        """
        Convert image coordinates to world coordinates
        
        Args:
            pt (list): Image coordinates [x, y] in pixels
            dep (float): Depth value in meters
            
        Returns:
            list: World coordinates [x, y, z] in meters
        """
        coordInCamera = self.img2camera(pt, dep)
        return self.camera2world(coordInCamera)


if __name__ == '__main__':
    camera = Camera()
    print(camera.InMatrix)
