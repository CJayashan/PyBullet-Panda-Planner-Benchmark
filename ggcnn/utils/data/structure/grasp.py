import numpy as np
import cv2
import math
import scipy.io as scio
from ggcnn.utils.dataset_processing import mmcv


GRASP_WIDTH_MAX = 200.0


class GraspMat:
    def __init__(self, file):
        self.grasp = scio.loadmat(file)['A']   # (3, h, w)

    def height(self):
        return self.grasp.shape[1]

    def width(self):
        return self.grasp.shape[2]

    def crop(self, bbox):
        """
        crop self.grasp

        args:
            bbox: list(x1, y1, x2, y2)
        """
        self.grasp = self.grasp[:, bbox[1]:bbox[3], bbox[0]:bbox[2]]

    def rescale(self, scale, interpolation='nearest'):
        """
        scal
        """
        ori_shape = self.grasp.shape[1]
        self.grasp = np.stack([
            mmcv.imrescale(grasp, scale, interpolation=interpolation)
            for grasp in self.grasp
        ])
        new_shape = self.grasp.shape[1]
        ratio = new_shape / ori_shape
        # Scale the grasp width simultaneously
        self.grasp[2, :, :] = self.grasp[2, :, :] * ratio

    def rotate(self, rota):
        """
        Rotate clockwise
        rota: angle in degrees
        """
        self.grasp = np.stack([mmcv.imrotate(grasp, rota) for grasp in self.grasp])
        # Rotate the angle
        rota = rota / 180. * np.pi
        self.grasp[1, :, :] -= rota
        self.grasp[1, :, :] = self.grasp[1, :, :] % (np.pi * 2)
        self.grasp[1, :, :] *= self.grasp[0, :, :]

    def _flipAngle(self, angle_mat, confidence_mat):
        """
        Flip angle horizontally
        Args:
            angle_mat: (h, w) in radians
            confidence_mat: (h, w) grasp confidence
        Returns:
        """
        # Flip all angles horizontally
        angle_out = (angle_mat // math.pi) * 2 * math.pi + math.pi - angle_mat
        # Set grasp angle to 0 for non-graspable regions
        angle_out = angle_out * confidence_mat
        # Take modulo 2π for all angles
        angle_out = angle_out % (2 * math.pi)

        return angle_out

    def flip(self, flip_direction='horizontal'):
        """
        Horizontal flip
        """
        assert flip_direction in ('horizontal', 'vertical')

        self.grasp = np.stack([
            mmcv.imflip(grasp, direction=flip_direction)
            for grasp in self.grasp
        ])
        # Flip grasp angles: both position and angle values need to be flipped
        self.grasp[1, :, :] = self._flipAngle(self.grasp[1, :, :], self.grasp[0, :, :])

    def encode(self):
        """
        (4, H, W) -> (angle_cls+2, H, W)
        """
        self.grasp[1, :, :] = (self.grasp[1, :, :] + 2 * math.pi) % math.pi
        
        self.grasp_point = self.grasp[0, :, :]
        self.grasp_cos = np.cos(self.grasp[1, :, :] * 2)
        self.grasp_sin = np.sin(self.grasp[1, :, :] * 2)
        self.grasp_width = self.grasp[2, :, :]
