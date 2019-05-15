
from __future__ import division # division returns a floating point number
import os

import numpy as np
import cv2

from enum import Enum
class Calibrator():
    
    def __init__(self, K=None, dist=None, error=None):
        '''
        Calibration parameters (self.K and self.dist) could be given when constructing the class.
        Otherwise, calculate these parameters using self.calibrate().
        '''
        self.K = K # 3x3 camera matrix: intrinsic parameters
        self.dist = dist # Distortion coefficients (k1, k2, p1, p2)
        self.error = error # RMS reprojection error of the calibration
    
    @staticmethod
    def load_images(filenames):
        '''Given a list of image filenames, return a list of images in numpy array format.'''
        return [cv2.cvtColor(cv2.imread(image), cv2.COLOR_BGR2RGB) for image in filenames]
    

    @staticmethod
    def get_chessboard_corners(images, board_size):
        '''
        Given a list of chessboard pictures and its size, returns the location of the detected inner corners.
        Args:
            images: list of RGB chessboard pictures in numpy array format
            board size: tuple (number of inner corners of width, number of inner corners of height).
                For example a chessboard of 10 squares of width and 7 squares of height has a size of (9, 6)
        '''
        ret_and_corners = [cv2.findChessboardCorners(cv2.cvtColor(image, cv2.COLOR_RGB2GRAY), board_size, None) 
                           for image in images]
        valid_corners = [corner for ret, corner in ret_and_corners if ret]
        return valid_corners
    

    @staticmethod
    def get_chessboard_points(board_size, dx=1, dy=1):
        '''
        Given the size of the chessboard and the distances between the corners, returns a 3D grid (z = 0)
        of unprojected corners.
        Args:
            board size: tuple (number of inner corners of width, number of inner corners of height).
                For example a chessboard of 10 squares of width and 7 squares of height has a size of (9, 6)
            dx: real distance between corners in the x direction (inches)
            dy: real distance betwen corners in the y direction (inches)
        '''
        X, Y = np.mgrid[0:board_size[1]*dx:dx, 0:board_size[0]*dy:dy]
        points = np.array([X.flatten(), Y.flatten(), np.zeros(len(X.flatten()))]).T
        return points
    

    def calibrate(self, filenames, board_size, dx=1, dy=1):
        '''
        Given a list of filenames and the chessboard size, calculate the calibration parameters
        (self.K and self.dist)
        '''
        images = self.load_images(filenames)
        
        valid_corners = self.get_chessboard_corners(images, board_size)
        num_valid_images = len(valid_corners)
        
        points = self.get_chessboard_points(board_size, dx, dy)
        
        # 3D points (z=0) with origin in the upper left corner
        objpoints = np.array([points] * num_valid_images, dtype=np.float32)
        # 
        imgpoints = np.array([corner[:,0,:] for corner in valid_corners], dtype=np.float32)
        im_shape = images[0].shape[:2]
        
        self.error, self.K, self.dist, vecR, vecT = cv2.calibrateCamera(objpoints, imgpoints, im_shape,
                                                                      self.K, self.dist,
                                                                      flags=cv2.CALIB_ZERO_TANGENT_DIST)


    def save_paramaters(self, directory='', K_file='K', dist_file='dist', error_file='error'):
        assert self.K is not None and self.dist is not None and self.error is not None, "Error: calibration parameters not found."
        np.save(os.path.join(directory, K_file), self.K)
        np.save(os.path.join(directory, dist_file), self.dist)
        np.save(os.path.join(directory, error_file), self.error)


    def undistort(self, image):
        '''
        Given a distorted (raw) image from the camera, and with the calibration parameters known, returns
        the undistorted image.
        '''
        assert self.K is not None and self.dist is not None, "Error: calibration parameters not found."
        return cv2.undistort(image, self.K, self.dist, None, self.K)
