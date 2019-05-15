#!/usr/bin/env python


import numpy as np
import cv2
import os

class CalibratorFishDepth():

    def __init__(self, image_fish, image_rgbd):
        ##t this class calibrates the fish an the rgbd
        self._H = None
        self._image_fish = image_fish
        self._image_rgbd = image_rgbd


    
    def process_image(self, image):
        (width, height, channels) = image.shape
        ## image to grayscale
        if channels == 3:
            image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            image_gray = image

        ## blurring
        img_blur = cv2.medianBlur(image_gray, 5)

        ## threshold gaussian
        img_gauss = cv2.adaptiveThreshold(img_blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, \
            cv2.THRESH_BINARY, 11, 2)

        ## open operation
        kernel = np.ones((5,5), np.uint8)
        img_open = cv2.morphologyEx(img_gauss, cv2.MORPH_OPEN, kernel)

        return img_open

    def find_pattern(self, image):
        success = False
        ## image gray scale !!!
        width, height = len(image), len(image[0])
        img_rgb = image.copy()
        ## so chan == 1
        w_0 = int(width/2)
        h_0 = int(height/2)
        w_roi = 20
        h_roi = 20
        pattern = []
        k = 0
        pattern = None
        while not success and k < 1000:
            img_crop = image[w_0:w_0+w_roi, h_0:h_0+h_roi]
            circles = cv2.HoughCircles(img_crop, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30,\
                minRadius=3, maxRadius=0)

            if circles is None:
                ##raise NameError ("Error: Pattern not found!")
                success = False
                break
            k += 1
            if len(circles) == 4:
                ## we have found the pattern
                pattern = circles
                success = True
            elif len(circles) < 4:
                ## increase the roi
                w_0 -= 10
                h_0 -= 10
                w_roi += 20
                h_roi += 20
                if w_0 < 0:
                    w_0 = 0
                if h_0 < 0:
                    h_0 = 0
                if w_roi + w_0 > width - 1:
                    w_roi = width - w_0 - 1
                if h_roi + h_0 > height -1:
                    h_roi = height - 1 - h_0

            else:
                # len(circles) > 4
                w_0 += 20
                h_0 += 20
                w_roi -= 40
                h_roi -= 40
                if w_0 < 0:
                    w_0 = 0
                if h_0 < 0:
                    h_0 = 0
                if w_roi + w_0 > width - 1:
                    w_roi = width - w_0 - 1
                if h_roi + h_0 > height -1:
                    h_roi = height - 1 - h_0

        ## we have found the pattern
        if success:
            for (x,y,r), in circles:
                ## draw the circles
                cv2.circle(img_rgb,(x,y), r, (0, 255, 0), 4)
                cv2.circle(img, (x, y), 2, (0,0, 255), 3)
                pattern.append([x + w_0, y + h_0])

            cv2.imshow("detected circles", img_rgb)
            cv2.waitKey(0)

            ## sort the centers
            pattern_aux = pattern
            x_centroid = 0
            y_centroid = 0
            for i in range(len(pattern)):
                w_centroid += pattern[i][0]
                h_centroid += pattern[i][1]

            w_centroid /= 4.0
            h_centroid /= 4.0

            for u in range(len(pattern)):
                idx = 0
                idx = int (pattern_aux[u][1] > h_centroid) * 2 + int (pattern_aux[u][0] > w_centroid)
                pattern[idx] = (pattern_aux[u])

        return pattern
        
    def calibrate(self):
        H = None
        img_fish_proc = self.process_image(self._image_fish)
        img_rgbd_proc = self.process_image(self._image_rgbd)
        print 'lookign for patterns'
        point2 = self.find_pattern(img_fish_proc)
        if point2 is not None: 

            point1 = self.find_pattern(img_rgbd_proc)
            if point1 is not None:
                H = cv2.getPerspectiveTransform(point1, point2)
                if H is not None:
                    self._H = H ## H from RGBD to FISH
        return H       
