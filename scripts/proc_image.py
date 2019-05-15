#!/usr/bin/env python

from __future__ import division # division returns a floating point number
import os

import numpy as np
import cv2
import math
import rospy
from enum import Enum
from openpose_pkg.msg import *

from sensor_msgs.msg import Image
from cocoparts import *


class HumanImage():

    def __init__(self, human, image_w, image_h, threshold, detect_links = False, features = False):
        self._human = human
        # Coordinates of all parts detected, ordered by humans.
        # List of humans. Each human is a dictionary of parts. Each part is a tuple of x and y coordinates.
        self.parts_coords = {}
        self._rect_humans = []
        # Vector components (magnitude and direction) of all pairs detected, ordered by humans
        # List of humans. Each human is a dictionary of pairs. Each pair is a tuple of magnitude and direction
        self.pairs_components = {}

        self.image_h = image_h
        self.image_w = image_w
        self.threshold = threshold
        self._use_former_human = False
        
        self._certainty = human.certainty
        self._detect_links = detect_links
        self._poor_det = False

        self._window_done = False
        self._features = features
        self.user_msg = UserRGBD()

        if self._certainty < self.threshold or self._human is None:
            self._poor_det = True


    def fill_pairs_components(self):
        for part in self._human.parts:
                
            x_percent = part.x_percent
            y_percent = part.y_percent
            score = part.score
            idx = part.idx
                        
            x = int(x_percent * self.image_w + 0.5) ## x is WIDTH 
            y = int(y_percent * self.image_h + 0.5) ## y is HEIGHT
            self.parts_coords[idx] = (x,y)


            bodypart3d = BodyPart3D()
            bodypart3d.idx = idx
            bodypart3d.score = score
            bodypart3d.x_3d = x
            bodypart3d.y_3d = y
            bodypart3d.z_3d = 0.0

            self.user_msg.body_part_3d.append(bodypart3d)


        

###########################################
    def window_human(self):
        self.fill_pairs_components() ## this function fills parts_coord
        self.close_human() ## this fucntion finds the rect where the human is
        self._window_done = True
        return [self._min_rect, self._center]

###########################################
    def fill_links(self):
        ## this function NEEDS parts_coords FILLED
        for pair_idx, pair in enumerate(CocoPairs): 
                    
            # if any of the parts that form the pair has not been detected, continue
            if pair[0] not in self.parts_coords.keys() \
                or pair[1] not in self.parts_coords.keys():
                continue
                    
            x_0, y_0 = self.parts_coords[pair[0]]
            x_1, y_1 = self.parts_coords[pair[1]]
            magnitude = np.linalg.norm([x_1 - x_0, y_1 - y_0])
            direction = np.arctan2(y_1 - y_0, x_1 - x_0)
            self.pairs_components[pair_idx] = (magnitude, direction)


###########################################
    def get_minimum_rect(self):
         ## this function NEEDS parts_coords FILLED
            ## WIDTH X = 0
            ## HEIGHT Y = 1
        width_min = min(self.parts_coords.values())[0]
        width_max = max(self.parts_coords.values())[0]


        height_min = min(self.parts_coords.values()[1])
        height_max = max(self.parts_coords.values()[1])

        return (width_min, width_max, height_min, height_max)
    

###########################################
    
    def close_human(self):
        ## images coordinates
        w_min, w_max, h_min, h_max = self.get_minimum_rect()
        center_w = int((w_min + w_max) / 2)
        center_h = int((h_min + h_max) / 2)
        self._center = [center_w, center_h]
        self._min_rect = [w_min, w_max - w_min, h_min, h_max - h_min]



############################################

    def create_msg_human(self, name_human = 'Duck'):
         
        ## fill the fields
        self.user_msg.name = str(name_human)


        self.user_msg.certainty = self._certainty

        if not self._window_done:
            self.window_human()

        if self._detect_links:
            self.fill_links()

                
        ## this certainty must be greater than threhsold
       
        self.user_msg.u = self._min_rect[0]
        self.user_msg.v = self._min_rect[2]
        self.user_msg.w = self._min_rect[1]
        self.user_msg.h = self._min_rect[3]

        self.user_msg.pose_3D.position.x = self._center[0]
        self.user_msg.pose_3D.position.y = self._center[1]
        self.user_msg.pose_3D.position.z = 0.0
            
                    
        return self.user_msg


    def draw(self, image):
        image_drawn = cv2.resize(image, (self.image_w, self.image_h), interpolation=cv2.INTER_CUBIC)
        centers = {}
    
        # draw point
        for part_idx in range(len(CocoPart)):
            # if the part has not been detected, continue
            
            if part_idx not in self.parts_coords.keys():
               continue
            center = self.parts_coords[part_idx]
            if self._detect_links:
                centers[part_idx] = (center)
            cv2.circle(image_drawn, center, 3, CocoColors[part_idx], thickness=2, lineType=8, shift=0)
           
        # draw line
        if self._detect_links:
            for pair_idx, pair in enumerate(CocoPairsRender):
                   # if the pair has not been detected, continue
                if pair_idx not in self.pairs_components.keys(): 
                    continue
                image_drawn = cv2.line(image_drawn, centers[pair[0]], centers[pair[1]],
                                          CocoColors[pair_idx], thickness = 1)
                    

        if self._features:
            ## draw rectangle, centre and human tags (name and certainty)
            '''cv2.rectangle(image_drawn, (self._min_rect[0], self._min_rect[2]), \
            (self._min_rect[0] + self._min_rect[1], self._min_rect[2] + self._min_rect[3]),
            (0, 255, 0), 1 )'''

            ## centre
            '''cv2.circle(image_drawn, (self._center[1], self._center[0]), \
                3, (255, 255, 255), -1)'''

            ## human tags
            font = cv2.FONT_HERSHEY_PLAIN 

            '''cv2.rectangle(image_drawn, (self._min_rect[0], self._min_rect[2]), \
            (self._min_rect[0] + 10, self._min_rect[2] + 10),
            (255, 255, 255), -1 )'''
            cv2.putText(image_drawn, 'Human #'+self.user_msg.name, \
                (self._min_rect[0], self._min_rect[2]+self._min_rect[3]), font, \
                1, (255, 255, 255), 1, cv2.LINE_AA)

            '''cv2.rectangle(image_drawn, (self._min_rect[0], self._min_rect[2]+15), \
            (self._min_rect[0] + 20, self._min_rect[2] + 25),
            (255, 255, 255), -1 )'''
            cv2.putText(image_drawn, 'Certainty: '+str(format(self._certainty, '.4f')), \
                (self._min_rect[0] , self._min_rect[2]+ self._min_rect[3]+15), font, \
                1,(255, 255, 255),  1, cv2.LINE_AA)
        return image_drawn         

class HumanImageSet():

    def __init__(self, msg_humans):
        ## upper clas for humans
        ## msg_humans is the msg from ROS 
        ## unpack the whole msg
        self.image_w = msg_humans.image_w
        self.image_h = msg_humans.image_h
        self._header_img = msg_humans.header
        self._humans_array = msg_humans.humans
        self._image_depth_full = msg_humans.image_human
        ##self._compute_depth = msg_humans.compute_depth
        

        self._zero_humans = False 
        if len(self._humans_array) <= 0:
            self._zero_humans = True
        

    def create_msg_user_image(self, threshold, frame = None, draw_image = False):

        if not self._zero_humans:
            userarray_msg = UserRGBDArray()
            ##prev_users_msg = UserRGBDArray()
            userarray_msg.header = self._header_img
            userarray_msg.image_h = self.image_h
            userarray_msg.image_w = self.image_w
            userarray_msg.compute_depth = self._image_depth_full.valid_depth
            ## now we need to fill the users : users
            img = None
            if draw_image:
                img = frame
            id_name = 0
            ## for everyhuman
            for human in self._humans_array:

                current_human = HumanImage(human, self.image_w, self.image_h, threshold, True, True)
                
                user_msg = current_human.create_msg_human(id_name)
                id_name += 1
                userarray_msg.users.append(user_msg)

                if draw_image:
                    img = current_human.draw(img)
                
            return userarray_msg, img

##############################################

        




   





