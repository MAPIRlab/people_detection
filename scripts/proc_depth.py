#!/usr/bin/env python

from __future__ import division # division returns a floating point number
import os

import numpy as np
import cv2
import math
import rospy
import time
from enum import Enum
from openpose_pkg.msg import *
from openpose_pkg.srv import *
from tf.transformations import *
from math import pi
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from tf.transformations import quaternion_from_euler
from pointcloud2msg import PointCloudClass
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, Pose
from cocoparts import *


class HumanDepth():

    def __init__(self, userrgbd,  image_w, image_h, factor, data_cloud, detect_body = False, header_img = None):
        self._user = userrgbd
        self._pose_3d = []
        self._body_3d = []
        ## keep these var for the new message
        self._pose_3d = userrgbd.pose_3D

        self._body_part_3d = userrgbd.body_part_3d

        self._certainty = userrgbd.certainty

        self._name_user = userrgbd.name
        self._header_img = header_img

        self._w_min, self._h_min = userrgbd.u, userrgbd.v
        self._w_max = userrgbd.w + userrgbd.u
        self._h_max = userrgbd.h + userrgbd.v

        self._detect_body = detect_body
        self.image_h = image_h
        self.image_w = image_w
        self._factor = factor

        self._new_user_rgbd_msg = User3D()
        self._marker_msg = Marker()
        self._marker_arrow_msg = Marker()
        self._marker_msg_body = MarkerArray()
        self._marker_body = Marker()
        self._marker_link_body = MarkerArray()

        ## cloud data
        self._data_cloud = data_cloud
        

########################################### 
    def get_mean_depth(self, w_min, w_max, h_min, h_max): ## DONT USE

        mean_depth = [0.0, 0.0, 0.0]
        correct = True
        n = 0
        for width in range (w_min, w_max):
            for height in range (h_min, h_max):
                w, h = self.get_point_from_fish(width, height)
                depth_xyz, has_nan = self._data_cloud.read_point(w, h)

                if not has_nan:
                    for i in range(3):
                        mean_depth[i] += depth_xyz[i]
                n += 1
        if n > 0: 
            for i in range(3):
                mean_depth[i] = float (mean_depth[i] / n)
        else:
            correct = False
        return correct, mean_depth


##############################################
#######     UNDO THE FACTOR RESIZE      ######
##############################################

    def get_point_from_fish(self, w, h):
        ## here comes the deform
        if self._factor < 1:
            w_out = - self._factor * w
            h_out = - self._factor * h
        elif self._factor > 1:
            w_out = int( w / self._factor)
            h_out = int ( h / self._factor) 
        else:
            w_out = w
            h_out = h
        return w_out, h_out

    
##############################################
#######          BODY 3D POSE           ######
##############################################

    def get_body_pose(self):
        mean_depth = [0.0, 0.0, 0.0]
        correct = False ## this flag says if the perosn is 3D
        depth_array = [] ##we are gonna use this for the var
        self._indices_body = []
        
        k = 0 ##number of parts
        for part_idx in (self._body_part_3d):
            ## read the message parts
            bodypart3d = BodyPart3D() ## 3d components
            w_img = part_idx.x_3d
            h_img = part_idx.y_3d
            point3d = [0.0, 0.0, 0.0]

            w, h = self.get_point_from_fish(w_img, h_img)
            point3d, has_nan = self._data_cloud.read_point(w, h)
            ##print point3d
            if not has_nan:
                for i in range(len(mean_depth)):
                    mean_depth[i] += float(point3d[i])
                    
                    if i == 2:
                        ##depth component
                        depth_array.append(point3d[i])
                k += 1

                if self._detect_body:
                    bodypart3d.x_3d = float(point3d[0])
                    bodypart3d.y_3d = float(point3d[1])
                    bodypart3d.z_3d = float(point3d[2])
                    bodypart3d.score = part_idx.score
                    bodypart3d.idx = part_idx.idx 
                    self._indices_body.append(part_idx.idx)
                    self._new_user_rgbd_msg.body_part_3d.append(bodypart3d)

                    self._marker_body = self.create_marker_sphere_body_msg(part_idx.idx, point3d)
                    self._marker_msg_body.markers.append(self._marker_body)

                    

        if len(depth_array) > 0:
            var_depth = float(np.var(depth_array))
            ##print var_depth
            if var_depth > 0.001:
                correct = True
    
            if k > 0:
                for i in range(len(mean_depth)):
                    mean_depth[i] = float (mean_depth[i] / k)

        if self._detect_body and len(self._indices_body) > 0:
            self.create_marker_line_body_msg()
            

        return correct, mean_depth

#####################################################
##########      MARKERS       #######################
#####################################################

    def create_marker_line_body_msg(self):
        
        self._k = 19
        for pair_idx, pair in enumerate(CocoPairs): 
                    
            # if any of the parts that form the pair has not been detected, continue
            if pair[0] not in self._indices_body \
                or pair[1] not in self._indices_body:
                continue
            
             
            id0 = self._indices_body.index(pair[0]) 
            id1 = self._indices_body.index(pair[1])
            x_0 = self._new_user_rgbd_msg.body_part_3d[id0].x_3d
            y_0 = self._new_user_rgbd_msg.body_part_3d[id0].y_3d 
            z_0 = self._new_user_rgbd_msg.body_part_3d[id0].z_3d 

            x_1 = self._new_user_rgbd_msg.body_part_3d[id1].x_3d
            y_1 = self._new_user_rgbd_msg.body_part_3d[id1].y_3d 
            z_1 = self._new_user_rgbd_msg.body_part_3d[id1].z_3d 

            
            marker_msg = Marker()
            
            init_point = Point()
            init_point.x = x_0
            init_point.y = y_0
            init_point.z = z_0


            final_point = Point()
            final_point.x = x_1
            final_point.y = y_1
            final_point.z = z_1
            
            marker_msg.ns = "body_user"
            marker_msg.header.stamp = rospy.Time.now()
            marker_msg.header.frame_id = self._header_img 
            marker_msg.type = marker_msg.LINE_STRIP

            ## trick
            id_part = self._k
            
            id_name_user = int(self._name_user)
            if id_name_user >= 10:
                id_part *= 100
            else:
                id_part *= 10
            id_part += id_name_user

            marker_msg.id = id_part

            self._k += 1

            marker_msg.action = marker_msg.ADD
            marker_msg.points.append(init_point)
            marker_msg.points.append(final_point)

            marker_msg.scale.x = 0.1
            marker_msg.scale.y = 0.1
            marker_msg.scale.z = 0.1
            
            color_point = CocoColors[pair[0]]
            
            marker_msg.color.r = color_point[0]
            marker_msg.color.g = color_point[1]
            marker_msg.color.b = color_point[2]
            marker_msg.color.a = 1.0
            
            self._marker_msg_body.markers.append(marker_msg)
        


    def create_marker_sphere_body_msg(self, id,  point):

        point3d = point
        marker_msg = Marker()
        pose = Pose()
        pose.position.x = float(point3d[0])
        pose.position.y = float(point3d[1]) 
        pose.position.z = float(point3d[2]) 
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 1
        marker_msg.ns = "body_user"
        marker_msg.header.stamp = rospy.Time.now()
        marker_msg.header.frame_id = self._header_img 
        marker_msg.type = marker_msg.SPHERE

        ### trick
        id_part = id
        if id_part < 10:
            id_part *= 10
        id_name_user = int(self._name_user)
        if id_name_user >= 10:
            id_part *= 100
        else:
            id_part *= 10
        id_part += id_name_user

        marker_msg.id = id_part
        marker_msg.action = marker_msg.ADD
        marker_msg.pose = pose

        marker_msg.scale.x = 0.1
        marker_msg.scale.y = 0.1
        marker_msg.scale.z = 0.1

        color_point = CocoColors[id]
        
        marker_msg.color.r = color_point[0]
        marker_msg.color.g = color_point[1]
        marker_msg.color.b = color_point[2]
        marker_msg.color.a = 1.0


        return marker_msg


    def create_marker_sphere_msg(self, pos_3d):

        marker_msg = Marker()
        marker_msg.ns = "appro_user"
        marker_msg.header.stamp = rospy.Time.now()
        marker_msg.header.frame_id = self._header_img 
        marker_msg.type = marker_msg.SPHERE
        marker_msg.id = 0
        marker_msg.action = marker_msg.ADD
        marker_msg.pose = pos_3d

        marker_msg.scale.x = 0.3
        marker_msg.scale.y = 0.3
        marker_msg.scale.z = 0.3

        marker_msg.color.r = 0.0
        marker_msg.color.g = 0.0
        marker_msg.color.b = 1.0
        marker_msg.color.a = 1.0

        return marker_msg

    def create_marker_arrow_msg(self, pos_3d):

        marker_msg = Marker()
        marker_msg.ns = "appro_user"
        marker_msg.type = marker_msg.ARROW
        marker_msg.id = 1
        marker_msg.action = marker_msg.ADD
        marker_msg.pose = pos_3d
        marker_msg.header.stamp = rospy.Time.now()
        marker_msg.header.frame_id = self._header_img 


        marker_msg.scale.x = 0.4
        marker_msg.scale.y = 0.1
        marker_msg.scale.z = 0.1

        marker_msg.color.r = 0.0
        marker_msg.color.g = 1.0
        marker_msg.color.b = 0.0
        marker_msg.color.a = 1.0
        marker_msg.lifetime = rospy.Duration()

        return marker_msg


    def get_human_height(self): ## weird integers!!!
        valid = False
        human_height = None
        exist_top, exist_feet = False, False
        

        for part in self._body_part_3d:

            if not exist_feet and part.idx == CocoPart.RAnkle.value:
                exist_feet = True
                w_l, h_l = self.get_point_from_fish(part.x_3d, part.y_3d)  
                depth_feet, has_nanfeet = self._data_cloud.read_point(w_l, h_l) 

            elif not exist_feet and part.idx == CocoPart.LAnkle.value:
                exist_feet = True
                w_l, h_l = self.get_point_from_fish(part.x_3d, part.y_3d)  
                depth_feet, has_nanfeet = self._data_cloud.read_point(w_l, h_l) 


            elif not exist_top and part.idx == CocoPart.REye.value:
                exist_top = True
                w_l, h_l = self.get_point_from_fish(part.x_3d, part.y_3d)  
                depth_top, has_nanfeet = self._data_cloud.read_point(w_l, h_l) 


            elif not exist_top and part.idx == CocoPart.LEye.value:
                exist_top = True
                w_l, h_l = self.get_point_from_fish(part.x_3d, part.y_3d)  
                depth_top, has_nanfeet = self._data_cloud.read_point(w_l, h_l) 


            elif not exist_top and part.idx == CocoPart.REar.value:
                exist_top = True
                w_l, h_l = self.get_point_from_fish(part.x_3d, part.y_3d)  
                depth_top, has_nanfeet = self._data_cloud.read_point(w_l, h_l) 


            elif not exist_top and part.idx == CocoPart.LEar.value:
                exist_top = True
                w_l, h_l = self.get_point_from_fish(part.x_3d, part.y_3d)  
                depth_top, has_nanfeet = self._data_cloud.read_point(w_l, h_l) 


            elif exist_top and exist_feet:
                break

        if exist_feet and exist_top and len(depth_top) > 0 and len(depth_feet) > 0:
            print 'head', depth_top
            print 'feet', depth_feet
            human_height = depth_top[1] - depth_feet[1]
            valid = True
            print human_height


        return human_height, valid

#####################################################
##########      ORIENTATION     #####################
#####################################################

    def detect_parts(self):
          
        exist_lear = False
        exist_rear = False

        exist_leye = False
        exist_reye = False

        ###print 'orientation'
        ## this way we just need this loop once

        for part in (self._body_part_3d):
                
            if not exist_lear and part.idx == CocoPart.LEar.value:
                exist_lear = True

            elif not exist_rear and part.idx == CocoPart.REar.value:
                exist_rear = True

            elif not exist_reye and part.idx == CocoPart.REye.value:
                exist_reye = True

            elif not exist_leye and part.idx == CocoPart.LEye.value:
                exist_leye = True

            elif exist_lear and exist_rear and exist_leye and exist_reye:
                break
        

        return exist_lear, exist_rear, exist_leye, exist_reye

    def get_human_orientation(self):

        H = np.array([0, 0, 0])

        ###print 'Computing human orientation'
        do_exist_lear, do_exist_rear, do_exist_leye, do_exist_reye = self.detect_parts()

        valid = True
        angle = None


        if not do_exist_lear and do_exist_rear:
            ## angle = 0 degrees
            angle = float(0.)
            ## print 'right'

        elif not do_exist_rear and do_exist_lear:
            #3 angle = 180 degrees
            angle = float(pi)
            ## print 'left'

        elif do_exist_lear and do_exist_rear:
            if do_exist_reye and do_exist_leye:
                angle = float( pi / 2.0)
                ## print 'front'

            elif not do_exist_leye and not do_exist_reye:
                angle = float(-pi * 0.5)
                ## print 'back'

        elif not do_exist_lear and not do_exist_rear and \
         not do_exist_leye and not do_exist_reye:
            valid = False

        if valid:
            if angle is not None:
                
                H = np.array([0., float(angle), 0.])
                ##print H
        return H, valid

  
############################################      
#######       HUMAN MSG         ############    
############################################

    def create_msg_human(self):
        
        self._new_user_rgbd_msg.certainty = self._certainty 

        self._new_user_rgbd_msg.name = self._name_user
        
        ##  update 3d pose (mean pose)
        ##ok, mean_depth = self.get_mean_depth(self._w_min, self._w_max, self._h_min, self._h_max)
        is_human_3d, mean_depth = self.get_body_pose() ## this function also computes the mean position
        ##print is_human_3d
        if is_human_3d:
            ##print 'meand depth valid'
            H, ok2 = self.get_human_orientation()

            self._new_user_rgbd_msg.valid_orientation = ok2

            ##print 'orientation valid'
            ##height_human, valid = self.get_human_height()

            ##if valid:
              ##  self._new_user_rgbd_msg.human_height = height_human

            self._new_user_rgbd_msg.pose_3D.position.x = float(mean_depth[0])
            self._new_user_rgbd_msg.pose_3D.position.y = float(mean_depth[1])
            self._new_user_rgbd_msg.pose_3D.position.z = float(mean_depth[2])
            ## we still need to update the orientaation
                            
            QH = quaternion_from_euler(*H)
            ## print QH
            self._new_user_rgbd_msg.pose_3D.orientation.x = QH[0]
            self._new_user_rgbd_msg.pose_3D.orientation.y = QH[1]
            self._new_user_rgbd_msg.pose_3D.orientation.z = QH[2]
            self._new_user_rgbd_msg.pose_3D.orientation.w = QH[3]

            self._marker_msg = self.create_marker_sphere_msg(self._new_user_rgbd_msg.pose_3D)
            self._marker_arrow_msg = self.create_marker_arrow_msg(self._new_user_rgbd_msg.pose_3D)


        return self._new_user_rgbd_msg, self._marker_msg, self._marker_arrow_msg, self._marker_msg_body, is_human_3d


############################################      
#######       HUMAN SET         ############    
############################################        

class HumanDepthSet():

    def __init__(self, msg_humans, factor, create_body_3d):
        ## msg humans is UserRGBDArray
        ## msg_humans is the msg from ROS 
        
        self._factor = factor
        self._create_body_3d = create_body_3d
        ## msg_humans
        self._header_img = msg_humans.header

        self._humans_array = msg_humans.users
        self.image_w = msg_humans.image_w
        self.image_h = msg_humans.image_h
        self.compute_depth = msg_humans.compute_depth
        self._point_cloud = PointCloudClass(msg_humans.cloud, field_names = ["x", "y", "z"])

        ## msg to return
        self.userarray_msg = User3DArray()
        self.userarrarmarker_msg = MarkerArray()

        self._user_body_3d_parts = MarkerArray()

        self._sort_userarray_msg = User3DArray()

############################################      
#######       HUMAN SET MSG     ############    
############################################

    def create_msg_depth_cloud(self):
        self.cloud_correct = True
        if self._point_cloud._width > 0 and self._point_cloud._height > 0:
            self._depth_array = []

            self.userarray_msg.header = self._header_img
            self._sort_userarray_msg.header = self._header_img
            for human in (self._humans_array):
                user_msg = User3D()
                marker_msg = Marker()
                marker_arrow_msg = Marker() 
                is_human_3d = False

                current_human = HumanDepth(human, self.image_w, self.image_h, self._factor, \
                    self._point_cloud, detect_body = True, header_img = self._header_img.frame_id)
                user_msg, marker_msg, marker_arrow_msg, markers_body, is_human_3d = current_human.create_msg_human()
                
                 
                ###print user_msg 
                if is_human_3d:
                    self._depth_array.append(user_msg.pose_3D.position.z)
                    ##users
                    self.userarray_msg.users.append(user_msg)
                    ## huamn center markers
                    self.userarrarmarker_msg.markers.append(marker_msg)
                    self.userarrarmarker_msg.markers.append(marker_arrow_msg)

                    ##huamn body
                    if self._create_body_3d:
                        for i in range(len(markers_body.markers)):
                            self._user_body_3d_parts.markers.append(markers_body.markers[i])
                

            if len(self._depth_array) > 0:
                indices = np.argsort(self._depth_array)
                for i in range(len(indices)):
                    self._sort_userarray_msg.users.append( self.userarray_msg.users[indices[i]])
                    
        else:
            self.cloud_correct = False

        return self._sort_userarray_msg, self.userarrarmarker_msg, self._user_body_3d_parts, self.cloud_correct

##############################################