# OpenPose - Human Pose Extractor
This node is an adaptation of the OpenPose framework, being its only objective to extract human poses from keypoints in an image stream (video).
It is implemented with [Mobilenets](https://arxiv.org/abs/1704.04861), an efficient convolutional neural network, running the inference postprocessing from [https://github.com/ildoonet/tf-pose-estimation](https://github.com/ildoonet/tf-pose-estimation)
**Important:** This node launchs a NN which is GPU intense, therefore it is recommended to run the node in a computer with a powerful GPU (for example the Jetson TX2 board).  
#########################################################################################################################
#                                                        Scheme:                                                        #
#                                                                                                                       #
#               Camera  -->     Openpose Detection      -->     Image 2D Processing     -->     Depth Processing        #
#                                                                                                                       #
#########################################################################################################################

NOTE: openpose.yaml contents all the parameters for this package, including ros topics and services.
#########################################################################################################################

## Publishers:


**/openpose/users_3d_depth**: (Type: User3DArray). The Pose (geometry_msgs/Pose) of each human, along with their names (string), the certainty value (float32) and the 3D parts of the body (if specified).
**/openpose/frame_humans/draw_img**:  The captured RGB image with the skeleton parts supperimposed on it. Very useful for visualization (Type: sensor_msgs/Image)  
**/openpose/humans**:  (Type: openpose_pkg/HumanArray2)  Humans body parts inside the image, output of the inference postprocessing.
**/openpose/usb_cam/image_dim**:  (Type: openpose_pkg/ImageDepthHuman) Msg with a image 2d (Type: sensor_msgs/Image), a point cloud data (Type: PointCloud2) and two flags which indicates if the point cloud is valid (Type: uint8) and if the human detection is active (Type: uint8)
**/openpose/users_3d**: The pose of detected humans in 2D (Type: openpose_pkg/UserRGBDArray. Its coming from [UserArray](https://gitlab.com/mapir/mapir-ros-sources/tree/kinetic-dev/utils/user_recognizer/msg)) 
**/openpose/appro_user_markes**: (Type: MarkerArray) Markers for the 3D-Pose of each human (center) and their orientations.
**/openpose/body_3d_markers** : (Type: MarkerArray) Markers (lines and spheres) for the whole human body (3D).

## Services:
**openpose/change_frec_detection_service**: Change the frec with which the camera is sending photos. Currently: 'off', 'low' and 'high'
**openpose/change_cam_detection**: Change the camera which is sending images to the CNN.
**openpose/no_human_detected**: Indicates if no human has been detected.
**openpose/start_detection_humans_service**: To start detecting people (GPU intense) and publishing their 3D poses.)

## OpenPose + RGB-D cameras
To allow an accurate estimation of the people pose in the /map, this package includes nodes to work with data from RGB-D cameras.
Initially, openpose will extract the user skeleton, and then by exploiting the depth image, it extracts the distance from the camra to the user.
Finally, using the /tf package, we can set the user location in the /map frame_id.

1. *ssh* into the Giraff (either by using username@ip or by configuring the etc/host file to use nicknames )
```
    ssh mapir@192.168.A.B (use the wireless IP of the Giraff)
    ssh giraff_orange (use a nickname)
```

2. Once in the Giraff, we need to execute the main launch file called "movecare_main.launch" (since the roscore is hosted in the Giraff PC). 
Be sure to configure the correct local/global planners in order to set if you want to use the global Unity planner.
```
    roslaunch missions_pkg movecare_main.launch
    roslaunch openpose_pkg openpose_giraff_complete.launch
```
This command will launch almost everything in the Giraff computer (Hw drivers, navigation engines, apps, etc.).

3. Within the Giraff computer, ssh the Jetson board (Notice that this connection is local via an ethernet cable, so can only be executted fron inside the Giraff).
To do so we will use the local ethernet network.
```
    ssh nvidia@192.168.2.2  #password: nvidia
    ssh jetson (if you prefer to use the already configured nickname)
```

4. Within the Jetson board we need to start the neuronal network of OpenPose by typing:
```
    roslaunch openpose_pkg openposeinference.launch
```
This launch file is part of the OpenPose wrapper in *src/utils/jetson*.

5. These steps will make all the system to be up and ready for operation. Yet, the neuronal network is a power-consuming app, so by default is off and waiting to be started (via rosservice).
The node in charge provides a service to start. You can shut down the NN by calling the change frecuency service with 'off' as desired frec or by calling the start service with 'off' as frec.
```
    rosservice call /openpose/start_detection_humans_service "initial_frec: ''" 
```
By default, it will initiate in 'high' frecuency. It can be modified while calling the service, using 'low' or 'off'. Note: off will stop the detection and you have to call again the service to start it.

**IMPORTANT**: Remember that this configuration of PC-Giraff-Jetson is not fully directional. The Giraff is able to see everything, while the PC and the Jetson do not see each other. 
This means, for example that to start/stop the jetson via service, it must be addressed fron the Giraff, and cannot be done by the external PC.

## Visualization / Debug
Sometimes it is interesting to see the result of OpenPose (skeleton over the captured images). To do so, you can either use RVIZ to visualize the topic **/openpose/frame_humans/draw_img**:,
or execute the roslaunch called 'show_window.launch' inside the 'openpose_pkg'
```
    roslaunch openpose_pkg show_window.launch
```

## Parameters: 
**show_frame**: If you want the node 'window2' (openpose_pkg) to show the image with the drawn humans, this flag must be set to 'True'
**high_frec, low_frec, off_frec** : High, low and off frecuency with which the camera node is goint to publish the images. 'high_frec' and 'low_frec' must be greater than 0. 'off_frec' can be 0; in this case, the node does not publish any image.
**threshold_human** : Minimum value for humans certainties. If the certainty is smaller than this value, we will not consider that this human is a human.
**single_human** : If this flag is set, we look just for one human (and send this human if found).
**compute_body** : If set, the whole 3D-body is computed and send along the user 3D.
**appro_user** : ToDo
**factor** : This value changes the dimensions of the image. 'factor' can be greater, smaller or equal to 0. 
- If 'factor' > 0, new dimensions are the old ones by the factor.  
- If 'factor' == 0, new dimensiones are the old ones.
- If 'factor' < 0, new dimesiones are the old ones by the factor. Note: If these dimensiones are less than 50 x 50, the factor is set to -1 ==> new dimensions = old dimensions.



## Subscriptions: 
**Camera Node**: 'camera_down/depth/points' (sensor_msgs/PointCloud2), 'usb_cam/image_raw' (sensor_msgs/Image), 'camera_down/rgb/image_rect_color' (sensor_msgs/Image).
**Openpose Node** : 'openpose/usb_cam/image_dim' (openpose_pkg/ImageDepthHuman). Msg with the image, the depth point cloud and two flags.
**Image Processing Node** : 'openpose_pkg/humans' (openpose_pkg/HumansArray2). Humans array with the body parts for each one.
**Depth Processing Node**: 'openpose_pkg/users_3d' (openpose_pkg/UserRGBDarray). User array with the body parts and pose refers to the robot. 

