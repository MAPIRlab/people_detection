cmake_minimum_required(VERSION 2.8.3)
project(openpose_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	rospy
  	std_msgs
  	geometry_msgs
  	message_generation
  	cv_bridge
  	sensor_msgs
  	visualization_msgs
  	actionlib_msgs
    ##proxemic2_pkg
    ## people_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)
## Find OpenCV
find_package(OpenCV 3.0.0 REQUIRED)

## Generate messages in the 'msg' folder

add_action_files(
  DIRECTORY action
  FILES HumanCentered.action
)

add_message_files(
  FILES
    ImageDepthHuman.msg
    BodyPart.msg
    Human.msg
    ##HumanArray.msg
    HumanArray2.msg
    ImageDepthHuman.msg
    User3D.msg
    User3DArray.msg
    ## User.msg
    ## UserArray.msg
    UserRGBD.msg
    UserRGBDArray.msg
    #HumanDepth.msg
    #DepthArray.msg
    BodyPart3D.msg
    PeopleGoalArray.msg
    PersonGoal.msg
)



## Generate services in the 'srv' folder
add_service_files(
	FILES
#   ChangeDetect.srv 
	#ChangeCam.srv
	#ChangeFrecDetection.srv
	#CalibrateSrv.srv
	StartDetectionHuman.srv
	#NoHuman.srv
#   Service1.srv
#   Service2.srv
)

generate_messages(
    DEPENDENCIES
   	 std_msgs
   	 sensor_msgs
	 geometry_msgs
	 visualization_msgs
	 actionlib_msgs
     ## proxemic2_pkg
     ## people_msgs

)


## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES openpose_pkg
  CATKIN_DEPENDS 
	rospy 
	std_msgs 
	geometry_msgs 
	message_runtime
	sensor_msgs
	visualization_msgs
	actionlib_msgs
    ## proxemic2_pkg
    ## people_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)
link_directories(${OpenCV_LIB_DIR})


############################################
##########  OPENPOSE2 COORDINATION #########
############################################
catkin_install_python(
	PROGRAMS
	scripts/openpose_new
  
	DESTINATION
	 ${CATKIN_PACKAGE_BIN_DESTINATION}
)

############################################
##########  OPENPOSE2 INFERENCE #########
############################################
catkin_install_python(
	PROGRAMS
	scripts/openpose_inference
  
	DESTINATION
	 ${CATKIN_PACKAGE_BIN_DESTINATION}
)


############################################
#####  OPENPOSE BASIC COORDINATION    ######
############################################
catkin_install_python(
    PROGRAMS
    scripts/openpose_basic
  
    DESTINATION
     ${CATKIN_PACKAGE_BIN_DESTINATION}
)


############################################
##########  CAMERA FISH NODE 	   #########
############################################
catkin_install_python(
	PROGRAMS
	scripts/camera_fish
  
	DESTINATION
	 ${CATKIN_PACKAGE_BIN_DESTINATION}
)

############################################
##########  CAMERA FISH NODE       #########
############################################
catkin_install_python(
    PROGRAMS
    scripts/User2Person2
  
    DESTINATION
     ${CATKIN_PACKAGE_BIN_DESTINATION}
)


############################################
##########  IMAGE PROCESSING   #########
############################################
catkin_install_python(
	PROGRAMS
	scripts/proc_human_node
  
	DESTINATION
	 ${CATKIN_PACKAGE_BIN_DESTINATION}
)


############################################
##########  IMAGE PROCESSING BASIC #########
############################################
catkin_install_python(
    PROGRAMS
    scripts/proc_human_basic
  
    DESTINATION
     ${CATKIN_PACKAGE_BIN_DESTINATION}
)



############################################
##########  DEPTH     PROCESSING   #########
############################################
catkin_install_python(
	PROGRAMS
	scripts/proc_depth_node
  
	DESTINATION
	 ${CATKIN_PACKAGE_BIN_DESTINATION}
)


############################################
##########  WINDOW SHOW            #########
############################################
catkin_install_python(
	PROGRAMS
	scripts/window2
  
	DESTINATION
	 ${CATKIN_PACKAGE_BIN_DESTINATION}
)

############################################
##########  CALIBRATE FISH         #########
############################################
catkin_install_python(
	PROGRAMS
	scripts/calibrate_fish_node
  
	DESTINATION
	 ${CATKIN_PACKAGE_BIN_DESTINATION}
)


############################################
##########  SIMULATOR        #########
############################################
catkin_install_python(
    PROGRAMS
    scripts/generator_users
  
    DESTINATION
     ${CATKIN_PACKAGE_BIN_DESTINATION}
)



############################################
##########  FIXED USERS GENERATOR      #########
############################################
catkin_install_python(
    PROGRAMS
    scripts/fixed_users
    
    DESTINATION
     ${CATKIN_PACKAGE_BIN_DESTINATION}
)







