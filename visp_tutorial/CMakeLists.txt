cmake_minimum_required(VERSION 2.8.3)
project(visp_tutorial)
add_compile_options(-std=c++14)
#set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(PCL_DIR "/home/shyreckdc/temp/pcl/build/PCLConfig.cmake")
set(PCL_DIR "/home/shyreckdc/dependency/pcl/share/pcl-1.9")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        PCL
  rosconsole
  roscpp
  visp_bridge
        pbd
)


#find_package(PCL 1.9 REQUIRED)
#include_directories(  ${catkin_INCLUDE_DIRS}  ${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)

find_package(VISP REQUIRED)
include(${VISP_USE_FILE})

find_package(Boost REQUIRED
        COMPONENTS filesystem system signals regex date_time program_options thread)

# Since visp_bridge doesn't propagate CMAKE_CXX_STANDARD we add the following line

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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
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

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES demo_pioneer
  CATKIN_DEPENDS 
    rosconsole
    roscpp
    visp_bridge
        pbd
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
#        ${PCL_INCLUDE_DIRS}
  )

## Declare a cpp library
# add_library(demo_pioneer
#   src/${PROJECT_NAME}/demo_pioneer.cpp
# )

## Declare a cpp executable
add_executable(demo_pioneer_joy2twist_pioneer_node src/joy2twist_pioneer.cpp)
add_executable(demo_pioneer_joy2twist_pioneer_pan_node src/joy2twist_pioneer_pan.cpp)
add_executable(demo_pioneer_visual_servo_pioneer_node src/visual_servo_pioneer.cpp)
add_executable(demo_pioneer_visual_servo_pioneer_pan_node src/visual_servo_pioneer_pan.cpp)

add_executable(image_viewer src/image_viewer.cpp)
add_executable(tutorial_ibvs_4pts src/ibvs/tutorial_ibvs_4pts.cpp)
add_executable(tutorial_ibvs_4pts_display src/ibvs/tutorial_ibvs_4pts_display.cpp)
add_executable(tutorial_ibvs_4pts_wireframe_camera src/ibvs/tutorial_ibvs_4pts_wireframe_camera.cpp)
add_executable(tutorial_ibvs_4pts_wireframe_robot_viper src/ibvs/tutorial_ibvs_4pts_wireframe_robot_viper.cpp)
add_executable(tutorial_blob_tracker_live_camera src/ibvs/tutorial_blob_tracker_live_camera.cpp)
add_executable(tutorial_blob_auto_detection_and_tracking src/ibvs/tutorial_blob_auto_detection_and_tracking.cpp)
add_executable(tutorial_ibvs_4pts_image_tracking src/ibvs/tutorial_ibvs_4pts_image_tracking.cpp)
add_executable(servo_pbvs src/ibvs/servo_pbvs.cpp)
add_executable(servo_ibvs src/ibvs/servo_ibvs.cpp)


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(demo_pioneer_node demo_pioneer_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(demo_pioneer_joy2twist_pioneer_node ${catkin_LIBRARIES})
target_link_libraries(demo_pioneer_joy2twist_pioneer_pan_node ${catkin_LIBRARIES})
target_link_libraries(demo_pioneer_visual_servo_pioneer_node ${catkin_LIBRARIES})
target_link_libraries(demo_pioneer_visual_servo_pioneer_pan_node ${catkin_LIBRARIES})

target_link_libraries(image_viewer ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(tutorial_ibvs_4pts ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(tutorial_ibvs_4pts_display ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(tutorial_ibvs_4pts_wireframe_camera ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(tutorial_ibvs_4pts_wireframe_robot_viper ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(tutorial_ibvs_4pts_image_tracking ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(tutorial_blob_tracker_live_camera ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(tutorial_blob_auto_detection_and_tracking ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(servo_pbvs ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(servo_ibvs ${PCL_LIBRARIES} ${catkin_LIBRARIES})


add_executable(matching_keypoint src/detection/matching_keypoint.cpp)
target_link_libraries(matching_keypoint ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(model_based_tracking src/tracking/model_based_tracking.cpp)
target_link_libraries(model_based_tracking ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(line_tracking src/tracking/line_tracking.cpp)
target_link_libraries(line_tracking ${PCL_LIBRARIES} ${catkin_LIBRARIES})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(
  TARGETS 
    demo_pioneer_joy2twist_pioneer_node 
    demo_pioneer_joy2twist_pioneer_pan_node 
    demo_pioneer_visual_servo_pioneer_node
    demo_pioneer_visual_servo_pioneer_pan_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  launch/demo-visual-servo-pioneer-pan.launch
  launch/demo-visual-servo-pioneer.launch
  launch/pioneer-teleop.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_demo_pioneer.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetest(test)
