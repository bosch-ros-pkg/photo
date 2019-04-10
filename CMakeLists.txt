cmake_minimum_required(VERSION 2.8.3)
project(photo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
  sensor_msgs
  self_test
)

find_package( OpenCV REQUIRED )

find_package(PkgConfig REQUIRED)
pkg_check_modules(gphoto2 REQUIRED libgphoto2)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files( FILES
  Capture.srv
  GetConfig.srv
  SetConfig.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages( DEPENDENCIES
  std_msgs
  sensor_msgs
)

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
  INCLUDE_DIRS include
  LIBRARIES photo2
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs self_test
  DEPENDS OpenCV gphoto2
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${gphoto2_INCLUDE_DIRS}
)

add_library(photo2
   src/libphoto2/photo_reporter.cpp
   src/libphoto2/photo_camera_list.cpp
   src/libphoto2/photo_camera.cpp
   src/libphoto2/photo_image.cpp
)

add_executable(photo_node src/photo_node/photo_node.cpp)
add_executable(test_photo src/test_photo/test_photo.cpp)

add_dependencies(photo_node photo_generate_messages_cpp)
add_dependencies(test_photo photo_generate_messages_cpp)

target_link_libraries(photo2
  ${OpenCV_LIBRARIES}
)  

target_link_libraries(photo_node
  photo2
  ${catkin_LIBRARIES}
  ${gphoto2_LIBRARIES}
)

target_link_libraries(test_photo
  photo2
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS photo2 photo_node test_photo
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_photo.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
