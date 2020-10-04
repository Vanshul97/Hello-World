cmake_minimum_required(VERSION 2.8.3)
project(nsra_computer_vision)

#set(OpenCV_DIR /usr/share/OpenCV/)


find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  roslib
  std_msgs
  #OpenCV
  message_generation
  nsra_odrive_interface
)

add_service_files(DIRECTORY srv FILES coords.srv)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS roslib roscpp std_msgs message_runtime nsra_odrive_interface
)

include_directories(
  include/
  config/
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

#add_executable(single_cam_callibration src/deprecated/single_cam_callibration.cpp)

#target_link_libraries(single_cam_callibration
  #${OpenCV_LIBS}
  #${catkin_LIBRARIES}
#)

#add_executable(calibrate src/deprecated/calib_intrinsic.cpp include/popt_pp.h)
#target_link_libraries(calibrate ${OpenCV_LIBS} "-lpopt")

#add_executable(read src/deprecated/read_images.cpp)
#target_link_libraries(read ${catkin_LIBRARIES} ${OpenCV_LIBS} "-lpopt" -lpthread)

#add_executable(calibrate_stereo src/calibration/calib_stereo.cpp)
#target_link_libraries(calibrate_stereo ${catkin_LIBRARIES} ${OpenCV_LIBS} "-lpopt")

#add_executable(undistort_rectify src/calibration/undistort_rectify.cpp)
#target_link_libraries(undistort_rectify ${catkin_LIBRARIES} ${OpenCV_LIBS} "-lpopt")

#add_executable(bottle_detection src/nsra_bottle_detection.cpp)
#target_link_libraries(bottle_detection ${catkin_LIBRARIES} ${OpenCV_LIBS} "-lpopt")

install(PROGRAMS
  src/blob_detection/blob_detection_left
  src/blob_detection/blob_detection_right
  src/corner_detection/cbc_detection_left
  src/corner_detection/cbc_detection_right
  src/calibration/calib_intrinsics
  src/plot_3d_points
  src/nsra_gripper
  src/bottle_detection_v1/top_view_tracking
  src/bottle_detection_v1/object_detection.py
  src/bottle_detection_v1/front_view_tracking
  src/bottle_detection_v1/nsra_bottle_detection
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install(TARGETS
#  single_cam_callibration
#  calibrate
#  read
#  calibrate_stereo
#  undistort_rectify
#  bottle_detection
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(FILES
#  srv/coords.srv
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY include/   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
