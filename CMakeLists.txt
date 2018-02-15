cmake_minimum_required(VERSION 3.1.0)
project(multicam_tools)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -fopenmp)

find_package(catkin_simple REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${OPENCV_INCLUDE_DIRS})

catkin_simple()

cs_add_library(undistort_nodelet
  src/tools.cpp
  src/calibration_data.cpp
  src/camera_extrinsics.cpp
  src/undistort_nodelet.cpp)
target_link_libraries(undistort_nodelet
  ${CATKIN_LIBRARIES}
  ${OpenCV_LIBRARIES})

cs_add_library(overlay_point_cloud_nodelet
  src/overlay_point_cloud_nodelet.cpp
  src/calibration_data.cpp
  src/camera_extrinsics.cpp
  src/camera_intrinsics.cpp
  src/tools.cpp
  )
target_link_libraries(overlay_point_cloud_nodelet
  ${CATKIN_LIBRARIES}
  ${OpenCV_LIBRARIES})

cs_install()
install(FILES nodelet_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

cs_export()
