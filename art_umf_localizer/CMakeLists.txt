cmake_minimum_required(VERSION 2.8.3)
project(art_umf_localizer)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  image_geometry
  image_transport
  roscpp
  cmake_modules
  cv_bridge
  eigen_conversions
  tf
  actionlib_msgs
)

add_action_files(
  DIRECTORY action
  FILES LocalizeAgainstUMF.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs
)

find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

# update git submodule
#execute_process(
#  COMMAND             git submodule update --init --recursive
#  WORKING_DIRECTORY   ${PROJECT_SOURCE_DIR}
#)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES art_umf_localizer
  CATKIN_DEPENDS geometry_msgs roscpp actionlib_msgs
#  DEPENDS system_lib
)

include_directories(
  include/art_umf_localizer
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  umflib/UMFDetectorLib/src
)

add_library(umflib umflib/UMFDetectorLib/src/umf.cpp
            umflib/UMFDetectorLib/src/decisiontree.cpp
            umflib/UMFDetectorLib/src/edge_dir_detector.cpp
            umflib/UMFDetectorLib/src/edgel_detector.cpp
            umflib/UMFDetectorLib/src/grid_detector.cpp
            umflib/UMFDetectorLib/src/marker.cpp
            umflib/UMFDetectorLib/src/model.cpp
            umflib/UMFDetectorLib/src/tracker.cpp
            umflib/UMFDetectorLib/src/util/calibration.cpp
            umflib/UMFDetectorLib/src/util/image.cpp
            umflib/UMFDetectorLib/src/util/point_tracker.cpp
            umflib/UMFDetectorLib/src/util/corner_detector.cpp
            umflib/UMFDetectorLib/src/util/line_iterator.cpp
            umflib/UMFDetectorLib/src/util/prosac.cpp
            umflib/UMFDetectorLib/src/util/distort.cpp
            umflib/UMFDetectorLib/src/util/mask_tracker.cpp
            umflib/UMFDetectorLib/src/util/renderer.cpp
            umflib/UMFDetectorLib/src/util/draw.cpp
            umflib/UMFDetectorLib/src/util/native_x.cpp
            umflib/UMFDetectorLib/src/util/robust_planar.cpp
            umflib/UMFDetectorLib/src/util/epnp.cpp
            umflib/UMFDetectorLib/src/util/objpose.cpp
            umflib/UMFDetectorLib/src/util/Rpoly.cpp
            umflib/UMFDetectorLib/src/util/opencv_factory.cpp
            umflib/UMFDetectorLib/src/util/scanline_tracker.cpp
            umflib/UMFDetectorLib/src/util/fixed_func.cpp
            umflib/UMFDetectorLib/src/util/orientation_filter.cpp
            umflib/UMFDetectorLib/src/util/shaders.cpp
            umflib/UMFDetectorLib/src/util/gl_util.cpp
            umflib/UMFDetectorLib/src/util/pnp_solver.cpp
            umflib/UMFDetectorLib/src/util/stream_factory.cpp
            umflib/UMFDetectorLib/src/util/grid_util.cpp
            umflib/UMFDetectorLib/src/util/pnp_solver_epnp.cpp
            umflib/UMFDetectorLib/src/util/umfdebug.cpp
            umflib/UMFDetectorLib/src/util/homography2d.cpp
            umflib/UMFDetectorLib/src/util/pnp_solver_lhm.cpp
            umflib/UMFDetectorLib/src/util/chromakey.cpp
            umflib/UMFDetectorLib/src/util/pnp_solver_opencv.cpp)

target_link_libraries(umflib ${OpenCV_LIBS} ${Eigen_LIBRARIES})

add_executable(umf_localizer_node src/umf_localizer_node.cpp)
add_dependencies(umf_localizer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(umf_localizer_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   umflib
)
