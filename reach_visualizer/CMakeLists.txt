cmake_minimum_required(VERSION 2.8.3)
project(reach_visualizer)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  colorized_mesh_display
  reach_core
  reach_msgs
  pcl_ros
  tf
  tf_conversions
  rbf_interpolation
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    rbf_solver
    colorization_utils
  CATKIN_DEPENDS
    colorized_mesh_display
    reach_core
    reach_msgs
    pcl_ros
    rbf_interpolation
  DEPENDS
    Eigen3
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(colorization_utils
  src/colorization_utils.cpp
)
add_dependencies(colorization_utils
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(colorization_utils
  ${catkin_LIBRARIES}
)

add_executable(rbf_test_node
  src/rbf_test_node.cpp
)
add_dependencies(rbf_test_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(rbf_test_node
  ${catkin_LIBRARIES}
  colorization_utils
)
