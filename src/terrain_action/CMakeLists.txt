cmake_minimum_required(VERSION 3.0.2)
project(terrain_action)

set(CMAKE_BUILD_TYPE "Release")
find_package(catkin REQUIRED
    COMPONENTS
        roscpp
        grid_map_core
        grid_map_ros
        grid_map_msgs
        visualization_msgs
        geometry_msgs
        tf2
        tf2_ros
        tf2_msgs
        tf2_geometry_msgs
        pcl_ros
)
find_package (Eigen3 REQUIRED)
add_definitions(${EIGEN3_DEFINITIONS})
find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})




catkin_package(
  INCLUDE_DIRS
        include
        ${EIGEN3_INCLUDE_DIRS}
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIRS}
)

add_library(robot_move_shape_visualization src/robot_move_region_visualization.cpp)
target_link_libraries(robot_move_shape_visualization
        ${catkin_LIBRARIES}
        )

add_library(interac_points_visualization src/interact_points_visualization.cpp)
target_link_libraries(interac_points_visualization
        ${catkin_LIBRARIES}
)

add_library(terrain_action src/terrain_action.cpp)
target_link_libraries(terrain_action
        ${catkin_LIBRARIES}
        Eigen3::Eigen
        robot_move_shape_visualization
        interac_points_visualization
        )



add_executable(terrain_action_node src/terrain_action_node.cpp)
target_link_libraries(terrain_action_node
    terrain_action
    ${catkin_LIBRARIES}
)

add_library(plane_detection_node src/plane_detect.cpp)
target_link_libraries(plane_detection_node
    ${catkin_LIBRARIES}
)
# target_include_directories(plane_detection_node ${PCL_INCLUDE_DIRS})


set_target_properties(terrain_action_node PROPERTIES OUTPUT_NAME terrain_action)

