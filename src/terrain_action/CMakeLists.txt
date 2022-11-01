cmake_minimum_required(VERSION 3.0.2)
project(terrain_action)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)
find_package (Eigen3 REQUIRED)
add_definitions(${EIGEN3_DEFINITIONS})
find_package(fcl REQUIRED)


catkin_package(
  INCLUDE_DIRS
        include
        ${EIGEN3_INCLUDE_DIRS}
)



include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIRS}
        ${FCL_INCLUDE_DIRS}
)

add_library(terrain_action src/terrain_action.cpp)
target_link_libraries(terrain_action
        ${catkin_LIBRARIES}
        fcl
        Eigen3::Eigen
        )

add_executable(terrain_action_node src/terrain_action_node.cpp)

target_link_libraries(terrain_action_node
    terrain_action
    ${catkin_LIBRARIES}
    fcl
    Eigen3::Eigen
)
set_target_properties(terrain_action_node PROPERTIES OUTPUT_NAME terrain_action)

