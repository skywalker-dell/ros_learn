cmake_minimum_required(VERSION 3.0.2)
project(test)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  grid_map_msgs
  message_filters
  sensor_msgs
  tf2
  tf2_ros
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test
#  CATKIN_DEPENDS roscpp
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
)




add_executable(demo src/demo.cpp)
target_link_libraries(demo ${catkin_LIBRARIES})

add_executable(timer_demo src/timer_demo.cpp)
target_link_libraries(timer_demo ${catkin_LIBRARIES})

add_executable(param_demo src/param_test.cpp)
target_link_libraries(param_demo ${catkin_LIBRARIES})

add_executable(mutiple_thread_demo src/mutiple_thread.cpp)
target_link_libraries(mutiple_thread_demo ${catkin_LIBRARIES})

add_executable(spin_demo src/spin.cpp)
target_link_libraries(spin_demo ${catkin_LIBRARIES})

add_executable(duration src/duration.cpp)
target_link_libraries(duration ${catkin_LIBRARIES})

add_executable(sub src/sub.cpp)
target_link_libraries(sub ${catkin_LIBRARIES})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

add_executable(points_cnt_demo src/points_size_count.cpp)
target_link_libraries(points_cnt_demo ${catkin_LIBRARIES})

add_executable(message_filter_demo src/message_filter_demo.cpp)
target_link_libraries(message_filter_demo ${catkin_LIBRARIES})

add_executable(talker_for_message_filter_demo src/talker_for_message_filter_demo.cpp)
target_link_libraries(talker_for_message_filter_demo ${catkin_LIBRARIES})

add_executable(mutiple_param_sub_demo src/mutiple_param_sub_demo.cpp)
target_link_libraries(mutiple_param_sub_demo ${catkin_LIBRARIES})

add_executable(thread_num_demo src/thread_num.cpp)
target_link_libraries(thread_num_demo ${catkin_LIBRARIES})

file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/nodelet_demo.cpp include/*.h)
add_library(MyNodeClass ${srcs})
target_include_directories(MyNodeClass PUBLIC include)
target_link_libraries(MyNodeClass ${catkin_LIBRARIES})

add_executable(point_cloud_demo src/point_cloud_callack.cpp)
target_link_libraries(point_cloud_demo ${catkin_LIBRARIES})

add_executable(tf1_demo src/tf1_demo.cpp)
target_link_libraries(tf1_demo ${catkin_LIBRARIES})

add_executable(tf2_demo src/tf2_demo.cpp)
target_link_libraries(tf2_demo ${catkin_LIBRARIES})

