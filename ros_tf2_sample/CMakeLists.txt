cmake_minimum_required(VERSION 3.0.2)
project(ros_tf2_sample)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

add_executable(sample1_tf2_connection src/sample1_tf2_connection.cpp)
target_link_libraries(sample1_tf2_connection  ${catkin_LIBRARIES} )

add_executable(sample2_tf2_moving_by_keyboard src/sample2_tf2_moving_by_keyboard.cpp)
target_link_libraries(sample2_tf2_moving_by_keyboard  ${catkin_LIBRARIES} )
