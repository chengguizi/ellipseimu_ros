cmake_minimum_required(VERSION 3.5)
project(ellipseimu_ros)

# enable C++11 standard (current directory scope)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON) # prevent fallback
# get_property(_variableNames GLOBAL PROPERTY CMAKE_CXX_STANDARD)
# message(WARNING "CMAKE_CXX_STANDARD=${CMAKE_CXX_STANDARD}")

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS
#  LIBRARIES rs2_ros
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

add_executable(ellipseimu_ros 
  src/main.cpp
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  ) # to find ros/ros.h
# alternatively and more specifically:
# target_include_directories(rs2_ros
#   PRIVATE
#     ${catkin_INCLUDE_DIRS}
# )

target_link_libraries(ellipseimu_ros
  PRIVATE
    ${catkin_LIBRARIES}
    ${CMAKE_SOURCE_DIR}/lib/libSbgECom.a
)

