cmake_minimum_required(VERSION 2.8.3)
project(davis_stereo_stitch)

find_package(catkin_simple REQUIRED)

catkin_simple()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)


#########################
# make 
#########################

cs_add_executable(davis_stereo_stitch
  src/davis_stereo_stitch_node.cpp
  src/davis_stereo_stitch.cpp
)

# link the executable to the necesarry libs
target_link_libraries(davis_stereo_stitch
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

