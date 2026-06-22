cmake_minimum_required(VERSION 3.10)
project(locate_face_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -pthread")

list(APPEND CMAKE_PREFIX_PATH /opt/ros/noetic)
if(DEFINED ENV{HOME})
  list(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/sim2real/devel")
  list(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/sim2real/install")
endif()

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

set(SRC
  src/main.cpp
  src/Camera.cpp
  src/FsmMonitor.cpp
  src/FaceTracker.cpp
  src/MediaPipeFaceBridge.cpp
)

add_executable(locate_face ${SRC})
target_link_libraries(locate_face ${catkin_LIBRARIES} ${OpenCV_LIBS} Threads::Threads)

install(TARGETS locate_face RUNTIME DESTINATION bin)
