cmake_minimum_required(VERSION 3.10)
project(hand_identify_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -pthread")

list(APPEND CMAKE_PREFIX_PATH /opt/ros/noetic)
if(DEFINED ENV{HOME})
  list(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/sim2real/devel")
  list(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/sim2real/install")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

find_package(sim2real_msg QUIET)
if(sim2real_msg_FOUND)
  message(STATUS "sim2real_msg found — /joy_msg 动作库已启用")
else()
  message(WARNING "sim2real_msg 未找到，手势2~4 仅打印日志（请 source ~/sim2real/devel/setup.bash）")
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

file(GLOB SRC src/*.cpp)
add_executable(vision_controller ${SRC})

target_link_libraries(vision_controller
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  Threads::Threads
)

if(sim2real_msg_FOUND)
  target_link_libraries(vision_controller ${sim2real_msg_LIBRARIES})
  target_include_directories(vision_controller PRIVATE ${sim2real_msg_INCLUDE_DIRS})
  target_compile_definitions(vision_controller PRIVATE HAVE_SIM2REAL_MSG=1)
endif()

install(TARGETS vision_controller RUNTIME DESTINATION bin)
