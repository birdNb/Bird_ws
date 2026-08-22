cmake_minimum_required(VERSION 2.8.3)
project(yesense_imu)

include(GNUInstallDirs)

find_package(Boost REQUIRED COMPONENTS chrono thread)
find_package(yaml-cpp REQUIRED)

# 以下添加第三方库，可选
# 安装了livelybot_robot,会安装以下库，选择find_package，否则选择add_subdirectory
# find_package(serial_cmake REQUIRED)
# find_package(lcm REQUIRED)

add_subdirectory(third_part/lcm)
add_subdirectory(third_part/serial_cmake)

find_library(SERIALPORT_LIBRARY serialport)
if(NOT SERIALPORT_LIBRARY)
  message(FATAL_ERROR "libserialport not found! Please install libserialport-dev.")
else()
  message(STATUS "Found libserialport library: ${SERIALPORT_LIBRARY}")
endif()


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include/yesense_imu
  ${Boost_INCLUDE_DIRS}
  msg/imu_msg
)

set(SERIAL_SRC
  src/yesense_driver.cpp
  src/analysis_data.cpp
)

# 定义配置文件安装路径
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
set(CONFIG_INSTALL_PATH "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/cfg/config.yaml")

add_library(${PROJECT_NAME} SHARED ${SERIAL_SRC})
target_link_libraries(${PROJECT_NAME} ${SERIALPORT_LIBRARY} ${Boost_LIBRARIES}  ${SERIAL_LIBRARY}  yaml-cpp lcm serial_cmake)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/msg>
  $<INSTALL_INTERFACE:include>
)

# 传递配置文件路径给编译器
target_compile_definitions(${PROJECT_NAME} PRIVATE
  CONFIG_INSTALL_PATH="${CONFIG_INSTALL_PATH}"
)

add_executable(yesense_node example/yesense_node.cpp)
target_link_libraries(yesense_node ${PROJECT_NAME})

add_executable(yesense_recv example/yesense_lcm_recv.cpp)
target_link_libraries(yesense_recv ${PROJECT_NAME})

#############
## Install ##
#############

# 安装库文件
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
)

# 安装 LCM 消息头文件
install(DIRECTORY msg/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp"
)

# 安装配置文件
install(DIRECTORY cfg/
  DESTINATION share/${PROJECT_NAME}/cfg
  FILES_MATCHING PATTERN "*.yaml"
)

# # 安装示例程序（可选）
# install(TARGETS yesense_node yesense_recv
#   RUNTIME DESTINATION bin
# )

# 生成并安装 CMake 配置文件
include(CMakePackageConfigHelpers)

# 设置包含目录变量
set(INCLUDE_INSTALL_DIR include)

# 生成版本文件
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  VERSION 1.0.0
  COMPATIBILITY AnyNewerVersion
)

# 导出目标
install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION lib/cmake/${PROJECT_NAME}
)

# 创建配置文件
configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
  PATH_VARS INCLUDE_INSTALL_DIR
)

# 安装配置文件
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION lib/cmake/${PROJECT_NAME}
)
