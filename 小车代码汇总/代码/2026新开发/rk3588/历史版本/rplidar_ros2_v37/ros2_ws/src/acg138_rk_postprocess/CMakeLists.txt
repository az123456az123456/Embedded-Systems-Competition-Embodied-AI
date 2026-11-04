cmake_minimum_required(VERSION 3.16)
project(acg138_rk_postprocess LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# The core library contains all mapping, smoothing, and aggregation logic and
# deliberately has no ROS dependency so it can be tested on a host machine.
add_library(acg138_rk_postprocess_core src/postprocess.cpp)
target_include_directories(acg138_rk_postprocess_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(acg138_rk_postprocess_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(rk_postprocess_node src/postprocess_node.cpp)
target_include_directories(rk_postprocess_node PRIVATE include)
ament_target_dependencies(rk_postprocess_node rclcpp nav_msgs geometry_msgs std_msgs)
target_link_libraries(rk_postprocess_node PRIVATE acg138_rk_postprocess_core)
target_compile_options(rk_postprocess_node PRIVATE -Wall -Wextra -Wpedantic)

install(
  TARGETS acg138_rk_postprocess_core rk_postprocess_node
  EXPORT export_acg138_rk_postprocess
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch" OR EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME} OPTIONAL)
endif()

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  enable_testing()
  add_test(
    NAME rk_postprocess_host
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/run_host_tests.py
  )
endif()

ament_export_targets(export_acg138_rk_postprocess HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()
