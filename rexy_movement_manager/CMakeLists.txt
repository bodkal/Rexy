cmake_minimum_required(VERSION 3.5)
project(rexy_movement_manager)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

include_directories(include ${YAML_CPP_LIBRARIES})
ament_export_include_directories(include)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rexy_msg REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(movement_manager src/movement_manager.cpp src/kinematics.cpp)
ament_target_dependencies(movement_manager rclcpp  rclpy std_msgs rexy_msg yaml-cpp)

target_link_libraries(movement_manager ${YAML_CPP_LIBRARIES})



target_include_directories(movement_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS movement_manager
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
