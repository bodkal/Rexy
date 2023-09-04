cmake_minimum_required(VERSION 3.5)
project(movement_manager)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

include_directories(include ${YAML_CPP_LIBRARIES})


find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rexy_msg REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(motor_control REQUIRED)

set(dependencies
rclcpp
std_msgs
geometry_msgs
sensor_msgs
tf2_ros
rexy_msg
tf2
yaml-cpp
motor_control
)


add_executable(movement_manager src/MovementManager.cpp)

ament_target_dependencies(movement_manager ${dependencies})

target_link_libraries(movement_manager 
                     ${YAML_CPP_LIBRARIES})

 target_include_directories(movement_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_export_include_directories(include)


install(TARGETS movement_manager
  DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY include/movement_manager
        DESTINATION include
        )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
