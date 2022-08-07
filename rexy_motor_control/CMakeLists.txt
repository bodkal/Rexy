cmake_minimum_required(VERSION 3.5)
project(rexy_motor_control)

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

add_executable(motor_control src/motor_control.cpp  include/JHPWMDriver/src/JHPWMPCA9685.cpp include/motor_control.h)
ament_target_dependencies(motor_control rclcpp  rclpy std_msgs rexy_msg yaml-cpp)
target_link_libraries(motor_control ${catkin_LIBRARIES} i2c)
target_link_libraries(motor_control ${YAML_CPP_LIBRARIES})

      
install(TARGETS
  motor_control
  DESTINATION lib/${PROJECT_NAME})



# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
