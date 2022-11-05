cmake_minimum_required(VERSION 3.5)
project(motor_control)

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


#include_directories(include ${YAML_CPP_LIBRARIES})

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rexy_msg REQUIRED)
find_package(tf2 REQUIRED)
find_package(yaml-cpp REQUIRED)

set(dependencies
std_msgs
rexy_msg
tf2
yaml-cpp
)

add_library(motor_control src/LegControl.cpp include/JHPWMDriver/src/JHPWMPCA9685.cpp)

target_include_directories(motor_control PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        )

ament_target_dependencies(motor_control ${dependencies})

target_link_libraries(motor_control ${YAML_CPP_LIBRARIES} i2c)



ament_export_targets(motor_control HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})
ament_export_include_directories(include)


install(DIRECTORY include/motor_control
        DESTINATION include
        )

install(DIRECTORY include/JHPWMDriver
        DESTINATION include
        )

install(
        TARGETS motor_control
        EXPORT motor_control
        LIBRARY DESTINATION lib/${PROJECT_NAME}
        ARCHIVE DESTINATION lib/${PROJECT_NAME}
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
