cmake_minimum_required(VERSION 3.8)
project(assign1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# make executables
add_executable(light_check src/light_check.cpp)
ament_target_dependencies(light_check rclcpp std_msgs sensor_msgs)

add_executable(light_position src/light_position.cpp)
ament_target_dependencies(light_position rclcpp std_msgs sensor_msgs)

# add executables
install(TARGETS
  light_check
  light_position
  DESTINATION lib/${PROJECT_NAME}
)

# install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
