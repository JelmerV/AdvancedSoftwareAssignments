cmake_minimum_required(VERSION 3.8)

########################################################################################################################
# Metadata
########################################################################################################################
# Read the package manifest.
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/package.xml" package_xml_str)

# Extract project name.
if(NOT package_xml_str MATCHES "<name>([A-Za-z0-9_]+)</name>")
  message(FATAL_ERROR "Could not parse project name from package manifest (aborting)")
else()
  set(extracted_name ${CMAKE_MATCH_1})
endif()

# Extract project version.
if(NOT package_xml_str MATCHES "<version>([0-9]+.[0-9]+.[0-9]+)</version>")
  message(FATAL_ERROR "Could not parse project version from package manifest (aborting)")
else()
  set(extracted_version ${CMAKE_MATCH_1})
endif()

########################################################################################################################
# CMake project
########################################################################################################################
project(${extracted_name} VERSION ${extracted_version})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(asdfr_interfaces REQUIRED)

include_directories(include)

add_executable(brightness_check src/brightness_check.cpp)
ament_target_dependencies(brightness_check rclcpp std_msgs sensor_msgs asdfr_interfaces)

add_executable(light_position src/light_position.cpp)
ament_target_dependencies(light_position rclcpp std_msgs sensor_msgs asdfr_interfaces)

add_executable(jiwy_trajectory src/jiwy_trajectory.cpp)
ament_target_dependencies(jiwy_trajectory rclcpp std_msgs sensor_msgs asdfr_interfaces)

# # Add library
# add_library(${PROJECT_NAME} SHARED
#   src/brightness_check.cpp
#   src/light_position.cpp
#   src/jiwy_trajectory.cpp
# )

# # Create nodes
# target_include_directories(${PROJECT_NAME} PUBLIC
#   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
# target_link_libraries(${PROJECT_NAME}
#   rclcpp::rclcpp
#   ${sensor_msgs_TARGETS}
#   ${std_msgs_TARGETS}
#   ${asdfr_interfaces_TARGETS}
#   rclcpp_components::component)
# rclcpp_components_register_node(${PROJECT_NAME} PLUGIN "BrightnessCheck" EXECUTABLE brightness_check)
# rclcpp_components_register_node(${PROJECT_NAME} PLUGIN "LightPosition" EXECUTABLE light_position)
# rclcpp_components_register_node(${PROJECT_NAME} PLUGIN "JiwyTrajectory" EXECUTABLE jiwy_trajectory)

# Install targets
install(
  TARGETS 
    brightness_check
    light_position
    jiwy_trajectory
    DESTINATION lib/${PROJECT_NAME}
  # TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
  # ARCHIVE DESTINATION lib
  # LIBRARY DESTINATION lib
  # RUNTIME DESTINATION bin
)

# Install include directory
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

# Install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Export stuff
# ament_export_libraries(${PROJECT_NAME})
# ament_export_targets(${PROJECT_NAME})
# ament_export_dependencies(rclcpp_components)

ament_package()
