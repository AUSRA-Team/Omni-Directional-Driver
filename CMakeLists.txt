cmake_minimum_required(VERSION 3.8)
project(omnidirectional_driver_cpp)

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
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

#### LIB (Math Logic) ####
add_library(omni_kinematics SHARED)

target_sources(omni_kinematics
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/omni_kinematics.cpp
)
target_include_directories(omni_kinematics
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Explicitly link Eigen to the math library
target_link_libraries(omni_kinematics Eigen3::Eigen)

#### Driver node (Executable) ####
# Changed to add_executable because it contains main()
add_executable(omni_driver
  src/omnidirectional_driver.cpp
)

target_include_directories(omni_driver
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# CRITICAL FIX: Explicitly list dependencies here
ament_target_dependencies(omni_driver
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(omni_driver
  omni_kinematics
)

# Install rules
install(TARGETS 
  omni_driver
  omni_kinematics
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  omni_kinematics
)

ament_package()