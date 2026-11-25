cmake_minimum_required(VERSION 3.8)
project(eltronico)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

add_executable(main
  src/main.cpp
  src/eltronico/Eltronico.cpp
)
target_include_directories(main PRIVATE include)
target_compile_features(main PRIVATE cxx_std_17)
ament_target_dependencies(main
  rclcpp rclcpp_lifecycle sensor_msgs nav_msgs std_msgs ros_gz_interfaces)

install(TARGETS main DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config urdf meshes worlds
  DESTINATION share/${PROJECT_NAME})

ament_package()
