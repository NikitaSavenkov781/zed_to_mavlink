cmake_minimum_required(VERSION 3.8)
project(zed_to_mavlink)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(MAVSDK REQUIRED IMPORTED_TARGET mavsdk)

add_executable(zed_to_mavlink_node src/zed_to_mavlink_node.cpp)
target_compile_features(zed_to_mavlink_node PUBLIC cxx_std_17)

ament_target_dependencies(zed_to_mavlink_node
  rclcpp
  geometry_msgs
  nav_msgs
)

target_link_libraries(zed_to_mavlink_node
  PkgConfig::MAVSDK
)

install(TARGETS zed_to_mavlink_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
