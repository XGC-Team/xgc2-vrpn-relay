cmake_minimum_required(VERSION 3.10)
project(xgc2_vrpn_relay_core_downstream LANGUAGES CXX)

# What another product writes to use the relay logic: CMAKE_PREFIX_PATH points at the install
# prefix (/opt/ros/<distro>), nothing else is needed, and no ROS package is found or linked.
find_package(xgc2_vrpn_relay_core CONFIG REQUIRED)

add_executable(downstream_smoke main.cpp)
target_link_libraries(downstream_smoke PRIVATE xgc2_vrpn_relay_core::core)
