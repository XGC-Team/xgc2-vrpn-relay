cmake_minimum_required(VERSION 3.10)
project(xgc2_vrpn_relay)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
)

catkin_package()

# The ROS-free core. Its own install rules put the library, the headers and a CMake config
# (find_package(xgc2_vrpn_relay_core CONFIG)) next to this package, for products that link it.
add_subdirectory(core)

add_executable(vrpn_relay src/vrpn_relay_node.cpp)
target_link_libraries(vrpn_relay PRIVATE xgc2_vrpn_relay_core ${catkin_LIBRARIES})
# ROS headers are system headers: only this package's own code is held to the core's warnings.
target_include_directories(vrpn_relay SYSTEM PRIVATE ${catkin_INCLUDE_DIRS})
get_target_property(xgc2_vrpn_relay_warnings xgc2_vrpn_relay_core COMPILE_OPTIONS)
if(xgc2_vrpn_relay_warnings)
  target_compile_options(vrpn_relay PRIVATE ${xgc2_vrpn_relay_warnings})
endif()
add_dependencies(vrpn_relay ${catkin_EXPORTED_TARGETS})

install(TARGETS vrpn_relay
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  catkin_add_gtest(${PROJECT_NAME}_core_test
    core/test/offset_test.cpp
    core/test/pose_relay_test.cpp
    core/test/quality_test.cpp
    core/test/types_test.cpp
    core/test/vision_cadence_test.cpp
  )
  target_link_libraries(${PROJECT_NAME}_core_test xgc2_vrpn_relay_core ${GTEST_MAIN_LIBRARIES})

  # The node against a fake tracker, under the master rostest starts.
  add_rostest_gtest(${PROJECT_NAME}_node_test
    test/vrpn_relay.test
    test/vrpn_relay_node_test.cpp
  )
  target_include_directories(${PROJECT_NAME}_node_test SYSTEM PRIVATE ${catkin_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME}_node_test ${catkin_LIBRARIES})
  add_dependencies(${PROJECT_NAME}_node_test vrpn_relay)
endif()
