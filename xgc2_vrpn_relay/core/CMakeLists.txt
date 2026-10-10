cmake_minimum_required(VERSION 3.10)

# The core takes its version from the ROS package so the two cannot drift apart.
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/../package.xml" xgc2_vrpn_relay_package_xml)
string(REGEX MATCH "<version>([0-9]+\\.[0-9]+\\.[0-9]+)</version>" _unused
       "${xgc2_vrpn_relay_package_xml}")
if(NOT CMAKE_MATCH_1)
  message(FATAL_ERROR "cannot read the package version from ../package.xml")
endif()
set(xgc2_vrpn_relay_core_version "${CMAKE_MATCH_1}")

# ROS-free core of the VRPN relay: the pose quality gate, the world-frame offset and the vision
# publish cadence. Nothing here includes or links ROS, so this directory also builds on its
# own (cmake -S core) and from catkin, which adds it as a subdirectory.
project(xgc2_vrpn_relay_core VERSION ${xgc2_vrpn_relay_core_version} LANGUAGES CXX)

include(CMakePackageConfigHelpers)

# CI builds with this on; a compiler upgrade must not break a user's build over a new warning.
option(XGC2_VRPN_RELAY_WERROR "Treat warnings in the code of this package as errors" OFF)

add_library(xgc2_vrpn_relay_core STATIC
  src/offset.cpp
  src/pose_relay.cpp
  src/quality.cpp
  src/types.cpp
  src/vision_cadence.cpp
)
add_library(xgc2_vrpn_relay_core::core ALIAS xgc2_vrpn_relay_core)
# Static so a product that links it carries no run-time dependency on this package. Position
# independent so that product may be a shared module.
set_target_properties(xgc2_vrpn_relay_core PROPERTIES
  EXPORT_NAME core
  POSITION_INDEPENDENT_CODE ON
)
target_compile_features(xgc2_vrpn_relay_core PUBLIC cxx_std_17)
target_include_directories(xgc2_vrpn_relay_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  set(xgc2_vrpn_relay_warnings -Wall -Wextra -Wpedantic)
  if(XGC2_VRPN_RELAY_WERROR)
    list(APPEND xgc2_vrpn_relay_warnings -Werror)
  endif()
  target_compile_options(xgc2_vrpn_relay_core PRIVATE ${xgc2_vrpn_relay_warnings})
endif()

install(TARGETS xgc2_vrpn_relay_core
  EXPORT xgc2_vrpn_relay_coreTargets
  ARCHIVE DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)
install(EXPORT xgc2_vrpn_relay_coreTargets
  NAMESPACE xgc2_vrpn_relay_core::
  DESTINATION share/cmake/xgc2_vrpn_relay_core
)
configure_package_config_file(cmake/xgc2_vrpn_relay_coreConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/xgc2_vrpn_relay_coreConfig.cmake
  INSTALL_DESTINATION share/cmake/xgc2_vrpn_relay_core
)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/xgc2_vrpn_relay_coreConfigVersion.cmake
  COMPATIBILITY SameMajorVersion
)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/xgc2_vrpn_relay_coreConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/xgc2_vrpn_relay_coreConfigVersion.cmake
  DESTINATION share/cmake/xgc2_vrpn_relay_core
)

# The unit tests run under catkin as part of the ROS package (see ../CMakeLists.txt). This option
# runs the same sources without ROS or catkin.
option(XGC2_VRPN_RELAY_CORE_BUILD_TESTING "Build the ROS-free core unit tests" OFF)
if(XGC2_VRPN_RELAY_CORE_BUILD_TESTING)
  enable_testing()
  find_package(GTest REQUIRED)
  add_executable(xgc2_vrpn_relay_core_test
    test/offset_test.cpp
    test/pose_relay_test.cpp
    test/quality_test.cpp
    test/types_test.cpp
    test/vision_cadence_test.cpp
  )
  target_link_libraries(xgc2_vrpn_relay_core_test PRIVATE
    xgc2_vrpn_relay_core
    GTest::GTest
    GTest::Main
    pthread
  )
  add_test(NAME xgc2_vrpn_relay_core_test COMMAND xgc2_vrpn_relay_core_test)
endif()
