cmake_minimum_required(VERSION 3.24)
project(radar_fusion)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -DNDEBUG")

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(radar_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(${PROJECT_NAME}_node
    src/runtime.cpp
    src/fusion_node.cpp
    src/kalman_tracker.cpp
)
target_link_libraries(${PROJECT_NAME}_node
    Eigen3::Eigen
)
ament_target_dependencies(${PROJECT_NAME}_node radar_interfaces)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(radar_fusion_tests
      test/test_fusion_node.cpp
      src/fusion_node.cpp
      src/kalman_tracker.cpp
  )
  target_include_directories(radar_fusion_tests PRIVATE include)
  ament_target_dependencies(radar_fusion_tests
      rclcpp
      geometry_msgs
      radar_interfaces
      visualization_msgs
      tf2_geometry_msgs
  )
  target_link_libraries(radar_fusion_tests
      Eigen3::Eigen
  )
endif()

install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})
ament_package()
