cmake_minimum_required(VERSION 3.8)
project(tortoisebot_waypoints)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/WaypointAction.action"
  DEPENDENCIES std_msgs  geometry_msgs nav_msgs# Add it when you use other dependency in the file 
)


## ADD LIBRARY
include_directories(include)
add_library(tortoisebot_waypoints_lib src/waypoint_action_server.cpp include/tortoisebot_waypoints/waypoint_action_server.h)
ament_target_dependencies(tortoisebot_waypoints_lib 
  rclcpp_action
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs)
rosidl_target_interfaces(tortoisebot_waypoints_lib ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_export_targets(tortoisebot_waypoints_lib HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/tortoisebot_waypoints
  DESTINATION include
)

## ADD EXECUTABLE
add_executable(main src/main.cpp)
target_link_libraries(main PUBLIC tortoisebot_waypoints_lib)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(std_srvs REQUIRED)

  set(TESTFILES
      test/main.cpp
      test/tortoisebot_waypoints_node_test.cpp
      )
  # Add a gtest executables
  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})
  target_link_libraries(${PROJECT_NAME}_test tortoisebot_waypoints_lib)
  ament_target_dependencies(${PROJECT_NAME}_test std_srvs)

  install(TARGETS
        ${PROJECT_NAME}_test
        # Install the generated test executable file to the path after DESTINATION
        DESTINATION lib/${PROJECT_NAME})
  # uncomment to enforce lintners and static code analyzers defined in ament_lint_common package        
  # ament_lint_auto_find_test_dependencies()
endif()


install(TARGETS tortoisebot_waypoints_lib
  DESTINATION lib/tortoisebot_waypoints_lib)

install(TARGETS 
  main
  DESTINATION lib/${PROJECT_NAME}/
)

install(
  TARGETS tortoisebot_waypoints_lib
  EXPORT tortoisebot_waypoints_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()