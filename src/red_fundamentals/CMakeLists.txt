# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 3.8)
project(red_fundamentals)

# Load dependencies required for this package (ROS 2)
find_package(ament_cmake REQUIRED)
# find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(create_fundamentals REQUIRED)
find_package(map_and_messages REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Set the build type.  Options are:
#  Coverage        : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug           : w/ debug symbols, w/o optimization
#  Release         : w/o debug symbols, w/ optimization
#  RelWithDebInfo  : w/ debug symbols, w/ optimization
#  MinSizeRel      : w/o debug symbols, w/ optimization, stripped binaries
## ROS 2 build type is configured via colcon; leave empty here.

find_package(rosidl_default_generators REQUIRED)
set(msg_files
  "msg/LineSegment.msg"
  "msg/LineSegmentArray.msg"
  "msg/Grid.msg"
  "msg/Row.msg"
  "msg/Cell.msg"
)
set(srv_files
  "srv/AddMove.srv"
  "srv/ClearMoveQueue.srv"
  "srv/AbortMove.srv"
  "srv/ExecutePlan.srv"
  "srv/MoveToPosition.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs map_and_messages
)

add_executable(square_with_encoders src/square_with_encoders.cpp)
target_include_directories(square_with_encoders PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
rosidl_target_interfaces(square_with_encoders
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
ament_target_dependencies(square_with_encoders rclcpp sensor_msgs create_fundamentals)

add_executable(movement_test src/movement_test.cpp)
target_include_directories(movement_test PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
rosidl_target_interfaces(movement_test
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
ament_target_dependencies(movement_test rclcpp sensor_msgs create_fundamentals)

add_executable(movement src/movement.cpp)
target_include_directories(movement PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
rosidl_target_interfaces(movement
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
ament_target_dependencies(movement rclcpp sensor_msgs create_fundamentals rosidl_default_runtime)

add_executable(conversion src/perception/conversion.cpp)
target_include_directories(conversion PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
rosidl_target_interfaces(conversion
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
ament_target_dependencies(conversion rclcpp sensor_msgs create_fundamentals rosidl_default_runtime tf2_ros tf2_sensor_msgs geometry_msgs)

add_executable(line_detection src/perception/line_detection.cpp)
target_include_directories(line_detection PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
rosidl_target_interfaces(line_detection
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
ament_target_dependencies(line_detection rclcpp sensor_msgs visualization_msgs geometry_msgs create_fundamentals rosidl_default_runtime)
target_link_libraries(line_detection Eigen3::Eigen)

add_executable(align src/align.cpp)
target_include_directories(align PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
rosidl_target_interfaces(align
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
ament_target_dependencies(align rclcpp visualization_msgs geometry_msgs create_fundamentals map_and_messages rosidl_default_runtime)
target_link_libraries(align Eigen3::Eigen)
add_executable(odometry src/odometry.cpp)
target_include_directories(odometry PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
ament_target_dependencies(odometry rclcpp nav_msgs geometry_msgs create_fundamentals tf2 tf2_ros)
add_executable(grid src/grid.cpp)
target_include_directories(grid PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
rosidl_target_interfaces(grid
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
ament_target_dependencies(grid rclcpp sensor_msgs create_fundamentals rosidl_default_runtime)
add_executable(map_rasterizer src/map_rasterizer.cpp)
target_include_directories(map_rasterizer PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
ament_target_dependencies(map_rasterizer
  rclcpp map_and_messages nav_msgs)
add_executable(particle_filter src/particle_filter.cpp)
target_include_directories(particle_filter PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
rosidl_target_interfaces(particle_filter
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
ament_target_dependencies(particle_filter
  rclcpp sensor_msgs nav_msgs geometry_msgs map_and_messages
  create_fundamentals tf2 tf2_ros rosidl_default_runtime)
add_executable(grid_fast src/grid_fast.cpp)
target_include_directories(grid_fast PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
rosidl_target_interfaces(grid_fast
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
ament_target_dependencies(grid_fast rclcpp sensor_msgs create_fundamentals std_msgs map_and_messages rosidl_default_runtime)
add_executable(localize src/localize.cpp)
target_include_directories(localize PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
rosidl_target_interfaces(localize
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
ament_target_dependencies(localize rclcpp visualization_msgs sensor_msgs create_fundamentals map_and_messages geometry_msgs std_msgs rosidl_default_runtime)
add_executable(move_to_position src/move_to_position.cpp)
target_include_directories(move_to_position PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
rosidl_target_interfaces(move_to_position
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
ament_target_dependencies(move_to_position rclcpp sensor_msgs create_fundamentals map_and_messages std_msgs rosidl_default_runtime)
add_executable(challenge src/challenge.cpp)
target_include_directories(challenge PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
rosidl_target_interfaces(challenge
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
ament_target_dependencies(challenge rclcpp create_fundamentals map_and_messages rosidl_default_runtime)
install(PROGRAMS
  ../map_and_messages/publish_map.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
	square_with_encoders
	movement
	conversion
	movement_test
	line_detection
	align
	grid
	grid_fast
	localize
	move_to_position
  challenge
  odometry
  map_rasterizer
  particle_filter
	DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
