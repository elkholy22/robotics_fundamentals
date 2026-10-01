cmake_minimum_required(VERSION 3.8)
project(create_fundamentals)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorPacket.msg"
  "srv/ResetEncoders.srv"
  "srv/Leds.srv"
  "srv/DiffDrive.srv"
  "srv/StoreSong.srv"
  "srv/PlaySong.srv"
  DEPENDENCIES std_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

install(PROGRAMS
  src/driver.py
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(example_laser src/examples/example_laser.cpp)
ament_target_dependencies(example_laser rclcpp sensor_msgs)

add_executable(example_moving src/examples/example_moving.cpp)
ament_target_dependencies(example_moving rclcpp)

add_executable(example_song src/examples/example_song.cpp)
ament_target_dependencies(example_song rclcpp)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
target_link_libraries(example_moving "${cpp_typesupport_target}")
target_link_libraries(example_song "${cpp_typesupport_target}")

install(TARGETS
  example_laser
  example_moving
  example_song
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  src/irobot
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  laser_config.yaml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
