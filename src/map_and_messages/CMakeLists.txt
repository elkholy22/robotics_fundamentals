cmake_minimum_required(VERSION 3.8)
project(map_and_messages)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Cell.msg"
  "msg/Row.msg"
  "msg/Grid.msg"
  "msg/Pose.msg"
)

install(PROGRAMS
  publish_map.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  map.txt
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
