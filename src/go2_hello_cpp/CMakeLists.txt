cmake_minimum_required(VERSION 3.8)
project(go2_hello_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_api REQUIRED)

add_executable(hello_node src/hello_node.cpp)
ament_target_dependencies(hello_node rclcpp unitree_api)

install(TARGETS
  hello_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
