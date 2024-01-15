cmake_minimum_required(VERSION 3.5)
project(template_drone_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

# Find Python libraries (specify the version you need, e.g., Python3)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
include_directories(${Python3_INCLUDE_DIRS})

# Declare a C++ executable
add_executable(template_drone_control_node src/template_drone_control_node.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(template_drone_control_node ${Python3_LIBRARIES})
ament_target_dependencies(template_drone_control_node rclcpp geometry_msgs mavros_msgs)

# Install the executable
install(TARGETS template_drone_control_node DESTINATION lib/${PROJECT_NAME})

# Ament package
ament_package()

