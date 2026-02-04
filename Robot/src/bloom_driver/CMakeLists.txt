cmake_minimum_required(VERSION 3.8)
<<<<<<<< HEAD:Robot/src/bloom_interface/CMakeLists.txt
project(bloom_interface)
========
project(bloom_driver)
>>>>>>>> 90ada47e1541badd97520020a789842874330999:Robot/src/bloom_driver/CMakeLists.txt

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(CURL REQUIRED)
find_package(bloom_node REQUIRED)

# Include directories
include_directories(include)

# Build robot_state_controller library
add_library(robot_state_controller
  src/robot_state_controller.cpp
)

# Add dependencies to robot_state_controller
target_include_directories(robot_state_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(robot_state_controller
  rclcpp
  std_msgs
  std_srvs
  bloom_node
)

# Link against uuid library
target_link_libraries(robot_state_controller uuid)

# Build bloom_agent executable
add_executable(bloom_agent
  bloom_agent.cpp
)

target_link_libraries(bloom_agent
  robot_state_controller
)

ament_target_dependencies(bloom_agent
  rclcpp
  bloom_node
  CURL
)

# Install targets
install(TARGETS
  robot_state_controller
  bloom_agent
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
