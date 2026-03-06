cmake_minimum_required(VERSION 3.8)
project(bloom_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(CURL REQUIRED)
find_package(ament_index_cpp REQUIRED)


# Build bloom_node library
add_library(bloom_node_lib
  src/state_manager.cpp
  src/web_service_client.cpp
  src/configuration_manager.cpp
  src/behavior_coordinator.cpp
  src/lesson_coordinator.cpp
  src/lesson_poller.cpp
  src/feedback_poller.cpp
  src/thread_pool.cpp
  src/curl_pool.cpp
)

target_include_directories(bloom_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(bloom_node_lib PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(bloom_node_lib rclcpp std_msgs std_srvs ament_index_cpp)
target_link_libraries(bloom_node_lib CURL::libcurl pthread)

# Build bloom_node executable
add_executable(bloom_node
  src/bloom_node.cpp
)

target_link_libraries(bloom_node bloom_node_lib)
ament_target_dependencies(bloom_node rclcpp)

install(TARGETS bloom_node_lib
  EXPORT bloom_nodeTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS bloom_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

ament_export_targets(bloom_nodeTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp std_msgs std_srvs CURL)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
