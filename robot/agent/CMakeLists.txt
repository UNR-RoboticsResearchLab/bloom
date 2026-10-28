# robot-agent: outbound remote-shell tunnel for Bloom robots. Plain CMake, deliberately not a colcon
# package (see COLCON_IGNORE): it must keep working when the ROS 2 stack is broken.
cmake_minimum_required(VERSION 3.22)
project(robot_agent LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# Version comes from Nerdbank.GitVersioning, same as the backend: CI runs `nbgv get-version` and
# passes the result in. Local builds without it are stamped 0.0.0-local so they're never mistaken
# for a release.
set(AGENT_VERSION "" CACHE STRING "SemVer 2.0 version (nbgv get-version -v SemVer2)")
set(AGENT_COMMIT "" CACHE STRING "Git commit (nbgv get-version -v GitCommitId)")
if(AGENT_COMMIT STREQUAL "")
  execute_process(COMMAND git rev-parse HEAD
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE AGENT_COMMIT OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
endif()
if(AGENT_VERSION STREQUAL "")
  string(SUBSTRING "${AGENT_COMMIT}" 0 10 _short_commit)
  set(AGENT_VERSION "0.0.0-local")
  if(_short_commit)
    set(AGENT_VERSION "0.0.0-local+${_short_commit}")
  endif()
endif()
configure_file(src/version.h.in ${CMAKE_CURRENT_BINARY_DIR}/generated/version.h @ONLY)

find_package(Boost 1.81 REQUIRED)
find_package(OpenSSL 3 REQUIRED)
find_package(Threads REQUIRED)

# nlohmann/json: system package if installed, else the copy bloom_node already vendors.
find_package(nlohmann_json 3 QUIET)
if(NOT nlohmann_json_FOUND)
  add_library(nlohmann_json INTERFACE)
  add_library(nlohmann_json::nlohmann_json ALIAS nlohmann_json)
  set(_vendored ${CMAKE_CURRENT_BINARY_DIR}/vendor/nlohmann)
  file(MAKE_DIRECTORY ${_vendored})
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/../src/bloom_node/include/bloom_node/json.hpp
    ${_vendored}/json.hpp COPYONLY)
  target_include_directories(nlohmann_json INTERFACE ${CMAKE_CURRENT_BINARY_DIR}/vendor)
endif()

# Everything except main.cpp, so the tests link the same code.
add_library(agent_core STATIC
  src/config.cpp
  src/doctor.cpp
  src/log.cpp
  src/net.cpp
  src/protocol.cpp
  src/proxy.cpp
  src/pty_process.cpp
  src/sessions.cpp
  src/supervisor.cpp
  src/tunnel.cpp
)
target_include_directories(agent_core PUBLIC src ${CMAKE_CURRENT_BINARY_DIR}/generated)
target_compile_definitions(agent_core PUBLIC BOOST_ASIO_NO_DEPRECATED)
target_compile_options(agent_core PRIVATE -Wall -Wextra)
target_link_libraries(agent_core PUBLIC
  Boost::headers OpenSSL::SSL OpenSSL::Crypto Threads::Threads nlohmann_json::nlohmann_json util)

add_executable(robot-agent src/main.cpp)
target_link_libraries(robot-agent PRIVATE agent_core)
# Fewer runtime dependencies on the robot; OpenSSL stays dynamic so it gets security updates via apt.
target_link_options(robot-agent PRIVATE -static-libstdc++ -static-libgcc)
install(TARGETS robot-agent RUNTIME DESTINATION bin)

option(AGENT_BUILD_TESTS "Build unit tests" ON)
if(AGENT_BUILD_TESTS)
  find_package(GTest REQUIRED)
  enable_testing()
  add_executable(agent_tests
    tests/backoff_test.cpp
    tests/liveness_test.cpp
    tests/protocol_test.cpp
    tests/proxy_test.cpp
    tests/pty_test.cpp
    tests/semver_test.cpp
    tests/sessions_test.cpp
    tests/supervisor_test.cpp
  )
  target_link_libraries(agent_tests PRIVATE agent_core GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(agent_tests)
endif()
