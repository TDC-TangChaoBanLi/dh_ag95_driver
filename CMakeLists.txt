cmake_minimum_required(VERSION 3.10)
project(dh_ag95_core_driver VERSION 0.1.0 LANGUAGES CXX)

option(DH_AG95_BUILD_EXAMPLES "Build examples" ON)

set(DH_AG95_CORE_SOURCES
  src/protocol.cpp
  src/ag95_gripper.cpp
  src/serial_port.cpp
  src/official_serial_transport.cpp
  src/slcan_transport.cpp
  src/pcanbasic_transport.cpp
  src/modbus_transport.cpp
)

if(NOT WIN32)
  list(APPEND DH_AG95_CORE_SOURCES src/socketcan_transport.cpp)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_library(dh_ag95_core ${DH_AG95_CORE_SOURCES})
add_library(dh_ag95::dh_ag95_core ALIAS dh_ag95_core)

target_compile_features(dh_ag95_core PUBLIC cxx_std_17)
target_include_directories(dh_ag95_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

if(MSVC)
  target_compile_options(dh_ag95_core PRIVATE /W4)
else()
  target_compile_options(dh_ag95_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(DH_AG95_BUILD_EXAMPLES)
  add_executable(ag95_cpp_example examples/cpp_example.cpp)
  target_link_libraries(ag95_cpp_example PRIVATE dh_ag95_core)
endif()

install(TARGETS dh_ag95_core
  EXPORT dh_ag95_coreTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(EXPORT dh_ag95_coreTargets
  FILE dh_ag95_coreTargets.cmake
  NAMESPACE dh_ag95::
  DESTINATION lib/cmake/dh_ag95_core
)
