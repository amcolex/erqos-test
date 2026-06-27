# Host build for the RO skid test suite.
#
# Compiles firmware/firmware.ino once (via sketch_tu.cpp) against the fake
# Arduino + EQSP32 headers in tests/fakes, then drives setup()/loop() from the
# test cases. No hardware, no vendor library.
cmake_minimum_required(VERSION 3.20)
project(ro_skid_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()
add_compile_options(-Wall -Wextra)

option(RO_COVERAGE "Build with coverage instrumentation" OFF)

find_package(doctest REQUIRED)

add_executable(ro_tests
  doctest_main.cpp
  sketch_tu.cpp           # compiles ../firmware/firmware.ino against the fakes
  fakes/fakes.cpp
  support/plant_model.cpp
  test_behaviors.cpp
  test_scenarios.cpp
)
# Order matters: tests/fakes must be found for <Arduino.h>/<EQSP32.h>.
target_include_directories(ro_tests PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/fakes
)
target_link_libraries(ro_tests PRIVATE doctest::doctest)

if(RO_COVERAGE)
  target_compile_options(ro_tests PRIVATE --coverage -O0 -g)
  target_link_options(ro_tests PRIVATE --coverage)
endif()

enable_testing()
add_test(NAME ro_tests COMMAND ro_tests)
