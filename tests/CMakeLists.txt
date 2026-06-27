# Host build for the RO skid control-core test suite.
#
# Compiles the platform-independent core (sensors / rgb_indicator /
# ro_controller) against FakeEqsp32 — never firmware.ino, which needs the EQSP32
# library. The core therefore must not include <Arduino.h>/<EQSP32.h>; if it
# ever did, this build would fail, which is exactly the guardrail we want.
cmake_minimum_required(VERSION 3.20)
project(ro_skid_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()
add_compile_options(-Wall -Wextra)

option(RO_COVERAGE "Build the control core with coverage instrumentation" OFF)

find_package(doctest REQUIRED)

set(CORE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../firmware/src)

add_library(ro_core STATIC
  ${CORE_DIR}/sensors.cpp
  ${CORE_DIR}/rgb_indicator.cpp
  ${CORE_DIR}/ro_controller.cpp
)
target_include_directories(ro_core PUBLIC ${CORE_DIR})

add_executable(ro_tests
  doctest_main.cpp
  support/plant_model.cpp
  test_sensors.cpp
  test_rgb.cpp
  test_controller_unit.cpp
  test_scenarios.cpp
)
target_include_directories(ro_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CORE_DIR})
target_link_libraries(ro_tests PRIVATE ro_core doctest::doctest)

if(RO_COVERAGE)
  target_compile_options(ro_core PRIVATE --coverage -O0 -g)
  target_link_options(ro_core PUBLIC --coverage)
  target_compile_options(ro_tests PRIVATE --coverage -O0 -g)
endif()

enable_testing()
add_test(NAME ro_tests COMMAND ro_tests)
