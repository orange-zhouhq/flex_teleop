cmake_minimum_required(VERSION 3.16)
project(flex_teleop VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

add_library(flex_teleop_core
  core/src/pose_math.cpp
  core/src/interpolator.cpp
  core/src/joint_interpolator.cpp
  core/src/teleop_core.cpp
)
target_include_directories(flex_teleop_core PUBLIC core/include)
target_compile_options(flex_teleop_core PRIVATE -Wall -Wextra -Wpedantic)

enable_testing()
add_executable(test_interpolator tests/test_interpolator.cpp)
target_link_libraries(test_interpolator PRIVATE flex_teleop_core)
target_compile_options(test_interpolator PRIVATE -Wall -Wextra)
add_test(NAME interpolator COMMAND test_interpolator)

add_executable(test_teleop_core tests/test_teleop_core.cpp)
target_link_libraries(test_teleop_core PRIVATE flex_teleop_core)
target_compile_options(test_teleop_core PRIVATE -Wall -Wextra)
add_test(NAME teleop_core COMMAND test_teleop_core)

add_executable(flex_teleop_minimal examples/minimal.cpp)
target_link_libraries(flex_teleop_minimal PRIVATE flex_teleop_core)
target_compile_options(flex_teleop_minimal PRIVATE -Wall -Wextra)
