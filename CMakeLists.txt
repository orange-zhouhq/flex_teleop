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

# ── 参考 IK 后端（可选）：S-R-S 速度级 QP ────────────────────────────────
# 依赖 Eigen3 与 proxsuite（纯头文件）。关闭时核心库不受影响。
option(FLEX_TELEOP_WITH_QP_IK "Build reference QP IK backend (Eigen + proxsuite)" OFF)

if(FLEX_TELEOP_WITH_QP_IK)
  find_package(Eigen3 3.3 REQUIRED NO_MODULE)

  find_package(proxsuite QUIET)
  if(NOT proxsuite_FOUND)
    file(GLOB _ROS_INCLUDE_DIRS "/opt/ros/*/include")
    find_path(PROXSUITE_INCLUDE_DIR proxsuite/config.hpp
      PATHS ${_ROS_INCLUDE_DIRS} NO_DEFAULT_PATH)
    if(PROXSUITE_INCLUDE_DIR)
      add_library(proxsuite::proxsuite INTERFACE IMPORTED)
      target_include_directories(proxsuite::proxsuite INTERFACE ${PROXSUITE_INCLUDE_DIR})
      set(proxsuite_FOUND TRUE)
    endif()
  endif()
  if(NOT proxsuite_FOUND)
    message(FATAL_ERROR "proxsuite 未找到：请安装 proxsuite 或 source ROS 环境")
  endif()

  add_library(flex_teleop_srs_dh_model STATIC
    third_party/srs_dh_model/src/srs_dh_model.cpp
  )
  target_include_directories(flex_teleop_srs_dh_model PUBLIC
    third_party/srs_dh_model/include)
  target_link_libraries(flex_teleop_srs_dh_model PUBLIC Eigen3::Eigen)
  target_compile_options(flex_teleop_srs_dh_model PRIVATE -O2)

  add_library(flex_teleop_qp_ik STATIC
    third_party/flex_qp_ik/src/flex_qp_ik.cpp
    third_party/flex_qp_ik/src/qp_solver.cpp
    third_party/flex_qp_ik/src/proxqp_solver.cpp
    third_party/flex_qp_ik/src/arm_angle.cpp
    plugins/qp_ik/src/qp_ik_backend.cpp
  )
  target_include_directories(flex_teleop_qp_ik
    PUBLIC third_party/flex_qp_ik/include plugins/qp_ik/include
    PRIVATE third_party/srs_dh_model/include)
  target_link_libraries(flex_teleop_qp_ik PUBLIC
    flex_teleop_core flex_teleop_srs_dh_model Eigen3::Eigen proxsuite::proxsuite)
  target_compile_options(flex_teleop_qp_ik PRIVATE -O2 -Wall -Wextra)

  enable_testing()
  add_executable(test_qp_backend tests/test_qp_backend.cpp)
  target_link_libraries(test_qp_backend PRIVATE flex_teleop_qp_ik)
  target_compile_options(test_qp_backend PRIVATE -Wall -Wextra)
  add_test(NAME qp_backend COMMAND test_qp_backend)

  message(STATUS "flex_teleop: QP IK backend enabled")
endif()
