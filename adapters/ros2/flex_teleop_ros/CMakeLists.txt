cmake_minimum_required(VERSION 3.8)
project(flex_teleop_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(flex_teleop_msgs REQUIRED)

# ── 核心库：源码内联编译，不要求预先安装 ────────────────────────────────
set(FLEX_TELEOP_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../..)

add_library(flex_teleop_core STATIC
  ${FLEX_TELEOP_ROOT}/core/src/pose_math.cpp
  ${FLEX_TELEOP_ROOT}/core/src/interpolator.cpp
  ${FLEX_TELEOP_ROOT}/core/src/joint_interpolator.cpp
  ${FLEX_TELEOP_ROOT}/core/src/teleop_core.cpp
)
target_include_directories(flex_teleop_core PUBLIC
  ${FLEX_TELEOP_ROOT}/core/include)

# ── 可选 QP 后端 ────────────────────────────────────────────────────────
option(FLEX_TELEOP_WITH_QP_IK "Build with reference QP IK backend" OFF)

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
    message(FATAL_ERROR "proxsuite 未找到")
  endif()

  add_library(flex_teleop_srs_dh_model STATIC
    ${FLEX_TELEOP_ROOT}/third_party/srs_dh_model/src/srs_dh_model.cpp)
  target_include_directories(flex_teleop_srs_dh_model PUBLIC
    ${FLEX_TELEOP_ROOT}/third_party/srs_dh_model/include)
  target_link_libraries(flex_teleop_srs_dh_model PUBLIC Eigen3::Eigen)

  add_library(flex_teleop_qp_ik STATIC
    ${FLEX_TELEOP_ROOT}/third_party/flex_qp_ik/src/flex_qp_ik.cpp
    ${FLEX_TELEOP_ROOT}/third_party/flex_qp_ik/src/qp_solver.cpp
    ${FLEX_TELEOP_ROOT}/third_party/flex_qp_ik/src/proxqp_solver.cpp
    ${FLEX_TELEOP_ROOT}/third_party/flex_qp_ik/src/arm_angle.cpp
    ${FLEX_TELEOP_ROOT}/plugins/qp_ik/src/qp_ik_backend.cpp)
  target_include_directories(flex_teleop_qp_ik
    PUBLIC ${FLEX_TELEOP_ROOT}/third_party/flex_qp_ik/include
           ${FLEX_TELEOP_ROOT}/plugins/qp_ik/include)
  target_link_libraries(flex_teleop_qp_ik PUBLIC
    flex_teleop_core flex_teleop_srs_dh_model Eigen3::Eigen proxsuite::proxsuite)
endif()

# ── 节点 ────────────────────────────────────────────────────────────────
add_executable(teleop_node src/teleop_node.cpp)
target_link_libraries(teleop_node flex_teleop_core)
ament_target_dependencies(teleop_node rclcpp builtin_interfaces flex_teleop_msgs)

if(FLEX_TELEOP_WITH_QP_IK)
  target_link_libraries(teleop_node flex_teleop_qp_ik)
  target_compile_definitions(teleop_node PRIVATE FLEX_TELEOP_WITH_QP_IK=1)
endif()

install(TARGETS teleop_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
