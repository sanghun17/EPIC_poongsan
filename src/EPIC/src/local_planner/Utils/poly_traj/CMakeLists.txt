cmake_minimum_required(VERSION 3.0.2)
project(poly_traj)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(Eigen3 REQUIRED)

# ARM 전용 - 크로스컴파일만 지원
find_package(catkin REQUIRED COMPONENTS
roscpp
std_msgs
  geometry_msgs
  swarmtal_msgs
  visualization_msgs
  nav_msgs
  # decomp_util 제거 - 원본에 없는 의존성
)

# ARM sysroot에서 ROS 라이브러리 직접 찾기
find_library(ARM_ROSCPP_LIB NAMES roscpp PATHS ${CMAKE_SYSROOT}/opt/ros/noetic/lib NO_DEFAULT_PATH)
find_library(ARM_ROSCPP_SERIALIZATION_LIB NAMES roscpp_serialization PATHS ${CMAKE_SYSROOT}/opt/ros/noetic/lib NO_DEFAULT_PATH)
find_library(ARM_ROSTIME_LIB NAMES rostime PATHS ${CMAKE_SYSROOT}/opt/ros/noetic/lib NO_DEFAULT_PATH)
find_library(ARM_CPP_COMMON_LIB NAMES cpp_common PATHS ${CMAKE_SYSROOT}/opt/ros/noetic/lib NO_DEFAULT_PATH)
find_library(ARM_ROSCONSOLE_LIB NAMES rosconsole PATHS ${CMAKE_SYSROOT}/opt/ros/noetic/lib NO_DEFAULT_PATH)

# IMPORTED 타겟 생성
if(ARM_ROSCPP_LIB)
  add_library(arm_roscpp SHARED IMPORTED)
  set_target_properties(arm_roscpp PROPERTIES IMPORTED_LOCATION ${ARM_ROSCPP_LIB})
  
  add_library(arm_roscpp_serialization SHARED IMPORTED)
  set_target_properties(arm_roscpp_serialization PROPERTIES IMPORTED_LOCATION ${ARM_ROSCPP_SERIALIZATION_LIB})
  
  add_library(arm_rostime SHARED IMPORTED)
  set_target_properties(arm_rostime PROPERTIES IMPORTED_LOCATION ${ARM_ROSTIME_LIB})
  
  add_library(arm_cpp_common SHARED IMPORTED)
  set_target_properties(arm_cpp_common PROPERTIES IMPORTED_LOCATION ${ARM_CPP_COMMON_LIB})
  
  add_library(arm_rosconsole SHARED IMPORTED)
  set_target_properties(arm_rosconsole PROPERTIES IMPORTED_LOCATION ${ARM_ROSCONSOLE_LIB})
endif()

# catkin_LIBRARIES 무효화
set(catkin_LIBRARIES "")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES poly_traj
  # CATKIN_DEPENDS 제거 - ARM 컴파일 시 의존성 자동 링크 방지
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  /usr/include/eigen3
  /opt/nxp-real-time-edge/2.5/sysroots/armv8a-poky-linux/usr/include/eigen3
)

# ARM 전용 - 모든 타겟 빌드
add_library(poly_traj src/polynomial_traj.cpp)

if(ARM_ROSCPP_LIB)
  target_link_libraries(poly_traj
    arm_roscpp
    arm_roscpp_serialization
    arm_rostime
    arm_cpp_common
  )
else()
  target_link_libraries(poly_traj ${catkin_LIBRARIES})
endif()

# traj_generator는 테스트/예제 코드이므로 ARM 크로스컴파일에서는 제외
if(NOT ARM_ROSCPP_LIB)
  add_executable(traj_generator
    src/traj_generator.cpp
  )

  target_link_libraries(traj_generator 
    ${catkin_LIBRARIES}
    poly_traj
  )
endif()
