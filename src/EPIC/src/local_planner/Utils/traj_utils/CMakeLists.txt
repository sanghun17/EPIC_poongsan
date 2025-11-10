cmake_minimum_required(VERSION 2.8.3)
project(traj_utils)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

# ARM 크로스컴파일 시 호환성 문제 해결
if(CMAKE_CROSSCOMPILING AND CMAKE_SYSROOT)
  # OpenCV/PCL I/O 함수 문제 해결
  add_definitions(-D_GNU_SOURCE)
  add_definitions(-include cstdio)
  add_definitions(-include unistd.h)
  
  # 컴파일러 플래그 추가
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive -Wno-error=cast-function-type")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_GNU_SOURCE")
  
  message(STATUS "✅ traj_utils: ARM compatibility flags added")
endif()

# ✅ ARM 컴파일 시 catkin find_package 의존성 조정
if(CMAKE_CROSSCOMPILING AND CMAKE_SYSROOT)
find_package(catkin REQUIRED COMPONENTS
    path_searching
    poly_traj
    roscpp
    std_msgs
    cv_bridge
    message_generation
)
  message(STATUS "✅ traj_utils: ARM find_package")
  
  # ARM sysroot에서 ROS 라이브러리 직접 찾기
  find_library(ARM_ROSCPP_LIB NAMES roscpp PATHS ${CMAKE_SYSROOT}/opt/ros/noetic/lib NO_DEFAULT_PATH)
  find_library(ARM_ROSCPP_SERIALIZATION_LIB NAMES roscpp_serialization PATHS ${CMAKE_SYSROOT}/opt/ros/noetic/lib NO_DEFAULT_PATH)
  find_library(ARM_ROSTIME_LIB NAMES rostime PATHS ${CMAKE_SYSROOT}/opt/ros/noetic/lib NO_DEFAULT_PATH)
  find_library(ARM_CPP_COMMON_LIB NAMES cpp_common PATHS ${CMAKE_SYSROOT}/opt/ros/noetic/lib NO_DEFAULT_PATH)
  find_library(ARM_ROSCONSOLE_LIB NAMES rosconsole PATHS ${CMAKE_SYSROOT}/opt/ros/noetic/lib NO_DEFAULT_PATH)
  find_library(ARM_CV_BRIDGE_LIB NAMES cv_bridge PATHS ${CMAKE_SYSROOT}/opt/ros/noetic/lib NO_DEFAULT_PATH)
  
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
    
    if(ARM_CV_BRIDGE_LIB)
      add_library(arm_cv_bridge SHARED IMPORTED)
      set_target_properties(arm_cv_bridge PROPERTIES IMPORTED_LOCATION ${ARM_CV_BRIDGE_LIB})
    endif()
    
    message(STATUS "✅ traj_utils: ARM IMPORTED targets created")
  endif()
  
  # catkin_LIBRARIES 무효화
  set(catkin_LIBRARIES "")
else()
  find_package(catkin REQUIRED COMPONENTS
    path_searching
    poly_traj
    roscpp
    std_msgs
    cv_bridge
    message_generation
  )
  message(STATUS "📍 traj_utils: Host find_package")
endif()

find_package(Eigen3 REQUIRED)

# PCL 찾기 (ARM sysroot 우선)
if(CMAKE_CROSSCOMPILING)
  find_package(PCL REQUIRED PATHS ${CMAKE_SYSROOT}/usr NO_DEFAULT_PATH)
else()
find_package(PCL 1.7 REQUIRED)
endif()

add_message_files(
    FILES
    PolyTraj.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# ✅ ARM 크로스 컴파일 시 catkin_package 조정
if(CMAKE_CROSSCOMPILING AND CMAKE_SYSROOT)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES traj_utils
    # CATKIN_DEPENDS 제거 - ARM 컴파일 시 의존성 자동 링크 방지
  )
  message(STATUS "✅ traj_utils: ARM catkin_package")
else()
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES traj_utils
    CATKIN_DEPENDS path_searching poly_traj message_runtime
)
  message(STATUS "📍 traj_utils: Host catkin_package")
endif()

include_directories(
    SYSTEM
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

if(CMAKE_CROSSCOMPILING)
  # ARM sysroot 헤더를 최우선으로 설정 (BEFORE SYSTEM)
  include_directories(BEFORE SYSTEM
    ${CMAKE_SYSROOT}/usr/include/pcl-1.14
    ${CMAKE_SYSROOT}/usr/include/opencv4
    ${CMAKE_SYSROOT}/usr/include/eigen3
    ${CMAKE_SYSROOT}/opt/ros/noetic/include
  )
  
  message(STATUS "✅ traj_utils: ARM sysroot headers prioritized")
endif()

link_directories(${PCL_LIBRARY_DIRS})

add_library(traj_utils
    src/planning_visualization.cpp
)

# ✅ ARM 크로스 컴파일 시 IMPORTED targets 사용
if(CMAKE_CROSSCOMPILING AND CMAKE_SYSROOT AND ARM_ROSCPP_LIB)
  target_link_libraries(traj_utils
    arm_roscpp
    arm_roscpp_serialization
    arm_rostime
    arm_cpp_common
    arm_rosconsole
  )
  
  if(ARM_CV_BRIDGE_LIB)
    target_link_libraries(traj_utils arm_cv_bridge)
  endif()
  
  message(STATUS "✅ traj_utils: Using ARM IMPORTED targets")
else()
target_link_libraries(traj_utils
    ${catkin_LIBRARIES}
)
  message(STATUS "📍 traj_utils: Using host catkin_LIBRARIES")
endif()

add_executable(process_msg
    src/process_msg.cpp
)

# ✅ ARM 크로스 컴파일 시 IMPORTED targets 사용
if(CMAKE_CROSSCOMPILING AND CMAKE_SYSROOT AND ARM_ROSCPP_LIB)
  target_link_libraries(process_msg
    arm_roscpp
    arm_roscpp_serialization
    arm_rostime
    arm_cpp_common
    arm_rosconsole
    ${PCL_LIBRARIES}
  )
  
  message(STATUS "✅ process_msg: Using ARM IMPORTED targets")
else()
target_link_libraries(process_msg
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)
  message(STATUS "📍 process_msg: Using host catkin_LIBRARIES")
endif()