cmake_minimum_required(VERSION 2.8.3)
project(path_searching)

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
  
  message(STATUS "✅ path_searching: ARM compatibility flags added")
endif()

find_package(Eigen3 REQUIRED)

# ✅ ARM 컴파일 시 catkin find_package 의존성 조정
if(CMAKE_CROSSCOMPILING AND CMAKE_SYSROOT)
  # PCL 찾기 (ARM sysroot 우선)
  find_package(PCL REQUIRED PATHS ${CMAKE_SYSROOT}/usr NO_DEFAULT_PATH)
  
  find_package(catkin REQUIRED COMPONENTS
      roscpp
      rospy
      std_msgs
      visualization_msgs
      pointcloud_topo
      cv_bridge
      lidar_map
  )
  message(STATUS "✅ path_searching: ARM find_package")
  
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
    
    message(STATUS "✅ path_searching: ARM IMPORTED targets created")
  endif()
  
  # catkin_LIBRARIES 무효화
  set(catkin_LIBRARIES "")
else()
find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    visualization_msgs
    pointcloud_topo
    cv_bridge
      lidar_map
  )
  message(STATUS "📍 path_searching: Host find_package")
endif()

# ✅ ARM 크로스 컴파일 시 catkin_package 조정
if(CMAKE_CROSSCOMPILING AND CMAKE_SYSROOT)
  catkin_package(
      INCLUDE_DIRS include
      LIBRARIES path_searching
      # CATKIN_DEPENDS 제거 - ARM 컴파일 시 의존성 자동 링크 방지
  )
  message(STATUS "✅ path_searching: ARM catkin_package")
else()
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES path_searching
      CATKIN_DEPENDS pointcloud_topo lidar_map
)
  message(STATUS "📍 path_searching: Host catkin_package")
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
  
  # 호스트 시스템 헤더 경로 명시적으로 제거
  list(REMOVE_ITEM PCL_INCLUDE_DIRS "/usr/include/pcl-1.10")
  list(REMOVE_ITEM catkin_INCLUDE_DIRS "/usr/include")
  
  message(STATUS "✅ path_searching: ARM sysroot headers prioritized")
endif()

add_library(path_searching
    src/bubble_astar.cpp
)

# ✅ ARM 크로스 컴파일 시 IMPORTED targets 사용
if(CMAKE_CROSSCOMPILING AND CMAKE_SYSROOT AND ARM_ROSCPP_LIB)
  target_link_libraries(path_searching
    arm_roscpp
    arm_roscpp_serialization
    arm_rostime
    arm_cpp_common
    arm_rosconsole
    ${PCL_LIBRARIES}
    pointcloud_topo
    unit_shpere
    ikd_tree
  )
  
  if(ARM_CV_BRIDGE_LIB)
    target_link_libraries(path_searching arm_cv_bridge)
  endif()
  
  message(STATUS "✅ path_searching: Using ARM IMPORTED targets")
else()
target_link_libraries(path_searching
    ${catkin_LIBRARIES}
)
  message(STATUS "📍 path_searching: Using host catkin_LIBRARIES")
endif()
