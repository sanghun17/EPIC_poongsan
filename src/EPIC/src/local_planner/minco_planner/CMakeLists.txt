cmake_minimum_required(VERSION 3.0.2)
project(plan_manage)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_BUILD_TYPE RelWithDebInfo)

set(CMAKE_CXX_FLAGS "-std=c++14")

# ARM 크로스컴파일을 위한 최적화 플래그
if(CMAKE_CROSSCOMPILING)
  # ARM 크로스컴파일 시 사용할 플래그 (-fpermissive 추가로 BOOST_FOREACH 오류 무시)
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -w -ffast-math -funroll-loops -fpermissive")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive")
else()
  # 네이티브 컴파일 시에만 native 플래그 사용
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -march=native -mtune=native -ffast-math -funroll-loops")
endif()

find_package(Eigen3 REQUIRED)

# Boost 찾기 - BOOST_FOREACH 매크로를 위해 필요
find_package(Boost REQUIRED)

# BOOST_FOREACH 매크로 강제 활성화
add_definitions(-DBOOST_FOREACH_NO_REVERSE_ITERATION)
add_definitions(-DBOOST_NO_CXX11_SCOPED_ENUMS)
add_definitions(-DBOOST_NO_CXX11_RANGE_BASED_FOR)

# ARM 크로스컴파일용 추가 BOOST 설정
if(CMAKE_CROSSCOMPILING)
  # BOOST 헤더를 명시적으로 포함하도록 강제
  add_definitions(-include boost/foreach.hpp)
endif()

# PCL 찾기 (ARM sysroot 우선)
if(CMAKE_CROSSCOMPILING)
  find_package(PCL REQUIRED PATHS ${CMAKE_SYSROOT}/usr NO_DEFAULT_PATH)
else()
  find_package(PCL REQUIRED)
endif()

# ARM 전용 - 크로스컴파일만 지원
find_package(catkin REQUIRED COMPONENTS
roscpp
std_msgs
  geometry_msgs
  swarmtal_msgs
  visualization_msgs
  nav_msgs
  path_searching
  pointcloud_topo
  lidar_map
  poly_traj
  traj_utils
  # decomp_util 제거 - 원본에 없는 의존성
)

# ARM sysroot에서 ROS 라이브러리 직접 찾기
find_library(ARM_ROSCPP_LIB NAMES roscpp PATHS ${CMAKE_SYSROOT}/opt/ros/noetic/lib NO_DEFAULT_PATH)
find_library(ARM_ROSCPP_SERIALIZATION_LIB NAMES roscpp_serialization PATHS ${CMAKE_SYSROOT}/opt/ros/noetic/lib NO_DEFAULT_PATH)
find_library(ARM_ROSTIME_LIB NAMES rostime PATHS ${CMAKE_SYSROOT}/opt/ros/noetic/lib NO_DEFAULT_PATH)
find_library(ARM_CPP_COMMON_LIB NAMES cpp_common PATHS ${CMAKE_SYSROOT}/opt/ros/noetic/lib NO_DEFAULT_PATH)
find_library(ARM_ROSCONSOLE_LIB NAMES rosconsole PATHS ${CMAKE_SYSROOT}/opt/ros/noetic/lib NO_DEFAULT_PATH)
find_library(ARM_XMLRPCPP_LIB NAMES xmlrpcpp PATHS ${CMAKE_SYSROOT}/opt/ros/noetic/lib NO_DEFAULT_PATH)

# ARM 라이브러리들에 대한 alias 생성
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
  
  add_library(arm_xmlrpcpp SHARED IMPORTED)
  set_target_properties(arm_xmlrpcpp PROPERTIES IMPORTED_LOCATION ${ARM_XMLRPCPP_LIB})
endif()

# catkin_LIBRARIES 무효화
set(catkin_LIBRARIES "")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES plan_manage 
  # CATKIN_DEPENDS 제거 - ARM 크로스컴파일에서 의존성 문제 방지
  # DEPENDS system_lib 제거 - system_lib이 정의되지 않아 경고 발생
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${CMAKE_SYSROOT}/opt/ros/noetic/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../Utils/traj_utils/include
)

# ARM 크로스컴파일용 헤더 경로 추가
if(CMAKE_CROSSCOMPILING)
  include_directories(BEFORE SYSTEM
    ${CMAKE_SYSROOT}/usr/include/eigen3
    ${CMAKE_SYSROOT}/opt/ros/noetic/include
    ${CMAKE_SYSROOT}/usr/include/pcl-1.14
    ${CMAKE_SYSROOT}/usr/include/boost
  )
else()
  include_directories(SYSTEM
    /usr/include/eigen3
    /usr/include/pcl-1.10
    /usr/include/boost
  )
endif()

# ARM 전용 - 모든 타겟 빌드
add_executable(fast_planner_node 
        src/fast_planner_node.cpp
        src/planner_manager.cpp
        # src/kino_replan_fsm.cpp 제거 - 파일이 존재하지 않음
        )

# fast_planner_node에만 특별한 컴파일 옵션 적용
if(CMAKE_CROSSCOMPILING)
  target_compile_options(fast_planner_node PRIVATE 
    -fpermissive 
    -Wno-error
    -DBOOST_NO_CXX11_SCOPED_ENUMS
  )
endif()

add_executable(traj_server 
        src/traj_server.cpp)

add_library(plan_manage 
        src/planner_manager.cpp
        # src/kino_replan_fsm.cpp 제거 - 파일이 존재하지 않음
        )

if(ARM_ROSCPP_LIB)
  target_link_libraries(plan_manage
    path_searching
    pointcloud_topo
    ikd_tree
    unit_shpere
    poly_traj
    traj_utils
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    arm_roscpp
    arm_roscpp_serialization
    arm_rostime
    arm_cpp_common
    arm_rosconsole
    arm_xmlrpcpp
  )
else()
  target_link_libraries(plan_manage 
    path_searching
    pointcloud_topo
    ikd_tree
    unit_shpere
    poly_traj
    traj_utils
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
  )
endif()

if(ARM_ROSCPP_LIB)
  target_link_libraries(fast_planner_node 
    plan_manage
    path_searching
    pointcloud_topo
    ikd_tree
    unit_shpere
    poly_traj
    traj_utils
    arm_roscpp
    arm_roscpp_serialization
    arm_rostime
    arm_cpp_common
    arm_rosconsole
    arm_xmlrpcpp
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
  )
else()
  target_link_libraries(fast_planner_node 
    plan_manage
    path_searching
    pointcloud_topo
    ikd_tree
    unit_shpere
    poly_traj
    traj_utils
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
  )
endif()

if(ARM_ROSCPP_LIB)
  target_link_libraries(traj_server 
    plan_manage
    arm_roscpp
    arm_roscpp_serialization
    arm_rostime
    arm_cpp_common
    arm_rosconsole
    arm_xmlrpcpp
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
  )
else()
  target_link_libraries(traj_server 
    plan_manage
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
  )
endif()



