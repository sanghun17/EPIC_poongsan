cmake_minimum_required(VERSION 2.8.3)
project(epic_planner)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_BUILD_TYPE "RelWithDebInfo")

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
  
  message(STATUS "✅ epic_planner: ARM compatibility flags added")
endif()

find_package(Eigen3 REQUIRED)

# PCL 찾기 (ARM sysroot 우선)
if(CMAKE_CROSSCOMPILING)
  find_package(PCL REQUIRED PATHS ${CMAKE_SYSROOT}/usr NO_DEFAULT_PATH)
else()
find_package(PCL 1.7 REQUIRED)
endif()

find_package(OpenMP REQUIRED)

# OpenCV 찾기 (ARM sysroot 우선)
if(CMAKE_CROSSCOMPILING)
  find_package(OpenCV REQUIRED PATHS ${CMAKE_SYSROOT}/usr/lib/cmake/opencv4 NO_DEFAULT_PATH)
else()
find_package(OpenCV REQUIRED)
endif()

# ✅ ARM 컴파일 시 catkin find_package 의존성 조정
if(CMAKE_CROSSCOMPILING AND CMAKE_SYSROOT)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  trajectory_msgs
  visualization_msgs
  nav_msgs
  quadrotor_msgs
  pointcloud_topo
  path_searching
  cv_bridge
    frontier_manager
    traj_utils
    lkh_tsp_solver
    plan_manage
    lidar_map
    poly_traj
    message_filters
  )
  message(STATUS "✅ epic_planner: ARM find_package")
  
  # ARM sysroot에서 ROS 라이브러리 직접 찾기
  find_library(ARM_ROSCPP_LIB NAMES roscpp PATHS ${CMAKE_SYSROOT}/opt/ros/noetic/lib NO_DEFAULT_PATH)
  find_library(ARM_ROSCPP_SERIALIZATION_LIB NAMES roscpp_serialization PATHS ${CMAKE_SYSROOT}/opt/ros/noetic/lib NO_DEFAULT_PATH)
  find_library(ARM_ROSTIME_LIB NAMES rostime PATHS ${CMAKE_SYSROOT}/opt/ros/noetic/lib NO_DEFAULT_PATH)
  find_library(ARM_CPP_COMMON_LIB NAMES cpp_common PATHS ${CMAKE_SYSROOT}/opt/ros/noetic/lib NO_DEFAULT_PATH)
  find_library(ARM_ROSCONSOLE_LIB NAMES rosconsole PATHS ${CMAKE_SYSROOT}/opt/ros/noetic/lib NO_DEFAULT_PATH)
  find_library(ARM_CV_BRIDGE_LIB NAMES cv_bridge PATHS ${CMAKE_SYSROOT}/opt/ros/noetic/lib NO_DEFAULT_PATH)
  find_library(ARM_MESSAGE_FILTERS_LIB NAMES message_filters PATHS ${CMAKE_SYSROOT}/opt/ros/noetic/lib NO_DEFAULT_PATH)
  
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
    
    if(ARM_MESSAGE_FILTERS_LIB)
      add_library(arm_message_filters SHARED IMPORTED)
      set_target_properties(arm_message_filters PROPERTIES IMPORTED_LOCATION ${ARM_MESSAGE_FILTERS_LIB})
    endif()
    
    message(STATUS "✅ epic_planner: ARM IMPORTED targets created")
  endif()
  
  # catkin_LIBRARIES 무효화
  set(catkin_LIBRARIES "")
else()
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    trajectory_msgs
    visualization_msgs
    nav_msgs
    quadrotor_msgs
    pointcloud_topo
    path_searching
    cv_bridge
  frontier_manager
  traj_utils
  lkh_tsp_solver
  plan_manage
    lidar_map
    poly_traj
  message_filters
  )
  message(STATUS "📍 epic_planner: Host find_package")
endif()

# ✅ ARM 크로스 컴파일 시 catkin_package 조정
if(CMAKE_CROSSCOMPILING AND CMAKE_SYSROOT)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES epic_planner
    # CATKIN_DEPENDS 제거 - ARM 컴파일 시 의존성 자동 링크 방지
  )
  message(STATUS "✅ epic_planner: ARM catkin_package")
else()
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES epic_planner
    CATKIN_DEPENDS pointcloud_topo path_searching traj_utils lkh_tsp_solver plan_manage frontier_manager lidar_map
)
  message(STATUS "📍 epic_planner: Host catkin_package")
endif()

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

if(CMAKE_CROSSCOMPILING)
  # ARM sysroot 헤더를 최우선으로 설정 (BEFORE SYSTEM)
  include_directories(BEFORE SYSTEM
    ${CMAKE_SYSROOT}/usr/include/pcl-1.14
    ${CMAKE_SYSROOT}/usr/include/opencv4
    ${CMAKE_SYSROOT}/usr/include/eigen3
    ${CMAKE_SYSROOT}/opt/ros/noetic/include
  )
  
  message(STATUS "✅ epic_planner: ARM sysroot headers prioritized")
endif()

add_library(epic_planner
  src/fast_exploration_manager.cpp
)

add_executable(exploration_node
  src/exploration_node.cpp
  src/fast_exploration_fsm.cpp
  src/fsm_utils.cpp
)

# Custom Trajectory Sampler Node
add_executable(custom_trajectory_sampler
  src/custom_trajectory_sampler.cpp
)

# ✅ ARM 크로스 컴파일 시 IMPORTED targets 사용
if(CMAKE_CROSSCOMPILING AND CMAKE_SYSROOT AND ARM_ROSCPP_LIB)
  target_link_libraries(exploration_node
    arm_roscpp
    arm_roscpp_serialization
    arm_rostime
    arm_cpp_common
    arm_rosconsole
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    OpenMP::OpenMP_CXX
    epic_planner
    pointcloud_topo
    path_searching
    frontier_manager
    unit_shpere
    ikd_tree
    lkh_tsp_solver
    plan_manage
    traj_utils
    poly_traj
  )
  
  target_link_libraries(epic_planner
    arm_roscpp
    arm_roscpp_serialization
    arm_rostime
    arm_cpp_common
    arm_rosconsole
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    pointcloud_topo
    path_searching
    frontier_manager
    unit_shpere
    ikd_tree
    lkh_tsp_solver
    plan_manage
    traj_utils
    poly_traj
  )
  
  # cv_bridge와 message_filters는 선택적으로 링크
  if(ARM_CV_BRIDGE_LIB)
    target_link_libraries(exploration_node arm_cv_bridge)
    target_link_libraries(epic_planner arm_cv_bridge)
  endif()
  if(ARM_MESSAGE_FILTERS_LIB)
    target_link_libraries(exploration_node arm_message_filters)
    target_link_libraries(epic_planner arm_message_filters)
  endif()
  
  # Custom Trajectory Sampler linking for ARM
  target_link_libraries(custom_trajectory_sampler
    arm_roscpp
    arm_roscpp_serialization
    arm_rostime
    arm_cpp_common
    arm_rosconsole
    traj_utils
    poly_traj
  )
  
  message(STATUS "✅ epic_planner: Using ARM IMPORTED targets")
else()
target_link_libraries(exploration_node
  ${catkin_LIBRARIES}
  OpenMP::OpenMP_CXX
  epic_planner
)

# Custom Trajectory Sampler linking for Host
target_link_libraries(custom_trajectory_sampler
  ${catkin_LIBRARIES}
)

target_link_libraries(epic_planner
  ${catkin_LIBRARIES}
)
  
  message(STATUS "📍 epic_planner: Using host catkin_LIBRARIES")
endif()
