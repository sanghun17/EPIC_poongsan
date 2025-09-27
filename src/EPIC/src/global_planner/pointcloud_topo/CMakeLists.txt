cmake_minimum_required(VERSION 3.16)
project(pointcloud_topo)
find_package(OpenMP REQUIRED)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_BUILD_TYPE "RelWithDebInfo")

# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

# find_package(CGAL REQUIRED)
# ✅ ARM 컴파일 시 catkin find_package 의존성 조정
if(CMAKE_CROSSCOMPILING AND CMAKE_SYSROOT)
  find_package(catkin REQUIRED COMPONENTS
    roscpp rospy std_msgs visualization_msgs cv_bridge message_filters
    lkh_tsp_solver lidar_map
  )
  message(STATUS "✅ pointcloud_topo: ARM find_package")
  
  # ARM sysroot에서 ROS 라이브러리 직접 찾기
  find_library(ARM_ROSCPP_LIB NAMES roscpp PATHS ${CMAKE_SYSROOT}/opt/ros/noetic/lib NO_DEFAULT_PATH)
  find_library(ARM_ROSCPP_SERIALIZATION_LIB NAMES roscpp_serialization PATHS ${CMAKE_SYSROOT}/opt/ros/noetic/lib NO_DEFAULT_PATH)
  find_library(ARM_ROSTIME_LIB NAMES rostime PATHS ${CMAKE_SYSROOT}/opt/ros/noetic/lib NO_DEFAULT_PATH)
  find_library(ARM_CPP_COMMON_LIB NAMES cpp_common PATHS ${CMAKE_SYSROOT}/opt/ros/noetic/lib NO_DEFAULT_PATH)
  find_library(ARM_ROSCONSOLE_LIB NAMES rosconsole PATHS ${CMAKE_SYSROOT}/opt/ros/noetic/lib NO_DEFAULT_PATH)
  find_library(ARM_CV_BRIDGE_LIB NAMES cv_bridge PATHS ${CMAKE_SYSROOT}/opt/ros/noetic/lib NO_DEFAULT_PATH)
  find_library(ARM_MESSAGE_FILTERS_LIB NAMES message_filters PATHS ${CMAKE_SYSROOT}/opt/ros/noetic/lib NO_DEFAULT_PATH)
  
  # ARM에서 OpenCV 찾기
  find_package(OpenCV REQUIRED PATHS ${CMAKE_SYSROOT}/usr/lib/cmake/opencv4 NO_DEFAULT_PATH)
  
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
    
    message(STATUS "✅ pointcloud_topo: ARM IMPORTED targets created")
  endif()
  
  # catkin_LIBRARIES 무효화
  set(catkin_LIBRARIES "")
  
  find_package(Boost REQUIRED COMPONENTS program_options PATHS ${CMAKE_SYSROOT}/usr/lib/cmake NO_DEFAULT_PATH)
else()
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(catkin REQUIRED COMPONENTS
      roscpp rospy std_msgs visualization_msgs cv_bridge message_filters
      lkh_tsp_solver lidar_map
  )
  message(STATUS "📍 pointcloud_topo: Host find_package")
endif()

find_package(Eigen3 REQUIRED)

# PCL 찾기 (ARM sysroot 우선)
if(CMAKE_CROSSCOMPILING AND CMAKE_SYSROOT)
  find_package(PCL REQUIRED COMPONENTS PATHS ${CMAKE_SYSROOT}/usr NO_DEFAULT_PATH)
else()
  find_package(PCL REQUIRED COMPONENTS)
endif()

# include_directories(/home/lin/workspace/CGAL-5.6.1/include)
# ✅ ARM 크로스 컴파일 시 catkin_package 조정
if(CMAKE_CROSSCOMPILING AND CMAKE_SYSROOT)
  catkin_package(
      INCLUDE_DIRS include
      LIBRARIES pointcloud_topo parallel_bubble_astar
      # CATKIN_DEPENDS 제거 - ARM 컴파일 시 ROS 의존성 자동 링크 방지
  )
  message(STATUS "✅ pointcloud_topo: ARM catkin_package")
else()
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES pointcloud_topo parallel_bubble_astar
    CATKIN_DEPENDS roscpp std_msgs lkh_tsp_solver lidar_map
)
  message(STATUS "📍 pointcloud_topo: Host catkin_package")
endif()

include_directories(
    SYSTEM
    include
    ${catkin_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
    ${Eigen3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

# ARM 크로스컴파일 시 헤더 경로 우선순위 조정
if(CMAKE_CROSSCOMPILING AND CMAKE_SYSROOT)
  include_directories(BEFORE SYSTEM
    ${CMAKE_SYSROOT}/usr/include/eigen3
    ${CMAKE_SYSROOT}/opt/ros/noetic/include
    ${CMAKE_SYSROOT}/usr/include/pcl-1.14
  )
endif()

link_directories(${PCL_LIBRARY_DIRS})

add_library(parallel_bubble_astar
    src/parallel_bubble_astar.cpp
)

# ✅ ARM 크로스 컴파일 시 IMPORTED targets 사용
if(CMAKE_CROSSCOMPILING AND CMAKE_SYSROOT AND ARM_ROSCPP_LIB)
  target_link_libraries(parallel_bubble_astar
    arm_roscpp
    arm_roscpp_serialization
    arm_rostime
    arm_cpp_common
    arm_rosconsole
    ${PCL_LIBRARIES}
  )
  
  if(ARM_CV_BRIDGE_LIB)
    target_link_libraries(parallel_bubble_astar arm_cv_bridge)
  endif()
  if(ARM_MESSAGE_FILTERS_LIB)
    target_link_libraries(parallel_bubble_astar arm_message_filters)
  endif()
  
  message(STATUS "✅ parallel_bubble_astar: Using ARM IMPORTED targets")
else()
target_link_libraries(parallel_bubble_astar
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)
  message(STATUS "📍 parallel_bubble_astar: Using host catkin_LIBRARIES")
endif()

add_library(pointcloud_topo
    src/topo_skeleton_graph.cpp
    src/historical_odom_graph.cpp
    src/bubble_split.cpp
)

# ✅ ARM 크로스 컴파일 시 IMPORTED targets 사용
if(CMAKE_CROSSCOMPILING AND CMAKE_SYSROOT AND ARM_ROSCPP_LIB)
target_link_libraries(pointcloud_topo
    arm_roscpp
    arm_roscpp_serialization
    arm_rostime
    arm_cpp_common
    arm_rosconsole
    ${PCL_LIBRARIES}
    OpenMP::OpenMP_CXX
    parallel_bubble_astar
)
  
  if(ARM_CV_BRIDGE_LIB)
    target_link_libraries(pointcloud_topo arm_cv_bridge)
  endif()
  if(ARM_MESSAGE_FILTERS_LIB)
    target_link_libraries(pointcloud_topo arm_message_filters)
  endif()
  
  message(STATUS "✅ pointcloud_topo: Using ARM IMPORTED targets")
else()
  target_link_libraries(pointcloud_topo
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES}
      OpenMP::OpenMP_CXX
      parallel_bubble_astar
  )
  message(STATUS "📍 pointcloud_topo: Using host catkin_LIBRARIES")
endif()

