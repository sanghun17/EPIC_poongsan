cmake_minimum_required(VERSION 3.0.2)
project(lidar_map)
find_package(OpenMP REQUIRED)
# set(CMAKE_BUILD_TYPE "Debug")
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
  
  message(STATUS "✅ lidar_map: ARM compatibility flags added")
endif()

# ✅ ARM 컴파일 시 catkin find_package 의존성 조정
if(CMAKE_CROSSCOMPILING AND CMAKE_SYSROOT)
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    cv_bridge
    message_filters
    visualization_msgs
  )
  message(STATUS "✅ lidar_map: ARM find_package (with components for headers)")
  
  # ARM sysroot에서 ROS 라이브러리 직접 찾기
  find_library(ARM_ROSCPP_LIB NAMES roscpp PATHS ${CMAKE_SYSROOT}/opt/ros/noetic/lib NO_DEFAULT_PATH)
  find_library(ARM_ROSCPP_SERIALIZATION_LIB NAMES roscpp_serialization PATHS ${CMAKE_SYSROOT}/opt/ros/noetic/lib NO_DEFAULT_PATH)
  find_library(ARM_ROSTIME_LIB NAMES rostime PATHS ${CMAKE_SYSROOT}/opt/ros/noetic/lib NO_DEFAULT_PATH)
  find_library(ARM_CPP_COMMON_LIB NAMES cpp_common PATHS ${CMAKE_SYSROOT}/opt/ros/noetic/lib NO_DEFAULT_PATH)
  find_library(ARM_ROSCONSOLE_LIB NAMES rosconsole PATHS ${CMAKE_SYSROOT}/opt/ros/noetic/lib NO_DEFAULT_PATH)
  find_library(ARM_CV_BRIDGE_LIB NAMES cv_bridge PATHS ${CMAKE_SYSROOT}/opt/ros/noetic/lib NO_DEFAULT_PATH)
  find_library(ARM_MESSAGE_FILTERS_LIB NAMES message_filters PATHS ${CMAKE_SYSROOT}/opt/ros/noetic/lib NO_DEFAULT_PATH)
  
  # ARM OpenCV 라이브러리 찾기
  find_package(OpenCV REQUIRED PATHS ${CMAKE_SYSROOT}/usr/lib/cmake/opencv4 NO_DEFAULT_PATH)
  
  # ARM Boost thread 라이브러리 찾기
  find_library(ARM_BOOST_THREAD_LIB NAMES boost_thread PATHS ${CMAKE_SYSROOT}/usr/lib NO_DEFAULT_PATH)
  find_library(ARM_BOOST_SYSTEM_LIB NAMES boost_system PATHS ${CMAKE_SYSROOT}/usr/lib NO_DEFAULT_PATH)
  
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
    
    # Boost thread IMPORTED targets
    if(ARM_BOOST_THREAD_LIB)
      add_library(arm_boost_thread SHARED IMPORTED)
      set_target_properties(arm_boost_thread PROPERTIES IMPORTED_LOCATION ${ARM_BOOST_THREAD_LIB})
    endif()
    
    if(ARM_BOOST_SYSTEM_LIB)
      add_library(arm_boost_system SHARED IMPORTED)
      set_target_properties(arm_boost_system PROPERTIES IMPORTED_LOCATION ${ARM_BOOST_SYSTEM_LIB})
    endif()
    
    message(STATUS "✅ lidar_map: ARM IMPORTED targets created")
  endif()
  
  # catkin_LIBRARIES 무효화
  set(catkin_LIBRARIES "")
else()
  find_package(OpenCV REQUIRED)
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    cv_bridge
    message_filters
    visualization_msgs
  )
  message(STATUS "📍 lidar_map: Host find_package (with ROS components)")
endif()
find_package(Eigen3 REQUIRED)

# PCL 찾기 (ARM sysroot 우선)
if(CMAKE_CROSSCOMPILING)
  find_package(PCL REQUIRED PATHS ${CMAKE_SYSROOT}/usr NO_DEFAULT_PATH)
else()
  find_package(PCL REQUIRED COMPONENTS segmentation)
endif()

# ✅ ARM 크로스 컴파일 시 catkin_package 조정
if(CMAKE_CROSSCOMPILING AND CMAKE_SYSROOT)
  catkin_package(
      INCLUDE_DIRS include
      LIBRARIES unit_shpere ikd_tree 
      # CATKIN_DEPENDS 제거 - ARM 컴파일 시 ROS 의존성 자동 링크 방지
      # DEPENDS system_lib
  )
  message(STATUS "✅ lidar_map: ARM catkin_package (no CATKIN_DEPENDS)")
else()
  catkin_package(
      INCLUDE_DIRS include
      LIBRARIES unit_shpere ikd_tree 
      CATKIN_DEPENDS roscpp std_msgs 
      # DEPENDS system_lib
  )
  message(STATUS "📍 lidar_map: Host catkin_package")
endif()

include_directories(
    SYSTEM
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
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
  
  # 호스트 시스템 헤더 경로 명시적으로 제거
  list(REMOVE_ITEM PCL_INCLUDE_DIRS "/usr/include/pcl-1.10")
  list(REMOVE_ITEM catkin_INCLUDE_DIRS "/usr/include")
  list(REMOVE_ITEM OpenCV_INCLUDE_DIRS "/usr/include/opencv4")
  
  message(STATUS "✅ lidar_map: ARM sysroot headers prioritized")
endif()

link_directories(${PCL_LIBRARY_DIRS})
add_library(ikd_tree
  src/ikd_Tree.cpp

)
add_library(unit_shpere
    src/sim_lio.cpp
    src/lidar_map_interface.cpp
)

# ✅ ARM 크로스 컴파일 시 IMPORTED targets 사용
if(CMAKE_CROSSCOMPILING AND CMAKE_SYSROOT AND ARM_ROSCPP_LIB)
  target_link_libraries(unit_shpere
    arm_roscpp
    arm_roscpp_serialization
    arm_rostime
    arm_cpp_common
    arm_rosconsole
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    OpenMP::OpenMP_CXX
    ikd_tree
  )
  
  # cv_bridge와 message_filters는 선택적으로 링크
  if(ARM_CV_BRIDGE_LIB)
    target_link_libraries(unit_shpere arm_cv_bridge)
  endif()
  if(ARM_MESSAGE_FILTERS_LIB)
    target_link_libraries(unit_shpere arm_message_filters)
  endif()
  
  # Boost thread 라이브러리 링크
  if(ARM_BOOST_THREAD_LIB)
    target_link_libraries(unit_shpere arm_boost_thread)
  endif()
  if(ARM_BOOST_SYSTEM_LIB)
    target_link_libraries(unit_shpere arm_boost_system)
  endif()
  
  message(STATUS "✅ lidar_map: Using ARM IMPORTED targets")
else()
  target_link_libraries(unit_shpere
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES}
      OpenMP::OpenMP_CXX
      ikd_tree
  )
  message(STATUS "📍 lidar_map: Using host catkin_LIBRARIES")
endif()