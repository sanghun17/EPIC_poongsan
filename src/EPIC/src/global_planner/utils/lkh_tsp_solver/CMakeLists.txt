cmake_minimum_required(VERSION 2.8.3)
project(lkh_tsp_solver)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS "-O3 -Wall")

# ✅ ARM 컴파일 시 catkin find_package 의존성 조정
if(CMAKE_CROSSCOMPILING AND CMAKE_SYSROOT)
  # ARM 컴파일 시에는 ROS 의존성 없이 catkin만 찾기
  find_package(catkin REQUIRED)
  message(STATUS "✅ lkh_tsp_solver: ARM find_package (catkin only)")
else()
  # 호스트 컴파일 시에는 모든 ROS 의존성 포함
find_package(catkin REQUIRED COMPONENTS
  roscpp
    rospy
    std_msgs
  )
  message(STATUS "📍 lkh_tsp_solver: Host find_package (with ROS components)")
endif()

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

# ✅ 디버깅: CMAKE 변수 상태 확인
message(STATUS "🔍 lkh_tsp_solver DEBUG INFO:")
message(STATUS "   CMAKE_CROSSCOMPILING: ${CMAKE_CROSSCOMPILING}")
message(STATUS "   CMAKE_SYSROOT: ${CMAKE_SYSROOT}")
message(STATUS "   CMAKE_C_COMPILER: ${CMAKE_C_COMPILER}")

# ✅ ARM 크로스 컴파일을 위한 링킹 설정
if(CMAKE_CROSSCOMPILING AND CMAKE_SYSROOT)
  message(STATUS "🔧 lkh_tsp_solver: ARM Cross-compilation setup started")
  
  # ARM sysroot의 라이브러리 디렉토리를 우선순위로 설정
  link_directories(
    BEFORE
    ${CMAKE_SYSROOT}/opt/ros/noetic/lib
    ${CMAKE_SYSROOT}/usr/lib
  )
  
  # 호스트 ROS 라이브러리 링크 방지
  set(CMAKE_IGNORE_PATH /opt/ros/noetic/lib)
  
  # ARM sysroot에서 roscpp 직접 찾기
  find_library(ARM_ROSCPP_LIB NAMES roscpp PATHS ${CMAKE_SYSROOT}/opt/ros/noetic/lib NO_DEFAULT_PATH)
  find_library(ARM_ROSCPP_SERIALIZATION_LIB NAMES roscpp_serialization PATHS ${CMAKE_SYSROOT}/opt/ros/noetic/lib NO_DEFAULT_PATH)
  find_library(ARM_ROSTIME_LIB NAMES rostime PATHS ${CMAKE_SYSROOT}/opt/ros/noetic/lib NO_DEFAULT_PATH)
  find_library(ARM_CPP_COMMON_LIB NAMES cpp_common PATHS ${CMAKE_SYSROOT}/opt/ros/noetic/lib NO_DEFAULT_PATH)
  
  message(STATUS "✅ ARM library search results:")
  message(STATUS "   ARM_ROSCPP_LIB: ${ARM_ROSCPP_LIB}")
  message(STATUS "   ARM_ROSCPP_SERIALIZATION_LIB: ${ARM_ROSCPP_SERIALIZATION_LIB}")
  message(STATUS "   ARM_ROSTIME_LIB: ${ARM_ROSTIME_LIB}")
  message(STATUS "   ARM_CPP_COMMON_LIB: ${ARM_CPP_COMMON_LIB}")
  
  # IMPORTED 타겟 생성으로 CMake가 자동 링킹을 우회하도록 함
  if(ARM_ROSCPP_LIB)
    add_library(arm_roscpp SHARED IMPORTED)
    set_target_properties(arm_roscpp PROPERTIES IMPORTED_LOCATION ${ARM_ROSCPP_LIB})
    
    add_library(arm_roscpp_serialization SHARED IMPORTED)
    set_target_properties(arm_roscpp_serialization PROPERTIES IMPORTED_LOCATION ${ARM_ROSCPP_SERIALIZATION_LIB})
    
    add_library(arm_rostime SHARED IMPORTED)
    set_target_properties(arm_rostime PROPERTIES IMPORTED_LOCATION ${ARM_ROSTIME_LIB})
    
    add_library(arm_cpp_common SHARED IMPORTED)
    set_target_properties(arm_cpp_common PROPERTIES IMPORTED_LOCATION ${ARM_CPP_COMMON_LIB})
    
    message(STATUS "✅ lkh_tsp_solver: ARM IMPORTED targets created")
  endif()
  
  # catkin_LIBRARIES 무효화 (ARM 컴파일 시)
  set(catkin_LIBRARIES "")
  
  # ROS 라이브러리들을 임시로 찾을 수 없게 만들기
  set(roscpp_LIBRARIES "")
  set(roscpp_serialization_LIBRARIES "")
  set(rostime_LIBRARIES "")
  set(cpp_common_LIBRARIES "")
  
  message(STATUS "✅ lkh_tsp_solver: catkin_LIBRARIES cleared for ARM build")
else()
  message(STATUS "📍 lkh_tsp_solver: Using host compilation (not cross-compiling)")
endif()

# ✅ ARM 컴파일 시 catkin_package 의존성 조정
if(CMAKE_CROSSCOMPILING AND CMAKE_SYSROOT)
  catkin_package(
   INCLUDE_DIRS include
   LIBRARIES lkh_tsp_solver
   # CATKIN_DEPENDS 제거 - ARM 컴파일 시 ROS 의존성 자동 링크 방지
   DEPENDS system_lib
  )
  message(STATUS "✅ lkh_tsp_solver: ARM catkin_package (no CATKIN_DEPENDS)")
else()
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES lkh_tsp_solver
   CATKIN_DEPENDS roscpp rospy std_msgs
   DEPENDS system_lib
)
  message(STATUS "📍 lkh_tsp_solver: Host catkin_package (with CATKIN_DEPENDS)")
endif()

include_directories( 
    include 
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

set(SRCS
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Activate.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/AddCandidate.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/AddExtraCandidates.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/AddTourCandidates.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/AdjustCandidateSet.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/AdjustClusters.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/AllocateStructures.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Ascent.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Best2OptMove.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Best3OptMove.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Best4OptMove.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Best5OptMove.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/BestKOptMove.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Between_SL.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Between_SSL.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Between.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/BridgeGain.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/BuildKDTree.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/C.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/CandidateReport.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ChooseInitialTour.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Connect.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Create_POPMUSIC_CandidateSet.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/CreateCandidateSet.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/CreateDelaunayCandidateSet.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/CreateNNCandidateSet.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/CreateQuadrantCandidateSet.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Delaunay.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Distance_SPECIAL.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Distance.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/eprintf.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ERXT.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Excludable.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Exclude.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/FindTour.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/FixedOrCommonCandidates.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Flip_SL.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Flip_SSL.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Flip.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Forbidden.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/FreeStructures.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/fscanint.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Gain23.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/GenerateCandidates.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Genetic.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/GeoConversion.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/GetTime.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/gpx.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/GreedyTour.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Hashing.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Heap.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/IsBackboneCandidate.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/IsCandidate.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/IsCommonEdge.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/IsPossibleCandidate.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/KSwapKick.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/LinKernighan.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/lkh_globals.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/lkh_interface.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/LKHmain.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Make2OptMove.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Make3OptMove.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Make4OptMove.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Make5OptMove.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/MakeKOptMove.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/MergeTourWithBestTour.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/MergeWithTourGPX2.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/MergeWithTourIPT.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Minimum1TreeCost.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/MinimumSpanningTree.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/NormalizeNodeList.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/NormalizeSegmentList.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/OrderCandidateSet.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/PatchCycles.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/printff.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/PrintParameters.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Random.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ReadCandidates.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ReadEdges.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ReadLine.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ReadParameters.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ReadPenalties.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ReadProblem.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/RecordBestTour.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/RecordBetterTour.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/RemoveFirstActive.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ResetCandidateSet.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/RestoreTour.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/SegmentSize.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Sequence.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/SFCTour.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/SolveCompressedSubproblem.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/SolveDelaunaySubproblems.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/SolveKarpSubproblems.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/SolveKCenterSubproblems.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/SolveKMeansSubproblems.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/SolveRoheSubproblems.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/SolveSFCSubproblems.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/SolveSubproblem.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/SolveSubproblemBorderProblems.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/SolveTourSegmentSubproblems.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Statistics.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/StoreTour.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/SymmetrizeCandidateSet.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/TrimCandidateSet.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/WriteCandidates.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/WritePenalties.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/WriteTour.c
)

add_library( lkh_tsp_solver ${SRCS} )

# ✅ lkh_tsp_solver 라이브러리에도 ARM 링킹 적용
if(CMAKE_CROSSCOMPILING AND CMAKE_SYSROOT AND ARM_ROSCPP_LIB)
  # IMPORTED 타겟 사용으로 자동 링킹 방지
  target_link_libraries(lkh_tsp_solver 
    arm_roscpp
    arm_roscpp_serialization
    arm_rostime
    arm_cpp_common
  )
  
  add_compile_definitions(CROSS_COMPILE_ARM)
  message(STATUS "✅ lkh_tsp_solver: Using ARM IMPORTED targets")
else()
  target_link_libraries(lkh_tsp_solver ${catkin_LIBRARIES})
  message(STATUS "📍 lkh_tsp_solver: Using host catkin_LIBRARIES")
endif()

add_executable( lkh_tsp 
  ${SRCS}
)

# ✅ ARM 크로스 컴파일 시 직접 ARM 라이브러리 링크
if(CMAKE_CROSSCOMPILING AND CMAKE_SYSROOT AND ARM_ROSCPP_LIB)
  # IMPORTED 타겟 사용으로 자동 링킹 방지
  target_link_libraries(lkh_tsp 
    arm_roscpp
    arm_roscpp_serialization
    arm_rostime
    arm_cpp_common
    -lm
  )
  
  message(STATUS "✅ lkh_tsp executable: Using ARM IMPORTED targets")
else()
target_link_libraries(lkh_tsp ${catkin_LIBRARIES} -lm)
  message(STATUS "📍 lkh_tsp executable: Using host catkin_LIBRARIES")
endif()