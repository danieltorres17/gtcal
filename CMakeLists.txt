cmake_minimum_required(VERSION 3.22.1)

project(gtcal)

set(CXX_STANDARD 20)
set(CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Ceres REQUIRED)
find_package(PCL REQUIRED)

set(GTCAL_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/include)

add_library(pose_solver src/pose_solver.cpp)
target_include_directories(pose_solver PRIVATE ${GTCAL_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})
target_link_libraries(pose_solver gtsam ${CERES_LIBRARIES})

add_library(pose_solver_gtsam src/pose_solver_gtsam.cpp)
target_include_directories(pose_solver_gtsam PRIVATE ${GTCAL_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR})
target_link_libraries(pose_solver_gtsam gtsam)

add_library(batch_solver src/batch_solver.cpp)
target_include_directories(batch_solver PRIVATE ${GTCAL_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR})
target_link_libraries(batch_solver gtsam)

add_subdirectory(test)
