cmake_minimum_required(VERSION 3.22.1)

project(gtcal)

set(CXX_STANDARD 20)
set(CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Ceres REQUIRED)
find_package(spdlog REQUIRED)
find_package(Pangolin REQUIRED)

set(GTCAL_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/include)

add_library(calibration_target src/calibration_target.cpp)
target_include_directories(calibration_target PRIVATE ${GTCAL_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR})
target_link_libraries(calibration_target gtsam)

add_library(pose_solver src/pose_solver.cpp)
target_include_directories(pose_solver PRIVATE ${GTCAL_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})
target_link_libraries(pose_solver gtsam ${CERES_LIBRARIES})

add_library(pose_solver_gtsam src/pose_solver_gtsam.cpp)
target_include_directories(pose_solver_gtsam PRIVATE ${GTCAL_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR})
target_link_libraries(pose_solver_gtsam gtsam)

add_library(batch_solver src/batch_solver.cpp)
target_include_directories(batch_solver PRIVATE ${GTCAL_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR})
target_link_libraries(batch_solver gtsam calibration_target)

add_library(visualizer src/viz.cpp)
target_include_directories(visualizer PRIVATE ${GTCAL_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR} ${PANGOLIN_INCLUDE_DIR})
target_link_libraries(visualizer ${Pangolin_LIBRARIES})

add_subdirectory(test)
