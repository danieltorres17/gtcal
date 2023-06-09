cmake_minimum_required(VERSION 3.22.1)

project(gtcal)
set(CMAKE_CXX_STANDARD 20)

find_package(GTSAM REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
  include
  ${GTSAM_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS} 
)

add_library(pose_solver src/pose_solver.cpp)
target_link_libraries(pose_solver gtsam ${CERES_LIBRARIES})

add_library(pose_solver_gtsam src/pose_solver_gtsam.cpp)
target_link_libraries(pose_solver_gtsam gtsam)

add_library(batch_solver src/batch_solver.cpp)
target_link_libraries(batch_solver gtsam)

add_executable(gtcal src/gtcal.cpp)
target_link_libraries(gtcal gtsam)

add_subdirectory(test)
