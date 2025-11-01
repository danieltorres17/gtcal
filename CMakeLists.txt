cmake_minimum_required(VERSION 3.22.1)

project(gtcal)

set(CXX_STANDARD 20)
set(CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Ceres REQUIRED)
find_package(spdlog REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenCV REQUIRED)

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

add_library(simulator src/simulator.cpp)
target_include_directories(simulator PRIVATE ${GTCAL_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR})
target_link_libraries(simulator gtsam calibration_target)

add_library(viewer src/viewer.cpp)
target_include_directories(viewer PRIVATE ${GTCAL_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR} ${PANGOLIN_INCLUDE_DIR})
target_link_libraries(viewer ${Pangolin_LIBRARIES} gtsam)

add_library(calibration_ctx src/calibration_ctx.cpp)
target_include_directories(calibration_ctx PRIVATE ${GTCAL_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR})
target_link_libraries(calibration_ctx gtsam calibration_target pose_solver pose_solver_gtsam)

add_library(random_pose_generator src/random_pose_generator.cpp)
target_include_directories(random_pose_generator PRIVATE ${GTCAL_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR})
target_link_libraries(random_pose_generator gtsam calibration_target)

add_executable(sim_dataset_generator src/sim_dataset_generator.cpp)
target_include_directories(sim_dataset_generator PRIVATE ${GTCAL_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${SPDLOG_INCLUDE_DIRS})
target_link_libraries(sim_dataset_generator random_pose_generator ${OpenCV_LIBS})

add_executable(single_camera_cal_example apps/single_camera_cal_example.cpp)
target_include_directories(single_camera_cal_example PRIVATE ${GTCAL_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR} ${SPDLOG_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(single_camera_cal_example calibration_ctx simulator viewer batch_solver gtsam random_pose_generator ${OpenCV_LIBS})

add_subdirectory(test)
