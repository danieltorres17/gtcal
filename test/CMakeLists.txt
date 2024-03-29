find_package(GTest REQUIRED)

add_executable(test_camera test_camera.cpp)
target_include_directories(test_camera PRIVATE ${GTCAL_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR})
target_link_libraries(test_camera GTest::GTest gtsam)

add_executable(test_pose_solvers test_pose_solvers.cpp)
target_include_directories(test_pose_solvers PRIVATE ${GTCAL_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR} ${CERES_INCLUDE_DIRS})
target_link_libraries(test_pose_solvers GTest::GTest gtsam pose_solver pose_solver_gtsam)

add_executable(test_batch_solver test_batch_solver.cpp)
target_include_directories(test_batch_solver PRIVATE ${GTCAL_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR})
target_link_libraries(test_batch_solver GTest::GTest gtsam batch_solver)

add_executable(test_gtcal_test_utils test_gtcal_test_utils.cpp)
target_include_directories(test_gtcal_test_utils PRIVATE ${GTCAL_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})
target_link_libraries(test_gtcal_test_utils GTest::GTest gtsam ${PCL_LIBRARIES})
