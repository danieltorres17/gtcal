find_package(GTest REQUIRED)

add_executable(test_camera test_camera.cpp)
target_link_libraries(test_camera GTest::GTest gtsam)

add_executable(test_pose_solver test_pose_solver.cpp)
target_link_libraries(test_pose_solver GTest::GTest gtsam pose_solver)

add_executable(test_batch_solver test_batch_solver.cpp)
target_link_libraries(test_batch_solver GTest::GTest gtsam)

add_executable(test_gtcal_test_utils test_gtcal_test_utils.cpp)
target_link_libraries(test_gtcal_test_utils GTest::GTest gtsam)

add_executable(test_pose_solver_gtsam test_pose_solver_gtsam.cpp)
target_link_libraries(test_pose_solver_gtsam GTest::GTest gtsam pose_solver_gtsam)
