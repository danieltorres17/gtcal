find_package(GTest REQUIRED)

add_executable(test_pose_solver test_pose_solver.cpp)
target_link_libraries(test_pose_solver GTest::GTest gtsam pose_solver)

add_executable(test_batch_solver test_batch_solver.cpp)
target_link_libraries(test_batch_solver GTest::GTest gtsam batch_solver)

add_executable(test_camera_rig test_camera_rig.cpp)
target_link_libraries(test_camera_rig GTest::GTest gtsam camera_rig)

add_executable(test_gtcal_test_utils test_gtcal_test_utils.cpp)
target_link_libraries(test_gtcal_test_utils GTest::GTest gtsam)
