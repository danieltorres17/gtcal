find_package(GTest REQUIRED)

add_executable(test_pose_solver test_pose_solver.cpp)
target_link_libraries(test_pose_solver GTest::GTest gtsam pose_solver)

add_executable(test_batch_solver test_batch_solver.cpp)
target_link_libraries(test_batch_solver GTest::GTest gtsam)
