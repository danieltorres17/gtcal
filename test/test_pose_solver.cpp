#include "gtcal/pose_solver.h"
#include "gtcal_test_utils.h"
#include <gtest/gtest.h>

#include <algorithm>

struct PoseSolverFixture : public testing::Test {
protected:
  // Target grid point parameters.
  const double grid_spacing = 0.3;
  const size_t num_rows = 10;
  const size_t num_cols = 13;
  const size_t num_target_pts = num_rows * num_cols;

  void SetUp() override {}
};

TEST_F(PoseSolverFixture, SinglePoseTranslationOnly) {
  // Create target object.
  const gtcal::utils::CalibrationTarget target(grid_spacing, num_rows, num_cols);
  const gtsam::Point3 target_center_pt3d = target.get3dCenter();
  const double target_center_x = target_center_pt3d.x();
  const double target_center_y = target_center_pt3d.y();

  // First camera pose in target frame.
  const gtsam::Rot3 R0_target_cam = gtsam::Rot3::RzRyRx(0., 0., 0.);
  const gtsam::Point3 xyz0_target_cam = gtsam::Point3(target_center_x, target_center_y, -0.85);
  const gtsam::Pose3 pose0_target_cam = gtsam::Pose3(R0_target_cam, xyz0_target_cam);

  // Second camera pose in target frame.
  const gtsam::Rot3 R1_target_cam = gtsam::Rot3::RzRyRx(0., 0., 0.);
  const gtsam::Point3 xyz1_target_cam = gtsam::Point3(target_center_x, target_center_y, -0.825);
  const gtsam::Pose3 pose1_target_cam = gtsam::Pose3(R1_target_cam, xyz1_target_cam);

  // Transform grid points at second pose into camera frame.
  gtsam::Point3Vector target_pts3d_cam_pose0;
  target_pts3d_cam_pose0.reserve(target.grid_pts3d_target.size());
  const gtsam::Pose3 pose0_cam_target = pose0_target_cam.inverse();  // Target in camera frame at pose 1.
  for (const auto& pt3d_target : target.grid_pts3d_target) {
    const gtsam::Point3 pt3d_cam = pose0_cam_target.transformFrom(pt3d_target);
    EXPECT_FLOAT_EQ(-xyz0_target_cam.z(), pt3d_cam.z());
    target_pts3d_cam_pose0.emplace_back(pt3d_cam);
  }

  // Create camera model.
  auto K = boost::make_shared<gtsam::Cal3Fisheye>(FX, FY, 0., CX, CY, 0., 0., 0., 0.);
  gtsam::PinholeCamera<gtsam::Cal3Fisheye> camera(pose1_target_cam, *K);

  // Get target point measurements at second pose.
  gtsam::Point2Vector uvs_pose1;
  uvs_pose1.reserve(target.grid_pts3d_target.size());
  for (const auto& pt3d_target : target.grid_pts3d_target) {
    const gtsam::Point2 uv = camera.project(pt3d_target);
    if (gtcal::utils::FilterPixelCoords(uv, IMAGE_WIDTH, IMAGE_HEIGHT)) {
      uvs_pose1.emplace_back(uv);
    }
  }
  // Check that the number of measurements is equal to the number of 3D points.
  EXPECT_EQ(uvs_pose1.size(), target.grid_pts3d_target.size());

  // Create pose solver problem.
  gtcal::PoseSolver pose_solver(true);
  gtsam::Pose3 pose_target_cam_init = pose0_target_cam;
      // gtsam::Pose3(gtsam::Rot3::RzRyRx(0.001, -0.0002, 0.01), gtsam::Point3(-0.002, 0.0, -0.81));
  const bool success = pose_solver.Solve(uvs_pose1, target_pts3d_cam_pose0, K, pose_target_cam_init);
  ASSERT_TRUE(success) << "Pose solver failed.";
  std::cout << "pose_target_cam_init " << pose_target_cam_init << "\n";

  // Check the estimated solution.
  const gtsam::Point3 solved_xyz_target_cam = pose_target_cam_init.translation();
  const gtsam::Point3 solved_pqr_target_cam = pose_target_cam_init.rotation().rpy();
  const gtsam::Point3 rpy1_target_cam = R1_target_cam.rpy();
  EXPECT_NEAR((solved_xyz_target_cam - xyz1_target_cam).norm(), 0.0, 1e-8);
  EXPECT_NEAR((solved_pqr_target_cam - rpy1_target_cam).norm(), 0.0, 1e-8);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
