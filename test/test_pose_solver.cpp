#include "gtcal/pose_solver.h"
#include "gtcal_test_utils.h"
#include <gtest/gtest.h>

#include <algorithm>

TEST(PoseSolver, SinglePoseTranslationOnly) {
  // Get grid points at first pose
  const double grid_spacing = 0.15;
  const int num_rows = 10;
  const int num_cols = 13;
  const int num_target_pts = num_rows * num_cols;

  // Get target points in target frame.
  gtsam::Point3Vector target0_pts3d_target = GenerateGridPts3d(grid_spacing, num_rows, num_cols);

  // Set first and second camera poses in target frame.
  const gtsam::Pose3 pose0_target_cam =
      gtsam::Pose3(gtsam::Rot3::RzRyRx(0., 0., 0.), gtsam::Point3(0., 0., 0.));
  const gtsam::Rot3 R1 = gtsam::Rot3::RzRyRx(0., 0., 0.);
  const gtsam::Point3 xyz1 = gtsam::Point3(0., 0., 0.1);
  const gtsam::Pose3 pose1_target_cam = gtsam::Pose3(R1, xyz1);

  // Transform target points two second pose in camera frame.
  gtsam::Point3Vector target1_pts3d_cam;
  target1_pts3d_cam.reserve(num_target_pts);
  std::transform(target0_pts3d_target.begin(), target0_pts3d_target.end(),
                 std::back_inserter(target1_pts3d_cam),
                 [&pose1_target_cam](const gtsam::Point3& pt3d_target0) -> gtsam::Point3 {
                   return pose1_target_cam.transformTo(pt3d_target0);
                 });

  // Define camera model.
  auto K = boost::make_shared<gtsam::Cal3Fisheye>(FX, FY, 0., CX, CY, 0., 0., 0., 0.);
  const gtsam::Pose3 pose1_cam_target = pose1_target_cam.inverse();
  gtsam::PinholeCamera<gtsam::Cal3Fisheye> camera(pose1_target_cam, *K);

  // Get pixel measurements for each target point.
  gtsam::Point2Vector target1_uvs;
  target1_uvs.reserve(num_target_pts);
  for (size_t ii = 0; ii < num_target_pts; ii++) {
    const gtsam::Point2 uv = camera.project(target1_pts3d_cam.at(ii));
    if (gtcal::utils::FilterPixelCoords(uv, IMAGE_WIDTH, IMAGE_HEIGHT)) {
      target1_uvs.push_back(uv);
    } else {
      std::cout << "uv not in image: " << uv << "\n";
    }
  }

  // Create pose solver problem.
  gtcal::PoseSolver pose_solver;
  gtsam::Pose3 pose_target_cam_init =
      gtsam::Pose3(gtsam::Rot3::RzRyRx(0., 0.1, 0.), gtsam::Point3(0.01, -0.1, 0.09));
  const bool success = pose_solver.Solve(target1_uvs, target0_pts3d_target, K, pose_target_cam_init);
  EXPECT_TRUE(success) << "Pose solver failed.";
  const gtsam::Point3 solved_xyz = pose_target_cam_init.translation();
  const gtsam::Point3 solved_pqr = pose_target_cam_init.rotation().rpy();
  const gtsam::Point3 pqr1 = R1.rpy();
  EXPECT_NEAR(solved_xyz.x(), xyz1.x(), 1e-8);
  EXPECT_NEAR(solved_xyz.y(), xyz1.y(), 1e-8);
  EXPECT_NEAR(solved_xyz.z(), xyz1.z(), 1e-8);
  EXPECT_NEAR(solved_pqr.x(), pqr1.x(), 1e-8);
  EXPECT_NEAR(solved_pqr.y(), pqr1.y(), 1e-8);
  EXPECT_NEAR(solved_pqr.z(), pqr1.z(), 1e-8);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
