#include "gtcal/utils.h"
#include "gtcal_test_utils.h"
#include <gtest/gtest.h>

#include <gtsam/geometry/PinholeCamera.h>
#include "gtcal/pose_solver_gtsam.h"

#include <vector>
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

  // Create camera model.
  auto K = boost::make_shared<gtsam::Cal3Fisheye>(FX, FY, 0., CX, CY, 0., 0., 0., 0.);
  gtsam::PinholeCamera<gtsam::Cal3Fisheye> camera(pose1_target_cam, *K);

  // Get measurements at second pose.
  std::vector<gtcal::Measurement> measurements;
  measurements.reserve(target.grid_pts3d_target.size());
  for (size_t ii = 0; ii < target.grid_pts3d_target.size(); ii++) {
    const gtsam::Point3 pt3d_target = target.grid_pts3d_target.at(ii);
    const gtsam::Point2 uv = camera.project(pt3d_target);
    if (gtcal::utils::FilterPixelCoords(uv, IMAGE_WIDTH, IMAGE_HEIGHT)) {
      measurements.push_back(gtcal::Measurement(uv, 0, ii));
    }
  }
  // Check that the number of measurements is equal to the number of 3D points.
  EXPECT_EQ(measurements.size(), target.grid_pts3d_target.size());

  // Estimate solution.
  const gtcal::PoseSolverGtsam::Options options;
  gtcal::PoseSolverGtsam pose_solver(options);
  const gtsam::Pose3 pose_target_cam_estimated =
      pose_solver.solve(pose0_target_cam, measurements, target.grid_pts3d_target, K);
  pose_target_cam_estimated.print("pose_target_cam_estimated estimated:\n");

  EXPECT_TRUE(pose_target_cam_estimated.equals(pose1_target_cam, 1e-5));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
