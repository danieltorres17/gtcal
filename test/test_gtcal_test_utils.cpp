#include "gtcal_test_utils.hpp"
#include <gtest/gtest.h>

#include <algorithm>

struct GtcalTestUtils : public testing::Test {
protected:
  // Get grid points at first pose
  const double grid_spacing = 0.3;
  const int num_rows = 10;
  const int num_cols = 13;
};

// Tests that poses are sampled around the target.
TEST_F(GtcalTestUtils, PosesAroundTarget) {
  // Create target object.
  const gtcal::CalibrationTarget target(grid_spacing, num_cols, num_rows);
  const gtsam::Point3 target_center_pt3d = target.targetCenter();
  const double target_center_x = target_center_pt3d.x();
  const double target_center_y = target_center_pt3d.y();
  const gtsam::Point3 initial_offset = {target_center_x, target_center_y, -0.75};

  // Get synthetic poses around target.
  const auto poses_target_cam =
      gtcal::utils::GeneratePosesAroundTarget(target, -3.0, -target_center_y / 2, initial_offset);
  EXPECT_EQ(poses_target_cam.size(), 10);
}

// Tests the default pose function.
TEST_F(GtcalTestUtils, DefaultCameraPoses) {
  // Create target object.
  gtcal::CalibrationTarget target(grid_spacing, num_cols, num_rows);
  const gtsam::Point3 target_center_pt3d = target.targetCenter();
  const double target_center_x = target_center_pt3d.x();
  const double target_center_y = target_center_pt3d.y();
  const gtsam::Point3 offset0 = {target_center_x, target_center_y, -1.0};

  // Get default poses around target.
  const gtsam::Pose3 pose0_target_cam_offset = gtsam::Pose3(gtsam::Rot3(), offset0);

  const double x_offset = 1.5;
  const double y_offset = 1.1;
  const gtsam::Point3 offset1 = {target_center_x + x_offset, target_center_y + y_offset, -2.0};
  const gtsam::Pose3 pose1_target_cam_offset = gtsam::Pose3(gtsam::Rot3(), offset1);

  const gtsam::Point3 offset2 = {target_center_x + x_offset, target_center_y - y_offset, -2.0};
  const gtsam::Pose3 pose2_target_cam_offset = gtsam::Pose3(gtsam::Rot3(), offset2);

  const gtsam::Point3 offset3 = {target_center_x - x_offset, target_center_y + y_offset, -2.0};
  const gtsam::Pose3 pose3_target_cam_offset = gtsam::Pose3(gtsam::Rot3(), offset3);

  const gtsam::Point3 offset4 = {target_center_x - x_offset, target_center_y - y_offset, -2.0};
  const gtsam::Pose3 pose4_target_cam_offset = gtsam::Pose3(gtsam::Rot3(), offset4);
  const std::vector<gtsam::Pose3> poses_target_cam_offset = {pose0_target_cam_offset, pose1_target_cam_offset,
                                                             pose2_target_cam_offset, pose3_target_cam_offset,
                                                             pose4_target_cam_offset};

  const auto poses_target_cam = gtcal::utils::DefaultCameraPoses(poses_target_cam_offset);
  EXPECT_EQ(poses_target_cam.size(), 5 * poses_target_cam_offset.size());

  // Visualize (uncomment to see visualization).
  // gtcal::viz::VisualizeSetup(poses_target_cam, target.pointsTarget());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
