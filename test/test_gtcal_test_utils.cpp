#include "gtcal_test_utils.h"
#include "gtcal_viz_utils.h"
#include <gtest/gtest.h>

#include <algorithm>

struct GtcalTestUtils : public testing::Test {
protected:
  // Get grid points at first pose
  const double grid_spacing = 0.3;
  const int num_rows = 10;
  const int num_cols = 13;
  const int num_target_pts = num_rows * num_cols;
};

// Tests that a calibration target object is constructed properly.
TEST_F(GtcalTestUtils, TestGenerateGridPts3d) {
  // Create target object.
  const gtcal::utils::CalibrationTarget target(grid_spacing, num_rows, num_cols);

  // Check that the grid spacing and number of rows and columns were set correctly.
  ASSERT_FLOAT_EQ(target.gridSpacing(), grid_spacing);
  ASSERT_EQ(target.numRows(), num_rows);
  ASSERT_EQ(target.numCols(), num_cols);

  // Get target points in target frame.
  ASSERT_EQ(target.pointsTarget().size(), num_rows * num_cols);

  // Define the expected target center point in target frame.
  const double x_center = (grid_spacing * num_cols) / 2;
  const double y_center = (grid_spacing * num_rows) / 2;
  const double z_center = 0.;
  const gtsam::Point3 target_3dcenter_expected = {x_center, y_center, z_center};

  // Get the center point and check that it's what we expect.
  const gtsam::Point3 target_3dcenter = target.get3dCenter();
  EXPECT_DOUBLE_EQ((target_3dcenter - target_3dcenter_expected).norm(), 0.0);
}

// Tests that poses are sampled around the target.
TEST_F(GtcalTestUtils, PosesAroundTarget) {
  // Create target object.
  const gtcal::utils::CalibrationTarget target(grid_spacing, num_rows, num_cols);
  const gtsam::Point3 target_center_pt3d = target.get3dCenter();
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
  gtcal::utils::CalibrationTarget target(grid_spacing, num_rows, num_cols);
  const gtsam::Point3 target_center_pt3d = target.get3dCenter();
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
