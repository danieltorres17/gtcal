#include "gtcal_test_utils.h"
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

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
