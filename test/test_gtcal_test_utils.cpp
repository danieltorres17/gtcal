#include "gtcal_test_utils.h"
#include <gtest/gtest.h>

#include <algorithm>

TEST(TargetPointGeneration, TestGenerateGridPts3d) {
  // Get grid points at first pose
  const double grid_spacing = 0.1;
  const int num_rows = 10;
  const int num_cols = 10;
  const int num_target_pts = num_rows * num_cols;

  // Create target object.
  const gtcal::utils::CalibrationTarget target(grid_spacing, num_rows, num_cols);

  // Check that the grid spacing and number of rows and columns were set correctly.
  ASSERT_FLOAT_EQ(target.grid_spacing, grid_spacing);
  ASSERT_EQ(target.num_rows, num_rows);
  ASSERT_EQ(target.num_cols, num_cols);

  // Get target points in target frame.
  ASSERT_EQ(target.grid_pts3d_target.size(), num_rows * num_cols);

  // Define the expected target center point in target frame.
  const double x_center = (grid_spacing * num_cols) / 2;
  const double y_center = (grid_spacing * num_rows) / 2;
  const double z_center = 0.;
  const gtsam::Point3 target_3dcenter_expected = {x_center, y_center, z_center};

  // Get the center point and check that it's what we expect.
  const gtsam::Point3 target_3dcenter = target.get3dCenter();
  EXPECT_DOUBLE_EQ((target_3dcenter - target_3dcenter_expected).norm(), 0.0);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
