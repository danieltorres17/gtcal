#include <gtest/gtest.h>
#include "gtcal/calibration_target.hpp"

TEST(CalibrationTarget, GridSize) {
  const double spacing = 0.1;
  const size_t cols = 7;
  const size_t rows = 5;
  gtcal::CalibrationTarget target(spacing, cols, rows);
  EXPECT_FLOAT_EQ(target.gridSpacing(), spacing);
  EXPECT_EQ(target.numCols(), cols);
  EXPECT_EQ(target.numRows(), rows);

  const std::vector<gtsam::Point3> pts3d_target = target.pointsTarget();
  EXPECT_EQ(pts3d_target.size(), cols * rows);
  const gtsam::Point3 target_center = target.targetCenter();
  EXPECT_FLOAT_EQ(target_center.x(), spacing * (cols - 1) / 2.0);
  EXPECT_FLOAT_EQ(target_center.y(), spacing * (rows - 1) / 2.0);
  EXPECT_FLOAT_EQ(target_center.z(), 0.0);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
