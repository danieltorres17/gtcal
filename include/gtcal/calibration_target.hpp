#pragma once

#include <gtsam/geometry/Pose3.h>

namespace gtcal {

class CalibrationTarget {
public:
  using Ptr = std::shared_ptr<CalibrationTarget>;

  CalibrationTarget(const double grid_spacing, const size_t num_cols, const size_t num_rows);

  const std::vector<gtsam::Point3>& pointsTarget() const { return grid_pts3d_target_; }
  double gridSpacing() const { return grid_spacing_; }
  size_t numCols() const { return num_cols_; }
  size_t numRows() const { return num_rows_; }
  gtsam::Point3 targetCenter() const;

private:
  std::vector<gtsam::Point3> generateGridPts3d(const double grid_spacing, const size_t num_cols,
                                               const size_t num_rows) const;

private:
  const double grid_spacing_;
  const size_t num_cols_;
  const size_t num_rows_;
  const std::vector<gtsam::Point3> grid_pts3d_target_;
  const double z_coord_ = 0.0;
};

}  // namespace gtcal