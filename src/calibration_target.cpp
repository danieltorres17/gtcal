#include "gtcal/calibration_target.hpp"

namespace gtcal {

CalibrationTarget::CalibrationTarget(const double grid_spacing, const size_t num_cols, const size_t num_rows)
  : grid_spacing_(grid_spacing), num_cols_(num_cols), num_rows_(num_rows),
    grid_pts3d_target_(generateGridPts3d(grid_spacing, num_cols, num_rows)) {}

gtsam::Point3 CalibrationTarget::targetCenter() const {
  const double xc = grid_spacing_ * (num_cols_ - 1) / 2.0;
  const double yc = grid_spacing_ * (num_rows_ - 1) / 2.0;

  return gtsam::Point3{xc, yc, z_coord_};
}

std::vector<gtsam::Point3> CalibrationTarget::generateGridPts3d(const double grid_spacing,
                                                                const size_t num_cols,
                                                                const size_t num_rows) const {
  std::vector<gtsam::Point3> grid_pts3d;
  grid_pts3d.reserve(num_rows * num_cols);

  for (size_t ii = 0; ii < num_rows; ii++) {
    for (size_t jj = 0; jj < num_cols; jj++) {
      grid_pts3d.emplace_back(jj * grid_spacing, ii * grid_spacing, z_coord_);
    }
  }

  return grid_pts3d;
}

}  // namespace gtcal