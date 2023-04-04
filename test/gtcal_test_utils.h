#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>

#define IMAGE_WIDTH 1024
#define IMAGE_HEIGHT 570
#define FX 200
#define FY 200
#define CX 512
#define CY 235

namespace gtcal {
namespace utils {

struct CalibrationTarget {
  CalibrationTarget(const double grid_spacing, const size_t num_rows, const size_t num_cols)
    : grid_spacing(grid_spacing)
    , num_rows(num_rows)
    , num_cols(num_cols)
    , grid_pts3d_target(generateGridPts3d(grid_spacing, num_rows, num_cols)) {}

  const double grid_spacing;
  const size_t num_rows;
  const size_t num_cols;
  const gtsam::Point3Vector grid_pts3d_target;

  /**
   * @brief Return the target center point in target frame so the z-coordinate is at 0.0.
   *
   * @return gtsam::Point3
   */
  gtsam::Point3 get3dCenter() const {
    const double x_center = (grid_spacing * num_cols) / 2;
    const double y_center = (grid_spacing * num_rows) / 2;

    return {x_center, y_center, 0.0};
  }

  /**
   * @brief Return vector with 3D points of calibration target in target frame using the target's grid
   * spacing, number of rows and columns. TODO: probably just make this a static method.
   *
   * @param grid_spacing spacing between target points.
   * @param num_rows  number of rows.
   * @param num_cols number of cols.
   * @return gtsam::Point3Vector
   */
  static gtsam::Point3Vector generateGridPts3d(const double grid_spacing, const size_t num_rows,
                                               const size_t num_cols) {
    gtsam::Point3Vector grid_pts3d;
    grid_pts3d.reserve(num_rows * num_cols);

    for (size_t jj = 0; jj < num_cols; jj++) {
      for (size_t ii = 0; ii < num_rows; ii++) {
        const gtsam::Point3 pt3d = (gtsam::Point3() << jj * grid_spacing, ii * grid_spacing, 0.0).finished();
        grid_pts3d.push_back(pt3d);
      }
    }

    return grid_pts3d;
  }
};

static gtsam::Pose3Vector GeneratePosesAroundTarget(const CalibrationTarget& target, const size_t num_poses) {
  gtsam::Pose3Vector poses_target_cam;
  poses_target_cam.reserve(num_poses);

  // Get target center offset.
  // const gtsam::Point3 target_center_offset =

  return poses_target_cam;
}

}  // namespace utils
}  // namespace gtcal
