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

class CalibrationTarget {
public:
  CalibrationTarget(const double grid_spacing, const size_t num_rows, const size_t num_cols)
    : grid_spacing_(grid_spacing), num_rows_(num_rows), num_cols_(num_cols),
      grid_pts3d_target_(generateGridPts3d(grid_spacing, num_rows, num_cols)) {}

  /**
   * @brief Return target 3D points in the target frame.
   *
   * @return const gtsam::Point3Vector&
   */
  const gtsam::Point3Vector& pointsTarget() const { return grid_pts3d_target_; }

  /**
   * @brief Return the grid spacing.
   *
   * @return double
   */
  double gridSpacing() const { return grid_spacing_; }

  /**
   * @brief Return the number of rows.
   *
   * @return size_t
   */
  size_t numRows() const { return num_rows_; }

  /**
   * @brief Return the number of columns.
   *
   * @return size_t
   */
  size_t numCols() const { return num_cols_; }

  /**
   * @brief Return the target center point in target frame with a z-coordinate = 0.0.
   *
   * @return gtsam::Point3
   */
  gtsam::Point3 get3dCenter() const {
    const double x_center = (grid_spacing_ * num_cols_) / 2;
    const double y_center = (grid_spacing_ * num_rows_) / 2;

    return {x_center, y_center, z_coordinate_};
  }

private:
  /**
   * @brief Return vector with 3D points of calibration target in target frame using the target's grid
   * spacing, number of rows and columns.
   *
   * @param grid_spacing spacing between target points.
   * @param num_rows  number of rows.
   * @param num_cols number of cols.
   * @return gtsam::Point3Vector
   */
  gtsam::Point3Vector generateGridPts3d(const double grid_spacing, const size_t num_rows,
                                        const size_t num_cols) {
    gtsam::Point3Vector grid_pts3d;
    grid_pts3d.reserve(num_rows * num_cols);

    for (size_t jj = 0; jj < num_cols; jj++) {
      for (size_t ii = 0; ii < num_rows; ii++) {
        const gtsam::Point3 pt3d =
            (gtsam::Point3() << jj * grid_spacing, ii * grid_spacing, z_coordinate_).finished();
        grid_pts3d.push_back(pt3d);
      }
    }

    return grid_pts3d;
  }

private:
  const double grid_spacing_;
  const size_t num_rows_;
  const size_t num_cols_;
  const gtsam::Point3Vector grid_pts3d_target_;
  const double z_coordinate_ = 0.0;
};

static gtsam::Pose3Vector GeneratePosesAroundTarget(const CalibrationTarget& target, const double radius,
                                                    const double y_dist,
                                                    const gtsam::Point3& initial_offset) {
  // Define bounds and theta angles.
  const double theta_min_rad = -M_PI_4;
  const double theta_max_rad = M_PI_4;
  const size_t num_poses = 10;
  const double theta_delta_rad = (theta_max_rad - theta_min_rad) / (num_poses - 1);
  std::vector<double> thetas_rad;
  for (size_t ii = 0; ii < num_poses; ii++) {
    thetas_rad.push_back(theta_min_rad + ii * theta_delta_rad);
  }

  // Get target center offset.
  const gtsam::Point3 target_center_pt3d = target.get3dCenter();
  const double target_center_x = target_center_pt3d.x();
  const double target_center_y = target_center_pt3d.y();

  // Generate poses.
  gtsam::Pose3Vector poses_target_cam;
  poses_target_cam.reserve(num_poses);
  for (size_t ii = 0; ii < num_poses; ii++) {
    const double& theta = thetas_rad.at(ii);
    const gtsam::Rot3 R_target_cam = gtsam::Rot3::RzRyRx(0., thetas_rad.at(ii), 0.);
    gtsam::Point3 xyz_target_cam = {radius * std::sin(theta), y_dist, radius * std::cos(theta)};
    xyz_target_cam += initial_offset;
    const gtsam::Pose3 pose_target_cam = gtsam::Pose3(R_target_cam, xyz_target_cam);
    poses_target_cam.push_back(pose_target_cam);
  }

  return poses_target_cam;
}

}  // namespace utils
}  // namespace gtcal
