#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>

#define IMAGE_WIDTH 1024
#define IMAGE_HEIGHT 570
#define FX 200
#define FY 200
#define CX 512
#define CY 235

struct TargetPoints {
  TargetPoints(const double grid_spacing, const size_t num_rows, const size_t num_cols)
    : grid_spacing(grid_spacing), num_rows(num_rows), num_cols(num_cols) {}

  const double grid_spacing;
  const size_t num_rows;
  const size_t num_cols;
};

gtsam::Point3Vector GenerateGridPts3d(const double grid_spacing, const size_t num_rows,
                                      const size_t num_cols) {
  gtsam::Point3Vector grid_pts3d;
  grid_pts3d.reserve(num_rows * num_cols);

  for (size_t jj = 0; jj < num_cols; jj++) {
    for (size_t ii = 0; ii < num_rows; ii++) {
      const gtsam::Point3 pt3d = (gtsam::Point3() << ii * grid_spacing, jj * grid_spacing, 1.0).finished();
      grid_pts3d.push_back(pt3d);
    }
  }

  return grid_pts3d;
}

gtsam::Pose3Vector GeneratePosesAroundTarget(const TargetPoints& target, const size_t num_poses) {
  gtsam::Pose3Vector poses_target_cam;
  poses_target_cam.reserve(num_poses);

  return poses_target_cam;
}
