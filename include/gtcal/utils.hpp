#pragma once

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose3.h>
#include <limits>

namespace gtcal {
namespace utils {

static constexpr double NaN = std::numeric_limits<double>::quiet_NaN();

static bool FilterPixelCoords(const gtsam::Point2& uv, const size_t image_width, const size_t image_height) {
  if ((uv.x() >= 0.) && (uv.y() >= 0.) && (uv.x() < image_width) && (uv.y() < image_height)) {
    return true;
  }
  return false;
}

static double DegToRad(const double degrees) { return degrees * M_PI / 180.0; }

static double RadToDeg(const double radians) { return radians * 180.0 / M_PI; }

static std::vector<gtsam::Pose3> InterpolatePoses(const std::vector<gtsam::Pose3>& trajectory_poses,
                                                  const size_t num_poses_to_interp_between) {
  std::vector<gtsam::Pose3> interpolated_poses;
  interpolated_poses.reserve(trajectory_poses.size() * num_poses_to_interp_between);

  for (size_t ii = 0; ii < trajectory_poses.size() - 1; ii++) {
    const gtsam::Pose3& pose0 = trajectory_poses.at(ii);
    const gtsam::Pose3& pose1 = trajectory_poses.at(ii + 1);
    for (size_t jj = 0; jj < num_poses_to_interp_between; jj++) {
      const double t = static_cast<double>(jj) / (num_poses_to_interp_between - 1);
      interpolated_poses.push_back(pose0.interpolateRt(pose1, t));
    }
  }

  return interpolated_poses;
}

static gtsam::Pose3 ApplyNoiseToPose(const gtsam::Pose3& pose, const double xyz_std_dev,
                                     const double rot_std_dev) {
  // Create random number generators.
  static std::default_random_engine gen(42);
  std::normal_distribution<double> xyz_dist(0.0, xyz_std_dev);
  std::normal_distribution<double> rot_dist(0.0, rot_std_dev);

  // Apply noise to translation.
  const gtsam::Vector3& xyz = pose.translation();
  const gtsam::Vector3 xyz_noisy = xyz + gtsam::Vector3{xyz_dist(gen), xyz_dist(gen), xyz_dist(gen)};

  // Apply noise to rotation.
  const gtsam::Rot3 R = pose.rotation();
  const gtsam::Rot3 R_noisy = gtsam::Rot3::Expmap({rot_dist(gen), rot_dist(gen), rot_dist(gen)}) * R;

  return gtsam::Pose3(R_noisy, xyz_noisy);
}

}  // namespace utils
}  // namespace gtcal