#pragma once

#include <gtsam/geometry/Pose3.h>
#include <iomanip>
#include "gtcal/utils.hpp"
#include "gtcal/calibration_target.hpp"

#define IMAGE_WIDTH 1024
#define IMAGE_HEIGHT 570
#define FX 200
#define FY 200
#define CX 512
#define CY 235

namespace gtcal {
namespace utils {

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

static gtsam::Pose3 ApplyNoise(const gtsam::Pose3& pose, const double xyz_std_dev, const double rot_std_dev) {
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

gtsam::Vector6 PoseToVector(const gtsam::Pose3& pose) {
  const gtsam::Vector3& xyz_vec = pose.translation();
  const gtsam::Vector3 rot_vec = pose.rotation().rpy();
  return (gtsam::Vector6() << xyz_vec.x(), xyz_vec.y(), xyz_vec.z(), rot_vec.x(), rot_vec.y(), rot_vec.z())
      .finished();
}

std::string PoseVectorFmt(const gtsam::Vector6& pose_vec) {
  std::stringstream ss;
  ss << std::fixed << std::setprecision(4) << pose_vec.transpose();
  return ss.str();
}

std::vector<gtsam::Pose3> DefaultCameraPoses(const std::vector<gtsam::Pose3>& poses_target_cam_offset) {
  // Define first camera poses directly in front of target.
  const gtsam::Rot3 R0_target_cam = gtsam::Rot3::RzRyRx(0., 0., 0.);
  const gtsam::Point3 xyz0_target_cam = {0., 0., 0.};
  gtsam::Pose3 pose0_target_cam = gtsam::Pose3(R0_target_cam, xyz0_target_cam);

  const gtsam::Rot3 R1_target_cam = gtsam::Rot3::RzRyRx(0., -gtcal::utils::DegToRad(25.0), 0.);
  const gtsam::Point3 xyz1_target_cam = {0., 0., 0.};
  gtsam::Pose3 pose1_target_cam = gtsam::Pose3(R1_target_cam, xyz1_target_cam);

  const gtsam::Rot3 R2_target_cam = gtsam::Rot3::RzRyRx(0., gtcal::utils::DegToRad(25.0), 0.);
  const gtsam::Point3 xyz2_target_cam = {0., 0., 0.};
  gtsam::Pose3 pose2_target_cam = gtsam::Pose3(R2_target_cam, xyz2_target_cam);

  const gtsam::Rot3 R3_target_cam = gtsam::Rot3::RzRyRx(-gtcal::utils::DegToRad(25.0), 0., 0.);
  const gtsam::Point3 xyz3_target_cam = {0., 0., 0.};
  gtsam::Pose3 pose3_target_cam = gtsam::Pose3(R3_target_cam, xyz3_target_cam);

  const gtsam::Rot3 R4_target_cam = gtsam::Rot3::RzRyRx(gtcal::utils::DegToRad(25.0), 0., 0.);
  const gtsam::Point3 xyz4_target_cam = {0., 0., 0.};
  gtsam::Pose3 pose4_target_cam = gtsam::Pose3(R4_target_cam, xyz4_target_cam);

  // Base poses to apply offsets to.
  std::vector<gtsam::Pose3> poses_target_cam_base = {pose0_target_cam, pose1_target_cam, pose2_target_cam,
                                                     pose3_target_cam, pose4_target_cam};

  // Apply offsets to poses.
  std::vector<gtsam::Pose3> poses_target_cam;
  poses_target_cam.reserve(poses_target_cam_base.size() * poses_target_cam_offset.size());

  for (const auto& pose_t_c_o : poses_target_cam_offset) {
    for (const auto& pose_t_c : poses_target_cam_base) {
      poses_target_cam.push_back(pose_t_c_o.compose(pose_t_c));
    }
  }

  return poses_target_cam;
}

}  // namespace utils
}  // namespace gtcal
