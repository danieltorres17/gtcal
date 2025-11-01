#include "gtcal/random_pose_generator.hpp"

#include <gtsam/geometry/PinholePose.h>

namespace gtcal {

RandomPoseGenerator::RandomPoseGenerator(const CalibrationTarget::Ptr cal_target, const unsigned int seed,
                                         const Config& config)
  : cal_target_(cal_target), seed_(seed), config_(config), gen_(seed_), dist_x_(config.x_min, config.x_max),
    dist_y_(config.y_min, config.y_max), dist_z_(config.z_min, config.z_max) {}

size_t RandomPoseGenerator::sampleTargetPointIndex(const size_t num_target_points) {
  std::uniform_int_distribution<size_t> dist(0, num_target_points - 1);

  return dist(gen_);
}

std::vector<gtsam::Pose3> RandomPoseGenerator::samplePoses(const size_t num_poses) {
  std::vector<gtsam::Pose3> poses_target_cam;
  poses_target_cam.reserve(num_poses);

  for (size_t ii = 0; ii < num_poses; ii++) {
    poses_target_cam.push_back(samplePose());
  }

  return poses_target_cam;
}

gtsam::Pose3 RandomPoseGenerator::samplePose() {
  // Generate eye position.
  const double eye_x = dist_x_(gen_);
  const double eye_y = dist_y_(gen_);
  const double eye_z = dist_z_(gen_);
  const gtsam::Point3 eye(eye_x, eye_y, eye_z);

  // Sample target point index.
  const size_t num_target_points = cal_target_->pointsTarget().size();
  const size_t sampled_pt_index = sampleTargetPointIndex(num_target_points);
  const gtsam::Point3 sampled_pt3d = cal_target_->pointsTarget().at(sampled_pt_index);
  const gtsam::Point3 target_pt = sampled_pt3d;

  return gtsam::PinholeBase::LookatPose(eye, target_pt, up_vector_);
}

}  // namespace gtcal