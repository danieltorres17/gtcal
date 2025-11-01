#pragma once

#include <random>

#include "gtcal/calibration_target.hpp"

namespace gtcal {

class RandomPoseGenerator {
public:
  struct Config {
    double x_min = -5.0;
    double x_max = 5.0;
    double y_min = -5.0;
    double y_max = 5.0;
    double z_min = 0.75;
    double z_max = 5.0;

    Config() {}
  };

  RandomPoseGenerator(const CalibrationTarget::Ptr cal_target,
                      const unsigned int seed = std::random_device{}(), const Config& config = Config{});

  size_t sampleTargetPointIndex(const size_t num_target_points);
  std::vector<gtsam::Pose3> samplePoses(const size_t num_poses);
  gtsam::Pose3 samplePose();

private:
  CalibrationTarget::Ptr cal_target_ = nullptr;
  const Config config_;
  unsigned int seed_;
  std::default_random_engine gen_;
  std::uniform_real_distribution<double> dist_x_;
  std::uniform_real_distribution<double> dist_y_;
  std::uniform_real_distribution<double> dist_z_;
  const gtsam::Point3 up_vector_ = gtsam::Point3(0., -1.0, 0.);  // Fixed up vector for camera lookat.
};

}  // namespace gtcal