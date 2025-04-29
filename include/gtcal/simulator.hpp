#pragma once

#include "gtcal/camera_rig.hpp"
#include "gtcal/calibration_target.hpp"
#include "gtcal/measurement.hpp"

namespace gtcal {

class Simulator {
public:
  struct Frame {
    size_t camera_id;
    size_t frame_count;
    std::vector<Measurement> measurements;
    gtsam::Pose3 pose_target_cam = gtsam::Pose3();
  };

  Simulator(const CalibrationTarget::Ptr cal_target, const CameraRig::Ptr camera_rig,
            const std::vector<gtsam::Pose3>& poses_target_cam0_gt);

  std::optional<std::vector<Frame>> nextFrames();
  void reset() { frame_counter_ = 0; }
  size_t frameCounter() const { return frame_counter_; }
  size_t numFrames() const { return poses_target_cam0_gt_.size(); }

  static Frame GenerateNoisyFrame(const Frame& frame, const double xyz_std_dev = 0.01,
                                  const double rot_std_dev = 0.1, const double meas_std_dev = 1.0,
                                  const unsigned int random_seed = 42);
  static std::vector<Frame> GenerateNoisyFrames(const std::vector<Frame>& frames, const double xyz_std_dev = 0.01,
                                  const double rot_std_dev = 0.1, const double meas_std_dev = 1.0,
                                  const unsigned int random_seed = 42);
  static std::vector<Measurement> AddNoiseToMeasurements(const std::vector<Measurement>& measurements,
                                                         const double std_dev = 1.0, unsigned int seed = 42);
  static gtsam::Pose3 ApplyNoiseToPose(const gtsam::Pose3& pose, const double xyz_std_dev,
                                       const double rot_std_dev);

private:
  size_t frame_counter_ = 0;
  CalibrationTarget::Ptr cal_target_ = nullptr;
  CameraRig::Ptr camera_rig_ = nullptr;
  const std::vector<gtsam::Pose3> poses_target_cam0_gt_;
};

}  // namespace gtcal