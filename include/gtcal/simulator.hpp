#pragma once

#include "gtcal/camera.hpp"
#include "gtcal/calibration_target.hpp"
#include "gtcal/measurement.hpp"

namespace gtcal {

class Simulator {
public:
  struct Frame {
    size_t camera_id;
    size_t frame_count;
    std::vector<Measurement> measurements;
    gtsam::Pose3 pose_target_cam_gt = gtsam::Pose3();
  };

  Simulator(const std::shared_ptr<CalibrationTarget> cal_target,
            const std::vector<gtsam::Pose3>& poses_extrinsics_gt,
            const std::vector<gtsam::Pose3>& poses_target_cam0_gt,
            const std::vector<std::shared_ptr<Camera>>& cameras_gt);

  std::optional<std::vector<Frame>> nextFrames();
  void reset() { frame_counter_ = 0; }
  size_t frameCounter() const { return frame_counter_; }
  size_t numFrames() const { return poses_target_cam0_gt_.size(); }

private:
  size_t frame_counter_ = 0;
  std::shared_ptr<CalibrationTarget> cal_target_ = nullptr;
  const std::vector<gtsam::Pose3> poses_extrinsics_gt_;
  const std::vector<gtsam::Pose3> poses_target_cam0_gt_;
  const std::vector<std::shared_ptr<Camera>> cameras_gt_;
};

}  // namespace gtcal