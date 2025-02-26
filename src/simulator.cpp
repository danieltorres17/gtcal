#include "gtcal/simulator.hpp"

namespace gtcal {

Simulator::Simulator(const std::shared_ptr<CalibrationTarget> cal_target,
                     const std::vector<gtsam::Pose3>& poses_extrinsics_gt,
                     const std::vector<gtsam::Pose3>& poses_target_cam0_gt,
                     const std::vector<std::shared_ptr<Camera>>& cameras_gt)
  : cal_target_(cal_target), poses_extrinsics_gt_(poses_extrinsics_gt),
    poses_target_cam0_gt_(poses_target_cam0_gt), cameras_gt_(cameras_gt) {}

std::optional<std::vector<Simulator::Frame>> Simulator::nextFrames() {
  if (frame_counter_ >= poses_target_cam0_gt_.size()) {
    return {};
  }

  std::vector<Simulator::Frame> sim_frames;
  sim_frames.reserve(cameras_gt_.size());

  // Current pose of camera 0.
  const gtsam::Pose3& pose_target_cam0_curr = poses_target_cam0_gt_.at(frame_counter_);

  for (size_t ii = 0; ii < cameras_gt_.size(); ii++) {
    const auto& camera = cameras_gt_.at(ii);

    // Compute pose of camera using extrinsics.
    const gtsam::Pose3 pose_target_cam_ii = pose_target_cam0_curr.compose(poses_extrinsics_gt_.at(ii));
    camera->setCameraPose(pose_target_cam_ii);

    // Get the measurements at the current pose.
    std::vector<Measurement> measurements;
    const auto& pts3d_target = cal_target_->pointsTarget();
    for (size_t jj = 0; jj < pts3d_target.size(); jj++) {
      const gtsam::Point3 pt3d_t = pts3d_target.at(jj);
      const gtsam::Point2 uv = camera->project(pt3d_t);

      if (utils::FilterPixelCoords(uv, camera->width(), camera->height())) {
        measurements.emplace_back(uv, ii, jj);
      }
    }

    // Add measurements to frame.
    Simulator::Frame frame;
    frame.camera_id = ii;
    frame.frame_count = frame_counter_;
    frame.measurements = std::move(measurements);
    frame.pose_target_cam_gt = pose_target_cam_ii;
    sim_frames.push_back(frame);
  }

  frame_counter_++;

  return sim_frames;
}

}  // namespace gtcal