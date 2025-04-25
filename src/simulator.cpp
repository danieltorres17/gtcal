#include "gtcal/simulator.hpp"
#include "gtcal/utils.hpp"

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
  const auto& pts3d_target = cal_target_->pointsTarget();

  for (size_t ii = 0; ii < cameras_gt_.size(); ii++) {
    const auto& camera = cameras_gt_.at(ii);

    // Compute pose of camera using extrinsics.
    const gtsam::Pose3 pose_target_cam_ii = pose_target_cam0_curr.compose(poses_extrinsics_gt_.at(ii));
    camera->setCameraPose(pose_target_cam_ii);

    // Get the measurements at the current pose.
    std::vector<Measurement> measurements;
    for (size_t jj = 0; jj < pts3d_target.size(); jj++) {
      const gtsam::Point3 pt3d_t = pts3d_target.at(jj);
      const gtsam::Point2 uv = camera->project(pt3d_t);

      if (utils::FilterPixelCoords(uv, camera->width(), camera->height())) {
        measurements.emplace_back(uv, ii, jj);
      }
    }

    // Add measurements to frame.
    sim_frames.push_back({.camera_id = ii,
                          .frame_count = frame_counter_,
                          .measurements = std::move(measurements),
                          .pose_target_cam = pose_target_cam_ii});
  }

  frame_counter_++;

  return sim_frames;
}

Simulator::Frame Simulator::GenerateNoisyFrame(const Simulator::Frame& frame, const double xyz_std_dev,
                                               const double rot_std_dev, const double meas_std_dev,
                                               const unsigned int random_seed) {
  return Simulator::Frame{
      .camera_id = frame.camera_id,
      .frame_count = frame.frame_count,
      .measurements = AddNoiseToMeasurements(frame.measurements, meas_std_dev, random_seed),
      .pose_target_cam = ApplyNoiseToPose(frame.pose_target_cam, xyz_std_dev, rot_std_dev)};
}

std::vector<Measurement> Simulator::AddNoiseToMeasurements(const std::vector<Measurement>& measurements,
                                                           const double std_dev, unsigned int seed) {
  std::normal_distribution<double> dist(0.0, std_dev);
  std::default_random_engine generator(seed);

  std::vector<Measurement> noisy_measurements;
  noisy_measurements.reserve(measurements.size());

  for (const auto& meas : measurements) {
    const double noise_x = dist(generator);
    const double noise_y = dist(generator);
    noisy_measurements.emplace_back(gtsam::Point2(meas.uv.x() + noise_x, meas.uv.y() + noise_y),
                                    meas.camera_id, meas.point_id);
  }

  return noisy_measurements;
}

gtsam::Pose3 Simulator::ApplyNoiseToPose(const gtsam::Pose3& pose, const double xyz_std_dev,
                                         const double rot_std_dev) {
  return utils::ApplyNoiseToPose(pose, xyz_std_dev, rot_std_dev);
}

}  // namespace gtcal