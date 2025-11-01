#include "gtcal/simulator.hpp"
#include "gtcal/utils.hpp"

namespace gtcal {

Simulator::Simulator(const CalibrationTarget::Ptr cal_target, const CameraRig::Ptr camera_rig,
                     const std::vector<gtsam::Pose3>& poses_target_cam0_gt)
  : cal_target_(cal_target), camera_rig_(camera_rig), poses_target_cam0_gt_(poses_target_cam0_gt) {}

std::optional<std::vector<Simulator::Frame>> Simulator::nextFrames() {
  if (frame_counter_ >= poses_target_cam0_gt_.size()) {
    return {};
  }

  std::vector<Simulator::Frame> sim_frames;
  sim_frames.reserve(camera_rig_->numCameras());

  // Current pose of camera 0.
  const gtsam::Pose3& pose_target_cam0_curr = poses_target_cam0_gt_.at(frame_counter_);
  const auto& pts3d_target = cal_target_->pointsTarget();

  for (size_t ii = 0; ii < camera_rig_->numCameras(); ii++) {
    const auto& camera = camera_rig_->cameras.at(ii);

    // Compute pose of camera using extrinsics.
    const gtsam::Pose3 pose_target_cam_ii =
        pose_target_cam0_curr.compose(camera_rig_->camera_extrinsics.at(ii));
    camera->setCameraPose(pose_target_cam_ii);

    // Get the measurements at the current pose.
    std::vector<Measurement> measurements;
    for (size_t jj = 0; jj < pts3d_target.size(); jj++) {
      const gtsam::Point3 pt3d_t = pts3d_target.at(jj);
      const auto [uv, safe] = camera->projectSafe(pt3d_t);
      if (!safe) {
        continue;
      }

      if (utils::FilterPixelCoords(uv, camera->width(), camera->height())) {
        measurements.emplace_back(uv, ii, jj);
      }
    }
    if (measurements.empty()) {
      // No measurements for this camera at this pose, skip frame.
      std::cout << "Warning: No measurements for camera " << ii << " at frame " << frame_counter_
                << ", skipping frame.\n";
      continue;
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

std::vector<Simulator::Frame> Simulator::GenerateNoisyFrames(const std::vector<Simulator::Frame>& frames,
                                                             const double xyz_std_dev,
                                                             const double rot_std_dev,
                                                             const double meas_std_dev,
                                                             const unsigned int random_seed) {
  std::vector<Simulator::Frame> noisy_frames;
  noisy_frames.reserve(frames.size());
  for (const auto& frame : frames) {
    noisy_frames.push_back(GenerateNoisyFrame(frame, xyz_std_dev, rot_std_dev, meas_std_dev, random_seed));
  }

  return noisy_frames;
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