#include <iostream>

#include "gtcal/calibration_target.hpp"
#include "gtcal/calibration_rig.hpp"
#include "gtcal/random_pose_generator.hpp"
#include "gtcal/simulator.hpp"
#include "gtcal/calibration_ctx.hpp"

#include <gtsam/nonlinear/utilities.h>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv) {
  // Image dimensions.
  const int image_width = 1280;
  const int image_height = 720;

  // Create target.
  const double grid_spacing = 0.2;
  const size_t num_rows = 10;
  const size_t num_cols = 13;
  gtcal::CalibrationTarget::Ptr target =
      std::make_shared<gtcal::CalibrationTarget>(grid_spacing, num_cols, num_rows);

  // Create ground truth camera rig.
  std::shared_ptr<gtsam::Cal3_S2> K_gt =
      std::make_shared<gtsam::Cal3_S2>(500.0, 500.0, 0.0, image_width / 2.0, image_height / 2.0);
  std::shared_ptr<gtcal::Camera> camera_gt = std::make_shared<gtcal::Camera>();
  camera_gt->setCameraModel<gtsam::Cal3_S2>(image_width, image_height, *K_gt);
  std::vector<std::shared_ptr<gtcal::Camera>> cameras_gt = {camera_gt};
  std::shared_ptr<gtcal::CalibrationRig> calibration_rig =
      std::make_shared<gtcal::CalibrationRig>(cameras_gt);
  gtcal::CameraRig::Ptr camera_rig =
      std::make_shared<gtcal::CameraRig>(cameras_gt, std::vector<gtsam::Pose3>{gtsam::Pose3()});

  // Sample random poses.
  const size_t num_poses_to_sample = 25;
  gtcal::RandomPoseGenerator::Config pose_gen_config;
  pose_gen_config.x_min = -0.75;
  pose_gen_config.x_max = num_cols * grid_spacing + 0.75;
  pose_gen_config.y_min = -0.75;
  pose_gen_config.y_max = num_rows * grid_spacing + 0.75;
  pose_gen_config.z_min = 1.0;
  pose_gen_config.z_max = 3.0;
  const unsigned int random_seed = 29;
  gtcal::RandomPoseGenerator pose_gen(target, /*seed=*/random_seed, pose_gen_config);
  const std::vector<gtsam::Pose3> sampled_poses = pose_gen.samplePoses(num_poses_to_sample);

  // Create simulator.
  gtcal::Simulator simulator(target, camera_rig, sampled_poses);

  // Calibration context.
  gtcal::CalibrationCtx ctx(calibration_rig, target);

  auto SimFramesToCtxFrame =
      [](const std::vector<gtcal::Simulator::Frame>& frames) -> std::vector<gtcal::CalibrationCtx::Frame> {
    std::vector<gtcal::CalibrationCtx::Frame> ctx_frames;
    ctx_frames.reserve(frames.size());
    for (const auto& frame : frames) {
      ctx_frames.emplace_back(frame.camera_id, frame.measurements, frame.pose_target_cam);
    }
    return ctx_frames;
  };

  for (size_t ii = 0; ii < simulator.numFrames(); ii++) {
    const auto frames_opt = simulator.nextFrames();
    if (!frames_opt.has_value()) {
      break;
    }

    // Generate noisy frames.
    const auto& frames = *frames_opt;
    const auto noisy_frames = gtcal::Simulator::GenerateNoisyFrames(frames, 0.025, 0.2, 1.0, random_seed);

    std::cout << "Processing frame set " << ii << " / " << simulator.numFrames() - 1 << "\n";

    // Convert to CalibrationCtx frames and print some info.
    const std::vector<gtcal::CalibrationCtx::Frame> ctx_frames = SimFramesToCtxFrame(noisy_frames);

    // Process frames.
    const gtsam::Values current_estimate = ctx.processFrames(ctx_frames);
    ctx.updateCameraRigCalibrations(current_estimate);
    ctx.updateCameraPosesInTargetFrame(current_estimate);

    // Get state from ctx and print calibration and pose info.
    const gtsam::Values all_poses = gtsam::utilities::allPose3s(current_estimate);
    for (size_t ii = 0; ii < all_poses.size(); ii++) {
      const gtsam::Pose3 est_pose = all_poses.at<gtsam::Pose3>(gtsam::Symbol('a', ii));
      const gtsam::Pose3 gt_pose = sampled_poses.at(ii);
      const gtsam::Pose3 delta_pose = gt_pose.between(est_pose);
      std::cout << "Pose " << ii << " delta: " << delta_pose.translation().transpose()
                << delta_pose.rotation().rpy().transpose() << "\n";
    }

    // Plot measurements.
    cv::Mat img_plot = cv::Mat::zeros(image_height, image_width, CV_8UC3);
    for (const auto& frame : noisy_frames) {
      const auto& camera = calibration_rig->cameras.at(frame.camera_id);
      for (const auto& meas : frame.measurements) {
        const cv::Point2d uv_cv(static_cast<double>(meas.uv.x()), static_cast<double>(meas.uv.y()));
        cv::circle(img_plot, uv_cv, 3, cv::Scalar(0, 255, 0), -1);
      }
    }
    for (const auto& frame : frames) {
      const auto& camera = calibration_rig->cameras.at(frame.camera_id);
      for (const auto& meas : frame.measurements) {
        const cv::Point2d uv_cv(static_cast<double>(meas.uv.x()), static_cast<double>(meas.uv.y()));
        cv::circle(img_plot, uv_cv, 3, cv::Scalar(255, 0, 0), 1);
      }
    }
    cv::imshow("Measurements (green: noisy, blue: ground truth)", img_plot);
    cv::waitKey(0);
  }

  return 0;
}
