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
  const double grid_spacing = 0.25;
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
  gtcal::CameraRig::Ptr camera_rig_gt =
      std::make_shared<gtcal::CameraRig>(cameras_gt, std::vector<gtsam::Pose3>{gtsam::Pose3()});

  // Sample random poses.
  const size_t num_poses_to_sample = 50;
  gtcal::RandomPoseGenerator::Config pose_gen_config;
  pose_gen_config.x_min = -0.75;
  pose_gen_config.x_max = num_cols * grid_spacing + 0.75;
  pose_gen_config.y_min = -0.75;
  pose_gen_config.y_max = num_rows * grid_spacing + 0.75;
  pose_gen_config.z_min = 1.0;
  pose_gen_config.z_max = 3.0;
  const unsigned int random_seed = 29;
  gtcal::RandomPoseGenerator pose_gen(target, /*seed=*/random_seed, pose_gen_config);
  const std::vector<gtsam::Pose3> sampled_poses_target_cam0_gt = pose_gen.samplePoses(num_poses_to_sample);

  // Create simulator using the ground truth camera rig.
  gtcal::Simulator simulator(target, camera_rig_gt, sampled_poses_target_cam0_gt);

  auto SimFramesToCtxFrame =  // Utility function to convert simulator frames to calibration context frames.
      [](const std::vector<gtcal::Simulator::Frame>& frames) -> std::vector<gtcal::CalibrationCtx::Frame> {
    std::vector<gtcal::CalibrationCtx::Frame> ctx_frames;
    ctx_frames.reserve(frames.size());
    for (const auto& frame : frames) {
      ctx_frames.emplace_back(frame.camera_id, frame.measurements, frame.pose_target_cam);
    }
    return ctx_frames;
  };

  // Generate ground truth frames once.
  const std::vector<std::vector<gtcal::Simulator::Frame>> all_gt_frames = simulator.allFrames();

  // Create noisy simulator frames.
  std::vector<std::vector<gtcal::Simulator::Frame>> all_noisy_frames;
  all_noisy_frames.reserve(all_gt_frames.size());
  for (const auto& gt_frames : all_gt_frames) {
    all_noisy_frames.push_back(
        gtcal::Simulator::GenerateNoisyFrames(gt_frames, 0.025, 0.2, 1.0, random_seed));
  }

  // Create camera rig with noisy calibration.
  std::shared_ptr<gtsam::Cal3_S2> K_noisy = std::make_shared<gtsam::Cal3_S2>(
      480.0, 512.0, 0.0, (image_width / 2.0) + 17.0, (image_height / 2.0) - 12.0);
  std::shared_ptr<gtcal::Camera> camera_noisy = std::make_shared<gtcal::Camera>();
  camera_noisy->setCameraModel<gtsam::Cal3_S2>(image_width, image_height, *K_noisy);
  std::vector<std::shared_ptr<gtcal::Camera>> cameras_noisy = {camera_noisy};
  gtcal::CameraRig::Ptr camera_rig_noisy =
      std::make_shared<gtcal::CameraRig>(cameras_noisy, std::vector<gtsam::Pose3>{gtsam::Pose3()});

  // Create calibration rig with noisy calibration.
  gtcal::CalibrationRig::Ptr calibration_rig = std::make_shared<gtcal::CalibrationRig>(cameras_noisy);

  // Calibration context.
  gtcal::CalibrationCtx ctx(calibration_rig, target);

  for (size_t ii = 0; ii < all_noisy_frames.size(); ii++) {
    // Convert the simulator frames to calibration context frames.
    const std::vector<gtcal::CalibrationCtx::Frame> ctx_frames_gt = SimFramesToCtxFrame(all_gt_frames.at(ii));
    const std::vector<gtcal::CalibrationCtx::Frame> ctx_frame = SimFramesToCtxFrame(all_noisy_frames.at(ii));

    // Process frames.
    const gtsam::Values current_estimate = ctx.processFrames(ctx_frame);
    ctx.updateCameraRigCalibrations(current_estimate);
    ctx.updateCameraPosesInTargetFrame(current_estimate);

    // Get state from ctx and print calibration and pose info.
    const gtsam::Values all_poses = gtsam::utilities::allPose3s(current_estimate);
    for (size_t jj = 0; jj < all_poses.size(); jj++) {
      const gtsam::Pose3 est_pose = all_poses.at<gtsam::Pose3>(calibration_rig->cameraPoseSymbol(0, jj));
      const gtsam::Pose3 gt_pose = sampled_poses_target_cam0_gt.at(jj);
      const gtsam::Pose3 delta_pose = gt_pose.between(est_pose);
      // std::cout << "Pose " << jj << " delta: " << delta_pose.translation().transpose()
      //           << delta_pose.rotation().rpy().transpose() << "\n";
    }
    const auto est_calibration = current_estimate.at<gtsam::Cal3_S2>(calibration_rig->calibrationSymbol(0));
    est_calibration.print("Estimated calibration: ");

    // // Plot measurements.
    // cv::Mat img_plot = cv::Mat::zeros(image_height, image_width, CV_8UC3);
    // for (const auto& frame : all_noisy_frames.at(ii)) {
    //   const auto& camera = calibration_rig->cameras.at(frame.camera_id);
    //   for (const auto& meas : frame.measurements) {
    //     const cv::Point2d uv_cv(static_cast<double>(meas.uv.x()), static_cast<double>(meas.uv.y()));
    //     cv::circle(img_plot, uv_cv, 3, cv::Scalar(0, 255, 0), -1);
    //   }
    // }
    // for (const auto& frame : all_gt_frames.at(ii)) {
    //   const auto& camera = calibration_rig->cameras.at(frame.camera_id);
    //   for (const auto& meas : frame.measurements) {
    //     const cv::Point2d uv_cv(static_cast<double>(meas.uv.x()), static_cast<double>(meas.uv.y()));
    //     cv::circle(img_plot, uv_cv, 3, cv::Scalar(255, 0, 0), 1);
    //   }
    // }
    // cv::imshow("Measurements (green: noisy, blue: ground truth)", img_plot);
    // cv::waitKey(0);
  }

  return 0;
}
