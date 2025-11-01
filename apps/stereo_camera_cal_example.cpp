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
  const double grid_spacing = 0.35;
  const size_t num_rows = 10;
  const size_t num_cols = 13;
  gtcal::CalibrationTarget::Ptr target =
      std::make_shared<gtcal::CalibrationTarget>(grid_spacing, num_cols, num_rows);

  // Create ground truth camera rig.
  // 1. Linear camera model.
  std::shared_ptr<gtsam::Cal3_S2> K_linear_gt =
      std::make_shared<gtsam::Cal3_S2>(500.0, 500.0, 0.0, image_width / 2.0, image_height / 2.0);
  std::shared_ptr<gtcal::Camera> camera_linear_gt = std::make_shared<gtcal::Camera>();
  camera_linear_gt->setCameraModel<gtsam::Cal3_S2>(image_width, image_height, *K_linear_gt);
  // 2. Fisheye camera model.
  std::shared_ptr<gtsam::Cal3Fisheye> K_fisheye_gt = std::make_shared<gtsam::Cal3Fisheye>(
      500.0, 500.0, 0.0, image_width / 2.0, image_height / 2.0, 0.01, -0.01, 0.005, 0.005);
  std::shared_ptr<gtcal::Camera> camera_fisheye_gt = std::make_shared<gtcal::Camera>();
  camera_fisheye_gt->setCameraModel<gtsam::Cal3Fisheye>(image_width, image_height, *K_fisheye_gt);

  std::vector<std::shared_ptr<gtcal::Camera>> cameras_gt = {camera_linear_gt, camera_fisheye_gt};
  const gtsam::Pose3 pose_linear_fisheye_gt(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0),
                                            gtsam::Point3(0.075, 0.0, 0.0));
  gtcal::CameraRig::Ptr camera_rig_gt = std::make_shared<gtcal::CameraRig>(
      cameras_gt, std::vector<gtsam::Pose3>{gtsam::Pose3(), pose_linear_fisheye_gt});

  // Sample random poses.
  const size_t num_poses_to_sample = 75;
  gtcal::RandomPoseGenerator::Config pose_gen_config;
  pose_gen_config.x_min = -0.75;
  pose_gen_config.x_max = num_cols * grid_spacing + 0.75;
  pose_gen_config.y_min = -0.75;
  pose_gen_config.y_max = num_rows * grid_spacing + 0.75;
  pose_gen_config.z_min = 0.5;
  pose_gen_config.z_max = 5.0;
  const unsigned int random_seed = 34;
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
    all_noisy_frames.push_back(gtcal::Simulator::GenerateNoisyFrames(gt_frames, 0.01, 0.1, 1.0, random_seed));
  }

  // Create camera rig with noisy calibrations.
  // 1. Linear camera model.
  std::shared_ptr<gtsam::Cal3_S2> K_linear_noisy =
      std::make_shared<gtsam::Cal3_S2>(491.0, 512.0, 0.0, (image_width / 2.0), (image_height / 2.0));
  std::shared_ptr<gtcal::Camera> camera_linear_noisy = std::make_shared<gtcal::Camera>();
  camera_linear_noisy->setCameraModel<gtsam::Cal3_S2>(image_width, image_height, *K_linear_noisy);
  // 2. Fisheye camera model.
  std::shared_ptr<gtsam::Cal3Fisheye> K_fisheye_noisy = std::make_shared<gtsam::Cal3Fisheye>(
      510.0, 492.0, 0.0, (image_width / 2.0) + 12.0, (image_height / 2.0), 0.005, -0.005, 0.0, 0.0);
  std::shared_ptr<gtcal::Camera> camera_fisheye_noisy = std::make_shared<gtcal::Camera>();
  camera_fisheye_noisy->setCameraModel<gtsam::Cal3Fisheye>(image_width, image_height, *K_fisheye_noisy);
  std::vector<std::shared_ptr<gtcal::Camera>> cameras_noisy = {camera_linear_noisy, camera_fisheye_noisy};
  const gtsam::Pose3 pose_linear_fisheye_noisy(gtsam::Rot3::RzRyRx(0.01, -0.005, 0.002),
                                               gtsam::Point3(0.072, -0.002, 0.003));
  gtcal::CameraRig::Ptr camera_rig_noisy = std::make_shared<gtcal::CameraRig>(
      cameras_noisy, std::vector<gtsam::Pose3>{gtsam::Pose3(), pose_linear_fisheye_noisy});

  // Create calibration rig from noisy calibration.
  gtcal::CalibrationRig::Ptr calibration_rig = std::make_shared<gtcal::CalibrationRig>(cameras_noisy);
  std::cout << "Number of cameras in calibration rig: " << calibration_rig->numCameras() << "\n";

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

    // Print the camera poses.
    // for (size_t cam_id = 0; cam_id < calibration_rig->numCameras(); cam_id++) {
    //   const gtsam::Pose3 pose_target_cam =
    //   calibration_rig->poses_target_camera_map.at(cam_id).at(ii).value(); std::cout << "Camera " << cam_id
    //   << " pose_target_cam after iteration " << ii << ": \n"
    //             << pose_target_cam.matrix() << "\n";
    // }
    // Print the extrinsics between cameras.
    const gtsam::Pose3 pose_target_cam0 = calibration_rig->poses_target_camera_map.at(0).at(ii).value();
    const gtsam::Pose3 pose_target_cam1 = calibration_rig->poses_target_camera_map.at(1).at(ii).value();
    const gtsam::Pose3 pose_cam0_cam1 = pose_target_cam0.between(pose_target_cam1);
    std::cout << "Extrinsics between camera 0 and 1 after iteration " << ii << ": \n"
              << pose_cam0_cam1.matrix() << "\n";

    // Plot measurements.
    cv::Mat img_plot = cv::Mat::zeros(image_height, image_width, CV_8UC3);
    // for (const auto& frame : all_noisy_frames.at(ii)) {
    //   const auto& camera = calibration_rig->cameras.at(frame.camera_id);
    //   for (const auto& meas : frame.measurements) {
    //     const cv::Point2d uv_cv(static_cast<double>(meas.uv.x()), static_cast<double>(meas.uv.y()));
    //     cv::circle(img_plot, uv_cv, 3, cv::Scalar(0, 255, 0), -1);
    //   }
    // }

    const auto est_calibration = current_estimate.at<gtsam::Cal3_S2>(calibration_rig->calibrationSymbol(0));
    est_calibration.print("Linear camera estimated calibration: ");
    const auto fisheye_est_calibration =
        current_estimate.at<gtsam::Cal3Fisheye>(calibration_rig->calibrationSymbol(1));
    fisheye_est_calibration.print("Fisheye camera estimated calibration: ");
    std::cout << "----------------------------------------\n";

    std::vector<cv::Mat> camera_imgs_plot;
    for (const auto& frame : all_gt_frames.at(ii)) {
      const auto& camera = calibration_rig->cameras.at(frame.camera_id);
      cv::Mat curr_camera_img_plot = cv::Mat::zeros(image_height, image_width, CV_8UC3);
      for (const auto& meas : frame.measurements) {
        const cv::Point2d uv_cv(static_cast<double>(meas.uv.x()), static_cast<double>(meas.uv.y()));
        cv::circle(curr_camera_img_plot, uv_cv, 3, cv::Scalar(255, 0, 255), 2);
      }
      camera_imgs_plot.push_back(curr_camera_img_plot);
    }
    // Concatenate camera images side by side.
    cv::hconcat(camera_imgs_plot, img_plot);
    cv::resize(img_plot, img_plot, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);
    cv::imshow("Measurements (green: noisy, blue: ground truth)", img_plot);
    cv::waitKey(0);
  }

  return 0;
}