#include <gtest/gtest.h>

#include "gtcal_test_utils.hpp"
#include "gtcal/simulator.hpp"
#include "gtcal/calibration_ctx.hpp"
#include "gtcal/viewer.hpp"
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/utilities.h>

#include <opencv2/opencv.hpp>

using gtsam::symbol_shorthand::A;
using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::C;
using gtsam::symbol_shorthand::K;
using gtsam::symbol_shorthand::L;
using gtsam::symbol_shorthand::X;

struct SimulatorFixture : public testing::Test {
protected:
  // Target grid point parameters.
  const double grid_spacing = 0.2;
  const size_t num_rows = 10;
  const size_t num_cols = 13;
  const size_t num_target_pts = num_rows * num_cols;
  std::shared_ptr<gtcal::CalibrationTarget> target = nullptr;

  // Camera utils.
  std::shared_ptr<gtsam::Cal3_S2> K_cal3_s2 = nullptr;
  std::vector<std::shared_ptr<gtcal::Camera>> cameras_vec;
  std::shared_ptr<gtcal::Camera> linear_camera0 = nullptr;
  std::shared_ptr<gtcal::Camera> linear_camera1 = nullptr;
  gtsam::Pose3 pose_cam0_cam1 = gtsam::Pose3(gtsam::Rot3::RzRyRx(0., 0., 0.), gtsam::Point3(0.1, 0., 0.));

  void initializeTarget() {
    target = std::make_shared<gtcal::CalibrationTarget>(grid_spacing, num_cols, num_rows);
  }

  void initializeCameras() {
    K_cal3_s2 = std::make_shared<gtsam::Cal3_S2>(FX, FY, 0., CX, CY);
    linear_camera0 = std::make_shared<gtcal::Camera>();
    linear_camera0->setCameraModel<gtsam::Cal3_S2>(IMAGE_WIDTH, IMAGE_HEIGHT, *K_cal3_s2);
    linear_camera1 = std::make_shared<gtcal::Camera>();
    linear_camera1->setCameraModel<gtsam::Cal3_S2>(IMAGE_WIDTH, IMAGE_HEIGHT, *K_cal3_s2);
    cameras_vec = {linear_camera0, linear_camera1};
  }

  void SetUp() override {
    target = std::make_shared<gtcal::CalibrationTarget>(grid_spacing, num_cols, num_rows);
  }
};

struct Simple3PoseScenario : public SimulatorFixture {
protected:
  std::vector<gtsam::Pose3> poses_target_cam0_gt;
  std::vector<gtcal::Camera::Ptr> camera_rig_vec;

  void SetUp() override {
    initializeTarget();
    initializeCameras();

    // Get target center point.
    const gtsam::Point3 target_center_pt3d = target->targetCenter();
    const double x_offset = 1.0;

    // Set up pose0 to be at the left of the target looking at the center.
    const gtsam::Pose3 pose0_target_cam = gtsam::PinholeCamera<gtsam::Cal3_S2>::LookatPose(
        gtsam::Point3(target_center_pt3d.x() - x_offset, target_center_pt3d.y(),
                      target_center_pt3d.z() - 1.25),
        target_center_pt3d, gtsam::Point3(0., -1.0, 0.));

    // Set up pose1 to be at the center of the target looking at the center.
    const gtsam::Pose3 pose1_target_cam = gtsam::PinholeCamera<gtsam::Cal3_S2>::LookatPose(
        gtsam::Point3(target_center_pt3d.x(), target_center_pt3d.y(), target_center_pt3d.z() - 1.5),
        target_center_pt3d, gtsam::Point3(0., -1.0, 0.));

    // Set up pose2 to be to the right of the target looking at the center.
    const gtsam::Pose3 pose2_target_cam = gtsam::PinholeCamera<gtsam::Cal3_S2>::LookatPose(
        gtsam::Point3(target_center_pt3d.x() + x_offset, target_center_pt3d.y(),
                      target_center_pt3d.z() - 1.25),
        target_center_pt3d, gtsam::Point3(0., -1.0, 0.));

    poses_target_cam0_gt = {pose0_target_cam, pose1_target_cam, pose2_target_cam};
  }
};

TEST_F(Simple3PoseScenario, Simulator3Pose) {
  // Create camera rigs.
  gtcal::CameraRig::Ptr camera_rig_gt = std::make_shared<gtcal::CameraRig>(
      cameras_vec, std::vector<gtsam::Pose3>{gtsam::Pose3(), pose_cam0_cam1});
  gtcal::CalibrationRig::Ptr calibration_rig = std::make_shared<gtcal::CalibrationRig>(cameras_vec);

  // Create calibration context.
  gtcal::CalibrationCtx ctx(calibration_rig, target);

  // Create Simulator object.
  gtcal::Simulator sim(target, camera_rig_gt, poses_target_cam0_gt);

  auto SimFramesToCtxFrame =
      [](const std::vector<gtcal::Simulator::Frame>& frames) -> std::vector<gtcal::CalibrationCtx::Frame> {
    std::vector<gtcal::CalibrationCtx::Frame> ctx_frames;
    ctx_frames.reserve(frames.size());
    for (const auto& frame : frames) {
      ctx_frames.emplace_back(frame.camera_id, frame.measurements, frame.pose_target_cam);
    }
    return ctx_frames;
  };

  for (size_t ii = 0; ii < poses_target_cam0_gt.size(); ii++) {
    std::cout << "Frame: " << ii << "\n";
    const auto frames_opt = sim.nextFrames();
    ASSERT_TRUE(frames_opt);

    // Get simulator frame and create noisy frame.
    const std::vector<gtcal::Simulator::Frame>& frames = *frames_opt;
    const std::vector<gtcal::Simulator::Frame> frames_noisy = gtcal::Simulator::GenerateNoisyFrames(frames);

    // Convert simulator frames to ctx frames.
    const std::vector<gtcal::CalibrationCtx::Frame> ctx_frames = SimFramesToCtxFrame(frames_noisy);

    // Process frames.
    const gtsam::Values estimate = ctx.processFrames(ctx_frames);
    ctx.updateCameraRigCalibrations(estimate);
    ctx.updateCameraPosesInTargetFrame(estimate);
    calibration_rig->cameras.at(0)->printCalibration();
    calibration_rig->cameras.at(1)->printCalibration();

    // Print camera poses in target frame.
    for (const auto& [k, v] : calibration_rig->poses_target_camera_map) {
      EXPECT_EQ(v.size(), ii + 1);
      for (size_t jj = 0; jj < v.size(); jj++) {
        const auto& pose_opt = v.at(jj);
        if (pose_opt) {
          std::cout << "pose_" << jj << "target_camera_" << k << ": \n" << pose_opt->matrix() << "\n";
        } else {
          std::cout << "Pose: nullopt\n";
        }
      }
    }

    // Print camera extrinsics.
    const auto pose_target_cam0_opt = calibration_rig->poses_target_camera_map.at(0).at(ii);
    const auto pose_target_cam1_opt = calibration_rig->poses_target_camera_map.at(1).at(ii);
    ASSERT_TRUE(pose_target_cam0_opt);
    ASSERT_TRUE(pose_target_cam1_opt);
    const gtsam::Pose3 pose_target_cam0 = pose_target_cam0_opt.value();
    const gtsam::Pose3 pose_target_cam1 = pose_target_cam1_opt.value();
    const gtsam::Pose3 extrinsics = pose_target_cam0.inverse().compose(pose_target_cam1);
    std::cout << "Estimated camera extrinsics:\n" << extrinsics.matrix() << "\n\n";
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
