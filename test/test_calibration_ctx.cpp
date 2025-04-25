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

TEST_F(Simple3PoseScenario, Simulator) {
  // Create camera rig.
  gtcal::CameraRig::Ptr camera_rig = std::make_shared<gtcal::CameraRig>(
      cameras_vec, std::vector<gtsam::Pose3>{gtsam::Pose3(), pose_cam0_cam1});

  // Create calibration context.
  gtcal::CalibrationCtx ctx(camera_rig, target);

  // Create Simulator object.
  gtcal::Simulator sim(target, camera_rig, poses_target_cam0_gt);
  std::cout << "cameras_vec.at(0).use_count: " << cameras_vec.at(0).use_count() << "\n";

  // Get simulator results.
  const auto frames0_opt = sim.nextFrames();
  ASSERT_TRUE(frames0_opt);
  const std::vector<gtcal::Simulator::Frame>& frames0 = *frames0_opt;
  EXPECT_EQ(frames0.size(), camera_rig->numCameras());

  // Generate noisy simulator results.
  std::vector<gtcal::Simulator::Frame> frames0_noisy;
  frames0_noisy.reserve(frames0.size());
  for (const auto& frame : frames0) {
    const auto noisy_frame = gtcal::Simulator::GenerateNoisyFrame(frame);
    frames0_noisy.push_back(noisy_frame);
  }

  auto SimFramesToCtxFrame =
      [](const std::vector<gtcal::Simulator::Frame>& frames) -> std::vector<gtcal::CalibrationCtx::Frame> {
    std::vector<gtcal::CalibrationCtx::Frame> ctx_frames;
    ctx_frames.reserve(frames.size());
    for (const auto& frame : frames) {
      ctx_frames.emplace_back(frame.camera_id, frame.measurements, frame.pose_target_cam);
    }
    return ctx_frames;
  };

  // Create calibration ctx frames.
  const std::vector<gtcal::CalibrationCtx::Frame> ctx_frames0 = SimFramesToCtxFrame(frames0_noisy);
  std::cout << "ctx_frames0.size(): " << ctx_frames0.size() << "\n";

  // Process frames.
  const gtsam::Values estimate0 = ctx.processFrames(ctx_frames0);
  const auto keys_set = estimate0.keys();
  for (const auto& key : keys_set) {
    const gtsam::Symbol key_sym(key);
    if (key_sym.chr() == 'l') {
      continue;
    }
    if (key_sym.chr() == 'k') {
      std::cout << "Symbol: " << key_sym << "\n";
      continue;
    }
    std::cout << "Symbol: " << key_sym << ", camera id: " << camera_rig->cameraPoseKeyToIndex(key_sym)
              << "\n";
  }

  for (size_t ii = 0; ii < frames0_noisy.size(); ii++) {
    std::cout << "camera id: " << frames0_noisy.at(ii).camera_id
              << ", pose id: " << frames0_noisy.at(ii).frame_count << "\n";
    std::cout << "Noisy frame initial pose: \n" << frames0_noisy.at(ii).pose_target_cam.matrix() << "\n";
    std::cout << "Ground truth pose: \n" << frames0.at(ii).pose_target_cam.matrix() << "\n";
    std::cout
        << "Estimated pose: \n"
        << estimate0.at<gtsam::Pose3>(camera_rig->cameraPoseKey(frames0_noisy.at(ii).camera_id, 0)).matrix()
        << "\n\n";
  }
  std::cout << "Estimated camera 0 calibration: \n" << estimate0.at<gtsam::Cal3_S2>(K(0)) << "\n";
  std::cout << "Estimated camera 1 calibration: \n" << estimate0.at<gtsam::Cal3_S2>(K(1)) << "\n";

  // Update the camera rig with the estimated camera calibrations.
  for (size_t ii = 0; ii < camera_rig->numCameras(); ii++) {
    camera_rig->cameras.at(ii)->setCameraModel<gtsam::Cal3_S2>(IMAGE_WIDTH, IMAGE_HEIGHT,
                                                               estimate0.at<gtsam::Cal3_S2>(K(ii)));
  }

  // Update the estimated camera extrinsics.
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
