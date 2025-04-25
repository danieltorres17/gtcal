#include <gtest/gtest.h>

#include "gtcal_test_utils.hpp"
#include "gtcal/simulator.hpp"
#include "gtcal/calibration_ctx.hpp"
#include "gtcal/viewer.hpp"

#include <opencv2/opencv.hpp>

struct SimulatorFixture : public testing::Test {
protected:
  // Target grid point parameters.
  const double grid_spacing = 0.2;
  const size_t num_rows = 10;
  const size_t num_cols = 13;
  const size_t num_target_pts = num_rows * num_cols;
  std::shared_ptr<gtcal::CalibrationTarget> target = nullptr;

  // Camera poses.
  const gtsam::Rot3 R0_target_cam = gtsam::Rot3::RzRyRx(0., 0., 0.);
  const gtsam::Rot3 R1_target_cam = gtsam::Rot3::RzRyRx(0., 0., 0.);
  gtsam::Point3 xyz0_target_cam;
  gtsam::Pose3 pose0_target_cam;
  gtsam::Point3 xyz1_target_cam;
  gtsam::Pose3 pose1_target_cam;
  std::vector<gtsam::Pose3> poses_target_cam0_gt;

  // Camera calibration, models and extrinsics.
  std::shared_ptr<gtsam::Cal3_S2> K_cal3_s2 = nullptr;
  std::shared_ptr<gtcal::Camera> linear_camera0 = nullptr;
  std::shared_ptr<gtcal::Camera> linear_camera1 = nullptr;
  const gtsam::Rot3 R_camera0_camera1 = gtsam::Rot3::RzRyRx(0., 0., 0.);
  const gtsam::Pose3 pose_camera0_camera1 = gtsam::Pose3(R_camera0_camera1, gtsam::Point3(0.25, 0.1, -0.1));
  std::vector<gtsam::Pose3> extrinsics_gt;
  std::vector<std::shared_ptr<gtcal::Camera>> cameras_vec;

  void SetUp() override {
    // Initialize calibration, models and extrinsics vector.
    K_cal3_s2 = std::make_shared<gtsam::Cal3_S2>(FX, FY, 0., CX, CY);
    linear_camera0 = std::make_shared<gtcal::Camera>();
    linear_camera0->setCameraModel<gtsam::Cal3_S2>(IMAGE_WIDTH, IMAGE_HEIGHT, *K_cal3_s2);
    linear_camera1 = std::make_shared<gtcal::Camera>();
    linear_camera1->setCameraModel<gtsam::Cal3_S2>(IMAGE_WIDTH, IMAGE_HEIGHT, *K_cal3_s2);
    extrinsics_gt = {gtsam::Pose3(), pose_camera0_camera1};
    cameras_vec = {linear_camera0, linear_camera1};

    // Set up target and camera 0 ground truth poses in the target frame.
    target = std::make_shared<gtcal::CalibrationTarget>(grid_spacing, num_cols, num_rows);
    const gtsam::Point3 target_center_pt3d = target->targetCenter();
    const double target_center_x = target_center_pt3d.x();
    const double target_center_y = target_center_pt3d.y();
    xyz0_target_cam = gtsam::Point3(target_center_x, target_center_y, -1.5);
    pose0_target_cam = gtsam::Pose3(R0_target_cam, xyz0_target_cam);
    xyz1_target_cam = gtsam::Point3(target_center_x, target_center_y, -1.25);
    pose1_target_cam = gtsam::Pose3(R1_target_cam, xyz1_target_cam);
    poses_target_cam0_gt = {pose0_target_cam, pose1_target_cam};
  }
};

TEST_F(SimulatorFixture, SimInitialization) {
  gtcal::CameraRig::Ptr camera_rig = std::make_shared<gtcal::CameraRig>(cameras_vec, extrinsics_gt);
  gtcal::Simulator sim(target, camera_rig, poses_target_cam0_gt);
  EXPECT_EQ(sim.numFrames(), poses_target_cam0_gt.size());
  EXPECT_EQ(sim.frameCounter(), 0);
}

TEST_F(SimulatorFixture, GetFrames) {
  gtcal::CameraRig::Ptr camera_rig = std::make_shared<gtcal::CameraRig>(cameras_vec, extrinsics_gt);
  gtcal::Simulator sim(target, camera_rig, poses_target_cam0_gt);
  EXPECT_EQ(sim.frameCounter(), 0);

  // Get first set of frames.
  const std::optional<std::vector<gtcal::Simulator::Frame>> frames0_opt = sim.nextFrames();
  EXPECT_TRUE(frames0_opt);
  const std::vector<gtcal::Simulator::Frame>& frames0 = *frames0_opt;
  EXPECT_EQ(frames0.size(), poses_target_cam0_gt.size());

  // Check first camera frame values.
  EXPECT_EQ(frames0.at(0).camera_id, 0);
  EXPECT_EQ(frames0.at(0).measurements.size(), target->pointsTarget().size());
  EXPECT_TRUE(frames0.at(0).pose_target_cam.equals(poses_target_cam0_gt.at(0)));

  // Check second camera frame values.
  EXPECT_EQ(frames0.at(1).camera_id, 1);
  EXPECT_EQ(frames0.at(1).measurements.size(), target->pointsTarget().size());
  EXPECT_TRUE(frames0.at(1).pose_target_cam.equals(poses_target_cam0_gt.at(0).compose(pose_camera0_camera1)));

  // Check frame counter.
  EXPECT_EQ(frames0.at(0).frame_count, 0);
  EXPECT_EQ(frames0.at(0).frame_count, frames0.at(1).frame_count);
  EXPECT_EQ(sim.frameCounter(), 1);

  // Plot camera detections.
  cv::Mat image0 = cv::Mat::zeros(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
  for (const auto& meas : frames0.at(0).measurements) {
    cv::circle(image0, cv::Point2f(meas.uv.x(), meas.uv.y()), 5, cv::Scalar(255, 0, 255), -1);
  }
  cv::Mat image1 = cv::Mat::zeros(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
  for (const auto& meas : frames0.at(1).measurements) {
    cv::circle(image1, cv::Point2f(meas.uv.x(), meas.uv.y()), 5, cv::Scalar(255, 0, 255), -1);
  }
  cv::Mat concat;
  cv::hconcat(image0, image1, concat);
  cv::imshow("Frame 0 Camera Detections", concat);
  cv::waitKey(0);

  // Get next set of frames.
  const std::optional<std::vector<gtcal::Simulator::Frame>> frames1_opt = sim.nextFrames();
  EXPECT_TRUE(frames1_opt);
  const std::vector<gtcal::Simulator::Frame>& frames1 = *frames1_opt;
  EXPECT_EQ(frames0.size(), frames1.size());

  // Check first camera frame values at next frame.
  EXPECT_EQ(frames1.at(0).camera_id, 0);
  EXPECT_EQ(frames1.at(0).measurements.size(), target->pointsTarget().size());
  EXPECT_TRUE(frames1.at(0).pose_target_cam.equals(poses_target_cam0_gt.at(1)));

  // Check second camera frame values.
  EXPECT_EQ(frames1.at(1).camera_id, 1);
  EXPECT_EQ(frames1.at(1).measurements.size(), target->pointsTarget().size());
  EXPECT_TRUE(frames1.at(1).pose_target_cam.equals(poses_target_cam0_gt.at(1).compose(pose_camera0_camera1)));

  // Check frame counter was updated correctly.
  EXPECT_EQ(frames1.at(0).frame_count, 1);
  EXPECT_EQ(frames1.at(0).frame_count, frames1.at(1).frame_count);
  EXPECT_EQ(sim.frameCounter(), 2);

  // Now the sim should return nullopt.
  const auto frames2_opt = sim.nextFrames();
  EXPECT_FALSE(frames2_opt);
  EXPECT_EQ(sim.frameCounter(), 2);

  // Test the reset method.
  sim.reset();
  EXPECT_EQ(sim.frameCounter(), 0);
}

TEST_F(SimulatorFixture, DISABLED_CalibrationCtx) {
  // Initialize calibration context.
  std::vector<gtcal::Camera::Ptr> camera_rig_vec{cameras_vec.at(0)};
  std::vector<gtsam::Pose3> camera_rig_extrinsics_vec{gtsam::Pose3()};
  gtcal::CameraRig::Ptr camera_rig =
      std::make_shared<gtcal::CameraRig>(camera_rig_vec, camera_rig_extrinsics_vec);
  gtcal::CalibrationCtx ctx(camera_rig, target);

  // Initialize simulator.
  gtcal::Simulator sim(target, camera_rig, std::vector<gtsam::Pose3>{poses_target_cam0_gt.at(0)});

  // Get frames from sim.
  const auto frames0_opt = sim.nextFrames();
  ASSERT_TRUE(frames0_opt);
  const auto& frames0 = *frames0_opt;
  EXPECT_EQ(frames0.size(), 1);

  // Attempt to run optimization.
  gtsam::Pose3 delta(gtsam::Rot3::RzRyRx(0.2, -0.1, -0.4), gtsam::Point3(-0.156, -0.072, 0.1));
  const gtsam::Pose3 pose_est_initial = frames0.at(0).pose_target_cam.compose(delta);
  gtcal::CalibrationCtx::Frame ctx_frame(frames0.at(0).camera_id, frames0.at(0).measurements,
                                         pose_est_initial);
  ctx.processFrames(std::vector<gtcal::CalibrationCtx::Frame>{ctx_frame});
  const auto& ctx_state = ctx.state();
  const auto& estimate = ctx_state->current_estimate;
  estimate.print();
  std::cout << "pose_est_initial:\n" << pose_est_initial.matrix() << "\n";
}

TEST_F(SimulatorFixture, DISABLED_Viewer) {
  gtcal::Viewer::Ptr viewer = std::make_shared<gtcal::Viewer>();

  // Initialize simulator.
  std::vector<gtcal::Camera::Ptr> camera_rig_vec{cameras_vec.at(0)};
  std::vector<gtsam::Pose3> camera_rig_extrinsics_vec{gtsam::Pose3()};
  gtcal::CameraRig::Ptr camera_rig = std::make_shared<gtcal::CameraRig>(cameras_vec, extrinsics_gt);
  gtcal::Simulator sim(target, camera_rig, poses_target_cam0_gt);

  // Use viewer to visualize the simulator.
  viewer->update(target->pointsTarget());
  viewer->update(poses_target_cam0_gt);

  viewer->close();
}

TEST(SimulatorInterp, Interpolation) {
  gtsam::Pose3 pose0;
  gtsam::Pose3 pose1(gtsam::Rot3::RzRyRx(0., 0., 0.), gtsam::Point3(1.0, 0., 0.));
  gtsam::Pose3 pose2(gtsam::Rot3::RzRyRx(0., 0., 0.), gtsam::Point3(2.0, 0., 0.));
  const size_t num_poses = 7;
  std::vector<gtsam::Pose3> poses{pose0, pose1, pose2};

  const std::vector<gtsam::Pose3> poses_traj = gtcal::utils::InterpolatePoses(poses, num_poses);
  ASSERT_TRUE(poses_traj.front().compose(pose0.inverse()).equals(gtsam::Pose3()));
  ASSERT_TRUE(poses_traj.back().compose(pose2.inverse()).equals(gtsam::Pose3()));
  EXPECT_EQ(poses_traj.size(), (num_poses) * (poses.size() - 1));

  gtcal::Viewer::Ptr viewer = std::make_shared<gtcal::Viewer>();

  // Initialize simulator.
  // std::vector<gtcal::Camera::Ptr> camera_rig_vec{cameras_vec.at(0)};
  // std::vector<gtsam::Pose3> camera_rig_extrinsics_vec{gtsam::Pose3()};
  // gtcal::Simulator sim(target, camera_rig_extrinsics_vec, poses_target_cam0_gt, camera_rig_vec);

  // Use viewer to visualize the simulator.
  // viewer->update(target->pointsTarget());
  viewer->update(poses_traj);

  viewer->close();
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
