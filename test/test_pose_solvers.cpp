#include "gtcal/pose_solver.h"
#include "gtcal_test_utils.h"
#include "gtcal/camera.h"
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <gtsam/slam/SmartProjectionFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include "gtcal/pose_solver_gtsam.h"
#include "gtcal/viz.hpp"

#include <gtest/gtest.h>
#include <algorithm>

using gtsam::symbol_shorthand::L;
using gtsam::symbol_shorthand::R;
using gtsam::symbol_shorthand::X;

struct BasePoseSolverFixture {
public:
  // Target grid point parameters.
  const double grid_spacing = 0.3;
  const size_t num_rows = 10;
  const size_t num_cols = 13;
  const size_t num_target_pts = num_rows * num_cols;

  // Create target object and define the target x and y center.
  gtcal::CalibrationTarget target{grid_spacing, num_cols, num_rows};
  double target_center_x = -1.0;
  double target_center_y = -1.0;
  std::vector<gtsam::Point3> target_points3d;

  // Camera and measurements vector.
  std::shared_ptr<gtcal::Camera> camera = nullptr;
  std::vector<gtcal::Measurement> measurements;

  void setUpTarget() {
    // Get the target center.
    target_points3d = target.pointsTarget();
    const gtsam::Point3 target_center_pt3d = target.targetCenter();
    target_center_x = target_center_pt3d.x();
    target_center_y = target_center_pt3d.y();
  }
};

struct TranslationOnlyFixture : public BasePoseSolverFixture, public testing::Test {
protected:
  // First two camera poses in target frame.
  const gtsam::Rot3 R_target_cam = gtsam::Rot3::RzRyRx(0., 0., 0.);
  gtsam::Point3 xyz0_target_cam, xyz1_target_cam;
  gtsam::Pose3 pose0_target_cam, pose1_target_cam;

  // Fisheye calibration.
  std::shared_ptr<gtsam::Cal3Fisheye> K = nullptr;

  void SetUp() override {
    // Set up camera model.
    setUpTarget();
    ASSERT_GT(target_center_x, 0.0);
    ASSERT_GT(target_center_y, 0.0);

    // Set up camera poses.
    xyz0_target_cam = gtsam::Point3(target_center_x, target_center_y, -0.85);
    xyz1_target_cam = gtsam::Point3(target_center_x, target_center_y, -0.825);
    pose0_target_cam = gtsam::Pose3(R_target_cam, xyz0_target_cam);
    pose1_target_cam = gtsam::Pose3(R_target_cam, xyz1_target_cam);

    // Set the camera calibration.
    K = std::make_shared<gtsam::Cal3Fisheye>(gtsam::Cal3Fisheye(FX, FY, 0., CX, CY, 0., 0., 0., 0.));

    camera = std::make_shared<gtcal::Camera>();
    camera->setCameraModel<gtsam::Cal3Fisheye>(IMAGE_WIDTH, IMAGE_HEIGHT, *K, pose1_target_cam);

    // Get measurements.
    // Get target point measurements at second pose.
    ASSERT_GT(target_points3d.size(), 0);
    // measurements.reserve(target_points3d.size());
    for (size_t ii = 0; ii < target_points3d.size(); ii++) {
      const gtsam::Point3& pt3d_target = target_points3d.at(ii);
      const gtsam::Point2 uv = camera->project(pt3d_target);
      if (gtcal::utils::FilterPixelCoords(uv, camera->width(), camera->height())) {
        measurements.push_back(gtcal::Measurement(uv, 0, ii));
      }
    }

    // Check that the number of measurements is equal to the number of 3D points (should be for this case).
    EXPECT_EQ(measurements.size(), target.pointsTarget().size());
  }
};

TEST_F(TranslationOnlyFixture, Visualizer) {
  gtcal::Visualizer viz;
  std::thread viz_thread([&]() { viz.run(); });

  for (size_t ii = 0; ii < 25; ii++) {
    viz.update(target_points3d, {pose0_target_cam});
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

// Tests that the Ceres pose solver is able to find a solution in the case of translation only.
TEST_F(TranslationOnlyFixture, CeresSinglePoseTranslationOnly) {
  // Create pose solver problem.
  gtcal::PoseSolver pose_solver(true);
  gtsam::Pose3 pose_target_cam_est = gtcal::utils::ApplyNoise(pose0_target_cam, 0.1, 0.5);
  const bool success = pose_solver.solve(measurements, target_points3d, camera, pose_target_cam_est);
  EXPECT_TRUE(success);

  // Check the estimated solution.
  EXPECT_TRUE(pose_target_cam_est.equals(pose1_target_cam, 1e-5));
}

// Tests that the gtsam pose solver is able to find a solution in the case of translation only.
TEST_F(TranslationOnlyFixture, DISABLED_GtsamSinglePoseTranslationOnly) {
  // Estimate solution.
  const gtcal::PoseSolverGtsam::Options options;
  gtcal::PoseSolverGtsam pose_solver(options);
  gtsam::Pose3 pose_target_cam_est = pose0_target_cam;
  std::cout << "pose_target_cam_est:\n" << pose_target_cam_est.matrix() << "\n";
  const bool success = pose_solver.solve(measurements, target_points3d, camera, pose_target_cam_est);
  std::cout << "pose_target_cam_est:\n" << pose_target_cam_est.matrix() << "\n";

  EXPECT_TRUE(success);
  EXPECT_TRUE(pose_target_cam_est.equals(pose1_target_cam, 1e-5));
}

struct PoseSolverFixture : public BasePoseSolverFixture, public testing::Test {
protected:
  // Fisheye calibration.
  std::shared_ptr<gtsam::Cal3Fisheye> K = nullptr;
  std::shared_ptr<gtcal::Camera> camera = nullptr;

  gtsam::Point3 initial_offset;
  std::vector<gtsam::Pose3> poses_target_cam;
  gtsam::Pose3 pose0_target_cam, pose1_target_cam;

  void SetUp() override {
    setUpTarget();
    ASSERT_GT(target_center_x, 0.0);
    ASSERT_GT(target_center_y, 0.0);

    // Set up camera offset.
    initial_offset = {target_center_x, target_center_y, -0.75};

    // Get the first two poses.
    poses_target_cam =
        gtcal::utils::GeneratePosesAroundTarget(target, -3.0, -target_center_y / 2, initial_offset);
    EXPECT_EQ(poses_target_cam.size(), 10);
    pose0_target_cam = poses_target_cam.at(0);
    pose1_target_cam = poses_target_cam.at(1);

    // Set up the camera at the second pose.
    K = std::make_shared<gtsam::Cal3Fisheye>(gtsam::Cal3Fisheye(FX, FY, 0., CX, CY, 0., 0., 0., 0.));
    camera = std::make_shared<gtcal::Camera>();
    camera->setCameraModel<gtsam::Cal3Fisheye>(IMAGE_WIDTH, IMAGE_HEIGHT, *K, pose1_target_cam);

    // Get the target point measurements.
    measurements.reserve(target_points3d.size());
    for (size_t ii = 0; ii < target_points3d.size(); ii++) {
      const gtsam::Point3& pt3d_target = target_points3d.at(ii);
      const gtsam::Point2 uv = camera->project(pt3d_target);
      if (gtcal::utils::FilterPixelCoords(uv, camera->width(), camera->height())) {
        measurements.push_back(gtcal::Measurement(uv, 0, ii));
      }
    }

    // Check that the number of measurements is equal to the number of 3D points (should be for this case).
    EXPECT_EQ(measurements.size(), target_points3d.size());
  }
};

// Tests that the Ceres pose solver is able to find a solution in the case of translation and rotation using
// the first two poses from the synthetic pose set.
TEST_F(PoseSolverFixture, DISABLED_CeresFirstAndSecondPoses) {
  // Create pose solver problem.
  gtcal::PoseSolver pose_solver(false);
  gtsam::Pose3 pose_target_cam_init = pose0_target_cam;
  const bool success = pose_solver.solve(measurements, target_points3d, camera, pose_target_cam_init);
  EXPECT_TRUE(success) << "Pose solver failed.";

  // Check estimated solution.
  EXPECT_TRUE(pose1_target_cam.equals(pose_target_cam_init, 1e-7));
}

// Tests that the gtsam pose solver is able to find a solution in the case of translation and rotation using
// the first two poses from the synthetic pose set.
TEST_F(PoseSolverFixture, DISABLED_GtsamFirstAndSecondPoses) {
  // Create pose solver problem.
  const gtcal::PoseSolverGtsam::Options options;
  gtcal::PoseSolverGtsam pose_solver(options);
  gtsam::Pose3 pose_target_cam_est = pose0_target_cam;
  const bool success = pose_solver.solve(measurements, target_points3d, camera, pose_target_cam_est);

  EXPECT_TRUE(success);
  EXPECT_TRUE(pose_target_cam_est.equals(pose1_target_cam, 1e-3));
}

TEST_F(PoseSolverFixture, DISABLED_GtsamSmartProjection) {
  // Pixel noise model.
  gtsam::noiseModel::Isotropic::shared_ptr pixel_meas_noise_model =
      gtsam::noiseModel::Isotropic::Sigma(2, 1.0);

  // Pose noise model.
  auto pose_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3::Constant(0.5), gtsam::Vector3{0.1, 0.1, 0.1}).finished());

  // Smart factors for each landmark.
  std::vector<gtsam::SmartProjectionPoseFactor<gtsam::Cal3Fisheye>::shared_ptr> smart_factors;
  smart_factors.reserve(target_points3d.size());
  for (size_t ii = 0; ii < target_points3d.size(); ii++) {
    smart_factors.push_back(
        std::make_shared<gtsam::SmartProjectionPoseFactor<gtsam::Cal3Fisheye>>(pixel_meas_noise_model, K));
  }

  // Get measurements at pose 0.
  camera->setCameraPose(pose0_target_cam);
  std::vector<gtcal::Measurement> measurements0;
  measurements0.reserve(target_points3d.size());
  for (size_t ii = 0; ii < target_points3d.size(); ii++) {
    const gtsam::Point3& pt3d_target = target_points3d.at(ii);
    const gtsam::Point2 uv = camera->project(pt3d_target);
    if (gtcal::utils::FilterPixelCoords(uv, camera->width(), camera->height())) {
      measurements0.push_back(gtcal::Measurement(uv, 0, ii));
    }
  }
  // Expect the number of observations to match the number of target points for the first pose.
  EXPECT_EQ(measurements0.size(), target_points3d.size());

  // Get measurements at pose 1.
  camera->setCameraPose(pose1_target_cam);
  std::vector<gtcal::Measurement> measurements1;
  measurements1.reserve(target_points3d.size());
  for (size_t ii = 0; ii < target_points3d.size(); ii++) {
    const gtsam::Point3& pt3d_target = target_points3d.at(ii);
    const gtsam::Point2 uv = camera->project(pt3d_target);
    if (gtcal::utils::FilterPixelCoords(uv, camera->width(), camera->height())) {
      measurements1.push_back(gtcal::Measurement(uv, 0, ii));
    }
  }
  // Expect the number of observations to match the number of target points for the first pose.
  EXPECT_EQ(measurements1.size(), target_points3d.size());

  // Create optimization problem.
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;

  // Add camera pose prior to graph and initial estimate to values.
  gtsam::Pose3 delta(gtsam::Rot3::Rodrigues(-0.033, 0.02, -0.05), gtsam::Point3(0.01, -0.016, 0.005));
  values.insert(X(0), pose0_target_cam.compose(delta));
  values.insert(X(1), pose1_target_cam.compose(delta));
  // graph.addPrior(X(0), pose0_target_cam);
  // graph.addPrior(X(1), pose1_target_cam, pose_noise);

  // Add first pose observations to factors.
  for (size_t ii = 0; ii < measurements0.size(); ii++) {
    const size_t pt3d_idx = measurements0.at(ii).point_id;
    smart_factors.at(pt3d_idx)->add(measurements0.at(ii).uv, X(0));
  }
  // Add second pose observations to factors.
  for (size_t ii = 0; ii < measurements1.size(); ii++) {
    const size_t pt3d_idx = measurements1.at(ii).point_id;
    smart_factors.at(pt3d_idx)->add(measurements1.at(ii).uv, X(1));
  }

  for (const auto& sf : smart_factors) {
    graph.push_back(sf);
  }

  // Solve problem.
  gtsam::LevenbergMarquardtParams params;
  gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, values, params).optimize();
  std::cout << "Ground truth x0:\n" << pose0_target_cam.matrix() << "\n";
  std::cout << "Ground truth x1:\n" << pose1_target_cam.matrix() << "\n";
  result.print();

  // // Compare to the ground truth.
  // const gtsam::Pose3 pose_est_gt_delta =
  //     pose0_target_cam.inverse().compose(result.at<gtsam::Pose3>(camera_key));
  // const gtsam::Vector6 pose_est_gt_delta_vec = gtcal::utils::PoseToVector(pose_est_gt_delta);
  // std::cout << "pose0_target_cam:\n" << pose0_target_cam.matrix() << std::endl;
  // std::cout << "Pose delta vec:\n" << pose_est_gt_delta_vec.transpose() << std::endl;
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
