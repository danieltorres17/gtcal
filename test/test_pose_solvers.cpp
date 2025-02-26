#include "gtcal/pose_solver.hpp"
#include "gtcal_test_utils.hpp"
#include "gtcal/camera.hpp"
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <gtsam/slam/SmartProjectionFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include "gtcal/pose_solver_gtsam.hpp"
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
TEST_F(TranslationOnlyFixture, GtsamSinglePoseTranslationOnly) {
  // Estimate solution.
  const gtcal::PoseSolverGtsam::Options options;
  gtcal::PoseSolverGtsam pose_solver(options);
  gtsam::Pose3 pose_target_cam_initial = pose0_target_cam;
  const auto pose_target_cam_est_opt =
      pose_solver.solve(measurements, target_points3d, camera, pose_target_cam_initial);
  EXPECT_TRUE(pose_target_cam_est_opt);
  const gtsam::Pose3& pose_target_cam_est = *pose_target_cam_est_opt;
  EXPECT_TRUE(pose_target_cam_est.equals(pose1_target_cam, 1e-3));
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
TEST_F(PoseSolverFixture, CeresFirstAndSecondPoses) {
  // Create pose solver problem.
  gtcal::PoseSolver pose_solver(false);
  gtsam::Pose3 pose_target_cam_init = pose0_target_cam;
  const bool success = pose_solver.solve(measurements, target_points3d, camera, pose_target_cam_init);
  EXPECT_TRUE(success) << "Pose solver failed.";

  // Check estimated solution.
  EXPECT_TRUE(pose1_target_cam.equals(pose_target_cam_init, 1e-7));
}

TEST_F(PoseSolverFixture, GenericProjectionFactor) {
  // Get measurements at pose 0.
  camera->setCameraPose(pose0_target_cam);
  std::vector<gtcal::Measurement> meas0;
  meas0.reserve(target_points3d.size());

  for (size_t ii = 0; ii < target_points3d.size(); ii++) {
    const gtsam::Point3& pt3d_target = target_points3d.at(ii);
    const gtsam::Point2 uv = camera->project(pt3d_target);
    if (gtcal::utils::FilterPixelCoords(uv, camera->width(), camera->height())) {
      meas0.emplace_back(uv, 0, ii);
    }
  }

  gtsam::NonlinearFactorGraph graph;
  gtsam::Pose3 delta(gtsam::Rot3::RzRyRx(0., 0., -0.4), gtsam::Point3(-0.156, 0.0, 0.1));
  gtsam::Pose3 pose_initial_est = pose0_target_cam.compose(delta);
  std::cout << "pose_initial_est:\n" << pose_initial_est.matrix() << "\n";
  gtsam::Values initial_estimate;
  initial_estimate.insert(X(0), pose_initial_est);
  auto meas_noise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);

  for (size_t ii = 0; ii < meas0.size(); ii++) {
    graph.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3Fisheye>>(
        meas0.at(ii).uv, meas_noise, X(0), L(ii), K);
    graph.emplace_shared<gtsam::NonlinearEquality<gtsam::Point3>>(L(ii), target_points3d.at(ii));
    if (!initial_estimate.exists(L(ii))) {
      initial_estimate.insert(L(ii), target_points3d.at(ii));
    }
  }

  gtsam::LevenbergMarquardtParams params;
  params.relativeErrorTol = 1e-6;
  params.maxIterations = 100;
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_estimate, params);
  gtsam::Values result = optimizer.optimize();
  std::cout << "pt0_est: " << result.at<gtsam::Point3>(L(0)) << "\n";
  gtsam::Pose3 pose_est = result.at<gtsam::Pose3>(X(0));
  std::cout << "pose_est:\n" << pose_est.matrix() << "\n";
  std::cout << "pose_gt:\n" << pose0_target_cam.matrix() << "\n";
}

// Tests that the gtsam 3 solver is able to find a solution in the case of translation and rotation using
// the first two poses from the synthetic pose set.
TEST_F(PoseSolverFixture, GtsamFirstAndSecondPoses) {
  // Create pose solver problem.
  const gtcal::PoseSolverGtsam::Options options;
  gtcal::PoseSolverGtsam pose_solver(options);
  gtsam::Pose3 pose_target_cam_initial = pose0_target_cam;
  const auto pose_target_cam_est_opt =
      pose_solver.solve(measurements, target_points3d, camera, pose_target_cam_initial);
  EXPECT_TRUE(pose_target_cam_est_opt);
  const gtsam::Pose3& pose_target_cam_est = *pose_target_cam_est_opt;
  EXPECT_TRUE(pose_target_cam_est.equals(pose1_target_cam, 1e-3));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
