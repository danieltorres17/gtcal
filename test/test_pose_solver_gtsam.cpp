#include "gtcal/pose_solver.h"
#include "gtcal_test_utils.h"
#include <gtest/gtest.h>

#include "gtsam/inference/Symbol.h"
#include "gtsam/slam/ProjectionFactor.h"
#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include "gtsam/nonlinear/Values.h"
#include "gtsam/nonlinear/utilities.h"
#include "gtsam/slam/StereoFactor.h"

#include <vector>
#include <algorithm>

struct PoseSolverFixture : public testing::Test {
protected:
  // Target grid point parameters.
  const double grid_spacing = 0.3;
  const size_t num_rows = 10;
  const size_t num_cols = 13;
  const size_t num_target_pts = num_rows * num_cols;

  void SetUp() override {}
};

TEST_F(PoseSolverFixture, SinglePoseTranslationOnly) {
  // Create target object.
  const gtcal::utils::CalibrationTarget target(grid_spacing, num_rows, num_cols);
  const gtsam::Point3 target_center_pt3d = target.get3dCenter();
  const double target_center_x = target_center_pt3d.x();
  const double target_center_y = target_center_pt3d.y();

  // First camera pose in target frame.
  const gtsam::Rot3 R0_target_cam = gtsam::Rot3::RzRyRx(0., 0., 0.);
  const gtsam::Point3 xyz0_target_cam = gtsam::Point3(target_center_x, target_center_y, -0.85);
  const gtsam::Pose3 pose0_target_cam = gtsam::Pose3(R0_target_cam, xyz0_target_cam);

  // Second camera pose in target frame.
  const gtsam::Rot3 R1_target_cam = gtsam::Rot3::RzRyRx(0., 0., 0.);
  const gtsam::Point3 xyz1_target_cam = gtsam::Point3(target_center_x, target_center_y, -0.825);
  const gtsam::Pose3 pose1_target_cam = gtsam::Pose3(R1_target_cam, xyz1_target_cam);

  // Transform grid points at second pose into camera frame.
  gtsam::Point3Vector target_pts3d_cam_pose0;
  target_pts3d_cam_pose0.reserve(target.grid_pts3d_target.size());
  const gtsam::Pose3 pose0_cam_target = pose0_target_cam.inverse();  // Target in camera frame at pose 1.
  for (const auto& pt3d_target : target.grid_pts3d_target) {
    const gtsam::Point3 pt3d_cam = pose0_cam_target.transformFrom(pt3d_target);
    EXPECT_FLOAT_EQ(-xyz0_target_cam.z(), pt3d_cam.z());
    target_pts3d_cam_pose0.emplace_back(pt3d_cam);
  }

  // Create camera model.
  auto K = boost::make_shared<gtsam::Cal3Fisheye>(FX, FY, 0., CX, CY, 0., 0., 0., 0.);
  gtsam::PinholeCamera<gtsam::Cal3Fisheye> camera(pose1_target_cam, *K);

  // Get target point measurements at second pose.
  gtsam::Point2Vector uvs_pose1;
  uvs_pose1.reserve(target.grid_pts3d_target.size());
  for (const auto& pt3d_target : target.grid_pts3d_target) {
    const gtsam::Point2 uv = camera.project(pt3d_target);
    if (gtcal::utils::FilterPixelCoords(uv, IMAGE_WIDTH, IMAGE_HEIGHT)) {
      uvs_pose1.emplace_back(uv);
    }
  }
  // Check that the number of measurements is equal to the number of 3D points.
  EXPECT_EQ(uvs_pose1.size(), target.grid_pts3d_target.size());

  // Create factor graph to solve pose estimation problem. Based on the gtsam's self calibration example.
  gtsam::NonlinearFactorGraph graph;
  // Add prior on the pose - 0.1 rad for roll, pitch, yaw and 15cm for x, y, z.
  auto poseNoise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3::Constant(0.1), gtsam::Vector3::Constant(0.15)).finished());
  graph.addPrior(gtsam::Symbol('x', 0), pose0_target_cam, poseNoise);

  for (size_t ii = 0; ii < uvs_pose1.size(); ii++) {
    graph.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3Fisheye>>(
        uvs_pose1[ii], gtsam::noiseModel::Isotropic::Sigma(2, 1.0), gtsam::Symbol('x', 0),
        gtsam::Symbol('l', ii), K);
  }

  // Add a prior to the landmarks (since these are rigidly placed).
  auto pointNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e-7);
  for (size_t ii = 0; ii < target.grid_pts3d_target.size(); ii++) {
    graph.addPrior(gtsam::Symbol('l', ii), target.grid_pts3d_target.at(ii), pointNoise);
  }

  // Create initial estimate.
  gtsam::Values initialEstimate;
  for (size_t ii = 0; ii < target.grid_pts3d_target.size(); ii++) {
    initialEstimate.insert<gtsam::Point3>(gtsam::Symbol('l', ii), target.grid_pts3d_target.at(ii));
  }
  pose0_target_cam.print("Pose0 target cam:\n");
  initialEstimate.insert<gtsam::Pose3>(gtsam::Symbol('x', 0), pose0_target_cam);

  // Optimize.
  gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initialEstimate).optimize();

  // Check that the pose is close to the ground truth.
  gtsam::Values pose3_result = gtsam::utilities::allPose3s(result);
  pose3_result.print("Pose3 estimated:\n");
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
