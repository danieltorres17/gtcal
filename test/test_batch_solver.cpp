// #include "gtcal/camera_rig.h"
#include "gtcal_test_utils.h"
#include "gtcal/batch_solver.h"
#include <gtest/gtest.h>

#include <gtsam/geometry/Point2.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/slam/SmartProjectionFactor.h>

#include <vector>

gtsam::Pose3Vector GenerateCameraPoses() {
  gtsam::Pose3Vector poses_target_cam;

  // Define rotation matrix and translations.
  const gtsam::Rot3 R0_target_cam = gtsam::Rot3::RzRyRx(0., 0., 0.);
  const gtsam::Point3 xyz0_target_cam = gtsam::Point3(-0.25, 0., -0.1);
  const gtsam::Point3 xyz1_target_cam = gtsam::Point3(0., 0., 0.1);
  const gtsam::Point3 xyz2_target_cam = gtsam::Point3(0.25, 0., 0.05);

  // Build poses.
  const gtsam::Pose3 pose0_target_cam = gtsam::Pose3(R0_target_cam, xyz0_target_cam);
  const gtsam::Pose3 pose1_target_cam = gtsam::Pose3(R0_target_cam, xyz1_target_cam);
  const gtsam::Pose3 pose2_target_cam = gtsam::Pose3(R0_target_cam, xyz2_target_cam);

  poses_target_cam = {pose0_target_cam, pose1_target_cam, pose2_target_cam};

  return poses_target_cam;
}

std::vector<gtsam::Point3Vector> GenerateLandmarks(const gtsam::Pose3Vector& poses_target_cam) {
  std::vector<gtsam::Point3Vector> target_points;
  for (size_t ii = 0; ii < poses_target_cam.size(); ii++) {
    const gtsam::Point3Vector target_points_target = GenerateGridPts3d(0.15, 10, 13);
    target_points.emplace_back(target_points_target);
  }

  return target_points;
}

TEST(BatchSolver, BatchSolver) {
  // Define initial camera parameters.
  gtsam::Cal3Fisheye K = gtsam::Cal3Fisheye(FX + 5, FY - 5, 0., CX - 5, CY + 5, 0.1, 0., 0., 0.);

  // Define set of camera poses.
  const size_t num_poses = 3;
  const gtsam::Pose3Vector poses_target_cam = GenerateCameraPoses();

  // Define vector of vector with target points seen at each pose (for base case, we're assuming all target
  // points can be seen at each camera pose.
  std::vector<gtsam::Point3Vector> points = GenerateLandmarks(poses_target_cam);

  // Create iSAM2 object to solve calibration problem incrementally.
  gtsam::ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  gtsam::ISAM2 isam(parameters);

  // Create factor graph and values to hold the new data.
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial_estimate;

  // Define the camera observation noise model, 1px stddev.
  auto measurement_noise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);

  // Add prior on the calibration.
  auto cal_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(9) << 50., 50., 0.001, 50., 50., 0.01, 0.001, 0.001, 0.001).finished());
  graph.addPrior(gtsam::Symbol('K', 0), K, cal_noise);
  initial_estimate.insert(gtsam::Symbol('K', 0), K);

  // Loop over poses, adding the observations to iSAM incrementally.
  for (size_t ii = 0; ii < poses_target_cam.size(); ii++) {
    // Add factors for each landmark observation.
    for (size_t jj = 0; jj < points.at(ii).size(); jj++) {
      gtsam::PinholeCamera<gtsam::Cal3Fisheye> camera(poses_target_cam.at(ii), K);
      gtsam::Point2 uv = camera.project(points.at(ii).at(jj));
      graph.emplace_shared<gtsam::GeneralSFMFactor2<gtsam::Cal3Fisheye>>(
          uv, measurement_noise, gtsam::Symbol('x', ii), gtsam::Symbol('l', jj), gtsam::Symbol('K', 0));
    }

    // Add an initial guess for the current pose. Intentionally initialize the variables off from the ground
    // truth.
    static gtsam::Pose3 k_delta_pose(gtsam::Rot3::Rodrigues(-0.05, 0.05, 0.1),
                                     gtsam::Point3(0.01, -0.01, 0.015));
    initial_estimate.insert(gtsam::Symbol('x', ii), poses_target_cam.at(ii) * k_delta_pose);

    // If it's the first iteration, add a prior on the first pose to set the coordinate frame and a prior on
    // the first landmark to set the scale. Also as iSAM2 solves incrementally, we must wait until each is
    // observed at least twice before adding it to iSAM.
    if (ii == 0) {
      // Add a prior on pose x0, 30cm std on x, y, z and 0.1 rad on roll, pitch and yaw.
      static auto k_pose_prior = gtsam::noiseModel::Diagonal::Sigmas(
          (gtsam::Vector(6) << gtsam::Vector3::Constant(0.25), gtsam::Vector3::Constant(0.01)).finished());
      graph.addPrior(gtsam::Symbol('x', 0), poses_target_cam.at(0), k_pose_prior);

      // Add a prior on landmark l0.
      static auto k_point_prior = gtsam::noiseModel::Isotropic::Sigma(3, 1e-8);
      graph.addPrior(gtsam::Symbol('l', 0), points.at(0).at(0), k_point_prior);

      // Add an initial guess to all observed landmarks. Intentionally initialize the variables off from the
      // ground truth.
      // static gtsam::Point3 k_delta_point(0.005, 0.002, -0.005);
      static gtsam::Point3 k_delta_point(0., 0., 0.);
      for (size_t jj = 0; jj < points.at(ii).size(); jj++) {
        initial_estimate.insert<gtsam::Point3>(gtsam::Symbol('l', jj), points.at(ii).at(jj) + k_delta_point);
      }
    } else {
      // Update iSAM with the new factors.
      isam.update(graph, initial_estimate);
      // Each call to iSAM2 update(*) performs one iteration of the iterative nonlinear solver. If accuracy
      // is desired at the expense of time, update(*) can be called additional times to perform multiple
      // optimizer iterations every step.
      isam.update();
      gtsam::Values current_estimate = isam.calculateEstimate();
      std::cout << "****************************************************\n";
      std::cout << "Frame " << ii << ": \n";
      current_estimate.print("Current estimate: ");

      // Clear the factor graph and values for the next iteration.
      graph.resize(0);
      initial_estimate.clear();

      // gtsam::Cal3Fisheye cal_values = current_estimate.at<gtsam::Cal3Fisheye>(gtsam::Symbol('K', 0));
      // std::cout << "cal_values: \n";
      // cal_values.print("");
    }
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
