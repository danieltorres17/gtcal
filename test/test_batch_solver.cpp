#include "gtcal_test_utils.h"
#include "gtcal/camera.h"
#include "gtcal/utils.h"
#include "gtcal/batch_solver.h"
#include <gtest/gtest.h>

#include <gtsam/geometry/Point2.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/slam/SmartProjectionFactor.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <vector>

using gtsam::symbol_shorthand::K;
using gtsam::symbol_shorthand::L;
using gtsam::symbol_shorthand::X;

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
  target_points.reserve(poses_target_cam.size());
  for (size_t ii = 0; ii < poses_target_cam.size(); ii++) {
    const gtcal::utils::CalibrationTarget target(0.15, 10, 13);
    target_points.emplace_back(target.grid_pts3d_target);
  }

  return target_points;
}

std::vector<gtcal::Measurement> GenerateMeasurements(const size_t camera_index,
                                                     const gtsam::Pose3& pose_target_cam,
                                                     const gtsam::Point3Vector& pts3d_target,
                                                     const std::shared_ptr<gtcal::Camera>& cmod) {
  std::vector<gtcal::Measurement> measurements;
  measurements.reserve(pts3d_target.size());

  for (size_t ii = 0; ii < pts3d_target.size(); ii++) {
    // Project point.
    const gtsam::Point2 uv = cmod->project(pts3d_target.at(ii));

    // Check if inside image.
    if (gtcal::utils::FilterPixelCoords(uv, cmod->width(), cmod->height())) {
      measurements.emplace_back(uv, camera_index, ii);
    }
  }

  return measurements;
}

struct BatchSolverFixture : public testing::Test {
protected:
  // Camera calibrations and models.
  const gtsam::Cal3Fisheye K_fisheye = gtsam::Cal3Fisheye(FX, FY, 0., CX, CY, 0., 0., 0., 0.);
  const gtsam::Cal3_S2 K_linear = gtsam::Cal3_S2(FX, FY, 0., CX, CY);
  std::shared_ptr<gtcal::Camera> fisheye_cam = nullptr;
  std::shared_ptr<gtcal::Camera> linear_cam = nullptr;

  // Target points.
  const gtcal::utils::CalibrationTarget target{0.15, 10, 13};

  // Camera poses.
  const gtsam::Rot3 R0_target_cam = gtsam::Rot3::RzRyRx(0., 0., 0.);
  gtsam::Pose3 pose0_target_cam = gtsam::Pose3();

  void SetUp() override {
    // Create the camera poses.
    const double target_center_x = target.get3dCenter().x();
    const double target_center_y = target.get3dCenter().y();
    pose0_target_cam = gtsam::Pose3(R0_target_cam, {target_center_x, target_center_y, -0.85});

    // Set the fisheye camera.
    fisheye_cam = std::make_shared<gtcal::Camera>();
    fisheye_cam->setCameraModel<gtsam::Cal3Fisheye>(IMAGE_WIDTH, IMAGE_HEIGHT, K_fisheye, pose0_target_cam);

    // Set the linear camera.
    // TODO: Add extrinsics offset.
    linear_cam = std::make_shared<gtcal::Camera>();
    linear_cam->setCameraModel<gtsam::Cal3_S2>(IMAGE_WIDTH, IMAGE_HEIGHT, K_linear, pose0_target_cam);
  }
};

// Tests the batch solver state initialization.
TEST_F(BatchSolverFixture, BatchSolverStateInitialization) {
  // Create batch solver state with both camera models.
  gtcal::BatchSolver::State state({linear_cam, fisheye_cam});

  // Check the number of cameras matches and that the camera pointers are not null.
  EXPECT_EQ(state.cameras.size(), 2);
  for (const auto& cam : state.cameras) {
    EXPECT_TRUE(cam);
  }

  // Make sure the correct models are properly obtained.
  const auto linear_cmod =
      std::get<std::shared_ptr<gtcal::CameraWrapper<gtsam::Cal3_S2>>>(state.cameras.at(0)->cameraVariant());
  EXPECT_TRUE(linear_cmod);
  const auto fisheye_cmod = std::get<std::shared_ptr<gtcal::CameraWrapper<gtsam::Cal3Fisheye>>>(
      state.cameras.at(1)->cameraVariant());
  EXPECT_TRUE(fisheye_cmod);
}

// Tests the initialization of the batch solver object.
TEST_F(BatchSolverFixture, Initialization) {
  // Create batch solver.
  gtcal::BatchSolver batch_solver(target.grid_pts3d_target);

  EXPECT_EQ(batch_solver.targetPoints().size(), target.grid_pts3d_target.size());
}

// Tests that camera calibration prior are successfully added to factor graph and initial values for the
// gtsam::Cal3_S2 model.
TEST_F(BatchSolverFixture, CalibrationPriorCal3_S2) {
  // Create graph and initial values.
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial_estimate;

  // Create batch solver object.
  gtcal::BatchSolver batch_solver(target.grid_pts3d_target);
  batch_solver.addCalibrationPriors(0, linear_cam, graph, initial_estimate);

  // Check the results.
  const gtsam::Cal3_S2 cal3_s2_prior = initial_estimate.at<gtsam::Cal3_S2>(K(0));
  EXPECT_EQ(initial_estimate.size(), 1);
  EXPECT_TRUE(cal3_s2_prior.equals(K_linear));
}

// Tests that camera calibration prior are successfully added to factor graph and initial values for the
// gtsam::Cal3Fisheye model.
TEST_F(BatchSolverFixture, CalibrationPriorCal3_Fisheye) {
  // Create graph and initial values.
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial_estimate;

  // Create batch solver object.
  gtcal::BatchSolver batch_solver(target.grid_pts3d_target);
  batch_solver.addCalibrationPriors(0, fisheye_cam, graph, initial_estimate);

  // Check the results.
  const gtsam::Cal3Fisheye cal3_fisheye_prior = initial_estimate.at<gtsam::Cal3Fisheye>(K(0));
  EXPECT_EQ(initial_estimate.size(), 1);
  EXPECT_TRUE(cal3_fisheye_prior.equals(K_fisheye));
}

// Tests that landmark factors are correctly added to the factor graph for a gtsam::Cal3_S2 model.
TEST_F(BatchSolverFixture, LandmarkFactorsCal3_S2) {
  // Create graph and initial values.
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial_estimate;

  // Create batch solver object.
  gtcal::BatchSolver batch_solver(target.grid_pts3d_target);
  const size_t camera_index = 0;
  batch_solver.addCalibrationPriors(camera_index, linear_cam, graph, initial_estimate);
  EXPECT_EQ(graph.size(), 1);

  // Create batch solver state.
  gtcal::BatchSolver::State state({linear_cam});

  // Get measurements.
  const auto measurements = GenerateMeasurements(camera_index, linear_cam->pose(), target.grid_pts3d_target,
                                                 state.cameras.at(camera_index));

  // Get the number of times the camera has been updated and add landmark factors.
  const size_t num_camera_updates = state.num_camera_updates.at(camera_index);
  if (num_camera_updates == 0) {
    // If it's the first time the camera has been updated, add a pose prior on the graph.
    batch_solver.addPosePrior(camera_index, state.cameras.at(camera_index)->pose(), graph);
  }

  // Check that pose prior was added to the graph. Should add more rigorous check for this.
  EXPECT_EQ(graph.size(), 2);

  // Add landmark priors.
  batch_solver.addLandmarkPriors(measurements, target.grid_pts3d_target, graph);
  const size_t graph_size_post_priors = graph.size();
  const size_t expect_graph_size_post_priors = 2 + measurements.size();
  EXPECT_EQ(graph_size_post_priors, expect_graph_size_post_priors);

  // Add landmark factors.
  batch_solver.addLandmarkFactors(camera_index, state.cameras.at(camera_index), num_camera_updates,
                                  measurements, graph);
  const size_t graph_size_pose_factors = graph.size();
  const size_t expected_graph_size_pose_factors = graph_size_post_priors + measurements.size();
  EXPECT_EQ(graph_size_pose_factors, expected_graph_size_pose_factors);
}

TEST(BatchSolver, DISABLED_GtsamBatchSolver) {
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
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
