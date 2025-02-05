#include "gtcal_test_utils.h"
#include "gtcal/camera.h"
#include "gtcal/utils.h"
#include "gtcal/batch_solver.h"
#include <gtest/gtest.h>

#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/slam/SmartProjectionFactor.h>
#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/utilities.h>

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

std::vector<gtcal::Measurement> GenerateMeasurements(const size_t camera_index,
                                                     const std::vector<gtsam::Point3>& pts3d_target,
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
  const gtsam::Cal3_S2 K_linear = gtsam::Cal3_S2(FX + 10, FY, 0., CX, CY);
  std::shared_ptr<gtcal::Camera> fisheye_cam = nullptr;
  std::shared_ptr<gtcal::Camera> linear_cam = nullptr;

  // Target points.
  const double grid_spacing = 0.15;
  const size_t num_rows = 10;
  const size_t num_cols = 13;
  const gtcal::CalibrationTarget target{grid_spacing, num_rows, num_cols};
  const std::vector<gtsam::Point3> target_points3d = target.pointsTarget();
  double target_center_x = -1.0;
  double target_center_y = -1.0;

  // Camera poses.
  const gtsam::Rot3 R0_target_cam = gtsam::Rot3::RzRyRx(0., 0., 0.);
  gtsam::Pose3 pose0_target_cam = gtsam::Pose3();

  void SetUp() override {
    // Create the camera poses.
    target_center_x = target.targetCenter().x();
    target_center_y = target.targetCenter().y();
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
  gtcal::BatchSolver batch_solver(target_points3d);

  EXPECT_EQ(batch_solver.targetPoints().size(), target_points3d.size());
}

// Tests the solve method (and the addPriorsAndFactors method implicitly).
TEST_F(BatchSolverFixture, Solve) {
  // Get set of ground truth camera poses in the target frame.
  const gtsam::Pose3Vector poses_target_cam_gt = gtcal::utils::DefaultCameraPoses({pose0_target_cam});

  // Create a batch solver state with the linear camera.
  gtcal::BatchSolver::State state({linear_cam});
  state.num_isam_iterations = 3;
  EXPECT_EQ(state.cameras.size(), 1);
  EXPECT_EQ(state.num_camera_updates.size(), 1);
  EXPECT_EQ(state.num_camera_updates.at(0), 0);
  EXPECT_FALSE(state.first_iteration_complete);

  // Create batch solver.
  gtcal::BatchSolver batch_solver(target_points3d);

  // To keep track of the noisy pose initial pose estimates given.
  std::vector<gtsam::Pose3> poses_target_cam_noisy;
  poses_target_cam_noisy.reserve(poses_target_cam_gt.size());

  // To keep track of the graph and values size at each iteration.
  std::vector<size_t> graph_sizes, values_sizes;
  graph_sizes.reserve(poses_target_cam_gt.size());
  values_sizes.reserve(poses_target_cam_gt.size());

  // To keep track of latest estimates.
  gtsam::Values updated_values;

  for (size_t ii = 0; ii < poses_target_cam_gt.size(); ii++) {
    // Update the camera pose for current iteration to generate measurements.
    state.cameras.at(0)->setCameraPose(poses_target_cam_gt.at(ii));
    const std::vector<gtcal::Measurement> measurements =
        GenerateMeasurements(0, target_points3d, state.cameras.at(0));
    ASSERT_GT(measurements.size(), 0);

    // Generate noisy pose estimate to initialize camera with.
    const gtsam::Pose3 pose_target_cam_noisy =
        gtcal::utils::ApplyNoise(poses_target_cam_gt.at(ii), 0.01, 0.15);
    poses_target_cam_noisy.push_back(pose_target_cam_noisy);
    state.cameras.at(0)->setCameraPose(pose_target_cam_noisy);
    // Add priors and factors to the state graph and values.
    batch_solver.addPriorsAndFactors(measurements, state);
    // For the first iteration, make sure the correct number of factors and initial values were added to both
    // the graph and values. Note the problem isn't solved for the first iteration.
    if (ii == 0) {
      // Check the state member values.
      EXPECT_EQ(state.cameras.size(), 1);
      EXPECT_EQ(state.num_camera_updates.size(), 1);
      EXPECT_EQ(state.num_camera_updates.at(0), 1);
      EXPECT_TRUE(state.first_iteration_complete);

      // Check the current values object. Expecting the pose (1), calibration prior (1) to be added as well as
      // the landmark locations (130 with default target configuration).
      EXPECT_EQ(state.current_estimate.size(), 2 + target_points3d.size());
      EXPECT_TRUE(state.current_estimate.at<gtsam::Cal3_S2>(K(0)).equals(K_linear));
      EXPECT_TRUE(state.current_estimate.at<gtsam::Pose3>(X(0)).equals(poses_target_cam_noisy.at(0)));
      for (size_t ii = 0; ii < target_points3d.size(); ii++) {
        const double err_norm =
            (state.current_estimate.at<gtsam::Point3>(L(ii)) - target_points3d.at(ii)).norm();
        EXPECT_FLOAT_EQ(err_norm, 0.);
      }

      // Check the graph. Expecting it to have 1 calibration factor, 1 pose prior and 2 *
      // target_points3d.size() number of factors to account for the target point priors (since it's the first
      // pass through) and landmarks.
      EXPECT_EQ(state.graph.size(), 2 + 2 * target_points3d.size());

      // Save the graph and value object sizes.
      graph_sizes.push_back(state.graph.size());
      values_sizes.push_back(state.current_estimate.size());
      continue;
    }

    // Solve the problem.
    updated_values = batch_solver.solve(state);
    EXPECT_EQ(state.graph.size(), 0);
    EXPECT_EQ(state.current_estimate.size(), 0);
    EXPECT_EQ(updated_values.size(), 2 + ii + target_points3d.size());
  }

  // Check the updated pose estimates.
  const gtsam::Values poses_target_cam_est = gtsam::utilities::allPose3s(updated_values);
  ASSERT_EQ(poses_target_cam_gt.size(), poses_target_cam_noisy.size());
  ASSERT_EQ(poses_target_cam_gt.size(), poses_target_cam_est.size());
  for (size_t ii = 0; ii < poses_target_cam_gt.size(); ii++) {
    std::cout << "Updated pose check iteration: " << ii << "\n";
    std::cout << "pose_target_cam_noisy:\n";
    std::cout << poses_target_cam_noisy.at(ii).matrix() << "\n";
    std::cout << "pose0_target_cam_est:\n";
    const gtsam::Pose3 pose_target_cam_est_ii = poses_target_cam_est.at<gtsam::Pose3>(X(ii));
    std::cout << pose_target_cam_est_ii.matrix() << "\n";
    std::cout << "pose0_target_cam_gt:\n";
    std::cout << poses_target_cam_gt.at(ii).matrix() << "\n\n";
    EXPECT_TRUE(pose_target_cam_est_ii.equals(poses_target_cam_gt.at(ii), 1e-2));
  }

  // Check the calibration.
  const gtsam::Cal3_S2 calib_est = updated_values.at<gtsam::Cal3_S2>(K(0));
  std::cout << "Ground truth calibration:\n";
  std::cout << K_linear << "\n";
  std::cout << "Estimated calibration:\n";
  std::cout << calib_est << "\n";
  EXPECT_TRUE(K_linear.equals(calib_est, 1));
}

// Tests that the calibration problem can be solved in a batch form using a singular factor graph rather than
// the ISAM2 framework. Used as a reference for the gtcal::BatchSolver implementation and it's also a cool
// exercise. Disabled since the batch solver implementation covers the same thing.
TEST_F(BatchSolverFixture, DISABLED_OutlineFactorGraph) {
  // Get sample poses.
  const gtsam::Pose3Vector poses_target_cam_gt = gtcal::utils::DefaultCameraPoses({pose0_target_cam});
  ASSERT_EQ(poses_target_cam_gt.size(), 5);

  // Create factor graph and values to hold the new data.
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial_estimate;

  // Camera observation noise model, 1px stddev.
  auto measurement_noise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);

  // Calibration noise.
  auto cal_noise =
      gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(5) << 50., 50., 0., 50., 50.).finished());

  // Add calibration prior to factor graph and values.
  graph.addPrior(K(0), K_linear, cal_noise);
  initial_estimate.insert(K(0), K_linear);

  // Add landmark initial estimates and priors.
  for (size_t ii = 0; ii < target_points3d.size(); ii++) {
    initial_estimate.insert<gtsam::Point3>(L(ii), target_points3d.at(ii));
    graph.addPrior(L(ii), target_points3d.at(ii), gtsam::noiseModel::Isotropic::Sigma(3, 1e-8));
  }

  std::vector<gtsam::Pose3> poses_target_cam_noisy;
  poses_target_cam_noisy.reserve(poses_target_cam_gt.size());
  for (size_t ii = 0; ii < poses_target_cam_gt.size(); ii++) {
    // Add initial guess for this pose by applying noise to ground truth pose.
    const gtsam::Pose3 pose_ii_est = gtcal::utils::ApplyNoise(poses_target_cam_gt.at(ii), 0.01, 0.25);
    initial_estimate.insert(X(ii), pose_ii_est);
    std::cout << "Noisy pose given at ii: " << ii << "\n";
    std::cout << pose_ii_est.matrix() << "\n\n";
    poses_target_cam_noisy.emplace_back(pose_ii_est);

    // At each pose, create a camera and add measurements for each target point.
    std::shared_ptr<gtcal::Camera> cmod = std::make_shared<gtcal::Camera>();
    cmod->setCameraModel<gtsam::Cal3_S2>(IMAGE_WIDTH, IMAGE_HEIGHT, K_linear, poses_target_cam_gt.at(ii));
    std::vector<gtcal::Measurement> measurements;
    for (size_t jj = 0; jj < target_points3d.size(); jj++) {
      const gtsam::Point2 uv = cmod->project(target_points3d.at(jj));
      if (gtcal::utils::FilterPixelCoords(uv, cmod->width(), cmod->height())) {
        measurements.emplace_back(uv, ii, jj);
        graph.emplace_shared<gtsam::GeneralSFMFactor2<gtsam::Cal3_S2>>(uv, measurement_noise, X(ii), L(jj),
                                                                       K(0));
      }
    }
  }

  gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initial_estimate).optimize();
  std::vector<gtsam::Pose3> poses_target_cam_est;
  for (size_t ii = 0; ii < poses_target_cam_gt.size(); ii++) {
    poses_target_cam_est.emplace_back(result.at<gtsam::Pose3>(X(ii)));
  }

  // See all estimated poses and compare with the ground truth.
  for (size_t ii = 0; ii < poses_target_cam_gt.size(); ii++) {
    std::cout << "Iteration " << ii << ": \n";
    std::cout << "Noisy pose: \n" << poses_target_cam_noisy.at(ii).matrix() << "\n";
    std::cout << "Pose estimate: \n" << poses_target_cam_est.at(ii).matrix() << "\n";
    std::cout << "Pose ground truth: \n" << poses_target_cam_gt.at(ii).matrix() << "\n\n";
  }
}

// Tests that the calibration problem can be solved in a batch form using the ISAM2 framework. Used as a
// reference for the gtcal::BatchSolver implementation and it's also a cool exercise. Disabled since the
// batch solver implementation covers the same thing.
TEST_F(BatchSolverFixture, DISABLED_OutlineIsam) {
  // Get sample poses.
  const gtsam::Pose3Vector poses_target_cam_gt = gtcal::utils::DefaultCameraPoses({pose0_target_cam});
  ASSERT_EQ(poses_target_cam_gt.size(), 5);

  // Create iSAM2 object to solve calibration problem incrementally.
  gtsam::ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  gtsam::ISAM2 isam(parameters);

  // Create factor graph and values to hold the new data.
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial_estimate;

  // Camera observation noise model, 1px stddev.
  auto measurement_noise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);

  // Calibration noise.
  auto cal_noise =
      gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(5) << 25., 25., 0., 25., 25.).finished());

  // Add calibration prior to factor graph and values.
  graph.addPrior(K(0), K_linear, cal_noise);
  initial_estimate.insert(K(0), K_linear);

  // Add landmark initial estimates and priors.
  for (size_t ii = 0; ii < target_points3d.size(); ii++) {
    initial_estimate.insert<gtsam::Point3>(L(ii), target_points3d.at(ii));
    graph.addPrior(L(ii), target_points3d.at(ii), gtsam::noiseModel::Isotropic::Sigma(3, 1e-8));
  }

  // Go through each pose and add the observations to iSAM incrementally.
  std::vector<gtsam::Pose3> poses_target_cam_noisy;
  for (size_t ii = 0; ii < poses_target_cam_gt.size(); ii++) {
    // At each pose, create a camera and add measurements for each target point.
    std::shared_ptr<gtcal::Camera> cmod = std::make_shared<gtcal::Camera>();
    cmod->setCameraModel<gtsam::Cal3_S2>(IMAGE_WIDTH, IMAGE_HEIGHT, K_linear, poses_target_cam_gt.at(ii));
    std::vector<gtcal::Measurement> measurements;
    for (size_t jj = 0; jj < target_points3d.size(); jj++) {
      const gtsam::Point2 uv = cmod->project(target_points3d.at(jj));
      if (gtcal::utils::FilterPixelCoords(uv, cmod->width(), cmod->height())) {
        measurements.emplace_back(uv, ii, jj);
        graph.emplace_shared<gtsam::GeneralSFMFactor2<gtsam::Cal3_S2>>(uv, measurement_noise, X(ii), L(jj),
                                                                       K(0));
      }
    }

    // Add initial guess for this pose by applying noise to ground truth pose.
    const gtsam::Pose3 pose_ii_est = gtcal::utils::ApplyNoise(poses_target_cam_gt.at(ii), 0.01, 0.1);
    poses_target_cam_noisy.emplace_back(pose_ii_est);
    initial_estimate.insert(X(ii), pose_ii_est);

    // Check if it's the first iteration. Add a prior to the first pose to set the coordinate and a prior on
    // the first landmark to set the scale. Also as iSAM2 solves incrementally, we must wait until each is
    // observed at least twice before adding it to iSAM.
    if (ii == 0) {
      std::cout << "Adding priors.\n";
      // Add a prior on pose x0, 30cm std on x, y, z and 0.1 rad on roll, pitch and yaw.
      static auto k_pose_prior = gtsam::noiseModel::Diagonal::Sigmas(
          (gtsam::Vector(6) << gtsam::Vector3::Constant(0.1), gtsam::Vector3::Constant(0.01)).finished());
      graph.addPrior(X(0), pose_ii_est, k_pose_prior);
    } else {
      // Catch gtsam indeterminate linear system case.
      try {
        gtsam::ISAM2Result res = isam.update(graph, initial_estimate);
        // Each call to iSAM2 update(*) performs one iteration of the iterative nonlinear solver. If
        // accuracy is desired at the expense of time, update(*) can be called additional times to perform
        // multiple optimizer iterations every step.
        isam.update();
        isam.update();
        isam.update();
        // Print the calibration estimate.
        std::cout << "Calibration estimate at ii: " << ii << "\n";
        std::cout << isam.calculateEstimate().at<gtsam::Cal3_S2>(K(0)) << "\n";
        std::cout << "Pose estimates at ii: " << ii << "\n";
        for (size_t jj = 0; jj < ii; jj++) {
          std::cout << "Pose estimate: " << jj << "\n";
          std::cout << isam.calculateEstimate().at<gtsam::Pose3>(X(jj)).matrix() << "\n";
          std::cout << "Pose ground truth: \n";
          std::cout << poses_target_cam_gt.at(jj).matrix() << "\n";
        }
        std::cout << "\n\n";
      } catch (const gtsam::IndeterminantLinearSystemException& e) {
        std::cout << "Indeterminate linear system exception caught at pose: " << ii << "\n";
      }
      // Clear the factor graph and values for the next iteration.
      graph.resize(0);
      initial_estimate.clear();
    }
  }
  gtsam::Values final_res = isam.calculateEstimate();

  // See all estimated poses and compare with the ground truth.
  for (size_t ii = 0; ii < poses_target_cam_gt.size(); ii++) {
    std::cout << "At " << ii << ": \n";
    std::cout << "Noisy pose: \n" << poses_target_cam_noisy.at(ii).matrix() << "\n";
    std::cout << "Pose estimate: \n" << final_res.at<gtsam::Pose3>(X(ii)).matrix() << "\n";
    std::cout << "Pose ground truth: \n" << poses_target_cam_gt.at(ii).matrix() << "\n";
  }
  // See final calibration estimate.
  std::cout << "Final calibration estimate: \n";
  std::cout << final_res.at<gtsam::Cal3_S2>(K(0)) << "\n";
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
