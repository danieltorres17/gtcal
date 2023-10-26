#include "gtcal/pose_solver.h"
#include "gtcal_test_utils.h"
#include "gtcal/camera.h"
#include <gtsam/geometry/PinholeCamera.h>
#include "gtcal/pose_solver_gtsam.h"

#include <gtest/gtest.h>
#include <algorithm>

struct BasePoseSolverFixture {
public:
  // Target grid point parameters.
  const double grid_spacing = 0.3;
  const size_t num_rows = 10;
  const size_t num_cols = 13;
  const size_t num_target_pts = num_rows * num_cols;

  // Create target object and define the target x and y center.
  gtcal::utils::CalibrationTarget target{grid_spacing, num_rows, num_cols};
  double target_center_x = -1.0;
  double target_center_y = -1.0;
  gtsam::Point3Vector target_points3d;

  // Camera and measurements vector.
  std::shared_ptr<gtcal::Camera> camera = nullptr;
  std::vector<gtcal::Measurement> measurements;

  void setUpTarget() {
    // Get the target center.
    target_points3d = target.pointsTarget();
    const gtsam::Point3 target_center_pt3d = target.get3dCenter();
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
  gtsam::Pose3 pose_target_cam_init =
      gtsam::Pose3(gtsam::Rot3::RzRyRx(0.001, -0.0002, 0.01),
                   gtsam::Point3(target_center_x - 0.002, target_center_y + 0.0, -0.81));
  const bool success = pose_solver.solve(measurements, target_points3d, camera, pose_target_cam_init);
  EXPECT_TRUE(success) << "Pose solver failed.";

  // Check the estimated solution.
  EXPECT_TRUE(pose_target_cam_init.equals(pose1_target_cam, 1e-5));
}

// Tests that the gtsam pose solver is able to find a solution in the case of translation only.
TEST_F(TranslationOnlyFixture, GtsamSinglePoseTranslationOnly) {
  // Estimate solution.
  const gtcal::PoseSolverGtsam::Options options;
  gtcal::PoseSolverGtsam pose_solver(options);
  const gtsam::Pose3 pose_target_cam_estimated =
      pose_solver.solve(pose0_target_cam, measurements, target_points3d, K);
  pose_target_cam_estimated.print("pose_target_cam_estimated estimated:\n");

  EXPECT_TRUE(pose_target_cam_estimated.equals(pose1_target_cam, 1e-5));
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

// Tests that the gtsam pose solver is able to find a solution in the case of translation and rotation using
// the first two poses from the synthetic pose set.
TEST_F(PoseSolverFixture, GtsamFirstAndSecondPoses) {
  // Create pose solver problem.
  const gtcal::PoseSolverGtsam::Options options;
  gtcal::PoseSolverGtsam pose_solver(options);
  const gtsam::Pose3 pose_target_cam_estimated =
      pose_solver.solve(pose0_target_cam, measurements, target_points3d, K);
  pose0_target_cam.print("pose0_target_cam:\n");
  pose_target_cam_estimated.print("pose_target_cam_estimated estimated:\n");
  pose1_target_cam.print("pose1_target_cam:\n");

  EXPECT_TRUE(pose_target_cam_estimated.equals(pose1_target_cam, 1e-3));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
