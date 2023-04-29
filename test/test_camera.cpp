#include "gtcal/utils.h"
#include "gtcal_test_utils.h"
#include <gtest/gtest.h>

#include "gtcal/camera.h"

#include <vector>
#include <algorithm>

struct CameraFixture : public testing::Test {
protected:
  // Target grid point parameters.
  const double grid_spacing = 0.3;
  const size_t num_rows = 10;
  const size_t num_cols = 13;
  const size_t num_target_pts = num_rows * num_cols;
  const gtcal::utils::CalibrationTarget target = {grid_spacing, num_rows, num_cols};
  const gtsam::Point3 target_center_pt3d = target.get3dCenter();
  const double target_center_x = target_center_pt3d.x();
  const double target_center_y = target_center_pt3d.y();

  // First camera pose in target frame.
  const gtsam::Rot3 R0_target_cam = gtsam::Rot3::RzRyRx(0., 0., 0.);
  const gtsam::Point3 xyz0_target_cam = gtsam::Point3(target_center_x, target_center_y, -0.85);
  const gtsam::Pose3 pose0_target_cam = gtsam::Pose3(R0_target_cam, xyz0_target_cam);

  // Camera models.
  std::shared_ptr<gtsam::Cal3Fisheye> K_fisheye =
      std::make_shared<gtsam::Cal3Fisheye>(FX, FY, 0., CX, CY, 0., 0., 0., 0.);
  std::shared_ptr<gtsam::Cal3_S2> K_cal3_s2 = std::make_shared<gtsam::Cal3_S2>(FX, FY, 0., CX, CY);
};

TEST_F(CameraFixture, FisheyeCameraConstruction) {
  // Create fisheye camera and check results.
  gtcal::CameraWrapper<gtsam::Cal3Fisheye> fisheye_camera(IMAGE_WIDTH, IMAGE_HEIGHT, *K_fisheye);
  EXPECT_TRUE(K_fisheye->equals(fisheye_camera.calibration()));
  EXPECT_TRUE(fisheye_camera.pose().equals(gtsam::Pose3()));
  EXPECT_EQ(IMAGE_WIDTH, fisheye_camera.width());
  EXPECT_EQ(IMAGE_HEIGHT, fisheye_camera.height());
}

TEST_F(CameraFixture, Cal3S2CameraConstruction) {
  // Create S3_2 camera and check results.
  gtcal::CameraWrapper<gtsam::Cal3_S2> cal3_s2_camera(IMAGE_WIDTH, IMAGE_HEIGHT, *K_cal3_s2);
  EXPECT_TRUE(K_cal3_s2->equals(cal3_s2_camera.calibration()));
  EXPECT_TRUE(cal3_s2_camera.pose().equals(gtsam::Pose3()));
  EXPECT_EQ(IMAGE_WIDTH, cal3_s2_camera.width());
  EXPECT_EQ(IMAGE_HEIGHT, cal3_s2_camera.height());
}

TEST_F(CameraFixture, FisheyeCameraProjection) {
  // Create gtcal fisheye camera and project first target point.
  gtcal::CameraWrapper<gtsam::Cal3Fisheye> fisheye_camera(IMAGE_WIDTH, IMAGE_HEIGHT, *K_fisheye,
                                                          pose0_target_cam);
  const gtsam::Point2 uv = fisheye_camera.project(target.grid_pts3d_target.at(0));

  // Create gtsam camera, project point and compare result.
  gtsam::PinholeCamera<gtsam::Cal3Fisheye> gtsam_fisheye_camera(pose0_target_cam, *K_fisheye);
  const gtsam::Point2 uv_expected = gtsam_fisheye_camera.project(target.grid_pts3d_target.at(0));
  EXPECT_FLOAT_EQ((uv.norm() - uv_expected.norm()), 0.0);
}

TEST_F(CameraFixture, Cal3S2CameraProjection) {
  // Create gtcal cal3_s2 camera and project first target point.
  gtcal::CameraWrapper<gtsam::Cal3_S2> cal3_s2(IMAGE_WIDTH, IMAGE_HEIGHT, *K_cal3_s2, pose0_target_cam);
  const gtsam::Point2 uv = cal3_s2.project(target.grid_pts3d_target.at(0));

  // Create gtsam camera, project point and compare result.
  gtsam::PinholeCamera<gtsam::Cal3_S2> gtsam_cal3_s2(pose0_target_cam, *K_cal3_s2);
  const gtsam::Point2 uv_expected = gtsam_cal3_s2.project(target.grid_pts3d_target.at(0));
  EXPECT_FLOAT_EQ((uv.norm() - uv_expected.norm()), 0.0);
}

TEST_F(CameraFixture, CameraVariant) {
  // Create fisheye camera.
  gtcal::Camera fisheye_cam;
  fisheye_cam.setCameraModel<gtsam::Cal3Fisheye>(IMAGE_WIDTH, IMAGE_HEIGHT, *K_fisheye, pose0_target_cam);

  // Create Cal3_S2 camera.
  gtcal::Camera cal3_s2_cam;
  cal3_s2_cam.setCameraModel<gtsam::Cal3_S2>(IMAGE_WIDTH, IMAGE_HEIGHT, *K_cal3_s2, pose0_target_cam);

  // Create vector of cameras.
  std::vector<gtcal::Camera> cameras = {fisheye_cam, cal3_s2_cam};

  // Project target point with both cameras.
  const gtsam::Point2 uv_fisheye = cameras.at(0).project(target.grid_pts3d_target.at(0));
  const gtsam::Point2 uv_cal3_s2 = cameras.at(1).project(target.grid_pts3d_target.at(0));

  // Create gtsam Cal3_S2 camera, project point and compare result.
  gtsam::PinholeCamera<gtsam::Cal3_S2> gtsam_cal3_s2(pose0_target_cam, *K_cal3_s2);
  const gtsam::Point2 uv_cal3_s2_expected = gtsam_cal3_s2.project(target.grid_pts3d_target.at(0));
  EXPECT_FLOAT_EQ((uv_cal3_s2.norm() - uv_cal3_s2_expected.norm()), 0.0);

  // Create gtsam fisheye camera, project point and compare result.
  gtsam::PinholeCamera<gtsam::Cal3Fisheye> gtsam_fisheye_camera(pose0_target_cam, *K_fisheye);
  const gtsam::Point2 uv_fisheye_expected = gtsam_fisheye_camera.project(target.grid_pts3d_target.at(0));
  EXPECT_FLOAT_EQ((uv_fisheye.norm() - uv_fisheye_expected.norm()), 0.0);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
