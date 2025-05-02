#include <gtest/gtest.h>

#include "gtcal/calibration_rig.hpp"

struct CalibrationRigFixture : public testing::Test {
  // Camera parameters.
  const double FX = 1000.;
  const double FY = 1000.;
  const double CX = 640.;
  const double CY = 480.;
  const int IMAGE_WIDTH = 1280;
  const int IMAGE_HEIGHT = 960;

  // Camera models.
  gtcal::Camera::Ptr linear_camera = nullptr;
  gtsam::Cal3_S2::shared_ptr K_cal3_s2 = nullptr;
  gtcal::Camera::Ptr fisheye_camera = nullptr;
  gtsam::Cal3Fisheye::shared_ptr K_cal3_fisheye = nullptr;

  std::vector<gtcal::Camera::Ptr> cameras_vec;

  void SetUp() override {
    // Initialize linear camera.
    K_cal3_s2 = std::make_shared<gtsam::Cal3_S2>(FX, FY, 0., CX, CY);
    linear_camera = std::make_shared<gtcal::Camera>();
    linear_camera->setCameraModel<gtsam::Cal3_S2>(IMAGE_WIDTH, IMAGE_HEIGHT, *K_cal3_s2);

    // Initialize fisheye camera.
    K_cal3_fisheye = std::make_shared<gtsam::Cal3Fisheye>(FX, FY, 0., CX, CY, 0.1, 0.2, 0.3, 0.4);
    fisheye_camera = std::make_shared<gtcal::Camera>();
    fisheye_camera->setCameraModel<gtsam::Cal3Fisheye>(IMAGE_WIDTH, IMAGE_HEIGHT, *K_cal3_fisheye);

    // Create calibration rig with both cameras.
    cameras_vec = {linear_camera, fisheye_camera};
  }
};

TEST_F(CalibrationRigFixture, Constructor) {
  gtcal::CalibrationRig::Ptr calibration_rig = std::make_shared<gtcal::CalibrationRig>(cameras_vec);
  EXPECT_EQ(calibration_rig->numCameras(), cameras_vec.size());
  EXPECT_TRUE(calibration_rig->poses_target_camera_map.empty());
}

TEST_F(CalibrationRigFixture, CameraPoseSymbol) {
  gtcal::CalibrationRig::Ptr calibration_rig = std::make_shared<gtcal::CalibrationRig>(cameras_vec);

  const gtsam::Symbol camera0_pose0_key = calibration_rig->cameraPoseSymbol(0, 0);
  EXPECT_EQ(camera0_pose0_key.chr(), 'a');
  EXPECT_EQ(camera0_pose0_key.index(), 0);
  const gtsam::Symbol camera1_pose0_key = calibration_rig->cameraPoseSymbol(1, 0);
  EXPECT_EQ(camera1_pose0_key.chr(), 'b');
  EXPECT_EQ(camera1_pose0_key.index(), 0);
}

TEST_F(CalibrationRigFixture, CalibrationSymbol) {
  gtcal::CalibrationRig::Ptr calibration_rig = std::make_shared<gtcal::CalibrationRig>(cameras_vec);

  const gtsam::Symbol camera0_calibration_key = calibration_rig->calibrationSymbol(0);
  EXPECT_EQ(camera0_calibration_key.chr(), 'k');
  EXPECT_EQ(camera0_calibration_key.index(), 0);
  const gtsam::Symbol camera1_calibration_key = calibration_rig->calibrationSymbol(1);
  EXPECT_EQ(camera1_calibration_key.chr(), 'k');
  EXPECT_EQ(camera1_calibration_key.index(), 1);
}

TEST_F(CalibrationRigFixture, CameraPoseSymbolToIndex) {
  gtcal::CalibrationRig::Ptr calibration_rig = std::make_shared<gtcal::CalibrationRig>(cameras_vec);

  const size_t camera0_index = calibration_rig->cameraPoseSymbolToIndex(gtsam::Symbol('a', 0));
  EXPECT_EQ(camera0_index, 0);
  const size_t camera1_index = calibration_rig->cameraPoseSymbolToIndex(gtsam::Symbol('b', 0));
  EXPECT_EQ(camera1_index, 1);
}

TEST_F(CalibrationRigFixture, CameraCalibrationSymbolToIndex) {
  gtcal::CalibrationRig::Ptr calibration_rig = std::make_shared<gtcal::CalibrationRig>(cameras_vec);

  const size_t camera0_index = calibration_rig->cameraCalibrationSymbolToIndex(gtsam::Symbol('k', 0));
  EXPECT_EQ(camera0_index, 0);
  const size_t camera1_index = calibration_rig->cameraCalibrationSymbolToIndex(gtsam::Symbol('k', 1));
  EXPECT_EQ(camera1_index, 1);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
