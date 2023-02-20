#include "gtcal/camera_rig.h"
#include <gtsam/geometry/Cal3_S2.h>
#include <gtest/gtest.h>

#define IMAGE_WIDTH 1024
#define IMAGE_HEIGHT 570
#define FX 200
#define FY 200
#define CX 512
#define CY 235

TEST(CameraRig, CameraRig) {
  std::vector<gtsam::Cal3Fisheye::shared_ptr> camera_params;
  auto K = boost::make_shared<gtsam::Cal3Fisheye>(FX, FY, 0., CX, CY, 0., 0., 0., 0.);
  auto K2 = boost::make_shared<gtsam::Cal3Fisheye>(FX, FY, 0.0, CX, CY, 2.0, 1.2, 2.3, 1.2);

  camera_params.push_back(K);
  camera_params.push_back(K2);

  gtcal::CameraRig rig(2, camera_params);
  const auto cam = rig.GetCameraParameters(0);
  EXPECT_FLOAT_EQ(cam->k1(), 0.0);
  EXPECT_FLOAT_EQ(rig.GetCameraParameters(1)->k1(), 2.0);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
