#include "gtcal/camera_rig.h"
#include "gtcal_test_utils.h"

#include <gtsam/geometry/Cal3_S2.h>
#include <gtest/gtest.h>

#include <memory>

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

TEST(Camera, CameraVariant) {
  auto K = gtsam::Cal3Fisheye(FX, FY, 0., CX, CY, 0., 0.1, 0., 0.);
  auto K2 = gtsam::Cal3_S2(FX, FY, 0.25, CX, CY);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
