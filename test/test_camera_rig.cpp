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
  std::vector<gtsam::Cal3Fisheye> camera_params;
  auto K = gtsam::Cal3Fisheye(FX, FY, 0., CX, CY, 0., 0., 0., 0.);
  camera_params.push_back(K);

  gtcal::CameraRig<gtsam::Cal3Fisheye> camera_rig(1, camera_params);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
