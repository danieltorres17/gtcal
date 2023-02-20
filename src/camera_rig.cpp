#include "gtcal/camera_rig.h"

namespace gtcal {

CameraRig::CameraRig(const size_t num_cameras, const std::vector<gtsam::Cal3Fisheye::shared_ptr>& camera_models)
  : num_cameras_(num_cameras), camera_models_(camera_models) {}



}  // namespace gtcal