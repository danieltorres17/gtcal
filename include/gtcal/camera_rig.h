#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3Fisheye.h>
#include <gtsam/geometry/Cal3_S2.h>

#include <variant>
#include <concepts>

namespace gtcal {

class CameraRig {
public:
  CameraRig(const size_t num_cameras, const std::vector<gtsam::Cal3Fisheye::shared_ptr>& camera_models);

  size_t GetNumCameras() const { return num_cameras_; }

  const gtsam::Cal3Fisheye::shared_ptr GetCameraParameters(const size_t camera_index) const {
    return camera_models_.at(camera_index);
  }

private:
  const size_t num_cameras_;
  std::vector<gtsam::Cal3Fisheye::shared_ptr> camera_models_;
  std::vector<gtsam::Pose3> camera_extrinsics_;
};

}  // namespace gtcal
