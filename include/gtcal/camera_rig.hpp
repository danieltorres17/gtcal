#pragma once

#include "gtcal/camera.hpp"

namespace gtcal {

struct CameraRig {
  using Ptr = std::shared_ptr<CameraRig>;

  CameraRig(const std::vector<std::shared_ptr<Camera>>& cameras_vec,
            const std::vector<gtsam::Pose3>& camera_extrinsics_vec)
    : cameras(cameras_vec), camera_extrinsics(camera_extrinsics_vec) {}

  std::vector<std::shared_ptr<Camera>> cameras;
  std::vector<gtsam::Pose3> camera_extrinsics;
};

}  // namespace gtcal