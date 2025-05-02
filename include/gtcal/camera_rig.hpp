#pragma once

#include "gtcal/camera.hpp"
#include "gtcal/utils.hpp"

namespace gtcal {

struct CameraRig {
  using Ptr = std::shared_ptr<CameraRig>;

  CameraRig(const std::vector<std::shared_ptr<Camera>>& cameras_vec,
            const std::vector<gtsam::Pose3>& camera_extrinsics_vec)
    : cameras(cameras_vec), camera_extrinsics(camera_extrinsics_vec) {}

  size_t numCameras() const { return cameras.size(); }

  static Ptr CreateNoisyCameraRig(const Ptr& original) {
    // Create a noisy copy of the original camera calibrations.
    std::vector<std::shared_ptr<Camera>> noisy_cameras;
    noisy_cameras.reserve(original->cameras.size());
    for (const auto& camera : original->cameras) {
      noisy_cameras.push_back(std::make_shared<Camera>(*camera));
    }

    // Create noisy copy of the original camera extrinsics.
    // The first camera extrinsic remains as the identity.
    std::vector<gtsam::Pose3> noisy_extrinsics;
    noisy_extrinsics.reserve(original->camera_extrinsics.size());
    noisy_extrinsics.push_back(gtsam::Pose3());
    for (size_t ii = 1; ii < original->camera_extrinsics.size(); ii++) {
      const auto& extrinsic = original->camera_extrinsics.at(ii);
      const gtsam::Pose3 noisy_extrinsic = utils::ApplyNoiseToPose(extrinsic, 0.01, 0.1);
      noisy_extrinsics.push_back(noisy_extrinsic);
    }

    return std::make_shared<CameraRig>(noisy_cameras, noisy_extrinsics);
  }

  const std::vector<std::shared_ptr<Camera>> cameras;
  const std::vector<gtsam::Pose3> camera_extrinsics;
};

}  // namespace gtcal