#pragma once

#include "gtcal/camera.hpp"

#include <gtsam/inference/Symbol.h>
#include <unordered_map>

namespace gtcal {

struct CameraRig {
  using Ptr = std::shared_ptr<CameraRig>;

  CameraRig(const std::vector<std::shared_ptr<Camera>>& cameras_vec,
            const std::vector<gtsam::Pose3>& camera_extrinsics_vec)
    : cameras(cameras_vec), camera_extrinsics(camera_extrinsics_vec) {
    // Assign unique keys to camera poses.
    char current_key = 'a';
    for (size_t ii = 0; ii < cameras.size(); ii++) {
      if (current_key == 'l' || current_key == 'k') {
        current_key++;
      }
      camera_pose_key_map[ii] = current_key++;
      camera_pose_key_map_reverse[camera_pose_key_map[ii]] = ii;
    }
  }

  gtsam::Symbol cameraPoseKey(const size_t camera_id, const size_t pose_index) const {
    if (camera_id >= camera_pose_key_map.size()) {
      throw std::out_of_range("Camera ID out of range");
    }

    return gtsam::Symbol(camera_pose_key_map.at(camera_id), pose_index);
  }

  gtsam::Symbol calibrationKey(const size_t camera_id) const {
    if (camera_id >= camera_pose_key_map.size()) {
      throw std::out_of_range("Camera ID out of range");
    }

    return gtsam::Symbol('k', camera_id);
  }

  size_t cameraPoseKeyToIndex(const gtsam::Symbol& key) const {
    if (camera_pose_key_map_reverse.count(key.chr() == 0)) {
      throw std::out_of_range("Camera pose key not found");
    }

    return camera_pose_key_map_reverse.at(key.chr());
  }

  size_t numCameras() const { return cameras.size(); }

  std::vector<std::shared_ptr<Camera>> cameras;
  std::vector<gtsam::Pose3> camera_extrinsics;

private:
  std::unordered_map<size_t, char> camera_pose_key_map;
  std::unordered_map<char, size_t> camera_pose_key_map_reverse;
};

}  // namespace gtcal