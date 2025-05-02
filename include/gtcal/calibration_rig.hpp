#pragma once

#include "gtcal/camera.hpp"
#include "gtcal/utils.hpp"

#include <gtsam/inference/Symbol.h>
#include <unordered_map>

namespace gtcal {

struct CalibrationRig {
  using Ptr = std::shared_ptr<CalibrationRig>;

  CalibrationRig(const std::vector<std::shared_ptr<Camera>>& cameras_vec) : cameras(cameras_vec) {
    // Assign unique keys to camera poses.
    char current_key = 'a';
    for (size_t ii = 0; ii < cameras.size(); ii++) {
      if (current_key == 'l' || current_key == 'k') {
        current_key++;
      }
      camera_pose_key_map_[ii] = current_key++;
      camera_pose_key_map_reverse_[camera_pose_key_map_[ii]] = ii;
    }
  }

  gtsam::Symbol cameraPoseSymbol(const size_t camera_id, const size_t pose_index) const {
    if (camera_id >= camera_pose_key_map_.size()) {
      throw std::out_of_range("Camera ID out of range");
    }

    return gtsam::Symbol(camera_pose_key_map_.at(camera_id), pose_index);
  }

  gtsam::Symbol calibrationSymbol(const size_t camera_id) const {
    if (camera_id >= camera_pose_key_map_.size()) {
      throw std::out_of_range("Camera ID out of range");
    }

    return gtsam::Symbol('k', camera_id);
  }

  size_t cameraPoseSymbolToIndex(const gtsam::Symbol& symbol) const {
    if (camera_pose_key_map_reverse_.count(symbol.chr() == 0)) {
      throw std::out_of_range("Camera pose symbol not found");
    }

    return camera_pose_key_map_reverse_.at(symbol.chr());
  }

  size_t cameraCalibrationSymbolToIndex(const gtsam::Symbol& symbol) const {
    if (symbol.chr() != 'k') {
      throw std::invalid_argument("Symbol key is not a calibration key.");
    }

    return static_cast<size_t>(symbol.index());
  }

  size_t numCameras() const { return cameras.size(); }

  std::vector<std::shared_ptr<Camera>> cameras;
  std::unordered_map<size_t, std::vector<std::optional<gtsam::Pose3>>> poses_target_camera_map;

private:
  std::unordered_map<size_t, char> camera_pose_key_map_;
  std::unordered_map<char, size_t> camera_pose_key_map_reverse_;
};

}  // namespace gtcal