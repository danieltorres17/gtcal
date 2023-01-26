#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3Fisheye.h>
#include <gtsam/geometry/PinholeCamera.h>

namespace gtcal {
class CameraRig {
public:
  CameraRig(const size_t num_cameras, const std::vector<gtsam::Cal3Fisheye>& camera_params)
    : num_cameras_(num_cameras), camera_params_(camera_params) {}

  size_t GetNumCameras() const { return num_cameras_; }
  const gtsam::Cal3Fisheye& GetCameraParams(const size_t camera_index) const;
  void UpdateCameraParams(const size_t camera_index);

private:
  const size_t num_cameras_;
  std::vector<gtsam::Cal3Fisheye> camera_params_;
  std::vector<gtsam::Pose3> camera_extrinsics_;
};
}  // namespace gtcal
