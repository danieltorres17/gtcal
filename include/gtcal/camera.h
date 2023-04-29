#pragma once

#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Cal3Fisheye.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>

#include <variant>

namespace gtcal {

template <typename T>
class CameraWrapper {
public:
  CameraWrapper(const size_t width, const size_t height, const T& calibration,
                const gtsam::Pose3& pose_world_camera = gtsam::Pose3())
    : width_(width)
    , height_(height)
    , camera_(std::make_shared<gtsam::PinholeCamera<T>>(pose_world_camera, calibration)) {}

  /**
   * @brief Return projection of 3D point.
   *
   * @param pt3d_world 3D point to project in world frame.
   * @return gtsam::Point2
   */
  gtsam::Point2 project(const gtsam::Point3& pt3d_world) const { return camera_->project(pt3d_world); }

  /**
   * @brief Update the camera's calibration.
   *
   * @param calibration gtsam calibration type.
   */
  void updateCalibration(const T& calibration) {
    // camera_->
  }

  /**
   * @brief Return true if the camera parameters are equal to given camera's parameters. Return false
   * otherwise.
   *
   * @param other gtcal::camera to compare to.
   * @param tol tolerance for comparison.
   * @return true
   * @return false
   */
  bool equals(const CameraWrapper<T>& other, double tol = 1e-9) const {
    return camera_->equals(*other.camera_, tol) && width_ == other.width_ && height_ == other.height_ &&
           camera_->pose().equals(other.camera_->pose(), tol);
  }

  /**
   * @brief Return gtsam::Pose3 denoting camera pose in world frame.
   *
   * @return gtsam::Pose3
   */
  gtsam::Pose3 pose() const { return camera_->pose(); }

  /**
   * @brief Return gtsam calibration object.
   *
   * @return T
   */
  T calibration() const { return camera_->calibration(); }

  /**
   * @brief Return camera image width.
   *
   * @return size_t
   */
  size_t width() const { return width_; }

  /**
   * @brief Return camera image height.
   *
   * @return size_t
   */
  size_t height() const { return height_; }

private:
  size_t width_ = 0;
  size_t height_ = 0;
  std::shared_ptr<gtsam::PinholeCamera<T>> camera_ = nullptr;
};

class Camera {
public:
  /**
   * @brief Set the Camera Model object.
   *
   * @tparam T gtsam calibration type.
   * @param width image width.
   * @param height image height.
   * @param calibration gtsam calibration object.
   * @param pose_world_camera gtsam::Pose3 denoting the camera pose in the world frame.
   */
  template <typename T>
  void setCameraModel(const size_t width, const size_t height, const T& calibration,
                      const gtsam::Pose3& pose_world_camera = gtsam::Pose3()) {
    camera_ = std::make_shared<CameraWrapper<T>>(width, height, calibration, pose_world_camera);
  }

  /**
   * @brief Return projection of 3D point given in world frame.
   *
   * @param pt3d_world 3D point in world frame to project.
   * @return gtsam::Point2
   */
  gtsam::Point2 project(const gtsam::Point3& pt3d_world) const {
    return std::visit(
        [&](auto&& arg) -> gtsam::Point2 {
          using T = std::decay_t<decltype(arg)>;
          if constexpr (std::is_same_v<T, std::shared_ptr<CameraWrapper<gtsam::Cal3_S2>>>) {
            return arg->project(pt3d_world);
          } else if constexpr (std::is_same_v<T, std::shared_ptr<CameraWrapper<gtsam::Cal3Fisheye>>>) {
            return arg->project(pt3d_world);
          } else {
            assert(false && "Invalid camera model.");
          }
        },
        camera_);
  }

private:
  std::variant<std::shared_ptr<CameraWrapper<gtsam::Cal3_S2>>,
               std::shared_ptr<CameraWrapper<gtsam::Cal3Fisheye>>>
      camera_;
};

}  // namespace gtcal
