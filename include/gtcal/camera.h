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
    : width_(width), height_(height), calibration_(calibration), pose_world_camera_(pose_world_camera) {}

  /**
   * @brief Return projection of 3D point.
   *
   * @param pt3d_world 3D point to project in world frame.
   * @return gtsam::Point2
   */
  gtsam::Point2 project(const gtsam::Point3& pt3d_world) const {
    gtsam::PinholeCamera<T> camera(pose_world_camera_, calibration_);
    return camera.project(pt3d_world);
  }

  /**
   * @brief Update the camera's calibration.
   *
   * @param calibration gtsam calibration type.
   */
  void updateCalibration(const T& calibration) { calibration_ = calibration; }

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
    return calibration_.equals(*other.calibration(), tol) && pose_world_camera_.equals(other.pose(), tol) &&
           width_ == other.width_ && height_ == other.height_;
  }

  /**
   * @brief Return gtsam::Pose3 denoting camera pose in world frame.
   *
   * @return gtsam::Pose3
   */
  gtsam::Pose3 pose() const { return pose_world_camera_; }

  /**
   * @brief Update the camera's pose in the world frame.
   *
   * @param pose_world_camera
   */
  void updatePose(const gtsam::Pose3& pose_world_camera) { pose_world_camera_ = pose_world_camera; }

  /**
   * @brief Return gtsam calibration object.
   *
   * @return T
   */
  T calibration() const { return calibration_; }

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
  T calibration_;
  gtsam::Pose3 pose_world_camera_;
};

class Camera {
public:
  using CameraVariant = std::variant<std::shared_ptr<CameraWrapper<gtsam::Cal3_S2>>,
                                     std::shared_ptr<CameraWrapper<gtsam::Cal3Fisheye>>>;

  enum class ModelType { CAL3_S2, CAL3_FISHEYE };

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
    setModelType();
  }

  /**
   * @brief Set the model type based on the variant type. Terrible workaround for not being able to get type
   * directly from the variant.
   *
   */
  void setModelType() {
    std::visit(
        [&](auto&& arg) -> void {
          using T = std::decay_t<decltype(arg)>;
          if constexpr (std::is_same_v<T, std::shared_ptr<CameraWrapper<gtsam::Cal3_S2>>>) {
            model_ = ModelType::CAL3_S2;
          } else if constexpr (std::is_same_v<T, std::shared_ptr<CameraWrapper<gtsam::Cal3Fisheye>>>) {
            model_ = ModelType::CAL3_FISHEYE;
          } else {
            assert(false && "Invalid camera model.");
          }
        },
        camera_);
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

  void setCameraPose(const gtsam::Pose3& pose_world_camera) {
    std::visit(
        [&](auto&& arg) -> void {
          using T = std::decay_t<decltype(arg)>;
          if constexpr (std::is_same_v<T, std::shared_ptr<CameraWrapper<gtsam::Cal3_S2>>>) {
            arg->updatePose(pose_world_camera);
          } else if constexpr (std::is_same_v<T, std::shared_ptr<CameraWrapper<gtsam::Cal3Fisheye>>>) {
            arg->updatePose(pose_world_camera);
          } else {
            assert(false && "Invalid camera model.");
          }
        },
        camera_);
  }

  size_t height() const {
    return std::visit([](auto&& arg) -> size_t { return arg->height(); }, camera_);
  }

  size_t width() const {
    return std::visit([](auto&& arg) -> size_t { return arg->width(); }, camera_);
  }

  const CameraVariant& cameraVariant() const { return camera_; }

  ModelType modelType() const { return model_; }

private:
  CameraVariant camera_;
  ModelType model_ = ModelType::CAL3_S2;
};

}  // namespace gtcal
