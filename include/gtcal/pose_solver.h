#pragma once

#include <ceres/ceres.h>
#include <ceres/loss_function.h>

#include "gtcal/camera.h"
#include "gtcal/utils.h"

namespace gtcal {

class ReprojectionErrorResidual {
public:
  /**
   * @brief Construct a new Reprojection Error Residual object.
   *
   * @param uv measurement taken at new camera pose (pose we're trying to solve for).
   * @param pt3d_target target point in the target frame.
   * @param cmod_params camera model parameters. Currently only supports gtsam::Cal3Fisheye but will modify
   * this to support other camera models soon.
   */
  ReprojectionErrorResidual(const gtsam::Point2& uv, const gtsam::Point3& pt3d_target,
                            const std::shared_ptr<Camera>& cmod);

  /**
   * @brief Residual functor for Ceres solver. Return true if a measurement was successfully calculated for
   * the target point and it's within the image bounds. Otherwise, return false.
   *
   * @param pose_target_cam_arr latest estimate of the camera pose in the target frame.
   * @param residuals residuals for functor.
   * @return true
   * @return false
   */
  bool operator()(const double* const pose_target_cam_arr, double* residuals) const;

  /**
   * @brief Return a pointer to a ceres::CostFunction object.
   *
   * @param uv measurement taken at new camera pose (pose we're trying to solve for).
   * @param pt3d_target target point in target frame.
   * @param cmod_params camera model parameters.
   * @return ceres::CostFunction*
   */
  static ceres::CostFunction* Create(const gtsam::Point2& uv, const gtsam::Point3& pt3d_target,
                                     const std::shared_ptr<Camera>& cmod);

private:
  const gtsam::Point2 uv_ = gtsam::Point2::Constant(utils::NaN);
  const gtsam::Point3 pt3d_target_ = gtsam::Point3::Constant(utils::NaN);
  const std::shared_ptr<Camera> cmod_ = nullptr;
  const size_t width_ = 0;
  const size_t height_ = 0;
};

class PoseSolver {
public:
  /**
   * @brief Construct a new Pose Solver object
   *
   * @param verbose if true, will print the solver summary to stdout.
   */
  PoseSolver(const bool verbose = false);

  /**
   * @brief Destroy the Pose Solver object.
   *
   */
  ~PoseSolver();

  /**
   * @brief Return true if the solver was able to solve for the camera pose in the target frame. Return false
   * otherwise.
   *
   * @param measurements vector of measurements taken at new camera pose (pose we're trying to solve for).
   * @param pts3d_target target points in the target frame.
   * @param camera gtcal::Camera shared pointer.
   * @param pose_target_cam initial estimate for the camera pose in the target frame.
   * @return true
   * @return false
   */
  bool solve(const std::vector<Measurement>& measurements, const std::vector<gtsam::Point3>& pts3d_target,
             const std::shared_ptr<Camera>& camera, gtsam::Pose3& pose_target_cam) const;

private:
  ceres::Solver::Options options_;
  ceres::LossFunction* loss_function_ = nullptr;
  const double loss_scaling_param_ = 1.0;
};

}  // namespace gtcal