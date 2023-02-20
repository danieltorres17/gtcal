#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3Fisheye.h>
#include <gtsam/geometry/PinholeCamera.h>

#include <ceres/ceres.h>
#include <ceres/loss_function.h>

#include "gtcal/utils.h"

namespace gtcal {
class ReprojectionErrorResidual {
public:
  ReprojectionErrorResidual(const gtsam::Point2& uv, const gtsam::Point3& pt3d,
                            const gtsam::Cal3Fisheye::shared_ptr& cmod_params);

  bool operator()(const double* const pose_target_cam_arr, double* residuals) const;

  static ceres::CostFunction* Create(const gtsam::Point2& uv, const gtsam::Point3& pt3d,
                                     const gtsam::Cal3Fisheye::shared_ptr& cmod_params);

private:
  const gtsam::Point2 uv_ = gtsam::Point2::Constant(utils::NaN);
  const gtsam::Point3 pt3d_ = gtsam::Point3::Constant(utils::NaN);
  const gtsam::Cal3Fisheye::shared_ptr cmod_params_ = nullptr;
};

class PoseSolver {
public:
  PoseSolver(const bool verbose = false);
  bool Solve(const gtsam::Point2Vector& uvs, const gtsam::Point3Vector& pts3d_target,
             const gtsam::Cal3Fisheye::shared_ptr& cmod_params, gtsam::Pose3& pose_target_cam) const;

private:
  ceres::Solver::Options options_;
  ceres::LossFunction* loss_function_ = nullptr;
  const double loss_scaling_param_ = 1.0;
};
}  // namespace gtcal