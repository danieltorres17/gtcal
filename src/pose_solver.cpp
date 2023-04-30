#include "gtcal/pose_solver.h"
#include <gtsam/geometry/PinholeCamera.h>

namespace gtcal {

ReprojectionErrorResidual::ReprojectionErrorResidual(const gtsam::Point2& uv,
                                                     const gtsam::Point3& pt3d_target,
                                                     const std::shared_ptr<Camera>& cmod)
  : uv_(uv), pt3d_target_(pt3d_target), cmod_(cmod), width_(cmod->width()), height_(cmod->height()) {}

bool ReprojectionErrorResidual::operator()(const double* const pose_target_cam_arr, double* residuals) const {
  // Get pose vector using current pose estimate.
  const gtsam::Rot3 rot =
      gtsam::Rot3::RzRyRx(pose_target_cam_arr[3], pose_target_cam_arr[4], pose_target_cam_arr[5]);
  const gtsam::Point3 xyz = {pose_target_cam_arr[0], pose_target_cam_arr[1], pose_target_cam_arr[2]};
  const gtsam::Pose3 pose_target_cam = gtsam::Pose3(rot, xyz);

  // Project 3D point using new pose estimate.
  cmod_->setCameraPose(pose_target_cam);
  const gtsam::Point2 uv = cmod_->project(pt3d_target_);
  if (!gtcal::utils::FilterPixelCoords(uv, width_, height_)) {
    return false;
  }

  // Calculate residuals
  residuals[0] = uv.x() - uv_.x();
  residuals[1] = uv.y() - uv_.y();
  return true;
}

ceres::CostFunction* ReprojectionErrorResidual::Create(const gtsam::Point2& uv,
                                                       const gtsam::Point3& pt3d_target,
                                                       const std::shared_ptr<Camera>& cmod) {
  return new ceres::NumericDiffCostFunction<ReprojectionErrorResidual, ceres::CENTRAL, 2, 6>(
      new ReprojectionErrorResidual(uv, pt3d_target, cmod));
}

PoseSolver::PoseSolver(const bool verbose) {
  // Set solver options.
  options_.max_num_iterations = 100;
  options_.linear_solver_type = ceres::DENSE_QR;
  options_.minimizer_progress_to_stdout = verbose ? true : false;
  options_.function_tolerance = 1e-6;

  // Set loss function.
  loss_function_ = new ceres::HuberLoss(loss_scaling_param_);
}

PoseSolver::~PoseSolver() {}

bool PoseSolver::solve(const gtsam::Point2Vector& uvs, const gtsam::Point3Vector& pts3d_target,
                       const std::shared_ptr<gtcal::Camera>& cmod, gtsam::Pose3& pose_target_cam) const {
  // Ensure the same number of pixels and 3D points were given.
  assert(uvs.size() == pts3d_target.size());

  // Create initial pose estimate array.
  const gtsam::Point3& xyz = pose_target_cam.translation();
  const gtsam::Point3 rpy = pose_target_cam.rotation().rpy();
  double pose_target_cam_arr[6] = {xyz.x(), xyz.y(), xyz.z(), rpy.x(), rpy.y(), rpy.z()};

  // Create residuals and solve problem.
  ceres::Problem problem;
  for (size_t ii = 0; ii < uvs.size(); ii++) {
    auto cost_functor = ReprojectionErrorResidual::Create(uvs.at(ii), pts3d_target.at(ii), cmod);
    problem.AddResidualBlock(cost_functor, loss_function_, pose_target_cam_arr);
  }

  // Solve problem and update pose argument.
  ceres::Solver::Summary summary;
  ceres::Solve(options_, &problem, &summary);
  gtsam::Rot3 R = gtsam::Rot3::RzRyRx(pose_target_cam_arr[3], pose_target_cam_arr[4], pose_target_cam_arr[5]);
  gtsam::Point3 t = {pose_target_cam_arr[0], pose_target_cam_arr[1], pose_target_cam_arr[2]};
  pose_target_cam = gtsam::Pose3(R, t);

  // Check if the problem successfully converged.
  if (summary.termination_type == ceres::FAILURE) {
    return false;
  }

  return true;
}

}  // namespace gtcal