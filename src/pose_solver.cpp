#include "gtcal/pose_solver.h"

namespace gtcal {
ReprojectionErrorResidual::ReprojectionErrorResidual(const gtsam::Point2& uv, const gtsam::Point3& pt3d,
                                                     const boost::shared_ptr<gtsam::Cal3Fisheye>& cmod_params)
  : uv_(uv), pt3d_(pt3d), cmod_params_(cmod_params) {}

bool ReprojectionErrorResidual::operator()(const double* const pose_target_cam_arr, double* residuals) const {
  // Get pose vector using current pose estimate.
  const gtsam::Rot3 rot =
      gtsam::Rot3::RzRyRx(pose_target_cam_arr[3], pose_target_cam_arr[4], pose_target_cam_arr[5]);
  const gtsam::Point3 xyz = {pose_target_cam_arr[0], pose_target_cam_arr[1], pose_target_cam_arr[2]};
  const gtsam::Pose3 pose_target_cam = gtsam::Pose3(rot, xyz);

  // Calculate 3D point using pose.
  const gtsam::Point3 pt3d_cam = pose_target_cam.transformTo(pt3d_);

  // Project 3D point using new pose estimate.
  gtsam::PinholeCamera<gtsam::Cal3Fisheye> cmod(pose_target_cam, *cmod_params_);
  const gtsam::Point2 uv = cmod.project(pt3d_cam);

  // Calculate residuals
  residuals[0] = uv.x() - uv_.x();
  residuals[1] = uv.y() - uv_.y();
  return true;
}

ceres::CostFunction* ReprojectionErrorResidual::Create(
    const gtsam::Point2& uv, const gtsam::Point3& pt3d,
    const boost::shared_ptr<gtsam::Cal3Fisheye>& cmod_params) {
  return new ceres::NumericDiffCostFunction<ReprojectionErrorResidual, ceres::CENTRAL, 2, 6>(
      new ReprojectionErrorResidual(uv, pt3d, cmod_params));
}

PoseSolver::PoseSolver(const bool verbose) {
  // Set solver options.
  options_.max_num_iterations = 100;
  options_.minimizer_progress_to_stdout = verbose ? true : false;
  options_.function_tolerance = 1e-6;

  // Set loss function.
  loss_function_ = new ceres::HuberLoss(loss_scaling_param_);
}

bool PoseSolver::Solve(const gtsam::Point2Vector& uvs, const gtsam::Point3Vector& pts3d_target,
                       const boost::shared_ptr<gtsam::Cal3Fisheye>& cmod_params,
                       gtsam::Pose3& pose_target_cam) {
  // Ensure the same number of pixels and 3D points were given.
  assert(uvs.size() == pts3d_target.size());

  // Create initial pose estimate vector.
  const gtsam::Point3& xyz = pose_target_cam.translation();
  const gtsam::Point3 rpy = pose_target_cam.rotation().rpy();
  double pose_init[6] = {xyz.x(), xyz.y(), xyz.z(), rpy.x(), rpy.y(), rpy.z()};

  // Create residuals and solve problem.
  for (size_t ii = 0; ii < uvs.size(); ii++) {
    auto cost_functor = ReprojectionErrorResidual::Create(uvs.at(ii), pts3d_target.at(ii), cmod_params);
    problem_.AddResidualBlock(cost_functor, loss_function_, pose_init);
  }

  // Solve problem and update pose argument.
  ceres::Solve(options_, &problem_, &summary_);
  gtsam::Rot3 R = gtsam::Rot3::RzRyRx(pose_init[3], pose_init[4], pose_init[5]);
  gtsam::Point3 t = {pose_init[0], pose_init[1], pose_init[2]};
  pose_target_cam = gtsam::Pose3(R, t);

  return true;
}

}  // namespace gtcal