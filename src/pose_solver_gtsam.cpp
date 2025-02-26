#include "gtcal/pose_solver_gtsam.hpp"
#include "gtcal/measurement.hpp"

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

using gtsam::symbol_shorthand::K;
using gtsam::symbol_shorthand::L;
using gtsam::symbol_shorthand::X;

namespace gtcal {

PoseSolverGtsam::PoseSolverGtsam(const Options& options) : options_(options) {}

std::optional<gtsam::Pose3> PoseSolverGtsam::solve(const std::vector<Measurement>& measurements,
                                                   const std::vector<gtsam::Point3>& pts3d_target,
                                                   const std::shared_ptr<const Camera>& camera,
                                                   const gtsam::Pose3& pose_target_cam_initial) const {
  if (measurements.empty()) {
    return {};
  }

  // Create factor graph and initial estimate values.
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial_estimate;
  const gtsam::Key camera_pose_key = X(0);

  // Helper function to add factors to graph based on camera model type.
  auto AddGenericProjectionFactors = [&camera_pose_key, &pts3d_target, &graph, &initial_estimate,
                                      &options_ = this->options_,
                                      &camera]<typename T>(const std::vector<Measurement>& measurements,
                                                           const std::shared_ptr<T>& K) -> void {
    for (size_t ii = 0; ii < measurements.size(); ii++) {
      const Measurement& meas = measurements.at(ii);
      // Add projection factor for each target point.
      graph.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, T>>(
          meas.uv, options_.pixel_meas_noise_model, camera_pose_key, L(meas.point_id), K);
      // Add nonlinear equality constraint for each target point. Fixes them in the optimization.
      graph.emplace_shared<gtsam::NonlinearEquality<gtsam::Point3>>(L(meas.point_id),
                                                                    pts3d_target.at(meas.point_id));
      // Add initial estimate for each target point.
      initial_estimate.insert(L(meas.point_id), pts3d_target.at(meas.point_id));
    }
  };

  // Add camera pose initial estimate to values.
  initial_estimate.insert<gtsam::Pose3>(camera_pose_key, pose_target_cam_initial);

  // Add projection factors to graph based on camera type.
  const auto model_type = camera->modelType();
  if (model_type == Camera::ModelType::CAL3_S2) {  // Cal3_S2.
    // Get camera model and calibration.
    const auto cmod = std::get<std::shared_ptr<CameraWrapper<gtsam::Cal3_S2>>>(camera->cameraVariant());
    assert(cmod && "[PoseSolverGtsam::solve] Camera model is not of type Cal3_S2.");
    const gtsam::Cal3_S2::shared_ptr K = std::make_shared<gtsam::Cal3_S2>(cmod->calibration());

    // Populate factor graph.
    AddGenericProjectionFactors(measurements, K);
  } else if (model_type == Camera::ModelType::CAL3_FISHEYE) {  // Cal3_Fisheye.
    const auto cmod = std::get<std::shared_ptr<CameraWrapper<gtsam::Cal3Fisheye>>>(camera->cameraVariant());
    assert(cmod && "[PoseSolverGtsam::solve] Camera model is not of type Cal3Fisheye.");
    const gtsam::Cal3Fisheye::shared_ptr K = std::make_shared<gtsam::Cal3Fisheye>(cmod->calibration());

    // Populate factor graph.
    AddGenericProjectionFactors(measurements, K);
  }

  // Solve and return the optimized camera pose. TODO: add check in case the optimization fails.
  gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initial_estimate).optimize();

  return result.at<gtsam::Pose3>(camera_pose_key);
}

}  // namespace gtcal
