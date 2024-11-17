#include "gtcal/pose_solver_gtsam.h"
#include "gtcal/utils.h"

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
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

bool PoseSolverGtsam::solve(const std::vector<Measurement>& measurements,
                            const gtsam::Point3Vector& pts3d_target,
                            const std::shared_ptr<const Camera>& camera,
                            gtsam::Pose3& pose_initial_target_cam) const {
  // Create smart factors based on camera calibration type.
  auto CreateSmartFactors = [&pts3d_target, &options = this->options_,
                             &camera]<typename T>(const std::shared_ptr<T>& K)
      -> std::vector<typename gtsam::SmartProjectionPoseFactor<T>::shared_ptr> {
    std::vector<gtsam::SmartProjectionPoseFactor<T>::shared_ptr> smart_factors;
    smart_factors.reserve(pts3d_target.size());
    for (const auto& pt : pts3d_target) {
      smart_factors.push_back(
          std::make_shared<gtsam::SmartProjectionPoseFactor<T>>(options.pixel_meas_noise_model, K));
    }

    return smart_factors;
  };

  // Create factor graph.
  gtsam::NonlinearFactorGraph graph;

  // Add camera pose prior to graph and initial estimate to values.
  const gtsam::Key camera_pose_key = X(0);
  graph.addPrior(camera_pose_key, pose_initial_target_cam, options_.pose_prior_noise_model);
  gtsam::Values initial_estimate;
  initial_estimate.insert<gtsam::Pose3>(camera_pose_key, pose_initial_target_cam);

  // Add projection factors to graph based on camera type.
  const auto model_type = camera->modelType();
  if (model_type == Camera::ModelType::CAL3_S2) {  // Cal3_S2.
    // Get camera model and calibration.
    const auto cmod = std::get<std::shared_ptr<CameraWrapper<gtsam::Cal3_S2>>>(camera->cameraVariant());
    assert(cmod && "[PoseSolverGtsam::solve] Camera model is not of type Cal3_S2.");
    const gtsam::Cal3_S2::shared_ptr K = std::make_shared<gtsam::Cal3_S2>(cmod->calibration());

    // Get smart factors.
    const std::vector<gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2>::shared_ptr> smart_factors =
        CreateSmartFactors(K);

    // Add projection factors to graph.
    for (const auto& meas : measurements) {
      smart_factors.at(meas.point_id)->add(meas.uv, camera_pose_key);
      graph.push_back(smart_factors.at(meas.point_id));
    }
  } else if (model_type == Camera::ModelType::CAL3_FISHEYE) {  // Cal3_Fisheye.
    const auto cmod = std::get<std::shared_ptr<CameraWrapper<gtsam::Cal3Fisheye>>>(camera->cameraVariant());
    assert(cmod && "[PoseSolverGtsam::solve] Camera model is not of type Cal3Fisheye.");
    const gtsam::Cal3Fisheye::shared_ptr K = std::make_shared<gtsam::Cal3Fisheye>(cmod->calibration());

    // Get smart factors.
    const std::vector<gtsam::SmartProjectionPoseFactor<gtsam::Cal3Fisheye>::shared_ptr> smart_factors =
        CreateSmartFactors(K);

    // Add projection factors to graph.
    for (const auto& meas : measurements) {
      smart_factors.at(meas.point_id)->add(meas.uv, camera_pose_key);
      graph.push_back(smart_factors.at(meas.point_id));
    }
  }

  // Solve and return the optimized camera pose. TODO: add check in case the optimization fails.
  gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initial_estimate).optimize();
  pose_initial_target_cam = result.at<gtsam::Pose3>(camera_pose_key);

  return true;
}

}  // namespace gtcal
