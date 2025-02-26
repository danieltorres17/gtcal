#include "gtcal/calibration_ctx.hpp"

#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/inference/Symbol.h>

using gtsam::symbol_shorthand::K;
using gtsam::symbol_shorthand::L;
using gtsam::symbol_shorthand::X;

namespace gtcal {

CalibrationCtx::CalibrationCtx(const CameraRig::Ptr camera_rig, const CalibrationTarget::Ptr target)
  : camera_rig_(camera_rig), target_(target) {
  // Initialize pose solver.
  pose_solver_ = std::make_unique<PoseSolverGtsam>();

  // Initialize noise models struct.
  noise_models_ = std::make_unique<NoiseModels>();

  // Initialize state.
  state_ = std::make_shared<State>();
  gtsam::ISAM2Params params;
  params.relinearizeThreshold = 0.01;
  params.relinearizeSkip = 1;
  state_->isam = gtsam::ISAM2(params);
}

void CalibrationCtx::processFrames(const std::vector<CalibrationCtx::Frame>& frames) {
  state_->graph.resize(0);
  state_->current_estimate.clear();

  for (size_t ii = 0; ii < frames.size(); ii++) {
    const auto& frame = frames.at(ii);

    // Attempt to refine camera pose in target frame using initial estimate and observations.
    const std::optional<gtsam::Pose3> pose_target_cam_refined_opt =
        pose_solver_->solve(frame.measurements, target_->pointsTarget(), camera_rig_->cameras.at(frame.cid),
                            frame.pose_target_cam_estimate);
    if (!pose_target_cam_refined_opt) {
      continue;
    }
    const gtsam::Pose3& pose_target_cam_refined = *pose_target_cam_refined_opt;

    // Add SFM factors to graph.
    addSFMFactors(frame.cid, camera_rig_->cameras.at(frame.cid), state_->cameraIterationCount(frame.cid),
                  frame.measurements, pose_target_cam_refined, state_->graph, state_->current_estimate);

    // Add calibration prior to graph.
    addCalibrationPrior(frame.cid, camera_rig_->cameras.at(frame.cid), state_->graph,
                        state_->current_estimate);
  }

  // After processing all frames, run optimization.
  state_->isam.update(state_->graph, state_->current_estimate);
  state_->isam.update();
}

void CalibrationCtx::addSFMFactors(const size_t camera_id, const Camera::Ptr& camera,
                                   const size_t num_camera_update,
                                   const std::vector<Measurement>& measurements,
                                   const gtsam::Pose3& pose_target_cam_estimate,
                                   gtsam::NonlinearFactorGraph& graph, gtsam::Values& values) const {
  // Get camera model.
  const auto model_type = camera->modelType();

  if (model_type == Camera::ModelType::CAL3_S2) {
    // Add SFM factors.
    for (const auto& meas : measurements) {
      // Add SFM factor.
      graph.emplace_shared<gtsam::GeneralSFMFactor2<gtsam::Cal3_S2>>(
          meas.uv, noise_models_->pixel_meas_noise_model, X(num_camera_update), L(meas.point_id),
          K(camera_id));

      // Add nonlinear equality constraint to landmark to fix during optimization.
      graph.emplace_shared<gtsam::NonlinearEquality<gtsam::Point3>>(
          L(meas.point_id), target_->pointsTarget().at(meas.point_id));

      // Add to values.
      values.insert(L(meas.point_id), target_->pointsTarget().at(meas.point_id));
    }
  }

  // Add initial camera pose estimate to values.
  values.insert<gtsam::Pose3>(X(num_camera_update), pose_target_cam_estimate);
}

void CalibrationCtx::addCalibrationPrior(const size_t camera_id, const std::shared_ptr<gtcal::Camera>& camera,
                                         gtsam::NonlinearFactorGraph& graph, gtsam::Values& values) const {
  const auto model_type = camera->modelType();

  if (model_type == Camera::ModelType::CAL3_S2) {
    const auto cmod = std::get<std::shared_ptr<CameraWrapper<gtsam::Cal3_S2>>>(camera->cameraVariant());
    assert(cmod);

    graph.addPrior(K(camera_id), cmod->calibration(), noise_models_->calibration_noise_model);
    values.insert(K(camera_id), cmod->calibration());
  }
}

size_t CalibrationCtx::State::cameraIterationCount(const size_t camera_id) {
  if (camera_iter_count_map.count(camera_id) == 0) {
    camera_iter_count_map.insert({camera_id, 0});

    return camera_iter_count_map.at(camera_id);
  }

  return camera_iter_count_map.at(camera_id)++;
}

}  // namespace gtcal