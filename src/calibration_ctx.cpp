#include "gtcal/calibration_ctx.hpp"

#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/inference/Symbol.h>

#include <unordered_set>

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

gtsam::Values CalibrationCtx::processFrames(const std::vector<CalibrationCtx::Frame>& frames) {
  // Add landmark constraints to graph.
  addLandmarkFactors(frames, state_->graph, state_->current_estimate, state_->isam);

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
                        state_->current_estimate, state_->isam);
  }

  // After processing all frames, run optimization.
  state_->isam.update(state_->graph, state_->current_estimate);
  state_->isam.update();
  state_->isam.update();

  state_->graph.resize(0);
  state_->current_estimate.clear();

  return state_->isam.calculateEstimate();
}

void CalibrationCtx::addLandmarkFactors(const std::vector<Frame>& frames, gtsam::NonlinearFactorGraph& graph,
                                        gtsam::Values& values, const gtsam::ISAM2& isam) const {
  std::unordered_set<size_t> landmark_ids;
  for (const auto& frame : frames) {
    for (const auto& meas : frame.measurements) {
      if (landmark_ids.count(meas.point_id) == 0 && !isam.valueExists(L(meas.point_id))) {
        landmark_ids.insert(meas.point_id);

        // Add nonlinear equality constraint to landmark to fix during optimization.
        graph.emplace_shared<gtsam::NonlinearEquality<gtsam::Point3>>(
            L(meas.point_id), target_->pointsTarget().at(meas.point_id));

        // Add to values.
        values.insert(L(meas.point_id), target_->pointsTarget().at(meas.point_id));
      }
    }
  }
}

void CalibrationCtx::addSFMFactors(const size_t camera_id, const Camera::Ptr& camera,
                                   const size_t num_camera_update,
                                   const std::vector<Measurement>& measurements,
                                   const gtsam::Pose3& pose_target_cam_estimate,
                                   gtsam::NonlinearFactorGraph& graph, gtsam::Values& values) const {
  // Get camera model.
  const auto model_type = camera->modelType();
  const gtsam::Symbol camera_pose_key = camera_rig_->cameraPoseSymbol(camera_id, num_camera_update);
  const gtsam::Symbol camera_calibration_key = camera_rig_->calibrationSymbol(camera_id);

  if (model_type == Camera::ModelType::CAL3_S2) {
    // Add SFM factors.
    for (const auto& meas : measurements) {
      // Add SFM factor.
      graph.emplace_shared<gtsam::GeneralSFMFactor2<gtsam::Cal3_S2>>(
          meas.uv, noise_models_->pixel_meas_noise_model, camera_pose_key, L(meas.point_id),
          camera_calibration_key);
    }
  }

  // Add initial camera pose estimate to values.
  graph.addPrior(camera_pose_key, pose_target_cam_estimate);
  values.insert<gtsam::Pose3>(camera_pose_key, pose_target_cam_estimate);
}

void CalibrationCtx::addCalibrationPrior(const size_t camera_id, const std::shared_ptr<gtcal::Camera>& camera,
                                         gtsam::NonlinearFactorGraph& graph, gtsam::Values& values,
                                         const gtsam::ISAM2& isam) const {
  const auto model_type = camera->modelType();
  const gtsam::Symbol camera_calibration_key = camera_rig_->calibrationSymbol(camera_id);
  if (isam.valueExists(camera_calibration_key)) {
    return;
  }

  if (model_type == Camera::ModelType::CAL3_S2) {
    const auto cmod = std::get<std::shared_ptr<CameraWrapper<gtsam::Cal3_S2>>>(camera->cameraVariant());
    assert(cmod);

    graph.addPrior(camera_calibration_key, cmod->calibration(), noise_models_->calibration_noise_model);
    values.insert(camera_calibration_key, cmod->calibration());
  }
}

void CalibrationCtx::updateCameraRigCalibrations(const gtsam::Values& values) {
  const auto keys_vector = values.keys();

  // Find the camera keys in the values.
  for (const auto& key : keys_vector) {
    const gtsam::Symbol key_sym(key);
    if (key_sym.chr() == 'k') {
      // Find the camera id.
      const size_t camera_id = camera_rig_->cameraCalibrationSymbolToIndex(key_sym);

      // Get the camera model type. TODO: find better way to handle this.
      const auto model_type = camera_rig_->cameras.at(camera_id)->modelType();
      if (model_type == Camera::ModelType::CAL3_S2) {
        camera_rig_->cameras.at(camera_id)->updateCalibration(values.at<gtsam::Cal3_S2>(key_sym));
      } else if (model_type == Camera::ModelType::CAL3_FISHEYE) {
        camera_rig_->cameras.at(camera_id)->updateCalibration(values.at<gtsam::Cal3Fisheye>(key_sym));
      } else {
        assert(false && "Invalid camera model.");
      }
    }
  }
}

void CalibrationCtx::updateCameraPosesInTargetFrame(const gtsam::Values& values) {
  const auto keys_vector = values.keys();

  // If the camera pose in the target frame was successfully estimated, update in state object.
  // Keep track of camera pose indices updated.
  std::unordered_set<size_t> camera_pose_indices_updated;

  for (const auto& key : keys_vector) {
    const gtsam::Symbol key_sym(key);

    if (key_sym.chr() == 'l' || key_sym.chr() == 'k') {
      continue;
    }

    // Get the corresponding camera id from the key.
    const size_t camera_id = camera_rig_->cameraPoseSymbolToIndex(key_sym);
    camera_pose_indices_updated.insert(camera_id);
    const gtsam::Pose3 pose_target_cid = values.at<gtsam::Pose3>(key_sym);
    const size_t pose_index = static_cast<size_t>(key_sym.index());

    // Check if camera id exists in the map.
    if (state_->poses_target_cam_map.count(camera_id) == 0) {
      state_->poses_target_cam_map.insert({camera_id, {}});
    }

    // If the pose index, is already in the map vector, update it.
    if (pose_index + 1 > state_->poses_target_cam_map.at(camera_id).size()) {
      state_->poses_target_cam_map.at(camera_id).resize(pose_index + 1);
    }
    state_->poses_target_cam_map.at(camera_id).at(pose_index) = pose_target_cid;
  }

  // Add nullopt for camera poses that were not updated.
  for (size_t ii = 0; ii < camera_rig_->numCameras(); ii++) {
    if (camera_pose_indices_updated.count(ii) == 0) {
      state_->poses_target_cam_map.at(ii).push_back(std::nullopt);
    }
  }
}

size_t CalibrationCtx::State::cameraIterationCount(const size_t camera_id) {
  if (camera_iter_count_map.count(camera_id) == 0) {
    camera_iter_count_map.insert({camera_id, 0});

    return camera_iter_count_map.at(camera_id);
  }

  return ++camera_iter_count_map.at(camera_id);
}

}  // namespace gtcal