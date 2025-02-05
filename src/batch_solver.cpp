#include "gtcal/batch_solver.h"
#include "gtcal/utils.h"

#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/inference/Symbol.h>

using gtsam::symbol_shorthand::K;
using gtsam::symbol_shorthand::L;
using gtsam::symbol_shorthand::X;

namespace gtcal {

BatchSolver::State::State(const std::vector<std::shared_ptr<Camera>>& camera_models)
  : cameras(camera_models) {
  // Update the number of camera updates.
  num_camera_updates.resize(camera_models.size(), 0);

  // Set the ISAM2 parameters.
  gtsam::ISAM2Params params;
  params.relinearizeThreshold = 0.01;
  params.relinearizeSkip = 1;
  isam = gtsam::ISAM2(params);
}

BatchSolver::BatchSolver(const std::vector<gtsam::Point3>& pts3d_target, const Options& options)
  : pts3d_target_(pts3d_target), options_(options) {}

gtsam::Values BatchSolver::solve(State& state) const {
  // Update ISAM and get updates.
  state.isam.update(state.graph, state.current_estimate);
  for (size_t ii = 1; ii < state.num_isam_iterations; ii++) {
    state.isam.update();
  }

  // Clear graph and reset values.
  state.graph.resize(0);
  state.current_estimate.clear();

  return state.isam.calculateEstimate();
}

void BatchSolver::addPriorsAndFactors(const std::vector<Measurement>& measurements, State& state) const {
  // Check that all measurements are from the same camera.
  const size_t camera_index = measurements.front().camera_id;
  const bool all_same_camera =
      std::all_of(measurements.begin(), measurements.end(),
                  [&camera_index](const Measurement& meas) { return meas.camera_id == camera_index; });
  assert(all_same_camera &&
         "[BatchSolver::addPriorsAndFactors] All measurements must be from the same camera.");

  // If this is the first time a solution is being found, add the landmark priors and locations to the graph.
  // Also add the landmark points to the current estimate.
  if (!state.first_iteration_complete) {
    // Add priors to graph.
    addLandmarkPriors(pts3d_target_, state.graph);
    // Points to values.
    for (size_t ii = 0; ii < pts3d_target_.size(); ii++) {
      state.current_estimate.insert(L(ii), pts3d_target_.at(ii));
    }

    // Add camera calibration prior to initial values and graph.
    addCalibrationPriors(camera_index, state.cameras.at(camera_index), state.graph, state.current_estimate);

    state.first_iteration_complete = true;
  }

  // Check the number of times a camera has been updated.
  const size_t num_camera_updates = state.num_camera_updates.at(camera_index);
  if (num_camera_updates == 0) {
    // If it's the first time the camera has been updated, add a pose prior on the graph.
    addPosePrior(camera_index, state.cameras.at(camera_index)->pose(), state.graph);
  }

  // Add pose initial estimate to the values object.
  state.current_estimate.insert(X(num_camera_updates), state.cameras.at(camera_index)->pose());

  // Add landmark factors.
  addLandmarkFactors(camera_index, state.cameras.at(camera_index), num_camera_updates, measurements,
                     state.graph);

  state.num_camera_updates.at(camera_index) += 1;
}

void BatchSolver::addCalibrationPriors(const size_t camera_index,
                                       const std::shared_ptr<gtcal::Camera>& camera,
                                       gtsam::NonlinearFactorGraph& graph, gtsam::Values& values) const {
  // Add camera calibration prior to initial values and graph according to type of model.
  // TODO: Add support for other camera models and clean up noise model vectors.
  const auto model_type = camera->modelType();
  if (model_type == Camera::ModelType::CAL3_S2) {  // Cal3_S2.
    // Get camera model.
    const auto cmod = std::get<std::shared_ptr<CameraWrapper<gtsam::Cal3_S2>>>(camera->cameraVariant());
    assert(cmod && "[BatchSolver::addCalibrationPriors] Camera model is not of type Cal3_S2.");

    // Add calibration prior to initial values and graph.
    values.insert(K(camera_index), cmod->calibration());
    graph.addPrior(
        K(camera_index), cmod->calibration(),
        gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(5) << 50., 50., 0.001, 50., 50.).finished()));
  } else if (model_type == Camera::ModelType::CAL3_FISHEYE) {  // Cal3_Fisheye.
    const auto cmod = std::get<std::shared_ptr<CameraWrapper<gtsam::Cal3Fisheye>>>(camera->cameraVariant());
    assert(cmod && "[BatchSolver::addCalibrationPriors] Camera model is not of type Cal3Fisheye.");

    // Add calibration prior to initial values and graph.
    values.insert(K(camera_index), cmod->calibration());
    graph.addPrior(
        K(camera_index), cmod->calibration(),
        gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(9) << 50., 50., 0.001, 50., 50., 0.01, 0.001, 0.001, 0.001).finished()));
  }
}

void BatchSolver::addLandmarkPriors(const std::vector<gtsam::Point3>& pts3d_target,
                                    gtsam::NonlinearFactorGraph& graph) const {
  // Add landmark priors to graph.
  for (size_t ii = 0; ii < pts3d_target.size(); ii++) {
    graph.addPrior(L(ii), pts3d_target.at(ii), options_.landmark_prior_noise_model);
  }
}

void BatchSolver::addLandmarkFactors(const size_t camera_index, const std::shared_ptr<gtcal::Camera>& camera,
                                     const size_t num_camera_update,
                                     const std::vector<Measurement>& measurements,
                                     gtsam::NonlinearFactorGraph& graph) const {
  // Get camera model.
  const auto model_type = camera->modelType();
  if (model_type == Camera::ModelType::CAL3_S2) {
    const auto cmod = std::get<std::shared_ptr<CameraWrapper<gtsam::Cal3_S2>>>(camera->cameraVariant());
    assert(cmod && "[BatchSolver::addLandmarkFactors] Camera model is not of type Cal3_S2.");

    // Add landmark factors to graph.
    for (const auto& meas : measurements) {
      // Add landmark measurement to graph.
      graph.emplace_shared<gtsam::GeneralSFMFactor2<gtsam::Cal3_S2>>(
          meas.uv, options_.pixel_meas_noise_model, X(num_camera_update), L(meas.point_id), K(camera_index));
    }
  }
}

void BatchSolver::addPosePrior(const size_t camera_index, const gtsam::Pose3& pose_target_cam,
                               gtsam::NonlinearFactorGraph& graph) const {
  // Add pose prior to graph.
  graph.addPrior(X(0), pose_target_cam, options_.pose_prior_noise_model);
}

}  // namespace gtcal
