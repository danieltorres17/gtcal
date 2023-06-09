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

BatchSolver::BatchSolver(const gtsam::Point3Vector& pts3d_target, const Options& options)
  : pts3d_target_(pts3d_target), options_(options) {}

void BatchSolver::updateCameraIndicesMap(const size_t camera_index, State& state) const {
  // Check if the camera index is already in the map.
  if (!state.camera_indices.contains(camera_index)) {
    // If not, add it to the map.
    state.camera_indices[camera_index] = state.numCameras();
  }

  // Update the number of camera updates.
  state.num_camera_updates.at(state.camera_indices.at(camera_index)) += 1;
}

void BatchSolver::solve(const std::vector<Measurement>& measurements, State& state) const {
  // Check that all measurements are from the same camera.
  const size_t& camera_index = measurements.front().camera_id;
  const bool all_same_camera =
      std::all_of(measurements.begin(), measurements.end(),
                  [&camera_index](const Measurement& meas) { return meas.camera_id == camera_index; });
  assert(all_same_camera && "[BatchSolver::solve] All measurements must be from the same camera.");

  // Check the camera id and update map if necessary.
  // updateCameraIndicesMap(camera_index, state);

  // Create graph and initial values.
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial_values;

  // Add camera calibration prior to initial values and graph.
  addCalibrationPriors(camera_index, state.cameras.at(camera_index), graph, initial_values);

  // Get the number of times the camera has been updated and add landmark factors.
  const size_t num_camera_updates = state.num_camera_updates.at(camera_index);
  if (num_camera_updates == 0) {
    // If it's the first time the camera has been updated, add a pose prior on the graph.
    addPosePrior(camera_index, state.cameras.at(camera_index)->pose(), graph);
    // Also add landmark priors.
    addLandmarkPriors(measurements, pts3d_target_, graph);
    return;
  }
  // Add landmark factors.
  addLandmarkFactors(camera_index, state.cameras.at(camera_index), num_camera_updates, measurements, graph);

  // Update iSAM with the new factors.
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

void BatchSolver::addLandmarkPriors(const std::vector<Measurement>& measurements,
                                    const gtsam::Point3Vector& pts3d_target,
                                    gtsam::NonlinearFactorGraph& graph) const {
  // Add landmark priors to graph.
  for (const auto& meas : measurements) {
    graph.addPrior(L(meas.point_id), pts3d_target.at(meas.point_id), options_.landmark_prior_noise_model);
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
      // Landmark measurement.
      const gtsam::Point2& uv = meas.uv;
      // Add to graph.
      graph.emplace_shared<gtsam::GeneralSFMFactor2<gtsam::Cal3_S2>>(
          uv, options_.pixel_meas_noise_model, X(num_camera_update), L(meas.point_id), K(camera_index));
    }
  }
}

void BatchSolver::addPosePrior(const size_t camera_index, const gtsam::Pose3& pose_target_cam,
                               gtsam::NonlinearFactorGraph& graph) const {
  // Add pose prior to graph.
  graph.addPrior(X(0), pose_target_cam, options_.pose_prior_noise_model);
}

}  // namespace gtcal
