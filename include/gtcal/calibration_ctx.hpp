#pragma once

#include <unordered_map>

#include "gtcal/camera_rig.hpp"
#include "gtcal/pose_solver_gtsam.hpp"
#include "gtcal/calibration_target.hpp"
#include "gtcal/measurement.hpp"

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/NoiseModel.h>

namespace gtcal {

class CalibrationCtx {
public:
  struct Frame {
    Frame(const size_t camera_id, const std::vector<Measurement>& measurements_vec,
          const gtsam::Pose3& pose_target_cam_est)
      : cid(camera_id), measurements(measurements_vec), pose_target_cam_estimate(pose_target_cam_est) {}

    const size_t cid;
    const std::vector<Measurement> measurements;
    const gtsam::Pose3 pose_target_cam_estimate;
  };

  struct State {
    size_t num_iterations = 0;
    std::unordered_map<size_t, size_t> camera_iter_count_map;

    gtsam::ISAM2 isam;
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values current_estimate;

    size_t cameraIterationCount(const size_t camera_id);
  };

  struct NoiseModels {
    gtsam::noiseModel::Isotropic::shared_ptr pixel_meas_noise_model = nullptr;
    gtsam::noiseModel::Diagonal::shared_ptr calibration_noise_model = nullptr;

    NoiseModels()
      : pixel_meas_noise_model(gtsam::noiseModel::Isotropic::Sigma(2, 1.0)),
        calibration_noise_model(
            gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(5) << 50., 50., 0.001, 50., 50).finished())) {}
  };

  CalibrationCtx(const CameraRig::Ptr camera_rig, const CalibrationTarget::Ptr target);
  void processFrames(const std::vector<Frame>& frames);
  void addSFMFactors(const size_t camera_id, const Camera::Ptr& camera, const size_t num_camera_update,
                     const std::vector<Measurement>& measurements,
                     const gtsam::Pose3& pose_target_cam_estimate, gtsam::NonlinearFactorGraph& graph,
                     gtsam::Values& values) const;

  void addCalibrationPrior(const size_t camera_id, const std::shared_ptr<gtcal::Camera>& camera,
                           gtsam::NonlinearFactorGraph& graph, gtsam::Values& values) const;

  const std::shared_ptr<const State> state() const { return state_; }

private:
  CameraRig::Ptr camera_rig_ = nullptr;
  CalibrationTarget::Ptr target_ = nullptr;

  std::unique_ptr<PoseSolverGtsam> pose_solver_ = nullptr;
  std::unique_ptr<NoiseModels> noise_models_ = nullptr;
  std::shared_ptr<State> state_ = nullptr;
};

}  // namespace gtcal