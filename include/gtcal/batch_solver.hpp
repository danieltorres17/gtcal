#pragma once

#include <unordered_map>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/NoiseModel.h>

#include "gtcal/camera.hpp"

namespace gtcal {
struct Measurement;

class BatchSolver {
public:
  // State of the solver.
  struct State {
    // To keep track of camera models.
    std::vector<std::shared_ptr<Camera>> cameras;

    // To keep track of the number of times each camera's model and pose has been updated.
    std::vector<size_t> num_camera_updates;

    // To keep track of whether the first iteration has been done. Used to see if landmarks need to be added
    // to the ISAM2 instance.
    bool first_iteration_complete = false;

    // Solver components.
    gtsam::ISAM2 isam;
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values current_estimate;
    // Total number of times to run ISAM2::update(). Including initial update.
    size_t num_isam_iterations = 1;

    /**
     * @brief Return the number of cameras.
     *
     * @return size_t
     */
    size_t numCameras() const { return cameras.size(); }

    /**
     * @brief Construct a new State object.
     *
     * @param camera_models
     */
    explicit State(const std::vector<std::shared_ptr<Camera>>& camera_models);
  };

  // Noise models for the different types of factors.
  struct Options {
    // Default noise model for initial camera pose prior.
    gtsam::noiseModel::Diagonal::shared_ptr pose_prior_noise_model = nullptr;

    // Default noise model for the landmark priors.
    gtsam::noiseModel::Diagonal::shared_ptr landmark_prior_noise_model = nullptr;

    // Default noise model for the pixel measurements.
    gtsam::noiseModel::Isotropic::shared_ptr pixel_meas_noise_model = nullptr;

    Options()
      : pose_prior_noise_model(gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << gtsam::Vector3::Constant(0.1), gtsam::Vector3::Constant(0.01)).finished())),
        landmark_prior_noise_model(gtsam::noiseModel::Isotropic::Sigma(3, 1e-8)),
        pixel_meas_noise_model(gtsam::noiseModel::Isotropic::Sigma(2, 1.0)) {}
  };

public:
  BatchSolver(const std::vector<gtsam::Point3>& pts3d_target, const Options& options = Options());

  /**
   * @brief
   *
   * @param state
   * @return gtsam::Values
   */
  [[nodiscard]] gtsam::Values solve(State& state) const;

  /**
   * @brief
   *
   * @param measurements
   * @param state
   */
  void addPriorsAndFactors(const std::vector<Measurement>& measurements, State& state) const;

  /**
   * @brief
   *
   * @param camera_index
   * @param camera
   * @param graph
   * @param values
   */
  void addCalibrationPriors(const size_t camera_index, const std::shared_ptr<gtcal::Camera>& camera,
                            gtsam::NonlinearFactorGraph& graph, gtsam::Values& values) const;

  /**
   * @brief
   *
   * @param pts3d_target
   * @param graph
   */
  void addLandmarkPriors(const std::vector<gtsam::Point3>& pts3d_target, gtsam::NonlinearFactorGraph& graph) const;

  /**
   * @brief
   *
   * @param camera_index
   * @param camera
   * @param num_camera_update
   * @param measurements
   * @param graph
   */
  void addLandmarkFactors(const size_t camera_index, const std::shared_ptr<gtcal::Camera>& camera,
                          const size_t num_camera_update, const std::vector<Measurement>& measurements,
                          gtsam::NonlinearFactorGraph& graph) const;

  /**
   * @brief
   *
   * @param camera_index
   * @param pose_target_cam
   * @param graph
   */
  void addPosePrior(const size_t camera_index, const gtsam::Pose3& pose_target_cam,
                    gtsam::NonlinearFactorGraph& graph) const;

  /**
   * @brief
   *
   * @return const std::vector<gtsam::Point3>&
   */
  const std::vector<gtsam::Point3>& targetPoints() const { return pts3d_target_; }

private:
  const std::vector<gtsam::Point3> pts3d_target_;
  const Options options_;
};

}  // namespace gtcal
