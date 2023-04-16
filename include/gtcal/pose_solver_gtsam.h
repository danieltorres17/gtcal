#pragma once

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Cal3Fisheye.h>

namespace gtcal {

class Measurement;

class PoseSolverGtsam {
public:
  // Contains the noise models used for the different factors in the factor graph.
  struct Options {
    // Default noise model for initial camera pose prior.
    gtsam::noiseModel::Diagonal::shared_ptr pose_prior_noise_model = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << gtsam::Vector3::Constant(0.1), gtsam::Vector3::Constant(0.1)).finished());

    // Default noise model for the landmark priors.
    gtsam::noiseModel::Isotropic::shared_ptr landmark_prior_noise_model =
        gtsam::noiseModel::Isotropic::Sigma(3, 1e-7);

    // Default noise model for the pixel measurements.
    gtsam::noiseModel::Isotropic::shared_ptr pixel_meas_noise_model =
        gtsam::noiseModel::Isotropic::Sigma(2, 1.0);
  };

public:
  PoseSolverGtsam(const Options& options);

  gtsam::Pose3 solve(const gtsam::Pose3& pose_initial_target_cam,
                     const std::vector<Measurement>& measurements, const gtsam::Point3Vector& pts3d_target,
                     const gtsam::Cal3Fisheye::shared_ptr& cmod_params) const;

private:
  const Options options_;
};

}  // namespace gtcal