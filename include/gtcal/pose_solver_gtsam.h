#pragma once

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Cal3Fisheye.h>

#include "gtcal/camera.h"

namespace gtcal {

class Measurement;

class PoseSolverGtsam {
public:
  // Contains the noise models used for the different factors in the factor graph.
  struct Options {
    // Default noise model for initial camera pose prior.
    gtsam::noiseModel::Diagonal::shared_ptr pose_prior_noise_model = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << gtsam::Vector3::Constant(0.75), gtsam::Vector3{0.5, 0.35, 0.35}).finished());

    // Default noise model for the landmark priors.
    gtsam::noiseModel::Isotropic::shared_ptr landmark_prior_noise_model =
        gtsam::noiseModel::Isotropic::Sigma(3, 1e-8);

    // Default noise model for the pixel measurements.
    gtsam::noiseModel::Isotropic::shared_ptr pixel_meas_noise_model =
        gtsam::noiseModel::Isotropic::Sigma(2, 1.0);
  };

public:
  PoseSolverGtsam(const Options& options);

  bool solve(const std::vector<Measurement>& measurements, const std::vector<gtsam::Point3>& pts3d_target,
             const std::shared_ptr<const Camera>& camera, gtsam::Pose3& pose_initial_target_cam) const;

private:
  const Options options_;
};

}  // namespace gtcal