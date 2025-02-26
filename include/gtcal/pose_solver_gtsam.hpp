#pragma once

#include <optional>

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Cal3Fisheye.h>

#include "gtcal/camera.hpp"

namespace gtcal {

class Measurement;

class PoseSolverGtsam {
public:
  // Contains the noise models used for the different factors in the factor graph.
  struct Options {
    Options() {}

    // Default noise model for the pixel measurements.
    gtsam::noiseModel::Isotropic::shared_ptr pixel_meas_noise_model =
        gtsam::noiseModel::Isotropic::Sigma(2, 1.0);
  };

public:
  PoseSolverGtsam(const Options& options = Options{});

  std::optional<gtsam::Pose3> solve(const std::vector<Measurement>& measurements,
                                    const std::vector<gtsam::Point3>& pts3d_target,
                                    const std::shared_ptr<const Camera>& camera,
                                    const gtsam::Pose3& pose_target_cam_initial) const;

private:
  const Options options_;
};

}  // namespace gtcal