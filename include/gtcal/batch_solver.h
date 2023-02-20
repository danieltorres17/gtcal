#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3Fisheye.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace gtcal {
class BatchSolver {
public:
  struct InitParameters {
    size_t num_cameras = 0;
    gtsam::ISAM2Params parameters;
  };

public:
  BatchSolver(const InitParameters& parameters);

  size_t GetNumCameras() const { return num_cameras_; }

private:
  const size_t num_cameras_;

  gtsam::ISAM2 isam_;
  gtsam::NonlinearFactorGraph graph_;
};
}  // namespace gtcal
