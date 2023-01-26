#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3Fisheye.h>
#include <gtsam/geometry/PinholeCamera.h>

namespace gtcal {
class BatchSolver {
public:
  BatchSolver();

private:
  const size_t num_cameras_;
};
}
