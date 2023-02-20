#include "gtcal/batch_solver.h"

namespace gtcal {

BatchSolver::BatchSolver(const InitParameters& parameters)
  : num_cameras_(parameters.num_cameras), isam_(gtsam::ISAM2(parameters.parameters)) {}

}  // namespace gtcal
