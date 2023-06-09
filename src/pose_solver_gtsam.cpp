#include "gtcal/pose_solver_gtsam.h"
#include "gtcal/utils.h"

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

using gtsam::symbol_shorthand::K;
using gtsam::symbol_shorthand::L;
using gtsam::symbol_shorthand::X;

namespace gtcal {

PoseSolverGtsam::PoseSolverGtsam(const Options& options) : options_(options) {}

gtsam::Pose3 PoseSolverGtsam::solve(const gtsam::Pose3& pose_initial_target_cam,
                                    const std::vector<Measurement>& measurements,
                                    const gtsam::Point3Vector& pts3d_target,
                                    const gtsam::Cal3Fisheye::shared_ptr& cmod_params) const {
  // Create factor graph.
  gtsam::NonlinearFactorGraph graph;

  // Add camera pose prior to graph.
  graph.addPrior(X(0), pose_initial_target_cam, options_.pose_prior_noise_model);

  // Add projection factors to graph.
  for (const auto& meas : measurements) {
    graph.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3Fisheye>>(
        meas.uv, options_.pixel_meas_noise_model, X(0), L(meas.point_id), cmod_params);
  }

  // Add landmark priors to graph.
  for (const auto& meas : measurements) {
    graph.addPrior(L(meas.point_id), pts3d_target.at(meas.point_id), options_.landmark_prior_noise_model);
  }

  // Create initial estimate for landmarks and camera pose.
  gtsam::Values initial_estimate;
  for (const auto& meas : measurements) {
    initial_estimate.insert<gtsam::Point3>(L(meas.point_id), pts3d_target.at(meas.point_id));
  }
  initial_estimate.insert<gtsam::Pose3>(X(0), pose_initial_target_cam);

  // Solve and return the optimized camera pose. TODO: add check in case the optimization fails.
  gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initial_estimate).optimize();

  return result.at<gtsam::Pose3>(X(0));
}

}  // namespace gtcal
