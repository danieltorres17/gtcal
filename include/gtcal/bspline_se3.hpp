#pragma once

#include <gtsam/geometry/Pose3.h>

namespace gtcal {

class BSplineSE3 {
public:
  struct PoseStamped {
    gtsam::Pose3 pose;
    double time;
    PoseStamped(const gtsam::Pose3& pose, double time) : pose(pose), time(time) {}
  };

  BSplineSE3();
  void feedTrajectory(const std::vector<PoseStamped>& trajectory);
  bool timestampsAreAscending(const std::vector<PoseStamped>& trajectory) const;
  double avgTimestep(const std::vector<PoseStamped>& trajectory) const;

private:

};

}