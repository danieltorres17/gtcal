#include "gtcal/bspline_se3.hpp"

#include <algorithm>

namespace gtcal {

BSplineSE3::BSplineSE3() {}

void BSplineSE3::feedTrajectory(const std::vector<PoseStamped>& trajectory) {
  // First, check if timestamps are increasing.
  const bool timestamps_ascending = timestampsAreAscending(trajectory);
  if (!timestamps_ascending) {
    throw std::runtime_error("Timestamps are not in ascending order.");
  }

  // Get the average time delta between pose timestamps.
  const double dt_s = avgTimestep(trajectory);

  // 
}

bool BSplineSE3::timestampsAreAscending(const std::vector<PoseStamped>& trajectory) const {
  return std::is_sorted(trajectory.begin(), trajectory.end(),
                        [](const PoseStamped& a, const PoseStamped& b) { return a.time < b.time; });
}

double BSplineSE3::avgTimestep(const std::vector<PoseStamped>& trajectory) const {
  double dt_sum = 0.0;
  for (size_t ii = 0; ii < trajectory.size() - 1; ii++) {
    dt_sum += trajectory.at(ii + 1).time - trajectory.at(ii).time;
  }

  double avg_dt_s = dt_sum / (trajectory.size() - 1);
  avg_dt_s = (avg_dt_s < 0.05) ? 0.05 : avg_dt_s;

  return avg_dt_s;
}

}  // namespace gtcal