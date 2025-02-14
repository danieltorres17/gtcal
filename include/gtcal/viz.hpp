#pragma once

#include <pangolin/pangolin.h>
#include <pangolin/scene/axis.h>
#include <gtsam/geometry/Pose3.h>

#include <mutex>
#include <condition_variable>

namespace gtcal {

class Visualizer {
public:
  Visualizer();
  void run();
  void update(const std::vector<gtsam::Point3>& points, const std::vector<gtsam::Pose3>& poses);

private:
  void drawCamera(const gtsam::Pose3& pose);
  pangolin::OpenGlMatrix pose3ToOpenGlMatrix(const gtsam::Pose3& pose) const;

private:
  pangolin::OpenGlRenderState s_cam_;
  pangolin::View* d_cam_ = nullptr;
  pangolin::Renderable tree_;

  std::vector<gtsam::Point3> points_;
  std::vector<gtsam::Pose3> poses_;
  std::mutex mutex_;
  std::condition_variable cv_;
  bool should_update_ = false;
};

}