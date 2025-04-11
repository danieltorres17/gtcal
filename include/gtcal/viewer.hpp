#pragma once

#include <pangolin/pangolin.h>
#include <pangolin/scene/axis.h>
#include <gtsam/geometry/Pose3.h>

#include <mutex>
#include <optional>

namespace gtcal {

class Viewer {
public:
  using Ptr = std::shared_ptr<Viewer>;

  Viewer();
  void close();
  void update(const std::vector<gtsam::Point3>& points, const std::vector<gtsam::Pose3>& poses);
  void update(const std::vector<gtsam::Point3>& points);
  void update(const std::vector<gtsam::Pose3>& poses);

private:
  void threadLoop();
  void drawPoints();
  void drawPoses();
  pangolin::OpenGlMatrix pose3ToOpenGlMatrix(const gtsam::Pose3& pose) const;

private:
  std::string window_name_ = "GTCAL Viewer";
  std::thread viewer_thread_;
  std::mutex viewer_data_mutex_;
  bool viewer_is_running_ = false;
  pangolin::Renderable tree_;

  std::optional<std::vector<gtsam::Point3>> points_;
  std::optional<std::vector<gtsam::Pose3>> poses_;
};

}  // namespace gtcal