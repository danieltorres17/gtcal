#include "gtcal/viz.hpp"

namespace gtcal {

Visualizer::Visualizer() {
  pangolin::CreateWindowAndBind("Visualizer", 1024, 768);
  glEnable(GL_DEPTH_TEST);

  s_cam_ = pangolin::OpenGlRenderState(pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 384, 0.1, 10),
                                       pangolin::ModelViewLookAt(0, -5, -10, 0, 0, 0, 0, -1, 0));

  d_cam_ = &pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
                .SetHandler(new pangolin::Handler3D(s_cam_));
}

void Visualizer::run() {
  while (!pangolin::ShouldQuit()) {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [this] { return should_update_; });  // Wait for new data.

    // Clear buffer and activate camera.
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam_->Activate(s_cam_);

    // Draw points.
    glPointSize(3.0f);
    glBegin(GL_POINTS);
    glColor3f(1.0, 0.0, 0.0);
    for (const auto& point : points_) {
      glVertex3d(point.x(), point.y(), point.z());
    }
    glEnd();

    // Draw camera poses.
    glLineWidth(2.0f);
    for (const auto& pose : poses_) {
      drawCamera(pose);
    }
  }
}

void Visualizer::update(const std::vector<gtsam::Point3>& points, const std::vector<gtsam::Pose3>& poses) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    points_ = points;
    poses_ = poses;
    should_update_ = true;
  }

  cv_.notify_one();
}

void Visualizer::drawCamera(const gtsam::Pose3& pose) {
  auto cam_axis = std::make_shared<pangolin::Axis>();
  cam_axis->axis_length = 0.35;
  cam_axis->T_pc = pose3ToOpenGlMatrix(pose);
  tree_.Add(cam_axis);
}

pangolin::OpenGlMatrix Visualizer::pose3ToOpenGlMatrix(const gtsam::Pose3& pose) const {
  // Get the rotation matrix (3x3) and translation vector from Pose3
  gtsam::Matrix3 rotationMatrix = pose.rotation().matrix();
  gtsam::Point3 translation = pose.translation();

  pangolin::OpenGlMatrix glMatrix;

  // Pangolin stores the matrix in column-major order.
  glMatrix.m[0] = rotationMatrix(0, 0);  // First column.
  glMatrix.m[1] = rotationMatrix(1, 0);
  glMatrix.m[2] = rotationMatrix(2, 0);
  glMatrix.m[3] = 0.0;

  glMatrix.m[4] = rotationMatrix(0, 1);  // Second column.
  glMatrix.m[5] = rotationMatrix(1, 1);
  glMatrix.m[6] = rotationMatrix(2, 1);
  glMatrix.m[7] = 0.0;

  glMatrix.m[8] = rotationMatrix(0, 2);  // Third column.
  glMatrix.m[9] = rotationMatrix(1, 2);
  glMatrix.m[10] = rotationMatrix(2, 2);
  glMatrix.m[11] = 0.0;

  glMatrix.m[12] = translation.x();  // Fourth column (translation).
  glMatrix.m[13] = translation.y();
  glMatrix.m[14] = translation.z();
  glMatrix.m[15] = 1.0;

  return glMatrix;
}

}  // namespace gtcal