#include "gtcal/viewer.hpp"

namespace gtcal {

Viewer::Viewer() {
  // Initialize the viewer thread.
  viewer_thread_ = std::thread(std::bind(&Viewer::threadLoop, this));
  viewer_is_running_ = true;
}

void Viewer::close() {
  viewer_is_running_ = false;
  if (viewer_thread_.joinable()) {
    viewer_thread_.join();
  }
}

void Viewer::threadLoop() {
  pangolin::CreateWindowAndBind(window_name_, 1024, 768);
  glEnable(GL_DEPTH_TEST);

  // pangolin::OpenGlRenderState vis_camera(pangolin::ProjectionMatrix(1024, 768, 400, 400, 512, 384,
  // 0.1, 15.),
  //                                        pangolin::ModelViewLookAt(0, -1.0, -0.25, 0, 0, 0, 0.0, -1.0,
  //                                        0.0));
  pangolin::OpenGlRenderState vis_camera(pangolin::ProjectionMatrix(1024, 768, 400, 400, 512, 384, 0.1, 15.),
                                         pangolin::ModelViewLookAt(0, -1.0, -0.25, 0, 0, 0, pangolin::AxisZ));

  // Add named OpenGL viewport to window and provide 3D handler.
  pangolin::Handler3D handler(vis_camera);
  pangolin::View& vis_display =
      pangolin::CreateDisplay().SetBounds(0.0, 1.0, 0.0, 1.0, -1024.f / 768.f).SetHandler(&handler);
  const float blue[3] = {0, 0, 1};
  const float green[3] = {0, 1, 0};

  while (!pangolin::ShouldQuit()) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    // glClearColor(1.0f, 1.0f, 1.0f, 1.0f); // Enable white background.
    vis_display.Activate(vis_camera);

    {
      std::unique_lock<std::mutex> lock(viewer_data_mutex_);
      if (points_) {
        drawPoints();
      }

      if (poses_) {
        drawPoses();
        tree_.Render();
      }
    }

    pangolin::FinishFrame();
    usleep(5000);
  }

  pangolin::GetBoundWindow()->RemoveCurrent();
}

void Viewer::update(const std::vector<gtsam::Point3>& points, const std::vector<gtsam::Pose3>& poses) {
  std::lock_guard<std::mutex> lock(viewer_data_mutex_);
  points_ = points;
  poses_ = poses;
}

void Viewer::update(const std::vector<gtsam::Point3>& points) {
  std::lock_guard<std::mutex> lock(viewer_data_mutex_);
  points_ = points;
}

void Viewer::update(const std::vector<gtsam::Pose3>& poses) {
  std::lock_guard<std::mutex> lock(viewer_data_mutex_);
  poses_ = poses;
}

void Viewer::drawPoints() {
  if (!points_) {
    return;
  }

  glPointSize(5);
  glBegin(GL_POINTS);
  for (const auto& point : *points_) {
    glColor3f(1.0f, 0.0f, 1.0f);
    glVertex3f(point.x(), point.y(), point.z());
  }
  glEnd();
}

void Viewer::drawPoses() {
  if (!poses_) {
    return;
  }

  for (const auto& pose : *poses_) {
    auto cam_axis = std::make_shared<pangolin::Axis>();
    cam_axis->axis_length = 0.15;
    cam_axis->T_pc = pose3ToOpenGlMatrix(pose);
    tree_.Add(cam_axis);
  }
}

pangolin::OpenGlMatrix Viewer::pose3ToOpenGlMatrix(const gtsam::Pose3& pose) const {
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