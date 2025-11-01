#include <iostream>

#include "gtcal/calibration_target.hpp"
#include "gtcal/calibration_rig.hpp"
#include "gtcal/random_pose_generator.hpp"

#include <opencv2/opencv.hpp>

int main() {
  const int image_width = 1280;
  const int image_height = 720;

  // Optional seed from argv[1] for reproducible runs.
  unsigned int seed = std::random_device{}();

  const gtsam::Cal3_S2 K(500, 500, 0, image_width / 2., image_height / 2.);
  std::shared_ptr<gtcal::Camera> camera = std::make_shared<gtcal::Camera>();
  camera->setCameraModel<gtsam::Cal3_S2>(image_width, image_height, K);
  // const gtsam::Cal3Fisheye K(750., 750., 0, image_width / 2., image_height / 2., 0.1, -0.12, 0.001, -0.001);
  // std::shared_ptr<gtcal::Camera> camera = std::make_shared<gtcal::Camera>();
  // camera->setCameraModel<gtsam::Cal3Fisheye>(image_width, image_height, K);

  // Target grid point parameters.
  const double grid_spacing = 0.2;
  const size_t num_rows = 10;
  const size_t num_cols = 13;
  const size_t num_target_pts = num_rows * num_cols;
  std::shared_ptr<gtcal::CalibrationTarget> target =
      std::make_shared<gtcal::CalibrationTarget>(grid_spacing, num_cols, num_rows);
  const double target_max_x = num_cols * grid_spacing;
  const double target_max_y = num_rows * grid_spacing;

  // Random pose generator.
  gtcal::RandomPoseGenerator::Config pose_gen_config;
  const double xy_margin = 1.0;
  pose_gen_config.x_min = -xy_margin;
  pose_gen_config.x_max = target_max_x + xy_margin;
  pose_gen_config.y_min = -xy_margin;
  pose_gen_config.y_max = target_max_y + xy_margin;
  pose_gen_config.z_min = 0.5;
  pose_gen_config.z_max = 3.5;
  gtcal::RandomPoseGenerator pose_gen(target, seed, pose_gen_config);

  // Number of poses to sample.
  const size_t num_poses = 25;
  const std::vector<gtsam::Pose3> sampled_poses = pose_gen.samplePoses(num_poses);
  for (size_t ii = 0; ii < sampled_poses.size(); ii++) {
    std::cout << "Sampled Pose " << ii << ":\n" << sampled_poses.at(ii) << "\n";

    // Generate projections at pose.
    const gtsam::Pose3& pose_target_cam_ii = sampled_poses.at(ii);
    camera->setCameraPose(pose_target_cam_ii);

    std::vector<gtsam::Point2> projected_pts2d;
    for (size_t jj = 0; jj < target->pointsTarget().size(); jj++) {
      const gtsam::Point3& pt3d = target->pointsTarget().at(jj);
      const auto [pt2d, safe] = camera->projectSafe(pt3d);
      if (!safe) {
        continue;
      }
      if (pt2d.x() < 0 || pt2d.x() >= image_width || pt2d.y() < 0 || pt2d.y() >= image_height) {
        continue;
      }
      projected_pts2d.push_back(pt2d);
    }

    // Visualize projected points.
    cv::Mat img = cv::Mat::zeros(image_height, image_width, CV_8UC3);
    for (const auto& pt2d : projected_pts2d) {
      cv::circle(img, cv::Point2f(static_cast<float>(pt2d.x()), static_cast<float>(pt2d.y())), 5,
                 cv::Scalar(0, 255, 0), -1);
    }
    cv::imshow("Projected Points Pose", img);
    cv::waitKey(0);
  }

  return 0;
}