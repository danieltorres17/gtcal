#pragma once

#include <gtsam/geometry/Pose3.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std::chrono_literals;

namespace gtcal {
namespace viz {

void VisualizeSetup(const std::vector<gtsam::Pose3>& poses_target_cam,
                    const gtsam::Point3Vector& pts3d_target) {
  // Viewer.
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);

  // Cloud.
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // Helper to plot points.
  auto Add3dPts = [&viewer, &cloud](const gtsam::Point3Vector& pts3d) -> void {
    // Add points to cloud.
    for (const auto& pt3d : pts3d) {
      pcl::PointXYZ pt_temp;
      pt_temp.x = pt3d.x();
      pt_temp.y = pt3d.y();
      pt_temp.z = pt3d.z();
      cloud->points.push_back(pt_temp);
    }
    viewer->addPointCloud(cloud, "PointCloud");
  };

  // Helper to plot poses.
  auto AddPose3s = [&viewer](const std::vector<gtsam::Pose3>& poses) -> void {
    const float axis_scale = 1.0;
    // Add camera poses to visualization.
    for (const auto& pose : poses) {
      // Convert pose to Eigen Matrix.
      const Eigen::Matrix4f pose_mat = pose.matrix().cast<float>();
      const Eigen::Affine3f pose_affine(pose_mat);
      viewer->addCoordinateSystem(axis_scale, pose_affine);
    }
  };

  // Add poses to visualization.
  AddPose3s(poses_target_cam);

  // Add target points to visualization.
  Add3dPts(pts3d_target);
  viewer->addCoordinateSystem(1.0, Eigen::Affine3f::Identity());

  // Increase point size to 3.
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "PointCloud");

  while (!viewer->wasStopped()) {
    viewer->spin();
    std::this_thread::sleep_for(100ms);
  }
}

}  // namespace viz
}  // namespace gtcal