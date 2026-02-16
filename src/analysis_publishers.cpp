#include "analysis_publishers.h"

#include <algorithm>
#include <array>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include "LIVMapper.h"
#include "utils/ros_pcl_conversions.h"

namespace fast_livo::analysis {

void publishVioEsikfIterations(const AnalysisPublishers &publishers, const std::vector<int> &iterations)
{
  if (!publishers.vio_esikf_iterations) return;

  std_msgs::msg::Int32MultiArray msg;
  msg.data.assign(iterations.begin(), iterations.end());
  publishers.vio_esikf_iterations(msg);
}

void publishVioFeatureCounts(const AnalysisPublishers &publishers, const std::vector<int> &features)
{
  if (!publishers.vio_feature_counts) return;

  std_msgs::msg::Int32MultiArray msg;
  msg.data.assign(features.begin(), features.end());
  publishers.vio_feature_counts(msg);
}

void publishVioInlierCount(const AnalysisPublishers &publishers, int count)
{
  if (!publishers.vio_inlier_count) return;

  std_msgs::msg::Int32 msg;
  msg.data = count;
  publishers.vio_inlier_count(msg);
}

void publishVioOutlierCount(const AnalysisPublishers &publishers, int count)
{
  if (!publishers.vio_outlier_count) return;

  std_msgs::msg::Int32 msg;
  msg.data = count;
  publishers.vio_outlier_count(msg);
}

void publishVioRaycastCount(const AnalysisPublishers &publishers, int count)
{
  if (!publishers.vio_raycast_count) return;

  std_msgs::msg::Int32 msg;
  msg.data = count;
  publishers.vio_raycast_count(msg);
}

void publishVioAddedVisualPoints(const AnalysisPublishers &publishers, int count)
{
  if (!publishers.vio_added_visual_points) return;

  std_msgs::msg::Int32 msg;
  msg.data = count;
  publishers.vio_added_visual_points(msg);
}

void publishVioCommonTrackedPoints(const AnalysisPublishers &publishers, int count)
{
  if (!publishers.vio_common_tracked_points) return;

  std_msgs::msg::Int32 msg;
  msg.data = count;
  publishers.vio_common_tracked_points(msg);
}

void publishVioDepthDiscontinuityRejects(const AnalysisPublishers &publishers, int count)
{
  if (!publishers.vio_depth_discontinuity_rejects) return;

  std_msgs::msg::Int32 msg;
  msg.data = count;
  publishers.vio_depth_discontinuity_rejects(msg);
}

void publishVioShiTomasiStats(const AnalysisPublishers &publishers, float avg, float min, float max)
{
  if (!publishers.vio_shitomasi_stats) return;

  std_msgs::msg::Float32MultiArray msg;
  msg.data.resize(3);
  msg.data[0] = avg;
  msg.data[1] = min;
  msg.data[2] = max;
  publishers.vio_shitomasi_stats(msg);
}

void publishDiscardedVisualGeneration(const AnalysisPublishers &publishers, int null_normal, int out_of_frame, float proportion)
{
  if (!publishers.vio_discarded_visual_generation) return;

  publishers.vio_discarded_visual_generation(null_normal, out_of_frame, proportion);
}

void publishVioSparseDepthMap(const AnalysisPublishers &publishers, const cv::Mat &img, const rclcpp::Time &stamp)
{
  if (!publishers.vio_sparse_depth_map) return;
  if (img.empty()) return;

  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = stamp;
  out_msg.header.frame_id = "camera";
  out_msg.encoding = sensor_msgs::image_encodings::BGR8;
  out_msg.image = img;
  publishers.vio_sparse_depth_map(*out_msg.toImageMsg());
}

void publishVioReconstructedView(const AnalysisPublishers &publishers, int level, const cv::Mat &img, const rclcpp::Time &stamp)
{
  if (!publishers.vio_reconstructed_view) return;
  if (img.empty()) return;

  cv::Mat gray;
  if (img.type() == CV_8UC1) {
    gray = img;
  } else {
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
  }

  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = stamp;
  out_msg.header.frame_id = "camera";
  out_msg.encoding = sensor_msgs::image_encodings::MONO8;
  out_msg.image = gray;
  publishers.vio_reconstructed_view(level, *out_msg.toImageMsg());
}

void publishVioInliersOutliersClouds(const AnalysisPublishers &publishers, const std::vector<V3D> &inlier_points,
                                     const std::vector<V3D> &outlier_points, const rclcpp::Time &stamp)
{
  if (!publishers.vio_inlier_points && !publishers.vio_outlier_points) return;
  if (inlier_points.empty() && outlier_points.empty()) return;

  if (publishers.vio_inlier_points && !inlier_points.empty())
  {
    PointCloudXYZI::Ptr inliers(new PointCloudXYZI());
    inliers->reserve(inlier_points.size());
    for (const auto &pos : inlier_points) {
      PointType pt;
      pt.x = pos[0];
      pt.y = pos[1];
      pt.z = pos[2];
      pt.intensity = 0.0f;
      inliers->push_back(pt);
    }
    sensor_msgs::msg::PointCloud2 msg;
    ros_pcl::toROSMsg(*inliers, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = "init_pose";
    publishers.vio_inlier_points(msg);
  }
  if (publishers.vio_outlier_points && !outlier_points.empty())
  {
    PointCloudXYZI::Ptr outliers(new PointCloudXYZI());
    outliers->reserve(outlier_points.size());
    for (const auto &pos : outlier_points) {
      PointType pt;
      pt.x = pos[0];
      pt.y = pos[1];
      pt.z = pos[2];
      pt.intensity = 0.0f;
      outliers->push_back(pt);
    }
    sensor_msgs::msg::PointCloud2 msg;
    ros_pcl::toROSMsg(*outliers, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = "init_pose";
    publishers.vio_outlier_points(msg);
  }
}

void publishVioOptimizationPointCount(const AnalysisPublishers &publishers, int count)
{
  if (!publishers.vio_optimization_point_count) return;

  std_msgs::msg::Int32 msg;
  msg.data = count;
  publishers.vio_optimization_point_count(msg);
}

void publishVioOptimizationPoints(const AnalysisPublishers &publishers, const std::vector<V3D> &points,
                                  const rclcpp::Time &stamp)
{
  if (!publishers.vio_optimization_points) return;
  if (points.empty()) return;

  PointCloudXYZI::Ptr cloud(new PointCloudXYZI());
  cloud->reserve(points.size());
  for (const auto &pos : points) {
    PointType pt;
    pt.x = pos[0];
    pt.y = pos[1];
    pt.z = pos[2];
    pt.intensity = 0.0f;
    cloud->push_back(pt);
  }
  sensor_msgs::msg::PointCloud2 msg;
  ros_pcl::toROSMsg(*cloud, msg);
  msg.header.stamp = stamp;
  msg.header.frame_id = "init_pose";
  publishers.vio_optimization_points(msg);
}

void publishVioConvergedPointCount(const AnalysisPublishers &publishers, int count)
{
  if (!publishers.vio_converged_point_count) return;

  std_msgs::msg::Int32 msg;
  msg.data = count;
  publishers.vio_converged_point_count(msg);
}

void publishVioConvergedPoints(const AnalysisPublishers &publishers, const std::vector<V3D> &points,
                               const rclcpp::Time &stamp)
{
  if (!publishers.vio_converged_points) return;
  if (points.empty()) return;

  PointCloudXYZI::Ptr cloud(new PointCloudXYZI());
  cloud->reserve(points.size());
  for (const auto &pos : points) {
    PointType pt;
    pt.x = pos[0];
    pt.y = pos[1];
    pt.z = pos[2];
    pt.intensity = 0.0f;
    cloud->push_back(pt);
  }
  sensor_msgs::msg::PointCloud2 msg;
  ros_pcl::toROSMsg(*cloud, msg);
  msg.header.stamp = stamp;
  msg.header.frame_id = "init_pose";
  publishers.vio_converged_points(msg);
}

void publishVioPointCandidates(const AnalysisPublishers &publishers, const std::vector<pointWithVar> &points_world,
                               const std::vector<V3D> &points_cam, const rclcpp::Time &stamp,
                               const std::string &world_frame_id, const std::string &cam_frame_id)
{
  if (!publishers.vio_point_candidates) return;

  sensor_msgs::msg::PointCloud2 msg_world;
  PointCloudXYZI::Ptr cloud_world(new PointCloudXYZI());
  cloud_world->reserve(points_world.size());
  for (const auto &pwv : points_world) {
    PointType pt;
    pt.x = pwv.point_w.x();
    pt.y = pwv.point_w.y();
    pt.z = pwv.point_w.z();
    pt.intensity = 1.0f;
    cloud_world->push_back(pt);
  }
  ros_pcl::toROSMsg(*cloud_world, msg_world);
  msg_world.header.stamp = stamp;
  msg_world.header.frame_id = world_frame_id;

  sensor_msgs::msg::PointCloud2 msg_cam;
  PointCloudXYZI::Ptr cloud_cam(new PointCloudXYZI());
  cloud_cam->reserve(points_cam.size());
  for (const auto &pos : points_cam) {
    PointType pt;
    pt.x = pos[0];
    pt.y = pos[1];
    pt.z = pos[2];
    pt.intensity = 0.0f;
    cloud_cam->push_back(pt);
  }
  ros_pcl::toROSMsg(*cloud_cam, msg_cam);
  msg_cam.header.stamp = stamp;
  msg_cam.header.frame_id = cam_frame_id;

  publishers.vio_point_candidates(msg_world, msg_cam);
}

void publishLioEsikfIterations(const AnalysisPublishers &publishers, int iterations)
{
  if (!publishers.lio_esikf_iterations) return;

  std_msgs::msg::Int32 msg;
  msg.data = iterations;
  publishers.lio_esikf_iterations(msg);
}

void publishEkfBiases(const AnalysisPublishers &publishers, const V3D &bias_g, const V3D &bias_a)
{
  if (!publishers.ekf_biases) return;

  std_msgs::msg::Float32MultiArray msg;
  msg.data.resize(6);
  msg.data[0] = static_cast<float>(bias_a.x());
  msg.data[1] = static_cast<float>(bias_a.y());
  msg.data[2] = static_cast<float>(bias_a.z());
  msg.data[3] = static_cast<float>(bias_g.x());
  msg.data[4] = static_cast<float>(bias_g.y());
  msg.data[5] = static_cast<float>(bias_g.z());

  publishers.ekf_biases(msg);
}

void publishCameraFov(const AnalysisPublishers &publishers, const VIOManager *vio_manager, const rclcpp::Time &stamp, const tf2::Transform &TF_lidar_cam, const tf2::Transform &TF_body_cam)
{
  if (!vio_manager || !vio_manager->cam || !publishers.camera_fov_markers) return;

  const double fx = vio_manager->cam->fx();
  const double fy = vio_manager->cam->fy();
  const double cx = vio_manager->cam->cx();
  const double cy = vio_manager->cam->cy();
  const int width = vio_manager->cam->width();
  const int height = vio_manager->cam->height();
  if (fx <= 0.0 || fy <= 0.0 || width <= 0 || height <= 0) return;

  const std::array<double, 3> depths = {0.5, 1.0, 1.5};
  visualization_msgs::msg::MarkerArray markers_cam;

  for (size_t i = 0; i < depths.size(); ++i)
  {
    const double d = depths[i];
    auto corner_cam = [&](double u, double v) {
      geometry_msgs::msg::Point p;
      p.x = (u - cx) / fx * d;
      p.y = (v - cy) / fy * d;
      p.z = d;
      return p;
    };

    geometry_msgs::msg::Point p0 = corner_cam(0.0, 0.0);
    geometry_msgs::msg::Point p1 = corner_cam(width - 1.0, 0.0);
    geometry_msgs::msg::Point p2 = corner_cam(width - 1.0, height - 1.0);
    geometry_msgs::msg::Point p3 = corner_cam(0.0, height - 1.0);

    visualization_msgs::msg::Marker rect;
    rect.header.frame_id = "camera";
    rect.header.stamp = stamp;
    rect.ns = "camera_fov_rect";
    rect.id = static_cast<int>(i);
    rect.type = visualization_msgs::msg::Marker::LINE_LIST;
    rect.action = visualization_msgs::msg::Marker::ADD;
    rect.pose.orientation.w = 1.0;
    rect.scale.x = 0.01;
    rect.color.a = 0.8f;
    rect.color.r = 0.2f;
    rect.color.g = 0.6f;
    rect.color.b = 1.0f;
    rect.points = {p0, p1, p1, p2, p2, p3, p3, p0};
    markers_cam.markers.push_back(rect);

    visualization_msgs::msg::Marker corners;
    corners.header.frame_id = "camera";
    corners.header.stamp = stamp;
    corners.ns = "camera_fov_corners";
    corners.id = static_cast<int>(i);
    corners.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    corners.action = visualization_msgs::msg::Marker::ADD;
    corners.pose.orientation.w = 1.0;
    corners.scale.x = 0.03;
    corners.scale.y = 0.03;
    corners.scale.z = 0.03;
    corners.color.a = 0.9f;
    corners.color.r = 1.0f;
    corners.color.g = 0.7f;
    corners.color.b = 0.2f;
    corners.points = {p0, p1, p2, p3};
    markers_cam.markers.push_back(corners);
  }

  // TODO: Duplicate the message BUT, for each point, 
  // - project it in the LiDAR frame "os_sensor"
  // - project it in the body frame "body"
  // Then send all 3 visualization messages

  // LiDAR frame
  visualization_msgs::msg::MarkerArray marker_lidar = markers_cam;
  for (auto &marker : marker_lidar.markers)
  {
    marker.header.frame_id = "os_sensor";

    for (auto &points: marker.points)
    {
      tf2::Vector3 p_cam(points.x, points.y, points.z);
      tf2::Vector3 p_lidar = TF_lidar_cam * p_cam;
      points.x = p_lidar.x();
      points.y = p_lidar.y();
      points.z = p_lidar.z();
    }

  }

  visualization_msgs::msg::MarkerArray marker_body = markers_cam;
  for (auto &marker : marker_body.markers)
  {
    marker.header.frame_id = "body";

    for (auto &points: marker.points)
    {
      tf2::Vector3 p_cam(points.x, points.y, points.z);
      tf2::Vector3 p_body = TF_body_cam * p_cam;
      points.x = p_body.x();
      points.y = p_body.y();
      points.z = p_body.z();
    }

  }

  if (publishers.camera_fov_markers) publishers.camera_fov_markers(markers_cam, marker_lidar, marker_body);
}

void publishProjectedLidarCamera(const AnalysisPublishers &publishers, const VIOManager *vio_manager,
                                 const PointCloudXYZI::Ptr &feats_undistort, const tf2::Transform &TF_cam_lidar,
                                 const tf2::Transform &TF_lidar_body, const rclcpp::Time &stamp)
{
  if (!publishers.projected_lidar_camera) return;
  if (!vio_manager || !feats_undistort) return;

  const int rows = vio_manager->img_rgb.rows;
  const int cols = vio_manager->img_rgb.cols;
  cv::Mat projected = cv::Mat::zeros(rows, cols, CV_8UC3);

  if(feats_undistort->points.empty())
  {
    // Write "No LiDAR points" on the image
    cv::putText(projected, "No LiDAR points", cv::Point(cols / 4, rows / 2),
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
  }

  // 1) Choose the LiDAR point source (e.g., feats_undistort or pcl_w_wait_pub)
  for (const auto &pt : feats_undistort->points) {
    tf2::Vector3 P_lidar(pt.x, pt.y, pt.z);
    tf2::Vector3 P_camera = TF_cam_lidar * P_lidar;

    Vector2d px = vio_manager->cam->world2cam(Eigen::Vector3d(P_camera.x(), P_camera.y(), P_camera.z()));
    if (px.x() < 0 || px.y() < 0 || px.x() >= projected.cols || px.y() >= projected.rows || P_camera.z() <= 0) {
      continue;
    }
    const float intensity = pt.intensity;
    const int u = static_cast<int>(px.x());
    const int v = static_cast<int>(px.y());
    const uint8_t gray_value = static_cast<uint8_t>(std::min(std::max(int(intensity), 0), 255));
    cv::circle(projected, cv::Point(u, v), 2, cv::Scalar(gray_value, gray_value, gray_value), -1);
  }

  // Fake corners defined in the body frame on the YZ plane at x = -1m.
  const double x_center = -1.0;
  const double half_side = 0.5;
  const double segment_len = 0.1;
  const int segment_points = 5;
  const double segment_step = segment_len / static_cast<double>(segment_points);
  // Colors: top-left (red), top-right (green), bottom-left (blue), bottom-right (yellow).
  const std::array<cv::Scalar, 4> corner_colors = {
      cv::Scalar(0, 0, 255),
      cv::Scalar(0, 255, 0),
      cv::Scalar(255, 0, 0),
      cv::Scalar(0, 255, 255)};

  struct CornerSpec {
    tf2::Vector3 corner;
    double y_dir;
    double z_dir;
    cv::Scalar color;
  };
  const std::array<CornerSpec, 4> corners = {
      CornerSpec{tf2::Vector3(x_center, half_side, -half_side), 1.0, -1.0, corner_colors[0]},  // top-left
      CornerSpec{tf2::Vector3(x_center, -half_side, -half_side), -1.0, -1.0, corner_colors[1]}, // top-right
      CornerSpec{tf2::Vector3(x_center, half_side, half_side), 1.0, 1.0, corner_colors[2]},     // bottom-left
      CornerSpec{tf2::Vector3(x_center, -half_side, half_side), -1.0, 1.0, corner_colors[3]}     // bottom-right
  };

  for (const auto &corner_spec : corners) {
    auto draw_point = [&](const tf2::Vector3 &pb_body) {
      const auto pb_lidar = TF_lidar_body * pb_body;
      const auto pb_cam = TF_cam_lidar * pb_lidar;
      const auto pb_cam_eigen = Eigen::Vector3d(pb_cam.x(), pb_cam.y(), pb_cam.z());

      Vector2d px = vio_manager->cam->world2cam(pb_cam_eigen);
      if (px.x() < 0 || px.y() < 0 || px.x() >= projected.cols || px.y() >= projected.rows) {
        return;
      }
      const int u = static_cast<int>(px.x());
      const int v = static_cast<int>(px.y());
      cv::circle(projected, cv::Point(u, v), 2, corner_spec.color, -1);
    };

    draw_point(corner_spec.corner);
    for (int i = 1; i <= segment_points; ++i) {
      draw_point(corner_spec.corner + tf2::Vector3(0.0, corner_spec.y_dir * segment_step * i, 0.0));
      draw_point(corner_spec.corner + tf2::Vector3(0.0, 0.0, corner_spec.z_dir * segment_step * i));
    }
  }

  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = stamp;
  out_msg.header.frame_id = "camera";
  out_msg.encoding = sensor_msgs::image_encodings::BGR8;
  out_msg.image = projected;

  publishers.projected_lidar_camera(*out_msg.toImageMsg());
}

}  // namespace fast_livo::analysis
