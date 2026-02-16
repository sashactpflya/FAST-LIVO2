#pragma once

#include <string>

#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Transform.h>
#include <opencv2/core.hpp>

#include "vio.h"
#include "common_lib.h"

namespace fast_livo {

struct AnalysisPublishers;
class VIOManager;

namespace analysis {

void publishVioAnalysisData(const AnalysisPublishers &publishers, const VioAnalysisData &vio_analysis_data);

void publishVioEsikfIterations(const AnalysisPublishers &publishers, const std::vector<int> &iterations);
void publishVioFeatureCounts(const AnalysisPublishers &publishers, const std::vector<int> &features);
void publishVioInlierCount(const AnalysisPublishers &publishers, int count);
void publishVioOutlierCount(const AnalysisPublishers &publishers, int count);
void publishVioRaycastCount(const AnalysisPublishers &publishers, int count);
void publishVioAddedVisualPoints(const AnalysisPublishers &publishers, int count);
void publishVioCommonTrackedPoints(const AnalysisPublishers &publishers, int count);
void publishVioDepthDiscontinuityRejects(const AnalysisPublishers &publishers, int count);
void publishVioShiTomasiStats(const AnalysisPublishers &publishers, float avg, float min, float max);
void publishDiscardedVisualGeneration(const AnalysisPublishers &publishers, int null_normal, int out_of_frame, float proportion);
void publishVioSparseDepthMap(const AnalysisPublishers &publishers, const cv::Mat &img, const rclcpp::Time &stamp);
void publishVioReconstructedView(const AnalysisPublishers &publishers, int level, const cv::Mat &img, const rclcpp::Time &stamp);
void publishVioInliersOutliersClouds(const AnalysisPublishers &publishers, const std::vector<V3D> &inlier_points,
                                     const std::vector<V3D> &outlier_points, const rclcpp::Time &stamp);
void publishVioOptimizationPointCount(const AnalysisPublishers &publishers, int count);
void publishVioOptimizationPoints(const AnalysisPublishers &publishers, const std::vector<V3D> &points,
                                  const rclcpp::Time &stamp);
void publishVioConvergedPointCount(const AnalysisPublishers &publishers, int count);
void publishVioConvergedPoints(const AnalysisPublishers &publishers, const std::vector<V3D> &points,
                               const rclcpp::Time &stamp);
/**
 * Publish visual map point candidates used in VIO optimization
 * @param publishers Analysis publishers
 * @param points Visual map point candidates in world frame
 */
void publishVioPointCandidates(const AnalysisPublishers &publishers, const std::vector<pointWithVar> &points_world,
                               const std::vector<V3D> &points_cam, const rclcpp::Time &stamp,
                               const std::string &world_frame_id, const std::string &cam_frame_id);
void publishLioEsikfIterations(const AnalysisPublishers &publishers, int iterations);
void publishCameraFov(const AnalysisPublishers &publishers, const VIOManager *vio_manager, const rclcpp::Time &stamp, const tf2::Transform &TF_lidar_cam, const tf2::Transform &TF_body_cam);
void publishProjectedLidarCamera(const AnalysisPublishers &publishers, const VIOManager *vio_manager,
                                 const PointCloudXYZI::Ptr &feats_undistort, const tf2::Transform &TF_cam_lidar,
                                 const tf2::Transform &TF_lidar_body, const rclcpp::Time &stamp);
void publishEkfBiases(const AnalysisPublishers &publishers, const V3D &bias_g, const V3D &bias_a);

}  // namespace analysis
}  // namespace fast_livo
