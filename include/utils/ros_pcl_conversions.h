/* Lightweight conversions between sensor_msgs::PointCloud2 and PCL types.
 * This avoids the dependency on pcl_conversions for to/from ROS msg helpers.
 */
#pragma once

#include <algorithm>
#include <cstdint>
#include <pcl/PCLPointCloud2.h>
#include <pcl/PCLPointField.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

namespace ros_pcl
{
inline pcl::PCLPointField toPCLField(const sensor_msgs::msg::PointField &field)
{
  pcl::PCLPointField pcl_field;
  pcl_field.name = field.name;
  pcl_field.offset = field.offset;
  pcl_field.datatype = field.datatype;
  pcl_field.count = field.count;
  return pcl_field;
}

inline sensor_msgs::msg::PointField toROSField(const pcl::PCLPointField &field)
{
  sensor_msgs::msg::PointField ros_field;
  ros_field.name = field.name;
  ros_field.offset = field.offset;
  ros_field.datatype = field.datatype;
  ros_field.count = field.count;
  return ros_field;
}

inline pcl::PCLHeader toPCLHeader(const std_msgs::msg::Header &header)
{
  pcl::PCLHeader pcl_header;
  pcl_header.seq = 0;
  pcl_header.stamp =
      static_cast<uint64_t>(header.stamp.sec) * 1000000000ull + static_cast<uint64_t>(header.stamp.nanosec);
  pcl_header.frame_id = header.frame_id;
  return pcl_header;
}

inline std_msgs::msg::Header toROSHeader(const pcl::PCLHeader &header)
{
  std_msgs::msg::Header ros_header;
  ros_header.stamp.sec = static_cast<int32_t>(header.stamp / 1000000000ull);
  ros_header.stamp.nanosec = static_cast<uint32_t>(header.stamp % 1000000000ull);
  ros_header.frame_id = header.frame_id;
  return ros_header;
}

inline void toPCL(const sensor_msgs::msg::PointCloud2 &msg, pcl::PCLPointCloud2 &pcl_pc2)
{
  pcl_pc2.header = toPCLHeader(msg.header);
  pcl_pc2.height = msg.height;
  pcl_pc2.width = msg.width;
  pcl_pc2.fields.resize(msg.fields.size());
  std::transform(msg.fields.begin(), msg.fields.end(), pcl_pc2.fields.begin(), toPCLField);
  pcl_pc2.is_bigendian = msg.is_bigendian;
  pcl_pc2.point_step = msg.point_step;
  pcl_pc2.row_step = msg.row_step;
  pcl_pc2.data = msg.data;
  pcl_pc2.is_dense = msg.is_dense;
}

inline void toROS(const pcl::PCLPointCloud2 &pcl_pc2, sensor_msgs::msg::PointCloud2 &msg)
{
  msg.header = toROSHeader(pcl_pc2.header);
  msg.height = pcl_pc2.height;
  msg.width = pcl_pc2.width;
  msg.fields.resize(pcl_pc2.fields.size());
  std::transform(pcl_pc2.fields.begin(), pcl_pc2.fields.end(), msg.fields.begin(), toROSField);
  msg.is_bigendian = pcl_pc2.is_bigendian;
  msg.point_step = pcl_pc2.point_step;
  msg.row_step = pcl_pc2.row_step;
  msg.data = pcl_pc2.data;
  msg.is_dense = pcl_pc2.is_dense;
}

template <typename PointT>
inline void fromROSMsg(const sensor_msgs::msg::PointCloud2 &msg, pcl::PointCloud<PointT> &cloud)
{
  pcl::PCLPointCloud2 pcl_pc2;
  toPCL(msg, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, cloud);
}

template <typename PointT>
inline void toROSMsg(const pcl::PointCloud<PointT> &cloud, sensor_msgs::msg::PointCloud2 &msg)
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::toPCLPointCloud2(cloud, pcl_pc2);
  toROS(pcl_pc2, msg);
}
}  // namespace ros_pcl
