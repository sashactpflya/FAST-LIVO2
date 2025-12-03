/*
 * Lightweight camera loader moved from vikit_ros to avoid depending on that package.
 */

#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <vikit/abstract_camera.h>
#include <vikit/pinhole_camera.h>
#include <vikit/atan_camera.h>
#include <vikit/omni_camera.h>
#include <vikit/equidistant_camera.h>
#include <vikit/polynomial_camera.h>

namespace fast_livo {
namespace camera_loader {

inline bool loadCamera(const rclcpp::Node::SharedPtr &node, vk::AbstractCamera *&cam)
{
  bool res = true;
  const std::string cam_model = node->declare_parameter<std::string>("cam_model", "Pinhole");
  if (cam_model == "Ocam")
  {
    std::string calib_file = node->declare_parameter<std::string>("cam_calib_file", "");
    cam = new vk::OmniCamera(calib_file);
  }
  else if (cam_model == "Pinhole")
  {
    const int cam_width = node->declare_parameter<int>("cam_width", 0);
    const int cam_height = node->declare_parameter<int>("cam_height", 0);
    const double scale = node->declare_parameter<double>("scale", 1.0);
    const double cam_fx = node->declare_parameter<double>("cam_fx", 0.0);
    const double cam_fy = node->declare_parameter<double>("cam_fy", 0.0);
    const double cam_cx = node->declare_parameter<double>("cam_cx", 0.0);
    const double cam_cy = node->declare_parameter<double>("cam_cy", 0.0);
    const double cam_d0 = node->declare_parameter<double>("cam_d0", 0.0);
    const double cam_d1 = node->declare_parameter<double>("cam_d1", 0.0);
    const double cam_d2 = node->declare_parameter<double>("cam_d2", 0.0);
    const double cam_d3 = node->declare_parameter<double>("cam_d3", 0.0);
    cam = new vk::PinholeCamera(
        cam_width,
        cam_height,
        scale,
        cam_fx,
        cam_fy,
        cam_cx,
        cam_cy,
        cam_d0,
        cam_d1,
        cam_d2,
        cam_d3);
  }
  else if (cam_model == "EquidistantCamera")
  {
    const int cam_width = node->declare_parameter<int>("cam_width", 0);
    const int cam_height = node->declare_parameter<int>("cam_height", 0);
    const double scale = node->declare_parameter<double>("scale", 1.0);
    const double cam_fx = node->declare_parameter<double>("cam_fx", 0.0);
    const double cam_fy = node->declare_parameter<double>("cam_fy", 0.0);
    const double cam_cx = node->declare_parameter<double>("cam_cx", 0.0);
    const double cam_cy = node->declare_parameter<double>("cam_cy", 0.0);
    const double k1 = node->declare_parameter<double>("k1", 0.0);
    const double k2 = node->declare_parameter<double>("k2", 0.0);
    const double k3 = node->declare_parameter<double>("k3", 0.0);
    const double k4 = node->declare_parameter<double>("k4", 0.0);
    cam = new vk::EquidistantCamera(
        cam_width,
        cam_height,
        scale,
        cam_fx,
        cam_fy,
        cam_cx,
        cam_cy,
        k1,
        k2,
        k3,
        k4);
  }
  else if (cam_model == "PolynomialCamera")
  {
    const int cam_width = node->declare_parameter<int>("cam_width", 0);
    const int cam_height = node->declare_parameter<int>("cam_height", 0);
    const double cam_fx = node->declare_parameter<double>("cam_fx", 0.0);
    const double cam_fy = node->declare_parameter<double>("cam_fy", 0.0);
    const double cam_cx = node->declare_parameter<double>("cam_cx", 0.0);
    const double cam_cy = node->declare_parameter<double>("cam_cy", 0.0);
    const double cam_skew = node->declare_parameter<double>("cam_skew", 0.0);
    const double k2 = node->declare_parameter<double>("k2", 0.0);
    const double k3 = node->declare_parameter<double>("k3", 0.0);
    const double k4 = node->declare_parameter<double>("k4", 0.0);
    const double k5 = node->declare_parameter<double>("k5", 0.0);
    const double k6 = node->declare_parameter<double>("k6", 0.0);
    const double k7 = node->declare_parameter<double>("k7", 0.0);
    cam = new vk::PolynomialCamera(
        cam_width,
        cam_height,
        cam_fx,
        cam_fy,
        cam_cx,
        cam_cy,
        cam_skew,
        k2,
        k3,
        k4,
        k5,
        k6,
        k7);
  }
  else if (cam_model == "ATAN")
  {
    const int cam_width = node->declare_parameter<int>("cam_width", 0);
    const int cam_height = node->declare_parameter<int>("cam_height", 0);
    const double cam_fx = node->declare_parameter<double>("cam_fx", 0.0);
    const double cam_fy = node->declare_parameter<double>("cam_fy", 0.0);
    const double cam_cx = node->declare_parameter<double>("cam_cx", 0.0);
    const double cam_cy = node->declare_parameter<double>("cam_cy", 0.0);
    const double cam_d0 = node->declare_parameter<double>("cam_d0", 0.0);
    cam = new vk::ATANCamera(
        cam_width,
        cam_height,
        cam_fx,
        cam_fy,
        cam_cx,
        cam_cy,
        cam_d0);
  }
  else
  {
    cam = nullptr;
    res = false;
  }
  return res;
}

} // namespace camera_loader
} // namespace fast_livo

