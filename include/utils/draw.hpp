#pragma once

#include <cmath>

#include <opencv2/imgproc.hpp>

namespace fast_livo::utils
{

inline void  drawCameraAxesOnRgb(cv::Mat &img_rgb)
{
    const int width = img_rgb.cols;
    const int height = img_rgb.rows;
    const int min_dim = std::min(width, height);
    const int margin = std::max(10, min_dim / 25);
    const int axis_len = std::max(20, min_dim / 8);
    const int thickness = 2;
    const int text_thickness = 1;
    const double font_scale = std::max(0.4, min_dim / 600.0);

    auto clamp_int = [](int v, int lo, int hi) { return std::max(lo, std::min(v, hi)); };

    cv::Point origin_uv(margin, margin);
    origin_uv.x = clamp_int(origin_uv.x, margin, width - margin - axis_len);
    origin_uv.y = clamp_int(origin_uv.y, margin, height - margin - axis_len);

    const int triad_offset = axis_len + 3 * margin;
    cv::Point origin_cam(origin_uv.x + triad_offset, origin_uv.y);
    origin_cam.x = clamp_int(origin_cam.x, margin, width - margin - axis_len);
    origin_cam.y = clamp_int(origin_cam.y, margin, height - margin - axis_len);

    cv::arrowedLine(img_rgb, origin_uv, origin_uv + cv::Point(axis_len, 0),
                    cv::Scalar(0, 255, 255), thickness, cv::LINE_AA);
    cv::arrowedLine(img_rgb, origin_uv, origin_uv + cv::Point(0, axis_len),
                    cv::Scalar(255, 255, 0), thickness, cv::LINE_AA);
    cv::putText(img_rgb, "u", origin_uv + cv::Point(axis_len + 6, -6),
                cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0, 255, 255), text_thickness, cv::LINE_AA);
    cv::putText(img_rgb, "v", origin_uv + cv::Point(-6, axis_len + 14),
                cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(255, 255, 0), text_thickness, cv::LINE_AA);

    cv::arrowedLine(img_rgb, origin_cam, origin_cam + cv::Point(axis_len, 0),
                    cv::Scalar(0, 0, 255), thickness, cv::LINE_AA);
    cv::arrowedLine(img_rgb, origin_cam, origin_cam + cv::Point(0, axis_len),
                    cv::Scalar(0, 255, 0), thickness, cv::LINE_AA);
    cv::putText(img_rgb, "X", origin_cam + cv::Point(axis_len + 6, -6),
                cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0, 0, 255), text_thickness, cv::LINE_AA);
    cv::putText(img_rgb, "Y", origin_cam + cv::Point(-6, axis_len + 14),
                cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0, 255, 0), text_thickness, cv::LINE_AA);

    const int radius = std::max(6, axis_len / 6);
    cv::circle(img_rgb, origin_cam, radius, cv::Scalar(255, 0, 0), thickness, cv::LINE_AA);
    const int cross = radius - 2;
    cv::line(img_rgb, origin_cam + cv::Point(-cross, -cross), origin_cam + cv::Point(cross, cross),
             cv::Scalar(255, 0, 0), thickness, cv::LINE_AA);
    cv::line(img_rgb, origin_cam + cv::Point(-cross, cross), origin_cam + cv::Point(cross, -cross),
             cv::Scalar(255, 0, 0), thickness, cv::LINE_AA);
    cv::putText(img_rgb, "Z", origin_cam + cv::Point(radius + 6, radius + 6),
                cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(255, 0, 0), text_thickness, cv::LINE_AA);
}

inline void overlayBorder(cv::Mat &img, int border, double alpha)
{
  if (img.empty() || border <= 0) return;
  if (img.channels() == 1) cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
  const int w = img.cols;
  const int h = img.rows;
  const int b = std::min(border, std::min(w / 2, h / 2));
  if (b <= 0) return;
  cv::Mat overlay = img.clone();
  const cv::Scalar red(0, 0, 255);
  cv::rectangle(overlay, cv::Rect(0, 0, w, b), red, cv::FILLED);
  cv::rectangle(overlay, cv::Rect(0, h - b, w, b), red, cv::FILLED);
  cv::rectangle(overlay, cv::Rect(0, b, b, h - 2 * b), red, cv::FILLED);
  cv::rectangle(overlay, cv::Rect(w - b, b, b, h - 2 * b), red, cv::FILLED);
  cv::addWeighted(overlay, alpha, img, 1.0 - alpha, 0.0, img);
}

inline void overlayBorderColor(cv::Mat &img, int border, const cv::Scalar &color, double alpha)
{
  if (img.empty() || border <= 0) return;
  if (img.channels() == 1) cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
  const int w = img.cols;
  const int h = img.rows;
  const int b = std::min(border, std::min((w - 1) / 2, (h - 1) / 2));
  if (b <= 0) return;
  const cv::Point tl(b, b);
  const cv::Point br(w - 1 - b, h - 1 - b);
  if (tl.x >= br.x || tl.y >= br.y) return;
  if (alpha >= 1.0) {
    cv::rectangle(img, tl, br, color, 2);
  } else {
    cv::Mat overlay = img.clone();
    cv::rectangle(overlay, tl, br, color, 2);
    cv::addWeighted(overlay, alpha, img, 1.0 - alpha, 0.0, img);
  }
}


} // namespace fast_livo::utils