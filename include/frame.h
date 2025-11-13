/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/


#pragma once
#include "utils/types.h"

#include <list>
#include <vector>
#include <boost/noncopyable.hpp>
#include <vikit/abstract_camera.h>
#include <sophus/se3.hpp>
#include <opencv2/core/core.hpp>

class VisualPoint;
struct Feature;

typedef std::list<Feature *> Features;
typedef std::vector<cv::Mat> ImgPyr;

/// A frame saves the image, the associated features and the estimated pose.
class Frame : boost::noncopyable
{

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static int frame_counter_; //!< Counts the number of created frames. Used to set the unique id.
  int id_;                   //!< Unique id of the frame.
  vk::AbstractCamera *cam_;  //!< Camera model.
  Sophus::SE3d T_f_w_;                //!< Transform (f)rame from (w)orld.
  Sophus::SE3d T_f_w_prior_;          //!< Transform (f)rame from (w)orld provided by the IMU prior.
  cv::Mat img_;              //!< Image of the frame.
  Features fts_;             //!< List of features in the image.

  Frame(vk::AbstractCamera *cam, const cv::Mat &img);
  ~Frame();

  /// Initialize new frame and create image pyramid.
  void initFrame(const cv::Mat &img);

  /// Return number of point observations.
  inline size_t nObs() const { return fts_.size(); }

  /// Transforms point coordinates in world-frame (w) to camera pixel coordinates (c).
  inline V2D w2c(const V3D &xyz_w) const { return cam_->world2cam(T_f_w_ * xyz_w); }

  /// Transforms point coordinates in world-frame (w) to camera pixel coordinates (c) using the IMU prior pose.
  inline V2D w2c_prior(const V3D &xyz_w) const { return cam_->world2cam(T_f_w_prior_ * xyz_w); }
  
  /// Transforms pixel coordinates (c) to frame unit sphere coordinates (f).
  inline V3D c2f(const V2D &px) const { return cam_->cam2world(px[0], px[1]); }

  /// Transforms pixel coordinates (c) to frame unit sphere coordinates (f).
  inline V3D c2f(const double x, const double y) const { return cam_->cam2world(x, y); }

  /// Transforms point coordinates in world-frame (w) to camera-frams (f).
  inline V3D w2f(const V3D &xyz_w) const { return T_f_w_ * xyz_w; }

  /// Transforms point from frame unit sphere (f) frame to world coordinate frame (w).
  inline V3D f2w(const V3D &f) const { return T_f_w_.inverse() * f; }

  /// Projects Point from unit sphere (f) in camera pixels (c).
  inline V2D f2c(const V3D &f) const { return cam_->world2cam(f); }

  /// Return the pose of the frame in the (w)orld coordinate frame.
  inline V3D pos() const { return T_f_w_.inverse().translation(); }
};

typedef std::unique_ptr<Frame> FramePtr;

/// Some helper functions for the frame object.
namespace frame_utils
{

/// Creates an image pyramid of half-sampled images.
void createImgPyramid(const cv::Mat &img_level_0, int n_levels, ImgPyr &pyr);

} // namespace frame_utils

