/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/


#pragma once
#include "voxel_map.h"

#include <opencv2/imgproc/imgproc_c.h>
#include <pcl/filters/voxel_grid.h>
#include <vikit/math_utils.h>
#include <vikit/robust_cost.h>
#include <vikit/vision.h>
#include <vikit/pinhole_camera.h>

#include "visual_point.h"

namespace fast_livo {

struct SubSparseMap
{
  vector<float> propa_errors;
  vector<float> errors;
  vector<vector<float>> warp_patch;
  vector<int> search_levels;
  vector<VisualPoint *> voxel_points;
  vector<double> inv_expo_list;
  vector<pointWithVar> add_from_voxel_map;

  SubSparseMap()
  {
    propa_errors.reserve(SIZE_LARGE);
    errors.reserve(SIZE_LARGE);
    warp_patch.reserve(SIZE_LARGE);
    search_levels.reserve(SIZE_LARGE);
    voxel_points.reserve(SIZE_LARGE);
    inv_expo_list.reserve(SIZE_LARGE);
    add_from_voxel_map.reserve(SIZE_SMALL);
  };

  void reset()
  {
    propa_errors.clear();
    errors.clear();
    warp_patch.clear();
    search_levels.clear();
    voxel_points.clear();
    inv_expo_list.clear();
    add_from_voxel_map.clear();
  }
};

class Warp
{
public:
  Matrix2d A_cur_ref;
  int search_level;
  Warp(int level, Matrix2d warp_matrix) : search_level(level), A_cur_ref(warp_matrix) {}
  ~Warp() {}
};

class VOXEL_POINTS
{
public:
  std::vector<VisualPoint *> voxel_points;
  int count;
  VOXEL_POINTS(int num) : count(num) {}
  ~VOXEL_POINTS() 
  { 
    for (VisualPoint* vp : voxel_points) 
    {
      if (vp != nullptr) { delete vp; vp = nullptr; }
    }
  }
};

struct VioAnalysisData
{
  std::vector<int> last_esikf_iterations_per_level_;
  std::vector<int> last_esikf_feature_counts_per_level_;
  int last_inlier_count_ = 0;
  int last_outlier_count_ = 0;
  int last_raycast_retrieved_count_ = 0;
  int last_added_visual_points_ = 0;
  int last_common_tracked_points_ = 0;
  int last_depth_discontinuity_rejects_ = 0;
  float last_shitomasi_avg_ = 0.0f;
  float last_shitomasi_min_ = 0.0f;
  float last_shitomasi_max_ = 0.0f;
  int last_discarded_visual_generation_null_normal_count_ = 0;
  int last_discarded_visual_generation_out_of_frame_count_ = 0;
  float last_discarded_visual_generation_proportion_ = 0.0f;
  cv::Mat depth_discontinuity_depth_map_;
  cv::Mat depth_discontinuity_overlay_;
  std::vector<V3D> last_vio_inlier_points_;
  std::vector<V3D> last_vio_outlier_points_;
  std::vector<V3D> last_vio_optimization_points_;
  std::vector<uint8_t> last_vio_optimization_flags_;
  std::vector<uint8_t> prev_vio_optimization_flags_;
  int last_converged_point_count_ = 0;
  std::vector<V3D> last_converged_points_;
};

class VIOManager
{
public:
  int grid_size;
  vk::AbstractCamera *cam;
  vk::PinholeCamera *pinhole_cam;
  StatesGroup *state;
  StatesGroup *state_propagat;
  M3D R_lidar_body, Rci, R_camera_lidar, Rcw, Jdphi_dR, Jdp_dt, Jdp_dR;
  V3D T_lidar_body, Pci, T_camera_lidar, Pcw;
  vector<int> grid_num;
  vector<int> map_index;
  vector<int> border_flag;
  vector<int> update_flag;
  vector<float> map_dist;
  vector<float> scan_value;
  vector<float> patch_buffer;
  bool normal_en, inverse_composition_en, exposure_estimate_en, raycast_en, has_ref_patch_cache;
  bool orientation_check_en;
  bool shitomasi_threshold_enabled = false;
  bool ncc_en = false, colmap_output_en = false;

  int width, height, grid_n_width, grid_n_height, length;
  double image_resize_factor;
  double fx, fy, cx, cy;
  int patch_pyrimid_level, patch_size, patch_size_total, patch_size_half, border, warp_len;
  int max_iterations;
  int total_points; ///< total number of points in the visual sparse map

  double img_point_cov, outlier_threshold;
  double ncc_threshold; ///< Rejection threshold with NCC
  double depth_discontinuity_threshold;
  double new_feature_min_translation;
  double new_feature_min_rotation;
  double new_feature_min_pixel_dist;
  double min_shitomasi_score = 5.0;
  double raycast_d_min = 0.1;
  double raycast_d_max = 3.0;
  double raycast_step = 0.2;
  double orientation_check_cos_threshold;
  
  SubSparseMap *visual_submap;
  std::vector<std::vector<V3D>> rays_with_sample_points; ///< Precomputed rays coordinate for raycasting

  double compute_jacobian_time, update_ekf_time;
  double ave_total = 0;
  // double ave_build_residual_time = 0;
  // double ave_ekf_time = 0;

  int frame_count = 0;
  bool plot_flag;
  bool rgb_output_en_ = true;

  Eigen::Matrix<double, DIM_STATE, DIM_STATE> G, H_T_H;
  MatrixXd K, H_sub_inv;

  ofstream fout_camera, fout_colmap;
  
  map_type<VOXEL_LOCATION, VOXEL_POINTS *> feat_map; ///< Visual feature map: Contains voxel that contains visual points
  map_type<VOXEL_LOCATION, int> sub_feat_map;  ///< Visual feature visibility indicator for voxels


  map_type<int, Warp *> warp_map;
  vector<VisualPoint *> retrieve_voxel_points;
  vector<pointWithVar> append_voxel_points;
  FramePtr new_frame_;
  cv::Mat img_cp, img_rgb, img_test;

  enum CellType
  {
    TYPE_MAP = 1,
    TYPE_POINTCLOUD,
    TYPE_UNKNOWN
  };

  VIOManager();
  ~VIOManager();
  int updateStateInverse(cv::Mat img, int level);
  int updateState(cv::Mat img, int level);
  void processFrame(cv::Mat &img, vector<pointWithVar> &pg, const map_type<VOXEL_LOCATION, VoxelOctoTree *> &feat_map, double img_time);
  void retrieveFromVisualSparseMap(cv::Mat img, vector<pointWithVar> &pg, const map_type<VOXEL_LOCATION, VoxelOctoTree *> &plane_map);
  void generateVisualMapPoints(cv::Mat img, vector<pointWithVar> &pg);
  void setRgbOutputEnabled(bool enabled) { rgb_output_en_ = enabled; }
  void setLidarToImuExtrinsic(const V3D &T_imu_lidar, const M3D &R_imu_lidar);
  void setLidarToCameraExtrinsic(vector<double> &R, vector<double> &P);
  void setLidarToCameraExtrinsic(const M3D &R, const V3D &P);
  void initializeVIO();
  void getImagePatch(cv::Mat img, V2D pc, float *patch_tmp, int level);
  void computeProjectionJacobian(V3D p, MD(2, 3) & J);
  void computeJacobianAndUpdateEKF(cv::Mat img);
  void resetGrid();
  void updateVisualMapPoints(cv::Mat img);
  void getWarpMatrixAffine(const vk::AbstractCamera &cam, const Vector2d &px_ref, const Vector3d &f_ref, const double depth_ref, const SE3d &T_cur_ref,
                           const int level_ref, 
                           const int pyramid_level, const int halfpatch_size, Matrix2d &A_cur_ref);
  void getWarpMatrixAffineHomography(const vk::AbstractCamera &cam, const V2D &px_ref,
                                     const V3D &xyz_ref, const V3D &normal_ref, const SE3d &T_cur_ref, const int level_ref, Matrix2d &A_cur_ref);
  void warpAffine(const Matrix2d &A_cur_ref, const cv::Mat &img_ref, const Vector2d &px_ref, const int level_ref, const int search_level,
                  const int pyramid_level, const int halfpatch_size, float *patch);
  void insertPointIntoVoxelMap(VisualPoint *pt_new);
  void plotTrackedPoints();
  void updateFrameState(StatesGroup state);
  void projectPatchFromRefToCur(const map_type<VOXEL_LOCATION, VoxelOctoTree *> &plane_map);
  void updateReferencePatch(const map_type<VOXEL_LOCATION, VoxelOctoTree *> &plane_map);
  void precomputeReferencePatches(int level);
  void dumpDataForColmap();
  double calculateNCC(float *ref_patch, float *cur_patch, int patch_size);
  int getBestSearchLevel(const Matrix2d &A_cur_ref, const int max_level);
  V3F getInterpolatedPixel(cv::Mat img, V2D pc);
  const VioAnalysisData &getVioAnalysisData() const { return analysis_data_; }
  const map_type<VOXEL_LOCATION, int> &getVisibleVoxelMap() const { return sub_feat_map; }
  void setGenerateReconstructedView(bool enable) { generate_reconstructed_view_ = enable; }
  void setGenerateSparseDepthMap(bool enable) { generate_sparse_depth_map_ = enable; }
  void setDepthDiscontinuityOverlayOnDepthMap(bool enable) { overlay_depth_discontinuity_on_depth_map_ = enable; }
  void setVoxelSize(double size) { voxel_size_ = size; }
  double getVoxelSize() const { return voxel_size_; }
  
  // void resetRvizDisplay();
  // deque<VisualPoint *> map_cur_frame;
  // deque<VisualPoint *> sub_map_ray;
  // deque<VisualPoint *> sub_map_ray_fov;
  // deque<VisualPoint *> visual_sub_map_cur;
  deque<VisualPoint *> visual_converged_point;
  // std::vector<std::vector<V3D>> sample_points;

  // PointCloudXYZI::Ptr pg_down;
  // pcl::VoxelGrid<PointType> downSizeFilter;

  bool generate_reconstructed_view_ = false;
  bool generate_sparse_depth_map_ = false;
  bool overlay_depth_discontinuity_on_depth_map_ = false;
  VioAnalysisData analysis_data_;

  void updateVioStatistics();
  void updateCommonTrackedPoints();
  void buildReconstructedView(const cv::Mat &img, cv::Mat &out, int level);

private:
  double voxel_size_ = 0.5; ///< Voxel size of the visual sparse map
};
typedef std::shared_ptr<VIOManager> VIOManagerPtr;

}
