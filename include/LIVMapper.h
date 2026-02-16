/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/


#pragma once
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include "IMU_Processing.h"
#include "vio.h"
#include "preprocess.h"
#include <cv_bridge/cv_bridge.hpp>
#include <nav_msgs/msg/path.hpp>

namespace fast_livo
{

/**
  * Additional statistics for Flyability's benchmarks
  */
struct AnalysisPublishers
{
  struct Vio
  {
    std::function<void(const std_msgs::msg::Int32MultiArray &)> esikf_iterations = nullptr;
  };

  Vio vio;

  std::function<void(const std_msgs::msg::Int32MultiArray &)> vio_esikf_iterations = nullptr;
  std::function<void(const std_msgs::msg::Int32MultiArray &)> vio_feature_counts = nullptr;
  std::function<void(const std_msgs::msg::Int32 &)> vio_inlier_count = nullptr;
  std::function<void(const std_msgs::msg::Int32 &)> vio_outlier_count = nullptr;
  std::function<void(const std_msgs::msg::Int32 &)> vio_raycast_count = nullptr;
  std::function<void(const std_msgs::msg::Int32 &)> vio_added_visual_points = nullptr;
  std::function<void(const std_msgs::msg::Int32 &)> vio_common_tracked_points = nullptr;
  std::function<void(const std_msgs::msg::Int32 &)> vio_depth_discontinuity_rejects = nullptr;
  std::function<void(const std_msgs::msg::Float32MultiArray &)> vio_shitomasi_stats = nullptr;
  std::function<void(int, int, float)> vio_discarded_visual_generation = nullptr;
  std::function<void(const sensor_msgs::msg::Image &)> vio_sparse_depth_map = nullptr;
  std::function<void(int, const sensor_msgs::msg::Image &)> vio_reconstructed_view = nullptr;
  std::function<void(const sensor_msgs::msg::PointCloud2 &)> vio_inlier_points = nullptr;
  std::function<void(const sensor_msgs::msg::PointCloud2 &)> vio_outlier_points = nullptr;
  std::function<void(const std_msgs::msg::Int32 &)> vio_optimization_point_count = nullptr;
  std::function<void(const sensor_msgs::msg::PointCloud2 &)> vio_optimization_points = nullptr;
  std::function<void(const std_msgs::msg::Int32 &)> vio_converged_point_count = nullptr;
  std::function<void(const sensor_msgs::msg::PointCloud2 &)> vio_converged_points = nullptr;
  std::function<void(const sensor_msgs::msg::PointCloud2 &, const sensor_msgs::msg::PointCloud2 &)> vio_point_candidates = nullptr;
  std::function<void(const sensor_msgs::msg::Image &)> projected_lidar_camera = nullptr;
  std::function<void(const std_msgs::msg::Int32 &)> lio_esikf_iterations = nullptr;
  std::function<void(const visualization_msgs::msg::MarkerArray &, const visualization_msgs::msg::MarkerArray &, const visualization_msgs::msg::MarkerArray &)> camera_fov_markers = nullptr;
  std::function<void(const std_msgs::msg::Float32MultiArray &)> ekf_biases = nullptr;
};

struct AppPublishers
{
  std::function<void(const visualization_msgs::msg::Marker &)> plane_marker = nullptr;
  std::function<void(const visualization_msgs::msg::MarkerArray &)> voxel_markers = nullptr;
  std::function<void(const sensor_msgs::msg::PointCloud2 &)> laser_cloud_full_res = nullptr;
  std::function<void(const visualization_msgs::msg::MarkerArray &)> normal_markers = nullptr;
  std::function<void(const sensor_msgs::msg::PointCloud2 &)> sub_visual_map = nullptr;
  std::function<void(const sensor_msgs::msg::PointCloud2 &)> laser_cloud_effect = nullptr;
  std::function<void(const sensor_msgs::msg::PointCloud2 &)> laser_cloud_map = nullptr;
  std::function<void(const nav_msgs::msg::Odometry &)> odom_aft_mapped = nullptr;
  std::function<void(const nav_msgs::msg::Path &)> path = nullptr;
  std::function<void(const sensor_msgs::msg::PointCloud2 &)> laser_cloud_dynamic = nullptr;
  std::function<void(const sensor_msgs::msg::PointCloud2 &)> laser_cloud_dynamic_removed = nullptr;
  std::function<void(const sensor_msgs::msg::PointCloud2 &)> laser_cloud_dynamic_debug = nullptr;
  std::function<void(const sensor_msgs::msg::PointCloud2 &)> visual_patches_body = nullptr;
  std::function<void(const sensor_msgs::msg::Image &)> image = nullptr;
  std::function<void(const geometry_msgs::msg::PoseStamped &)> mavros_pose = nullptr;
  std::function<void(const nav_msgs::msg::Odometry &)> imu_prop_odom = nullptr;
};

class LIVMapper
{
public:
  LIVMapper(rclcpp::Node::SharedPtr node);
  ~LIVMapper();
  void initializeSubscribersAndPublishers();
  void initializeComponents();
  void initializeFiles();
  void run();
  void gravityAlignment();
  void handleFirstFrame();
  void stateEstimationAndMapping();
  void handleVIO();
  void handleLIO();
  void savePCD();
  void processImu();
  
  bool sync_packages(LidarMeasureGroup &meas);
  void prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr, V3D angvel_avr);
  void imu_prop_callback();
  void transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud);
  void pointBodyToWorld(const PointType &pi, PointType &po);
 
  void RGBpointBodyToWorld(PointType const *const pi, PointType *const po);
  void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
//   void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstSharedPtr &msg_in);
  void imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr &msg_in);
  void img_cbk(const sensor_msgs::msg::Image::ConstSharedPtr &msg_in);
  void publish_img_rgb(const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &pubImage, VIOManagerPtr vio_manager, const rclcpp::Time &stamp);
  void publish_frame_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFullRes, VIOManagerPtr vio_manager,
                           const rclcpp::Time &stamp);
  void publish_visual_sub_map(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubSubVisualMap, const rclcpp::Time &stamp);
  void publish_effect_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudEffect, const std::vector<PointToPlane> &ptpl_list,
                            const rclcpp::Time &stamp);
  void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &pubOdomAftMapped, const rclcpp::Time &stamp);
  void publish_mavros(const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr &mavros_pose_publisher, const rclcpp::Time &stamp);
  void publish_path(const rclcpp::Time &stamp);
  void publishStaticTf();
  void readParameters(const rclcpp::Node::SharedPtr &node);
  template <typename T> void set_posestamp(T &out);
  template <typename T> void pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi, Eigen::Matrix<T, 3, 1> &po);
  template <typename T> Eigen::Matrix<T, 3, 1> pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi);
  void setAppPublishers(AppPublishers publishers);
  cv::Mat getImageFromMsg(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg);
  rclcpp::Time makeTimeFromSeconds(double seconds) const;

  // Offline app interfaces
  void resetRosInterfaces();

  std::mutex mtx_buffer, mtx_buffer_imu_prop;
  std::condition_variable sig_buffer;
  std::thread spin_thread_;

  SLAM_MODE slam_mode_;
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map;
  
  string root_dir;
  string lid_topic, imu_topic, seq_name, img_topic;
  V3D extT; ///< Translation from Lidar to Body (T_body_lidar)
  M3D extR; ///< Rotation from Lidar to Body (R_body_lidar)

  int feats_down_size = 0, max_iterations = 0;

  double res_mean_last = 0.05;
  double gyr_cov = 0, acc_cov = 0, inv_expo_cov = 0;
  double blind_rgb_points = 0.0;
  bool colorize_map_en = true;
  bool pub_rgb_img_en = true;
  double last_timestamp_img = -1.0;
  double filter_size_surf_min = 0;
  double filter_size_pcd = 0;
  double _first_lidar_time = 0.0;
  double match_time = 0, solve_time = 0, solve_const_H_time = 0;

  bool lidar_map_inited = false, pcd_save_en = false, pub_effect_point_en = false, pose_output_en = false, ros_driver_fix_en = false, hilti_en = false;
  int video_downsampler = 1;
  int pcd_save_interval = -1, pcd_index = 0;
  int pub_scan_num = 1;
  bool save_dense_map_en = true;

  StatesGroup imu_propagate, latest_ekf_state;

  bool new_imu = false, state_update_flg = false, imu_prop_enable = true, ekf_finish_once = false;
  deque<sensor_msgs::msg::Imu> prop_imu_buffer;
  sensor_msgs::msg::Imu newest_imu;
  double latest_ekf_time;
  nav_msgs::msg::Odometry imu_prop_odom;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubImuPropOdom;
  double imu_time_offset = 0.0;
  double lidar_time_offset = 0.0;

  bool gravity_align_en = false, gravity_align_finished = false;

  bool sync_jump_flag = false;

  bool lidar_pushed = false, imu_en, gravity_est_en, flg_reset = false, ba_bg_est_en = true;
  bool dense_map_en = false;
  bool publish_visible_voxels_en = false;
  int img_en = 1, imu_int_frame = 3;
  bool normal_en = true;
  bool orientation_check_en = false;
  double max_view_angle = 85.0;
  bool ncc_en = false;
  double ncc_outlier_threshold = 0.8;
  bool exposure_estimate_en = false;
  double exposure_time_init = 0.0;
  bool inverse_composition_en = false;
  bool raycast_en = false;
  bool debug_lidar_projection_en = false;
  bool generate_projection_images = false;
  std::vector<int> reconstructed_view_levels;
  bool publish_sparse_depth_map = false;
  bool draw_camera_axes_on_rgb = false;
  bool depth_discontinuity_overlay_on_depth_map = false;
  bool publish_vio_inliers_outliers_clouds = false;
  bool publish_vio_optimization_points = true;
  bool publish_converged_points = false;
  bool publish_vio_point_candidates = false;
  bool publish_camera_fov_markers = false;
  double raycast_d_min = 0.1;
  double raycast_d_max = 3.0;
  double raycast_step = 0.2;
  double depth_discontinuity_threshold = 0.5;
  double vio_voxel_size = 0.5;
  bool shitomasi_threshold_enabled = false;
  double new_feature_min_translation = 0.5;
  double new_feature_min_rotation = 0.3;
  double new_feature_min_pixel_dist = 40.0;
  double min_shitomasi_score = 5.0;
  int lidar_en = 1;
  bool is_first_frame = false;
  int grid_size, patch_size, grid_n_width, grid_n_height, patch_pyrimid_level;
  double outlier_threshold;
  double plot_time;
  int frame_cnt;
  double img_time_offset = 0.0;
  deque<PointCloudXYZI::Ptr> lid_raw_data_buffer;
  deque<double> lid_header_time_buffer;
  deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;
  deque<cv::Mat> img_buffer;
  deque<double> img_time_buffer;
  vector<pointWithVar> _pv_list;
  vector<double> extrinT; ///< LiDAR extrinsic translation (T_body_lidar)
  vector<double> extrinR; ///< LiDAR extrinsic rotation as quaternion (qx, qy, qz, qw) (R_body_lidar)
  vector<double> T_camera_lidar_raw;
  std::vector<double> R_camera_lidar_raw;
  bool use_intermediate_extrinsic_ = true; ///< Use body->vio and vio->cam chain to derive lidar->cam
  std::vector<double> T_camera_vio_raw; ///< Translation of VIO in camera frame
  std::vector<double> R_camera_vio_raw; ///< Quaternion (qx, qy, qz, qw) for VIO -> Camera
  int camera_id_ = 0;
  std::string camera_extrinsics_prefix_;
  std::string camera_extrinsics_T_param_;
  std::string camera_extrinsics_R_param_;
  std::vector<double> T_body_vio_raw;       ///< Translation of VIO in IMU frame
  std::vector<double> R_body_vio_raw;       ///< Quaternion (qx, qy, qz, qw) for VIO -> Body 
  double IMG_POINT_COV;

  PointCloudXYZI::Ptr visual_sub_map;
  PointCloudXYZI::Ptr feats_undistort;
  PointCloudXYZI::Ptr feats_down_body;
  PointCloudXYZI::Ptr feats_down_world;
  PointCloudXYZI::Ptr pcl_w_wait_pub;
  PointCloudXYZI::Ptr pcl_wait_pub;
  PointCloudXYZRGB::Ptr pcl_wait_save;
  PointCloudXYZI::Ptr pcl_wait_save_intensity;

  ofstream fout_pre, fout_out, fout_pcd_pos, fout_points;

  pcl::VoxelGrid<PointType> downSizeFilterSurf;

  V3D euler_cur;

  LidarMeasureGroup LidarMeasures;
  StatesGroup _state;
  StatesGroup  state_propagat;

  nav_msgs::msg::Path path;
  nav_msgs::msg::Odometry odomAftMapped;
  geometry_msgs::msg::Quaternion geoQuat;
  geometry_msgs::msg::PoseStamped msg_body_pose;
  geometry_msgs::msg::Transform latest_tf_transform_;
  rclcpp::Time latest_tf_time_;
  rclcpp::Time latest_tf_wall_time_;
  bool latest_tf_valid_ = false;

  PreprocessPtr p_pre;
  ImuProcessPtr p_imu;
  VoxelMapManagerPtr voxelmap_manager;
  VIOManagerPtr vio_manager;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr plane_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr voxel_pub;
  rclcpp::SubscriptionBase::SharedPtr sub_pcl;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullRes;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubNormal;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSubVisualMap;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudDyn;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudDynRmed;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudDynDbg;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pubImage;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mavros_pose_publisher;
  rclcpp::TimerBase::SharedPtr imu_prop_timer;

  int frame_num = 0;
  double aver_time_consu = 0;
  double aver_time_icp = 0;
  double aver_time_map_inre = 0;
  bool colmap_output_en = false;

  private:
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr static_tf_timer_;
    rclcpp::TimerBase::SharedPtr tf_hold_timer_;
    tf2::Transform TF_lidar_body_; ///< Transform to compute P_lidar = R_lidar_body * P_body + T_lidar_body
    tf2::Transform TF_vio_body_; ///< Transform to compute P_vio = R_vio_body * P_body + T_vio_body
    tf2::Transform TF_cam_vio_; ///< Transform to compute P_camera = R_camera_vio * P_vio + T_camera_vio
    tf2::Transform TF_cam_body_; 
    tf2::Transform TF_cam_lidar_;
    bool has_vio_tf_ = false;
    bool has_cam_tf_ = false;
    std::string lidar_frame_id_ = "lidar_frame";
    std::string vio_frame_id_ = "vio";
    std::string camera_frame_id_ = "camera";
    AppPublishers app_publishers_;

    std::optional<rclcpp::Time> last_timestamp_lidar_;
    std::optional<rclcpp::Time> last_timestamp_imu_;

    bool checkParametersValidity() const;
    void initializeTransforms();
    tf2::Transform computeBodyToLidarTf() const;
};

} // namespace fast_livo
