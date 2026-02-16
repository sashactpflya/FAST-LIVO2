/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "LIVMapper.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <rclcpp/clock.hpp>
#include <spdlog/spdlog.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "utils/color.h"
#include "utils/ros_tf2_conversions.hpp"
#include "utils/ros_pcl_conversions.h"
#include "utils/camera_loader.hpp"
#include "utils/time.hpp"

namespace fast_livo {

LIVMapper::LIVMapper(rclcpp::Node::SharedPtr node)
    : extT(0, 0, 0),
      extR(M3D::Identity()),
      node_(node)
{
  extrinT.assign(3, 0.0);
  extrinR = {0.0, 0.0, 0.0, 1.0};
  T_camera_lidar_raw.assign(3, 0.0);
  R_camera_lidar_raw.assign(9, 0.0);

  p_pre = std::make_shared<Preprocess>();
  p_imu = std::make_shared<ImuProcess>();

  readParameters(node_);

  if ( !checkParametersValidity())
  {
    throw std::runtime_error("Invalid parameters detected. See log for details.");
  }

  lidar_frame_id_ = getLidarFrameName(static_cast<LID_TYPE>(p_pre->lidar_type));
  vio_frame_id_ = "vio";
  camera_frame_id_ = "camera";
  VoxelMapConfig voxel_config;
  loadVoxelConfig(node_, voxel_config);

  visual_sub_map = std::make_shared<PointCloudXYZI>();
  feats_undistort = std::make_shared<PointCloudXYZI>();
  feats_down_body = std::make_shared<PointCloudXYZI>();
  feats_down_world = std::make_shared<PointCloudXYZI>();
  pcl_w_wait_pub = std::make_shared<PointCloudXYZI>();
  pcl_wait_pub = std::make_shared<PointCloudXYZI>();
  pcl_wait_save = std::make_shared<PointCloudXYZRGB>();
  pcl_wait_save_intensity = std::make_shared<PointCloudXYZI>();
  voxelmap_manager = std::make_shared<VoxelMapManager>(voxel_config, voxel_map);
  vio_manager = std::make_shared<VIOManager>();
  root_dir = ROOT_DIR;
  initializeFiles();
  initializeComponents();
  path.header.stamp = rclcpp::Time();
  path.header.frame_id = "init_pose";

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
  static_tf_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
}

LIVMapper::~LIVMapper() {}

void LIVMapper::readParameters(const rclcpp::Node::SharedPtr &node)
{
  lid_topic = node->declare_parameter<std::string>("common.lid_topic", "/livox/lidar");
  imu_topic = node->declare_parameter<std::string>("common.imu_topic", "/livox/imu");
  ros_driver_fix_en = node->declare_parameter<bool>("common.ros_driver_bug_fix", false);
  img_en = node->declare_parameter<int>("common.img_en", 0);
  lidar_en = node->declare_parameter<int>("common.lidar_en", 0);
  img_topic = node->declare_parameter<std::string>("common.img_topic", "/left_camera/image");

  normal_en = node->declare_parameter<bool>("vio.normal_en", true);
  orientation_check_en = node->declare_parameter<bool>("vio.orientation_check_en", false);
  max_view_angle = node->declare_parameter<double>("vio.max_view_angle", 85.0);
  ncc_en = node->declare_parameter<bool>("vio.ncc_en", false);
  ncc_outlier_threshold = node->declare_parameter<double>("vio.ncc_outlier_threshold", 0.8);
  inverse_composition_en = node->declare_parameter<bool>("vio.inverse_composition_en", false);
  max_iterations = node->declare_parameter<int>("vio.max_iterations", 5);
  IMG_POINT_COV = node->declare_parameter<double>("vio.img_point_cov", 100.0);
  raycast_en = node->declare_parameter<bool>("vio.raycast_en", false);
  raycast_d_min = node->declare_parameter<double>("vio.raycast_d_min", 0.1);
  raycast_d_max = node->declare_parameter<double>("vio.raycast_d_max", 3.0);
  raycast_step = node->declare_parameter<double>("vio.raycast_step", 0.2);
  depth_discontinuity_threshold = node->declare_parameter<double>("vio.depth_discontinuity_threshold", 0.5);
  vio_voxel_size = node->declare_parameter<double>("vio.voxel_size", 0.5);
  shitomasi_threshold_enabled = node->declare_parameter<bool>("vio.enable_shi_tomasi_threshold", false);
  exposure_estimate_en = node->declare_parameter<bool>("vio.exposure_estimate_en", true);
  inv_expo_cov = node->declare_parameter<double>("vio.inv_expo_cov", 0.2);
  grid_size = node->declare_parameter<int>("vio.grid_size", 5);
  grid_n_height = node->declare_parameter<int>("vio.grid_n_height", 17);
  patch_pyrimid_level = node->declare_parameter<int>("vio.patch_pyrimid_level", 3);
  patch_size = node->declare_parameter<int>("vio.patch_size", 8);
  outlier_threshold = node->declare_parameter<double>("vio.outlier_threshold", 1000.0);
  new_feature_min_translation = node->declare_parameter<double>("vio.new_feature_min_translation", 0.5);
  new_feature_min_rotation = node->declare_parameter<double>("vio.new_feature_min_rotation", 0.3);
  new_feature_min_pixel_dist = node->declare_parameter<double>("vio.new_feature_min_pixel_dist", 40.0);
  min_shitomasi_score = node->declare_parameter<double>("vio.min_shitomasi_score", 5.0);

  exposure_time_init = node->declare_parameter<double>("time_offset.exposure_time_init", 0.0);
  img_time_offset = node->declare_parameter<double>("time_offset.img_time_offset", 0.0);
  imu_time_offset = node->declare_parameter<double>("time_offset.imu_time_offset", 0.0);
  lidar_time_offset = node->declare_parameter<double>("time_offset.lidar_time_offset", 0.0);
  imu_prop_enable = node->declare_parameter<bool>("uav.imu_rate_odom", false);
  gravity_align_en = node->declare_parameter<bool>("uav.gravity_align_en", false);

  camera_id_ = node->declare_parameter<int>("camera_id", 0);
  camera_extrinsics_prefix_ = "intermediate_camera_extrinsics.cameras.camera_" + std::to_string(camera_id_) + ".";
  camera_extrinsics_T_param_ = camera_extrinsics_prefix_ + "T_camera_vio";
  camera_extrinsics_R_param_ = camera_extrinsics_prefix_ + "R_camera_vio";

  seq_name = node->declare_parameter<std::string>("evo.seq_name", "01");
  pose_output_en = node->declare_parameter<bool>("evo.pose_output_en", false);
  gyr_cov = node->declare_parameter<double>("imu.gyr_cov", 1.0);
  acc_cov = node->declare_parameter<double>("imu.acc_cov", 1.0);
  imu_int_frame = node->declare_parameter<int>("imu.imu_int_frame", 3);
  imu_en = node->declare_parameter<bool>("imu.imu_en", false);
  gravity_est_en = node->declare_parameter<bool>("imu.gravity_est_en", true);
  ba_bg_est_en = node->declare_parameter<bool>("imu.ba_bg_est_en", true);

  p_pre->blind = node->declare_parameter<double>("preprocess.blind", 0.01);
  filter_size_surf_min = node->declare_parameter<double>("preprocess.filter_size_surf", 0.5);
  hilti_en = node->declare_parameter<bool>("preprocess.hilti_en", false);
  video_downsampler = node->declare_parameter<int>("preprocess.video_downsampler", 1);
  p_pre->lidar_type = node->declare_parameter<int>("preprocess.lidar_type", UNKNOWN);
  p_pre->N_SCANS = node->declare_parameter<int>("preprocess.scan_line", 6);
  p_pre->point_filter_num = node->declare_parameter<int>("preprocess.point_filter_num", 3);
  p_pre->feature_enabled = node->declare_parameter<bool>("preprocess.feature_extract_enabled", false);

  pcd_save_interval = node->declare_parameter<int>("pcd_save.interval", -1);
  pcd_save_en = node->declare_parameter<bool>("pcd_save.pcd_save_en", false);
  colmap_output_en = node->declare_parameter<bool>("pcd_save.colmap_output_en", false);
  save_dense_map_en = node->declare_parameter<bool>("pcd_save.save_dense_map", true);
  filter_size_pcd = node->declare_parameter<double>("pcd_save.filter_size_pcd", 0.5);
  extrinT = node->declare_parameter<std::vector<double>>("extrin_calib.extrinsic_T", extrinT);
  extrinR = node->declare_parameter<std::vector<double>>("extrin_calib.extrinsic_R", extrinR);
  T_camera_lidar_raw = node->declare_parameter<std::vector<double>>("extrin_calib.T_camera_lidar", std::vector<double>{});
  R_camera_lidar_raw = node->declare_parameter<std::vector<double>>("extrin_calib.R_camera_lidar", std::vector<double>{});
  T_camera_vio_raw =
      node->declare_parameter<std::vector<double>>(camera_extrinsics_T_param_, std::vector<double>{});
  R_camera_vio_raw =
      node->declare_parameter<std::vector<double>>(camera_extrinsics_R_param_, std::vector<double>{});
  T_body_vio_raw =
      node->declare_parameter<std::vector<double>>("intermediate_camera_extrinsics.T_body_vio", std::vector<double>{});
  R_body_vio_raw =
      node->declare_parameter<std::vector<double>>("intermediate_camera_extrinsics.R_body_vio", std::vector<double>{});
  plot_time = node->declare_parameter<double>("analysis.plot_time", -10.0);
  frame_cnt = node->declare_parameter<int>("analysis.frame_cnt", 6);
  debug_lidar_projection_en = node->declare_parameter<bool>("analysis.lidar_projection_en", false);
  generate_projection_images = node->declare_parameter<bool>("analysis.generate_projection_images", false);
  const std::vector<int64_t> reconstructed_view_levels_param =
      node->declare_parameter<std::vector<int64_t>>("analysis.reconstructed_view_levels", std::vector<int64_t>{});
  reconstructed_view_levels.clear();
  reconstructed_view_levels.reserve(reconstructed_view_levels_param.size());
  for (int64_t level : reconstructed_view_levels_param) {
    reconstructed_view_levels.push_back(static_cast<int>(level));
  }
  publish_sparse_depth_map = node->declare_parameter<bool>("analysis.publish_sparse_depth_map", false);
  draw_camera_axes_on_rgb = node->declare_parameter<bool>("analysis.draw_camera_axes_on_rgb", false);
  publish_converged_points = node->declare_parameter<bool>("analysis.publish_converged_points", false);
  publish_camera_fov_markers = node->declare_parameter<bool>("analysis.publish_camera_fov_markers", false);
  depth_discontinuity_overlay_on_depth_map =
      node->declare_parameter<bool>("analysis.depth_discontinuity_overlay_on_depth_map", false);
  publish_vio_inliers_outliers_clouds =
      node->declare_parameter<bool>("analysis.publish_vio_inliers_outliers_clouds", false);
  publish_vio_optimization_points =
      node->declare_parameter<bool>("analysis.publish_vio_optimization_points", false);
  publish_vio_point_candidates =
      node->declare_parameter<bool>("analysis.publish_vio_point_candidates", false);

  blind_rgb_points = node->declare_parameter<double>("publish.blind_rgb_points", 0.01);
  colorize_map_en = node->declare_parameter<bool>("common.colorize_map_en", true);
  pub_scan_num = node->declare_parameter<int>("publish.pub_scan_num", 1);
  pub_effect_point_en = node->declare_parameter<bool>("publish.pub_effect_point_en", false);
  dense_map_en = node->declare_parameter<bool>("publish.dense_map_en", false);

  use_intermediate_extrinsic_ = node->declare_parameter<bool>("extrin_calib.use_intermediate_extrinsic", true);
  p_pre->blind_sqr = p_pre->blind * p_pre->blind;
}

bool LIVMapper::checkParametersValidity() const
{
    bool valid = true;

    if ( lidar_en == 0 && img_en == 0 )
    {
        spdlog::error("Both lidar_en and img_en are set to 0; at least one sensor must be enabled.");
        valid = false;
    }

    if ( p_pre->lidar_type == UNKNOWN )
    {
        spdlog::error("Invalid lidar_type: {}. Please choose a valid LID_TYPE enum value.", p_pre->lidar_type);
        valid = false;
    }

    if( extrinT.size() != 3 )
    {
        spdlog::error("extrin_calib.extrinsic_T should have exactly 3 elements.");
        valid = false;
    }
    if( extrinR.size() != 4 )
    {
        spdlog::error("extrin_calib.extrinsic_R should have exactly 4 elements (quaternion).");
        valid = false;
    }
    if( T_camera_lidar_raw.size() != 3 )
    {
        spdlog::error("extrin_calib.T_camera_lidar should have exactly 3 elements.");
        valid = false;
    }
    if( R_camera_lidar_raw.size() != 9 )
    {
        spdlog::error("extrin_calib.R_camera_lidar should have exactly 9 elements (rotation matrix).");
        valid = false;
    }

    return valid;
}

void LIVMapper::initializeTransforms()
{
    auto getVector3 = [](const std::vector<double> &vec, const char *name, bool &ok) {
      if (vec.size() == 3) return tf2::Vector3(VEC_FROM_ARRAY(vec));
      spdlog::warn("Parameter {} has {} entries, expected 3. Using zeros.", name, vec.size());
      ok = false;
      return tf2::Vector3(0.0, 0.0, 0.0);
    };
    auto getQuaternion = [](const std::vector<double> &vec, const char *name, bool &ok) {
      if (vec.size() == 4) return TF2_QUAT_FROM_ARRAY(vec);
      spdlog::warn("Parameter {} has {} entries, expected 4. Using identity.", name, vec.size());
      ok = false;
      return tf2::Quaternion(0.0, 0.0, 0.0, 1.0);
    };
    auto getRotationMatrix = [](const std::vector<double> &vec, const char *name, bool &ok) {
      if (vec.size() == 9) {
        tf2::Matrix3x3 R = tf2::Matrix3x3(
            vec[0], vec[1], vec[2],
            vec[3], vec[4], vec[5],
            vec[6], vec[7], vec[8]);
        return R;
      }
      spdlog::warn("Parameter {} has {} entries, expected 9. Using identity.", name, vec.size());
      ok = false;
      return tf2::Matrix3x3::getIdentity();
    };

    // Lidar -> Body

    // P_body = R_body_lidar * P_lidar + T_body_lidar
    tf2::Vector3 T_body_lidar= tf2::Vector3(VEC_FROM_ARRAY(extrinT));
    tf2::Quaternion R_body_lidar = TF2_QUAT_FROM_ARRAY(extrinR).normalized();
    
    
    tf2::Transform TF_body_lidar;
    TF_body_lidar.setOrigin(T_body_lidar);
    TF_body_lidar.setRotation(R_body_lidar);
    TF_lidar_body_ = TF_body_lidar.inverse();

    if(use_intermediate_extrinsic_)
    {
// P_vio = R_body_vio * P_body + T_body_vio
      // P_body = R_body_vio^-1 * (P_vio - T_body_vio)
      // <=>
      // P_body = R_body_vio^-1 * P_vio + (- R_body_vio^-1 * T_body_vio)
      bool vio_to_body_ok = true;
      
      tf2::Vector3 T_body_vio = getVector3(T_body_vio_raw, "extrin_calib.T_body_vio", vio_to_body_ok);
        tf2::Quaternion Q_body_vio = getQuaternion(R_body_vio_raw, "extrin_calib.R_body_vio", vio_to_body_ok);

        tf2::Transform TF_body_vio;
      if( vio_to_body_ok )
        {
            has_vio_tf_ = true;
TF_body_vio.setOrigin(T_body_vio);
TF_body_vio.setRotation(Q_body_vio);

        TF_vio_body_ = TF_body_vio.inverse();
        }

        // Camera to VIO:
      bool cam_to_vio_ok = true;
      tf2::Vector3 T_vio_camera = getVector3(T_vio_camera_raw, "extrin_calib.T_vio_camera", cam_to_vio_ok);
      tf2::Quaternion Q_vio_camera = getQuaternion(R_vio_camera_raw, "extrin_calib.R_vio_camera", cam_to_vio_ok);

      tf2::Transform TF_camera_vio;
      if( cam_to_vio_ok )
      {
        
        TF_camera_vio.setOrigin(T_vio_camera);
        TF_camera_vio.setRotation(Q_vio_camera);

        TF_cam_vio_ = TF_camera_vio.inverse();
                }

        if( cam_to_vio_ok && vio_to_body_ok )
        {
            spdlog::info("Using intermediate extrinsic chain for lidar->camera transform.");
            
            has_cam_tf_ = true;

            tf2::Transform TF_cam_body;
        TF_cam_body = TF_camera_vio * TF_body_vio.inverse();

            // Compose Body -> Camera
body_to_cam_tf_ = TF_cam_body;
            lidar_to_cam_tf_ = TF_cam_body * TF_body_lidar;
            return;
        }
        else
        {
          spdlog::warn("Incomplete intermediate extrinsic parameters; fallback to direct lidar->camera extrinsic.");
        }
    }

    // Otherwise, use direct R_camera_lidar/T_camera_lidar
    lidar_to_cam_tf_.setOrigin(getVector3(T_camera_lidar_raw, "extrin_calib.T_camera_lidar", has_cam_tf_));
    tf2::Quaternion q;
    getRotationMatrix(R_camera_lidar_raw, "extrin_calib.R_camera_lidar", has_cam_tf_).getRotation(q);
    lidar_to_cam_tf_.setRotation(q);
}

void LIVMapper::initializeComponents()
{
  using fast_livo::camera_loader::loadCamera;

  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
  extT << VEC_FROM_ARRAY(extrinT);
  extR = EIGEN_QUAT_FROM_ARRAY(extrinR).toRotationMatrix();

  initializeTransforms();

  voxelmap_manager->extT_ << VEC_FROM_ARRAY(extrinT);
  voxelmap_manager->extR_ = extR;

  if (!loadCamera(node_, vio_manager->cam)) throw std::runtime_error("Camera model not correctly specified.");

  vio_manager->grid_size = grid_size;
  vio_manager->patch_size = patch_size;
  vio_manager->outlier_threshold = outlier_threshold;
  vio_manager->setImuToLidarExtrinsic(extT, extR);

  Eigen::Quaterniond lidar_to_cam_q(lidar_to_cam_tf_.getRotation().w(),
                                        lidar_to_cam_tf_.getRotation().x(),
                                        lidar_to_cam_tf_.getRotation().y(),
                                        lidar_to_cam_tf_.getRotation().z());
  Eigen::Matrix3d lidar_to_cam_R = lidar_to_cam_q.normalized().toRotationMatrix();
  V3D lidar_to_cam_T = V3D(lidar_to_cam_tf_.getOrigin().x(),
                                    lidar_to_cam_tf_.getOrigin().y(),
                                    lidar_to_cam_tf_.getOrigin().z());
  vio_manager->setLidarToCameraExtrinsic(lidar_to_cam_R, lidar_to_cam_T);

  vio_manager->state = &_state;
  vio_manager->state_propagat = &state_propagat;
  vio_manager->max_iterations = max_iterations;
  vio_manager->img_point_cov = IMG_POINT_COV;
  vio_manager->normal_en = normal_en;
  vio_manager->inverse_composition_en = inverse_composition_en;
  vio_manager->raycast_en = raycast_en;
vio_manager->depth_discontinuity_threshold = depth_discontinuity_threshold;
  vio_manager->grid_n_width = grid_n_width;
  vio_manager->grid_n_height = grid_n_height;
  vio_manager->patch_pyrimid_level = patch_pyrimid_level;
  vio_manager->exposure_estimate_en = exposure_estimate_en;
  vio_manager->colmap_output_en = colmap_output_en;
  vio_manager->ncc_en = ncc_en;
  vio_manager->ncc_threshold = ncc_outlier_threshold;
vio_manager->new_feature_min_translation = new_feature_min_translation;
  vio_manager->new_feature_min_rotation = new_feature_min_rotation;
  vio_manager->new_feature_min_pixel_dist = new_feature_min_pixel_dist;
  vio_manager->initializeVIO();

  p_imu->set_extrinsic(extT, extR);
  p_imu->set_gyr_cov_scale(V3D(gyr_cov, gyr_cov, gyr_cov));
  p_imu->set_acc_cov_scale(V3D(acc_cov, acc_cov, acc_cov));
  p_imu->set_inv_expo_cov(inv_expo_cov);
  p_imu->set_gyr_bias_cov(V3D(0.0001, 0.0001, 0.0001));
  p_imu->set_acc_bias_cov(V3D(0.0001, 0.0001, 0.0001));
  p_imu->set_imu_init_frame_num(imu_int_frame);

  if (!imu_en) p_imu->disable_imu();
  if (!gravity_est_en) p_imu->disable_gravity_est();
  if (!ba_bg_est_en) p_imu->disable_bias_est();
  if (!exposure_estimate_en) p_imu->disable_exposure_est();

  slam_mode_ = (img_en && lidar_en) ? LIVO : imu_en ? ONLY_LIO : ONLY_LO;
}

void LIVMapper::initializeFiles() 
{
  if (pcd_save_en && colmap_output_en)
  {
      const std::string folderPath = std::string(ROOT_DIR) + "/scripts/colmap_output.sh";
      
      std::string chmodCommand = "chmod +x " + folderPath;
      
      int chmodRet = system(chmodCommand.c_str());  
      if (chmodRet != 0) {
          std::cerr << "Failed to set execute permissions for the script." << std::endl;
          return;
      }

      int executionRet = system(folderPath.c_str());
      if (executionRet != 0) {
          std::cerr << "Failed to execute the script." << std::endl;
          return;
      }
  }
  if(colmap_output_en) fout_points.open(std::string(ROOT_DIR) + "Log/Colmap/sparse/0/points3D.txt", std::ios::out);
  if(pcd_save_interval > 0) fout_pcd_pos.open(std::string(ROOT_DIR) + "Log/PCD/scans_pos.json", std::ios::out);
  fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"), std::ios::out);
  fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), std::ios::out);
}

void LIVMapper::initializeSubscribersAndPublishers() 
{
  using std::placeholders::_1;
  const auto high_queue_qos = rclcpp::QoS(rclcpp::KeepLast(200000));
//   if (p_pre->lidar_type == AVIA)
//   {
//     sub_pcl = node->create_subscription<livox_ros_driver::CustomMsg>(
//         lid_topic, high_queue_qos,
//         std::bind(&LIVMapper::livox_pcl_cbk, this, _1));
//   }
//   else
//   {
//     sub_pcl = node->create_subscription<sensor_msgs::msg::PointCloud2>(
//         lid_topic, high_queue_qos,
//         std::bind(&LIVMapper::standard_pcl_cbk, this, _1));
//   }
  sub_pcl = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    lid_topic, high_queue_qos,
    std::bind(&LIVMapper::standard_pcl_cbk, this, _1));

  sub_imu = node_->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, high_queue_qos,
      std::bind(&LIVMapper::imu_cbk, this, _1));
  sub_img = node_->create_subscription<sensor_msgs::msg::Image>(
      img_topic, high_queue_qos,
      std::bind(&LIVMapper::img_cbk, this, _1));

  pubLaserCloudFullRes = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/livo2/cloud_registered", 100);
  pubNormal = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/livo2/visualization_marker", 100);
  pubSubVisualMap = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/livo2/cloud_visual_sub_map_before", 100);
  pubLaserCloudEffect = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/livo2/cloud_effected", 100);
  pubLaserCloudMap = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/livo2/Laser_map", 100);
  pubOdomAftMapped = node_->create_publisher<nav_msgs::msg::Odometry>("/livo2/aft_mapped_to_init", 10);
  pubPath = node_->create_publisher<nav_msgs::msg::Path>("/livo2/path", 10);
  plane_pub = node_->create_publisher<visualization_msgs::msg::Marker>("/livo2/planner_normal", 1);
  voxel_pub = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/livo2/voxels", 1);
  pubLaserCloudDyn = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/livo2/dyn_obj", 100);
  pubLaserCloudDynRmed = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/livo2/dyn_obj_removed", 100);
  pubLaserCloudDynDbg = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/livo2/dyn_obj_dbg_hist", 100);
  mavros_pose_publisher = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/livo2/mavros/vision_pose/pose", 10);
  pubImage = it.advertise("/livo2/rgb_img", 1);
  pubImuPropOdom = node_->create_publisher<nav_msgs::msg::Odometry>("/livo2/LIVO2/imu_propagate", 10000);
  imu_prop_timer = node_->create_wall_timer(
      std::chrono::milliseconds(4),
      std::bind(&LIVMapper::imu_prop_callback, this));
  
  voxelmap_manager->voxel_map_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/livo2/planes", 10000);
  static_tf_timer_ = node_->create_wall_timer(
      std::chrono::seconds(10),
      std::bind(&LIVMapper::publishStaticTf, this));
  tf_hold_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&LIVMapper::publish_tf_hold, this));

  app_publishers_.plane_marker = [pub = plane_pub](const visualization_msgs::msg::Marker &msg) { pub->publish(msg); };
  app_publishers_.voxel_markers = [pub = voxel_pub](const visualization_msgs::msg::MarkerArray &msg) { pub->publish(msg); };
  app_publishers_.laser_cloud_full_res = [pub = pubLaserCloudFullRes](const sensor_msgs::msg::PointCloud2 &msg) { pub->publish(msg); };
  app_publishers_.normal_markers = [pub = pubNormal](const visualization_msgs::msg::MarkerArray &msg) { pub->publish(msg); };
  app_publishers_.sub_visual_map = [pub = pubSubVisualMap](const sensor_msgs::msg::PointCloud2 &msg) { pub->publish(msg); };
  app_publishers_.laser_cloud_effect = [pub = pubLaserCloudEffect](const sensor_msgs::msg::PointCloud2 &msg) { pub->publish(msg); };
  app_publishers_.laser_cloud_map = [pub = pubLaserCloudMap](const sensor_msgs::msg::PointCloud2 &msg) { pub->publish(msg); };
  app_publishers_.odom_aft_mapped = [pub = pubOdomAftMapped](const nav_msgs::msg::Odometry &msg) { pub->publish(msg); };
  app_publishers_.path = [pub = pubPath](const nav_msgs::msg::Path &msg) { pub->publish(msg); };
  app_publishers_.laser_cloud_dynamic = [pub = pubLaserCloudDyn](const sensor_msgs::msg::PointCloud2 &msg) { pub->publish(msg); };
  app_publishers_.laser_cloud_dynamic_removed = [pub = pubLaserCloudDynRmed](const sensor_msgs::msg::PointCloud2 &msg) { pub->publish(msg); };
  app_publishers_.laser_cloud_dynamic_debug = [pub = pubLaserCloudDynDbg](const sensor_msgs::msg::PointCloud2 &msg) { pub->publish(msg); };
  app_publishers_.visual_patches_body = [pub = pubVisualPatchesBody](const sensor_msgs::msg::PointCloud2 &msg) { pub->publish(msg); };
  app_publishers_.image = [pub = pubImage](const sensor_msgs::msg::Image &msg) { pub->publish(msg); };
  app_publishers_.mavros_pose = [pub = mavros_pose_publisher](const geometry_msgs::msg::PoseStamped &msg) { pub->publish(msg); };
  app_publishers_.imu_prop_odom = [pub = pubImuPropOdom](const nav_msgs::msg::Odometry &msg) { pub->publish(msg); };

}

void LIVMapper::handleFirstFrame() 
{
  if (!is_first_frame)
  {
    _first_lidar_time = LidarMeasures.last_lio_update_time;
    p_imu->first_lidar_time = _first_lidar_time; // Only for IMU data log
    is_first_frame = true;
    spdlog::info("FIRST LIDAR FRAME!");
  }
}

void LIVMapper::gravityAlignment() {
  if (!p_imu->imu_need_init && !gravity_align_finished) {
    spdlog::info("Gravity Alignment Starts");
    V3D ez(0, 0, -1), gz(_state.gravity);
    Quaterniond G_q_I0 = Quaterniond::FromTwoVectors(gz, ez);
    M3D G_R_I0 = G_q_I0.toRotationMatrix();

    _state.pos_end = G_R_I0 * _state.pos_end;
    _state.rot_end = G_R_I0 * _state.rot_end;
    _state.vel_end = G_R_I0 * _state.vel_end;
    _state.gravity = G_R_I0 * _state.gravity;
    gravity_align_finished = true;
    spdlog::info("Gravity Alignment Finished");
  }
}

void LIVMapper::processImu() 
{
  // double t0 = fast_livo::utils::getWTime();

  p_imu->Process2(LidarMeasures, _state, feats_undistort);

  if (gravity_align_en) gravityAlignment();

  state_propagat = _state;
  voxelmap_manager->state_ = _state;
  voxelmap_manager->feats_undistort_ = feats_undistort;

  // double t_prop = fast_livo::utils::getWTime();

  // std::cout << "[ Mapping ] feats_undistort: " << feats_undistort->size() << std::endl;
  // std::cout << "[ Mapping ] predict cov: " << _state.cov.diagonal().transpose() << std::endl;
  // std::cout << "[ Mapping ] predict sta: " << state_propagat.pos_end.transpose() << state_propagat.vel_end.transpose() << std::endl;
}

void LIVMapper::stateEstimationAndMapping() 
{
  switch (LidarMeasures.lio_vio_flg) 
  {
    case VIO:
      handleVIO();
      break;
    case LIO:
    case LO:
      handleLIO();
      break;
  }
}

void LIVMapper::handleVIO() 
{
  const rclcpp::Time current_stamp = makeTimeFromSeconds(LidarMeasures.last_lio_update_time);
  euler_cur = RotMtoEuler(_state.rot_end);
#ifdef ENABLE_PERFORMANCE_TIMING
  fout_pre << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << std::endl;
#endif

  if (!pcl_w_wait_pub || pcl_w_wait_pub->empty())
  {
    spdlog::info("[ VIO ] No point!!!");
    return;
  }

  spdlog::debug("[ VIO ] Raw feature num: {:d}", pcl_w_wait_pub->points.size());

  if (fabs((LidarMeasures.last_lio_update_time - _first_lidar_time) -
           plot_time) < (frame_cnt / 2 * 0.1)) {
    vio_manager->plot_flag = true;
  } 
  else 
  {
    vio_manager->plot_flag = false;
  }

  vio_manager->processFrame(LidarMeasures.measures.back().img, _pv_list, voxelmap_manager->voxel_map_, LidarMeasures.last_lio_update_time - _first_lidar_time);

  if (imu_prop_enable) 
  {
    ekf_finish_once = true;
    latest_ekf_state = _state;
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    state_update_flg = true;
  }

  // int size_sub_map = vio_manager->visual_sub_map_cur.size();
  // visual_sub_map->reserve(size_sub_map);
  // for (int i = 0; i < size_sub_map; i++) 
  // {
  //   PointType temp_map;
  //   temp_map.x = vio_manager->visual_sub_map_cur[i]->pos_[0];
  //   temp_map.y = vio_manager->visual_sub_map_cur[i]->pos_[1];
  //   temp_map.z = vio_manager->visual_sub_map_cur[i]->pos_[2];
  //   temp_map.intensity = 0.;
  //   visual_sub_map->push_back(temp_map);
  // }

  publish_frame_world(pubLaserCloudFullRes, vio_manager, current_stamp);
  publish_img_rgb(pubImage, vio_manager, current_stamp);

  euler_cur = RotMtoEuler(_state.rot_end);
  fout_out << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << " " << feats_undistort->points.size() << std::endl;
}

void LIVMapper::handleLIO() 
{    
  const rclcpp::Time current_stamp = makeTimeFromSeconds(LidarMeasures.last_lio_update_time);
  euler_cur = RotMtoEuler(_state.rot_end);
  fout_pre << setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
           << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
           << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << endl;

  // [@sashactpflya] Changed: The order of the following two checks is swapped to
  // avoid potential dereferencing of a null pointer.
  if (!feats_undistort || feats_undistort->empty()) {
    spdlog::info("[ LIO ]: No point!!!");
    return;
  }

  double t0 = fast_livo::utils::getWTime();

  downSizeFilterSurf.setInputCloud(feats_undistort);
  downSizeFilterSurf.filter(*feats_down_body);
  
  double t_down = fast_livo::utils::getWTime();

  feats_down_size = feats_down_body->points.size();
  voxelmap_manager->feats_down_body_ = feats_down_body;
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, feats_down_world);
  voxelmap_manager->feats_down_world_ = feats_down_world;
  voxelmap_manager->feats_down_size_ = feats_down_size;
  
  if (!lidar_map_inited) 
  {
    lidar_map_inited = true;
    voxelmap_manager->BuildVoxelMap();
  }

  double t1 = fast_livo::utils::getWTime();

  voxelmap_manager->StateEstimation(state_propagat);
  _state = voxelmap_manager->state_;
  _pv_list = voxelmap_manager->pv_list_;

  double t2 = fast_livo::utils::getWTime();

  if (imu_prop_enable) 
  {
    ekf_finish_once = true;
    latest_ekf_state = _state;
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    state_update_flg = true;
  }

  if (pose_output_en) {
    static std::ofstream evoFile;
    static bool pos_opened = false;
    if (!pos_opened) {
      evoFile.open(std::string(ROOT_DIR) + "Log/result/" + seq_name + ".txt",
                   std::ios::out);
      pos_opened = true;
      if (!evoFile.is_open())
        RCLCPP_ERROR(node_->get_logger(), "open fail\n");
      evoFile << std::fixed;
    }
    Eigen::Quaterniond q(_state.rot_end);
    evoFile << std::fixed;
    evoFile << LidarMeasures.last_lio_update_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
  }
  
  euler_cur = RotMtoEuler(_state.rot_end);
  tf2::Quaternion quat;
  quat.setRPY(euler_cur(0), euler_cur(1), euler_cur(2));
  geoQuat = fast_livo::utils::toMsg(quat);
  publish_odometry(pubOdomAftMapped, current_stamp);

  double t3 = fast_livo::utils::getWTime();

  PointCloudXYZI::Ptr world_lidar(new PointCloudXYZI());
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, world_lidar);
  for (size_t i = 0; i < world_lidar->points.size(); i++) 
  {
    voxelmap_manager->pv_list_[i].point_w << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;
    M3D point_crossmat = voxelmap_manager->cross_mat_list_[i];
    M3D var = voxelmap_manager->body_cov_list_[i];
    var = (_state.rot_end * extR) * var * (_state.rot_end * extR).transpose() +
          (-point_crossmat) * _state.cov.block<3, 3>(0, 0) * (-point_crossmat).transpose() + _state.cov.block<3, 3>(3, 3);
    voxelmap_manager->pv_list_[i].var = var;
  }
  voxelmap_manager->UpdateVoxelMap(voxelmap_manager->pv_list_);
  spdlog::debug("[ LIO ] Update Voxel Map");
  _pv_list = voxelmap_manager->pv_list_;
  
  double t4 = fast_livo::utils::getWTime();

  if(voxelmap_manager->config_setting_.map_sliding_en)
  {
    voxelmap_manager->mapSliding();
  }
  
  PointCloudXYZI::Ptr laserCloudFullRes(dense_map_en ? feats_undistort : feats_down_body);
  int size = laserCloudFullRes->points.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

  for (int i = 0; i < size; i++) 
  {
    RGBpointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
  }
  *pcl_w_wait_pub = *laserCloudWorld;

  if (!img_en)
    publish_frame_world(pubLaserCloudFullRes, vio_manager, current_stamp);
  if (pub_effect_point_en)
    publish_effect_world(pubLaserCloudEffect, voxelmap_manager->ptpl_list_,
                         current_stamp);
  if (voxelmap_manager->config_setting_.is_pub_plane_map_)
    voxelmap_manager->pubVoxelMap();
  publish_path(current_stamp);
  publish_mavros(mavros_pose_publisher, current_stamp);

  frame_num++;
  aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t4 - t0) / frame_num;

  // aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + (t2 - t1) / frame_num;
  // aver_time_map_inre = aver_time_map_inre * (frame_num - 1) / frame_num + (t4 - t3) / frame_num;
  // aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + (solve_time) / frame_num;
  // aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1) / frame_num + solve_const_H_time / frame_num;
  // printf("[ mapping time ]: per scan: propagation %0.6f downsample: %0.6f match: %0.6f solve: %0.6f  ICP: %0.6f  map incre: %0.6f total: %0.6f \n"
  //         "[ mapping time ]: average: icp: %0.6f construct H: %0.6f, total: %0.6f \n",
  //         t_prop - t0, t1 - t_prop, match_time, solve_time, t3 - t1, t5 - t3, t5 - t0, aver_time_icp, aver_time_const_H_time, aver_time_consu);

  // printf("\033[1;36m[ LIO mapping time ]: current scan: icp: %0.6f secs, map
  // incre: %0.6f secs, total: %0.6f secs.\033[0m\n"
  //         "\033[1;36m[ LIO mapping time ]: average: icp: %0.6f secs, map
  //         incre: %0.6f secs, total: %0.6f secs.\033[0m\n", t2 - t1, t4 - t3,
  //         t4 - t0, aver_time_icp, aver_time_map_inre, aver_time_consu);
#ifdef ENABLE_PERFORMANCE_TIMING
  printf("\033[1;34m+----------------------------------------------------------"
         "---+\033[0m\n");
  printf("\033[1;34m|                         LIO Mapping Time                 "
         "   |\033[0m\n");
  printf("\033[1;34m+----------------------------------------------------------"
         "---+\033[0m\n");
  printf("\033[1;34m| %-29s | %-27s |\033[0m\n", "Algorithm Stage",
         "Time (secs)");
  printf("\033[1;34m+----------------------------------------------------------"
         "---+\033[0m\n");
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "DownSample", t_down - t0);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "ICP", t2 - t1);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "updateVoxelMap", t4 - t3);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Current Total Time", t4 - t0);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Average Total Time", aver_time_consu);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
#endif

  euler_cur = RotMtoEuler(_state.rot_end);
  fout_out << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << " " << feats_undistort->points.size() << std::endl;
}

void LIVMapper::savePCD() {
  const bool save_colorized = img_en && colorize_map_en;
  const std::string pcd_suffix = save_colorized ? "_color" : "";
  const bool has_colorized_points =
      save_colorized && (pcl_wait_save->points.size() > 0);
  const bool has_intensity_points =
      !save_colorized && (pcl_wait_save_intensity->points.size() > 0);
  if (pcd_save_en && (has_colorized_points || has_intensity_points) &&
      pcd_save_interval < 0) {
    
    spdlog::info("Saving PCD files...");

    std::string raw_points_dir =
        std::string(ROOT_DIR) + "Log/PCD/all_raw_points" + pcd_suffix + ".pcd";
    std::string downsampled_points_dir =
        std::string(ROOT_DIR) + "Log/PCD/all_downsampled_points" + pcd_suffix +
        ".pcd";
    pcl::PCDWriter pcd_writer;

    if (save_colorized) {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(
          new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
      voxel_filter.setInputCloud(pcl_wait_save);
      voxel_filter.setLeafSize(filter_size_pcd, filter_size_pcd, filter_size_pcd);
      voxel_filter.filter(*downsampled_cloud);
  
      pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save); // Save the raw point cloud data
      std::cout << GREEN << "Raw point cloud data saved to: " << raw_points_dir 
                << " with point count: " << pcl_wait_save->points.size() << RESET << std::endl;
      
      pcd_writer.writeBinary(downsampled_points_dir, *downsampled_cloud); // Save the downsampled point cloud data
      std::cout << GREEN << "Downsampled point cloud data saved to: " << downsampled_points_dir 
                << " with point count after filtering: " << downsampled_cloud->points.size() << RESET << std::endl;

      if (save_dense_map_en) {
        pcd_writer.writeBinary(
            raw_points_dir,
            *pcl_wait_save); // Save the raw point cloud data
        spdlog::info("Raw point cloud data saved to: {} with point count: {}",
                     raw_points_dir, pcl_wait_save->points.size());
        spdlog::info(
            "{}[PCD] Raw colorized map export completed successfully.{}",
            GREEN, RESET);
      } else {
        spdlog::info("Skipping raw colorized map export (pcd_save.save_dense_map=false).");
      }

      pcd_writer.writeBinary(
          downsampled_points_dir,
          *downsampled_cloud); // Save the downsampled point cloud data
      spdlog::info("Downsampled point cloud data saved to: {} with point count after filtering: {}",
                   downsampled_points_dir, downsampled_cloud->points.size());
      spdlog::info(
          "{}[PCD] Downsampled colorized map export completed successfully.{}",
          GREEN, RESET);

      if (colmap_output_en) {
        fout_points << "# 3D point list with one line of data per point\n";
        fout_points << "#  POINT_ID, X, Y, Z, R, G, B, ERROR\n";
        for (size_t i = 0; i < downsampled_cloud->size(); ++i) 
        {
            const auto& point = downsampled_cloud->points[i];
            fout_points << i << " "
                        << std::fixed << std::setprecision(6)
                        << point.x << " " << point.y << " " << point.z << " "
                        << static_cast<int>(point.r) << " "
                        << static_cast<int>(point.g) << " "
                        << static_cast<int>(point.b) << " "
                        << 0 << std::endl;
        }
      }
    } else {
      PointCloudXYZI::Ptr downsampled_cloud(new PointCloudXYZI);
      pcl::VoxelGrid<PointType> voxel_filter;
      voxel_filter.setInputCloud(pcl_wait_save_intensity);
      voxel_filter.setLeafSize(filter_size_pcd, filter_size_pcd,
                               filter_size_pcd);
      voxel_filter.filter(*downsampled_cloud);

      if (save_dense_map_en) {
        pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save_intensity);
        spdlog::info("Raw point cloud data saved to: {} with point count: {}",
                     raw_points_dir, pcl_wait_save_intensity->points.size());
        spdlog::info("{}[PCD] Intensity map export completed successfully.{}",
                     GREEN, RESET);
      } else {
        spdlog::info(
            "Skipping raw intensity map export (pcd_save.save_dense_map=false).");
      }

      pcd_writer.writeBinary(downsampled_points_dir, *downsampled_cloud);
      spdlog::info("Downsampled point cloud data saved to: {} with point count after filtering: {}",
                   downsampled_points_dir, downsampled_cloud->points.size());
      spdlog::info(
          "{}[PCD] Downsampled intensity map export completed successfully.{}",
          GREEN, RESET);
    }
  }
}

void LIVMapper::run()
{
//   // Spin callbacks in a separate thread to avoid executor overhead in main loop
//   spin_thread_ = std::thread([this]() {
//     rclcpp::spin(node_);
//   });

//   rclcpp::Rate rate(5000);

//   while (rclcpp::ok()) {
//     if (!sync_packages(LidarMeasures)) {
//       rate.sleep();
//       continue;
//     }

//     handleFirstFrame();
//     processImu();
//     stateEstimationAndMapping();
//   }

//   if (spin_thread_.joinable()) {
//     spin_thread_.join();
//   }
//   savePCD();

  // Spin callbacks in a separate thread to avoid executor overhead in main loop
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node_);
  // WallRate avoids throwing if the ROS context shuts down while sleeping
  rclcpp::WallRate rate(5000);

  while (rclcpp::ok()) {
    executor.spin_some(std::chrono::milliseconds(0));

    if (!sync_packages(LidarMeasures)) {
      if (!rclcpp::ok()) {
        break;
      }
      rate.sleep();
      continue;
    }
    handleFirstFrame();
    processImu();
    stateEstimationAndMapping();
  }
  savePCD();
}

void LIVMapper::prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr, V3D angvel_avr)
{
  double mean_acc_norm = p_imu->IMU_mean_acc_norm;
  acc_avr = acc_avr * G_m_s2 / mean_acc_norm - imu_prop_state.bias_a;
  angvel_avr -= imu_prop_state.bias_g;

  M3D Exp_f = Exp(angvel_avr, dt);
  /* propogation of IMU attitude */
  imu_prop_state.rot_end = imu_prop_state.rot_end * Exp_f;

  /* Specific acceleration (global frame) of IMU */
  V3D acc_imu = imu_prop_state.rot_end * acc_avr + V3D(imu_prop_state.gravity[0], imu_prop_state.gravity[1], imu_prop_state.gravity[2]);

  /* propogation of IMU */
  imu_prop_state.pos_end = imu_prop_state.pos_end + imu_prop_state.vel_end * dt + 0.5 * acc_imu * dt * dt;

  /* velocity of IMU */
  imu_prop_state.vel_end = imu_prop_state.vel_end + acc_imu * dt;
}

void LIVMapper::imu_prop_callback()
{
  if (p_imu->imu_need_init || !new_imu || !ekf_finish_once) { return; }
  mtx_buffer_imu_prop.lock();
  new_imu = false; // 控制propagate频率和IMU频率一致
  if (imu_prop_enable && !prop_imu_buffer.empty())
  {
    static double last_t_from_lidar_end_time = 0;
    if (state_update_flg)
    {
      imu_propagate = latest_ekf_state;
      // drop all useless imu pkg
      while ((!prop_imu_buffer.empty() &&
              rclcpp::Time(prop_imu_buffer.front().header.stamp).seconds() < latest_ekf_time))
      {
        prop_imu_buffer.pop_front();
      }
      last_t_from_lidar_end_time = 0;
      for (int i = 0; i < prop_imu_buffer.size(); i++)
      {
        const double imu_time = rclcpp::Time(prop_imu_buffer[i].header.stamp).seconds();
        double t_from_lidar_end_time = imu_time - latest_ekf_time;
        double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
        // cout << "prop dt" << dt << ", " << t_from_lidar_end_time << ", " << last_t_from_lidar_end_time << endl;
        V3D acc_imu(prop_imu_buffer[i].linear_acceleration.x, prop_imu_buffer[i].linear_acceleration.y, prop_imu_buffer[i].linear_acceleration.z);
        V3D omg_imu(prop_imu_buffer[i].angular_velocity.x, prop_imu_buffer[i].angular_velocity.y, prop_imu_buffer[i].angular_velocity.z);
        prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
        last_t_from_lidar_end_time = t_from_lidar_end_time;
      }
      state_update_flg = false;
    }
    else
      {
        V3D acc_imu(newest_imu.linear_acceleration.x, newest_imu.linear_acceleration.y, newest_imu.linear_acceleration.z);
        V3D omg_imu(newest_imu.angular_velocity.x, newest_imu.angular_velocity.y, newest_imu.angular_velocity.z);
        const double newest_imu_time = rclcpp::Time(newest_imu.header.stamp).seconds();
        double t_from_lidar_end_time = newest_imu_time - latest_ekf_time;
        double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
        prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
        last_t_from_lidar_end_time = t_from_lidar_end_time;
      }

    V3D posi, vel_i;
    Eigen::Quaterniond q;
    posi = imu_propagate.pos_end;
    vel_i = imu_propagate.vel_end;
    q = Eigen::Quaterniond(imu_propagate.rot_end);
  imu_prop_odom.header.frame_id = "init_pose";
    imu_prop_odom.header.stamp = newest_imu.header.stamp;
    imu_prop_odom.pose.pose.position.x = posi.x();
    imu_prop_odom.pose.pose.position.y = posi.y();
    imu_prop_odom.pose.pose.position.z = posi.z();
    imu_prop_odom.pose.pose.orientation.w = q.w();
    imu_prop_odom.pose.pose.orientation.x = q.x();
    imu_prop_odom.pose.pose.orientation.y = q.y();
    imu_prop_odom.pose.pose.orientation.z = q.z();
    imu_prop_odom.twist.twist.linear.x = vel_i.x();
    imu_prop_odom.twist.twist.linear.y = vel_i.y();
    imu_prop_odom.twist.twist.linear.z = vel_i.z();
    if (app_publishers_.imu_prop_odom) app_publishers_.imu_prop_odom(imu_prop_odom);
  }
  mtx_buffer_imu_prop.unlock();
}

void LIVMapper::transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud)
{
  PointCloudXYZI().swap(*trans_cloud);
  trans_cloud->reserve(input_cloud->size());
  for (size_t i = 0; i < input_cloud->size(); i++)
  {
    pcl::PointXYZINormal p_c = input_cloud->points[i];
    Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
    p = (rot * (extR * p + extT) + t);
    PointType pi;
    pi.x = p(0);
    pi.y = p(1);
    pi.z = p(2);
    pi.intensity = p_c.intensity;
    trans_cloud->points.push_back(pi);
  }
}

void LIVMapper::pointBodyToWorld(const PointType &pi, PointType &po)
{
  V3D p_body(pi.x, pi.y, pi.z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po.x = p_global(0);
  po.y = p_global(1);
  po.z = p_global(2);
  po.intensity = pi.intensity;
}

template <typename T> void LIVMapper::pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi, Eigen::Matrix<T, 3, 1> &po)
{
  V3D p_body(pi[0], pi[1], pi[2]);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po[0] = p_global(0);
  po[1] = p_global(1);
  po[2] = p_global(2);
}

template <typename T> Eigen::Matrix<T, 3, 1> LIVMapper::pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi)
{
  V3D p(pi[0], pi[1], pi[2]);
  p = (_state.rot_end * (extR * p + extT) + _state.pos_end);
  Eigen::Matrix<T, 3, 1> po(p[0], p[1], p[2]);
  return po;
}

void LIVMapper::RGBpointBodyToWorld(PointType const *const pi, PointType *const po)
{
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

void LIVMapper::standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
{
  if (!lidar_en) return;
  mtx_buffer.lock();

  rclcpp::Time adjusted_stamp = msg->header.stamp + rclcpp::Duration::from_seconds(lidar_time_offset);
  // cout<<"got feature"<<endl;
  if (last_timestamp_lidar_ && adjusted_stamp < *last_timestamp_lidar_)
  {
    RCLCPP_ERROR(node_->get_logger(), "lidar loop back, clear buffer");
    lid_raw_data_buffer.clear();
  }
  // ROS_INFO("get point cloud at time: %.6f", msg->header.stamp.toSec());
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);
  lid_raw_data_buffer.push_back(ptr);
  lid_header_time_buffer.push_back(adjusted_stamp.seconds());
  last_timestamp_lidar_ = adjusted_stamp;

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

// void LIVMapper::livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstSharedPtr &msg_in)
// {
//   if (!lidar_en) return;
//   mtx_buffer.lock();
//   livox_ros_driver::CustomMsg::Ptr msg(new livox_ros_driver::CustomMsg(*msg_in));
//   // if ((abs(msg->header.stamp.toSec() - last_timestamp_lidar) > 0.2 && last_timestamp_lidar > 0) || sync_jump_flag)
//   // {
//   //   ROS_WARN("lidar jumps %.3f\n", msg->header.stamp.toSec() - last_timestamp_lidar);
//   //   sync_jump_flag = true;
//   //   msg->header.stamp = ros::Time().fromSec(last_timestamp_lidar + 0.1);
//   // }
//   if (abs(last_timestamp_imu - msg->header.stamp.toSec()) > 1.0 && !imu_buffer.empty())
//   {
//     double timediff_imu_wrt_lidar = last_timestamp_imu - msg->header.stamp.toSec();
//     printf("\033[95mSelf sync IMU and LiDAR, HARD time lag is %.10lf \n\033[0m", timediff_imu_wrt_lidar - 0.100);
//     // imu_time_offset = timediff_imu_wrt_lidar;
//   }

//   double cur_head_time = msg->header.stamp.toSec();
//   ROS_INFO("Get LiDAR, its header time: %.6f", cur_head_time);
//   if (cur_head_time < last_timestamp_lidar)
//   {
//     ROS_ERROR("lidar loop back, clear buffer");
//     lid_raw_data_buffer.clear();
//   }
//   // ROS_INFO("get point cloud at time: %.6f", msg->header.stamp.toSec());
//   PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
//   p_pre->process(msg, ptr);

//   if (!ptr || ptr->empty()) {
//     ROS_ERROR("Received an empty point cloud");
//     mtx_buffer.unlock();
//     return;
//   }

//   lid_raw_data_buffer.push_back(ptr);
//   lid_header_time_buffer.push_back(cur_head_time);
//   last_timestamp_lidar = cur_head_time;

//   mtx_buffer.unlock();
//   sig_buffer.notify_all();
// }

void LIVMapper::imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr &msg_in)
{
  if (!imu_en) return;

  if (!last_timestamp_lidar_)
    return;
  sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));
 
  // Apply IMU time offset
  rclcpp::Time stamp = msg->header.stamp;
  stamp += rclcpp::Duration::from_seconds(imu_time_offset);

  // Check for large time difference (potential desync)
  double time_diff = (*last_timestamp_lidar_ - stamp).seconds();

  if (fabs(time_diff) > 0.5 && (!ros_driver_fix_en)) {
    spdlog::warn("IMU and LiDAR not synced! delta time: {:.4f}", time_diff);
  }

  // [@sashactpflya] Fixed: Improved ros_driver_fix to avoid drift
  // Only apply correction if ros_driver_fix is enabled AND difference is significant
    // Only fix if the drift is more than 0.1s (likely a driver bug)
if (ros_driver_fix_en && fabs(time_diff) > 0.1) {
    auto correction = rclcpp::Duration::from_seconds(std::round(time_diff));
    stamp += correction;
    spdlog::debug("Applied ros_driver_fix correction: {:.4f}s", correction.seconds());
}
  
  // Update message timestamp
  msg->header.stamp = stamp;

  mtx_buffer.lock();

  // Check for time going backwards
  if (last_timestamp_imu_ && *last_timestamp_imu_ > stamp) {
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    spdlog::error("IMU loop back, offset: {:.4f}", 
                  (*last_timestamp_imu_ - stamp).seconds());
    return;
  }

  // Check for unreasonable time jumps
  if (last_timestamp_imu_ && (stamp - *last_timestamp_imu_) > rclcpp::Duration::from_seconds(0.2)) {
    spdlog::warn("IMU timestamp jump: {:.4f}s", (stamp - *last_timestamp_imu_).seconds());
    // Don't reject, but warn
  }

  last_timestamp_imu_ = stamp;

  imu_buffer.push_back(msg);
  // cout<<"got imu: "<<timestamp<<" imu size "<<imu_buffer.size()<<endl;
  mtx_buffer.unlock();
  if (imu_prop_enable)
  {
    mtx_buffer_imu_prop.lock();
    if (imu_prop_enable && !p_imu->imu_need_init) { prop_imu_buffer.push_back(*msg); }
    newest_imu = *msg;
    new_imu = true;
    mtx_buffer_imu_prop.unlock();
  }
  sig_buffer.notify_all();
}

cv::Mat LIVMapper::getImageFromMsg(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
{
  cv::Mat img;
  img = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
  return img;
}

void LIVMapper::img_cbk(const sensor_msgs::msg::Image::ConstSharedPtr &msg_in)
{
  if (!img_en) return;
  sensor_msgs::msg::Image::SharedPtr msg = make_shared<sensor_msgs::msg::Image>(*msg_in);
  // if ((abs(msg->header.stamp.toSec() - last_timestamp_img) > 0.2 && last_timestamp_img > 0) || sync_jump_flag)
  // {
  //   ROS_WARN("img jumps %.3f\n", msg->header.stamp.toSec() - last_timestamp_img);
  //   sync_jump_flag = true;
  //   msg->header.stamp = ros::Time().fromSec(last_timestamp_img + 0.1);
  // }

  // Hiliti2022 40Hz
  if (hilti_en)
  {
    static int frame_counter = 0;
    if (++frame_counter % 4 != 0) return;
  }
  // double msg_header_time =  msg->header.stamp.toSec();
  double msg_header_time = rclcpp::Time(msg->header.stamp).seconds() + img_time_offset;
  if (abs(msg_header_time - last_timestamp_img) < 0.001)
    return;
  spdlog::debug("Get image, its header time: {:.6f}", msg_header_time);
  if (!last_timestamp_lidar_)
    return;

  if (msg_header_time < last_timestamp_img) {
    spdlog::error("image loop back. \n");
    return;
  }

  mtx_buffer.lock();

  double img_time_correct = msg_header_time; // last_timestamp_lidar + 0.105;

  if (img_time_correct - last_timestamp_img < 0.02)
  {
    spdlog::warn("Image need Jumps: {:.6f}", img_time_correct);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    return;
  }

  cv::Mat img_cur = getImageFromMsg(msg);
  img_buffer.push_back(img_cur);
  img_time_buffer.push_back(img_time_correct);

  // ROS_INFO("Correct Image time: %.6f", img_time_correct);

  last_timestamp_img = img_time_correct;
  // cv::imshow("img", img);
  // cv::waitKey(1);
  // cout<<"last_timestamp_img:::"<<last_timestamp_img<<endl;
  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

bool LIVMapper::sync_packages(LidarMeasureGroup &meas)
{
  if (lid_raw_data_buffer.empty() && lidar_en) return false;
  if (img_buffer.empty() && img_en) return false;
  if (imu_buffer.empty() && imu_en) return false;
  if (!last_timestamp_imu_ && !last_timestamp_lidar_) return false;

  switch (slam_mode_)
  {
  case ONLY_LIO:
  {
    if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer.front();
    if (!lidar_pushed)
    {
      // If not push the lidar into measurement data buffer
      meas.lidar = lid_raw_data_buffer.front(); // push the first lidar topic
      if (meas.lidar->points.size() <= 1) return false;

      meas.lidar_frame_beg_time = lid_header_time_buffer.front();                                                // generate lidar_frame_beg_time
      meas.lidar_frame_end_time = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000); // calc lidar scan end time
      meas.pcl_proc_cur = meas.lidar;
      lidar_pushed = true;                                                                                       // flag
    }

    if (imu_en && last_timestamp_imu_->seconds() < meas.lidar_frame_end_time)
    { // waiting imu message needs to be
      // larger than _lidar_frame_end_time,
      // make sure complete propagate.
      // ROS_ERROR("out sync");
      return false;
    }

    struct MeasureGroup m; // standard method to keep imu message.

    m.imu.clear();
    m.lio_time = meas.lidar_frame_end_time;
    mtx_buffer.lock();
    while (!imu_buffer.empty())
    {
      if (rclcpp::Time(imu_buffer.front()->header.stamp).seconds() > meas.lidar_frame_end_time) break;
      m.imu.push_back(imu_buffer.front());
      imu_buffer.pop_front();
    }
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();
    sig_buffer.notify_all();

    meas.lio_vio_flg = LIO; // process lidar topic, so timestamp should be lidar scan end.
    meas.measures.push_back(m);
    // ROS_INFO("ONlY HAS LiDAR and IMU, NO IMAGE!");
    lidar_pushed = false; // sync one whole lidar scan.
    return true;

    break;
  }

  case LIVO:
  {
    /*** For LIVO mode, the time of LIO update is set to be the same as VIO, LIO
     * first than VIO imediatly ***/
    EKF_STATE last_lio_vio_flg = meas.lio_vio_flg;
    double t0 = fast_livo::utils::getWTime();
    switch (last_lio_vio_flg)
    {
    // double img_capture_time = meas.lidar_frame_beg_time + exposure_time_init;
    case WAIT:
    case VIO:
    {
      // printf("!!! meas.lio_vio_flg: %d \n", meas.lio_vio_flg);
      double img_capture_time = img_time_buffer.front() + exposure_time_init;
      /*** has img topic, but img topic timestamp larger than lidar end time,
       * process lidar topic. After LIO update, the meas.lidar_frame_end_time
       * will be refresh. ***/
      if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer.front();
      // printf("[ Data Cut ] wait \n");
      // printf("[ Data Cut ] last_lio_update_time: %lf \n",
      // meas.last_lio_update_time);

      double lid_newest_time = lid_header_time_buffer.back() + lid_raw_data_buffer.back()->points.back().curvature / double(1000);
      double imu_newest_time = rclcpp::Time(imu_buffer.back()->header.stamp).seconds();

      if (img_capture_time < meas.last_lio_update_time + 0.00001)
      {
        img_buffer.pop_front();
        img_time_buffer.pop_front();
        spdlog::error("[ Data Cut ] Throw one image frame! \n");
        return false;
      }

      if (img_capture_time > lid_newest_time || img_capture_time > imu_newest_time)
      {
        // ROS_ERROR("lost first camera frame");
        // printf("img_capture_time, lid_newest_time, imu_newest_time: %lf , %lf
        // , %lf \n", img_capture_time, lid_newest_time, imu_newest_time);
        return false;
      }

      struct MeasureGroup m;

      // printf("[ Data Cut ] LIO \n");
      // printf("[ Data Cut ] img_capture_time: %lf \n", img_capture_time);
      m.imu.clear();
      m.lio_time = img_capture_time;
      mtx_buffer.lock();
      while (!imu_buffer.empty())
      {
        const double imu_front_time = rclcpp::Time(imu_buffer.front()->header.stamp).seconds();
        if (imu_front_time > m.lio_time) break;

        if (imu_front_time > meas.last_lio_update_time) m.imu.push_back(imu_buffer.front());

        imu_buffer.pop_front();
        // printf("[ Data Cut ] imu time: %lf \n",
        // imu_buffer.front()->header.stamp.toSec());
      }
      mtx_buffer.unlock();
      sig_buffer.notify_all();

      *(meas.pcl_proc_cur) = *(meas.pcl_proc_next);
      PointCloudXYZI().swap(*meas.pcl_proc_next);

      int lid_frame_num = lid_raw_data_buffer.size();
      int max_size = meas.pcl_proc_cur->size() + 24000 * lid_frame_num;
      meas.pcl_proc_cur->reserve(max_size);
      meas.pcl_proc_next->reserve(max_size);
      // deque<PointCloudXYZI::Ptr> lidar_buffer_tmp;

      while (!lid_raw_data_buffer.empty())
      {
        if (lid_header_time_buffer.front() > img_capture_time) break;
        auto pcl(lid_raw_data_buffer.front()->points);
        double frame_header_time(lid_header_time_buffer.front());
        float max_offs_time_ms = (m.lio_time - frame_header_time) * 1000.0f;

        for (int i = 0; i < pcl.size(); i++)
        {
          auto pt = pcl[i];
          if (pcl[i].curvature < max_offs_time_ms)
          {
            pt.curvature += (frame_header_time - meas.last_lio_update_time) * 1000.0f;
            meas.pcl_proc_cur->points.push_back(pt);
          }
          else
          {
            pt.curvature += (frame_header_time - m.lio_time) * 1000.0f;
            meas.pcl_proc_next->points.push_back(pt);
          }
        }
        lid_raw_data_buffer.pop_front();
        lid_header_time_buffer.pop_front();
      }

      meas.measures.push_back(m);
      meas.lio_vio_flg = LIO;
      // meas.last_lio_update_time = m.lio_time;
      // printf("!!! meas.lio_vio_flg: %d \n", meas.lio_vio_flg);
      // printf("[ Data Cut ] pcl_proc_cur number: %d \n", meas.pcl_proc_cur
      // ->points.size()); printf("[ Data Cut ] LIO process time: %lf \n",
      //   fast_livo::utils::getWTime() - t0);
      return true;
    }

    case LIO:
    {
      double img_capture_time = img_time_buffer.front() + exposure_time_init;
      meas.lio_vio_flg = VIO;
      // printf("[ Data Cut ] VIO \n");
      meas.measures.clear();
      double imu_time = rclcpp::Time(imu_buffer.front()->header.stamp).seconds();

      struct MeasureGroup m;
      m.vio_time = img_capture_time;
      m.lio_time = meas.last_lio_update_time;
      m.img = img_buffer.front();
      mtx_buffer.lock();
      // while ((!imu_buffer.empty() && (imu_time < img_capture_time)))
      // {
      //   imu_time = imu_buffer.front()->header.stamp.toSec();
      //   if (imu_time > img_capture_time) break;
      //   m.imu.push_back(imu_buffer.front());
      //   imu_buffer.pop_front();
      //   printf("[ Data Cut ] imu time: %lf \n",
      //   imu_buffer.front()->header.stamp.toSec());
      // }
      img_buffer.pop_front();
      img_time_buffer.pop_front();
      mtx_buffer.unlock();
      sig_buffer.notify_all();
      meas.measures.push_back(m);
      lidar_pushed = false; // after VIO update, the _lidar_frame_end_time will be refresh.
    //   printf("[ Data Cut ] VIO process time: %lf \n", fast_livo::utils::getWTime() - t0);
      return true;
    }

    default:
    {
      // printf("!! WRONG EKF STATE !!");
      return false;
    }
      // return false;
    }
    break;
  }

  case ONLY_LO:
  {
    if (!lidar_pushed) 
    { 
      // If not in lidar scan, need to generate new meas
      if (lid_raw_data_buffer.empty())  return false;
      meas.lidar = lid_raw_data_buffer.front(); // push the first lidar topic
      meas.lidar_frame_beg_time = lid_header_time_buffer.front(); // generate lidar_beg_time
      meas.lidar_frame_end_time  = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000); // calc lidar scan end time
      lidar_pushed = true;             
    }
    struct MeasureGroup m; // standard method to keep imu message.
    m.lio_time = meas.lidar_frame_end_time;
    mtx_buffer.lock();
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    lidar_pushed = false; // sync one whole lidar scan.
    meas.lio_vio_flg = LO; // process lidar topic, so timestamp should be lidar scan end.
    meas.measures.push_back(m);
    return true;
    break;
  }

  default:
  {
    printf("!! WRONG SLAM TYPE !!");
    return false;
  }
  }
  spdlog::error("out sync");
}

void LIVMapper::publish_img_rgb(const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &pubImage, VIOManagerPtr vio_manager, const rclcpp::Time &stamp)
{
  (void)pubImage;
  cv::Mat img_rgb = vio_manager->img_cp;
  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = stamp;
  out_msg.header.frame_id = "init_pose";
  out_msg.encoding = sensor_msgs::image_encodings::BGR8;
  out_msg.image = img_rgb;
  if (app_publishers_.image) app_publishers_.image(*out_msg.toImageMsg());
}

void LIVMapper::publish_frame_world(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        &pubLaserCloudFullRes,
    VIOManagerPtr vio_manager, const rclcpp::Time &stamp) {
  if (pcl_w_wait_pub->empty())
    return;
  const bool do_colorize = img_en && colorize_map_en;
  const std::string pcd_suffix = do_colorize ? "_color" : "";
  PointCloudXYZRGB::Ptr laserCloudWorldRGB(new PointCloudXYZRGB());
  if (do_colorize) {
    static int pub_num = 1;
    *pcl_wait_pub += *pcl_w_wait_pub;
    if(pub_num == pub_scan_num)
    {
      pub_num = 1;
      size_t size = pcl_wait_pub->points.size();
      laserCloudWorldRGB->reserve(size);
      // double inv_expo = _state.inv_expo_time;
      cv::Mat img_rgb = vio_manager->img_rgb;
      for (size_t i = 0; i < size; i++)
      {
        PointTypeRGB pointRGB;
        pointRGB.x = pcl_wait_pub->points[i].x;
        pointRGB.y = pcl_wait_pub->points[i].y;
        pointRGB.z = pcl_wait_pub->points[i].z;

        V3D p_w(pcl_wait_pub->points[i].x, pcl_wait_pub->points[i].y, pcl_wait_pub->points[i].z);
        V3D pf(vio_manager->new_frame_->w2f(p_w)); if (pf[2] < 0) continue;
        V2D pc(vio_manager->new_frame_->w2c(p_w));

        if (vio_manager->new_frame_->cam_->isInFrame(pc.cast<int>(), 3)) // 100
        {
          V3F pixel = vio_manager->getInterpolatedPixel(img_rgb, pc);
          pointRGB.r = pixel[2];
          pointRGB.g = pixel[1];
          pointRGB.b = pixel[0];
          // pointRGB.r = pixel[2] * inv_expo; pointRGB.g = pixel[1] * inv_expo; pointRGB.b = pixel[0] * inv_expo;
          // if (pointRGB.r > 255) pointRGB.r = 255;
          // else if (pointRGB.r < 0) pointRGB.r = 0;
          // if (pointRGB.g > 255) pointRGB.g = 255;
          // else if (pointRGB.g < 0) pointRGB.g = 0;
          // if (pointRGB.b > 255) pointRGB.b = 255;
          // else if (pointRGB.b < 0) pointRGB.b = 0;
          if (pf.norm() > blind_rgb_points) laserCloudWorldRGB->push_back(pointRGB);
        }
      }
    }
    else
    {
      pub_num++;
    }
  }

  /*** Publish Frame ***/
  sensor_msgs::msg::PointCloud2 laserCloudmsg;
  if (do_colorize) {
    // cout << "RGB pointcloud size: " << laserCloudWorldRGB->size() << endl;
    ros_pcl::toROSMsg(*laserCloudWorldRGB, laserCloudmsg);
  }
  else 
  { 
    ros_pcl::toROSMsg(*pcl_w_wait_pub, laserCloudmsg); 
  }
  laserCloudmsg.header.stamp = stamp;
  laserCloudmsg.header.frame_id = "init_pose";
  (void)pubLaserCloudFullRes;
  if (app_publishers_.laser_cloud_full_res) app_publishers_.laser_cloud_full_res(laserCloudmsg);

  /**************** save map ****************/
  /* 1. make sure you have enough memories
  /* 2. noted that pcd save will influence the real-time performences **/
  if (pcd_save_en)
  {
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));
    static int scan_wait_num = 0;

    if (do_colorize) {
      *pcl_wait_save += *laserCloudWorldRGB;
    }
    else
    {
      *pcl_wait_save_intensity += *pcl_w_wait_pub;
    }
    scan_wait_num++;

    if ((pcl_wait_save->size() > 0 || pcl_wait_save_intensity->size() > 0) && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval)
    {
      pcd_index++;
      string all_points_dir(string(string(ROOT_DIR) + "Log/PCD/") +
                            to_string(pcd_index) + pcd_suffix + string(".pcd"));
      pcl::PCDWriter pcd_writer;
      if (pcd_save_en) {
        spdlog::info("Current scan saved to /PCD/{}", all_points_dir);
        if (do_colorize) {
          pcd_writer.writeBinary(all_points_dir,
                                 *pcl_wait_save); // pcl::io::savePCDFileASCII(all_points_dir,
                                                  // *pcl_wait_save);
          PointCloudXYZRGB().swap(*pcl_wait_save);
        }
        else
        {
          pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_intensity);
          PointCloudXYZI().swap(*pcl_wait_save_intensity);
        }        
        Eigen::Quaterniond q(_state.rot_end);
        fout_pcd_pos << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " " << q.w() << " " << q.x() << " " << q.y()
                     << " " << q.z() << " " << endl;
        scan_wait_num = 0;
      }
    }
  }
  if (do_colorize && laserCloudWorldRGB->size() > 0)
    PointCloudXYZI().swap(*pcl_wait_pub);
  PointCloudXYZI().swap(*pcl_w_wait_pub);
}

void LIVMapper::publish_visual_sub_map(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubSubVisualMap, const rclcpp::Time &stamp)
{
  (void)pubSubVisualMap;
  PointCloudXYZI::Ptr laserCloudFullRes(visual_sub_map);
  int size = laserCloudFullRes->points.size(); if (size == 0) return;
  PointCloudXYZI::Ptr sub_pcl_visual_map_pub(new PointCloudXYZI());
  *sub_pcl_visual_map_pub = *laserCloudFullRes;
  if (1)
  {
    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    ros_pcl::toROSMsg(*sub_pcl_visual_map_pub, laserCloudmsg);
    laserCloudmsg.header.stamp = stamp;
    laserCloudmsg.header.frame_id = "init_pose";
    if (app_publishers_.sub_visual_map) app_publishers_.sub_visual_map(laserCloudmsg);
  }
}

void LIVMapper::publish_effect_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudEffect,
                                     const std::vector<PointToPlane> &ptpl_list, const rclcpp::Time &stamp)
{
  int effect_feat_num = ptpl_list.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(effect_feat_num, 1));
  for (int i = 0; i < effect_feat_num; i++)
  {
    laserCloudWorld->points[i].x = ptpl_list[i].point_w_[0];
    laserCloudWorld->points[i].y = ptpl_list[i].point_w_[1];
    laserCloudWorld->points[i].z = ptpl_list[i].point_w_[2];
  }
  sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
  ros_pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
  laserCloudFullRes3.header.stamp = stamp;
  laserCloudFullRes3.header.frame_id = "init_pose";
  (void)pubLaserCloudEffect;
  if (app_publishers_.laser_cloud_effect) app_publishers_.laser_cloud_effect(laserCloudFullRes3);
}

template <typename T> void LIVMapper::set_posestamp(T &out)
{
  out.position.x = _state.pos_end(0);
  out.position.y = _state.pos_end(1);
  out.position.z = _state.pos_end(2);
  out.orientation.x = geoQuat.x;
  out.orientation.y = geoQuat.y;
  out.orientation.z = geoQuat.z;
  out.orientation.w = geoQuat.w;
}

void LIVMapper::publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &pubOdomAftMapped, const rclcpp::Time &stamp)
{
  (void)pubOdomAftMapped;
  odomAftMapped.header.frame_id = "init_pose";
  odomAftMapped.child_frame_id = "body";
  odomAftMapped.header.stamp = stamp;
  set_posestamp(odomAftMapped.pose.pose);

  tf2::Transform transform;
  tf2::Quaternion q;
  transform.setOrigin(tf2::Vector3(_state.pos_end(0), _state.pos_end(1), _state.pos_end(2)));
  q.setW(geoQuat.w);
  q.setX(geoQuat.x);
  q.setY(geoQuat.y);
  q.setZ(geoQuat.z);
  transform.setRotation(q);

  // init_pose --> body; store for zero-order hold publisher
  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg.transform = fast_livo::utils::toMsg(transform);
  latest_tf_transform_ = transform_msg.transform;
  latest_tf_time_ = stamp;
  latest_tf_wall_time_ = last_timestamp_imu_.value_or(stamp);
  latest_tf_valid_ = true;

  if (app_publishers_.odom_aft_mapped) app_publishers_.odom_aft_mapped(odomAftMapped);
}

void LIVMapper::publish_mavros(const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr &mavros_pose_publisher, const rclcpp::Time &stamp)
{
  (void)mavros_pose_publisher;
  msg_body_pose.header.stamp = stamp;
  msg_body_pose.header.frame_id = "init_pose";
  set_posestamp(msg_body_pose.pose);
  if (app_publishers_.mavros_pose) app_publishers_.mavros_pose(msg_body_pose);
}

void LIVMapper::publish_path(const rclcpp::Time &stamp) {
  set_posestamp(msg_body_pose.pose);
  msg_body_pose.header.stamp = stamp;
  msg_body_pose.header.frame_id = "init_pose";
  path.header.stamp = stamp;
  path.poses.push_back(msg_body_pose);
  if (app_publishers_.path)
    app_publishers_.path(path);
  else
    spdlog::warn("Path publisher is nullptr.");
}

void LIVMapper::publishStaticTf() {
  if (!app_publishers_.static_tfs || !last_timestamp_imu_)
    return;
  const auto stamp = *last_timestamp_imu_; // Used as "now"
  std::vector<geometry_msgs::msg::TransformStamped> tfs;

  // The TFs are publlished as:
  // - Parent, Child along the the transform that maps Child to Parent
  tfs.push_back(fast_livo::utils::toMsg(TF_lidar_body_.inverse(), stamp, "body", lidar_frame_id_));

  if (has_vio_tf_)
  {
    tfs.push_back(fast_livo::utils::toMsg(TF_vio_body_.inverse(), stamp, "body", "vio"));
  }
  if (has_cam_tf_)
  {
    tfs.push_back(fast_livo::utils::toMsg(TF_cam_vio_.inverse(), stamp, "vio", "camera"));
  }
  
  if ( has_vio_tf_ && has_cam_tf_ )
  {
      tfs.push_back(fast_livo::utils::toMsg(TF_cam_body_.inverse(), stamp, "body", "camera_from_body"));
  }
  tfs.push_back(fast_livo::utils::toMsg(TF_cam_lidar_.inverse(), stamp, lidar_frame_id_, "camera_from_lidar"));


  app_publishers_.static_tfs(tfs);
}

void LIVMapper::publish_tf_hold() {
  if (!latest_tf_valid_ || !tf_broadcaster_ || !last_timestamp_imu_)
    return;

  // Use zero-order hold from last odometry stamp, advancing by elapsed wall time
  const int64_t delta_ns = last_timestamp_imu_->nanoseconds() - latest_tf_wall_time_.nanoseconds();
  if (delta_ns < 0) return; // avoid publishing with negative offset
  rclcpp::Time stamped_time(latest_tf_time_.nanoseconds() + delta_ns);

  // init_pose -> body (latest held)
  geometry_msgs::msg::TransformStamped transform_cam_to_aft;
  transform_cam_to_aft.header.stamp = stamped_time;
  transform_cam_to_aft.header.frame_id = "init_pose";
  transform_cam_to_aft.child_frame_id = "body";
  transform_cam_to_aft.transform = latest_tf_transform_;
  tf_broadcaster_->sendTransform(transform_cam_to_aft);
}

rclcpp::Time LIVMapper::makeTimeFromSeconds(double seconds) const
{
  if (std::isfinite(seconds) && seconds >= 0.0)
  {
    return rclcpp::Time(static_cast<int64_t>(seconds * 1e9));
  }
  return node_->get_clock()->now();
}

void LIVMapper::setAppPublishers(AppPublishers publishers)
{
  app_publishers_ = std::move(publishers);
}

void LIVMapper::resetRosInterfaces()
{
    // Reset all ROS subscribers and publishers to nullptr
    pubLaserCloudFullRes = nullptr;
    pubOdomAftMapped = nullptr;
    pubPath = nullptr;
    pubSubVisualMap = nullptr;
    pubVisualPatchesBody = nullptr;
    pubLaserCloudEffect = nullptr;
    mavros_pose_publisher = nullptr;
    pubImage = nullptr;
    sub_pcl = nullptr;
    sub_imu = nullptr;
    sub_img = nullptr;
    
    // Reset timer callbacks
    imu_prop_timer = nullptr;
}

} // namespace fast_livo
