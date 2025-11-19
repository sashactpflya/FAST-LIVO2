# FAST-LIVO2 Published Topics

| Topic | Message type | Avg. rate (approx) | Notes |
| --- | --- | --- | --- |
| `/livo2/cloud_registered` | `sensor_msgs/msg/PointCloud2` | Matches LiDAR update rate (typically 10–20 Hz depending on the sensor) | Registered LiDAR scan expressed in `camera_init`. Contains either the downsampled cloud alone or the colorized cloud if image coloring is enabled. |
| `/livo2/cloud_effected` *(optional)* | `sensor_msgs/msg/PointCloud2` | LiDAR rate when `publish.pub_effect_point_en` is true | Debug cloud built from the matched lidar–plane features stored in `voxelmap_manager->ptpl_list_`. Disabled by default. |
| `/livo2/cloud_visual_sub_map_before` *(unused)* | `sensor_msgs/msg/PointCloud2` | Not emitted in current code | Placeholder publisher for a visual sub-map cloud. The helper exists but is never invoked, so no messages are sent. |
| `/livo2/aft_mapped_to_init` | `nav_msgs/msg/Odometry` | LiDAR update rate | Full LIO pose (`aft_mapped` in FAST-LIO naming) relative to the `camera_init` origin. |
| `/livo2/path` | `nav_msgs/msg/Path` | LiDAR update rate | Accumulated trajectory using the same pose samples published on `/livo2/aft_mapped_to_init`. |
| `/livo2/mavros/vision_pose/pose` | `geometry_msgs/msg/PoseStamped` | LiDAR update rate | Pose converted for MAVROS/FCU consumption; identical data to the odometry topic but in `PoseStamped` form. |
| `/livo2/rgb_img` | `sensor_msgs/msg/Image` | Matches camera frame rate (e.g., 20–30 Hz depending on dataset) | Latest left-camera image forwarded after being copied inside VIO. Used mainly for visualization/debugging. |
| `/livo2/imu_propagate` | `nav_msgs/msg/Odometry` | Timer-driven at 250 Hz (4 ms wall timer) | High-rate IMU propagation result published even between LiDAR updates for downstream consumers that need a smooth pose stream. |
| `/livo2/planes` *(optional)* | `visualization_msgs/msg/MarkerArray` | LiDAR update rate when `voxel_map.is_pub_plane_map` is true | Marker array produced by `voxelmap_manager->pubVoxelMap()` that visualizes plane primitives extracted from the voxel map. Disabled unless explicitly requested. |

### Notable non-emitting publishers

- `/livo2/Laser_map`, `/livo2/dyn_obj`, `/livo2/dyn_obj_removed`, `/livo2/dyn_obj_dbg_hist`, `/livo2/planner_normal`, `/livo2/voxels`: publishers are instantiated in `LIVMapper` but no code currently calls `publish()`, so these topics stay idle unless additional features are enabled downstream.

### TF streams

FAST-LIVO2 also broadcasts TF frames (`camera_init → aft_mapped` and a static `aft_mapped → PandarXT-32` transform) via `tf2_ros::TransformBroadcaster`. Since TF uses its own transport, it is not listed in the ROS topic table above.
