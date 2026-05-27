#include "plan_env/grid_map.h"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 
#include <tf2/exceptions.h>

#include <stdexcept>

#include <sensor_msgs/image_encodings.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

// #define current_img_ md_.depth_image_[image_cnt_ & 1]
// #define last_img_ md_.depth_image_[!(image_cnt_ & 1)]

void GridMap::initMap(rclcpp::Node::SharedPtr node)
{
  node_ = node;
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  /* get parameter */
  double x_size, y_size, z_size;
  node_->declare_parameter("grid_map/resolution", -1.0);
  node_->declare_parameter("grid_map/map_size_x", -1.0);
  node_->declare_parameter("grid_map/map_size_y", -1.0);
  node_->declare_parameter("grid_map/map_size_z", -1.0);
  node_->declare_parameter("grid_map/local_update_range_x", -1.0);
  node_->declare_parameter("grid_map/local_update_range_y", -1.0);
  node_->declare_parameter("grid_map/local_update_range_z", -1.0);
  node_->declare_parameter("grid_map/obstacles_inflation", -1.0);
  node_->declare_parameter("grid_map/fx", -1.0);
  node_->declare_parameter("grid_map/fy", -1.0);
  node_->declare_parameter("grid_map/cx", -1.0);
  node_->declare_parameter("grid_map/cy", -1.0);
  node_->declare_parameter("grid_map/use_depth_filter", true);
  node_->declare_parameter("grid_map/depth_filter_tolerance", -1.0);
  node_->declare_parameter("grid_map/depth_filter_maxdist", -1.0);
  node_->declare_parameter("grid_map/depth_filter_mindist", -1.0);
  node_->declare_parameter("grid_map/depth_filter_margin", -1);
  node_->declare_parameter("grid_map/k_depth_scaling_factor", -1.0);
  node_->declare_parameter("grid_map/skip_pixel", -1);
  node_->declare_parameter("grid_map/p_hit", 0.70);
  node_->declare_parameter("grid_map/p_miss", 0.35);
  node_->declare_parameter("grid_map/p_min", 0.12);
  node_->declare_parameter("grid_map/p_max", 0.97);
  node_->declare_parameter("grid_map/p_occ", 0.80);
  node_->declare_parameter("grid_map/min_ray_length", -0.1);
  node_->declare_parameter("grid_map/max_ray_length", -0.1);
  node_->declare_parameter("grid_map/visualization_truncate_height", -0.1);
  node_->declare_parameter("grid_map/virtual_ceil_height", -0.1);
  node_->declare_parameter("grid_map/virtual_ceil_yp", -0.1);
  node_->declare_parameter("grid_map/virtual_ceil_yn", -0.1);
  node_->declare_parameter("grid_map/show_occ_time", false);
  node_->declare_parameter("grid_map/pose_type", 1);
  node_->declare_parameter("grid_map/frame_id", "map");
  node_->declare_parameter("grid_map/local_map_margin", 1);
  node_->declare_parameter("grid_map/ground_height", 1.0);
  node_->declare_parameter("grid_map/odom_depth_timeout", 1.0);
  node_->declare_parameter("grid_map/sector_filter_enable", false);
  node_->declare_parameter("grid_map/sector_min_angle_deg", -80.0);
  node_->declare_parameter("grid_map/sector_max_angle_deg", 80.0);
  node_->declare_parameter("grid_map/sector_use_stride", false);
  node_->declare_parameter("grid_map/sector_stride", 1);
  node_->declare_parameter("grid_map/sector_use_voxel", false);
  node_->declare_parameter("grid_map/sector_voxel_leaf", 0.10);
  node_->declare_parameter("grid_map/sector_adaptive_recovery", true);
  node_->declare_parameter("grid_map/sector_replan_status_topic", "/planning/replan_success");
  node_->declare_parameter("grid_map/sector_replan_fail_threshold", 1);
  node_->declare_parameter("grid_map/sector_full_view_hold_sec", 2.0);
  node_->declare_parameter("grid_map/sector_replan_burst_enable", false);
  node_->declare_parameter("grid_map/sector_replan_attempt_topic", "/planning/replan_attempt");
  node_->declare_parameter("grid_map/sector_replan_burst_hold_sec", 0.8);
  node_->declare_parameter("grid_map/sector_replan_burst_cooldown_sec", 2.0);
  node_->declare_parameter("grid_map/sector_auto_disable_on_front_block", false);
  node_->declare_parameter("grid_map/sector_front_block_dist", 2.0);
  node_->declare_parameter("grid_map/sector_front_block_points_threshold", 15);
  node_->declare_parameter("grid_map/sector_front_block_half_angle_deg", 25.0);

  node_->get_parameter("grid_map/resolution", mp_.resolution_);
  node_->get_parameter("grid_map/map_size_x", x_size);
  node_->get_parameter("grid_map/map_size_y", y_size);
  node_->get_parameter("grid_map/map_size_z", z_size);
  node_->get_parameter("grid_map/local_update_range_x", mp_.local_update_range_(0));
  node_->get_parameter("grid_map/local_update_range_y", mp_.local_update_range_(1));
  node_->get_parameter("grid_map/local_update_range_z", mp_.local_update_range_(2));
  node_->get_parameter("grid_map/obstacles_inflation", mp_.obstacles_inflation_);
  node_->get_parameter("grid_map/fx", mp_.fx_);
  node_->get_parameter("grid_map/fy", mp_.fy_);
  node_->get_parameter("grid_map/cx", mp_.cx_);
  node_->get_parameter("grid_map/cy", mp_.cy_);
  node_->get_parameter("grid_map/use_depth_filter", mp_.use_depth_filter_);
  node_->get_parameter("grid_map/depth_filter_tolerance", mp_.depth_filter_tolerance_);
  node_->get_parameter("grid_map/depth_filter_maxdist", mp_.depth_filter_maxdist_);
  node_->get_parameter("grid_map/depth_filter_mindist", mp_.depth_filter_mindist_);
  node_->get_parameter("grid_map/depth_filter_margin", mp_.depth_filter_margin_);
  node_->get_parameter("grid_map/k_depth_scaling_factor", mp_.k_depth_scaling_factor_);
  node_->get_parameter("grid_map/skip_pixel", mp_.skip_pixel_);
  node_->get_parameter("grid_map/p_hit", mp_.p_hit_);
  node_->get_parameter("grid_map/p_miss", mp_.p_miss_);
  node_->get_parameter("grid_map/p_min", mp_.p_min_);
  node_->get_parameter("grid_map/p_max", mp_.p_max_);
  node_->get_parameter("grid_map/p_occ", mp_.p_occ_);
  node_->get_parameter("grid_map/min_ray_length", mp_.min_ray_length_);
  node_->get_parameter("grid_map/max_ray_length", mp_.max_ray_length_);
  node_->get_parameter("grid_map/visualization_truncate_height", mp_.visualization_truncate_height_);
  node_->get_parameter("grid_map/virtual_ceil_height", mp_.virtual_ceil_height_);
  node_->get_parameter("grid_map/virtual_ceil_yp", mp_.virtual_ceil_yp_);
  node_->get_parameter("grid_map/virtual_ceil_yn", mp_.virtual_ceil_yn_);
  node_->get_parameter("grid_map/show_occ_time", mp_.show_occ_time_);
  node_->get_parameter("grid_map/pose_type", mp_.pose_type_);
  node_->get_parameter("grid_map/frame_id", mp_.frame_id_);
  node_->get_parameter("grid_map/local_map_margin", mp_.local_map_margin_);
  node_->get_parameter("grid_map/ground_height", mp_.ground_height_);
  node_->get_parameter("grid_map/odom_depth_timeout", mp_.odom_depth_timeout_);
  node_->get_parameter("grid_map/sector_filter_enable", sf_enable_);
  RCLCPP_INFO(node_->get_logger(), "[GridMap] sector_filter_enable=%d", (int)sf_enable_);
  node_->get_parameter("grid_map/sector_use_stride", sf_use_stride_);
  node_->get_parameter("grid_map/sector_use_voxel", sf_use_voxel_);
  node_->get_parameter("grid_map/sector_adaptive_recovery", sf_adaptive_recovery_);
  node_->get_parameter("grid_map/sector_replan_burst_enable", sf_replan_burst_enable_);
  node_->get_parameter("grid_map/sector_auto_disable_on_front_block", sf_auto_disable_on_front_block_);

  double sf_min_angle_deg = -80.0;
  double sf_max_angle_deg = 80.0;
  double sf_front_block_half_angle_deg = 25.0;
  node_->get_parameter("grid_map/sector_min_angle_deg", sf_min_angle_deg);
  node_->get_parameter("grid_map/sector_max_angle_deg", sf_max_angle_deg);
  node_->get_parameter("grid_map/sector_stride", sf_stride_);
  node_->get_parameter("grid_map/sector_voxel_leaf", sf_voxel_leaf_);
  node_->get_parameter("grid_map/sector_replan_status_topic", sf_replan_status_topic_);
  node_->get_parameter("grid_map/sector_replan_fail_threshold", sf_replan_fail_threshold_);
  node_->get_parameter("grid_map/sector_full_view_hold_sec", sf_full_view_hold_sec_);
  node_->get_parameter("grid_map/sector_replan_attempt_topic", sf_replan_attempt_topic_);
  node_->get_parameter("grid_map/sector_replan_burst_hold_sec", sf_replan_burst_hold_sec_);
  node_->get_parameter("grid_map/sector_replan_burst_cooldown_sec", sf_replan_burst_cooldown_sec_);
  node_->get_parameter("grid_map/sector_front_block_dist", sf_front_block_dist_);
  node_->get_parameter("grid_map/sector_front_block_points_threshold", sf_front_block_points_threshold_);
  node_->get_parameter("grid_map/sector_front_block_half_angle_deg", sf_front_block_half_angle_deg);
  sf_min_angle_rad_ = sf_min_angle_deg * M_PI / 180.0;
  sf_max_angle_rad_ = sf_max_angle_deg * M_PI / 180.0;
  sf_front_block_half_angle_rad_ = sf_front_block_half_angle_deg * M_PI / 180.0;

  // ---- parameter sanity (중요) ----
  if (mp_.resolution_ <= 0.0) {
    RCLCPP_FATAL(node_->get_logger(),
                 "[GridMap] resolution(%.3f) invalid. Check params!", mp_.resolution_);
    // 여기서는 강제값 넣기보다, 죽이는게 안전함
    throw std::runtime_error("GridMap resolution invalid");
  }

  if (mp_.local_update_range_.minCoeff() <= 0.0) {
    RCLCPP_WARN(node_->get_logger(),
                "[GridMap] local_update_range invalid (%.2f,%.2f,%.2f). Force to (5,5,3).",
                mp_.local_update_range_.x(), mp_.local_update_range_.y(), mp_.local_update_range_.z());
    mp_.local_update_range_ = Eigen::Vector3d(5.0, 5.0, 3.0);
  }

  if (mp_.obstacles_inflation_ <= 0.0) {
    RCLCPP_WARN(node_->get_logger(),
                "[GridMap] obstacles_inflation(%.3f) invalid. Force to 0.2.",
                mp_.obstacles_inflation_);
    mp_.obstacles_inflation_ = 0.2;
  }

  if (mp_.k_depth_scaling_factor_ <= 0.0) {
    RCLCPP_WARN(node_->get_logger(),
                "[GridMap] k_depth_scaling_factor(%.3f) invalid. Force to 1000.0",
                mp_.k_depth_scaling_factor_);
    mp_.k_depth_scaling_factor_ = 1000.0;
  }

  if (mp_.max_ray_length_ <= 0.0) {
    RCLCPP_WARN(node_->get_logger(),
                "[GridMap] max_ray_length(%.3f) invalid. Force to 5.0",
                mp_.max_ray_length_);
    mp_.max_ray_length_ = 5.0;
  }

  if (mp_.min_ray_length_ < 0.0) {
    mp_.min_ray_length_ = 0.0;
  }

  if (mp_.use_depth_filter_) {
    if (mp_.depth_filter_maxdist_ <= 0.0) {
      RCLCPP_WARN(node_->get_logger(),
                "[GridMap] use_depth_filter=true but depth_filter_maxdist(%.3f) invalid. "
                "Set depth_filter_maxdist = max_ray_length(%.3f)",
                mp_.depth_filter_maxdist_, mp_.max_ray_length_);
      mp_.depth_filter_maxdist_ = mp_.max_ray_length_;
    }

    if (mp_.depth_filter_mindist_ < 0.0) {
      mp_.depth_filter_mindist_ = mp_.min_ray_length_;
    }
  }

  if (mp_.virtual_ceil_height_ - mp_.ground_height_ > z_size)
  {
    mp_.virtual_ceil_height_ = mp_.ground_height_ + z_size;
  }

  if (sf_stride_ < 1) sf_stride_ = 1;
  if (sf_voxel_leaf_ <= 0.0) sf_voxel_leaf_ = 0.10;
  if (sf_replan_fail_threshold_ < 1) sf_replan_fail_threshold_ = 1;
  if (sf_full_view_hold_sec_ < 0.1) sf_full_view_hold_sec_ = 0.1;
  if (sf_replan_burst_hold_sec_ < 0.1) sf_replan_burst_hold_sec_ = 0.1;
  if (sf_replan_burst_cooldown_sec_ < 0.0) sf_replan_burst_cooldown_sec_ = 0.0;
  if (sf_front_block_dist_ < 0.1) sf_front_block_dist_ = 0.1;
  if (sf_front_block_points_threshold_ < 1) sf_front_block_points_threshold_ = 1;
  sf_front_block_half_angle_rad_ = std::max(0.0, std::min(sf_front_block_half_angle_rad_, M_PI));
  sf_force_full_view_until_ = node_->now();
  sf_next_replan_burst_allowed_ = node_->now();

  mp_.resolution_inv_ = 1 / mp_.resolution_;
  mp_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_.ground_height_);
  mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

  mp_.prob_hit_log_ = logit(mp_.p_hit_);
  mp_.prob_miss_log_ = logit(mp_.p_miss_);
  mp_.clamp_min_log_ = logit(mp_.p_min_);
  mp_.clamp_max_log_ = logit(mp_.p_max_);
  mp_.min_occupancy_log_ = logit(mp_.p_occ_);
  mp_.unknown_flag_ = 0.01;

  std::cout << "hit: " << mp_.prob_hit_log_ << std::endl;
  std::cout << "miss: " << mp_.prob_miss_log_ << std::endl;
  std::cout << "min log: " << mp_.clamp_min_log_ << std::endl;
  std::cout << "max: " << mp_.clamp_max_log_ << std::endl;
  std::cout << "thresh log: " << mp_.min_occupancy_log_ << std::endl;

  for (int i = 0; i < 3; ++i)
    mp_.map_voxel_num_(i) = std::ceil(mp_.map_size_(i) / mp_.resolution_);

  mp_.map_min_boundary_ = mp_.map_origin_;
  mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;

  // initialize data buffers

  int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);

  md_.occupancy_buffer_ = std::vector<double>(buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);
  md_.occupancy_buffer_inflate_ = std::vector<char>(buffer_size, 0);

  md_.count_hit_and_miss_ = std::vector<short>(buffer_size, 0);
  md_.count_hit_ = std::vector<short>(buffer_size, 0);
  md_.flag_rayend_ = std::vector<uint32_t>(buffer_size, 0);
  md_.flag_traverse_ = std::vector<uint32_t>(buffer_size, 0);

  md_.raycast_num_ = 0u;
  
  if (mp_.skip_pixel_ <= 0) mp_.skip_pixel_ = 1;
  md_.proj_points_.resize(640 * 480 / mp_.skip_pixel_ / mp_.skip_pixel_);
  md_.proj_points_cnt = 0;


  md_.cam2body_ << 0.0, 0.0, 1.0, 0.0,
      -1.0, 0.0, 0.0, 0.0,
      0.0, -1.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0;

  /* init callback */
  auto qos_sensor = rclcpp::SensorDataQoS();
  auto rmw_qos_sensor = qos_sensor.get_rmw_qos_profile(); 
  // 初始化 message_filters::Subscriber
  depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
      node_, "grid_map/depth", rmw_qos_sensor);

  extrinsic_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "/vins_estimator/extrinsic", 10,
      std::bind(&GridMap::extrinsicCallback, this, std::placeholders::_1));

  if (mp_.pose_type_ == POSE_STAMPED)
  {
    pose_sub_ = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>>(
        node_, "grid_map/pose", rmw_qos_sensor);

    sync_image_pose_ = std::make_shared<message_filters::Synchronizer<SyncPolicyImagePose>>(
        SyncPolicyImagePose(100), *depth_sub_, *pose_sub_);
    sync_image_pose_->registerCallback(
        std::bind(&GridMap::depthPoseCallback, this, std::placeholders::_1, std::placeholders::_2));
  }
  else if (mp_.pose_type_ == ODOMETRY)
  {
    odom_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(
        node_, "grid_map/odom", rmw_qos_sensor);

    sync_image_odom_ = std::make_shared<message_filters::Synchronizer<SyncPolicyImageOdom>>(
        SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_);
    sync_image_odom_->registerCallback(
        std::bind(&GridMap::depthOdomCallback, this, std::placeholders::_1, std::placeholders::_2));
  }

  // 使用独立的里程计和点云订阅
  // cloud/odom 콜백은 ReentrantCallbackGroup → A* 타이머와 병렬 실행 가능
  // (ROS1 AsyncSpinner(4) 동작과 동일)
  auto reentrant_group = node_->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
  auto sub_opts = rclcpp::SubscriptionOptions();
  sub_opts.callback_group = reentrant_group;

  indep_cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "grid_map/cloud", qos_sensor,
      std::bind(&GridMap::cloudCallback, this, std::placeholders::_1),
      sub_opts);

  indep_odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "grid_map/odom", qos_sensor,
      std::bind(&GridMap::odomCallback, this, std::placeholders::_1),
      sub_opts);

  if (sf_enable_ && sf_adaptive_recovery_) {
    replan_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        sf_replan_status_topic_, 10,
        std::bind(&GridMap::replanStatusCallback, this, std::placeholders::_1));
    if (sf_replan_burst_enable_) {
      replan_attempt_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
          sf_replan_attempt_topic_, 10,
          std::bind(&GridMap::replanAttemptCallback, this, std::placeholders::_1));
    }
  }

  // 定时器
  occ_timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(0.5),  // LiDAR 모드에서 즉시 return → 낮은 빈도로 충분
      std::bind(&GridMap::updateOccupancyCallback, this));

  vis_timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(0.20),
      std::bind(&GridMap::visCallback, this));

  // 发布者
  map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/occupancy", 10);
  map_inf_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/occupancy_inflate", 10);

  md_.occ_need_update_ = false;
  md_.local_updated_ = false;
  md_.has_first_depth_ = false;
  md_.has_odom_ = false;
  md_.has_cloud_ = false;
  md_.image_cnt_ = 0;
  md_.last_occ_update_time_ = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);

  md_.fuse_time_ = 0.0;
  md_.update_num_ = 0;
  md_.max_fuse_time_ = 0.0;

  md_.flag_depth_odom_timeout_ = false;
  md_.flag_use_depth_fusion = false;

  if (sf_enable_) {
    RCLCPP_INFO(node_->get_logger(),
                "[GridMap] embedded sector_filter enabled: angle=[%.1f, %.1f]deg stride=%d voxel=%d adaptive=%d",
                sf_min_angle_rad_ * 180.0 / M_PI,
                sf_max_angle_rad_ * 180.0 / M_PI,
                sf_use_stride_ ? sf_stride_ : 1,
                sf_use_voxel_ ? 1 : 0,
                sf_adaptive_recovery_ ? 1 : 0);
  } else {
    RCLCPP_INFO(node_->get_logger(), "[GridMap] embedded sector_filter disabled");
  }

  // rand_noise_ = uniform_real_distribution<double>(-0.2, 0.2);
  // rand_noise2_ = normal_distribution<double>(0, 0.2);
  // random_device rd;
  // eng_ = default_random_engine(rd());
}

void GridMap::resetBuffer()
{
  Eigen::Vector3d min_pos = mp_.map_min_boundary_;
  Eigen::Vector3d max_pos = mp_.map_max_boundary_;

  resetBuffer(min_pos, max_pos);

  md_.local_bound_min_ = Eigen::Vector3i::Zero();
  md_.local_bound_max_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
}

void GridMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
{

  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);

  boundIndex(min_id);
  boundIndex(max_id);

  /* reset occ and dist buffer */
  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z)
      {
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
      }
}

int GridMap::setCacheOccupancy(Eigen::Vector3d pos, int occ)
{
  if (occ != 1 && occ != 0)
    return INVALID_IDX;

  Eigen::Vector3i id;
  posToIndex(pos, id);
  int idx_ctns = toAddress(id);

  md_.count_hit_and_miss_[idx_ctns] += 1;

  if (md_.count_hit_and_miss_[idx_ctns] == 1)
  {
    md_.cache_voxel_.push(id);
  }

  if (occ == 1)
    md_.count_hit_[idx_ctns] += 1;

  return idx_ctns;
}

void GridMap::projectDepthImage()
{
  const int cols = md_.depth_image_.cols;
  const int rows = md_.depth_image_.rows;
  const int skip_pix = std::max(1, mp_.skip_pixel_);

  const std::size_t need =
      (std::size_t)((cols + skip_pix - 1) / skip_pix) *
      (std::size_t)((rows + skip_pix - 1) / skip_pix);

  if (md_.proj_points_.size() < need) md_.proj_points_.resize(need);
  md_.proj_points_cnt = 0;

  const Eigen::Matrix3d camera_r = md_.camera_r_m_;

  // scaling factor 안전 가드
  const double inv_scale =
      (mp_.k_depth_scaling_factor_ > 0.0) ? (1.0 / mp_.k_depth_scaling_factor_) : 1.0;

  if (!mp_.use_depth_filter_)
  {
    // ✅✅✅ (추가) filter를 안 써도 "depth 한번이라도 받았다" 플래그는 켜줘야 함
    if (!md_.has_first_depth_) md_.has_first_depth_ = true;

    for (int v = 0; v < rows; v += skip_pix)
    {
      const uint16_t* row = md_.depth_image_.ptr<uint16_t>(v);

      for (int u = 0; u < cols; u += skip_pix)
      {
        const uint16_t raw = row[u];
        double depth = static_cast<double>(raw) * inv_scale;

        if (raw == 0)
          depth = mp_.max_ray_length_ + 0.1;

        Eigen::Vector3d proj_pt;
        proj_pt(0) = (u - mp_.cx_) * depth / mp_.fx_;
        proj_pt(1) = (v - mp_.cy_) * depth / mp_.fy_;
        proj_pt(2) = depth;

        proj_pt = camera_r * proj_pt + md_.camera_pos_;
        md_.proj_points_[md_.proj_points_cnt++] = proj_pt;
      }
    }
  }
  else
  {
    // depth filter를 쓰는 경우: 첫 프레임은 "이전 프레임 없음"이라서 체크 안 함
    if (!md_.has_first_depth_)
    {
      md_.has_first_depth_ = true;
    }
    else
    {
      Eigen::Matrix3d last_camera_r_inv = md_.last_camera_r_m_.inverse();
      const int margin = std::max(0, mp_.depth_filter_margin_);

      for (int v = margin; v < rows - margin; v += skip_pix)
      {
        const uint16_t* row = md_.depth_image_.ptr<uint16_t>(v);

        for (int u = margin; u < cols - margin; u += skip_pix)
        {
          const uint16_t raw = row[u];
          double depth = static_cast<double>(raw) * inv_scale;

          if (raw == 0)
            depth = mp_.max_ray_length_ + 0.1;
          else if (depth < mp_.depth_filter_mindist_)
            continue;
          else if (depth > mp_.depth_filter_maxdist_)
            depth = mp_.max_ray_length_ + 0.1;

          Eigen::Vector3d pt_cur;
          pt_cur(0) = (u - mp_.cx_) * depth / mp_.fx_;
          pt_cur(1) = (v - mp_.cy_) * depth / mp_.fy_;
          pt_cur(2) = depth;

          Eigen::Vector3d pt_world = camera_r * pt_cur + md_.camera_pos_;
          md_.proj_points_[md_.proj_points_cnt++] = pt_world;

          // consistency check (원 코드 false 유지)
          if (false)
          {
            Eigen::Vector3d pt_reproj = last_camera_r_inv * (pt_world - md_.last_camera_pos_);
            const double uu = pt_reproj.x() * mp_.fx_ / pt_reproj.z() + mp_.cx_;
            const double vv = pt_reproj.y() * mp_.fy_ / pt_reproj.z() + mp_.cy_;

            if (uu >= 0 && uu < cols && vv >= 0 && vv < rows)
            {
              const uint16_t last_raw = md_.last_depth_image_.at<uint16_t>((int)vv, (int)uu);
              const double last_depth = static_cast<double>(last_raw) * inv_scale;

              if (std::fabs(last_depth - pt_reproj.z()) < mp_.depth_filter_tolerance_)
                md_.proj_points_[md_.proj_points_cnt++] = pt_world;
            }
            else
            {
              md_.proj_points_[md_.proj_points_cnt++] = pt_world;
            }
          }
        }
      }
    }
  }

  // maintain camera pose
  md_.last_camera_pos_ = md_.camera_pos_;
  md_.last_camera_r_m_ = md_.camera_r_m_;
  md_.last_depth_image_ = md_.depth_image_;
}


void GridMap::raycastProcess()
{
  if (md_.proj_points_cnt == 0) return;

  if (++md_.raycast_num_ == 0u) { // overflow로 0이 되면
    std::fill(md_.flag_rayend_.begin(), md_.flag_rayend_.end(), 0u);
    std::fill(md_.flag_traverse_.begin(), md_.flag_traverse_.end(), 0u);
    md_.raycast_num_ = 1u;
  }

  int vox_idx;
  double length;

  // bounding box of updated region
  double min_x = mp_.map_max_boundary_(0);
  double min_y = mp_.map_max_boundary_(1);
  double min_z = mp_.map_max_boundary_(2);

  double max_x = mp_.map_min_boundary_(0);
  double max_y = mp_.map_min_boundary_(1);
  double max_z = mp_.map_min_boundary_(2);

  RayCaster raycaster;
  Eigen::Vector3d half(0.5, 0.5, 0.5);
  Eigen::Vector3d ray_pt, pt_w;

  for (int i = 0; i < md_.proj_points_cnt; ++i)
  {
    pt_w = md_.proj_points_[i];

    // set flag for projected point
    if (!isInMap(pt_w))
    {
      pt_w = closetPointInMap(pt_w, md_.camera_pos_);

      length = (pt_w - md_.camera_pos_).norm();
      if (length > mp_.max_ray_length_)
      {
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
      }
      vox_idx = setCacheOccupancy(pt_w, 0);
    }
    else
    {
      length = (pt_w - md_.camera_pos_).norm();

      if (length > mp_.max_ray_length_)
      {
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
        vox_idx = setCacheOccupancy(pt_w, 0);
      }
      else
      {
        vox_idx = setCacheOccupancy(pt_w, 1);
      }
    }

    // ✅ update local AABB
    max_x = std::max(max_x, pt_w(0));
    max_y = std::max(max_y, pt_w(1));
    max_z = std::max(max_z, pt_w(2));
    min_x = std::min(min_x, pt_w(0));
    min_y = std::min(min_y, pt_w(1));
    min_z = std::min(min_z, pt_w(2));

    // raycasting between camera center and point
    if (vox_idx != INVALID_IDX)
    {
      if (md_.flag_rayend_[vox_idx] == md_.raycast_num_)
        continue;
      md_.flag_rayend_[vox_idx] = md_.raycast_num_;
    }

    raycaster.setInput(pt_w / mp_.resolution_, md_.camera_pos_ / mp_.resolution_);

    while (raycaster.step(ray_pt))
    {
      Eigen::Vector3d tmp = (ray_pt + half) * mp_.resolution_;

      vox_idx = setCacheOccupancy(tmp, 0);

      if (vox_idx != INVALID_IDX)
      {
        if (md_.flag_traverse_[vox_idx] == md_.raycast_num_)
          break;
        md_.flag_traverse_[vox_idx] = md_.raycast_num_;
      }
    }
  }

  // ✅✅✅ (중요) local_bound 반드시 갱신해야 clear/inflate/publish 범위가 정상
  min_x = std::min(min_x, md_.camera_pos_(0));
  min_y = std::min(min_y, md_.camera_pos_(1));
  min_z = std::min(min_z, md_.camera_pos_(2));

  max_x = std::max(max_x, md_.camera_pos_(0));
  max_y = std::max(max_y, md_.camera_pos_(1));
  max_z = std::max(max_z, md_.camera_pos_(2));
  max_z = std::max(max_z, mp_.ground_height_);

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);
  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);

  md_.local_updated_ = true;

  // update occupancy cached in queue
  Eigen::Vector3d local_range_min = md_.camera_pos_ - mp_.local_update_range_;
  Eigen::Vector3d local_range_max = md_.camera_pos_ + mp_.local_update_range_;

  Eigen::Vector3i min_id, max_id;
  posToIndex(local_range_min, min_id);
  posToIndex(local_range_max, max_id);
  boundIndex(min_id);
  boundIndex(max_id);

  while (!md_.cache_voxel_.empty())
  {
    Eigen::Vector3i idx = md_.cache_voxel_.front();
    int idx_ctns = toAddress(idx);
    md_.cache_voxel_.pop();

    double log_odds_update =
        md_.count_hit_[idx_ctns] >= (md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns])
            ? mp_.prob_hit_log_
            : mp_.prob_miss_log_;

    md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;

    if (log_odds_update >= 0 && md_.occupancy_buffer_[idx_ctns] >= mp_.clamp_max_log_)
      continue;

    if (log_odds_update <= 0 && md_.occupancy_buffer_[idx_ctns] <= mp_.clamp_min_log_)
    {
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
      continue;
    }

    bool in_local =
        idx(0) >= min_id(0) && idx(0) <= max_id(0) &&
        idx(1) >= min_id(1) && idx(1) <= max_id(1) &&
        idx(2) >= min_id(2) && idx(2) <= max_id(2);

    if (!in_local)
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;

    md_.occupancy_buffer_[idx_ctns] =
        std::min(std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update, mp_.clamp_min_log_),
                 mp_.clamp_max_log_);
  }
}

Eigen::Vector3d GridMap::closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt)
{
  Eigen::Vector3d diff = pt - camera_pt;
  Eigen::Vector3d max_tc = mp_.map_max_boundary_ - camera_pt;
  Eigen::Vector3d min_tc = mp_.map_min_boundary_ - camera_pt;

  double min_t = 1000000;

  for (int i = 0; i < 3; ++i)
  {
    if (std::fabs(diff[i]) > 0)
    {

      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t)
        min_t = t1;

      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t)
        min_t = t2;
    }
  }

  return camera_pt + (min_t - 1e-3) * diff;
}

void GridMap::clearAndInflateLocalMap()
{
  /*clear outside local*/
  const int vec_margin = 5;
  // Eigen::Vector3i min_vec_margin = min_vec - Eigen::Vector3i(vec_margin,
  // vec_margin, vec_margin); Eigen::Vector3i max_vec_margin = max_vec +
  // Eigen::Vector3i(vec_margin, vec_margin, vec_margin);

  Eigen::Vector3i min_cut = md_.local_bound_min_ -
                            Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  Eigen::Vector3i max_cut = md_.local_bound_max_ +
                            Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  boundIndex(min_cut);
  boundIndex(max_cut);

  Eigen::Vector3i min_cut_m = min_cut - Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  Eigen::Vector3i max_cut_m = max_cut + Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  boundIndex(min_cut_m);
  boundIndex(max_cut_m);

  // clear data outside the local range

  for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
    {

      for (int z = min_cut_m(2); z < min_cut(2); ++z)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }

      for (int z = max_cut(2) + 1; z <= max_cut_m(2); ++z)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }
    }

  for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
    for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    {

      for (int y = min_cut_m(1); y < min_cut(1); ++y)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }

      for (int y = max_cut(1) + 1; y <= max_cut_m(1); ++y)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }
    }

  for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
    for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
    {

      for (int x = min_cut_m(0); x < min_cut(0); ++x)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }

      for (int x = max_cut(0) + 1; x <= max_cut_m(0); ++x)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }
    }

  // inflate occupied voxels to compensate robot size

  int inf_step = std::ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  // int inf_step_z = 1;
  std::vector<Eigen::Vector3i> inf_pts(pow(2 * inf_step + 1, 3));
  // inf_pts.resize(4 * inf_step + 3);
  Eigen::Vector3i inf_pt;

  // clear outdated data
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z)
      {
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
      }

  // inflate obstacles
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z)
      {

        if (md_.occupancy_buffer_[toAddress(x, y, z)] > mp_.min_occupancy_log_)
        {
          inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_pts);

          for (int k = 0; k < (int)inf_pts.size(); ++k)
          {
            inf_pt = inf_pts[k];
            int idx_inf = toAddress(inf_pt);
            if (idx_inf < 0 ||
                idx_inf >= mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2))
            {
              continue;
            }
            md_.occupancy_buffer_inflate_[idx_inf] = 1;
          }
        }
      }

  // add virtual ceiling to limit flight height
  if (mp_.virtual_ceil_height_ > -0.5) {
    int ceil_id = (int)std::floor((mp_.virtual_ceil_height_ - mp_.map_origin_(2)) * mp_.resolution_inv_) - 1;

    if (ceil_id >= 0 && ceil_id < mp_.map_voxel_num_(2)) {
      for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
        for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
          md_.occupancy_buffer_inflate_[toAddress(x, y, ceil_id)] = 1;
    }
  }

}

void GridMap::visCallback()
{
  publishMapInflate(true);
  publishMap();
}

void GridMap::updateOccupancyCallback()
{
  if (md_.last_occ_update_time_.seconds() < 1.0)
    md_.last_occ_update_time_ = node_->now();

  if (!md_.occ_need_update_)
  {
    if (md_.flag_use_depth_fusion &&
        (node_->now() - md_.last_occ_update_time_).seconds() > mp_.odom_depth_timeout_)
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "odom or depth lost! now=%f, last_occ_update_time=%f, odom_depth_timeout=%f",
                   node_->now().seconds(),
                   md_.last_occ_update_time_.seconds(),
                   mp_.odom_depth_timeout_);
      md_.flag_depth_odom_timeout_ = true;
      md_.flag_use_depth_fusion = false;  // ✅ odomCallback 재활성화
      md_.occ_need_update_ = false;       // ✅ depth 없으니 업데이트 중단
    }
    return;
  }
  md_.last_occ_update_time_ = node_->now();

  /* update occupancy */
  // ros::Time t1, t2, t3, t4;
  // t1 = ros::Time::now();

  projectDepthImage();
  // t2 = ros::Time::now();
  raycastProcess();
  // t3 = ros::Time::now();

  if (md_.local_updated_)
    clearAndInflateLocalMap();

  // t4 = ros::Time::now();

  // cout << setprecision(7);
  // cout << "t2=" << (t2-t1).toSec() << " t3=" << (t3-t2).toSec() << " t4=" << (t4-t3).toSec() << endl;;

  // md_.fuse_time_ += (t2 - t1).toSec();
  // md_.max_fuse_time_ = max(md_.max_fuse_time_, (t2 - t1).toSec());

  // if (mp_.show_occ_time_)
  //   ROS_WARN("Fusion: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).toSec(),
  //            md_.fuse_time_ / md_.update_num_, md_.max_fuse_time_);

  md_.occ_need_update_ = false;
  md_.local_updated_ = false;
}

void GridMap::armFullViewRecovery(const std::string& reason, double hold_sec)
{
  if (!sf_enable_) return;  // sector filter 비활성화 시 무시
  const double use_hold_sec = (hold_sec > 0.0) ? hold_sec : sf_full_view_hold_sec_;

  // 호출 시점에 이미 full view 모드인지 확인
  const auto now = node_->now();
  const int prev_frames = sf_full_view_frames_remaining_.load();
  const bool was_in_full_view = (now < sf_force_full_view_until_) || (prev_frames > 0);

  // 상태 업데이트
  const auto until = now + rclcpp::Duration::from_seconds(use_hold_sec);
  const int frames = std::max(40, static_cast<int>(use_hold_sec * 20.0));
  if (until > sf_force_full_view_until_) {
    sf_force_full_view_until_ = until;
  }
  sf_full_view_frames_remaining_.store(std::max(prev_frames, frames));

  // sector → full view 전환 시에만 로그
  if (!was_in_full_view) {
    RCLCPP_WARN(node_->get_logger(),
                "[GridMap] Sector full-view recovery %.2fs / %d frames (%s)",
                use_hold_sec, frames, reason.c_str());
  }
}

void GridMap::replanStatusCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!sf_enable_ || !sf_adaptive_recovery_) return;

  const bool in_full_view = (node_->now() < sf_force_full_view_until_);

  if (msg->data) {
    // 리플래닝 성공
    sf_consecutive_replan_failures_ = 0;

    if (in_full_view) {
      // 실패 기반 복구(3600s hold)만 성공 카운터로 조기 복귀
      // 시간 기반 복구(웨이포인트 등, 짧은 hold)는 자연 만료에 맡김
      const double remaining = (sf_force_full_view_until_ - node_->now()).seconds();
      const bool is_failure_recovery = (remaining > 30.0);
      if (is_failure_recovery) {
        sf_consecutive_replan_successes_++;
        if (sf_consecutive_replan_successes_ >= sf_return_after_n_success_) {
          sf_force_full_view_until_ = node_->now();  // 시간 카운터 리셋
          sf_full_view_frames_remaining_.store(0);   // 프레임 카운터도 리셋
          sf_consecutive_replan_successes_ = 0;
          RCLCPP_INFO(node_->get_logger(),
            "[GridMap] Sector filter restored after %d consecutive successes",
            sf_return_after_n_success_);
        }
      }
    } else {
      sf_consecutive_replan_successes_ = 0;
    }
    return;
  }

  // 리플래닝 실패
  sf_consecutive_replan_successes_ = 0;
  sf_consecutive_replan_failures_++;
  if (sf_consecutive_replan_failures_ >= sf_replan_fail_threshold_) {
    // full view hold: 시간 기반 대신 성공 횟수 기반으로 복귀하므로
    // hold_sec을 매우 길게 설정 (성공 카운터가 복귀 조건)
    armFullViewRecovery("replan failures", 3600.0);
    sf_consecutive_replan_failures_ = 0;
    sf_consecutive_replan_successes_ = 0;
  }
}

void GridMap::replanAttemptCallback(const std_msgs::msg::Empty::SharedPtr /*msg*/)
{
  if (!sf_enable_ || !sf_adaptive_recovery_ || !sf_replan_burst_enable_) return;

  const auto now = node_->now();
  if (now < sf_next_replan_burst_allowed_) {
    return;
  }
  armFullViewRecovery("replan burst", sf_replan_burst_hold_sec_);
  sf_next_replan_burst_allowed_ =
      now + rclcpp::Duration::from_seconds(sf_replan_burst_cooldown_sec_);
}

// 기존: map frame에서 역회전 후 각도 필터 (source==target 또는 fallback 경로용)
void GridMap::applySectorFilter(pcl::PointCloud<pcl::PointXYZ>& cloud_map)
{
  if (!sf_enable_ || cloud_map.points.empty()) return;
  if (!md_.has_odom_) return;

  const int frames_left = sf_full_view_frames_remaining_.load();
  const auto now = node_->now();
  const bool time_based = sf_adaptive_recovery_ && (now < sf_force_full_view_until_);
  const bool frame_based = sf_adaptive_recovery_ && (frames_left > 0);
  bool force_full_view = time_based || frame_based;

  pcl::PointCloud<pcl::PointXYZ> filtered;
  filtered.header = cloud_map.header;
  filtered.points.reserve(cloud_map.points.size());

  const Eigen::Matrix3d world_to_sensor = md_.camera_r_m_.transpose();
  int front_block_points = 0;

  for (size_t i = 0; i < cloud_map.points.size(); ++i) {
    if (sf_use_stride_ && (i % static_cast<size_t>(sf_stride_) != 0)) continue;

    const auto& p = cloud_map.points[i];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;

    const Eigen::Vector3d pw(p.x, p.y, p.z);
    const Eigen::Vector3d ps = world_to_sensor * (pw - md_.camera_pos_);
    const double ang = std::atan2(ps.y(), ps.x());

    if (sf_adaptive_recovery_ && sf_auto_disable_on_front_block_) {
      const double range_xy = std::hypot(ps.x(), ps.y());
      if (range_xy < sf_front_block_dist_ && std::fabs(ang) <= sf_front_block_half_angle_rad_) {
        ++front_block_points;
      }
    }

    if (!force_full_view) {
      if (ang < sf_min_angle_rad_ || ang > sf_max_angle_rad_) continue;
    }

    filtered.points.push_back(p);
  }

  if (!force_full_view && sf_adaptive_recovery_ && sf_auto_disable_on_front_block_ &&
      front_block_points >= sf_front_block_points_threshold_) {
    armFullViewRecovery("front blocked");

    filtered.clear();
    filtered.points.reserve(cloud_map.points.size());
    for (size_t i = 0; i < cloud_map.points.size(); ++i) {
      if (sf_use_stride_ && (i % static_cast<size_t>(sf_stride_) != 0)) continue;
      const auto& p = cloud_map.points[i];
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
      filtered.points.push_back(p);
    }
  }

  // Avoid map update stalls when sector constraints are temporarily too strict.
  // If filtered cloud is empty, keep the original raw cloud for this frame.
  if (filtered.points.empty()) {
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "[GridMap] sector filter produced empty cloud. Falling back to raw cloud.");
    return;
  }

  filtered.width = static_cast<uint32_t>(filtered.points.size());
  filtered.height = 1;
  filtered.is_dense = false;

  if (sf_use_voxel_ && !filtered.points.empty()) {
    pcl::PointCloud<pcl::PointXYZ> voxel_out;
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(filtered.makeShared());
    vg.setLeafSize(static_cast<float>(sf_voxel_leaf_),
                   static_cast<float>(sf_voxel_leaf_),
                   static_cast<float>(sf_voxel_leaf_));
    vg.filter(voxel_out);
    voxel_out.header = filtered.header;
    cloud_map.swap(voxel_out);
  } else {
    cloud_map.swap(filtered);
  }
}

// 최적화: sensor frame에서 직접 각도 필터 (TF 변환 전에 호출, 역회전 불필요)
void GridMap::applySectorFilterSensorFrame(pcl::PointCloud<pcl::PointXYZ>& cloud_sensor)
{
  if (!sf_enable_ || cloud_sensor.points.empty()) return;

  // Frame-counter: decrement each call, use full view while > 0
  const int frames_left = sf_full_view_frames_remaining_.load();
  if (frames_left > 0) sf_full_view_frames_remaining_.store(frames_left - 1);

  const auto now = node_->now();
  const bool time_based = sf_adaptive_recovery_ && (now < sf_force_full_view_until_);
  const bool frame_based = sf_adaptive_recovery_ && (frames_left > 0);
  bool force_full_view = time_based || frame_based;

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
    "[GridMap] sector filter: force_full=%d (time=%d frames=%d left=%d)",
    force_full_view, time_based, frame_based, frames_left);

  pcl::PointCloud<pcl::PointXYZ> filtered;
  filtered.header = cloud_sensor.header;
  filtered.points.reserve(cloud_sensor.points.size());

  int front_block_points = 0;

  for (size_t i = 0; i < cloud_sensor.points.size(); ++i) {
    if (sf_use_stride_ && (i % static_cast<size_t>(sf_stride_) != 0)) continue;

    const auto& p = cloud_sensor.points[i];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;

    // sensor frame이므로 역회전 없이 바로 수평 각도 계산
    const double ang = std::atan2(static_cast<double>(p.y), static_cast<double>(p.x));

    if (sf_adaptive_recovery_ && sf_auto_disable_on_front_block_) {
      const double range_xy = std::hypot(static_cast<double>(p.x), static_cast<double>(p.y));
      if (range_xy < sf_front_block_dist_ && std::fabs(ang) <= sf_front_block_half_angle_rad_) {
        ++front_block_points;
      }
    }

    if (!force_full_view) {
      if (ang < sf_min_angle_rad_ || ang > sf_max_angle_rad_) continue;
    }

    filtered.points.push_back(p);
  }

  if (!force_full_view && sf_adaptive_recovery_ && sf_auto_disable_on_front_block_ &&
      front_block_points >= sf_front_block_points_threshold_) {
    armFullViewRecovery("front blocked");

    filtered.clear();
    filtered.points.reserve(cloud_sensor.points.size());
    for (size_t i = 0; i < cloud_sensor.points.size(); ++i) {
      if (sf_use_stride_ && (i % static_cast<size_t>(sf_stride_) != 0)) continue;
      const auto& p = cloud_sensor.points[i];
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
      filtered.points.push_back(p);
    }
  }

  filtered.width = static_cast<uint32_t>(filtered.points.size());
  filtered.height = 1;
  filtered.is_dense = false;

  if (sf_use_voxel_ && !filtered.points.empty()) {
    pcl::PointCloud<pcl::PointXYZ> voxel_out;
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(filtered.makeShared());
    vg.setLeafSize(static_cast<float>(sf_voxel_leaf_),
                   static_cast<float>(sf_voxel_leaf_),
                   static_cast<float>(sf_voxel_leaf_));
    vg.filter(voxel_out);
    voxel_out.header = filtered.header;
    cloud_sensor.swap(voxel_out);
  } else {
    cloud_sensor.swap(filtered);
  }
}

void GridMap::depthPoseCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr img,
  const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose)
{

  // ✅ (추가) depth/pose가 들어왔다는 heartbeat: timeout 오탐 방지
  md_.last_occ_update_time_ = node_->now();
  md_.flag_depth_odom_timeout_ = false;


  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
    "[GridMap] depth callback alive enc=%s size=%ux%u",
    img->encoding.c_str(), img->width, img->height);
  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);

  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    double scale = mp_.k_depth_scaling_factor_;
    if (scale <= 0.0) scale = 1000.0;
    cv_ptr->image.convertTo(cv_ptr->image, CV_16UC1, scale);
  }
  cv_ptr->image.copyTo(md_.depth_image_);
  md_.flag_use_depth_fusion = true;

  // std::cout << "depth: " << md_.depth_image_.cols << ", " << md_.depth_image_.rows << std::endl;

  /* get pose */
  md_.camera_pos_(0) = pose->pose.position.x;
  md_.camera_pos_(1) = pose->pose.position.y;
  md_.camera_pos_(2) = pose->pose.position.z;
  md_.camera_r_m_ = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
                                       pose->pose.orientation.y, pose->pose.orientation.z)
                        .toRotationMatrix();

  md_.has_odom_ = true;
                          
  if (isInMap(md_.camera_pos_))
  {
    md_.update_num_ += 1;
    md_.occ_need_update_ = true;
  }
  else
  {
    md_.occ_need_update_ = false;
  }

}

void GridMap::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  // depth-fusion(DepthPose/DepthOdom)을 쓰기 시작하면, indep odom으로 pose 덮어쓰지 않음
  if (md_.flag_use_depth_fusion)
    return;

  const std::string target_frame = mp_.frame_id_;        // 예: "map"
  const std::string source_frame = odom->header.frame_id; // odom pose 기준 frame

  if (source_frame.empty()) {
    RCLCPP_WARN(node_->get_logger(), "[GridMap] odom frame_id empty, skip");
    return;
  }

  geometry_msgs::msg::PoseStamped ps_in, ps_out;
  ps_in.header = odom->header;
  ps_in.pose   = odom->pose.pose;

  // 1) 같은 프레임이면 그대로 사용
  if (source_frame == target_frame) {
    ps_out = ps_in;
  } else {
    // 2) stamp 기준 TF 시도 -> 실패하면 latest로 fallback
    try {
      auto tf = tf_buffer_->lookupTransform(
          target_frame, source_frame,
          rclcpp::Time(odom->header.stamp),
          rclcpp::Duration::from_seconds(0.05));
      tf2::doTransform(ps_in, ps_out, tf);

    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 2000,
          "[GridMap] odom TF lookup failed (%s -> %s) at stamp: %.3f, err: %s. Fallback to latest.",
          source_frame.c_str(), target_frame.c_str(),
          rclcpp::Time(odom->header.stamp).seconds(), ex.what());

      try {
        auto tf_latest = tf_buffer_->lookupTransform(
            target_frame, source_frame, tf2::TimePointZero);
        tf2::doTransform(ps_in, ps_out, tf_latest);

      } catch (const tf2::TransformException &ex2) {
        // Keep mapping alive even if TF tree is briefly disconnected.
        // Use raw odom pose as approximate map-frame pose.
        ps_out = ps_in;
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 2000,
            "[GridMap] odom TF fallback failed (%s -> %s): %s. Using raw odom pose as fallback.",
            source_frame.c_str(), target_frame.c_str(), ex2.what());
      }
    }
  }

  md_.camera_pos_(0) = ps_out.pose.position.x;
  md_.camera_pos_(1) = ps_out.pose.position.y;
  md_.camera_pos_(2) = ps_out.pose.position.z;
  Eigen::Quaterniond q(ps_out.pose.orientation.w,
                       ps_out.pose.orientation.x,
                       ps_out.pose.orientation.y,
                       ps_out.pose.orientation.z);
  if (q.norm() > 1e-6) {
    q.normalize();
    md_.camera_r_m_ = q.toRotationMatrix();
  } else {
    md_.camera_r_m_ = Eigen::Matrix3d::Identity();
  }
  md_.has_odom_ = true;
}

void GridMap::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (md_.flag_use_depth_fusion)
    return;

  const std::string target_frame = mp_.frame_id_;         // "map"
  const std::string source_frame = msg->header.frame_id;  // 예: lidar/body

  if (source_frame.empty()) {
    RCLCPP_WARN(node_->get_logger(), "[GridMap] cloud frame_id empty, skip");
    return;
  }

  md_.has_cloud_ = true;

  if (!md_.has_odom_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "[GridMap] no odom yet, skip cloud");
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> latest_cloud;

  // 최적화 경로: sensor frame != map frame인 정상 TF 경로
  // fromROSMsg → sensor frame에서 필터 → 줄어든 포인트만 TF 변환
  if (sf_enable_ && source_frame != target_frame) {
    pcl::PointCloud<pcl::PointXYZ> cloud_sensor;
    pcl::fromROSMsg(*msg, cloud_sensor);
    const size_t before = cloud_sensor.points.size();

    applySectorFilterSensorFrame(cloud_sensor);

    if (cloud_sensor.points.empty()) return;

    geometry_msgs::msg::TransformStamped tf_stamped;
    bool tf_ok = false;
    try {
      tf_stamped = tf_buffer_->lookupTransform(
          target_frame, source_frame,
          rclcpp::Time(msg->header.stamp),
          rclcpp::Duration::from_seconds(0.05));
      tf_ok = true;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "[GridMap] TF lookup failed (%s -> %s) at stamp: %.3f, err: %s. Fallback to latest.",
          source_frame.c_str(), target_frame.c_str(),
          rclcpp::Time(msg->header.stamp).seconds(), ex.what());
      try {
        tf_stamped = tf_buffer_->lookupTransform(
            target_frame, source_frame, tf2::TimePointZero);
        tf_ok = true;
      } catch (const tf2::TransformException &ex2) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "[GridMap] TF fallback failed (%s -> %s): %s. Using odom-pose.",
                             source_frame.c_str(), target_frame.c_str(), ex2.what());
      }
    }

    if (tf_ok) {
      const auto& t = tf_stamped.transform.translation;
      const auto& r = tf_stamped.transform.rotation;
      Eigen::Quaternionf q(
          static_cast<float>(r.w), static_cast<float>(r.x),
          static_cast<float>(r.y), static_cast<float>(r.z));
      Eigen::Affine3f T = Eigen::Affine3f::Identity();
      T.linear() = q.toRotationMatrix();
      T.translation() = Eigen::Vector3f(
          static_cast<float>(t.x), static_cast<float>(t.y), static_cast<float>(t.z));
      pcl::transformPointCloud(cloud_sensor, latest_cloud, T);
    } else {
      // TF 완전 실패 시: odom 포즈로 근사 변환
      Eigen::Affine3f T = Eigen::Affine3f::Identity();
      T.linear() = md_.camera_r_m_.cast<float>();
      T.translation() = md_.camera_pos_.cast<float>();
      pcl::transformPointCloud(cloud_sensor, latest_cloud, T);
    }
    latest_cloud.header.frame_id = target_frame;

  } else {
    // 기존 경로: source==target이거나 sf 비활성화인 경우
    sensor_msgs::msg::PointCloud2 cloud_map;
    if (source_frame == target_frame) {
      cloud_map = *msg;
    } else {
      try {
        auto tf = tf_buffer_->lookupTransform(
            target_frame, source_frame,
            rclcpp::Time(msg->header.stamp),
            rclcpp::Duration::from_seconds(0.0));  // 0ms: 즉시 실패 → TimePointZero로 낙하
        tf2::doTransform(*msg, cloud_map, tf);
      } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "[GridMap] TF lookup failed (%s -> %s) at stamp: %.3f, err: %s. Fallback to latest.",
            source_frame.c_str(), target_frame.c_str(),
            rclcpp::Time(msg->header.stamp).seconds(), ex.what());
        try {
          auto tf_latest = tf_buffer_->lookupTransform(
              target_frame, source_frame, tf2::TimePointZero);
          tf2::doTransform(*msg, cloud_map, tf_latest);
        } catch (const tf2::TransformException &ex2) {
          if (!md_.has_odom_) {
            cloud_map = *msg;
            cloud_map.header.frame_id = target_frame;
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 1000,
                "[GridMap] TF fallback failed (%s -> %s): %s (no odom). Using raw cloud fallback.",
                source_frame.c_str(), target_frame.c_str(), ex2.what());
          } else {
            pcl::PointCloud<pcl::PointXYZ> cloud_src, cloud_fallback;
            pcl::fromROSMsg(*msg, cloud_src);
            Eigen::Affine3f T = Eigen::Affine3f::Identity();
            T.linear() = md_.camera_r_m_.cast<float>();
            T.translation() = md_.camera_pos_.cast<float>();
            pcl::transformPointCloud(cloud_src, cloud_fallback, T);
            pcl::toROSMsg(cloud_fallback, cloud_map);
            cloud_map.header = msg->header;
            cloud_map.header.frame_id = target_frame;
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                 "[GridMap] TF fallback failed (%s -> %s). Using odom-pose cloud fallback.",
                                 source_frame.c_str(), target_frame.c_str());
          }
        }
      }
    }
    pcl::fromROSMsg(cloud_map, latest_cloud);
    applySectorFilter(latest_cloud);
  }

  if (latest_cloud.points.empty())
    return;

  if (!std::isfinite(md_.camera_pos_(0)) ||
      !std::isfinite(md_.camera_pos_(1)) ||
      !std::isfinite(md_.camera_pos_(2)))
    return;

  // ---- guard: 파라미터 이상하면 컷 ----
  if (mp_.resolution_ <= 0.0 ||
      mp_.local_update_range_.minCoeff() <= 0.0 ||
      mp_.obstacles_inflation_ <= 0.0) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "[GridMap] invalid params: res=%.3f local=(%.2f,%.2f,%.2f) inflation=%.2f",
      mp_.resolution_,
      mp_.local_update_range_.x(), mp_.local_update_range_.y(), mp_.local_update_range_.z(),
      mp_.obstacles_inflation_);
    return;
  }

  // local 영역 inflate map 클리어 (sector filter 시 시각화는 publishMapInflate 마스킹이 담당)
  this->resetBuffer(md_.camera_pos_ - mp_.local_update_range_,
                    md_.camera_pos_ + mp_.local_update_range_);

  // ---- inflation step ----
  const int inf_step_xy = std::max(1, (int)std::ceil(mp_.obstacles_inflation_ / mp_.resolution_));
  const int inf_step_z  = 1; // 필요하면 파라미터화

  // offsets 캐시
  static int last_xy = -1, last_z = -1;
  static std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> offsets;
  if (inf_step_xy != last_xy || inf_step_z != last_z) {
    offsets.clear();
    offsets.reserve((2*inf_step_xy + 1) * (2*inf_step_xy + 1) * (2*inf_step_z + 1));
    for (int dx = -inf_step_xy; dx <= inf_step_xy; ++dx)
      for (int dy = -inf_step_xy; dy <= inf_step_xy; ++dy)
        for (int dz = -inf_step_z; dz <= inf_step_z; ++dz)
          offsets.emplace_back(dx, dy, dz);
    last_xy = inf_step_xy;
    last_z  = inf_step_z;
  }

  // local bound (index 기준)
  Eigen::Vector3i bound_min = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
  Eigen::Vector3i bound_max = Eigen::Vector3i::Zero();
  bool any = false;
  const double min_keep_dist = std::max(mp_.min_ray_length_, 1.5 * mp_.resolution_);

  // camera voxel 포함(항상 publish 범위가 0이 되지 않게)
  Eigen::Vector3i cam_id;
  posToIndex(md_.camera_pos_, cam_id);
  boundIndex(cam_id);
  bound_min = bound_min.cwiseMin(cam_id);
  bound_max = bound_max.cwiseMax(cam_id);

  for (const auto &p : latest_cloud.points)
  {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
      continue;

    // local update range 필터
    const double dx = p.x - md_.camera_pos_.x();
    const double dy = p.y - md_.camera_pos_.y();
    const double dz = p.z - md_.camera_pos_.z();
    const double range = std::sqrt(dx * dx + dy * dy + dz * dz);

    // Ignore near-field points to avoid self-occupancy around the drone body/sensor.
    if (range < min_keep_dist)
      continue;

    if (std::fabs(dx) > mp_.local_update_range_.x() ||
        std::fabs(dy) > mp_.local_update_range_.y() ||
        std::fabs(dz) > mp_.local_update_range_.z())
      continue;

    Eigen::Vector3i base_id;
    posToIndex(Eigen::Vector3d(p.x, p.y, p.z), base_id);
    if (!isInMap(base_id))
      continue;

    any = true;

    bound_min = bound_min.cwiseMin(base_id - Eigen::Vector3i(inf_step_xy, inf_step_xy, inf_step_z));
    bound_max = bound_max.cwiseMax(base_id + Eigen::Vector3i(inf_step_xy, inf_step_xy, inf_step_z));

    for (const auto &off : offsets)
    {
      const Eigen::Vector3i id = base_id + off;
      if (!isInMap(id)) continue;
      md_.occupancy_buffer_inflate_[toAddress(id)] = 1;
    }
  }

  // local bound 확정
  if (any) {
    boundIndex(bound_min);
    boundIndex(bound_max);
    md_.local_bound_min_ = bound_min;
    md_.local_bound_max_ = bound_max;
  } else {
    md_.local_bound_min_ = cam_id;
    md_.local_bound_max_ = cam_id;
  }


  // virtual ceiling
  if (mp_.virtual_ceil_height_ > -0.5) {
    int ceil_id =
        (int)std::floor((mp_.virtual_ceil_height_ - mp_.map_origin_(2)) * mp_.resolution_inv_) - 1;

    if (ceil_id >= 0 && ceil_id < mp_.map_voxel_num_(2)) {
      for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
        for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
          md_.occupancy_buffer_inflate_[toAddress(x, y, ceil_id)] = 1;
    }
  }

  // LiDAR 데이터 처리 즉시 grid 발행 → vis_timer_ 지연 없이 points와 동기화
  publishMapInflate(true);
}

void GridMap::publishMap()
{

  if (map_pub_->get_subscription_count() <= 0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  int lmm = mp_.local_map_margin_ / 2;
  min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
  max_cut += Eigen::Vector3i(lmm, lmm, lmm);

  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        if (md_.occupancy_buffer_[toAddress(x, y, z)] < mp_.min_occupancy_log_)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (mp_.visualization_truncate_height_ > mp_.map_min_boundary_(2) &&
            pos(2) > mp_.visualization_truncate_height_)
          continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp = node_->now();
  cloud_msg.header.frame_id = mp_.frame_id_;
  map_pub_->publish(cloud_msg);
}

void GridMap::publishMapInflate(bool all_info)
{

  if (map_inf_pub_->get_subscription_count()<= 0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  if (all_info)
  {
    int lmm = mp_.local_map_margin_;
    min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
    max_cut += Eigen::Vector3i(lmm, lmm, lmm);
  }

  boundIndex(min_cut);
  boundIndex(max_cut);

  // sector filter 시각화 마스킹: full-view recovery 중에는 마스킹 해제
  const int vis_frames_left = sf_full_view_frames_remaining_.load();
  const bool vis_in_full_view = sf_adaptive_recovery_ &&
      ((node_->now() < sf_force_full_view_until_) || (vis_frames_left > 0));
  const bool vis_sector_mask = sf_enable_ && md_.has_odom_ && !vis_in_full_view;
  Eigen::Matrix3d cam_r_inv;
  if (vis_sector_mask) cam_r_inv = md_.camera_r_m_.transpose();

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        if (md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 0)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (mp_.visualization_truncate_height_ > mp_.map_min_boundary_(2) &&
            pos(2) > mp_.visualization_truncate_height_)
          continue;

        if (vis_sector_mask) {
          // world frame 벡터를 드론 local frame으로 변환하여 수평 각도 계산
          Eigen::Vector3d local_dir = cam_r_inv * (pos - md_.camera_pos_);
          const double ang = std::atan2(local_dir(1), local_dir(0));
          if (ang < sf_min_angle_rad_ || ang > sf_max_angle_rad_) continue;
        }

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp = node_->now();
  cloud_msg.header.frame_id = mp_.frame_id_;
  map_inf_pub_->publish(cloud_msg);

  // RCLCPP_INFO(rclcpp::get_logger("publishMapInflate"), "pub map");
}

bool GridMap::odomValid() { return md_.has_odom_; }

bool GridMap::hasDepthObservation() { return md_.has_first_depth_; }

Eigen::Vector3d GridMap::getOrigin() { return mp_.map_origin_; }

// int GridMap::getVoxelNum() {
//   return mp_.map_voxel_num_[0] * mp_.map_voxel_num_[1] * mp_.map_voxel_num_[2];
// }

void GridMap::getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size)
{
  ori = mp_.map_origin_, size = mp_.map_size_;
}

void GridMap::extrinsicCallback(
  const nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
  Eigen::Quaterniond cam2body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                     odom->pose.pose.orientation.x,
                                                     odom->pose.pose.orientation.y,
                                                     odom->pose.pose.orientation.z);
  Eigen::Matrix3d cam2body_r_m = cam2body_q.toRotationMatrix();
  md_.cam2body_.block<3, 3>(0, 0) = cam2body_r_m;
  md_.cam2body_(0, 3) = odom->pose.pose.position.x;
  md_.cam2body_(1, 3) = odom->pose.pose.position.y;
  md_.cam2body_(2, 3) = odom->pose.pose.position.z;
  md_.cam2body_(3, 3) = 1.0;
}

void GridMap::depthOdomCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr img,
  const nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
  md_.last_occ_update_time_ = node_->now();
  md_.flag_depth_odom_timeout_ = false;

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
    "[GridMap] depthOdom cb alive enc=%s size=%ux%u odom_frame=%s",
    img->encoding.c_str(), img->width, img->height,
    odom->header.frame_id.c_str());
  
    /* get pose */
  // --- (교체) odom pose를 mp_.frame_id_로 변환해서 body2world 생성 ---
  const std::string target_frame = mp_.frame_id_;        // 보통 "map"
  const std::string source_frame = odom->header.frame_id; // 보통 "odom"

  geometry_msgs::msg::PoseStamped body_in, body_out;
  body_in.header = odom->header;
  body_in.pose   = odom->pose.pose;

  if (source_frame.empty()) {
    RCLCPP_WARN(node_->get_logger(), "[GridMap] depthOdom: odom frame_id empty, skip");
    return;
  }

  if (source_frame == target_frame) {
    body_out = body_in;
  } else {
    try {
      auto tf = tf_buffer_->lookupTransform(
          target_frame, source_frame,
          rclcpp::Time(odom->header.stamp),
          rclcpp::Duration::from_seconds(0.05));
      tf2::doTransform(body_in, body_out, tf);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "[GridMap] depthOdom TF lookup failed (%s->%s) stamp=%.3f err=%s. Fallback latest.",
        source_frame.c_str(), target_frame.c_str(),
        rclcpp::Time(odom->header.stamp).seconds(), ex.what());
      try {
        auto tf_latest = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
        tf2::doTransform(body_in, body_out, tf_latest);
      } catch (const tf2::TransformException &ex2) {
        // Keep depth fusion alive with raw odom pose fallback.
        body_out = body_in;
        RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 2000,
          "[GridMap] depthOdom TF fallback failed (%s->%s): %s. Using raw odom pose.",
          source_frame.c_str(), target_frame.c_str(), ex2.what());
      }
    }
  }

  Eigen::Quaterniond body_q(body_out.pose.orientation.w,
                            body_out.pose.orientation.x,
                            body_out.pose.orientation.y,
                            body_out.pose.orientation.z);

  Eigen::Matrix4d body2world = Eigen::Matrix4d::Identity();
  body2world.block<3,3>(0,0) = body_q.toRotationMatrix();
  body2world(0,3) = body_out.pose.position.x;
  body2world(1,3) = body_out.pose.position.y;
  body2world(2,3) = body_out.pose.position.z;

  Eigen::Matrix4d cam_T = body2world * md_.cam2body_;
  md_.camera_pos_(0) = cam_T(0, 3);
  md_.camera_pos_(1) = cam_T(1, 3);
  md_.camera_pos_(2) = cam_T(2, 3);
  md_.camera_r_m_ = cam_T.block<3, 3>(0, 0);

  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    double scale = mp_.k_depth_scaling_factor_;
    if (scale <= 0.0) scale = 1000.0;
    cv_ptr->image.convertTo(cv_ptr->image, CV_16UC1, scale);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  md_.flag_use_depth_fusion = true;

  // ✅ pose가 들어왔으니 odom 유효
  md_.has_odom_ = true;

  // ✅ map 업데이트 트리거 (맵 안에 있을 때만)
  const bool in_map = isInMap(md_.camera_pos_);
  if (in_map) {
    md_.update_num_ += 1;
    md_.occ_need_update_ = true;
  } else {
    md_.occ_need_update_ = false;
  }

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
    "[GridMap] depthOdom fused cam=(%.2f %.2f %.2f) in_map=%d occ_need_update=%d",
    md_.camera_pos_.x(), md_.camera_pos_.y(), md_.camera_pos_.z(),
    (int)in_map, (int)md_.occ_need_update_);
}
