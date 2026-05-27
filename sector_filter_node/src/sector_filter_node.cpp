#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <chrono>
#include <vector>
#include <string>

class SectorFilterNode : public rclcpp::Node
{
public:
  SectorFilterNode() : rclcpp::Node("sector_filter_node")
  {
    // --- params (defaults) ---
    this->declare_parameter<std::string>("input_topic",  "/livox/lidar");
    this->declare_parameter<std::string>("output_topic", "/filtered_cloud");

    // 각도(θ) 필터만 사용
    this->declare_parameter<double>("min_angle_deg", -60.0);
    this->declare_parameter<double>("max_angle_deg",  60.0);

    // 퍼포먼스용 옵션만 유지
    this->declare_parameter<bool>("use_stride", false);
    this->declare_parameter<int>("stride", 1);
    this->declare_parameter<bool>("use_voxel", false);
    this->declare_parameter<double>("voxel_leaf", 0.10);

    // Adaptive recovery: disable sector filtering when planner keeps failing.
    this->declare_parameter<bool>("adaptive_recovery", true);
    this->declare_parameter<std::string>("replan_status_topic", "/planning/replan_success");
    this->declare_parameter<int>("replan_fail_threshold", 3);
    this->declare_parameter<double>("full_view_hold_sec", 3.0);
    this->declare_parameter<bool>("replan_burst_enable", true);
    this->declare_parameter<std::string>("replan_attempt_topic", "/planning/replan_attempt");
    this->declare_parameter<double>("replan_burst_hold_sec", 0.8);
    this->declare_parameter<double>("replan_burst_cooldown_sec", 2.0);

    // Adaptive recovery: disable sector filtering when front is heavily blocked.
    this->declare_parameter<bool>("auto_disable_on_front_block", true);
    this->declare_parameter<double>("front_block_dist", 1.2);
    this->declare_parameter<int>("front_block_points_threshold", 20);
    this->declare_parameter<double>("front_block_half_angle_deg", 20.0);

    input_topic_  = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    refresh_params();

    // sub / pub
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SectorFilterNode::cloudCallback, this, std::placeholders::_1));

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    qos.durability(rclcpp::DurabilityPolicy::Volatile);
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);

    if (adaptive_recovery_) {
      replan_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        replan_status_topic_, 10,
        std::bind(&SectorFilterNode::replanStatusCallback, this, std::placeholders::_1));

      if (replan_burst_enable_) {
        replan_attempt_sub_ = this->create_subscription<std_msgs::msg::Empty>(
          replan_attempt_topic_, 10,
          std::bind(&SectorFilterNode::replanAttemptCallback, this, std::placeholders::_1));
      }
    }

    RCLCPP_INFO(get_logger(),
      "SectorFilterNode: in='%s' -> out='%s' | angle=[%.1f, %.1f]deg | use_stride=%d(stride=%d) | use_voxel=%d(leaf=%.2f) | adaptive=%d fail_topic='%s' burst=%d",
      input_topic_.c_str(), output_topic_.c_str(),
      rad2deg(min_angle_rad_), rad2deg(max_angle_rad_),
      use_stride_?1:0, stride_, use_voxel_?1:0, voxel_leaf_,
      adaptive_recovery_ ? 1 : 0, replan_status_topic_.c_str(), replan_burst_enable_ ? 1 : 0);
  }

private:
  static inline double deg2rad(double d){ return d * M_PI / 180.0; }
  static inline double rad2deg(double r){ return r * 180.0 / M_PI; }

  void refresh_params()
  {
    min_angle_rad_ = deg2rad(get_parameter("min_angle_deg").as_double());
    max_angle_rad_ = deg2rad(get_parameter("max_angle_deg").as_double());

    use_stride_ = get_parameter("use_stride").as_bool();
    stride_     = get_parameter("stride").as_int();
    if (stride_ < 1) stride_ = 1;

    use_voxel_  = get_parameter("use_voxel").as_bool();
    voxel_leaf_ = get_parameter("voxel_leaf").as_double();
    if (voxel_leaf_ <= 0.0) voxel_leaf_ = 0.10;

    adaptive_recovery_ = get_parameter("adaptive_recovery").as_bool();
    replan_status_topic_ = get_parameter("replan_status_topic").as_string();
    replan_fail_threshold_ = get_parameter("replan_fail_threshold").as_int();
    if (replan_fail_threshold_ < 1) replan_fail_threshold_ = 1;
    full_view_hold_sec_ = get_parameter("full_view_hold_sec").as_double();
    if (full_view_hold_sec_ < 0.1) full_view_hold_sec_ = 0.1;
    replan_burst_enable_ = get_parameter("replan_burst_enable").as_bool();
    replan_attempt_topic_ = get_parameter("replan_attempt_topic").as_string();
    replan_burst_hold_sec_ = get_parameter("replan_burst_hold_sec").as_double();
    if (replan_burst_hold_sec_ < 0.1) replan_burst_hold_sec_ = 0.1;
    replan_burst_cooldown_sec_ = get_parameter("replan_burst_cooldown_sec").as_double();
    if (replan_burst_cooldown_sec_ < 0.0) replan_burst_cooldown_sec_ = 0.0;

    auto_disable_on_front_block_ = get_parameter("auto_disable_on_front_block").as_bool();
    front_block_dist_ = get_parameter("front_block_dist").as_double();
    if (front_block_dist_ < 0.1) front_block_dist_ = 0.1;
    front_block_points_threshold_ = get_parameter("front_block_points_threshold").as_int();
    if (front_block_points_threshold_ < 1) front_block_points_threshold_ = 1;
    front_block_half_angle_rad_ = deg2rad(get_parameter("front_block_half_angle_deg").as_double());
    if (front_block_half_angle_rad_ < 0.0) front_block_half_angle_rad_ = 0.0;
    if (front_block_half_angle_rad_ > M_PI) front_block_half_angle_rad_ = M_PI;
  }

  void armFullViewRecovery(const std::string& reason, double hold_sec = -1.0)
  {
    const double use_hold_sec = (hold_sec > 0.0) ? hold_sec : full_view_hold_sec_;
    const auto now = this->get_clock()->now();
    const auto until = now + rclcpp::Duration::from_seconds(use_hold_sec);
    if (until > force_full_view_until_) {
      force_full_view_until_ = until;
    }
    RCLCPP_WARN(
      get_logger(),
      "Adaptive recovery ON for %.2fs (%s): publish full-view cloud",
      use_hold_sec, reason.c_str());
  }

  void replanStatusCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      consecutive_replan_failures_ = 0;
      return;
    }

    consecutive_replan_failures_++;
    if (consecutive_replan_failures_ >= replan_fail_threshold_) {
      armFullViewRecovery("replan failures");
      consecutive_replan_failures_ = 0;
    }
  }

  void replanAttemptCallback(const std_msgs::msg::Empty::SharedPtr /*msg*/)
  {
    const auto now = this->get_clock()->now();
    if (now < next_replan_burst_allowed_) {
      return;
    }
    armFullViewRecovery("replan burst", replan_burst_hold_sec_);
    next_replan_burst_allowed_ = now + rclcpp::Duration::from_seconds(replan_burst_cooldown_sec_);
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg_in)
  {
    // x/y/z 오프셋 찾기
    int x_off=-1, y_off=-1, z_off=-1;
    for (const auto & field : msg_in->fields) {
      if (field.name == "x") x_off = field.offset;
      else if (field.name == "y") y_off = field.offset;
      else if (field.name == "z") z_off = field.offset;
    }
    if (x_off < 0 || y_off < 0 || z_off < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No x/y/z fields. Skip frame.");
      return;
    }

    const uint8_t* raw = msg_in->data.data();
    const size_t point_step   = msg_in->point_step;
    const size_t total_points = msg_in->data.size() / point_step;

    bool force_full_view = false;
    if (adaptive_recovery_) {
      force_full_view = (this->get_clock()->now() < force_full_view_until_);
    }

    std::vector<uint8_t> filtered_bytes;
    filtered_bytes.reserve(msg_in->data.size());

    int front_block_points = 0;
    size_t kept = 0;
    for (size_t i = 0; i < total_points; ++i) {
      if (use_stride_ && (i % static_cast<size_t>(stride_) != 0)) continue;

      const uint8_t* p = raw + i * point_step;
      float px = *reinterpret_cast<const float*>(p + x_off);
      float py = *reinterpret_cast<const float*>(p + y_off);
      // float pz = *reinterpret_cast<const float*>(p + z_off); // z는 사용 안 함

      const double ang = std::atan2(py, px);  // [-pi, pi]

      if (adaptive_recovery_ && auto_disable_on_front_block_) {
        const double range_xy = std::hypot(px, py);
        if (range_xy < front_block_dist_ && std::fabs(ang) <= front_block_half_angle_rad_) {
          ++front_block_points;
        }
      }

      if (!force_full_view) {
        if (ang < min_angle_rad_ || ang > max_angle_rad_) continue;
      }

      filtered_bytes.insert(filtered_bytes.end(), p, p + point_step);
      ++kept;
    }

    if (!force_full_view && adaptive_recovery_ && auto_disable_on_front_block_ &&
        front_block_points >= front_block_points_threshold_) {
      armFullViewRecovery("front blocked");
      force_full_view = true;

      // Rebuild as full-view in the same frame to react immediately.
      filtered_bytes.clear();
      kept = 0;
      for (size_t i = 0; i < total_points; ++i) {
        if (use_stride_ && (i % static_cast<size_t>(stride_) != 0)) continue;
        const uint8_t* p = raw + i * point_step;
        filtered_bytes.insert(filtered_bytes.end(), p, p + point_step);
        ++kept;
      }
    }

    sensor_msgs::msg::PointCloud2 out = *msg_in;  // 헤더/필드/포맷 유지
    out.height   = 1;
    out.width    = static_cast<uint32_t>(kept);
    out.is_dense = false;
    out.data     = std::move(filtered_bytes);
    out.row_step = out.width * out.point_step;

    if (use_voxel_ && out.width > 0) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_in(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_out(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::fromROSMsg(out, *pcl_in);

      pcl::VoxelGrid<pcl::PointXYZI> vg;
      vg.setInputCloud(pcl_in);
      vg.setLeafSize(static_cast<float>(voxel_leaf_),
                     static_cast<float>(voxel_leaf_),
                     static_cast<float>(voxel_leaf_));
      vg.filter(*pcl_out);

      sensor_msgs::msg::PointCloud2 out_voxel;
      pcl::toROSMsg(*pcl_out, out_voxel);
      out_voxel.header = msg_in->header; // frame 유지
      pub_->publish(out_voxel);
    } else {
      pub_->publish(out);
    }
  }

  // params cache
  std::string input_topic_;
  std::string output_topic_;
  double min_angle_rad_{-M_PI/3.0};
  double max_angle_rad_{ M_PI/3.0};
  bool   use_stride_{false};
  int    stride_{1};
  bool   use_voxel_{false};
  double voxel_leaf_{0.10};

  bool adaptive_recovery_{true};
  std::string replan_status_topic_{"/planning/replan_success"};
  int replan_fail_threshold_{3};
  double full_view_hold_sec_{3.0};
  bool replan_burst_enable_{true};
  std::string replan_attempt_topic_{"/planning/replan_attempt"};
  double replan_burst_hold_sec_{0.8};
  double replan_burst_cooldown_sec_{2.0};
  bool auto_disable_on_front_block_{true};
  double front_block_dist_{1.2};
  int front_block_points_threshold_{20};
  double front_block_half_angle_rad_{20.0 * M_PI / 180.0};
  int consecutive_replan_failures_{0};
  rclcpp::Time force_full_view_until_{0, 0, RCL_ROS_TIME};
  rclcpp::Time next_replan_burst_allowed_{0, 0, RCL_ROS_TIME};

  // ros
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr replan_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr replan_attempt_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr     pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SectorFilterNode>());
  rclcpp::shutdown();
  return 0;
}
