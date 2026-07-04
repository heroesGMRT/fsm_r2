// KFS cube localizer for R2 (ABU Robocon 2026).
//
// Consumes the D435i aligned-depth stream and produces one settled,
// multi-frame-averaged centroid of the KFS cube per LocateKfs action goal.
// The centroid is reported in the camera color optical frame; downstream
// (pick_servo_node) transforms it into the gripper frame via TF.
//
// Pipeline per goal:
//   1. Wait for IMU angular velocity to settle (robot stationary).
//   2. For each aligned depth frame: project valid pixels to 3D,
//      PassThrough on Z (depth window) and Y (height band for the target
//      block height), voxel downsample, Euclidean clustering, keep the
//      cluster whose extents match a ~350 mm cube, compute3DCentroid.
//   3. Average centroids over `centroid_avg_frames` valid frames; reject
//      the measurement if the per-frame spread is too large.
//
// The node starts and waits gracefully when no camera topics exist: goals
// are aborted with a clear message after `measure_timeout_s`, never a crash.

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <r2_interfaces/action/locate_kfs.hpp>

using LocateKfs = r2_interfaces::action::LocateKfs;
using GoalHandle = rclcpp_action::ServerGoalHandle<LocateKfs>;
using std::placeholders::_1;
using std::placeholders::_2;

namespace
{

struct FrameMeasurement
{
  Eigen::Vector4f centroid;
  uint32_t points_used;
  rclcpp::Time stamp;
};

}  // namespace

class KfsLocalizerNode : public rclcpp::Node
{
public:
  KfsLocalizerNode()
  : Node("kfs_localizer")
  {
    declareParams();

    auto sensor_qos = rclcpp::SensorDataQoS();

    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, sensor_qos,
      std::bind(&KfsLocalizerNode::depthCallback, this, _1));
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      info_topic_, sensor_qos,
      std::bind(&KfsLocalizerNode::infoCallback, this, _1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, sensor_qos,
      std::bind(&KfsLocalizerNode::imuCallback, this, _1));

    centroid_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      "/r2_perception/kfs_centroid", 10);

    action_server_ = rclcpp_action::create_server<LocateKfs>(
      this, "locate_kfs",
      std::bind(&KfsLocalizerNode::handleGoal, this, _1, _2),
      std::bind(&KfsLocalizerNode::handleCancel, this, _1),
      std::bind(&KfsLocalizerNode::handleAccepted, this, _1));

    RCLCPP_INFO(
      get_logger(),
      "kfs_localizer ready. Waiting for goals on 'locate_kfs' "
      "(depth: %s). Node idles gracefully if the camera is not up.",
      depth_topic_.c_str());
  }

private:
  // ── Parameters ──────────────────────────────────────────────────────────
  void declareParams()
  {
    depth_topic_ = declare_parameter<std::string>(
      "depth_topic", "/camera/aligned_depth_to_color/image_raw");
    info_topic_ = declare_parameter<std::string>(
      "camera_info_topic", "/camera/color/camera_info");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/camera/imu");

    min_depth_m_ = declare_parameter<double>("min_depth_m", 0.30);
    max_depth_m_ = declare_parameter<double>("max_depth_m", 1.20);
    centroid_avg_frames_ =
      static_cast<uint32_t>(declare_parameter<int>("centroid_avg_frames", 15));
    imu_settle_threshold_ =
      declare_parameter<double>("imu_settle_threshold_rad_s", 0.05);
    settle_window_s_ = declare_parameter<double>("imu_settle_window_s", 0.5);
    settle_timeout_s_ = declare_parameter<double>("settle_timeout_s", 10.0);
    measure_timeout_s_ = declare_parameter<double>("measure_timeout_s", 15.0);

    cluster_tolerance_m_ = declare_parameter<double>("cluster_tolerance_m", 0.05);
    cluster_min_points_ = declare_parameter<int>("cluster_min_points", 50);
    cluster_max_points_ = declare_parameter<int>("cluster_max_points", 5000);
    cluster_dim_min_m_ = declare_parameter<double>("cluster_dim_min_m", 0.15);
    cluster_dim_max_m_ = declare_parameter<double>("cluster_dim_max_m", 0.45);

    voxel_leaf_m_ = declare_parameter<double>("voxel_leaf_m", 0.01);
    pixel_stride_ = declare_parameter<int>("pixel_stride", 2);
    max_spread_mm_ = declare_parameter<double>("max_spread_mm", 25.0);

    // Vertical (optical-frame Y, +Y down) acceptance band per block height.
    // These depend on the physical camera mount and MUST be tuned on the
    // robot; the wide defaults effectively disable the band until then.
    y_band_a_ = declare_parameter<std::vector<double>>("y_band_a", {-2.0, 2.0});
    y_band_b_ = declare_parameter<std::vector<double>>("y_band_b", {-2.0, 2.0});
    y_band_c_ = declare_parameter<std::vector<double>>("y_band_c", {-2.0, 2.0});
  }

  // ── Subscriptions ───────────────────────────────────────────────────────
  void infoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    fx_ = msg->k[0];
    fy_ = msg->k[4];
    cx_ = msg->k[2];
    cy_ = msg->k[5];
    camera_frame_ = msg->header.frame_id;
    have_info_ = true;
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const auto & w = msg->angular_velocity;
    const double mag = std::sqrt(w.x * w.x + w.y * w.y + w.z * w.z);
    std::lock_guard<std::mutex> lk(mutex_);
    const rclcpp::Time stamp(msg->header.stamp);
    imu_samples_.emplace_back(stamp, mag);
    while (imu_samples_.size() > 1 &&
      (stamp - imu_samples_.front().first).seconds() > settle_window_s_)
    {
      imu_samples_.pop_front();
    }
    have_imu_ = true;
  }

  bool imuSettled()
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (imu_samples_.empty()) {
      return false;
    }
    return std::all_of(
      imu_samples_.begin(), imu_samples_.end(),
      [this](const auto & s) {return s.second < imu_settle_threshold_;});
  }

  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!collecting_.load()) {
      return;
    }
    {
      std::lock_guard<std::mutex> lk(mutex_);
      if (!have_info_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Depth frames arriving but no CameraInfo yet; skipping.");
        return;
      }
      if (measurements_.size() >= centroid_avg_frames_) {
        return;
      }
    }

    FrameMeasurement m;
    if (processFrame(*msg, m)) {
      std::lock_guard<std::mutex> lk(mutex_);
      measurements_.push_back(m);
    }
  }

  // ── Depth frame → candidate cube centroid ───────────────────────────────
  bool processFrame(const sensor_msgs::msg::Image & img, FrameMeasurement & out)
  {
    double depth_scale = 0.0;
    if (img.encoding == "16UC1") {
      depth_scale = 0.001;  // mm → m (RealSense default)
    } else if (img.encoding == "32FC1") {
      depth_scale = 1.0;
    } else {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Unsupported depth encoding '%s'", img.encoding.c_str());
      return false;
    }

    double fx, fy, cx, cy;
    std::vector<double> y_band;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      fx = fx_; fy = fy_; cx = cx_; cy = cy_;
      y_band = active_y_band_;
    }

    const int stride = std::max(1, pixel_stride_);
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud->reserve((img.width / stride) * (img.height / stride) / 4);

    // Project valid pixels inside the depth window and Y band. Doing the
    // PassThrough limits during projection avoids building a full cloud
    // only to discard most of it.
    for (uint32_t v = 0; v < img.height; v += stride) {
      const uint8_t * row = &img.data[v * img.step];
      for (uint32_t u = 0; u < img.width; u += stride) {
        double d;
        if (depth_scale == 1.0) {
          float f;
          std::memcpy(&f, row + u * 4, 4);
          d = f;
        } else {
          uint16_t raw;
          std::memcpy(&raw, row + u * 2, 2);
          d = raw * depth_scale;
        }
        if (!(d >= min_depth_m_ && d <= max_depth_m_)) {
          continue;
        }
        const double x = (u - cx) * d / fx;
        const double y = (v - cy) * d / fy;
        if (y < y_band[0] || y > y_band[1]) {
          continue;
        }
        cloud->emplace_back(
          static_cast<float>(x), static_cast<float>(y), static_cast<float>(d));
      }
    }

    if (cloud->size() < static_cast<size_t>(cluster_min_points_)) {
      return false;
    }

    // Downsample to keep clustering cheap and density uniform.
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(cloud);
    const float leaf = static_cast<float>(voxel_leaf_m_);
    voxel.setLeafSize(leaf, leaf, leaf);
    auto downsampled = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    voxel.filter(*downsampled);

    if (downsampled->size() < static_cast<size_t>(cluster_min_points_)) {
      return false;
    }

    auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    tree->setInputCloud(downsampled);

    std::vector<pcl::PointIndices> clusters;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance_m_);
    ec.setMinClusterSize(cluster_min_points_);
    ec.setMaxClusterSize(cluster_max_points_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(downsampled);
    ec.extract(clusters);

    // Keep the largest cluster whose visible extents are plausible for a
    // 350 mm cube face (partial faces allowed via cluster_dim_min_m).
    const pcl::PointIndices * best = nullptr;
    for (const auto & c : clusters) {
      Eigen::Vector4f min_pt, max_pt;
      pcl::getMinMax3D(*downsampled, c.indices, min_pt, max_pt);
      const double dx = max_pt.x() - min_pt.x();
      const double dy = max_pt.y() - min_pt.y();
      const double dz = max_pt.z() - min_pt.z();
      // Sort the three extents; the two largest describe the visible face.
      std::array<double, 3> dims{dx, dy, dz};
      std::sort(dims.begin(), dims.end());
      const bool face_ok =
        dims[2] >= cluster_dim_min_m_ && dims[2] <= cluster_dim_max_m_ &&
        dims[1] >= cluster_dim_min_m_ && dims[1] <= cluster_dim_max_m_;
      if (!face_ok) {
        continue;
      }
      if (best == nullptr || c.indices.size() > best->indices.size()) {
        best = &c;
      }
    }

    if (best == nullptr) {
      return false;
    }

    Eigen::Vector4f centroid;
    // Cloud contains only valid, in-range points, so the centroid is never
    // polluted by speckle holes on the low-texture cardboard face.
    pcl::compute3DCentroid(*downsampled, *best, centroid);

    out.centroid = centroid;
    out.points_used = static_cast<uint32_t>(best->indices.size());
    out.stamp = rclcpp::Time(img.header.stamp);
    return true;
  }

  // ── Action server ───────────────────────────────────────────────────────
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &, LocateKfs::Goal::ConstSharedPtr goal)
  {
    if (busy_.load()) {
      RCLCPP_WARN(get_logger(), "Rejecting locate_kfs goal: already measuring.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->block_height > LocateKfs::Goal::HEIGHT_C) {
      RCLCPP_ERROR(
        get_logger(), "Rejecting locate_kfs goal: bad block_height %u",
        goal->block_height);
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    busy_.store(true);
    std::thread{std::bind(&KfsLocalizerNode::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<LocateKfs::Result>();
    auto feedback = std::make_shared<LocateKfs::Feedback>();

    {
      std::lock_guard<std::mutex> lk(mutex_);
      switch (goal->block_height) {
        case LocateKfs::Goal::HEIGHT_A: active_y_band_ = y_band_a_; break;
        case LocateKfs::Goal::HEIGHT_B: active_y_band_ = y_band_b_; break;
        default: active_y_band_ = y_band_c_; break;
      }
      measurements_.clear();
    }

    RCLCPP_INFO(
      get_logger(), "locate_kfs: block %d height %u — waiting for IMU settle",
      goal->block_id, goal->block_height);

    const auto abort = [&](const std::string & msg) {
        collecting_.store(false);
        busy_.store(false);
        result->success = false;
        result->message = msg;
        RCLCPP_ERROR(get_logger(), "locate_kfs failed: %s", msg.c_str());
        goal_handle->abort(result);
      };

    // Phase 1: IMU settle. If no IMU stream exists at all (bench testing,
    // sensor fault), proceed after the timeout rather than deadlocking —
    // an unsettled measurement is recoverable, a stuck action server is not.
    const auto settle_deadline =
      now() + rclcpp::Duration::from_seconds(settle_timeout_s_);
    feedback->state = "WAITING_FOR_SETTLE";
    feedback->frames_collected = 0;
    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        collecting_.store(false);
        busy_.store(false);
        result->success = false;
        result->message = "canceled";
        goal_handle->canceled(result);
        return;
      }
      if (imuSettled()) {
        break;
      }
      if (now() >= settle_deadline) {
        bool have_imu;
        {
          std::lock_guard<std::mutex> lk(mutex_);
          have_imu = have_imu_;
        }
        if (have_imu) {
          abort("IMU did not settle within timeout (robot still moving?)");
          return;
        }
        RCLCPP_WARN(
          get_logger(),
          "No IMU messages on %s — proceeding WITHOUT settle check.",
          imu_topic_.c_str());
        break;
      }
      goal_handle->publish_feedback(feedback);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // Phase 2: collect averaged centroid frames.
    feedback->state = "COLLECTING";
    collecting_.store(true);
    const auto measure_deadline =
      now() + rclcpp::Duration::from_seconds(measure_timeout_s_);
    size_t collected = 0;
    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        collecting_.store(false);
        busy_.store(false);
        result->success = false;
        result->message = "canceled";
        goal_handle->canceled(result);
        return;
      }
      {
        std::lock_guard<std::mutex> lk(mutex_);
        collected = measurements_.size();
      }
      if (collected >= centroid_avg_frames_) {
        break;
      }
      if (now() >= measure_deadline) {
        abort(
          "Timed out with " + std::to_string(collected) + "/" +
          std::to_string(centroid_avg_frames_) +
          " valid frames (camera down, cube out of view, or filters too tight)");
        return;
      }
      feedback->frames_collected = static_cast<uint32_t>(collected);
      goal_handle->publish_feedback(feedback);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    collecting_.store(false);

    // Phase 3: average + spread check.
    std::vector<FrameMeasurement> frames;
    std::string camera_frame;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      frames = measurements_;
      camera_frame = camera_frame_;
    }

    Eigen::Vector4f mean = Eigen::Vector4f::Zero();
    for (const auto & f : frames) {
      mean += f.centroid;
    }
    mean /= static_cast<float>(frames.size());

    double max_dev_mm = 0.0;
    for (const auto & f : frames) {
      const double dev = (f.centroid.head<3>() - mean.head<3>()).norm() * 1000.0;
      max_dev_mm = std::max(max_dev_mm, dev);
    }
    if (max_dev_mm > max_spread_mm_) {
      abort(
        "Centroid spread " + std::to_string(max_dev_mm) + " mm exceeds " +
        std::to_string(max_spread_mm_) + " mm (vibration or unstable depth)");
      return;
    }

    geometry_msgs::msg::PointStamped centroid_msg;
    centroid_msg.header.frame_id = camera_frame;
    centroid_msg.header.stamp = frames.back().stamp;
    centroid_msg.point.x = mean.x();
    centroid_msg.point.y = mean.y();
    centroid_msg.point.z = mean.z();
    centroid_pub_->publish(centroid_msg);

    result->success = true;
    result->centroid = centroid_msg;
    result->points_used = frames.back().points_used;
    result->spread_mm = static_cast<float>(max_dev_mm);
    result->message = "ok";
    busy_.store(false);
    RCLCPP_INFO(
      get_logger(),
      "locate_kfs OK: centroid (%.3f, %.3f, %.3f) in %s, spread %.1f mm",
      mean.x(), mean.y(), mean.z(), camera_frame.c_str(), max_dev_mm);
    goal_handle->succeed(result);
  }

  // ── Members ─────────────────────────────────────────────────────────────
  std::string depth_topic_, info_topic_, imu_topic_;
  double min_depth_m_{}, max_depth_m_{};
  uint32_t centroid_avg_frames_{};
  double imu_settle_threshold_{}, settle_window_s_{}, settle_timeout_s_{};
  double measure_timeout_s_{};
  double cluster_tolerance_m_{};
  int cluster_min_points_{}, cluster_max_points_{};
  double cluster_dim_min_m_{}, cluster_dim_max_m_{};
  double voxel_leaf_m_{};
  int pixel_stride_{};
  double max_spread_mm_{};
  std::vector<double> y_band_a_, y_band_b_, y_band_c_;

  std::mutex mutex_;
  bool have_info_{false};
  bool have_imu_{false};
  double fx_{}, fy_{}, cx_{}, cy_{};
  std::string camera_frame_{"camera_color_optical_frame"};
  std::deque<std::pair<rclcpp::Time, double>> imu_samples_;
  std::vector<FrameMeasurement> measurements_;
  std::vector<double> active_y_band_{-2.0, 2.0};

  std::atomic<bool> collecting_{false};
  std::atomic<bool> busy_{false};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr centroid_pub_;
  rclcpp_action::Server<LocateKfs>::SharedPtr action_server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KfsLocalizerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
