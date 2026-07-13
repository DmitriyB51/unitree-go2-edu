// map_matcher_node.cpp
//
// Tracking-only map-matcher localization for Unitree Go2 + Unilidar L1, layered
// on top of Point-LIO (which stays untouched as the odometry engine).
//
// Architecture:
//   Point-LIO publishes /registered_scan (world cloud in frame `camera_init`)
//   and TF camera_init -> aft_mapped (drifting odometry).
//   This node registers an accumulated window of /registered_scan against a
//   prebuilt PCD map and publishes the correction TF  map -> camera_init.
//   Full robot pose in the map is then  map -> camera_init -> aft_mapped.
//
// Because /registered_scan is already expressed in camera_init, accumulating a
// sliding window is just concatenation + voxel downsample (no per-scan pose
// transforms needed). GICP (default) or NDT aligns that window to the map, using
// the current correction as the initial guess. A fitness gate holds the last good
// correction when the match is untrustworthy (= "localization unsure").

#include <atomic>
#include <chrono>
#include <deque>
#include <mutex>
#include <thread>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float32.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>

#include <Eigen/Geometry>

using PointT = pcl::PointXYZ;
using Cloud = pcl::PointCloud<PointT>;

namespace
{
// Build a 4x4 float transform from x,y,z,roll,pitch,yaw.
Eigen::Matrix4f poseToMatrix(double x, double y, double z,
                             double roll, double pitch, double yaw)
{
  Eigen::Affine3f t = Eigen::Affine3f::Identity();
  t.translation() << static_cast<float>(x), static_cast<float>(y), static_cast<float>(z);
  t.rotate(Eigen::AngleAxisf(static_cast<float>(yaw),   Eigen::Vector3f::UnitZ()) *
           Eigen::AngleAxisf(static_cast<float>(pitch), Eigen::Vector3f::UnitY()) *
           Eigen::AngleAxisf(static_cast<float>(roll),  Eigen::Vector3f::UnitX()));
  return t.matrix();
}
}  // namespace

class MapMatcherNode : public rclcpp::Node
{
public:
  MapMatcherNode() : Node("map_matcher_node")
  {
    // ---- parameters ----
    map_path_        = declare_parameter<std::string>("map_path", "/home/dmitriyb51/maps/final_map_lc.pcd");
    map_voxel_       = declare_parameter<double>("map_voxel", 0.15);
    scan_voxel_      = declare_parameter<double>("scan_voxel", 0.10);
    window_sec_      = declare_parameter<double>("window_sec", 1.0);
    match_every_m_   = declare_parameter<double>("match_every_m", 0.3);
    crop_radius_     = declare_parameter<double>("crop_radius", 30.0);
    registration_    = declare_parameter<std::string>("registration", "gicp");
    gicp_max_corr_   = declare_parameter<double>("gicp_max_corr_dist", 1.0);
    gicp_max_iter_   = declare_parameter<int>("gicp_max_iter", 30);
    gicp_tf_eps_     = declare_parameter<double>("gicp_transform_eps", 1e-4);
    ndt_resolution_  = declare_parameter<double>("ndt_resolution", 1.0);
    fitness_thresh_  = declare_parameter<double>("fitness_thresh", 0.3);
    max_jump_        = declare_parameter<double>("max_correction_jump", 1.0);
    map_frame_       = declare_parameter<std::string>("map_frame", "map");
    world_frame_     = declare_parameter<std::string>("world_frame", "camera_init");
    base_frame_      = declare_parameter<std::string>("base_frame", "aft_mapped");
    std::string scan_topic = declare_parameter<std::string>("scan_topic", "/registered_scan");
    std::string odom_topic = declare_parameter<std::string>("odom_topic", "/state_estimation");
    auto init = declare_parameter<std::vector<double>>("initial_pose",
                    std::vector<double>{0, 0, 0, 0, 0, 0});
    if (init.size() != 6) {
      RCLCPP_WARN(get_logger(), "initial_pose must have 6 values, got %zu; using identity", init.size());
      init = {0, 0, 0, 0, 0, 0};
    }
    T_map_cam_ = poseToMatrix(init[0], init[1], init[2], init[3], init[4], init[5]);

    // ---- load + downsample map ----
    if (!loadMap()) {
      RCLCPP_FATAL(get_logger(), "Failed to load map '%s' - shutting down.", map_path_.c_str());
      throw std::runtime_error("map load failed");
    }

    // ---- configure registration; set the map as target ONCE (covariances/voxels cached) ----
    gicp_.setMaxCorrespondenceDistance(gicp_max_corr_);
    gicp_.setMaximumIterations(gicp_max_iter_);
    gicp_.setTransformationEpsilon(gicp_tf_eps_);
    gicp_.setInputTarget(map_ds_);
    ndt_.setResolution(ndt_resolution_);
    ndt_.setMaximumIterations(gicp_max_iter_);
    ndt_.setTransformationEpsilon(gicp_tf_eps_);
    ndt_.setInputTarget(map_ds_);

    // ---- I/O ----
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    rclcpp::QoS scan_qos = rclcpp::SensorDataQoS();

    scan_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        scan_topic, scan_qos,
        std::bind(&MapMatcherNode::onScan, this, std::placeholders::_1));
    // Point-LIO publishes /state_estimation at propagation rate (~7 kHz). Keep only
    // the latest (best-effort, depth 1) and do zero heavy work in the callback.
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&MapMatcherNode::onOdom, this, std::placeholders::_1));
    initpose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", rclcpp::QoS(1),
        std::bind(&MapMatcherNode::onInitialPose, this, std::placeholders::_1));

    pose_pub_    = create_publisher<nav_msgs::msg::Odometry>("/localization/pose", rclcpp::QoS(10));
    aligned_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/localization/aligned_cloud", rclcpp::QoS(1));
    fitness_pub_ = create_publisher<std_msgs::msg::Float32>("/localization/fitness", rclcpp::QoS(10));

    // Prior map published latched (transient-local) so RViz always shows it as a fixed
    // background, even if it connects after startup. Uses the already-downsampled map.
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/localization/map", rclcpp::QoS(1).transient_local().reliable());
    {
      sensor_msgs::msg::PointCloud2 m;
      pcl::toROSMsg(*map_ds_, m);
      m.header.frame_id = map_frame_;
      m.header.stamp = now();
      map_pub_->publish(m);
      RCLCPP_INFO(get_logger(), "published prior map (%zu pts) latched on /localization/map", map_ds_->size());
    }

    // TF + pose published at a fixed 50 Hz, decoupled from the ~7 kHz odom stream.
    pub_timer_ = create_wall_timer(std::chrono::milliseconds(20),
                                   std::bind(&MapMatcherNode::publishTf, this));

    // Heavy registration runs on its OWN thread so a slow align never stalls TF/pose.
    match_thread_ = std::thread([this] { matchLoop(); });

    RCLCPP_INFO(get_logger(), "map_matcher_node up. registration=%s, map_voxel=%.2f, window=%.1fs, match_every=%.2fm",
                registration_.c_str(), map_voxel_, window_sec_, match_every_m_);
    RCLCPP_INFO(get_logger(), "initial correction map->camera_init = [%.3f %.3f %.3f]",
                T_map_cam_(0,3), T_map_cam_(1,3), T_map_cam_(2,3));
  }

  ~MapMatcherNode() override
  {
    stop_ = true;
    if (match_thread_.joinable()) match_thread_.join();
  }

private:
  bool loadMap()
  {
    auto raw = std::make_shared<Cloud>();
    if (pcl::io::loadPCDFile<PointT>(map_path_, *raw) != 0 || raw->empty()) {
      return false;
    }
    map_ds_ = std::make_shared<Cloud>();
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(raw);
    vg.setLeafSize(map_voxel_, map_voxel_, map_voxel_);
    vg.filter(*map_ds_);
    RCLCPP_INFO(get_logger(), "loaded %zu pts -> downsampled to %zu (voxel %.2f m)",
                raw->size(), map_ds_->size(), map_voxel_);
    return !map_ds_->empty();
  }

  void onScan(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    auto cloud = std::make_shared<Cloud>();
    pcl::fromROSMsg(*msg, *cloud);
    if (cloud->empty()) return;

    const double t = rclcpp::Time(msg->header.stamp).seconds();
    std::lock_guard<std::mutex> lk(mtx_);
    window_.push_back({t, cloud});
    // prune scans older than window_sec relative to the newest
    while (!window_.empty() && (t - window_.front().stamp) > window_sec_) {
      window_.pop_front();
    }
  }

  // Trivial: just cache the latest odom pose (camera_init -> aft_mapped). Called at ~7 kHz.
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    Eigen::Vector3f p(msg->pose.pose.position.x,
                      msg->pose.pose.position.y,
                      msg->pose.pose.position.z);
    Eigen::Quaternionf q(msg->pose.pose.orientation.w,
                         msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z);
    Eigen::Matrix4f T_cam_base = Eigen::Matrix4f::Identity();
    T_cam_base.block<3,3>(0,0) = q.normalized().toRotationMatrix();
    T_cam_base.block<3,1>(0,3) = p;

    std::lock_guard<std::mutex> lk(mtx_);
    odom_pos_cam_ = p;
    T_cam_base_ = T_cam_base;
    odom_stamp_ = msg->header.stamp;
    have_odom_ = true;
  }

  // 50 Hz: broadcast correction map->camera_init and publish full pose map->aft_mapped.
  void publishTf()
  {
    Eigen::Matrix4f T_map_cam, T_cam_base;
    rclcpp::Time stamp;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (!have_odom_) return;
      T_map_cam = T_map_cam_;
      T_cam_base = T_cam_base_;
      stamp = odom_stamp_;
    }
    publishCorrectionTf(stamp, T_map_cam);
    publishPose(stamp, T_map_cam * T_cam_base);
  }

  void onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    double r, p, y; tf2::Matrix3x3(q).getRPY(r, p, y);
    Eigen::Matrix4f T = poseToMatrix(msg->pose.pose.position.x, msg->pose.pose.position.y,
                                     msg->pose.pose.position.z, r, p, y);
    std::lock_guard<std::mutex> lk(mtx_);
    T_map_cam_ = T;
    force_match_ = true;
    RCLCPP_INFO(get_logger(), "initial pose reset from /initialpose (x=%.2f y=%.2f yaw=%.1f deg)",
                msg->pose.pose.position.x, msg->pose.pose.position.y, y * 180.0 / M_PI);
  }

  void matchLoop()
  {
    while (rclcpp::ok() && !stop_) {
      tryMatch();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

  void tryMatch()
  {
    // --- snapshot shared state ---
    Cloud::Ptr src = std::make_shared<Cloud>();
    Eigen::Matrix4f guess;
    Eigen::Vector3f odom_pos;
    bool force;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (!have_odom_ || window_.empty()) return;
      const double moved = have_last_match_ ? (odom_pos_cam_ - last_match_pos_).norm() : 1e9;
      force = force_match_;
      if (!force && have_last_match_ && moved < match_every_m_) return;  // travel gate
      for (auto & e : window_) *src += *e.cloud;
      guess = T_map_cam_;
      odom_pos = odom_pos_cam_;
      force_match_ = false;
    }

    if (src->empty()) return;

    // Downsample the accumulated window.
    Cloud::Ptr src_ds = std::make_shared<Cloud>();
    { pcl::VoxelGrid<PointT> vg; vg.setInputCloud(src);
      vg.setLeafSize(scan_voxel_, scan_voxel_, scan_voxel_); vg.filter(*src_ds); }
    if (src_ds->size() < 50) {
      RCLCPP_WARN(get_logger(), "accumulated window too small (%zu pts), skipping match", src_ds->size());
      return;
    }

    // --- register: source(camera_init) -> target(map, set once at startup), guess = correction ---
    // The registration target (whole downsampled map) is set once in the ctor so GICP/NDT
    // precompute target covariances / voxel grid a single time; each match only sets the
    // small source cloud + aligns. This keeps matching fast enough to run at ~4 Hz.
    Cloud aligned;
    Eigen::Matrix4f result = guess;
    bool converged = false;
    double fitness = 1e9;
    const auto t0 = std::chrono::steady_clock::now();

    if (registration_ == "ndt") {
      ndt_.setInputSource(src_ds);
      ndt_.align(aligned, guess);
      converged = ndt_.hasConverged();
      result = ndt_.getFinalTransformation();
      fitness = ndt_.getFitnessScore();
    } else {
      gicp_.setInputSource(src_ds);
      gicp_.align(aligned, guess);
      converged = gicp_.hasConverged();
      result = gicp_.getFinalTransformation();
      fitness = gicp_.getFitnessScore();
    }
    const double ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();
    RCLCPP_DEBUG(get_logger(), "align %.1f ms (src=%zu)", ms, src_ds->size());

    // publish fitness always (diagnostics)
    { std_msgs::msg::Float32 f; f.data = static_cast<float>(fitness); fitness_pub_->publish(f); }

    // --- health gate ---
    // How far this match would move the correction. Drift correction is smooth (cm); a
    // big jump means GICP found a wrong minimum (e.g. slid through a wall) -> reject it.
    // Exempt the first lock and manual /initialpose resets (force), which are legitimately large.
    const double jump = (result.block<3,1>(0,3) - guess.block<3,1>(0,3)).norm();
    const bool ok_fitness = converged && fitness < fitness_thresh_;
    const bool ok_jump = force || !have_last_match_ || jump <= max_jump_;

    if (ok_fitness && ok_jump) {
      {
        std::lock_guard<std::mutex> lk(mtx_);
        T_map_cam_ = result;
        last_match_pos_ = odom_pos;
        have_last_match_ = true;
      }
      if (aligned_pub_->get_subscription_count() > 0) {
        sensor_msgs::msg::PointCloud2 out;
        pcl::toROSMsg(aligned, out);
        out.header.frame_id = map_frame_;
        out.header.stamp = now();
        aligned_pub_->publish(out);
      }
      RCLCPP_DEBUG(get_logger(), "match ok: fitness=%.4f jump=%.3f pts src=%zu", fitness, jump, src_ds->size());
    } else if (ok_fitness && !ok_jump) {
      RCLCPP_WARN(get_logger(),
                  "match REJECTED: implausible jump %.2f m (> %.2f), fitness=%.4f - holding last correction.",
                  jump, max_jump_, fitness);
    } else {
      RCLCPP_WARN(get_logger(),
                  "match REJECTED (converged=%d fitness=%.4f >= %.4f) - holding last correction. Localization unsure.",
                  converged, fitness, fitness_thresh_);
    }
  }

  void publishCorrectionTf(const rclcpp::Time & stamp, const Eigen::Matrix4f & T)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = map_frame_;      // parent
    tf.child_frame_id = world_frame_;     // child = camera_init
    tf.transform.translation.x = T(0,3);
    tf.transform.translation.y = T(1,3);
    tf.transform.translation.z = T(2,3);
    Eigen::Quaternionf q(T.block<3,3>(0,0));
    q.normalize();
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf);
  }

  void publishPose(const rclcpp::Time & stamp, const Eigen::Matrix4f & T_map_base)
  {
    nav_msgs::msg::Odometry od;
    od.header.stamp = stamp;
    od.header.frame_id = map_frame_;
    od.child_frame_id = base_frame_;
    od.pose.pose.position.x = T_map_base(0,3);
    od.pose.pose.position.y = T_map_base(1,3);
    od.pose.pose.position.z = T_map_base(2,3);
    Eigen::Quaternionf q(T_map_base.block<3,3>(0,0));
    q.normalize();
    od.pose.pose.orientation.x = q.x();
    od.pose.pose.orientation.y = q.y();
    od.pose.pose.orientation.z = q.z();
    od.pose.pose.orientation.w = q.w();
    pose_pub_->publish(od);
  }

  // ---- params ----
  std::string map_path_, registration_, map_frame_, world_frame_, base_frame_;
  double map_voxel_, scan_voxel_, window_sec_, match_every_m_, crop_radius_;
  double gicp_max_corr_, gicp_tf_eps_, ndt_resolution_, fitness_thresh_, max_jump_;
  int gicp_max_iter_;

  // ---- map + persistent registration (target set once) ----
  Cloud::Ptr map_ds_;
  pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp_;
  pcl::NormalDistributionsTransform<PointT, PointT> ndt_;

  // ---- sliding window ----
  struct Stamped { double stamp; Cloud::Ptr cloud; };
  std::deque<Stamped> window_;

  // ---- state (guarded by mtx_) ----
  std::mutex mtx_;
  Eigen::Matrix4f T_map_cam_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f T_cam_base_ = Eigen::Matrix4f::Identity();
  Eigen::Vector3f odom_pos_cam_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f last_match_pos_ = Eigen::Vector3f::Zero();
  rclcpp::Time odom_stamp_;
  bool have_odom_ = false;
  bool have_last_match_ = false;
  bool force_match_ = false;

  // ---- I/O ----
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initpose_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr fitness_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  std::thread match_thread_;
  std::atomic<bool> stop_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMatcherNode>());
  rclcpp::shutdown();
  return 0;
}
