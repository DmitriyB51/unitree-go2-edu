/* plio2sm: convert a Point-LIO output rosbag2 into a MOLA/MRPT .simplemap.
 *
 * Inputs (topics in the bag):
 *   - odometry  (nav_msgs/Odometry)      : per-scan poses  T_world_from_body
 *   - cloud     (sensor_msgs/PointCloud2): scan in body frame (already deskewed)
 *
 * Output: a .simplemap where each keyframe = (pose PDF, CObservationPointCloud).
 * The cloud is stored in the sensor (body) frame with sensorPose=Identity, so
 * keyframe placement = odometry pose. This is exactly the structure MOLA-LO
 * builds internally; only the pose source differs (Point-LIO instead of ICP).
 */

#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/time.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/ros2bridge/imu.h>
#include <mrpt/ros2bridge/point_cloud2.h>
#include <mrpt/ros2bridge/pose.h>
#include <mrpt/ros2bridge/time.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

static double stampSec(const builtin_interfaces::msg::Time& t)
{
  return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
}

int main(int argc, char** argv)
{
  if (argc < 3)
  {
    std::printf(
        "Usage: %s <bag_dir> <out.simplemap> [kf_dist_m=0.5] "
        "[submap_radius_m=2.0] [odom_topic=/state_estimation] "
        "[cloud_topic=/cloud_registered_body] [imu_topic=/utlidar/imu]\n",
        argv[0]);
    return 1;
  }
  const std::string bagDir = argv[1];
  const std::string outFile = argv[2];
  const double kfDist = (argc > 3) ? std::stod(argv[3]) : 0.5;
  const double submapRadius = (argc > 4) ? std::stod(argv[4]) : 2.0;
  const std::string odomTopic = (argc > 5) ? argv[5] : "/state_estimation";
  const std::string cloudTopic = (argc > 6) ? argv[6] : "/cloud_registered_body";
  const std::string imuTopic = (argc > 7) ? argv[7] : "/utlidar/imu";
  const int POSE_SUBSAMPLE = 5;  // keep every 5th odom msg (>1kHz -> plenty)

  rosbag2_storage::StorageOptions so;
  so.uri = bagDir;
  so.storage_id = "sqlite3";
  rosbag2_cpp::ConverterOptions co;
  co.input_serialization_format = "cdr";
  co.output_serialization_format = "cdr";

  rosbag2_cpp::readers::SequentialReader reader;
  reader.open(so, co);

  rclcpp::Serialization<nav_msgs::msg::Odometry> odomSer;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pcSer;
  rclcpp::Serialization<sensor_msgs::msg::Imu> imuSer;

  std::vector<std::pair<double, mrpt::poses::CPose3D>> poses;
  struct CloudE
  {
    double t;
    mrpt::obs::CObservationPointCloud::Ptr obs;
  };
  std::vector<CloudE> clouds;
  std::vector<std::pair<double, mrpt::obs::CObservationIMU::Ptr>> imus;

  size_t poseMsgCount = 0;
  while (reader.has_next())
  {
    auto bagMsg = reader.read_next();
    if (bagMsg->topic_name == odomTopic)
    {
      if ((poseMsgCount++ % POSE_SUBSAMPLE) != 0) continue;
      rclcpp::SerializedMessage s(*bagMsg->serialized_data);
      nav_msgs::msg::Odometry m;
      odomSer.deserialize_message(&s, &m);
      poses.emplace_back(
          stampSec(m.header.stamp), mrpt::ros2bridge::fromROS(m.pose.pose));
    }
    else if (bagMsg->topic_name == cloudTopic)
    {
      rclcpp::SerializedMessage s(*bagMsg->serialized_data);
      sensor_msgs::msg::PointCloud2 m;
      pcSer.deserialize_message(&s, &m);

      auto pts = mrpt::maps::CGenericPointsMap::Create();
      mrpt::ros2bridge::fromROS(m, *pts);

      auto obs = mrpt::obs::CObservationPointCloud::Create();
      obs->sensorLabel = "lidar";
      obs->pointcloud = pts;
      obs->sensorPose = mrpt::poses::CPose3D::Identity();
      obs->timestamp = mrpt::ros2bridge::fromROS(rclcpp::Time(m.header.stamp));

      clouds.push_back({stampSec(m.header.stamp), obs});
    }
    else if (bagMsg->topic_name == imuTopic)
    {
      rclcpp::SerializedMessage s(*bagMsg->serialized_data);
      sensor_msgs::msg::Imu m;
      imuSer.deserialize_message(&s, &m);

      auto io = mrpt::obs::CObservationIMU::Create();
      if (mrpt::ros2bridge::fromROS(m, *io))
      {
        io->sensorLabel = "imu";
        io->sensorPose = mrpt::poses::CPose3D::Identity();
        io->timestamp = mrpt::ros2bridge::fromROS(rclcpp::Time(m.header.stamp));
        imus.emplace_back(stampSec(m.header.stamp), io);
      }
    }
  }
  std::printf(
      "Read %zu poses (subsampled 1/%d), %zu clouds, %zu imu\n", poses.size(),
      POSE_SUBSAMPLE, clouds.size(), imus.size());
  if (poses.empty() || clouds.empty())
  {
    std::printf("ERROR: missing poses or clouds (check topic names)\n");
    return 2;
  }

  std::sort(
      poses.begin(), poses.end(),
      [](const auto& a, const auto& b) { return a.first < b.first; });
  std::sort(
      imus.begin(), imus.end(),
      [](const auto& a, const auto& b) { return a.first < b.first; });

  auto nearestIMU = [&](double t) -> mrpt::obs::CObservationIMU::Ptr
  {
    if (imus.empty()) return nullptr;
    auto it = std::lower_bound(
        imus.begin(), imus.end(), t,
        [](const std::pair<double, mrpt::obs::CObservationIMU::Ptr>& p, double v)
        { return p.first < v; });
    if (it == imus.begin()) return it->second;
    if (it == imus.end()) return (imus.end() - 1)->second;
    auto prev = it - 1;
    return (t - prev->first <= it->first - t) ? prev->second : it->second;
  };

  auto nearestPose = [&](double t) -> const mrpt::poses::CPose3D& {
    auto it = std::lower_bound(
        poses.begin(), poses.end(), t,
        [](const std::pair<double, mrpt::poses::CPose3D>& p, double v) {
          return p.first < v;
        });
    if (it == poses.begin()) return it->second;
    if (it == poses.end()) return (poses.end() - 1)->second;
    auto prev = it - 1;
    return (t - prev->first <= it->first - t) ? prev->second : it->second;
  };

  // Precompute the (nearest) pose for every scan.
  std::vector<mrpt::poses::CPose3D> cloudPose(clouds.size());
  for (size_t i = 0; i < clouds.size(); ++i)
    cloudPose[i] = nearestPose(clouds[i].t);

  // Cumulative path length (monotonic non-decreasing) for path-window submaps.
  std::vector<double> pathLen(clouds.size(), 0.0);
  for (size_t i = 1; i < clouds.size(); ++i)
  {
    const double dx = cloudPose[i].x() - cloudPose[i - 1].x();
    const double dy = cloudPose[i].y() - cloudPose[i - 1].y();
    const double dz = cloudPose[i].z() - cloudPose[i - 1].z();
    pathLen[i] = pathLen[i - 1] + std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  // Anchors every kfDist of path length; each keyframe accumulates ALL scans
  // within +/- submapRadius of path length around the anchor (a local submap
  // from the SAME pass -> consistent poses). Dense, well-structured, overlapping
  // clouds -> far more robust loop-closure ICP than thin single-scan keyframes.
  mrpt::maps::CSimpleMap sm;
  size_t kf = 0;

  auto emitKeyframe = [&](size_t anchorIdx)
  {
    const mrpt::poses::CPose3D& kfP = cloudPose[anchorIdx];
    const double s0 = pathLen[anchorIdx];
    const size_t lo = static_cast<size_t>(
        std::lower_bound(pathLen.begin(), pathLen.end(), s0 - submapRadius) -
        pathLen.begin());
    const size_t hi = static_cast<size_t>(
        std::upper_bound(pathLen.begin(), pathLen.end(), s0 + submapRadius) -
        pathLen.begin());

    auto merged = mrpt::maps::CGenericPointsMap::Create();
    merged->insertionOptions.minDistBetweenLaserPoints = 0.05f;  // cap density
    for (size_t j = lo; j < hi; ++j)
    {
      const mrpt::poses::CPose3D rel = cloudPose[j] - kfP;  // j frame in kf frame
      merged->insertAnotherMap(clouds[j].obs->pointcloud.get(), rel);
    }
    auto obs = mrpt::obs::CObservationPointCloud::Create();
    obs->sensorLabel = "lidar";
    obs->pointcloud = merged;
    obs->sensorPose = mrpt::poses::CPose3D::Identity();
    obs->timestamp = clouds[anchorIdx].obs->timestamp;

    auto sf = mrpt::obs::CSensoryFrame::Create();
    sf->insert(obs);
    if (auto imuObs = nearestIMU(clouds[anchorIdx].t)) sf->insert(imuObs);
    auto pdf = mrpt::poses::CPose3DPDFGaussian::Create(kfP);
    pdf->cov.setIdentity();
    pdf->cov *= 1e-4;
    sm.insert(pdf, sf);
    ++kf;
  };

  if (!clouds.empty())
  {
    double lastAnchorS = -1e18;
    for (size_t i = 0; i < clouds.size(); ++i)
    {
      if (pathLen[i] - lastAnchorS >= kfDist)
      {
        emitKeyframe(i);
        lastAnchorS = pathLen[i];
      }
    }
  }
  std::printf(
      "Built simplemap: %zu keyframes (kf_dist=%.2f m, submap_radius=%.2f m)\n",
      kf, kfDist, submapRadius);

  if (!sm.saveToFile(outFile))
  {
    std::printf("ERROR: could not save %s\n", outFile.c_str());
    return 3;
  }
  std::printf("Saved: %s\n", outFile.c_str());
  return 0;
}
