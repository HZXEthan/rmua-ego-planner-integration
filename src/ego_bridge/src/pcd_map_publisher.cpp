#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "pcd_map_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string pcd_path;
  std::string frame_id;
  double publish_rate_hz = 1.0;
  double voxel_leaf_size = 0.0;
  double clear_origin_radius = 0.0;
  double clear_origin_min_z = -1e9;
  double clear_origin_max_z = 1e9;
  double min_z = -1e9;
  double max_z = 1e9;

  pnh.param<std::string>("pcd_path", pcd_path, std::string());
  pnh.param<std::string>("frame_id", frame_id, std::string("world"));
  pnh.param("publish_rate_hz", publish_rate_hz, 1.0);
  pnh.param("voxel_leaf_size", voxel_leaf_size, 0.0);
  pnh.param("clear_origin_radius", clear_origin_radius, 0.0);
  pnh.param("clear_origin_min_z", clear_origin_min_z, -1e9);
  pnh.param("clear_origin_max_z", clear_origin_max_z, 1e9);
  pnh.param("min_z", min_z, -1e9);
  pnh.param("max_z", max_z, 1e9);

  if (pcd_path.empty()) {
    ROS_FATAL("Parameter '~pcd_path' is empty.");
    return 1;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *raw_cloud) != 0) {
    ROS_FATAL_STREAM("Failed to load PCD file: " << pcd_path);
    return 1;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  filtered_cloud->reserve(raw_cloud->size());
  for (const auto& pt : raw_cloud->points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
      continue;
    }
    if (pt.z < min_z || pt.z > max_z) {
      continue;
    }
    if (clear_origin_radius > 0.0 && pt.z >= clear_origin_min_z && pt.z <= clear_origin_max_z) {
      const double xy_norm = std::hypot(pt.x, pt.y);
      if (xy_norm < clear_origin_radius) {
        continue;
      }
    }
    filtered_cloud->push_back(pt);
  }
  filtered_cloud->width = filtered_cloud->size();
  filtered_cloud->height = 1;
  filtered_cloud->is_dense = false;

  pcl::PointCloud<pcl::PointXYZ>::Ptr publish_cloud(new pcl::PointCloud<pcl::PointXYZ>(*filtered_cloud));
  if (voxel_leaf_size > 0.0) {
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    voxel_filter.setInputCloud(filtered_cloud);
    voxel_filter.filter(*publish_cloud);
  }

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*publish_cloud, cloud_msg);
  cloud_msg.header.frame_id = frame_id;

  ros::Publisher ego_cloud_pub =
      nh.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 1, true);
  ros::Publisher global_cloud_pub =
      nh.advertise<sensor_msgs::PointCloud2>("/global_map", 1, true);

  ROS_INFO_STREAM("Loaded PCD map: " << pcd_path
                                     << ", raw points: " << raw_cloud->size()
                                     << ", filtered points: " << publish_cloud->size()
                                     << ", frame: " << frame_id);

  ros::Rate rate(std::max(0.1, publish_rate_hz));
  while (ros::ok()) {
    cloud_msg.header.stamp = ros::Time::now();
    ego_cloud_pub.publish(cloud_msg);
    global_cloud_pub.publish(cloud_msg);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
