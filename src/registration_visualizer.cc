#include <ros/ros.h>
#include <geometry_msgs/Transform.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <string>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <Eigen/Core>
#include <eigen_conversions/eigen_msg.h>
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

pcl::PointCloud<PointType>::Ptr sparse_cloud = boost::make_shared<pcl::PointCloud<PointType>>();
pcl::PointCloud<PointType>::Ptr dense_cloud =
    boost::make_shared<pcl::PointCloud<PointType>>();
pcl::PointCloud<PointType>::Ptr aligned_sparse =
    boost::make_shared<pcl::PointCloud<PointType>>();
ros::Publisher pub;

void transformCallback(const geometry_msgs::Transform::ConstPtr& tranform) {
Eigen::Affine3d eigen_transform;
  tf::transformMsgToEigen(*tranform, eigen_transform);
  pcl::transformPointCloud(*sparse_cloud, *aligned_sparse, eigen_transform);
  aligned_sparse->header.frame_id = "/world";
  pub.publish(aligned_sparse);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "registration_visualizer");
  ros::NodeHandle n;

  ROS_INFO("Loading sparse point cloud");
  std::string sparse_file_name;
  sparse_cloud = boost::make_shared<pcl::PointCloud<PointType>>();
  if (ros::param::get("~sparse_file_name", sparse_file_name) == false ||
      pcl::io::loadPCDFile<PointType>(sparse_file_name, *sparse_cloud) == -1) {
    ROS_INFO("Could not load sparse cloud, closing...");
    exit(1);
  } else {
    ROS_INFO("Using file %s as sparse point cloud", sparse_file_name.c_str());
    std::vector<int> tmp_indices;
    pcl::removeNaNFromPointCloud(*sparse_cloud, *sparse_cloud, tmp_indices);
    ROS_INFO("Removed %d NaN points from sparse cloud", tmp_indices.size());
  }

  ROS_INFO("Loading dense point cloud");
  dense_cloud = boost::make_shared<pcl::PointCloud<PointType>>();
  std::string dense_file_name;
  if (ros::param::get("~dense_file_name", dense_file_name) == false ||
      pcl::io::loadPCDFile<PointType>(dense_file_name, *dense_cloud) == -1) {
    ROS_INFO("Could not load dense cloud, closing...");
    exit(1);
  } else {
    ROS_INFO("Using file %s as dense point cloud", dense_file_name.c_str());
    std::vector<int> tmp_indices;
    pcl::removeNaNFromPointCloud(*dense_cloud, *dense_cloud, tmp_indices);
    ROS_INFO("Removed %d NaN points from dense cloud", tmp_indices.size());
  }
  pub = n.advertise<PointCloud>("point_clouds", 1);
  ros::Publisher dense_pub = n.advertise<PointCloud>("dense_map", 1);
  ros::Rate loop_rate(1);
  loop_rate.sleep();
  dense_cloud->header.frame_id = "/world";
  sparse_cloud->header.frame_id = "/world";
  dense_pub.publish(dense_cloud);
  pub.publish(sparse_cloud);
  ROS_INFO("Published Dense");
  ros::Subscriber sub = n.subscribe("transforms", 10000, transformCallback);
  ros::spin();

  return 0;
}
