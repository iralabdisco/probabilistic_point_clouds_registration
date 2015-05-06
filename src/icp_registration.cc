#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <ros/ros.h>
#include <pcl/filters/filter.h>


typedef pcl::PointXYZ PointType;

int main(int argc, char** argv) {
  ros::init(argc, argv, "icp_registration", ros::init_options::AnonymousName);
  ROS_INFO("Loading sparse point cloud");
  std::string sparse_file_name;
  pcl::PointCloud<PointType>::Ptr sparse_cloud =
      boost::make_shared<pcl::PointCloud<PointType>>();
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
  pcl::PointCloud<PointType>::Ptr dense_cloud =
      boost::make_shared<pcl::PointCloud<PointType>>();
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

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputCloud(sparse_cloud);
  icp.setInputTarget(dense_cloud);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.setMaxCorrespondenceDistance (40);
  icp.align(Final);
  std::cout << "has converged:" << icp.hasConverged()
            << " score: " << icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
  pcl::io::savePCDFile("icp_aligned_sparse_cloud.pcd", Final, true);

  return (0);
}