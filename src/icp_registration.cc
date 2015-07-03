#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <ros/ros.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Geometry>

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

  double sparse_filter_size;
  ros::param::param<double>("~sparse_filter_size", sparse_filter_size, 0);
  ROS_INFO("The leaf size of the voxel filter of the sparse cloud : %f",
           sparse_filter_size);

  double dense_filter_size;
  ros::param::param<double>("~dense_filter_size", dense_filter_size, 0);
  ROS_INFO("The leaf size of the voxel filter of the dense cloud : %f",
           dense_filter_size);
  double radius;
  ros::param::param<double>("~radius", radius, 3);
  ROS_INFO("Radius of the neighborhood search: %f", radius);

  pcl::PointCloud<PointType>::Ptr filtered_sparse_cloud(
      new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr filtered_dense_cloud(
      new pcl::PointCloud<PointType>());
  pcl::VoxelGrid<PointType> filter;

  if (sparse_filter_size > 0) {
    filter.setInputCloud(sparse_cloud);
    filter.setLeafSize(sparse_filter_size, sparse_filter_size,
                       sparse_filter_size);
    filter.filter(*filtered_sparse_cloud);
  } else {
    filtered_sparse_cloud = sparse_cloud;
  }

  if (dense_filter_size > 0) {
    filter.setInputCloud(dense_cloud);
    filter.setLeafSize(dense_filter_size, dense_filter_size, dense_filter_size);
    filter.filter(*filtered_dense_cloud);
  } else {
    filtered_dense_cloud = dense_cloud;
  }
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputCloud(filtered_sparse_cloud);
  icp.setInputTarget(filtered_dense_cloud);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.setMaxCorrespondenceDistance(radius);
  icp.align(Final);
  std::cout << "has converged:" << icp.hasConverged()
            << " score: " << icp.getFitnessScore() << std::endl;
  Eigen::Affine3f final_transform(icp.getFinalTransformation());
  auto estimated_translation = final_transform.translation();
  auto estimated_rotation =
      Eigen::Quaternion<float>(final_transform.rotation());
  ROS_INFO("Estimated trans: [%f\t %f\t %f]", estimated_translation[0],
           estimated_translation[1], estimated_translation[2]);
  ROS_INFO("Estimated rot: [%f\t %f\t %f\t %f]", estimated_rotation.w(),
           estimated_rotation.x(), estimated_rotation.y(),
           estimated_rotation.z());
  pcl::io::savePCDFile("icp_aligned_sparse_cloud.pcd", Final, true);

  return (0);
}
