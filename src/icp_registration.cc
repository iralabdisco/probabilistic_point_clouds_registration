#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "ros/ros.h"

#include "map_merging/dense_sparse_simulator.h"

typedef pcl::PointXYZ PointType;

double surfaceEquation(double x, double y) { return sin(x) + cos(y); }

int main(int argc, char** argv) {
  std::string node_name = "icp_map_merging";
  ros::init(argc, argv, node_name);

  int num_landmarks;
  ros::param::param<int>("~num_landmarks", num_landmarks, 1000);
  ROS_INFO("Using sparse map with %d landmarks", num_landmarks);

  double noise_std;
  ros::param::param<double>("~noise_std", noise_std, 0.05);
  ROS_INFO("Using gaussian noise with standard deviation %f", noise_std);

  std::string file_name;
  std::unique_ptr<dense_sparse_simulator::DenseSparseSimulator<PointType>>
      simulator;
  if (ros::param::get("~cloud_file_name", file_name) == false) {
    ROS_INFO(
        "No file name provided, using equation z = sin(x)+cos(y) to generate a "
        "point cloud");
    simulator.reset(new dense_sparse_simulator::DenseSparseSimulator<PointType>(
        num_landmarks, 5, noise_std, noise_std, *surfaceEquation));
  } else {
    ROS_INFO("Using file %s", file_name.c_str());
    simulator.reset(new dense_sparse_simulator::DenseSparseSimulator<PointType>(
        num_landmarks, noise_std, file_name));
  }

  pcl::PointCloud<PointType>::Ptr dense_map = simulator->denseMap();
  pcl::PointCloud<PointType>::Ptr dense_map_filtered(
      new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr sparse_map(simulator->sparseMap());

  ROS_INFO("Filtering point cloud");
  pcl::VoxelGrid<PointType> filter;
  filter.setInputCloud(dense_map);
  filter.setLeafSize(0.01f, 0.01f, 0.01f);
  filter.filter(*dense_map_filtered);

  pcl::visualization::PCLVisualizer viewer;
  viewer.initCameraParameters();
  int v0(0);
  viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v0);
  viewer.setBackgroundColor(0, 0, 0, v0);

  ROS_INFO("Estimating normals");
  // Normal estimation
  pcl::NormalEstimation<PointType, pcl::Normal> normal_estimator;
  normal_estimator.setInputCloud(dense_map_filtered);
  pcl::search::KdTree<PointType>::Ptr dense_tree(
      new pcl::search::KdTree<PointType>());
  normal_estimator.setSearchMethod(dense_tree);
  pcl::PointCloud<pcl::Normal>::Ptr dense_cloud_normals(
      new pcl::PointCloud<pcl::Normal>);
  normal_estimator.setRadiusSearch(0.4);
  normal_estimator.compute(*dense_cloud_normals);

  normal_estimator.setInputCloud(sparse_map);
  pcl::search::KdTree<PointType>::Ptr sparse_tree(
      new pcl::search::KdTree<PointType>());
  normal_estimator.setSearchMethod(sparse_tree);
  pcl::PointCloud<pcl::Normal>::Ptr sparse_cloud_normals(
      new pcl::PointCloud<pcl::Normal>);
  normal_estimator.compute(*sparse_cloud_normals);

  pcl::SampleConsensusInitialAlignment<PointType, PointType, pcl::Normal>
      sample_consensus;
  sample_consensus.setInputSource(dense_map_filtered);
  sample_consensus.setSourceFeatures(dense_cloud_normals);
  sample_consensus.setInputTarget(sparse_map);
  sample_consensus.setTargetFeatures(sparse_cloud_normals);
  pcl::PointCloud<PointType>::Ptr pre_aligned_dense_map(
      new pcl::PointCloud<PointType>);

  ROS_INFO("Computing initial guess");
  sample_consensus.align(*pre_aligned_dense_map);

  pcl::visualization::PointCloudColorHandlerCustom<PointType> pre_dense_blue(
      pre_aligned_dense_map, 0, 0, 255);
  pcl::visualization::PointCloudColorHandlerCustom<PointType> sparse_red(
      sparse_map, 255, 0, 0);
  viewer.addPointCloud<PointType>(pre_aligned_dense_map, pre_dense_blue,
                                  "pre_aligned_dense", v0);
  viewer.addPointCloud<PointType>(sparse_map, sparse_red, "sparse_map_0", v0);

  pcl::IterativeClosestPoint<PointType, PointType> icp;
  icp.setInputTarget(sparse_map);
  icp.setInputSource(pre_aligned_dense_map);
  pcl::PointCloud<PointType>::Ptr aligned_cloud(new pcl::PointCloud<PointType>);
  icp.setMaximumIterations(50);
  icp.setTransformationEpsilon(1e-8);
  icp.setRANSACOutlierRejectionThreshold(0.05);

  ROS_INFO("Performing ICP");
  icp.align(*aligned_cloud);
  ROS_INFO("has converged: %d\tscore: %f", icp.hasConverged(),
           icp.getFitnessScore());
  ROS_INFO("Convergence state: %d",
           icp.getConvergeCriteria()->getConvergenceState());

  int v1(0);
  viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v1);
  viewer.setBackgroundColor(0, 0, 0, v1);
  viewer.addPointCloud<PointType>(sparse_map, sparse_red, "sparse_map_1", v1);
  pcl::visualization::PointCloudColorHandlerCustom<PointType> dense_blue(
      aligned_cloud, 0, 0, 255);
  viewer.addPointCloud<PointType>(aligned_cloud, dense_blue, "aligned_dense",
                                  v1);
  viewer.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sparse_map_0");
  viewer.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sparse_map_1");
  while (!viewer.wasStopped()) {
    viewer.spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }
}
