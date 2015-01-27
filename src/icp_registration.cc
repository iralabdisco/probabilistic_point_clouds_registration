#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "map_merging/dense_sparse_simulator.h"

typedef pcl::PointXYZ PointType;

int main() {
  int num_landmarks = 1000;
  dense_sparse_simulator::DenseSparseSimulator<PointType> simulator(
      num_landmarks, 0.05, "/home/simone/Downloads/kitchen/Rf10.pcd");
  pcl::PointCloud<PointType>::Ptr dense_map = simulator.denseMap();
  pcl::PointCloud<PointType>::Ptr dense_map_filtered(
      new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr sparse_map(simulator.sparseMap());

  std::printf("Filtering point cloud\n");
  pcl::VoxelGrid<PointType> filter;
  filter.setInputCloud(dense_map);
  filter.setLeafSize(0.05f, 0.05f, 0.05f);
  filter.filter(*dense_map_filtered);

  pcl::visualization::PCLVisualizer viewer;
  viewer.initCameraParameters();
  int v0(0);
  viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v0);
  viewer.setBackgroundColor(0, 0, 0, v0);

  std::printf("Estimating normals\n");
  // Normal estimation
  pcl::NormalEstimation<PointType, pcl::Normal> normal_estimator;
  normal_estimator.setInputCloud(dense_map_filtered);
  pcl::search::KdTree<PointType>::Ptr dense_tree(
      new pcl::search::KdTree<PointType>());
  normal_estimator.setSearchMethod(dense_tree);
  pcl::PointCloud<pcl::Normal>::Ptr dense_cloud_normals(
      new pcl::PointCloud<pcl::Normal>);
  normal_estimator.setRadiusSearch(0.2);
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

  std::printf("Computing initial guess\n");
  sample_consensus.align(*pre_aligned_dense_map);

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
      pre_dense_blue(pre_aligned_dense_map, 0, 0, 255);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sparse_red(
      sparse_map, 255, 0, 0);
  viewer.addPointCloud<PointType>(pre_aligned_dense_map, pre_dense_blue,
                                  "pre_aligned_dense", v0);
  viewer.addPointCloud<PointType>(sparse_map, sparse_red, "sparse_map_0", v0);

  pcl::IterativeClosestPoint<PointType, PointType> icp;
  /**std::printf("Default converge criteria:"<<std::endl;
  std::printf("Max iterations:
  "<<icp.getConvergeCriteria()->getMaximumIterations()<<std::endl;
  std::printf("TransformationEpsilon:
  "<<icp.getConvergeCriteria()->getMaximumIterations()<<std::endl;
  std::printf("Max iterations:
  "<<icp.getConvergeCriteria()->getMaximumIterations()<<std::endl;**/
  icp.setInputTarget(sparse_map);
  icp.setInputSource(pre_aligned_dense_map);
  pcl::PointCloud<PointType>::Ptr aligned_cloud(new pcl::PointCloud<PointType>);
  icp.setMaximumIterations(50);
  icp.setTransformationEpsilon(1e-8);
  icp.setRANSACOutlierRejectionThreshold(0.05);

  std::printf("Performing ICP\n");
  icp.align(*aligned_cloud);
  std::printf("has converged: %d\tscore: %f\n", icp.hasConverged(),
              icp.getFitnessScore());
  std::printf("Convergence state: %d\n",
              icp.getConvergeCriteria()->getConvergenceState());
  // std::printf("%s\n",icp.getFinalTransformation());
  // std::printf("Exact
  // transformation:"<<std::endl<<simulator.denseToSparseTransform().matrix()<<std::endl;
  // icp.setTransformationEpsilon (1e-8);
  // icp.setEuclideanFitnessEpsilon (1);

  int v1(0);
  viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v1);
  viewer.setBackgroundColor(0, 0, 0, v1);
  viewer.addPointCloud<PointType>(sparse_map, sparse_red, "sparse_map_1", v1);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> dense_blue(
      aligned_cloud, 0, 0, 255);
  viewer.addPointCloud<PointType>(aligned_cloud, dense_blue, "aligned_dense",
                                  v1);

  while (!viewer.wasStopped()) {
    viewer.spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }
}
