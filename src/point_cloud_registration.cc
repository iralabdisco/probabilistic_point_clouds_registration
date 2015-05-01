#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "ros/ros.h"
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ros/console.h>
#include <pcl/filters/filter.h>

#include <aslam/backend/ErrorTermEuclidean.hpp>
#include <aslam/backend/EuclideanExpression.hpp>
#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/MEstimatorPolicies.hpp>
#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/Optimizer.hpp>
#include <aslam/backend/ProbDataAssocPolicy.hpp>
#include <aslam/backend/RotationQuaternion.hpp>

using aslam::backend::ProbDataAssocPolicy;

typedef pcl::PointXYZ PointType;

int main(int argc, char** argv) {
  std::string node_name = "aslam_map_merging";
  ros::init(argc, argv, node_name, ros::init_options::AnonymousName);

  int dim_neighborhood;
  ros::param::param<int>("~dim_neighborhood", dim_neighborhood, 10);
  ROS_INFO("The dimension of neighborhood: %d", dim_neighborhood);

  bool use_gaussian;
  double dof;

  ros::param::param<bool>("~use_gaussian", use_gaussian, false);
  if (use_gaussian) {
    ROS_INFO("Using gaussian model");
  } else {
    ros::param::param<double>("~dof", dof, 5);
    ROS_INFO("Degree of freedom of t-distribution: %f", dof);
  }
  double radius;
  ros::param::param<double>("~radius", radius, 3);
  ROS_INFO("Radius of the neighborhood search: %f", radius);

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

  double dense_filter_size;
  ros::param::param<int>("~dense_filter_size", dense_filter_size, 0);
  ROS_INFO("Dimension of filter for dense map: %d", dense_filter_size);  

  double sparse_filter_size;
  ros::param::param<int>("~sparse_filter_size", sparse_filter_size, 0);
  ROS_INFO("Dimension of filter for sparse map: %d", sparse_filter_size);

  pcl::PointCloud<PointType>::Ptr filterd_dense_cloud;

  pcl::VoxelGrid<PointType> voxel_filter;
  sor.setInputCloud (dense_cloud);
  sor.setLeafSize (dense_filter_size, dense_filter_size, dense_filter_size);
  sor.filter (*cloud_filtered);


  pcl::KdTreeFLANN<PointType> kdtree;
  kdtree.setInputCloud(dense_cloud);
  std::vector<boost::shared_ptr<std::vector<int>>> correspondences(
      sparse_cloud->size());
  std::vector<float> distances(dim_neighborhood);
  for (std::size_t i = 0; i < sparse_cloud->size(); i++) {
    boost::shared_ptr<std::vector<int>> results =
        boost::make_shared<std::vector<int>>();
    kdtree.radiusSearch(*sparse_cloud, i, radius, *results, distances,
                        dim_neighborhood);
    ROS_DEBUG("Found %d correspondences", results->size());
    correspondences[i] = results;
  }

  boost::shared_ptr<aslam::backend::OptimizationProblem> problem(
      new aslam::backend::OptimizationProblem);

  // Random initialization
  std::random_device rd;
  std::mt19937 generator(rd());
  std::uniform_real_distribution<double> random_quat(0, 1);
  std::uniform_real_distribution<double> random_trans(-2, 2);

  // Creates the design variables: rotation and traslation
  Eigen::Vector3d traslation_initial_guess(random_trans(generator),
                                           random_trans(generator),
                                           random_trans(generator));
  boost::shared_ptr<aslam::backend::EuclideanPoint> traslation(
      new aslam::backend::EuclideanPoint(traslation_initial_guess));
  traslation->setActive(true);
  problem->addDesignVariable(traslation);

  Eigen::Vector4d rotation_initial_guess(
      random_quat(generator), random_quat(generator), random_quat(generator),
      random_quat(generator));
  rotation_initial_guess.normalize();
  boost::shared_ptr<aslam::backend::RotationQuaternion> rotation(
      new aslam::backend::RotationQuaternion(rotation_initial_guess));
  rotation->setActive(true);
  problem->addDesignVariable(rotation);

  ProbDataAssocPolicy::ErrorTermGroups error_groups(
      new std::vector<ProbDataAssocPolicy::ErrorTermGroup>);
  Eigen::Vector3d sparse_point, dense_point;
  for (std::size_t i = 0; i < sparse_cloud->size(); i++) {
    ProbDataAssocPolicy::ErrorTermGroup error_group(
        new std::vector<ProbDataAssocPolicy::ErrorTermPtr>);
    error_groups->push_back(error_group);
    sparse_point << sparse_cloud->at(i).x, sparse_cloud->at(i).y,
        sparse_cloud->at(i).z;
    for (int dense_index : *(correspondences[i])) {
      dense_point << dense_cloud->at(dense_index).x,
          dense_cloud->at(dense_index).y, dense_cloud->at(dense_index).z;
      boost::shared_ptr<aslam::backend::ErrorTermEuclidean> error_term(
          new aslam::backend::ErrorTermEuclidean(
              (rotation->toExpression() *
               aslam::backend::EuclideanExpression(sparse_point)) +
                  traslation->toExpression(),
              dense_point, 1));
      boost::shared_ptr<aslam::backend::FixedWeightMEstimator> weight(
          new aslam::backend::FixedWeightMEstimator(1));
      error_term->setMEstimatorPolicy(weight);
      error_group->push_back(error_term);
      problem->addErrorTerm(error_term);
    }
  }
  aslam::backend::OptimizerOptions options;
  options.verbose = true;
  // options.linearSolver = "cholmod";
  // options.levenbergMarquardtLambdaInit = 10;
  // options.doSchurComplement = false;
  // options.doLevenbergMarquardt = true;
  options.convergenceDeltaX = 1e-3;
  options.convergenceDeltaJ = 1e-3;
  options.maxIterations = std::numeric_limits<int>::max();
  aslam::backend::Optimizer optimizer(options);
  optimizer.setProblem(problem);
  boost::shared_ptr<ProbDataAssocPolicy> weight_updater;
  if (use_gaussian) {
    weight_updater.reset(new ProbDataAssocPolicy(
        error_groups, std::numeric_limits<double>::infinity(), 3));
  } else {
    weight_updater.reset(new ProbDataAssocPolicy(error_groups, dof, 3));
  }
  optimizer.setPerIterationCallback(weight_updater);
  optimizer.optimize();

  Eigen::Vector3d estimated_translation =
      traslation->toExpression().toEuclidean();
  Eigen::Quaternion<double> estimated_rot(rotation->getQuaternion());
  estimated_rot = estimated_rot.conjugate();
  Eigen::Affine3d estimated_transform = Eigen::Affine3d::Identity();
  estimated_transform.rotate(estimated_rot);
  estimated_transform.pretranslate(Eigen::Vector3d(estimated_translation));
  pcl::PointCloud<PointType>::Ptr aligned_sparse(
      new pcl::PointCloud<PointType>());
  pcl::transformPointCloud(*sparse_cloud, *aligned_sparse, estimated_transform);

  ROS_INFO("Estimated trans: [%f, %f, %f]", estimated_translation[0],
           estimated_translation[1], estimated_translation[2]);
  ROS_INFO("Estimated rot: [%f, %f, %f, %f]", estimated_rot.x(),
           estimated_rot.y(), estimated_rot.z(), estimated_rot.w());

  std::string aligned_file_name = "aligned_" + sparse_file_name;
  pcl::io::savePCDFile(aligned_file_name, *aligned_sparse, true);

  pcl::visualization::PCLVisualizer viewer;
  viewer.initCameraParameters();
  int v0(0);
  viewer.createViewPort(0, 0.0, 1.0, 1.0, v0);
  viewer.setBackgroundColor(0, 0, 0, v0);
  pcl::visualization::PointCloudColorHandlerCustom<PointType>
      aligned_sparse_red(aligned_sparse, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<PointType> dense_blue(
      dense_cloud, 0, 0, 255);
  viewer.addPointCloud<PointType>(aligned_sparse, aligned_sparse_red,
                                  "aligned_sparse", v0);
  viewer.addPointCloud<PointType>(dense_cloud, dense_blue, "dense", v0);

  while (!viewer.wasStopped()) {
    viewer.spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }
  return 0;
}
