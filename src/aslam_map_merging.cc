#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "ros/ros.h"
#include <ros/console.h>
#include <boost/shared_ptr.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <boost/make_shared.hpp>

#include <aslam/backend/ErrorTermEuclidean.hpp>
#include <aslam/backend/EuclideanExpression.hpp>
#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/MEstimatorPolicies.hpp>
#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/Optimizer.hpp>
#include <aslam/backend/ProbDataAssocPolicy.hpp>
#include <aslam/backend/RotationQuaternion.hpp>

#include "map_merging/dense_sparse_simulator.h"

using aslam::backend::ProbDataAssocPolicy;

typedef pcl::PointXYZ PointType;

int main(int argc, char** argv) {
  std::string node_name = "aslam_map_merging";
  ros::init(argc, argv, node_name);

  int dim_neighborhood;
  ros::param::param<int>("~dim_neighborhood", dim_neighborhood, 10);
  ROS_INFO("The dimension of neighborhood: %d", dim_neighborhood);

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
  }

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<PointType> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.5);
  seg.setInputCloud(sparse_cloud);
  // seg.segment(*inliers, *coefficients);

  pcl::ExtractIndices<PointType> extract;
  extract.setInputCloud(sparse_cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  pcl::PointCloud<PointType>::Ptr segmented_sparse =
      boost::make_shared<pcl::PointCloud<PointType>>();
  extract.filter(*sparse_cloud);

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
    ROS_INFO("Found %d correspondences", results->size());
    correspondences[i] = results;
  }

  boost::shared_ptr<aslam::backend::OptimizationProblem> problem(
      new aslam::backend::OptimizationProblem);

  // Creates the design variables: rotation and traslation
  Eigen::Vector3d traslation_initial_guess;
  boost::shared_ptr<aslam::backend::EuclideanPoint> traslation(
      new aslam::backend::EuclideanPoint(traslation_initial_guess));
  traslation->setActive(true);
  problem->addDesignVariable(traslation);

  Eigen::Vector4d rotation_initial_guess(0, 0, 0, 1);
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
  options.convergenceDeltaX = 1e-5;
  options.convergenceDeltaJ = 1e-5;
  options.maxIterations = std::numeric_limits<int>::max();
  // Then create the optimizer and go!
  aslam::backend::Optimizer optimizer(options);
  optimizer.setProblem(problem);
  boost::shared_ptr<ProbDataAssocPolicy> weight_updater(
      new ProbDataAssocPolicy(error_groups, 1));
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

  /*  ROS_INFO("\nEstimated trans\t| Real trans\n%f\t| %f\n%f\t| %f\n%f\t|
     %f\n",
             real_trans[0], estimated_translation[1],
             real_trans[1], estimated_translation[2], real_trans[2]);*/

  /*  ROS_INFO(
        "\nEstimated rot\t| Real rot\n%f\t| %f\n%f\t| %f\n%f\t| %f\n%f\t| %f\n",
        estimated_rot.x(), real_rot.x(), estimated_rot.y(), real_rot.y(),
        estimated_rot.z(), real_rot.z(), estimated_rot.w(), real_rot.w());*/

  while (!viewer.wasStopped()) {
    viewer.spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }
  return 0;
}
