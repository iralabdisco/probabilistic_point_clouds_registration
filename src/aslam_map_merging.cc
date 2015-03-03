#include <cmath>
#include <limits>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "ros/ros.h"
#include <ros/console.h>
#include <boost/shared_ptr.hpp>

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

double surfaceEquation(double x, double y) { return sin(x) + cos(y); }

int main(int argc, char** argv) {
  std::string node_name = "aslam_map_merging";
  ros::init(argc, argv, node_name);

  double noise_std;
  ros::param::param<double>("~noise_std", noise_std, 0.05);
  ROS_INFO("Using gaussian noise with standard deviation %f", noise_std);

  int num_landmarks;
  ros::param::param<int>("~num_landmarks", num_landmarks, 1000);
  ROS_INFO("Generating sparse map with %i landmarks", num_landmarks);

  int dim_neighborhood;
  ros::param::param<int>("~dim_neighborhood", dim_neighborhood, 5);
  ROS_INFO("Dimension of neighborhood: %d", dim_neighborhood);

  pcl::visualization::PCLVisualizer viewer;
  viewer.initCameraParameters();
  int v0(0);
  viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v0);
  viewer.setBackgroundColor(0, 0, 0, v0);

  std::string file_name;
  std::unique_ptr<dense_sparse_simulator::DenseSparseSimulator<PointType>>
      simulator;
  if (ros::param::get("~cloud_file_name", file_name) == false) {
    ROS_INFO(
        "No file name provided, using equation z = sin(x)+cos(y) to generate a "
        "point cloud");
    simulator.reset(new dense_sparse_simulator::DenseSparseSimulator<PointType>(
        num_landmarks, 5, 0.01, noise_std, dim_neighborhood, *surfaceEquation));
  } else {
    ROS_INFO("Using file %s", file_name.c_str());
    simulator.reset(new dense_sparse_simulator::DenseSparseSimulator<PointType>(
        num_landmarks, noise_std, dim_neighborhood, file_name));
  }

  pcl::PointCloud<PointType>::Ptr dense_map = simulator->denseMap();

  if (simulator->state() == 1) {
    pcl::VoxelGrid<PointType> filter;
    filter.setInputCloud(dense_map);
    filter.setLeafSize(1, 1, 1);
    // filter.filter(*dense_map);
  }

  pcl::PointCloud<PointType>::Ptr sparse_map(simulator->sparseMap());

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
  for (std::size_t i = 0; i < sparse_map->points.size(); i++) {
    ProbDataAssocPolicy::ErrorTermGroup error_group(
        new std::vector<ProbDataAssocPolicy::ErrorTermPtr>);
    error_groups->push_back(error_group);
    sparse_point << sparse_map->points[i].x, sparse_map->points[i].y,
        sparse_map->points[i].z;

    for (std::size_t j = 0; j < dim_neighborhood; j++) {
      std::shared_ptr<std::vector<int>> neighborhood =
          simulator->dataAssociation()[i];
      int dense_index = neighborhood->at(j);
      dense_point << dense_map->points[dense_index].x,
          dense_map->points[dense_index].y, dense_map->points[dense_index].z;
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
  // Force it to over-optimize
  options.convergenceDeltaX = 1e-19;
  options.convergenceDeltaJ = 1e-19;
  options.maxIterations = std::numeric_limits<int>::max();
  // Then create the optimizer and go!
  aslam::backend::Optimizer optimizer(options);
  optimizer.setProblem(problem);
  boost::shared_ptr<ProbDataAssocPolicy> weight_updater(
      new ProbDataAssocPolicy(error_groups, 1));
  optimizer.setPerIterationCallback(weight_updater);
  optimizer.optimize();

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> dense_blue(
      dense_map, 0, 0, 255);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sparse_red(
      sparse_map, 255, 0, 0);
  viewer.addPointCloud<PointType>(dense_map, dense_blue, "dense_map", v0);
  viewer.addPointCloud<PointType>(sparse_map, sparse_red, "sparse_map", v0);

  Eigen::Affine3d real_transform =
      simulator->denseToSparseTransform().inverse();

  Eigen::Vector3d estimated_translation =
      traslation->toExpression().toEuclidean();
  Eigen::Quaternion<double> estimated_rot(rotation->getQuaternion());
  estimated_rot = estimated_rot.conjugate();
  Eigen::Affine3d estimated_transform = Eigen::Affine3d::Identity();
  estimated_transform.rotate(estimated_rot);
  estimated_transform.pretranslate(Eigen::Vector3d(estimated_translation));
  pcl::PointCloud<PointType>::Ptr aligned_sparse(
      new pcl::PointCloud<PointType>());
  pcl::transformPointCloud(*sparse_map, *aligned_sparse, estimated_transform);

  int v1(0);
  viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v1);
  viewer.setBackgroundColor(0, 0, 0, v1);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
      aligned_sparse_red(aligned_sparse, 255, 0, 0);
  viewer.addPointCloud<PointType>(aligned_sparse, aligned_sparse_red,
                                  "aligned_sparse", v1);
  viewer.addPointCloud<PointType>(dense_map, dense_blue, "dense_2", v1);

  Eigen::Vector3d real_trans = real_transform.translation();
  Eigen::Quaternion<double> real_rot =
      Eigen::Quaternion<double>(real_transform.rotation());

  ROS_INFO("\nEstimated trans\t| Real trans\n%f\t| %f\n%f\t| %f\n%f\t| %f\n",
           estimated_translation[0], real_trans[0], estimated_translation[1],
           real_trans[1], estimated_translation[2], real_trans[2]);

  ROS_INFO(
      "\nEstimated rot\t| Real rot\n%f\t| %f\n%f\t| %f\n%f\t| %f\n%f\t| %f\n",
      estimated_rot.x(), real_rot.x(), estimated_rot.y(), real_rot.y(),
      estimated_rot.z(), real_rot.z(), estimated_rot.w(), real_rot.w());

  while (!viewer.wasStopped()) {
    viewer.spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }
  return 0;
}
