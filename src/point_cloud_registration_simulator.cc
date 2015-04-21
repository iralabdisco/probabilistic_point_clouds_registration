#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ros/console.h>
#include "ros/ros.h"
#include "ceres/ceres.h"
#include "ceres/loss_function.h"

#include <aslam/backend/ErrorTermEuclidean.hpp>
#include <aslam/backend/EuclideanExpression.hpp>
#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/MEstimatorPolicies.hpp>
#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/Optimizer.hpp>
#include <aslam/backend/ProbDataAssocPolicy.hpp>
#include <aslam/backend/RotationQuaternion.hpp>

#include "point_cloud_registration/dense_sparse_simulator.h"
#include "point_cloud_registration/reprojectionError.h"

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

  bool use_gaussian;
  double dof;

  ros::param::param<bool>("~use_gaussian", use_gaussian, false);
  if (use_gaussian) {
    ROS_INFO("Using gaussian model");
  } else {
    ros::param::param<double>("~dof", dof, 5);
    ROS_INFO("Degree of freedom of t-distribution: %f", dof);
  }

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

  pcl::PointCloud<PointType>::Ptr sparse_map(simulator->sparseMap());

  ceres::Problem problem;
  double rotation[] = {1, 0, 0, 0};
  double translation[] = {0, 0, 0};
  // ProbDataAssocPolicy::ErrorTermGroups error_groups(
  //     new std::vector<ProbDataAssocPolicy::ErrorTermGroup>);
  Eigen::Vector3d sparse_point, dense_point;
  std::vector<std::vector<ceres::LossFunctionWrapper*>> weights(
      sparse_map->points.size(), std::vector<ceres::LossFunctionWrapper*>());
  for (std::size_t i = 0; i < sparse_map->points.size(); i++) {
    // ProbDataAssocPolicy::ErrorTermGroup error_group(
        // new std::vector<ProbDataAssocPolicy::ErrorTermPtr>);
    // error_groups->push_back(error_group);
    sparse_point << sparse_map->points[i].x, sparse_map->points[i].y,
        sparse_map->points[i].z;

    for (std::size_t j = 0; j < dim_neighborhood; j++) {
      std::shared_ptr<std::vector<int>> neighborhood =
          simulator->dataAssociation()[i];
      int dense_index = neighborhood->at(j);
      dense_point << dense_map->points[dense_index].x,
          dense_map->points[dense_index].y, dense_map->points[dense_index].z;
      ceres::LossFunctionWrapper* weight = new ceres::LossFunctionWrapper(
          new ceres::ScaledLoss(NULL, 1, ceres::DO_NOT_TAKE_OWNERSHIP),
          ceres::TAKE_OWNERSHIP);
      weights[i][j] = weight;
      ceres::CostFunction* cost_function =
          ReprojectionError::Create(sparse_point, dense_point);
      problem.AddResidualBlock(cost_function, weight, rotation, translation);

      // boost::shared_ptr<aslam::backend::ErrorTermEuclidean> error_term(
      //     new aslam::backend::ErrorTermEuclidean(
      //         (rotation->toExpression() *
      //          aslam::backend::EuclideanExpression(sparse_point)) +
      //             traslation->toExpression(),
      //         dense_point, 1));
      // boost::shared_ptr<aslam::backend::FixedWeightMEstimator> weight(
      //     new aslam::backend::FixedWeightMEstimator(1));
      // error_term->setMEstimatorPolicy(weight);
      // error_group->push_back(error_term);
    }
  }
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";

/*  boost::shared_ptr<ProbDataAssocPolicy> weight_updater;
  if (use_gaussian) {
    weight_updater.reset(new ProbDataAssocPolicy(
        error_groups, std::numeric_limits<double>::infinity(), 3));
  } else {
    weight_updater.reset(new ProbDataAssocPolicy(error_groups, dof, 3));
  }*/

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> dense_blue(
      dense_map, 0, 0, 255);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sparse_red(
      sparse_map, 255, 0, 0);
  viewer.addPointCloud<PointType>(dense_map, dense_blue, "dense_map", v0);
  viewer.addPointCloud<PointType>(sparse_map, sparse_red, "sparse_map", v0);

  Eigen::Affine3d real_transform =
      simulator->denseToSparseTransform().inverse();

  Eigen::Vector3d estimated_translation(translation);
  Eigen::Quaternion<double> estimated_rot(rotation[0], rotation[1], rotation[2],
                                          rotation[3]);  // Needed becouse eigen
                                                         // stores quaternion as
                                                         // [x,y,z,w] instead
                                                         // than [w,x,y,z]
  // estimated_rot = estimated_rot.conjugate();
  estimated_rot.normalize();
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
