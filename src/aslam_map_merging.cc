#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <aslam/backend/ErrorTermEuclidean.hpp>
#include <aslam/backend/EuclideanExpression.hpp>
#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/MEstimatorPolicies.hpp>
#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/Optimizer.hpp>
#include <aslam/backend/RotationQuaternion.hpp>

#include "map_merging/dense_sparse_simulator.h"

typedef pcl::PointXYZ PointType;

int main(int argc, char** argv) {
  pcl::visualization::PCLVisualizer viewer;
  viewer.initCameraParameters();
  int v0(0);
  viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v0);
  viewer.setBackgroundColor(0, 0, 0, v0);

  int num_landmarks = 10000;
  dense_sparse_simulator::DenseSparseSimulator<PointType> simulator(
      num_landmarks, 0.05, "/home/simone/Scaricati/kitchen/Rf12.pcd");
  pcl::PointCloud<PointType>::Ptr dense_map = simulator.denseMap();
  pcl::PointCloud<PointType>::Ptr sparse_map(simulator.sparseMap());

  pcl::VoxelGrid<PointType> filter;
  filter.setInputCloud(dense_map);
  filter.setLeafSize(0.05f, 0.05f, 0.05f);
  filter.filter(*dense_map);

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

  Eigen::Vector3d sparse_point, dense_point;
  for (uint i = 0; i < sparse_map->points.size(); i++) {
    sparse_point << sparse_map->points[i].x, sparse_map->points[i].y,
        sparse_map->points[i].z;

    int dense_index = simulator.dataAssociation()[i];
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
    problem->addErrorTerm(error_term);
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
  // Then create the optimizer and go!
  aslam::backend::Optimizer optimizer(options);
  optimizer.setProblem(problem);
  optimizer.optimize();

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> dense_blue(
      dense_map, 0, 0, 255);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sparse_red(
      sparse_map, 255, 0, 0);
  viewer.addPointCloud<PointType>(dense_map, dense_blue, "dense_map", v0);
  viewer.addPointCloud<PointType>(sparse_map, sparse_red, "sparse_map", v0);

  Eigen::Affine3d real_transform = simulator.denseToSparseTransform().inverse();

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

  std::printf("\nEstimated trans\t| Real trans\n");
  std::printf("%f\t| %f\n", estimated_translation[0], real_trans[0]);
  std::printf("%f\t| %f\n", estimated_translation[1], real_trans[1]);
  std::printf("%f\t| %f\n", estimated_translation[2], real_trans[2]);

  std::printf("\nEstimated rot\t| Real rot\n");
  std::printf("%f\t| %f\n", estimated_rot.x(), real_rot.x());
  std::printf("%f\t| %f\n", estimated_rot.y(), real_rot.y());
  std::printf("%f\t| %f\n", estimated_rot.z(), real_rot.z());
  std::printf("%f\t| %f\n", estimated_rot.w(), real_rot.w());

  while (!viewer.wasStopped()) {
    viewer.spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }
  return 0;
}
