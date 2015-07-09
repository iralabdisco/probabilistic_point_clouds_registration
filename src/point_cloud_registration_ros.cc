#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include "point_cloud_registration/EigenMatrixSerialize.h"
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
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/Transform.h>

#include "point_cloud_registration/point_cloud_registration.h"

typedef pcl::PointXYZ PointType;
bool start = false;

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,
                           void* viewer_void) {
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer =
      *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer>*>(
           viewer_void);
  if (event.getKeySym() == "s" && event.keyDown()) {
    std::cout << "s was pressed => Starting video" << std::endl;
    start = true;
  }
}

int main(int argc, char** argv) {
  std::string node_name = "aslam_map_merging";
  ros::init(argc, argv, node_name, ros::init_options::AnonymousName);
  ros::NodeHandle n;
  ros::Publisher trans_pub =
      n.advertise<geometry_msgs::Transform>("transforms", 10000);

  int dim_neighborhood;
  ros::param::param<int>("~dim_neighborhood", dim_neighborhood, 10);
  ROS_INFO("The dimension of neighborhood: %d", dim_neighborhood);

  bool use_gaussian, previous;
  double dof;

  ros::param::param<bool>("~use_gaussian", use_gaussian, false);
  if (use_gaussian) {
    ROS_INFO("Using gaussian model");
    dof = std::numeric_limits<double>::infinity();
  } else {
    ros::param::param<double>("~dof", dof, 5);
    ROS_INFO("Degree of freedom of t-distribution: %f", dof);
  }
  double radius;
  ros::param::param<double>("~radius", radius, 3);
  ROS_INFO("Radius of the neighborhood search: %f", radius);

  double sparse_filter_size;
  ros::param::param<double>("~sparse_filter_size", sparse_filter_size, 3);
  ROS_INFO("The leaf size of the voxel filter of the sparse cloud : %f",
           sparse_filter_size);

  double dense_filter_size;
  ros::param::param<double>("~dense_filter_size", dense_filter_size, 3);
  ROS_INFO("The leaf size of the voxel filter of the dense cloud : %f",
           dense_filter_size);

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

  Eigen::Affine3d previous_transform;
  ros::param::param<bool>("~previous", previous, false);
  if (previous) {
    std::ifstream previous_matrix_file(sparse_file_name+"matrix.dat");
    if(previous_matrix_file.good()){
      ROS_INFO("Loading previous transform matrix");
      boost::archive::text_iarchive previous_matrix(previous_matrix_file);
      previous_matrix >> previous_transform.matrix();
      ROS_INFO_STREAM("Previous transform: " << previous_transform.matrix());
    }
    else{
      ROS_INFO("Error opening previous transform, closing...");
      exit(0);
    }
  }

  std::string log_file_name;
  ros::param::param<std::string>("~log_file_name", log_file_name, "");

  pcl::PointCloud<PointType>::Ptr filtered_sparse_cloud(
      new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr filtered_dense_cloud(
      new pcl::PointCloud<PointType>());
  pcl::VoxelGrid<PointType> filter;

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> plane_segmentation;
  // Optional
  plane_segmentation.setOptimizeCoefficients(true);
  // Mandatory
  plane_segmentation.setModelType(pcl::SACMODEL_PLANE);
  plane_segmentation.setMethodType(pcl::SAC_RANSAC);
  plane_segmentation.setDistanceThreshold(1);

  plane_segmentation.setInputCloud(dense_cloud);
  plane_segmentation.segment(*inliers, *coefficients);
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(dense_cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  // extract.filter (*dense_cloud);

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

  pcl::KdTreeFLANN<PointType> kdtree;
  kdtree.setInputCloud(filtered_dense_cloud);
  std::vector<std::vector<int>> correspondences(filtered_sparse_cloud->size());
  std::vector<float> distances;
  for (std::size_t i = 0; i < filtered_sparse_cloud->size(); i++) {
    std::vector<int> results;
    kdtree.radiusSearch(*filtered_sparse_cloud, i, radius, results, distances,
                        dim_neighborhood);
    ROS_DEBUG("Found %d correspondences", results.size());
    correspondences[i] = results;
  }

/*  std::random_device rd;
  std::mt19937 generator(rd());
  std::uniform_real_distribution<double> unif_distribution(-0.5, 0.5);*/
  const double initial_rotation[4] = {1, 0, 0, 0};
  const double initial_translation[3] = {0, 0, 0};
  point_cloud_registration::PointCloudRegistration registration(
      *filtered_sparse_cloud, *filtered_dense_cloud, correspondences, dof,
      initial_rotation, initial_translation);

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.use_nonmonotonic_steps = true;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = std::numeric_limits<int>::max();
  options.function_tolerance = 10e-5;
  options.num_threads = 8;
  // options.line_search_direction_type = ceres::STEEPEST_DESCENT;
  ceres::Solver::Summary summary;
  registration.solve(options, &summary);
  ROS_INFO_STREAM(summary.FullReport());

  auto estimated_transform = registration.transformation();
  auto estimated_translation = estimated_transform.translation();
  auto estimated_rotation =
      Eigen::Quaternion<double>(estimated_transform.rotation());
  pcl::PointCloud<PointType>::Ptr aligned_sparse(
      new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr aligned_filtered_sparse(
      new pcl::PointCloud<PointType>());
  pcl::transformPointCloud(*sparse_cloud, *aligned_sparse, estimated_transform);
  pcl::transformPointCloud(*filtered_sparse_cloud, *aligned_filtered_sparse,
                           estimated_transform);
  if (previous) {
    estimated_transform = estimated_transform * previous_transform;
  }

  ROS_INFO("Estimated trans: [%f\t %f\t %f]", estimated_translation[0],
           estimated_translation[1], estimated_translation[2]);
  ROS_INFO("Estimated rot: [%f\t %f\t %f\t %f]", estimated_rotation.w(),
           estimated_rotation.x(), estimated_rotation.y(),
           estimated_rotation.z());
  ROS_INFO_STREAM(estimated_transform.matrix());

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

  auto transformation_history = registration.transformation_history();

  std::ofstream log_file;
  if (log_file_name != "") {
    log_file.open(log_file_name);
    if (log_file.is_open()) {
      log_file << "Sparse cloud: " << sparse_file_name
               << " with voxel filter of size: " << sparse_filter_size
               << std::endl;
      log_file << "Dense cloud: " << dense_file_name
               << " with voxel filter of size: " << dense_filter_size
               << std::endl;
      if (use_gaussian) {
        log_file << "Using a gaussian" << std::endl;
      } else {
        log_file << "Using t-distribution with " << dof << " dof" << std::endl;
      }
      log_file << "Transformation history: " << std::endl;
      for (auto& transformation : transformation_history) {
        auto estimated_translation = transformation.translation();
        auto estimated_rotation =
            Eigen::Quaternion<double>(transformation.rotation());
        log_file << estimated_translation[0] << "\t" << estimated_translation[1]
                 << "\t" << estimated_translation[2] << "\t";
        log_file << estimated_rotation.w() << "\t" << estimated_rotation.x()
                 << "\t" << estimated_rotation.y() << "\t"
                 << estimated_rotation.z() << "\n";
      }
      log_file.close();
    }
  }
  {
    std::ofstream data_assoc_file("data_assoc_" + log_file_name);
    boost::archive::text_oarchive data_assoc_archive(data_assoc_file);
    data_assoc_archive << correspondences;
  }
  {
    std::ofstream matrix_file(aligned_file_name + "matrix.dat");
    boost::archive::text_oarchive matrix_archive(matrix_file);
    matrix_archive << estimated_transform.matrix();
  }
  std::ofstream residuals_file;
  residuals_file.open("residuals.txt");
  std::vector<std::vector<Eigen::Vector3d>> residuals(correspondences.size());
  for (size_t i = 0; i < correspondences.size(); i++) {
    for (size_t j = 0; j < correspondences[i].size(); j++) {
      Eigen::Vector3d res;
      res[0] = (*aligned_filtered_sparse)[i].x -
               (*filtered_dense_cloud)[correspondences[i][j]].x;
      res[1] = (*aligned_filtered_sparse)[i].y -
               (*filtered_dense_cloud)[correspondences[i][j]].y;
      res[2] = (*aligned_filtered_sparse)[i].z -
               (*filtered_dense_cloud)[correspondences[i][j]].z;
      residuals_file << res[0] << " , " << res[1] << " , " << res[2] << " ; ";
    }
    residuals_file << std::endl;
  }
  residuals_file.close();
  std::ofstream sparse_file;
  sparse_file.open("sparse_points.txt");
  for (auto point : *aligned_filtered_sparse) {
      sparse_file << point.x << ",";
      sparse_file << point.y << ",";
      sparse_file << point.z;
    sparse_file << std::endl;
  }
  sparse_file.close();

  std::ofstream dense_file;
  dense_file.open("dense_points.txt");
  for (auto point : *filtered_dense_cloud) {
      dense_file << point.x << ",";
      dense_file << point.y << ",";
      dense_file << point.z;
    dense_file << std::endl;
  }
  dense_file.close();

  while (!viewer.wasStopped()) {
    viewer.spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }
  return 0;
}
