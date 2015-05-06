#include <Eigen/Core>
#include <boost/tokenizer.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>

#include <iostream>
#include <string>
#include <vector>

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

std::string getFileName(std::string line, int position) {
  boost::char_separator<char> sep(" ");
  boost::tokenizer<boost::char_separator<char>> tokenizer(line, sep);
  std::string file_name = "";
  int n_token = 0;
  for (auto& t : tokenizer) {
    if (n_token == position) {
      file_name = t;
    }
    n_token++;
  }
  return file_name;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "registration_visualizer", ros::init_options::AnonymousName);
  ros::NodeHandle n;

  ROS_INFO("Loading sparse point cloud");
  std::string log_file_name;
  if (ros::param::get("~log_file_name", log_file_name) == false) {
    ROS_INFO("Log file not specified, closing...");
    exit(1);
  }
  std::ifstream log_file(log_file_name);
  if (!log_file.is_open()) {
    ROS_INFO("Could not read log file from %s", log_file_name.c_str());
    exit(1);
  }
  std::string line;
  std::string value;
  std::string sparse_file_name;
  std::string dense_file_name;
  bool in_transformation_history = false;
  std::vector<Eigen::Affine3d> transformations;
  while (std::getline(log_file, line)) {
    if (!in_transformation_history) {
      ROS_INFO("%s", line.c_str());
      if (line.find("Transformation history:") != std::string::npos) {
        in_transformation_history = true;
      } else if (line.find("Sparse cloud:") != std::string::npos) {
        sparse_file_name = getFileName(line, 2);
      } else if (line.find("Dense cloud:") != std::string::npos) {
        dense_file_name = getFileName(line, 2);
      }
    } else {
      ROS_INFO("%s", line.c_str());
      std::string buffer;
      std::stringstream line_stream(line);
      std::vector<double> values;
      while (line_stream >> buffer) {
        values.push_back(std::atof(buffer.c_str()));
      }
      Eigen::Quaternion<double> rotation(values[3], values[4], values[5],
                                         values[6]);
      rotation.normalize();
      Eigen::Vector3d translation(values[0], values[1], values[2]);
      Eigen::Affine3d transformation = Eigen::Affine3d::Identity();
      transformation.rotate(rotation);
      transformation.pretranslate(translation);
      transformations.push_back(transformation);
    }
  }
  log_file.close();

  pcl::PointCloud<PointType>::Ptr sparse_cloud =
      boost::make_shared<pcl::PointCloud<PointType>>();
  pcl::PointCloud<PointType>::Ptr aligned_sparse =
      boost::make_shared<pcl::PointCloud<PointType>>();
  pcl::PointCloud<PointType>::Ptr dense_cloud =
      boost::make_shared<pcl::PointCloud<PointType>>();

  if (sparse_file_name == "" ||
      pcl::io::loadPCDFile<PointType>(sparse_file_name, *sparse_cloud)) {
    ROS_INFO("Could not load sparse map %s", sparse_file_name.c_str());
  }
  if (dense_file_name == "" ||
      pcl::io::loadPCDFile<PointType>(dense_file_name, *dense_cloud)) {
    ROS_INFO("Could not load sparse map %s", dense_file_name.c_str());
  }
  ros::Publisher sparse_publisher = n.advertise<PointCloud>("sparse_map", 1);
  ros::Publisher dense_publishier = n.advertise<PointCloud>("dense_map", 1);
  ros::Rate loop_rate(1);
  loop_rate.sleep();
  dense_cloud->header.frame_id = "/map";
  sparse_cloud->header.frame_id = "/map";
  ROS_INFO("Waiting for a keypress before starting...");
  std::cin.ignore();
  dense_publishier.publish(dense_cloud);
  sparse_publisher.publish(sparse_cloud);
  loop_rate.sleep();
  for (auto& t : transformations) {
    pcl::transformPointCloud(*sparse_cloud, *aligned_sparse, t);
    aligned_sparse->header.frame_id = "/map";
    sparse_publisher.publish(aligned_sparse);
    loop_rate.sleep();
  }
  return 0;
}
