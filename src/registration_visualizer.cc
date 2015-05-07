#include <Eigen/Core>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/tokenizer.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>


#include <iostream>
#include <string>
#include <vector>

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

std::string getWord(std::string line, int position) {
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
  double dense_filter_size;
  double sparse_filter_size;
  bool in_transformation_history = false;
  std::vector<Eigen::Affine3d> transformations;
  while (std::getline(log_file, line)) {
    if (!in_transformation_history) {
      ROS_INFO("%s", line.c_str());
      if (line.find("Transformation history:") != std::string::npos) {
        in_transformation_history = true;
      } else if (line.find("Sparse cloud:") != std::string::npos) {
        sparse_file_name = getWord(line, 2);
        sparse_filter_size = std::atof(getWord(line, 8).c_str());
      } else if (line.find("Dense cloud:") != std::string::npos) {
        dense_file_name = getWord(line, 2);
        dense_filter_size = std::atof(getWord(line, 8).c_str());
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

  std::ifstream data_assoc_file("data_assoc_" + log_file_name);
  boost::archive::text_iarchive data_assoc_arc(data_assoc_file);
  std::vector<std::vector<int>> data_association;
  data_assoc_arc >> data_association;


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
pcl::PointCloud<PointType>::Ptr filtered_sparse_cloud(
      new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr filtered_dense_cloud(
      new pcl::PointCloud<PointType>());
  pcl::VoxelGrid<PointType> filter;
  filter.setInputCloud(sparse_cloud);
  filter.setLeafSize(sparse_filter_size, sparse_filter_size,
                     sparse_filter_size);
  filter.filter(*filtered_sparse_cloud);

  filter.setInputCloud(dense_cloud);
  filter.setLeafSize(dense_filter_size, dense_filter_size, dense_filter_size);
  filter.filter(*filtered_dense_cloud);

  ros::Publisher sparse_publisher =
      n.advertise<PointCloud>("filtered_sparse_map", 1);
  ros::Publisher dense_publishier =
      n.advertise<PointCloud>("filtered_dense_map", 1);
  ros::Publisher vis_pub =
      n.advertise<visualization_msgs::Marker>("data_association", 0);
  ros::Rate loop_rate(1);
  filtered_dense_cloud->header.frame_id = "/map";
  filtered_sparse_cloud->header.frame_id = "/map";
  ROS_INFO("Waiting for a keypress before publishing initial clouds...");
  std::cin.ignore();
  dense_publishier.publish(filtered_dense_cloud);
  sparse_publisher.publish(filtered_sparse_cloud);

  visualization_msgs::Marker line_list;
  line_list.header.frame_id = "/map";
  line_list.header.stamp = ros::Time::now();
  line_list.ns = "data_association";
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.id = 0;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x = 0.01;
  line_list.color.r = 1.0;
  line_list.color.a = 0.7;
  for (std::size_t i = 0; i < data_association.size(); i++) {
    geometry_msgs::Point sparse_point;
    sparse_point.x = filtered_sparse_cloud->at(i).x;
    sparse_point.y = filtered_sparse_cloud->at(i).y;
    sparse_point.z = filtered_sparse_cloud->at(i).z;
    for (auto point_index : data_association[i]) {
      line_list.points.push_back(sparse_point);
      geometry_msgs::Point dense_point;
      dense_point.x = filtered_dense_cloud->at(point_index).x;
      dense_point.y = filtered_dense_cloud->at(point_index).y;
      dense_point.z = filtered_dense_cloud->at(point_index).z;
      line_list.points.push_back(dense_point);
    }
  }
  vis_pub.publish(line_list);
  ROS_INFO("Waiting for a keypress before starting registration...");
  std::cin.ignore();
  for (auto& t : transformations) {
    pcl::transformPointCloud(*filtered_sparse_cloud, *aligned_sparse, t);
    aligned_sparse->header.frame_id = "/map";
    sparse_publisher.publish(aligned_sparse);
    loop_rate.sleep();
  }
  return 0;
}
