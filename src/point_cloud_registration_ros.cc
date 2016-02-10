#include <limits>
#include <vector>

#include <Eigen/Sparse>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>


#include "point_cloud_registration/point_cloud_registration.h"

typedef pcl::PointXYZ PointType;

using point_cloud_registration::PointCloudRegistration;
using point_cloud_registration::PointCloudRegistrationParams;

int main(int argc, char** argv)
{
    std::string node_name = "point_cloud_registration_ros";
    ros::init(argc, argv, node_name, ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ros::Publisher aligned_source_pub = n.advertise<pcl::PointCloud<PointType>>("aligned_source", 1);
    ros::Publisher source_pub = n.advertise<pcl::PointCloud<PointType>>("source_cloud", 1);
    ros::Publisher target_pub = n.advertise<pcl::PointCloud<PointType>>("target_cloud", 1);
    ros::Publisher ground_truth_pub = n.advertise<pcl::PointCloud<PointType>>("ground_truth", 1);


    int max_neighbours;
    ros::param::param<int>("~max_neighbours", max_neighbours, 10);
    ROS_INFO("Max dimension of neighborhood: %d", max_neighbours);

    bool use_gaussian;
    double dof;
    ros::param::param<bool>("~use_gaussian", use_gaussian, false);
    if (use_gaussian)
    {
        ROS_INFO("Using gaussian model");
        dof = std::numeric_limits<double>::infinity();
    }
    else
    {
        ros::param::param<double>("~dof", dof, 5);
        ROS_INFO("Degree of freedom of t-distribution: %f", dof);
    }

    double radius;
    ros::param::param<double>("~radius", radius, 3);
    ROS_INFO("Radius of the neighborhood search: %f", radius);


    ROS_INFO("Loading source point cloud");
    std::string source_file_name;
    pcl::PointCloud<PointType>::Ptr source_cloud =
        boost::make_shared<pcl::PointCloud<PointType>>();
    if (ros::param::get("~source_cloud", source_file_name) == false ||
            pcl::io::loadPCDFile<PointType>(source_file_name, *source_cloud) == -1)
    {
        ROS_INFO("Could not load sparse cloud, closing...");
        exit(1);
    }
    else
    {
        ROS_INFO("Using file %s as sparse point cloud", source_file_name.c_str());
    }
    source_cloud->header.frame_id="map";
    source_pub.publish(source_cloud);

    ROS_INFO("Loading target point cloud");
    pcl::PointCloud<PointType>::Ptr target_cloud =
        boost::make_shared<pcl::PointCloud<PointType>>();
    std::string target_file_name;
    if (ros::param::get("~target_cloud", target_file_name) == false ||
            pcl::io::loadPCDFile<PointType>(target_file_name, *target_cloud) == -1)
    {
        ROS_INFO("Could not load target cloud, closing...");
        exit(1);
    }
    else
    {
        ROS_INFO("Using file %s as target point cloud", target_file_name.c_str());
    }
    target_cloud->header.frame_id="map";
    target_pub.publish(target_cloud);


    bool ground_truth = false;
    ROS_INFO("Loading target point cloud");
    pcl::PointCloud<PointType>::Ptr source_ground_truth =
        boost::make_shared<pcl::PointCloud<PointType>>();
    std::string ground_truth_file;
    if (ros::param::get("~ground_truth", ground_truth_file) == false ||
            pcl::io::loadPCDFile<PointType>(ground_truth_file, *source_ground_truth) == -1)
    {
        ROS_INFO("Could not load ground truth...");
    }
    else
    {
        ROS_INFO("Using file %s as ground truth point cloud", ground_truth_file.c_str());
        ground_truth = true;
    }
    source_ground_truth->header.frame_id="map";
    ground_truth_pub.publish(source_ground_truth);


    double source_filter_size, target_filter_size;
    ros::param::param<double>("~source_filter_size", source_filter_size, 0);
    ROS_INFO("The leaf size of the voxel filter of the source cloud is: %f",source_filter_size);
    ros::param::param<double>("~target_filter_size", target_filter_size, 0);
    ROS_INFO("The leaf size of the voxel filter of the dense cloud : %f", target_filter_size);
    pcl::VoxelGrid<PointType> filter;
    pcl::PointCloud<PointType>::Ptr filtered_source_cloud =
        boost::make_shared<pcl::PointCloud<PointType>>();
    if (source_filter_size > 0) {
        filter.setInputCloud(source_cloud);
        filter.setLeafSize(source_filter_size, source_filter_size, source_filter_size);
        filter.filter(*filtered_source_cloud);
      }
      if (target_filter_size > 0) {
        filter.setInputCloud(target_cloud);
        filter.setLeafSize(target_filter_size, target_filter_size, target_filter_size);
        filter.filter(*target_cloud);
      }
    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud(target_cloud);
    std::vector<float> distances;
    Eigen::SparseMatrix<int, Eigen::RowMajor> data_association(filtered_source_cloud->size(), target_cloud->size());
    std::vector<Eigen::Triplet<int>> tripletList;
    for (std::size_t i = 0; i < filtered_source_cloud->size(); i++)
    {
        std::vector<int> neighbours;
        kdtree.radiusSearch(*filtered_source_cloud, i, radius, neighbours, distances, max_neighbours);
        for (int j : neighbours)
        {
            tripletList.push_back(Eigen::Triplet<int>(i, j, 1));
        }
    }
    data_association.setFromTriplets(tripletList.begin(), tripletList.end());
    data_association.makeCompressed();

    PointCloudRegistrationParams params;
    params.dof = std::numeric_limits<double>::infinity();
    params.max_neighbours = max_neighbours;
    params.dimension = 3;
    PointCloudRegistration registration(*filtered_source_cloud, *target_cloud, data_association, params);
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.use_nonmonotonic_steps = true;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = std::numeric_limits<int>::max();
    options.function_tolerance = 10e-10;
    options.num_threads = 8;
    ceres::Solver::Summary summary;
    registration.solve(options, &summary);

    ROS_INFO_STREAM(summary.FullReport());
    auto estimated_transform = registration.transformation();
    pcl::PointCloud<PointType>::Ptr aligned_source = boost::make_shared<pcl::PointCloud<PointType>>();
    pcl::transformPointCloud (*source_cloud, *aligned_source, estimated_transform);
    source_cloud->header.frame_id="map";
    target_cloud->header.frame_id="map";
    aligned_source->header.frame_id="map";

    if(ground_truth){
        double mean_error = 0;
        for (std::size_t i = 0; i < source_ground_truth->size(); ++i)
        {
            double error = std::sqrt(std::pow(source_ground_truth->at(i).x - aligned_source->at(i).x, 2) +
                                     std::pow(source_ground_truth->at(i).y - aligned_source->at(i).y, 2) +
                                     std::pow(source_ground_truth->at(i).z - aligned_source->at(i).z, 2));
            mean_error += error;
        }
        mean_error /= target_cloud->size();
        ROS_INFO("Mean error: %f", mean_error);
    }
    ros::Rate rate(1);
    while(ros::ok()){
        source_pub.publish(source_cloud);
        target_pub.publish(target_cloud);
        aligned_source_pub.publish(aligned_source);
        if(ground_truth){
            ground_truth_pub.publish(source_ground_truth);
        }
        rate.sleep();
    }
    return 0;
}
