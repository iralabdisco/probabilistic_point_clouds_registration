#include <limits>
#include <vector>

#include <Eigen/Sparse>
#include <pcl/common/transforms.h>
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

    int dim_neighborhood;
    ros::param::param<int>("~dim_neighborhood", dim_neighborhood, 10);
    ROS_INFO("Dimension of neighborhood: %d", dim_neighborhood);

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

    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud(target_cloud);
    std::vector<float> distances;
    Eigen::SparseMatrix<int, Eigen::RowMajor> data_association(source_cloud->size(), target_cloud->size());
    std::vector<Eigen::Triplet<int>> tripletList;
    for (std::size_t i = 0; i < source_cloud->size(); i++)
    {
        std::vector<int> neighbours;
        kdtree.radiusSearch(*source_cloud, i, radius, neighbours, distances, dim_neighborhood);
        for (int j : neighbours)
        {
            tripletList.push_back(Eigen::Triplet<int>(i, j, 1));
        }
    }
    data_association.setFromTriplets(tripletList.begin(), tripletList.end());
    data_association.makeCompressed();

    PointCloudRegistrationParams params;
    params.dof = std::numeric_limits<double>::infinity();
    params.max_neighbours = dim_neighborhood;
    params.dimension = 3;
    PointCloudRegistration registration(*source_cloud, *target_cloud, data_association, params);
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.use_nonmonotonic_steps = true;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = std::numeric_limits<int>::max();
    options.function_tolerance = 10e-5;
    options.num_threads = 8;
    ceres::Solver::Summary summary;
    registration.solve(options, &summary);

    ROS_INFO_STREAM(summary.FullReport());
    auto estimated_transform = registration.transformation();
    pcl::PointCloud<PointType>::Ptr aligned_source = boost::make_shared<pcl::PointCloud<PointType>>();
    pcl::transformPointCloud (*source_cloud, *aligned_source, estimated_transform);
    source_cloud->header.frame_id="map";
    source_pub.publish(source_cloud);
    target_cloud->header.frame_id="map";
    target_pub.publish(target_cloud);
    aligned_source->header.frame_id="map";
    aligned_source_pub.publish(aligned_source);
    return 0;
}
