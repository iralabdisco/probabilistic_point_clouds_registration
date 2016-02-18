#include <vector>

#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

typedef pcl::PointXYZ PointType;

int main(int argc, char** argv)
{
    std::string node_name = "point_cloud_registration_ros";
    ros::init(argc, argv, node_name, ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ros::Publisher aligned_source_pub = n.advertise<pcl::PointCloud<PointType>>("aligned_source", 1);
    ros::Publisher source_pub = n.advertise<pcl::PointCloud<PointType>>("source_cloud", 1);
    ros::Publisher target_pub = n.advertise<pcl::PointCloud<PointType>>("target_cloud", 1);
    ros::Publisher ground_truth_pub = n.advertise<pcl::PointCloud<PointType>>("ground_truth", 1);

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
        ROS_INFO("Could not load source cloud, closing...");
        exit(1);
    }
    else
    {
        ROS_INFO("Using file %s as source point cloud", source_file_name.c_str());
    }
    source_cloud->header.frame_id = "map";
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
    target_cloud->header.frame_id = "map";
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
    source_ground_truth->header.frame_id = "map";
    ground_truth_pub.publish(source_ground_truth);


    double source_filter_size, target_filter_size;
    ros::param::param<double>("~source_filter_size", source_filter_size, 0);
    ROS_INFO("The leaf size of the voxel filter of the source cloud is: %f", source_filter_size);
    ros::param::param<double>("~target_filter_size", target_filter_size, 0);
    ROS_INFO("The leaf size of the voxel filter of the dense cloud : %f", target_filter_size);
    pcl::VoxelGrid<PointType> filter;
    pcl::PointCloud<PointType>::Ptr filtered_source_cloud =
        boost::make_shared<pcl::PointCloud<PointType>>();
    if (source_filter_size > 0)
    {
        filter.setInputCloud(source_cloud);
        filter.setLeafSize(source_filter_size, source_filter_size, source_filter_size);
        filter.filter(*filtered_source_cloud);
    }
    else
    {
        *filtered_source_cloud = *source_cloud;
    }
    if (target_filter_size > 0)
    {
        filter.setInputCloud(target_cloud);
        filter.setLeafSize(target_filter_size, target_filter_size, target_filter_size);
        filter.filter(*target_cloud);
    }

     pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
     ndt.setTransformationEpsilon (0.01);
     ndt.setStepSize (0.1);
     ndt.setResolution (1);
     ndt.setMaximumIterations (35);

     // Setting point cloud to be aligned.
    ndt.setInputSource(filtered_source_cloud);
    ndt.setInputTarget(target_cloud);
    pcl::PointCloud<PointType>::Ptr aligned_source = boost::make_shared<pcl::PointCloud<PointType>>();
    ndt.align(*aligned_source);
    ROS_INFO_STREAM("Normal Distributions Transform has converged:" << ndt.hasConverged () << " score: " << ndt.getFitnessScore ());
    source_cloud->header.frame_id = "map";
    target_cloud->header.frame_id = "map";
    aligned_source->header.frame_id = "map";

    if (ground_truth)
    {
        double mean_error_after = 0;
        double mean_error_before = 0;
        for (std::size_t i = 0; i < source_ground_truth->size(); ++i)
        {
            double error_after = std::sqrt(std::pow(source_ground_truth->at(i).x - aligned_source->at(i).x, 2) +
                                           std::pow(source_ground_truth->at(i).y - aligned_source->at(i).y, 2) +
                                           std::pow(source_ground_truth->at(i).z - aligned_source->at(i).z, 2));
            double error_before = std::sqrt(std::pow(source_ground_truth->at(i).x - source_cloud->at(i).x, 2) +
                                            std::pow(source_ground_truth->at(i).y - source_cloud->at(i).y, 2) +
                                            std::pow(source_ground_truth->at(i).z - source_cloud->at(i).z, 2));
            mean_error_after += error_after;
            mean_error_before += error_before;
        }
        mean_error_after /= source_ground_truth->size();
        mean_error_before /= source_ground_truth->size();
        ROS_INFO("Mean error before alignment: %f", mean_error_before);
        ROS_INFO("Mean error after alignment: %f,", mean_error_after);
    }

    std::string aligned_source_name = "aligned_" + source_file_name;
    ROS_INFO("Saving aligned source cloud to: %s", aligned_source_name.c_str());
    pcl::io::savePCDFile(aligned_source_name, *aligned_source);

    ros::Rate rate(1);
    while (ros::ok())
    {
        source_pub.publish(source_cloud);
        target_pub.publish(target_cloud);
        aligned_source_pub.publish(aligned_source);
        if (ground_truth)
        {
            ground_truth_pub.publish(source_ground_truth);
        }
        rate.sleep();
    }
    return 0;
}
