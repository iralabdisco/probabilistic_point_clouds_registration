#include <limits>
#include <vector>

#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/point_types.h>
#include <tclap/CmdLine.h>

#include "point_cloud_registration/utilities.hpp"

typedef pcl::PointXYZ PointType;

int main(int argc, char **argv)
{
    double radius;
    float source_filter_size, target_filter_size;
    bool ground_truth = false, use_generalized;
    std::string source_file_name;
    std::string target_file_name;
    std::string ground_truth_file_name;

    try {
        TCLAP::CmdLine cmd("Probabilistic point cloud registration", ' ', "1.0");
        TCLAP::UnlabeledValueArg<std::string> source_file_name_arg("source_file_name",
                                                                   "The path of the source point cloud", true, "source_cloud.pcd", "string", cmd);
        TCLAP::UnlabeledValueArg<std::string> target_file_name_arg("target_file_name",
                                                                   "The path of the target point cloud", true, "target_cloud.pcd", "string", cmd);
        TCLAP::ValueArg<float> source_filter_arg("s", "source_filter_size",
                                                 "The leaf size of the voxel filter of the source cloud", false, 0, "float", cmd);
        TCLAP::ValueArg<float> target_filter_arg("t", "target_filter_size",
                                                 "The leaf size of the voxel filter of the target cloud", false, 0, "float", cmd);
        TCLAP::ValueArg<double> radius_arg("r", "radius", "The radius of the neighborhood search", false, 3,
                                           "double", cmd);
        TCLAP::ValueArg<std::string> ground_truth_arg("g", "ground_truth",
                                                      "The path of the ground truth for the source cloud, if available", false, "ground_truth.pcd",
                                                      "string", cmd);
        TCLAP::SwitchArg use_generalized_arg("u", "use_generalized", "Whether to use a GeneralizedICP", cmd,
                                             false);
        cmd.parse(argc, argv);

        radius = radius_arg.getValue();
        source_file_name = source_file_name_arg.getValue();
        target_file_name = target_file_name_arg.getValue();
        source_filter_size = source_filter_arg.getValue();
        target_filter_size = target_filter_arg.getValue();
        use_generalized = use_generalized_arg.getValue();

        if (ground_truth_arg.isSet()) {
            ground_truth = true;
            ground_truth_file_name = ground_truth_arg.getValue();
        }

    } catch (TCLAP::ArgException &e) {
        std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
        exit(EXIT_FAILURE);
    }

    std::cout << "Max radius of the nearest-point search: " << radius << std::endl;

    std::cout << "Loading source point cloud from " << source_file_name << std::endl;
    pcl::PointCloud<PointType>::Ptr source_cloud =
        boost::make_shared<pcl::PointCloud<PointType>>();
    if (pcl::io::loadPCDFile<PointType>(source_file_name, *source_cloud) == -1) {
        std::cout << "Could not load source cloud, closing" << std::endl;
        exit(EXIT_FAILURE);
    }

    std::cout << "Loading target point cloud from " << target_file_name << std::endl;
    pcl::PointCloud<PointType>::Ptr target_cloud =
        boost::make_shared<pcl::PointCloud<PointType>>();
    if (pcl::io::loadPCDFile<PointType>(target_file_name, *target_cloud) == -1) {
        std::cout << "Could not load target cloud, closing" << std::endl;
        exit(EXIT_FAILURE);
    }

    pcl::PointCloud<PointType>::Ptr source_ground_truth;
    if (ground_truth) {
        std::cout << "Loading ground truth point cloud from " << ground_truth_file_name << std::endl;
        source_ground_truth = boost::make_shared<pcl::PointCloud<PointType>>();
        if (pcl::io::loadPCDFile<PointType>(ground_truth_file_name, *source_ground_truth) == -1) {
            std::cout << "Could not load ground truth" << std::endl;
            ground_truth = false;
        }
    }

    pcl::VoxelGrid<PointType> filter;
    pcl::PointCloud<PointType>::Ptr filtered_source_cloud =
        boost::make_shared<pcl::PointCloud<PointType>>();
    if (source_filter_size > 0) {
        std::cout << "Filtering source point cloud with leaf of size " << source_filter_size << std::endl;
        filter.setInputCloud(source_cloud);
        filter.setLeafSize(source_filter_size, source_filter_size, source_filter_size);
        filter.filter(*filtered_source_cloud);
    } else {
        *filtered_source_cloud = *source_cloud;
    }
    if (target_filter_size > 0) {
        std::cout << "Filtering target point cloud with leaf of size " << target_filter_size << std::endl;
        filter.setInputCloud(target_cloud);
        filter.setLeafSize(target_filter_size, target_filter_size, target_filter_size);
        filter.filter(*target_cloud);
    }

    pcl::Registration<PointType, PointType> *icp;
    if (use_generalized) {
        std::cout << "Using GeneralizedICP" << std::endl;
        icp = new pcl::GeneralizedIterativeClosestPoint<PointType, PointType>;

    } else {
        std::cout << "Using standard ICP" << std::endl;
        icp = new pcl::IterativeClosestPoint<PointType, PointType>;
    }

    icp->setMaxCorrespondenceDistance(radius);
    icp->setMaximumIterations(100);
    icp->setInputSource(filtered_source_cloud);
    icp->setInputTarget(target_cloud);
    pcl::PointCloud<PointType>::Ptr aligned_source = boost::make_shared<pcl::PointCloud<PointType>>();
    icp->align(*aligned_source);

    aligned_source->clear();
    pcl::transformPointCloud (*source_cloud, *aligned_source, icp->getFinalTransformation());

    if (ground_truth) {
        double mean_error_before = point_cloud_registration::calculateMSE(source_ground_truth, source_cloud);
        double mean_error_after = point_cloud_registration::calculateMSE(source_ground_truth, aligned_source);
        std::cout << "Mean error before alignment: " << mean_error_before << std::endl;
        std::cout << "Mean error after alignment: " << mean_error_after << std::endl;
        std::cerr <<  target_filter_size << " , "<<mean_error_after << std::endl;
    }

    std::string aligned_source_name = "aligned_" + source_file_name;
    std::cout << "Saving aligned source cloud to: " << aligned_source_name.c_str() << std::endl;
    pcl::io::savePCDFile(aligned_source_name, *aligned_source);

    return 0;
}
