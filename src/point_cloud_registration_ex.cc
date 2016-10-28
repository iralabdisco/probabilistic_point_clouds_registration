#include <limits>
#include <vector>
#include <stdlib.h>

#include <Eigen/Sparse>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <tclap/CmdLine.h>

#include "point_cloud_registration/point_cloud_registration.h"

typedef pcl::PointXYZ PointType;

using point_cloud_registration::PointCloudRegistration;
using point_cloud_registration::PointCloudRegistrationParams;

int main(int argc, char **argv)
{
    float source_filter_size, target_filter_size;
    bool use_gaussian = false, ground_truth = false;
    std::string source_file_name;
    std::string target_file_name;
    std::string ground_truth_file_name;
    PointCloudRegistrationParams params;
    params.dimension = 3;

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
        TCLAP::ValueArg<int> max_neighbours_arg("n", "max_neighbours",
                                                "The max cardinality of the neighbours' set", false, 10, "int", cmd);
        TCLAP::ValueArg<int> num_iter_arg("i", "num_iter",
                                          "The maximum number of iterations to perform", false, 10, "int", cmd);
        TCLAP::ValueArg<float> dof_arg("d", "dof", "The Degree of freedom of t-distribution", false, 5,
                                       "float", cmd);
        TCLAP::ValueArg<float> radius_arg("r", "radius", "The radius of the neighborhood search", false, 3,
                                          "float", cmd);
        TCLAP::ValueArg<float> dist_tresh_arg("x", "dist_treshold",
                                              "If the mse drops below dist_treshold, the algorithm terminate", false, 0.01, "float", cmd);
        TCLAP::SwitchArg use_gaussian_arg("u", "use_gaussian",
                                          "Whether to use a gaussian instead the a t-distribution", cmd, false);
        TCLAP::ValueArg<std::string> ground_truth_arg("g", "ground_truth",
                                                      "The path of the ground truth for the source cloud, if available", false, "ground_truth.pcd",
                                                      "string", cmd);
        cmd.parse(argc, argv);

        params.max_neighbours = max_neighbours_arg.getValue();
        use_gaussian = use_gaussian_arg.getValue();
        params.dof = dof_arg.getValue();
        params.radius = radius_arg.getValue();
        params.n_iter = num_iter_arg.getValue();
        params.verbose = true;
        params.dist_treshold = dist_tresh_arg.getValue();
        source_file_name = source_file_name_arg.getValue();
        target_file_name = target_file_name_arg.getValue();
        source_filter_size = source_filter_arg.getValue();
        target_filter_size = target_filter_arg.getValue();

        if (ground_truth_arg.isSet()) {
            ground_truth = true;
            ground_truth_file_name = ground_truth_arg.getValue();
        }

    } catch (TCLAP::ArgException &e) {
        std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
        exit(EXIT_FAILURE);
    }


    if (use_gaussian) {
        std::cout << "Using gaussian model" << std::endl;
        params.dof = std::numeric_limits<double>::infinity();
    } else {
        std::cout << "Using a t-distribution with " << params.dof << " dof" << std::endl;
    }
    std::cout << "Radius of the neighborhood search: " << params.radius << std::endl;
    std::cout << "Max number of neighbours: " << params.max_neighbours << std::endl;
    std::cout << "Max number of iterations: " << params.n_iter << std::endl;
    std::cout << "MSE treshold: " << params.dist_treshold << std::endl;

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
    params.debug = true;
    PointCloudRegistration registration(filtered_source_cloud, target_cloud, params);
    registration.align();
    auto estimated_transform = registration.transformation();
    pcl::PointCloud<PointType>::Ptr aligned_source = boost::make_shared<pcl::PointCloud<PointType>>();
    pcl::transformPointCloud (*source_cloud, *aligned_source, estimated_transform);

    if (ground_truth) {
        double mean_error_after = 0;
        double mean_error_before = 0;
        for (std::size_t i = 0; i < source_ground_truth->size(); ++i) {
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
        std::cout << "Mean error before alignment: " << mean_error_before << std::endl;
        std::cout << "Mean error after alignment: " << mean_error_after << std::endl;
    }
    std::cout << "Transformation history:" << std::endl;
    for (auto trans : registration.transformation_history()) {
        Eigen::Quaterniond rotq(trans.rotation());
        std::cout << "T: " << trans.translation().x() << ", " << trans.translation().y() << ", " <<
                  trans.translation().z() << " ||| R: " << rotq.x() << ", " << rotq.y() << ", " << rotq.z() << ", " <<
                  rotq.w() << std::endl;
    }
    std::string aligned_source_name = "aligned_" + source_file_name;
    std::cout << "Saving aligned source cloud to: " << aligned_source_name.c_str() << std::endl;
    pcl::io::savePCDFile(aligned_source_name, *aligned_source);

    return 0;
}
