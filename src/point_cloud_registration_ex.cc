#include <limits>
#include <vector>
#include <stdlib.h>

#include <Eigen/Sparse>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <tclap/CmdLine.h>

#include "point_cloud_registration/point_cloud_registration_iteration.h"

typedef pcl::PointXYZ PointType;

using point_cloud_registration::PointCloudRegistrationIteration;
using point_cloud_registration::PointCloudRegistrationParams;

int main(int argc, char **argv)
{
    int max_neighbours;
    float dof, radius, source_filter_size, target_filter_size;
    bool use_gaussian = false, ground_truth = false;
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
        TCLAP::ValueArg<int> max_neighbours_arg("n", "max_neighbours",
                                                "The max cardinality of the neighbours' set", false, 10, "int", cmd);
        TCLAP::ValueArg<float> dof_arg("d", "dof", "The Degree of freedom of t-distribution", false, 5,
                                       "float", cmd);
        TCLAP::ValueArg<float> radius_arg("r", "radius", "The radius of the neighborhood search", false, 3,
                                          "float", cmd);
        TCLAP::SwitchArg use_gaussian_arg("u", "use_gaussian",
                                          "Whether to use a gaussian instead the a t-distribution", cmd, false);
        TCLAP::ValueArg<std::string> ground_truth_arg("g", "ground_truth",
                                                      "The path of the ground truth for the source cloud, if available", false, "ground_truth.pcd",
                                                      "string", cmd);
        cmd.parse(argc, argv);

        max_neighbours = max_neighbours_arg.getValue();
        use_gaussian = use_gaussian_arg.getValue();
        dof = dof_arg.getValue();
        radius = radius_arg.getValue();
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
        dof = std::numeric_limits<double>::infinity();
    } else {
        std::cout << "Using a t-distribution with " << dof << " dof" << std::endl;
    }
    std::cout << "Radius of the neighborhood search: " << radius << std::endl;
    std::cout << "Max number of neighbours: " << max_neighbours << std::endl;


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
    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud(target_cloud);
    std::vector<float> distances;
    Eigen::SparseMatrix<int, Eigen::RowMajor> data_association(filtered_source_cloud->size(),
                                                               target_cloud->size());
    std::vector<Eigen::Triplet<int>> tripletList;
    for (std::size_t i = 0; i < filtered_source_cloud->size(); i++) {
        std::vector<int> neighbours;
        kdtree.radiusSearch(*filtered_source_cloud, i, radius, neighbours, distances, max_neighbours);
        for (int j : neighbours) {
            tripletList.push_back(Eigen::Triplet<int>(i, j, 1));
        }
    }
    data_association.setFromTriplets(tripletList.begin(), tripletList.end());
    data_association.makeCompressed();

    PointCloudRegistrationParams params;
    params.dof = std::numeric_limits<double>::infinity();
    params.max_neighbours = max_neighbours;
    params.dimension = 3;

    PointCloudRegistrationIteration registration(*filtered_source_cloud, *target_cloud,
                                                 data_association,
                                                 params);
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.use_nonmonotonic_steps = true;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = std::numeric_limits<int>::max();
    options.function_tolerance = 10e-16;
    options.num_threads = 8;
    ceres::Solver::Summary summary;
    registration.solve(options, &summary);

    std::cout << summary.FullReport() << std::endl;
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

    std::string aligned_source_name = "aligned_" + source_file_name;
    std::cout << "Saving aligned source cloud to: " << aligned_source_name.c_str() << std::endl;
    pcl::io::savePCDFile(aligned_source_name, *aligned_source);

    return 0;
}
