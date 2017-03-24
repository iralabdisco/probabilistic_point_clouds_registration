#include <limits>
#include <memory>
#include <vector>
#include <fstream>
#include <string>
#include <stdlib.h>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <tclap/CmdLine.h>

#include "point_cloud_registration/point_cloud_registration.h"
#include "point_cloud_registration/utilities.hpp"

typedef pcl::PointXYZ PointType;

using point_cloud_registration::PointCloudRegistration;
using point_cloud_registration::PointCloudRegistrationParams;

int main(int argc, char **argv)
{
    bool use_gaussian = false, ground_truth = false;
    std::string source_file_name, target_file_name;
    PointCloudRegistrationParams params;

    try {
        TCLAP::CmdLine cmd("Objective Function Analyzer", ' ', "1.0");
        TCLAP::UnlabeledValueArg<std::string> source_file_name_arg("source_file_name",
                                                                   "The path of the source point cloud", true, "source_cloud.pcd", "string", cmd);
        TCLAP::UnlabeledValueArg<std::string> target_file_name_arg("target_file_name",
                                                                   "The path of the target point cloud", true, "target_cloud.pcd", "string", cmd);
        TCLAP::ValueArg<float> source_filter_arg("s", "source_filter_size",
                                                 "The leaf size of the voxel filter of the source cloud", false, 0, "float", cmd);
        TCLAP::ValueArg<float> target_filter_arg("t", "target_filter_size",
                                                 "The leaf size of the voxel filter of the target cloud", false, 0, "float", cmd);
        TCLAP::ValueArg<int> max_neighbours_arg("m", "max_neighbours",
                                                "The max cardinality of the neighbours' set", false, 10, "int", cmd);
        TCLAP::ValueArg<int> num_iter_arg("i", "num_iter",
                                          "The maximum number of iterations to perform", false, 10, "int", cmd);
        TCLAP::ValueArg<float> dof_arg("d", "dof", "The Degree of freedom of t-distribution", false, 5,
                                       "float", cmd);
        TCLAP::ValueArg<float> radius_arg("r", "radius", "The radius of the neighborhood search", false, 3,
                                          "float", cmd);
        TCLAP::ValueArg<float> cost_drop_tresh_arg("c", "cost_drop_treshold",
                                                   "If the cost_drop drops below this threshold for too many iterations, the algorithm terminate",
                                                   false, 0.01, "float", cmd);
        TCLAP::ValueArg<int> num_drop_iter_arg("n", "num_drop_iter",
                                               "The maximum number of iterations during which the cost drop is allowed to be under cost_drop_thresh",
                                               false, 5, "int", cmd);
        TCLAP::SwitchArg use_gaussian_arg("u", "use_gaussian",
                                          "Whether to use a gaussian instead the a t-distribution", cmd, false);
        TCLAP::SwitchArg verbose_arg("v", "verbose",
                                     "Verbosity", cmd, false);
        cmd.parse(argc, argv);

        params.max_neighbours = max_neighbours_arg.getValue();
        use_gaussian = use_gaussian_arg.getValue();
        params.dof = dof_arg.getValue();
        params.radius = radius_arg.getValue();
        params.n_iter = num_iter_arg.getValue();
        params.verbose = verbose_arg.getValue();
        params.cost_drop_thresh = cost_drop_tresh_arg.getValue();
        params.n_cost_drop_it = num_drop_iter_arg.getValue();
        source_file_name = source_file_name_arg.getValue();
        target_file_name = target_file_name_arg.getValue();
        params.source_filter_size = source_filter_arg.getValue();
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
    std::cout << "Cost drop threshold: " << params.cost_drop_thresh << std::endl;
    std::cout << "Num cost drop iter: " << params.n_cost_drop_it << std::endl;
    std::cout << "Loading source point cloud from " << source_file_name << std::endl;
    pcl::PointCloud<PointType>::Ptr source_cloud =
        boost::make_shared<pcl::PointCloud<PointType>>();
    pcl::PointCloud<PointType>::Ptr target_cloud =
        boost::make_shared<pcl::PointCloud<PointType>>();
    if (pcl::io::loadPCDFile<PointType>(source_file_name, *source_cloud) == -1) {
        std::cout << "Could not load source cloud, closing" << std::endl;
        exit(EXIT_FAILURE);
    }

    if (pcl::io::loadPCDFile<PointType>(target_file_name, *target_cloud) == -1) {
        std::cout << "Could not load target cloud, closing" << std::endl;
        exit(EXIT_FAILURE);
    }

    std::unique_ptr<PointCloudRegistration> registration;
    pcl::PointCloud<PointType>::Ptr aligned_source = boost::make_shared<pcl::PointCloud<PointType>>();
    pcl::PointCloud<PointType>::Ptr moved_source = boost::make_shared<pcl::PointCloud<PointType>>();
    std::string report_file_name = source_file_name + "_analyzer_summary.txt";
    std::ofstream report_file;
    report_file.open(report_file_name);
    report_file << "Source: " << source_file_name << " with filter size: " << params.source_filter_size
                <<
                std::endl;
    report_file << "dof: " << params.dof << " | Radius: " << params.radius << " | Max_iter: " <<
                params.n_iter << " | Max neigh: " << params.max_neighbours << " | Cost_drop_thresh_: " <<
                params.cost_drop_thresh << " | N_cost_drop_it: " << params.n_cost_drop_it << std::endl;
    auto axis = Eigen::Vector3d(0, 0, 1);
//    report_file << "axis = " << axis[0] << " " << axis[1] << " " << axis[2] << std::endl;
    report_file << "Displacement, MSE_Initial, MSE_G_Truth, Score" << std::endl;
//    for (double angle = 0.08; angle < 3.14; angle += 0.08) {
//        //0.08 rad = 5 deg

//        Eigen::Affine3d trans(Eigen::AngleAxis<double>(angle, axis));
//        pcl::transformPointCloud (*source_cloud, *moved_source, trans);
//        registration = std::make_unique<PointCloudRegistration>(moved_source, target_cloud, params,
//                                                                source_cloud);
//        registration->align();
//        auto estimated_transform = registration->transformation();
//        pcl::transformPointCloud (*moved_source, *aligned_source, estimated_transform);
//        double mse_ground_truth = point_cloud_registration::calculateMSE(aligned_source,
//                                                                         source_cloud);
//        double mse_initial = point_cloud_registration::calculateMSE(moved_source, source_cloud);
//        double score = point_cloud_registration::medianClosestDistance(aligned_source, target_cloud);
//        report_file << angle << ", " << mse_initial << ", " << mse_ground_truth << ", " << score <<
//                    std::endl;
//        std::cout << "Registered angle " << angle << ", MSE " << mse_ground_truth << std::endl;
//    }
    for (double displacement = 0.01; displacement < 10; displacement += 0.1) {

        Eigen::Affine3d trans(Eigen::Translation<double, 3>(displacement, displacement, displacement));
        pcl::transformPointCloud (*source_cloud, *moved_source, trans);
        params.radius = displacement;
        registration = std::make_unique<PointCloudRegistration>(moved_source, target_cloud, params,
                                                                source_cloud);
        registration->align();
        auto estimated_transform = registration->transformation();
        pcl::transformPointCloud (*moved_source, *aligned_source, estimated_transform);
        double mse_ground_truth = point_cloud_registration::calculateMSE(aligned_source,
                                                                         source_cloud);
        double mse_initial = point_cloud_registration::calculateMSE(moved_source, source_cloud);
        double score = point_cloud_registration::averageClosestDistance(aligned_source, target_cloud);
        report_file << displacement << ", " << mse_initial << ", " << mse_ground_truth << ", " << score <<
                    std::endl;
        std::cout << "Registered displacement " << displacement << ", MSE " << mse_ground_truth <<
                  std::endl;
    }
    report_file.close();

    return 0;
}
