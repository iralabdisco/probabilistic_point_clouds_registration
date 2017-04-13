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
    std::string source_file_name;
    std::string target_file_name;
    std::string ground_truth_file_name;
    PointCloudRegistrationParams params;
    Eigen::Affine3d initial_guess(Eigen::Affine3d::Identity());
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
        TCLAP::ValueArg<int> max_neighbours_arg("m", "max_neighbours",
                                                "The max cardinality of the neighbours' set", false, 10, "int", cmd);
        TCLAP::ValueArg<int> num_iter_arg("i", "num_iter",
                                          "The maximum number of iterations to perform", false, 10, "int", cmd);
        TCLAP::ValueArg<float> dof_arg("d", "dof", "The Degree of freedom of t-distribution", false, 5,
                                       "float", cmd);
        TCLAP::ValueArg<float> radius_arg("r", "radius", "The radius of the neighborhood search", false, 3,
                                          "float", cmd);
        TCLAP::ValueArg<float> tx_arg("", "tx", "tx", false, 0, "float", cmd);
        TCLAP::ValueArg<float> ty_arg("", "ty", "ty", false, 0, "float", cmd);
        TCLAP::ValueArg<float> tz_arg("", "tz", "tz", false, 0, "float", cmd);

        TCLAP::ValueArg<float> roll_arg("", "roll", "roll", false, 0, "float", cmd);
        TCLAP::ValueArg<float> pitch_arg("", "pitch", "pitch", false, 0, "float", cmd);
        TCLAP::ValueArg<float> yaw_arg("", "yaw", "yaw", false, 0, "float", cmd);

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
        TCLAP::ValueArg<std::string> ground_truth_arg("g", "ground_truth",
                                                      "The path of the ground truth for the source cloud, if available", false, "ground_truth.pcd",
                                                      "string", cmd);
        TCLAP::SwitchArg dump_arg("", "dump",
                                  "Dump registration data to file", cmd, false);
        cmd.parse(argc, argv);

        params.max_neighbours = max_neighbours_arg.getValue();
        use_gaussian = use_gaussian_arg.getValue();
        params.dof = dof_arg.getValue();
        params.radius = radius_arg.getValue();
        params.n_iter = num_iter_arg.getValue();
        params.verbose = verbose_arg.getValue();
        params.cost_drop_thresh = cost_drop_tresh_arg.getValue();
        params.n_cost_drop_it = num_drop_iter_arg.getValue();
        params.summary = dump_arg.getValue();
        source_file_name = source_file_name_arg.getValue();
        target_file_name = target_file_name_arg.getValue();
        params.source_filter_size = source_filter_arg.getValue();
        params.target_filter_size = target_filter_arg.getValue();

        if (tx_arg.isSet()) {
            initial_guess.translate(Eigen::Vector3d(tx_arg.getValue(), ty_arg.getValue(), tz_arg.getValue()));
            initial_guess.rotate(point_cloud_registration::euler2Quaternion(roll_arg.getValue() * 0.0174533,
                                                                            pitch_arg.getValue() * 0.0174533,
                                                                            yaw_arg.getValue() * 0.0174533));
        }

        if (ground_truth_arg.isSet()) {
            ground_truth = true;
            ground_truth_file_name = ground_truth_arg.getValue();
        }

    } catch (TCLAP::ArgException &e) {
        std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
        exit(EXIT_FAILURE);
    }


    if (use_gaussian) {
        if (params.verbose) {
            std::cout << "Using gaussian model" << std::endl;
        }
        params.dof = std::numeric_limits<double>::infinity();
    } else {
        if (params.verbose) {
            std::cout << "Using a t-distribution with " << params.dof << " dof" << std::endl;
        }
    }
    if (params.verbose) {
        std::cout << "Radius of the neighborhood search: " << params.radius << std::endl;
        std::cout << "Max number of neighbours: " << params.max_neighbours << std::endl;
        std::cout << "Max number of iterations: " << params.n_iter << std::endl;
        std::cout << "Cost drop threshold: " << params.cost_drop_thresh << std::endl;
        std::cout << "Num cost drop iter: " << params.n_cost_drop_it << std::endl;
        std::cout << "Loading source point cloud from " << source_file_name << std::endl;
    }
    pcl::PointCloud<PointType>::Ptr source_cloud =
        boost::make_shared<pcl::PointCloud<PointType>>();
    if (pcl::io::loadPCDFile<PointType>(source_file_name, *source_cloud) == -1) {
        std::cout << "Could not load source cloud, closing" << std::endl;
        exit(EXIT_FAILURE);
    }
    pcl::transformPointCloud (*source_cloud, *source_cloud, initial_guess);

    if (params.verbose) {
        std::cout << "Loading target point cloud from " << target_file_name << std::endl;
    }
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

    std::unique_ptr<PointCloudRegistration> registration;
    if (ground_truth) {
        registration = std::make_unique<PointCloudRegistration>(source_cloud, target_cloud, params,
                                                                source_ground_truth);
    } else {
        registration = std::make_unique<PointCloudRegistration>(source_cloud, target_cloud,
                                                                params);
    }
    if (params.verbose) {
        std::cout << "Registration\n";
    }
    registration->align();
    auto estimated_transform = registration->transformation();
    pcl::PointCloud<PointType>::Ptr aligned_source = boost::make_shared<pcl::PointCloud<PointType>>();
    pcl::transformPointCloud (*source_cloud, *aligned_source, estimated_transform);
    if (params.verbose) {
        std::cout << "Transformation history:" << std::endl;
        for (auto trans : registration->transformation_history()) {
            Eigen::Quaterniond rotq(trans.rotation());
            std::cout << "T: " << trans.translation().x() << ", " << trans.translation().y() << ", " <<
                      trans.translation().z() << " ||| R: " << rotq.x() << ", " << rotq.y() << ", " << rotq.z() << ", " <<
                      rotq.w() << std::endl;
        }
        std::string aligned_source_name = "aligned_" + source_file_name;
        std::cout << "Saving aligned source cloud to: " << aligned_source_name.c_str() << std::endl;
        pcl::io::savePCDFile(aligned_source_name, *aligned_source);
    }
    if (params.summary) {
        std::string report_file_name = source_file_name + "_" + target_file_name + "_summary.txt";
        std::cout << "Saving registration report to: " << report_file_name << std::endl;
        std::ofstream report_file;
        report_file.open(report_file_name);
        report_file << "Source: " << source_file_name << " with filter size: " << params.source_filter_size
                    <<
                    std::endl;
        report_file << "Target:" << target_file_name << " with filter size: " << params.target_filter_size
                    <<
                    std::endl;
        report_file << "dof: " << params.dof << " | Radius: " << params.radius << " | Max_iter: " <<
                    params.n_iter << " | Max neigh: " << params.max_neighbours << " | Cost_drop_thresh_: " <<
                    params.cost_drop_thresh << " | N_cost_drop_it: " << params.n_cost_drop_it << std::endl;
        report_file << registration->report();
    }
    double score = point_cloud_registration::averageClosestDistance(aligned_source, target_cloud);
    std::cout << score;
    return 0;
}
