#include <limits>
#include <memory>
#include <vector>
#include <fstream>
#include <string>
#include <stdlib.h>

#include <boost/make_shared.hpp>

#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <tclap/CmdLine.h>

#include "point_cloud_registration/utilities.hpp"
#include "point_cloud_registration/particle.hpp"
#include "point_cloud_registration/swarm.hpp"



typedef pcl::PointXYZ PointType;

using point_cloud_registration::Particle;
using point_cloud_registration::Swarm;
using point_cloud_registration::calculateMSE;

int main(int argc, char **argv)
{
    std::string source_file_name;
    std::string target_file_name;
    std::string ground_truth_name;
    bool ground_truth = false, display=false;
    int num_part = 0, num_gen = 0;
    double source_filter_size, target_filter_size;
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;

    try {
        TCLAP::CmdLine cmd("PSO Initial Guess", ' ', "1.0");
        TCLAP::UnlabeledValueArg<std::string> source_file_name_arg("source_file_name",
                                                                   "The path of the source point cloud", true, "source_cloud.pcd", "string", cmd);
        TCLAP::UnlabeledValueArg<std::string> target_file_name_arg("target_file_name",
                                                                   "The path of the target point cloud", true, "target_cloud.pcd", "string", cmd);
        TCLAP::ValueArg<float> source_filter_arg("s", "source_filter_size",
                                                 "The leaf size of the voxel filter of the source cloud", false, 0, "float", cmd);
        TCLAP::ValueArg<float> target_filter_arg("t", "target_filter_size",
                                                 "The leaf size of the voxel filter of the target cloud", false, 0, "float", cmd);
        TCLAP::ValueArg<int> num_part_arg("p", "num_part",
                                          "The number of particles of the swarm", false, 50, "int", cmd);
        TCLAP::ValueArg<int> num_gen_arg("e", "num_it",
                                         "The number of iterations (generations) of the algorithm", false, 1000, "int", cmd);
        TCLAP::ValueArg<std::string> ground_truth_arg("g", "ground_truth",
                                                      "The path of the ground truth for the source cloud, if available", false, "ground_truth.pcd",
                                                      "string", cmd);
        TCLAP::SwitchArg verbose_arg("v", "verbose", "Verbosity", cmd, false);
        cmd.parse(argc, argv);


        source_file_name = source_file_name_arg.getValue();
        target_file_name = target_file_name_arg.getValue();
        source_filter_size = source_filter_arg.getValue();
        target_filter_size = target_filter_arg.getValue();
        num_part = num_part_arg.getValue();
        num_gen = num_gen_arg.getValue();
        display = verbose_arg.getValue();

        if (ground_truth_arg.isSet()) {
            ground_truth = true;
            ground_truth_name = ground_truth_arg.getValue();
        }

    } catch (TCLAP::ArgException &e) {
        std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
        exit(EXIT_FAILURE);
    }

    std::cout << "Loading source point cloud from " << source_file_name << std::endl;
    pcl::PointCloud<PointType>::Ptr source_cloud = boost::make_shared<pcl::PointCloud<PointType>>();
    if (pcl::io::loadPCDFile(source_file_name, *source_cloud) == -1) {
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

 pcl::PointCloud<PointType>::Ptr filtered_source_cloud = boost::make_shared<pcl::PointCloud<PointType>>();
    if (source_filter_size != 0) {
        voxel_filter.setInputCloud(source_cloud);
        voxel_filter.setLeafSize(source_filter_size, source_filter_size, source_filter_size);
        voxel_filter.filter(*filtered_source_cloud);
    }
    else{
        *filtered_source_cloud = *source_cloud;
    }

    if (target_filter_size != 0) {
        voxel_filter.setInputCloud(target_cloud);
        voxel_filter.setLeafSize(target_filter_size, target_filter_size, target_filter_size);
        voxel_filter.filter(*target_cloud);
    }

    pcl::PointCloud<PointType>::Ptr ground_truth_cloud = boost::make_shared<pcl::PointCloud<PointType>>();
    pcl::PointCloud<PointType>::Ptr filtered_ground_truth_cloud = boost::make_shared<pcl::PointCloud<PointType>>();

    if (ground_truth) {
        std::cout << "Loading ground truth point cloud from " << ground_truth_name << std::endl;
        if (pcl::io::loadPCDFile<PointType>(ground_truth_name, *ground_truth_cloud) == -1) {
            std::cout << "Could not load ground truth" << std::endl;
            ground_truth = false;
        } else {
            if (source_filter_size != 0) {
                voxel_filter.setInputCloud(ground_truth_cloud);
                voxel_filter.setLeafSize(source_filter_size, source_filter_size, source_filter_size);
                voxel_filter.filter(*filtered_ground_truth_cloud);
            }else{
                *filtered_ground_truth_cloud = *ground_truth_cloud;
            }
        }

    }
        pcl::visualization::PCLVisualizer viewer("PSO Viewer");

if(display){
    viewer.setBackgroundColor(255, 255, 255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color (target_cloud, 0, 255,
                                                                                  0);

    viewer.addPointCloud<pcl::PointXYZ> (target_cloud, target_color, "target");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color (filtered_source_cloud, 0,
                                                                                  0, 255);
    viewer.addPointCloud<pcl::PointXYZ> (filtered_source_cloud, source_color, "source");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> truth_color (filtered_ground_truth_cloud,
                                                                                 255,
                                                                                 0, 0);
    viewer.addPointCloud<pcl::PointXYZ> (filtered_ground_truth_cloud, truth_color, "groundTruth");
}
    Swarm swarm;
    for (int i = 0; i < num_part; i++) {
        swarm.add_particle(Particle(filtered_source_cloud, target_cloud, i));
    }
    Particle best;
    swarm.init();
    if (display)
    {
        std::cout << swarm << std::endl;
    }
    for (int i = 0; i < num_gen; i++)
    {
        swarm.evolve();
        best = swarm.getBest();
        if (display)
        {
            std::cout << swarm << std::endl;
            viewer.updatePointCloudPose("source", best.getTransformation().cast<float>());
            viewer.spinOnce(1);
        }
    }
    if (display)
    {
        viewer.spin();
    }
    pcl::transformPointCloud (*source_cloud, *source_cloud, best.getTransformation());
    pcl::io::savePCDFile("guess_"+source_file_name, *source_cloud);
    double mse_gtruth = calculateMSE(source_cloud, ground_truth_cloud);
    std::cout<<"MSE Gtruth: "<<mse_gtruth<<std::endl;
    std::cerr<<mse_gtruth<<std::endl;
    return 0;
}
