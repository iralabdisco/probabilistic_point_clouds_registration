#include <limits>
#include <memory>
#include <vector>
#include <fstream>
#include <string>
#include <stdlib.h>

#include <boost/make_shared.hpp>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <tclap/CmdLine.h>

#include "point_cloud_registration/utilities.hpp"


using point_cloud_registration::medianClosestDistance;
using point_cloud_registration::averageClosestDistance;
using point_cloud_registration::calculateMSE;

typedef pcl::PointXYZ PointType;

int main(int argc, char **argv)
{
    std::string source_cloud_file_name;
    std::string target_cloud_file_name;
    std::string report_file_name;

    try {
        TCLAP::CmdLine cmd("Metri tester", ' ', "1.0");

        TCLAP::UnlabeledValueArg<std::string> source_file_name_arg("source_file_name",
                                                                   "The path of the source point cloud", true, "source_cloud.pcd", "string", cmd);
        TCLAP::UnlabeledValueArg<std::string> target_file_name_arg("target_file_name",
                                                                   "The path of the target point cloud", true, "target_cloud.pcd", "string", cmd);
        TCLAP::UnlabeledValueArg<std::string> report_file_name_arg("report_file_name",
                                                                   "The name of the report file", true, "report.pcd", "string", cmd);
        cmd.parse(argc, argv);
        source_cloud_file_name = source_file_name_arg.getValue();
        target_cloud_file_name = target_file_name_arg.getValue();
        report_file_name = report_file_name_arg.getValue();
    } catch (TCLAP::ArgException &e) {
        std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
        exit(EXIT_FAILURE);
    }


    pcl::PointCloud<PointType>::Ptr source_cloud = boost::make_shared<pcl::PointCloud<PointType>>();
    if (pcl::io::loadPCDFile<PointType>(source_cloud_file_name, *source_cloud) == -1) {
        std::cout << "Could not load source cloud, closing" << std::endl;
        exit(EXIT_FAILURE);
    }

    pcl::PointCloud<PointType>::Ptr target_cloud = boost::make_shared<pcl::PointCloud<PointType>>();
    if (pcl::io::loadPCDFile<PointType>(target_cloud_file_name, *target_cloud) == -1) {
        std::cout << "Could not load target cloud, closing" << std::endl;
        exit(EXIT_FAILURE);
    }

    std::ofstream report_file;
    report_file.open(report_file_name);
    report_file << "Source cloud: " << source_cloud_file_name << " Target cloud: " <<
                target_cloud_file_name << std::endl;
    auto axis = Eigen::Vector3d(0, 1, 0);
    report_file << "Axis: " << axis[0] << " " << axis[1] << " " << axis[2] << std::endl;
    report_file << "Angle, MSE_gtruth, MedianClosestDistance, AverageClosestDistance" << std::endl;
    for (double disp = 0; disp <= 2; disp += 0.01) {
        //for (double angle = 0; angle <= 360; angle += 1) {
        pcl::PointCloud<PointType>::Ptr moved_source_cloud =
            boost::make_shared<pcl::PointCloud<PointType>>();
        //Eigen::Affine3d trans(Eigen::AngleAxis<double>(angle * 0.0174533, axis));
        Eigen::Affine3d trans(Eigen::Translation<double, 3>(disp, 0, 0));
        pcl::transformPointCloud (*source_cloud, *moved_source_cloud, trans);
        double mse_gtruth = calculateMSE(moved_source_cloud, target_cloud);
        double median_score = medianClosestDistance(moved_source_cloud, target_cloud);
        double average_score = averageClosestDistance(moved_source_cloud, target_cloud);
//        report_file << angle << ", " << mse_gtruth << ", " << median_score << ", " << average_score <<
//                    std::endl;
        report_file << disp << ", " << mse_gtruth << ", " << median_score << ", " << average_score <<
                    std::endl;
    }


    return 0;
}
