#include <limits>
#include <memory>
#include <vector>
#include <fstream>
#include <string>
#include <stdlib.h>

#include <boost/make_shared.hpp>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <tclap/CmdLine.h>

#include "point_cloud_registration/utilities.hpp"


using point_cloud_registration::medianClosestDistance;
using point_cloud_registration::robustMedianClosestDistance;
using point_cloud_registration::averageClosestDistance;
using point_cloud_registration::calculateMSE;
using point_cloud_registration::sumSquaredError;
using point_cloud_registration::robustSumSquaredError;
using point_cloud_registration::robustAveragedSumSquaredError;

typedef pcl::PointXYZ PointType;

int main(int argc, char **argv)
{
    std::string source_cloud_file_name;
    std::string target_cloud_file_name;
    std::string report_file_name;
    bool test_trans = false;
    double source_filter_size, target_filter_size;
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;

    try
    {
        TCLAP::CmdLine cmd("Metri tester", ' ', "1.0");

        TCLAP::UnlabeledValueArg<std::string> source_file_name_arg("source_file_name",
                                                                   "The path of the source point cloud", true, "source_cloud.pcd", "string", cmd);
        TCLAP::UnlabeledValueArg<std::string> target_file_name_arg("target_file_name",
                                                                   "The path of the target point cloud", true, "target_cloud.pcd", "string", cmd);
        TCLAP::UnlabeledValueArg<std::string> report_file_name_arg("report_file_name",
                                                                   "The name of the report file", true, "report.pcd", "string", cmd);
        TCLAP::SwitchArg test_trans_arg("", "trans",
                                        "Whether to the for the translation instead than rotation", cmd,
                                        false);
        TCLAP::ValueArg<float> source_filter_arg("s", "source_filter_size",
                                                 "The leaf size of the voxel filter of the source cloud", false, 0, "float", cmd);
        TCLAP::ValueArg<float> target_filter_arg("t", "target_filter_size",
                                                 "The leaf size of the voxel filter of the target cloud", false, 0, "float", cmd);
        cmd.parse(argc, argv);
        source_cloud_file_name = source_file_name_arg.getValue();
        target_cloud_file_name = target_file_name_arg.getValue();
        report_file_name = report_file_name_arg.getValue();
        test_trans = test_trans_arg.getValue();
        source_filter_size = source_filter_arg.getValue();
        target_filter_size = target_filter_arg.getValue();
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

    if (source_filter_size != 0)
    {
        voxel_filter.setInputCloud(source_cloud);
        voxel_filter.setLeafSize(source_filter_size, source_filter_size, source_filter_size);
        voxel_filter.filter(*source_cloud);
    }

    if (target_filter_size != 0)
    {
        voxel_filter.setInputCloud(target_cloud);
        voxel_filter.setLeafSize(target_filter_size, target_filter_size, target_filter_size);
        voxel_filter.filter(*target_cloud);
    }

    std::ofstream report_file;
    report_file.open(report_file_name);
    report_file << "Source cloud: " << source_cloud_file_name << " Target cloud: " <<
                target_cloud_file_name << std::endl;
    if (test_trans) {
        report_file <<
                    "tx, ty, MSE_gtruth, MedianClosestDistance, RobustMedian, sse, robustSSE, robustAvgSSE" <<
                    std::endl;
    } else {
        // report_file <<
                    // "Pitch, Yaw,  MSE_gtruth, MedianClosestDistance, RobustMedian, sse, robustSSE, robustAvgSSE"
                    // << std::endl;
    }

    if (test_trans) {
        for (double tx = 0; tx <= 5; tx += 0.1) {
            for (double ty = 0; ty <= 5; ty += 0.1) {
                pcl::PointCloud<PointType>::Ptr moved_source_cloud =
                    boost::make_shared<pcl::PointCloud<PointType>>();
                Eigen::Affine3d trans(Eigen::Affine3d::Identity());
                trans.translate(Eigen::Vector3d(tx, ty, 0));
                pcl::transformPointCloud (*source_cloud, *moved_source_cloud, trans);

                //            double mse_gtruth = calculateMSE(moved_source_cloud, target_cloud);
                double mse_gtruth = 1;
                double median_score = medianClosestDistance(moved_source_cloud, target_cloud);
                double robust_median_score = robustMedianClosestDistance(moved_source_cloud, target_cloud);
                double sse = sumSquaredError (moved_source_cloud, target_cloud);
                double robustSse = robustSumSquaredError(moved_source_cloud, target_cloud);
                double robustAveSSe = robustAveragedSumSquaredError(moved_source_cloud, target_cloud);
                report_file << tx << ", " << ty << ", " << mse_gtruth << ", " << median_score << ", " <<
                            robust_median_score << ", " << sse <<  ", " << robustSse << ", " << robustAveSSe << std::endl;
            }
        }
    } else {
        for (double pitch = 0; pitch <= 360; pitch += 1) {
            // for (double yaw = 0; yaw <= 360; yaw += 3) {
                double yaw = 0;
                pcl::PointCloud<PointType>::Ptr moved_source_cloud =
                    boost::make_shared<pcl::PointCloud<PointType>>();
                Eigen::Affine3d trans(Eigen::Affine3d::Identity());
                // trans.rotate(point_cloud_registration::euler2Quaternion(0, pitch * 0.0174533, yaw * 0.0174533));
                trans.rotate(Eigen::AngleAxisd(pitch * 0.0174533, Eigen::Vector3d(0,1,1)));

                pcl::transformPointCloud (*source_cloud, *moved_source_cloud, trans);

//            double mse_gtruth = calculateMSE(moved_source_cloud, target_cloud);
                // double mse_gtruth = 1;
                // double median_score = medianClosestDistance(moved_source_cloud, target_cloud);
                // double robust_median_score = robustMedianClosestDistance(moved_source_cloud, target_cloud);
                // double sse = sumSquaredError (moved_source_cloud, target_cloud);
                // double robustSse = robustSumSquaredError(moved_source_cloud, target_cloud);
                // double robustAveSSe = robustAveragedSumSquaredError(moved_source_cloud, target_cloud);
                
                report_file << pitch<<" , ";

                for(int i =2;i<=6;i++){
                    report_file << robustSumSquaredError(moved_source_cloud, target_cloud,i)<<" , ";
                }
                report_file<<std::endl;
                // report_file << pitch << ", " << yaw << ", " << mse_gtruth << ", " << median_score << ", " <<
                //             robust_median_score << ", " << sse <<  ", " << robustSse << ", " << robustAveSSe << std::endl;
            // }
        }

    }
    return 0;
}
