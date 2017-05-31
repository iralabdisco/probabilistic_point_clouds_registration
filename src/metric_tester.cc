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
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <tclap/CmdLine.h>

#include "point_cloud_registration/utilities.hpp"


using point_cloud_registration::medianClosestDistance;
using point_cloud_registration::robustMedianClosestDistance;
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
    auto axis = Eigen::Vector3d(1, 1, 1);
    report_file <<
                "Pitch, Yaw,  MSE_gtruth, MedianClosestDistance, AverageClosestDistance, RobustMedianClosestDistance"
                << std::endl;
//    report_file <<
//                "Pitch, Yaw,  cross_prod, squaredNorm"
//            << std::endl;
    pcl::PCA<pcl::PointXYZ> pca_target(*target_cloud);
    auto target_eigen_vectors = pca_target.getEigenVectors();

    for (double pitch = 0; pitch <= 360; pitch += 1) {
        for (double yaw = 0; yaw <= 360; yaw += 1) {
            pcl::PointCloud<PointType>::Ptr moved_source_cloud =
                boost::make_shared<pcl::PointCloud<PointType>>();
            Eigen::Affine3d trans(Eigen::Affine3d::Identity());
            trans.rotate(point_cloud_registration::euler2Quaternion(0, pitch * 0.0174533, yaw * 0.0174533));

            pcl::transformPointCloud (*source_cloud, *moved_source_cloud, trans);
//            pcl::PCA<pcl::PointXYZ> pca_source(*moved_source_cloud);
//            auto source_eigen_vectors = pca_source.getEigenVectors();
//            double res = 0;
//            for (int i = 0; i < 3; i++) {
//                double closest = 100000000000000000;
//                for (int j = 0; j < 3; j++) {
//                    double distance = (source_eigen_vectors.col(i) - target_eigen_vectors.col(i)).squaredNorm();
//                    if (distance < closest) {
//                        closest = distance;
//                    }
//                }
//                res += closest;
//            }

//            double mse_gtruth = calculateMSE(moved_source_cloud, target_cloud);
//            double median_score = medianClosestDistance(moved_source_cloud, target_cloud);
//            double average_score = averageClosestDistance(moved_source_cloud, target_cloud);
            double robust_median_score = point_cloud_registration::sumSquaredError(moved_source_cloud,
                                                                                   target_cloud);

//            report_file << pitch << ", " << yaw << ", " << mse_gtruth << ", " << median_score << ", " <<
//                        average_score << ", "
//                        << robust_median_score << std::endl;
//            report_file << pitch << ", " << yaw << ", " << res[0] << ", " << res[1] << ", " << res[2] << ", " <<
//                        res.squaredNorm() <<
//                        std::endl;
            report_file << pitch << ", " << yaw << ", " << robust_median_score << std::endl;
//        report_file << disp << ", " << mse_gtruth << ", " << median_score << ", " << average_score <<
//                    std::endl;
        }
    }


    return 0;
}
