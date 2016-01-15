#include <iostream>
#include <limits>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <gtest/gtest.h>
#include "point_cloud_registration/point_cloud_registration.h"

using point_cloud_registration::PointCloudRegistration;
using point_cloud_registration::PointCloudRegistrationParams;
pcl::PointCloud<pcl::PointXYZ> randomCloud()
{
    pcl::PointCloud<pcl::PointXYZ> cloud;

    // Fill in the CloudIn data
    cloud.width    = 20;
    cloud.height   = 1;
    cloud.is_dense = false;
    cloud.points.resize (cloud.width * cloud.height);
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
        cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    }
    return cloud;
}

TEST(PointCloudRegistrationTestSuite, exactDataAssociationTest)
{
    auto source_cloud = randomCloud();
    pcl::PointCloud<pcl::PointXYZ> target_cloud;
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << 2.5, 0.0, 0.0;
    transform.prerotate (Eigen::AngleAxisf (0.34, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud(source_cloud, target_cloud, transform);
    Eigen::SparseMatrix<int, Eigen::RowMajor> data_association(source_cloud.width, target_cloud.width);
    std::vector<Eigen::Triplet<int>> tripletList;
    for (std::size_t i = 0; i < source_cloud.width; ++i)
    {
        tripletList.push_back(Eigen::Triplet<int>(i, i, 1));
    }
    data_association.setFromTriplets(tripletList.begin(), tripletList.end());
    data_association.makeCompressed();
    PointCloudRegistrationParams params;
    params.dof = std::numeric_limits<double>::infinity();
    params.max_neighbours = 1;
    params.dimension = 3;
    PointCloudRegistration registration(source_cloud, target_cloud, data_association, params);
}
