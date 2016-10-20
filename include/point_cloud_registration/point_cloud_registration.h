#ifndef POINT_CLOUD_REGISTRATION_POINT_CLOUD_REGISTRATION_H
#define POINT_CLOUD_REGISTRATION_POINT_CLOUD_REGISTRATION_H

#include <ceres/ceres.h>
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

#include "point_cloud_registration/point_cloud_registration_iteration.h"
#include "point_cloud_registration/point_cloud_registration_params.hpp"

namespace point_cloud_registration {

class PointCloudRegistration
{
public:
    PointCloudRegistration(
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud,
        PointCloudRegistrationParams parameters);
    void align(bool verbose);
    inline Eigen::Affine3d transformation()
    {
        return transformation_;
    }

private:
    PointCloudRegistrationParams parameters_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_;
    Eigen::Affine3d transformation_;
};

}  // namespace point_cloud_registration

#endif
