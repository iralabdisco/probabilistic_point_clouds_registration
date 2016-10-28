#ifndef POINT_CLOUD_REGISTRATION_POINT_CLOUD_REGISTRATION_H
#define POINT_CLOUD_REGISTRATION_POINT_CLOUD_REGISTRATION_H

#include <fstream>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "point_cloud_registration/point_cloud_registration_iteration.hpp"
#include "point_cloud_registration/point_cloud_registration_params.hpp"

namespace point_cloud_registration {

class PointCloudRegistration
{
public:
    PointCloudRegistration(
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud,
        PointCloudRegistrationParams parameters);
    void align();
    bool hasConverged();
    inline Eigen::Affine3d transformation()
    {
        return transformation_history_.back();
    }

    inline std::vector<Eigen::Affine3d> transformation_history()
    {
        return transformation_history_;
    }

private:
    PointCloudRegistrationParams parameters_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr prev_source_cloud_;
    int current_iteration_;
    std::vector<Eigen::Affine3d> transformation_history_;
    std::ofstream debug_file_;
};

}  // namespace point_cloud_registration

#endif
