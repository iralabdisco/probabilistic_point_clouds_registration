#ifndef PROB_POINT_CLOUD_REGISTRATION_POINT_CLOUD_REGISTRATION_HPP
#define PROB_POINT_CLOUD_REGISTRATION_POINT_CLOUD_REGISTRATION_HPP

#include <sstream>

#include <Eigen/Core>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>


#include "prob_point_cloud_registration/output_stream.hpp"
#include "prob_point_cloud_registration/prob_point_cloud_registration_iteration.hpp"
#include "prob_point_cloud_registration/prob_point_cloud_registration_params.hpp"

namespace prob_point_cloud_registration {

class ProbPointCloudRegistration
{
public:
    ProbPointCloudRegistration(
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud,
        ProbPointCloudRegistrationParams parameters);
    ProbPointCloudRegistration(
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud,
        ProbPointCloudRegistrationParams parameters,
        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_truth_cloud);
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

    inline std::string report()
    {
        return report_.str();
    }

private:
    ProbPointCloudRegistrationParams parameters_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_;
    pcl::KdTreeFLANN<pcl::PointXYZ> target_kdtree_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_source_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr prev_source_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_truth_cloud_;
    bool ground_truth_;
    double mse_ground_truth_;
    double mse_prev_it_;
    double cost_drop_;
    int num_unusefull_iter_;
    int current_iteration_;
    OutputStream output_stream_;
    std::vector<Eigen::Affine3d> transformation_history_;
    std::stringstream report_;
    pcl::VoxelGrid<pcl::PointXYZ> filter_;
};

}  // namespace prob_point_cloud_registration

#endif
