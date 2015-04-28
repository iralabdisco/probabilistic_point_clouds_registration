#ifndef POINT_CLOUD_REGISTRATION_POINT_CLOUD_REGISTRATION_H
#define POINT_CLOUD_REGISTRATION_POINT_CLOUD_REGISTRATION_H

#include <vector>
#include <Eigen/Core>
#include "ceres/ceres.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "point_cloud_registration/weightUpdater.h"
#include "point_cloud_registration/weighted_error_term.h"
namespace point_cloud_registration {

typedef std::vector<std::shared_ptr<WeightedErrorTerm>> WeightedErrorTermGroup;

class PointCloudRegistration {
 public:
  PointCloudRegistration(const pcl::PointCloud<pcl::PointXYZ>& source_cloud,
                         const pcl::PointCloud<pcl::PointXYZ>& target_cloud,
                         const std::vector<std::vector<int>>& data_association,
                         double dof);
  void Solve(ceres::Solver::Options* options,
             ceres::Solver::Summary* Summary);
  std::unique_ptr<Eigen::Quaternion<double>> rotation();
  std::unique_ptr<Eigen::Vector3d> translation();

 private:
  std::vector<WeightedErrorTermGroup> weighted_error_terms_;
  std::unique_ptr<ceres::Problem> problem_;
  ceres::Solver::Options problem_options_;
  ceres::Solver::Summary problem_summary_;
  std::unique_ptr<WeightUpdater> weight_updater_;
  double rotation_[4];
  double translation_[3];
};

}  // namespace point_cloud_registration

#endif
