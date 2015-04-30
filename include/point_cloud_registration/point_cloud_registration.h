#ifndef POINT_CLOUD_REGISTRATION_POINT_CLOUD_REGISTRATION_H
#define POINT_CLOUD_REGISTRATION_POINT_CLOUD_REGISTRATION_H

#include <ceres/ceres.h>
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

#include "point_cloud_registration/weight_updater.h"
#include "point_cloud_registration/weighted_error_term.h"

namespace point_cloud_registration {

typedef std::vector<std::shared_ptr<WeightedErrorTerm>> WeightedErrorTermGroup;

class PointCloudRegistration {
 public:
  PointCloudRegistration(const pcl::PointCloud<pcl::PointXYZ>& source_cloud,
                         const pcl::PointCloud<pcl::PointXYZ>& target_cloud,
                         const std::vector<std::vector<int>>& data_association,
                         double dof,
                         const double rotation_initial_guess[4] =
                             PointCloudRegistration::default_rotation,
                         const double translation_initial_guess[3] =
                             PointCloudRegistration::default_translation);

  void Solve(ceres::Solver::Options* options, ceres::Solver::Summary* Summary);
  std::unique_ptr<Eigen::Quaternion<double>> rotation();
  std::unique_ptr<Eigen::Vector3d> translation();
  inline std::vector<std::shared_ptr<Eigen::Quaternion<double>>>*
  rotation_history() {
    return weight_updater_->rotation_history();
  }
  inline std::vector<std::shared_ptr<Eigen::Vector3d>>* translation_history() {
    return weight_updater_->translation_history();
  }

 private:
  std::vector<WeightedErrorTermGroup> weighted_error_terms_;
  std::unique_ptr<ceres::Problem> problem_;
  ceres::Solver::Options problem_options_;
  ceres::Solver::Summary problem_summary_;
  std::unique_ptr<WeightUpdater> weight_updater_;
  double rotation_[4];
  double translation_[3];
  static const double default_rotation[4];
  static const double default_translation[3];
};

}  // namespace point_cloud_registration

#endif
