#ifndef POINT_CLOUD_REGISTRATION_POINT_CLOUD_REGISTRATION_H
#define POINT_CLOUD_REGISTRATION_POINT_CLOUD_REGISTRATION_H

#include <ceres/ceres.h>
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

#include "point_cloud_registration/weight_updater.h"
#include "point_cloud_registration/weighted_error_term.h"

namespace point_cloud_registration {

typedef std::vector<std::unique_ptr<WeightedErrorTerm>> WeightedErrorTermGroup;

class PointCloudRegistration: public ceres::IterationCallback {
 public:
  PointCloudRegistration(const pcl::PointCloud<pcl::PointXYZ>& source_cloud,
                         const pcl::PointCloud<pcl::PointXYZ>& target_cloud,
                         Eigen::Sparse<int>& data_association,
                         double dof,
                         const double rotation_initial_guess[4] =
                             PointCloudRegistration::default_rotation,
                         const double translation_initial_guess[3] =
                             PointCloudRegistration::default_translation);

  void solve(ceres::Solver::Options options, ceres::Solver::Summary* Summary);
  Eigen::Affine3d transformation();

 private:
  std::vector<ErrorTerm> error_terms_;
  std::unique_ptr<ceres::Problem> problem_;
  double rotation_[4];
  double translation_[3];
  static const double default_rotation[4];
  static const double default_translation[3];
  int max_neighbours_;
  Eigen::Sparse<int> data_association_;
};

}  // namespace point_cloud_registration

#endif
