#include <memory>
#include <vector>
#include "point_cloud_registration/pointCloudRegistration.h"

using std::size_t;

namespace point_cloud_registration {

PointCloudRegistration::PointCloudRegistration(
    const pcl::PointCloud<pcl::PointXYZ>& source_cloud,
    const pcl::PointCloud<pcl::PointXYZ>& target_cloud,
    const std::vector<std::vector<int>>& data_association, double dof)
    : weighted_error_terms_(source_cloud.size()) {
  for (size_t i = 0; i < data_association.size(); i++) {
    for (size_t j = 0; j < data_association[i].size(); j++) {
      std::shared_ptr<WeightedErrorTerm> error_term(new WeightedErrorTerm(
          source_cloud[i], target_cloud[data_association[i][j]]));
      weighted_error_terms_[i].push_back(error_term);
      problem_.AddResidualBlock(error_term->costFunction(),
                                error_term->weight(), rotation_, translation_);
    }
  }
  weight_updater_.reset(new WeightUpdater(&weighted_error_terms_, dof));
  (*weight_updater_)(ceres::IterationSummary());
}

void PointCloudRegistration::Solve(ceres::Solver::Options* options,
                                   ceres::Solver::Summary* summary) {
  options->callbacks.push_back(weight_updater_.get());
  ceres::Solve(*options, &problem_, summary);
}

std::unique_ptr<Eigen::Quaternion<double>> PointCloudRegistration::rotation() {
  std::unique_ptr<Eigen::Quaternion<double>> estimated_rot(
      new Eigen::Quaternion<double>(rotation_[0], rotation_[1], rotation_[2],
                                    rotation_[3]));  // Needed becouse eigen
                                                     // stores quaternion as
                                                     // [x,y,z,w] instead
                                                     // than [w,x,y,z]
  estimated_rot->normalize();
  return estimated_rot;
}

std::unique_ptr<Eigen::Vector3d> PointCloudRegistration::translation() {
  std::unique_ptr<Eigen::Vector3d> estimated_translation(
      new Eigen::Vector3d(translation_));
  return estimated_translation;
}

}  // namespace point_cloud_registration
