#ifndef POINT_CLOUD_REGISTRATION_POINT_CLOUD_REGISTRATION_H
#define POINT_CLOUD_REGISTRATION_POINT_CLOUD_REGISTRATION_H

#include <ceres/ceres.h>
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

#include "point_cloud_registration/error_term.hpp"
#include "point_cloud_registration/probabilistic_weights.hpp"

namespace point_cloud_registration {

struct PointCloudRegistrationParams {
  int max_neighbours;
  double dof;
  int dimension = 3;
  double initial_rotation[4] = {1, 0, 0, 0};
  double initial_translation[3] = {0, 0, 0};
};

class PointCloudRegistration : public ceres::IterationCallback {
 public:
  PointCloudRegistration(
      const pcl::PointCloud<pcl::PointXYZ> &source_cloud,
      const pcl::PointCloud<pcl::PointXYZ> &target_cloud,
      const Eigen::SparseMatrix<int, Eigen::RowMajor> &data_association,
      PointCloudRegistrationParams parameters);

  void solve(ceres::Solver::Options options, ceres::Solver::Summary *Summary);
  ceres::CallbackReturnType operator()(const ceres::IterationSummary &summary);
  ceres::CallbackReturnType callback(const ceres::IterationSummary &summary);
  Eigen::Affine3d transformation();

 private:
  std::vector<ErrorTerm> error_terms_;
  std::unique_ptr<ceres::Problem> problem_;
  double rotation_[4];
  double translation_[3];
  Eigen::SparseMatrix<int, Eigen::RowMajor> data_association_;
  ProbabilisticWeights weight_updater_;
  PointCloudRegistrationParams parameters_;
};

}  // namespace point_cloud_registration

#endif
