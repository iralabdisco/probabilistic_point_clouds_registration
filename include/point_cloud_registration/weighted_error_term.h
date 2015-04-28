#ifndef POINT_CLOUD_REGISTRATION_WEIGHTED_ERROR_TERM_H
#define POINT_CLOUD_REGISTRATION_WEIGHTED_ERROR_TERM_H

#include <ceres/ceres.h>
#include <ceres/loss_function.h>
#include <pcl/point_types.h>

#include "point_cloud_registration/reprojection_error.h"

namespace point_cloud_registration {
class WeightedErrorTerm {
 public:
  WeightedErrorTerm(const pcl::PointXYZ& source_point,
                    const pcl::PointXYZ& target_point);
  inline double squaredError() { return error_term_->squaredError(); }
  inline void updateWeight(double new_weight) {
    weight_->Reset(
        new ceres::ScaledLoss(NULL, new_weight, ceres::TAKE_OWNERSHIP),
        ceres::TAKE_OWNERSHIP);
  }
  inline ceres::CostFunction* costFunction() { return cost_function_; }
  inline ceres::LossFunctionWrapper* weight() { return weight_; }

 private:
  ReprojectionError* error_term_;
  ceres::AutoDiffCostFunction<ReprojectionError, ReprojectionError::kResiduals,
                              4, 3>* cost_function_;
  ceres::LossFunctionWrapper* weight_;
};
}  // namespace point_cloud_registration
#endif
