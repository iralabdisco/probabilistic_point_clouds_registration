#include "point_cloud_registration/weighted_error_term.h"

namespace point_cloud_registration {

WeightedErrorTerm::WeightedErrorTerm(const pcl::PointXYZ& source_point,
                                     const pcl::PointXYZ& target_point)
    : error_term_(new ReprojectionError(source_point, target_point)),
      cost_function_(new ceres::AutoDiffCostFunction<
          ReprojectionError, ReprojectionError::kResiduals, 4, 3>(
          error_term_.get())),
      weight_(new ceres::LossFunctionWrapper(
          new ceres::ScaledLoss(NULL, 1, ceres::DO_NOT_TAKE_OWNERSHIP),
          ceres::TAKE_OWNERSHIP)) {}

}  // namespace point_cloud_registration
