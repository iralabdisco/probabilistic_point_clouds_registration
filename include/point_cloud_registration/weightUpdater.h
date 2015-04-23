#ifndef POINT_CLOUD_REGISTRATION_WEIGHT_UPDATER_H
#define POINT_CLOUD_REGISTRATION_WEIGHT_UPDATER_H

#include <vector>
#include "ceres/ceres.h"
#include "ceres/loss_function.h"
#include "point_cloud_registration/probabilisticWeights.h"
#include "point_cloud_registration/pointCloudRegistration.h"
#include "point_cloud_registration/reprojectionError.h"

namespace point_cloud_registration {

class WeightUpdater : ceres::IterationCallback {
 public:
  explicit WeightUpdater(
      std::vector<WeightedErrorTermGroup>* weighted_error_terms);
  ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary);

 private:
  std::vector<WeightedErrorTermGroup>* weighted_error_terms_;
  ProbabilisticWeights weightCalculator_;
};
}  // namespace point_cloud_registration

#endif
