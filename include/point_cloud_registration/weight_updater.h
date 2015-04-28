#ifndef POINT_CLOUD_REGISTRATION_WEIGHT_UPDATER_H
#define POINT_CLOUD_REGISTRATION_WEIGHT_UPDATER_H

#include <ceres/ceres.h>
#include <ceres/loss_function.h>

#include <vector>

#include "point_cloud_registration/probabilistic_weights.h"
#include "point_cloud_registration/weighted_error_term.h"

namespace point_cloud_registration {
typedef std::vector<std::shared_ptr<WeightedErrorTerm>> WeightedErrorTermGroup;

class WeightUpdater : public ceres::IterationCallback {
 public:
  explicit WeightUpdater(
      std::vector<WeightedErrorTermGroup>* weighted_error_terms, double dof);
  ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary);

 private:
  std::vector<WeightedErrorTermGroup>* weighted_error_terms_;
  ProbabilisticWeights weightCalculator_;
};
}  // namespace point_cloud_registration

#endif
