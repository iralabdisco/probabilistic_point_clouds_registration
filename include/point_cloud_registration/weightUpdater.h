#ifndef POINT_CLOUD_REGISTRATION_WEIGHT_UPDATER_H
#define POINT_CLOUD_REGISTRATION_WEIGHT_UPDATER_H

#include <vector>
#include "ceres/ceres.h"
#include "ceres/loss_function.h"
#include "point_cloud_registration/probabilisticWeights.h"
#include "point_cloud_registration/reprojectionError.h"

using std::vector;

namespace point_cloud_registration {
class WeightedErrorTerm {
 public:
  WeightedErrorTerm(ReprojectionError* cost_function,
                    ceres::LossFunctionWrapper* weight)
      : cost_function_(cost_function), weight_(weight) {}
  inline double squaredError() { return cost_function_->squaredError(); }
  inline void updateWeight(double new_weight) {
    weight_->Reset(
        new ceres::ScaledLoss(NULL, new_weight, ceres::DO_NOT_TAKE_OWNERSHIP),
        ceres::TAKE_OWNERSHIP);
  }

 private:
  ReprojectionError* cost_function_;
  ceres::LossFunctionWrapper* weight_;
};

class WeightUpdater : ceres::IterationCallback {
 public:
  WeightUpdater(const ProbabilisticWeights& weightCalculator,
                vector<vector<WeightedErrorTerm*>> weighted_error_terms);
  ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary);

 private:
  vector<vector<WeightedErrorTerm*>> weighted_error_terms_;
  double* rotation_;
  double* translation_;
  ProbabilisticWeights weightCalculator_;
};
}  // namespace point_cloud_registration

#endif
