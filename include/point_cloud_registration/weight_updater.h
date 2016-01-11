#ifndef POINT_CLOUD_REGISTRATION_WEIGHT_UPDATER_H
#define POINT_CLOUD_REGISTRATION_WEIGHT_UPDATER_H

#include <ceres/ceres.h>
#include <ceres/loss_function.h>
#include <Eigen/Core>
#include <Eigen/Sparse>

#include <vector>

#include "point_cloud_registration/probabilistic_weights.h"
#include "point_cloud_registration/weighted_error_term.h"

namespace point_cloud_registration {
typedef std::vector<std::unique_ptr<WeightedErrorTerm>> WeightedErrorTermGroup;

class WeightUpdater : public ceres::IterationCallback {
 public:
  explicit WeightUpdater(int num_groups, double dof,
      double* rotation, double* translation, std::vector<std::vector<int>> data_association, int dense_size_);
  ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary);
  void addErrorTerm(int index,
                            std::unique_ptr<WeightedErrorTerm> error_term);
  std::vector<WeightedErrorTermGroup> weighted_error_terms_;

 private:
  ProbabilisticWeights weightCalculator_;
  double* rotation_;
  double* translation_;
  int max_neighbours_;
  Eigen::SparseMatrix<int> data_association_;
};
}  // namespace point_cloud_registration

#endif
