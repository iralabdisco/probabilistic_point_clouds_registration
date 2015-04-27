#ifndef POINT_CLOUD_REGISTRATION_PROBABILISTIC_WEIGHTS_H
#define POINT_CLOUD_REGISTRATION_PROBABILISTIC_WEIGHTS_H

#include <memory>
#include <vector>

namespace point_cloud_registration {

class ProbabilisticWeights {
 public:
  void updateWeights(const std::vector<std::vector<double>>& residuals,
                     std::vector<std::vector<double>>* weights);
  ProbabilisticWeights(double v, int dimension);

 private:
  double t_exponent_;
  double log_factor_;
  double v_;
  double log_norm_constant_;
  int dimension_;
  bool is_normal_;
};
}  // namespace point_cloud_registration

#endif
