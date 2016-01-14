#include <limits>
#include <vector>
#include <Eigen/Sparse>
#include <Eigen/Core>
#include <gtest/gtest.h>
#include "point_cloud_registration/probabilistic_weights.hpp"

using point_cloud_registration::ProbabilisticWeights;

Eigen::SparseMatrix<double, Eigen::RowMajor> squaredErrors() {
  /**
  Squared Errors Matrix = | 1 0 1 1 |
                          |1 4 9 16 |
  **/
  std::vector<Eigen::Triplet<double>> tripletList;
  tripletList.push_back(Eigen::Triplet<double>(0, 0, 1));
  tripletList.push_back(Eigen::Triplet<double>(0, 2, 1));
  tripletList.push_back(Eigen::Triplet<double>(0, 3, 1));
  tripletList.push_back(Eigen::Triplet<double>(1, 0, 1));
  tripletList.push_back(Eigen::Triplet<double>(1, 1, 4));
  tripletList.push_back(Eigen::Triplet<double>(1, 2, 9));
  tripletList.push_back(Eigen::Triplet<double>(1, 3, 16));
  Eigen::SparseMatrix<double, Eigen::RowMajor> squared_errors(2, 4);
  squared_errors.setFromTriplets(tripletList.begin(), tripletList.end());
  squared_errors.makeCompressed();
  return squared_errors;
}

TEST(UpdateWeightsTestSuite, tCallbackTest) {
  auto squared_errors = squaredErrors();
  ProbabilisticWeights weightUpdater(5, 1, 4);
  auto weights = weightUpdater.updateWeights(squared_errors);
  Eigen::MatrixXd expected_weights(2, 4);
  expected_weights << 1.0 / 3, 0, 1.0 / 3, 1.0 / 3, 0.7151351, 0.1412613,
      0.0241258, 0.0047656;
  for (std::size_t i = 0; i < expected_weights.rows(); i++) {
    for (std::size_t j = 0; j < expected_weights.cols(); j++) {
      EXPECT_NEAR(expected_weights(i, j), weights.coeffRef(i, j), 1e-6);
    }
  }
}

TEST(UpdateWeightsTestSuite, gaussianCallbackTest) {
  auto squared_errors = squaredErrors();
  ProbabilisticWeights weightUpdater(std::numeric_limits<double>::infinity(), 1,
                                     4);
  auto weights = weightUpdater.updateWeights(squared_errors);
  Eigen::MatrixXd expected_weights(2, 4);
  expected_weights << 1.0 / 3, 0, 1.0 / 3, 1.0 / 3, 0.805153702921689,
      0.179654074677018, 0.0147469044726408, 0.000445317928652638;
  for (std::size_t i = 0; i < expected_weights.rows(); i++) {
    for (std::size_t j = 0; j < expected_weights.cols(); j++) {
      EXPECT_NEAR(expected_weights(i, j), weights.coeffRef(i, j), 1e-6);
    }
  }
}
