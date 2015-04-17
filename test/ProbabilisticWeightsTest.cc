#include <limits>
#include <vector>

#include <sm/eigen/gtest.hpp>

#include "point_cloud_registration/probabilisticWeights.h"

using point_cloud_registration::ProbabilisticWeights;

std::shared_ptr<std::vector<std::vector<double>>> fillResiduals() {
  std::vector<double> error_group_1(3, 1);
  std::vector<double> error_group_2 = {1, 4, 9, };
  auto residuals = std::make_shared<std::vector<std::vector<double>>>();
  residuals->push_back(error_group_1);
  residuals->push_back(error_group_2);
  return residuals;
}

TEST(UpdateWeightsTestSuite, tCallbackTest) {
  auto residuals = fillResiduals();
  ProbabilisticWeights weightUpdater(5, 1);
  auto weights = std::make_shared<std::vector<std::vector<double>>>(2, std::vector<double>());
  weights->at(0) = std::vector<double>(3);
  weights->at(1) = std::vector<double>(4);
  weightUpdater.updateWeights(*residuals, weights);
  std::vector<double> group1_expected_w = {1.0 / 3, 1.0 / 3, 1.0 / 3};
  std::vector<double> group2_expected_w = {0.7151351, 0.1412613,
                                           0.0241258, 0.0047656};
  std::vector<std::vector<double>> expected_weights;
  expected_weights.push_back(group1_expected_w);
  expected_weights.push_back(group2_expected_w);
  for (std::size_t i = 0; i < expected_weights.size(); i++) {
    std::vector<double>* expected_weight_group = &expected_weights[i];
    for (std::size_t j = 0; j < expected_weight_group->size(); j++) {
      EXPECT_NEAR(expected_weight_group->at(j), weights->at(i)[j], 1e-6);
    }
  }
}

TEST(UpdateWeightsTestSuite, gaussianCallbackTest) {
  auto residuals = fillResiduals();
  ProbabilisticWeights weightUpdater(std::numeric_limits<double>::infinity(),
                                     1);
  auto weights = std::make_shared<std::vector<std::vector<double>>>(
      2, std::vector<double>());
  weights->at(0) = std::vector<double>(3);
  weights->at(1) = std::vector<double>(4);
  weightUpdater.updateWeights(*residuals, weights);
  std::vector<double> group1_expected_w = {1.0 / 3, 1.0 / 3, 1.0 / 3};
  std::vector<double> group2_expected_w = {
      0.805153702921689,  0.179654074677018,
      0.0147469044726408, 0.000445317928652638};
  std::vector<std::vector<double>> expected_weights;
  expected_weights.push_back(group1_expected_w);
  expected_weights.push_back(group2_expected_w);
  for (std::size_t i = 0; i < expected_weights.size(); i++) {
    std::vector<double>* expected_weight_group = &expected_weights[i];
    for (std::size_t j = 0; j < expected_weight_group->size(); j++) {
      EXPECT_NEAR(expected_weight_group->at(j), weights->at(i)[j], 1e-6);
    }
  }
}
