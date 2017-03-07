#ifndef POINT_CLOUD_REGISTRATION_POINT_CLOUD_REGISTRATION_ITERATION_H
#define POINT_CLOUD_REGISTRATION_POINT_CLOUD_REGISTRATION_ITERATION_H

#include <ceres/ceres.h>
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

#include "point_cloud_registration/error_term.hpp"
#include "point_cloud_registration/probabilistic_weights.hpp"
#include "point_cloud_registration/weight_updater_callback.hpp"
#include "point_cloud_registration/point_cloud_registration_params.hpp"

#define DIMENSIONS 3

namespace point_cloud_registration {

class PointCloudRegistrationIteration
{
public:
    PointCloudRegistrationIteration(const pcl::PointCloud<pcl::PointXYZ> &source_cloud,
                                    const pcl::PointCloud<pcl::PointXYZ> &target_cloud,
                                    const Eigen::SparseMatrix<double, Eigen::RowMajor> &data_association,
                                    PointCloudRegistrationParams parameters)
        : error_terms_(), data_association_(data_association), parameters_(parameters),
          weight_updater_(parameters.dof, DIMENSIONS, parameters.max_neighbours)
    {
        std::copy(std::begin(parameters_.initial_rotation), std::end(parameters_.initial_rotation),
                  std::begin(rotation_));
        std::copy(std::begin(parameters_.initial_translation), std::end(parameters_.initial_translation),
                  std::begin(translation_));
        error_terms_.reserve(source_cloud.size());
        problem_.reset(new ceres::Problem());
        for (size_t i = 0; i < data_association.outerSize(); i++) {
            for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(data_association, i); it;
                    ++it) {
                ErrorTerm *error_term = new ErrorTerm(source_cloud[it.row()], target_cloud[it.col()]);
                error_terms_.push_back(error_term);
                problem_->AddResidualBlock(new ceres::AutoDiffCostFunction
                                           < ErrorTerm, ErrorTerm::kResiduals, 4, 3 > (error_term), error_term->weight(), rotation_,
                                           translation_);
            }
        }
        weight_updater_callback_.reset(new WeightUpdaterCallback(&data_association_, &parameters_,
                                                                 &error_terms_, &weight_updater_, rotation_, translation_));
        (*weight_updater_callback_)(ceres::IterationSummary());
    }

    void solve(ceres::Solver::Options options, ceres::Solver::Summary *summary)
    {
        options.callbacks.push_back(weight_updater_callback_.get());
        options.update_state_every_iteration = true;
        ceres::Solve(options, problem_.get(), summary);
    }

    Eigen::Affine3d transformation()
    {
        Eigen::Affine3d affine = Eigen::Affine3d::Identity();
        Eigen::Quaternion<double> estimated_rot(rotation_[0], rotation_[1], rotation_[2], rotation_[3]);
        estimated_rot.normalize();
        affine.rotate(estimated_rot);
        affine.pretranslate(Eigen::Vector3d(translation_));
        return affine;
    }

private:
    std::vector<ErrorTerm *> error_terms_;
    std::unique_ptr<ceres::Problem> problem_;
    double rotation_[4];
    double translation_[3];
    Eigen::SparseMatrix<double, Eigen::RowMajor> data_association_;
    PointCloudRegistrationParams parameters_;
    ProbabilisticWeights weight_updater_;
    std::unique_ptr<WeightUpdaterCallback> weight_updater_callback_;
};

}  // namespace point_cloud_registration

#endif
