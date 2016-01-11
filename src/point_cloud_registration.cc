#include "point_cloud_registration/point_cloud_registration.h"

#include <fstream>
#include <memory>
#include <string>
#include <vector>

using std::size_t;

namespace point_cloud_registration {
const double PointCloudRegistration::default_rotation[4] = {1, 0, 0, 0};
const double PointCloudRegistration::default_translation[3] = {0, 0, 0};

PointCloudRegistration::PointCloudRegistration(
    const pcl::PointCloud<pcl::PointXYZ>& source_cloud,
    const pcl::PointCloud<pcl::PointXYZ>& target_cloud,
    const Eigen::SparseMatrix<int>& data_association, double dof,
    const double rotation_initial_guess[4],
    const double translation_initial_guess[3])
    : error_terms_(source_cloud.size()),
      data_association_(data_association),
      rotation_ {rotation_initial_guess[0], rotation_initial_guess[1],rotation_initial_guess[2], rotation_initial_guess[3]},
translation_ {translation_initial_guess[0], translation_initial_guess[1],translation_initial_guess[2]} {
    problem_.reset(new ceres::Problem());
    for (size_t i = 0; i < data_association.outerSize(); i++) {
        for (auto it(data_association,i; it; ++it) {
        ErrorTerm error_term(source_cloud[i], target_cloud[j]);
            problem_->AddResidualBlock(new ceres::AutoDiffCostFunction<ReprojectionError, ReprojectionError::kResiduals, 4, 3>(error_term_),
            error_term->weight(), rotation_, translation_);
            error_terms_.push_back(error_term);
        }
    }
    this(ceres::IterationSummary());
}

void PointCloudRegistration::solve(ceres::Solver::Options options,
ceres::Solver::Summary* summary) {
    options.callbacks.push_back(weight_updater_.get());
    options.update_state_every_iteration = true;
    ceres::Solve(options, problem_.get(), summary);
}

Eigen::Affine3d PointCloudRegistration::transformation() {
    Eigen::Affine3d affine = Eigen::Affine3d::Identity();
    Eigen::Quaternion<double> estimated_rot(rotation_[0], rotation_[1],
                                            rotation_[2], rotation_[3]);
    estimated_rot.normalize();
    affine.rotate(estimated_rot);
    affine.pretranslate(Eigen::Vector3d(translation_));
    return affine;
}

ceres::CallbackReturnType PointCloudRegistration::operator()(const ceres::IterationSummary& summary) {
    Eigen::Sparse<double> residuals(data_association_.rows(), data_association_.cols());
    residuals.reserve(VectorXi::Constant(data_association_.cols(),max_neighbours_));
    int k = 0;
    for (int i=0; i<data_association_.outerSize(); ++i) {
        for (SparseMatrix<int>::InnerIterator it(data_association_,i); it; ++it)
        {
            double residual[3];
            (error_terms_[k])(rotation_, translation_,residual);
            residuals.insert(it.row(), it.col()) = residual;
            k++;
        }
    }
    Eigen::Sparse<double> weights_ = weightCalculator_.updateWeights(residuals);
    k = 0;
    for (int i=0; i<weights_.outerSize(); ++i) {
        for (SparseMatrix<int>::InnerIterator it(weights_,i); it; ++it)
        {
            error_terms_[k].updateWeight(it.value());
            k++;
        }
    }
    return ceres::SOLVER_CONTINUE;
}
}  // namespace point_cloud_registration
