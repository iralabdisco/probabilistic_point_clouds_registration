#include <thread>

#include <boost/make_shared.hpp>
#include <pcl/common/distances.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "point_cloud_registration/point_cloud_registration.h"

namespace point_cloud_registration {

using pcl::euclideanDistance;

PointCloudRegistration::PointCloudRegistration(
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud,
    PointCloudRegistrationParams parameters): parameters_(parameters),
    target_cloud_(target_cloud), current_iteration_(0)
{
    source_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*source_cloud);
    prev_source_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
}

void PointCloudRegistration::align()
{
    while (!hasConverged()) {
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(target_cloud_);
        std::vector<float> distances;
        Eigen::SparseMatrix<int, Eigen::RowMajor> data_association(source_cloud_->size(),
                                                                   target_cloud_->size());
        std::vector<Eigen::Triplet<int>> tripletList;
        for (std::size_t i = 0; i < source_cloud_->size(); i++) {
            std::vector<int> neighbours;
            kdtree.radiusSearch(*source_cloud_, i, parameters_.radius, neighbours, distances,
                                parameters_.max_neighbours);
            for (int j : neighbours) {
                tripletList.push_back(Eigen::Triplet<int>(i, j, 1));
            }
        }
        data_association.setFromTriplets(tripletList.begin(), tripletList.end());
        data_association.makeCompressed();

        PointCloudRegistrationIteration registration(*source_cloud_, *target_cloud_,
                                                     data_association,
                                                     parameters_);
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.use_nonmonotonic_steps = true;
        if (parameters_.verbose) {
            options.minimizer_progress_to_stdout = true;
        } else {
            options.minimizer_progress_to_stdout = false;
        }
        options.max_num_iterations = std::numeric_limits<int>::max();
        options.function_tolerance = 10e-16;
        options.num_threads = std::thread::hardware_concurrency();
        ceres::Solver::Summary summary;
        registration.solve(options, &summary);
        Eigen::Affine3d current_trans;
        if (current_iteration_ > 0) {
            current_trans = registration.transformation() * transformation_history_.back();
        } else {
            current_trans = registration.transformation();
        }
        transformation_history_.push_back(current_trans);
        if (parameters_.verbose) {
            std::cout << summary.FullReport() << std::endl;
        }
        pcl::transformPointCloud (*source_cloud_, *source_cloud_, registration.transformation());
        current_iteration_++;
    }

}

bool PointCloudRegistration::hasConverged()
{
    if (current_iteration_ == parameters_.n_iter) {
        if (parameters_.verbose) {
            std::cout << "Terminating because maximum number of iterations has been reached (" <<
                      current_iteration_ << " iter)" << std::endl;
        }
        return true;
    }
    if (current_iteration_ > 1) {
        double mse = 0;
        for (int i = 0; i < source_cloud_->size(); i++) {
            mse += euclideanDistance(source_cloud_->at(i), prev_source_cloud_->at(i));
        }
        mse /= source_cloud_->size();
        if (mse <= parameters_.dist_treshold) {
            std::cout << "Terminating because mse is below the treshould (mse = " << mse << "; threshold = " <<
                      parameters_.dist_treshold << ")" << std::endl;
            return true;
        }
    }
    *prev_source_cloud_ = *source_cloud_;

    return false;
}




}  // namespace point_cloud_registration
