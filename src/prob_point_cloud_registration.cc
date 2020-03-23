#include <cstdio>
#include <fstream>
#include <thread>

#include <boost/make_shared.hpp>
#include <pcl/common/angles.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "prob_point_cloud_registration/prob_point_cloud_registration.h"
#include "prob_point_cloud_registration/utilities.hpp"

namespace prob_point_cloud_registration {

ProbPointCloudRegistration::ProbPointCloudRegistration(
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud,
    ProbPointCloudRegistrationParams parameters): parameters_(parameters),
    target_cloud_(target_cloud), mse_ground_truth_(0), current_iteration_(0), mse_prev_it_(0),
    cost_drop_(0), num_unusefull_iter_(0), output_stream_(parameters.verbose)
{
    source_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*source_cloud);
    filtered_source_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if (parameters_.source_filter_size > 0) {
        output_stream_ << "Filtering source point cloud with leaf of size " <<
                       parameters_.source_filter_size << "\n";
        filter_.setInputCloud(source_cloud_);
        filter_.setLeafSize(parameters_.source_filter_size, parameters_.source_filter_size,
                            parameters_.source_filter_size);
        filter_.filter(*filtered_source_cloud_);
    } else {
        *filtered_source_cloud_ = *source_cloud_;
    }
    if (parameters_.target_filter_size > 0) {
        output_stream_ << "Filtering target point cloud with leaf of size " <<
                       parameters_.target_filter_size << "\n";
        filter_.setInputCloud(target_cloud_);
        filter_.setLeafSize(parameters_.target_filter_size, parameters_.target_filter_size,
                            parameters_.target_filter_size);
        filter_.filter(*target_cloud_);
    }
    if (parameters_.summary) {
        prev_source_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*source_cloud);
        report_ <<
                "iter, n_success_steps, initial_cost, final_cost, tx, ty, tz, roll, pitch, yaw, mse_prev_iter, mse_gtruth"
                << std::endl;
    }
    ground_truth_ = false;
}
ProbPointCloudRegistration::ProbPointCloudRegistration(
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud,
    ProbPointCloudRegistrationParams parameters,
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_truth_cloud):
    ProbPointCloudRegistration::ProbPointCloudRegistration(source_cloud, target_cloud, parameters)
{
    ground_truth_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*ground_truth_cloud);
    ground_truth_ = true;
    mse_ground_truth_ = prob_point_cloud_registration::calculateMSE(source_cloud_, ground_truth_cloud_);
    output_stream_ << "Initial MSE w.r.t. ground truth: " << mse_ground_truth_ << "\n";
}

void ProbPointCloudRegistration::align()
{
    while (!hasConverged()) {
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(target_cloud_);
        std::vector<float> distances;
        Eigen::SparseMatrix<double, Eigen::RowMajor> data_association(filtered_source_cloud_->size(),
                                                                      target_cloud_->size());
        std::vector<Eigen::Triplet<double>> tripletList;
        for (std::size_t i = 0; i < filtered_source_cloud_->size(); i++) {
            std::vector<int> neighbours;
            kdtree.radiusSearch(*filtered_source_cloud_, i, parameters_.radius, neighbours, distances,
                                parameters_.max_neighbours);
            int k = 0;
            for (int j : neighbours) {
                tripletList.push_back(Eigen::Triplet<double>(i, j, distances[k]));
                k++;
            }
        }
        data_association.setFromTriplets(tripletList.begin(), tripletList.end());
        data_association.makeCompressed();

        ProbPointCloudRegistrationIteration registration(*filtered_source_cloud_, *target_cloud_,
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
        options.function_tolerance = 10e-6;
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
        output_stream_ << summary.FullReport() << "\n";

        pcl::transformPointCloud (*source_cloud_, *source_cloud_, registration.transformation());
        pcl::transformPointCloud (*filtered_source_cloud_, *filtered_source_cloud_,
                                  registration.transformation());

        if (ground_truth_) {
            mse_ground_truth_ = prob_point_cloud_registration::calculateMSE(source_cloud_, ground_truth_cloud_);
            output_stream_ << "MSE w.r.t. ground truth: " << mse_ground_truth_ << "\n";
        }

        cost_drop_ = (summary.initial_cost - summary.final_cost) / summary.initial_cost;
        if (parameters_.summary) {
            mse_prev_it_ = prob_point_cloud_registration::calculateMSE(source_cloud_, prev_source_cloud_);
            *prev_source_cloud_ = *source_cloud_;
            auto rpy = current_trans.rotation().eulerAngles(0, 1, 2);
            report_ << current_iteration_ << ", " << summary.num_successful_steps << ", " <<
                    summary.initial_cost << ", " << summary.final_cost << ", " << current_trans.translation().x() <<
                    ", " << current_trans.translation().y() << ", " << current_trans.translation().z() << ", " <<
                    pcl::rad2deg(rpy(0, 0)) << ", " << pcl::rad2deg(rpy(1, 0)) << ", " << pcl::rad2deg(rpy(2,
                                                                                                           0)) << ", " << mse_prev_it_ << ", " << mse_ground_truth_ << std::endl;
        }
        current_iteration_++;
    }
    if (ground_truth_) {
        mse_ground_truth_ = prob_point_cloud_registration::calculateMSE(source_cloud_, ground_truth_cloud_);
        std::cout << "MSE w.r.t. ground truth: " << mse_ground_truth_ << std::endl;
    }
}

bool ProbPointCloudRegistration::hasConverged()
{
    if (current_iteration_ == parameters_.n_iter) {
        output_stream_ << "Terminating because maximum number of iterations has been reached ( " <<
                       current_iteration_ << " iter)\n";
        return true;
    }
    if (cost_drop_ < parameters_.cost_drop_thresh) {
        if (num_unusefull_iter_ > parameters_.n_cost_drop_it) {
            output_stream_ << "Terminating because cost drop has been under " << parameters_.cost_drop_thresh *
                           100 <<
                           " % for more than " << parameters_.n_cost_drop_it << " iterations\n";
            return true;
        } else {
            num_unusefull_iter_++;
        }
    } else {
        num_unusefull_iter_ = 0;
    }
    return false;
}

}  // namespace prob_point_cloud_registration
