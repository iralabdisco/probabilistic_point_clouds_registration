#ifndef POINT_CLOUD_REGISTRATION_PSO_REGISTRATION_HPP
#define POINT_CLOUD_REGISTRATION_PSO_REGISTRATION_HPP

#include <thread>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>

#include "point_cloud_registration/point_cloud_registration_iteration.hpp"
#include "point_cloud_registration/point_cloud_registration_params.hpp"
#include "point_cloud_registration/utilities.hpp"

namespace point_cloud_registration {

class PSORegistration
{
public:
    PSORegistration(
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud,
        PointCloudRegistrationParams parameters): parameters_(parameters), source_cloud_(source_cloud),
        target_cloud_(target_cloud), current_iteration_(0), cost_drop_(0), num_unusefull_iter_(0)
    {}

    double align()
    {
        while (!hasConverged()) {
            pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
            kdtree.setInputCloud(target_cloud_);
            std::vector<float> distances;
            Eigen::SparseMatrix<double, Eigen::RowMajor> data_association(source_cloud_->size(),
                                                                          target_cloud_->size());
            std::vector<Eigen::Triplet<double>> tripletList;
            for (std::size_t i = 0; i < source_cloud_->size(); i++) {
                std::vector<int> neighbours;
                kdtree.radiusSearch(*source_cloud_, i, parameters_.radius, neighbours, distances,
                                    parameters_.max_neighbours);
                int k = 0;
                for (int j : neighbours) {
                    tripletList.push_back(Eigen::Triplet<double>(i, j, distances[k]));
                    k++;
                }
            }
            std::vector<Eigen::Triplet<double>> tripletList_filtered;
            if (tripletList.size() > 0) {
                double median = point_cloud_registration::medianDistance(tripletList);
                for (auto t : tripletList) {
                    if (t.value() <= median * 5) {
                        tripletList_filtered.push_back(t);
                    }
                }
            }
            data_association.setFromTriplets(tripletList.begin(), tripletList.end());
            data_association.makeCompressed();

            PointCloudRegistrationIteration registration(*source_cloud_, *target_cloud_, data_association,
                                                         parameters_);
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.use_nonmonotonic_steps = true;
            options.minimizer_progress_to_stdout = false;
            options.max_num_iterations = std::numeric_limits<int>::max();
            options.function_tolerance = 10e-6;
            options.num_threads = std::thread::hardware_concurrency();
            ceres::Solver::Summary summary;
            registration.solve(options, &summary);

            pcl::transformPointCloud (*source_cloud_, *source_cloud_, registration.transformation());
            cost_drop_ = (summary.initial_cost - summary.final_cost) / summary.initial_cost;
            current_iteration_++;
        }
        return point_cloud_registration::medianClosestDistance(source_cloud_, target_cloud_);

    }

    bool hasConverged()
    {
        if (current_iteration_ == parameters_.n_iter) {
            return true;
        }
        if (cost_drop_ < parameters_.cost_drop_thresh) {
            if (num_unusefull_iter_ > parameters_.n_cost_drop_it) {
                return true;
            } else {
                num_unusefull_iter_++;
            }
        } else {
            num_unusefull_iter_ = 0;
        }
        return false;
    }


private:
    PointCloudRegistrationParams parameters_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_;
    double cost_drop_;
    int num_unusefull_iter_;
    int current_iteration_;
};

}  // namespace point_cloud_registration

#endif
