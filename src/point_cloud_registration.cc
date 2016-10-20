#include <thread>

#include <pcl/kdtree/kdtree_flann.h>

#include "point_cloud_registration/point_cloud_registration.h"

namespace point_cloud_registration {

PointCloudRegistration::PointCloudRegistration(
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud,
    PointCloudRegistrationParams parameters): parameters_(parameters), source_cloud_(source_cloud),
    target_cloud_(target_cloud), transformation_(Eigen::Affine3d::Identity())
{
}

void PointCloudRegistration::align(bool verbose)
{
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
    if (verbose) {
        options.minimizer_progress_to_stdout = true;
    } else {
        options.minimizer_progress_to_stdout = false;
    }
    options.max_num_iterations = std::numeric_limits<int>::max();
    options.function_tolerance = 10e-16;
    options.num_threads = std::thread::hardware_concurrency();
    ceres::Solver::Summary summary;
    registration.solve(options, &summary);
    transformation_ = registration.transformation();
    if (verbose) {
        std::cout << summary.FullReport() << std::endl;
    }
}




}  // namespace point_cloud_registration
