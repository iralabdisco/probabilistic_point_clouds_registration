#include <math.h>

#include "map_merging/dense_sparse_simulator.h"

namespace dense_sparse_simulator {

template <typename PointType>
DenseSparseSimulator<PointType>::DenseSparseSimulator(
    int num_sparse_points, float dense_surface_size, float dense_resolution,
    float sparse_noise_std, double (*surfaceEquation)(double, double)) {
  assert(dense_surface_size > 0);
  assert(num_sparse_points <
         pow(static_cast<int>(dense_surface_size / dense_resolution), 2));
  assert((dense_resolution > 0) && (dense_resolution <= dense_surface_size));

  num_sparse_points_ = num_sparse_points;
  dense_surface_size_ = dense_surface_size;
  dense_resolution_ = dense_resolution;
  surfaceEquation_ = surfaceEquation;
  // Generates dense map
  dense_map_.reset(new pcl::PointCloud<PointType>);
  dense_map_->width = static_cast<int>(dense_surface_size_ / dense_resolution_);
  dense_map_->height =
      static_cast<int>(dense_surface_size_ / dense_resolution_);
  dense_map_->is_dense = true;
  dense_map_->points.resize(dense_map_->width * dense_map_->height);

  int64_t num_points = 0;
  double x, y = 0;
  for (uint i = 0; i < dense_map_->width; i++) {
    for (uint j = 0; j < dense_map_->height; j++) {
      x = i * dense_resolution_;
      y = j * dense_resolution_;
      dense_map_->at(i, j).x = x;
      dense_map_->at(i, j).y = y;
      dense_map_->at(i, j).z = surfaceEquation_(x, y);
      ++num_points;
    }
  }

  generateSparse(sparse_noise_std);
  sparseRandomTransform(dense_surface_size / 10);
  state_ = 1;
}

template <typename PointType>
DenseSparseSimulator<PointType>::DenseSparseSimulator(
    int num_sparse_points, float sparse_noise_std,
    std::string dense_file_name) {
  dense_map_.reset(new pcl::PointCloud<PointType>);
  num_sparse_points_ = num_sparse_points;
  if (pcl::io::loadPCDFile<PointType>(dense_file_name, *dense_map_) == -1) {
    state_ = 0;
  } else {
    generateSparse(sparse_noise_std);
    sparseRandomTransform(1);
    state_ = 1;
  }
}

template <typename PointType>
void DenseSparseSimulator<PointType>::generateSparse(float sparse_noise_std) {
  // Generates sparse map from dense map
  sparse_map_.reset(new pcl::PointCloud<PointType>);
  sparse_map_->width = num_sparse_points_;
  sparse_map_->height = 1;
  sparse_map_->is_dense = false;
  sparse_map_->points.resize(num_sparse_points_);

  std::random_device rd;
  std::default_random_engine generator(rd());
  std::normal_distribution<float> gaussian_noise(0, sparse_noise_std);
  std::uniform_int_distribution<int64_t> distribution(0,
                                                      dense_map_->size() - 1);
  for (int i = 0; i < num_sparse_points_; i++) {
    int64_t p = distribution(generator);
    sparse_map_->points[i].x = dense_map_->points[p].x + gaussian_noise(rd);
    sparse_map_->points[i].y = dense_map_->points[p].y + gaussian_noise(rd);
    sparse_map_->points[i].z = dense_map_->points[p].z + gaussian_noise(rd);
  }
}

template <typename PointType>
void DenseSparseSimulator<PointType>::sparseRandomTransform(
    float traslation_range) {
  pcl::PointCloud<PointType> tmp_cloud = *sparse_map_;
  std::random_device rd;
  std::default_random_engine generator(rd());

  // Applies a random transformation to the sparse map
  denseToSparseTransform_ = Eigen::Affine3f::Identity();
  std::uniform_real_distribution<float> real_distribution(-traslation_range,
                                                          traslation_range);
  denseToSparseTransform_.translation() << real_distribution(generator),
      real_distribution(generator), real_distribution(generator);
  Eigen::Quaternion<float> rotation(
      real_distribution(generator), real_distribution(generator),
      real_distribution(generator), real_distribution(generator));
  denseToSparseTransform_.rotate(rotation.normalized());
  pcl::transformPointCloud(tmp_cloud, *sparse_map_, denseToSparseTransform_);
}

PCL_INSTANTIATE(DenseSparseSimulator, PCL_XYZ_POINT_TYPES);

// template class DenseSparseSimulator<pcl::PointXYZ>;
// template class DenseSparseSimulator<pcl::PointXYZRGB>;
// template class DenseSparseSimulator<pcl::PointXYZRGA>;
// template class DenseSparseSimulator<pcl::PointXYZI>;
}  // namespace dense_sparse_simulator
