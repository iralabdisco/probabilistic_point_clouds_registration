#ifndef MAP_MERGING_DENSE_SPARSE_SIMULATOR_H_
#define MAP_MERGING_DENSE_SPARSE_SIMULATOR_H_

#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>
#include <pcl/common/transforms.h>
#include <Eigen/Geometry>

namespace dense_sparse_simulator {

#define PCL_INSTANTIATE_DenseSparseSimulator(T) \
  template class PCL_EXPORTS dense_sparse_simulator::DenseSparseSimulator<T>;

template <typename PointType>
class DenseSparseSimulator {
 public:
  DenseSparseSimulator(int num_sparse_points, float dense_surface_size,
                       float dense_resolution, float sparse_noise_std,
                       double (*surfaceEquation)(double, double));
  DenseSparseSimulator(int num_sparse_points, float sparse_noise_std,
                       std::string dense_file_name);
  // ~DenseSparseSimulator();
  typename pcl::PointCloud<PointType>::Ptr sparseMap();
  typename pcl::PointCloud<PointType>::Ptr denseMap();
  Eigen::Affine3f& denseToSparseTransform();
  bool state();

 private:
  float dense_surface_size_;
  float dense_resolution_;
  int num_sparse_points_;
  bool state_;
  typename pcl::PointCloud<PointType>::Ptr dense_map_;
  typename pcl::PointCloud<PointType>::Ptr sparse_map_;
  Eigen::Affine3f denseToSparseTransform_;
  void generateSparse(float sparse_noise_std);
  void sparseRandomTransform(float traslation_range);
  double (*surfaceEquation_)(double x, double y);
};

template <typename PointType>
inline typename pcl::PointCloud<PointType>::Ptr
DenseSparseSimulator<PointType>::sparseMap() {
  return sparse_map_;
}

template <typename PointType>
inline typename pcl::PointCloud<PointType>::Ptr
DenseSparseSimulator<PointType>::denseMap() {
  return dense_map_;
}

template <typename PointType>
inline typename Eigen::Affine3f&
DenseSparseSimulator<PointType>::denseToSparseTransform() {
  return denseToSparseTransform_;
}

}  // namespace dense_sparse_simulator

#endif  // MAP_MERGING_DENSE_SPARSE_SIMULATOR_H_
