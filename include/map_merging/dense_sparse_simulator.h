/*
  This class creates a dense point cloud, either from an equation-defined
  surface or from a . pcd file and
  generates the corresponding sparse point cloud taking random points from the
  dense one.
  It then applies gaussian noise to these points and a random roto-traslation to
  the sparse map.
*/
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
  /**
  Creates the dense map using a surface defined by a two-variables equation
  implemented by surfaceEquation
  @param num_sparse_points the number of points in the sparse map
  @param dense_surface_size the size of the surface. Both x and y are less then
  dense_surface_size for every point in the dense map
  @param dense_resolution the resolution of the dense map (i.e. x-y the distance
  between points)
  @param sparse_noise_std the standard deviation of the noise applied to the
  points of the sparse map
  @param surfaceEquation a pointer to a function of two variables that defines
  the surface(i.e. z = surfaceEquation(x,y))
  */
  DenseSparseSimulator(int num_sparse_points, float dense_surface_size,
                       float dense_resolution, float sparse_noise_std,
                       double (*surfaceEquation)(double, double));

  /**
  Creates the dense map using a point cloud saved in a pcd file format
  @param num_sparse_points the number of points in the sparse map
  @param sparse_noise_std the standard deviation of the noise applied to the
  points of the sparse map
  @param dense_file_name the filename of the .pcd file
  */
  DenseSparseSimulator(int num_sparse_points, float sparse_noise_std,
                       std::string dense_file_name);

  // Returns a shared pointer to the sparse point cloud
  typename pcl::PointCloud<PointType>::Ptr sparseMap();

  // Returns a shared pointer to the dense point cloud
  typename pcl::PointCloud<PointType>::Ptr denseMap();

  // Returns the transformation that was applied to the sparse point cloud
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
