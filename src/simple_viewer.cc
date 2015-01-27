#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include "map_merging/dense_sparse_simulator.h"

// The function used to generate the dense surface
inline double surfaceEquation(double x, double y) { return sin(x) + cos(y); }

int main() {
  dense_sparse_simulator::DenseSparseSimulator<pcl::PointXYZ> simulator(
      1000, 0, "/home/simone/Downloads/kitchen/Rf10.pcd");
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_dense_map(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::copyPointCloud(*(simulator.denseMap()), *colored_dense_map);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_sparse_map(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::copyPointCloud(*(simulator.sparseMap()), *colored_sparse_map);

  for (size_t i = 0; i < colored_dense_map->points.size(); i++) {
    colored_dense_map->points[i].r = 255;
  }
  for (size_t i = 0; i < colored_sparse_map->points.size(); i++) {
    colored_sparse_map->points[i].g = 255;
    colored_sparse_map->points[i].r = 0;
  }

  pcl::visualization::CloudViewer viewer("Cloud Viewer");
  viewer.showCloud(colored_dense_map, "Dense");
  viewer.showCloud(colored_sparse_map, "Sparse");
  while (!viewer.wasStopped()) {
  }
}
