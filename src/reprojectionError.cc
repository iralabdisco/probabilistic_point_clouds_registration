#include <Eigen/Dense>

#include "ceres/ceres.h"
#include "ceres/rotation.h"

#include "point_cloud_registration/reprojectionError.h"

ReprojectionError::ReprojectionError(const Eigen::Vector3d& x,
                                     const Eigen::Vector3d& y)
    : x_(x), y_(y) {}

// Factory to hide the construction of the CostFunction object from the client
// code.
ceres::CostFunction* ReprojectionError::Create(const Eigen::Vector3d& x,
                                               const Eigen::Vector3d& y) {
  return (new ceres::AutoDiffCostFunction<ReprojectionError,
                                          ReprojectionError::kResiduals, 4, 3>(
      new ReprojectionError(x, y)));
}

template <typename T>
bool ReprojectionError::operator()(const T* const rotation,
                                   const T* const translation,
                                   T* residuals) const {
  T point_x[kResiduals];
  T point_y[kResiduals];
  for (int i = 0; i < kResiduals; i++) {
    point_x[i] = T(x_[i]);
    point_y[i] = T(y_[i]);
  }
  T transformed_point[kResiduals];
  ceres::QuaternionRotatePoint(rotation, point_x, transformed_point);
  for (int i = 0; i < kResiduals; i++) {
    transformed_point[i] = transformed_point[i] + translation[i];
    residuals[i] = point_y[i] - transformed_point[i];
  }
  return true;
}
