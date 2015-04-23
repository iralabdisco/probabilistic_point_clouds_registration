#ifndef POINT_CLOUD_REGISTRATION_REPROJECTION_ERROR_H
#define POINT_CLOUD_REGISTRATION_REPROJECTION_ERROR_H
#include <Eigen/Core>
#include <pcl/point_types.h>
#include "ceres/ceres.h"
#include "ceres/rotation.h"

namespace point_cloud_registration {
struct ReprojectionError {
  ReprojectionError(const pcl::PointXYZ& source_point,
                    const pcl::PointXYZ& target_point);

  template <typename T>
  bool operator()(const T* const rotation, const T* const translation,
                  T* residuals) const;

  inline double squaredError() { return squared_error_; }

  static const int kResiduals = 3;
  Eigen::Vector3d source_point_;
  Eigen::Vector3d target_point_;
  mutable double squared_error_;
};

template <typename T>
bool ReprojectionError::operator()(const T* const rotation,
                                   const T* const translation,
                                   T* residuals) const {
  T point_x[kResiduals];
  T point_y[kResiduals];
  for (int i = 0; i < kResiduals; i++) {
    point_x[i] = T(source_point_[i]);
    point_y[i] = T(target_point_[i]);
  }
  T transformed_point[kResiduals];
  ceres::QuaternionRotatePoint(rotation, point_x, transformed_point);
  for (int i = 0; i < kResiduals; i++) {
    transformed_point[i] = transformed_point[i] + translation[i];
    residuals[i] = point_y[i] - transformed_point[i];
  }
  return true;
}

}  // namespace point_cloud_registration
#endif
