#include "point_cloud_registration/reprojectionError.h"

namespace point_cloud_registration {

ReprojectionError::ReprojectionError(const pcl::PointXYZ& source_point,
                                     const pcl::PointXYZ& target_point)
    : source_point_(source_point.x, source_point.y, source_point.z),
      target_point_(target_point.x, target_point.y, target_point.z) {}

template <>
bool ReprojectionError::operator()<double>(const double* const rotation,
                                           const double* const translation,
                                           double* residuals) const {
  const double* point_x = source_point_.data();
  const double* point_y = target_point_.data();

  double transformed_point[kResiduals];
  ceres::QuaternionRotatePoint(rotation, point_x, transformed_point);
  squared_error_ = 0;
  for (int i = 0; i < kResiduals; i++) {
    transformed_point[i] = transformed_point[i] + translation[i];
    residuals[i] = point_y[i] - transformed_point[i];
    squared_error_ += residuals[i] * residuals[i];
  }
  return true;
}

}  // namespace point_cloud_registration
