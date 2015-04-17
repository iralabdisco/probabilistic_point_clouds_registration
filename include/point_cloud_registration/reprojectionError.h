#ifndef POINT_CLOUD_REGISTRATION_REPROJECTION_ERROR_H
#define POINT_CLOUD_REGISTRATION_REPROJECTION_ERROR_H

#include <Eigen/Dense>

struct ReprojectionError {
  ReprojectionError(const Eigen::Vector3d& x, const Eigen::Vector3d& y);

  // Factory to hide the construction of the CostFunction object from the client
  // code.
  static ceres::CostFunction* Create(const Eigen::Vector3d& x,
                                     const Eigen::Vector3d& y);

  template <typename T>
  bool operator()(const T* const rotation, const T* const translation,
                  T* residuals) const;

  inline double squaredError() { return squared_error; }

  static const int kResiduals = 3;
  Eigen::Vector3d x_;
  Eigen::Vector3d y_;
  mutable double squared_error;
};

#endif
