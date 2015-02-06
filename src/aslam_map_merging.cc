#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/Optimizer.hpp>
#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/RotationQuaternion.hpp>
#include <sm/random.hpp>
#include <vector>
#include <algorithm>
#include <boost/foreach.hpp>

int main(int argc, char** argv) {
  // TODO(fontana.simone): Load the point clouds and creates the associations
  aslam::backend::OptimizationProblem problem;

  // Creates the design variables: rotation and traslation
  Eigen::Vector3d traslation_initial_guess;
  boost::shared_ptr<aslam::backend::EuclideanPoint> traslation(
      new aslam::backend::EuclideanPoint(traslation_initial_guess));
  traslation->setActive(true);
  problem.addDesignVariable(traslation);

  Eigen::Vector4d rotation_initial_guess;
  boost::shared_ptr<aslam::backend::RotationQuaternion> rotation(
      new aslam::backend::RotationQuaternion(rotation_initial_guess));
  rotation->setActive(true);
  problem.addDesignVariable(rotation);
  return 0;
}
