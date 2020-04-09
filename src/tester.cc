#include <stdlib.h>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <tclap/CmdLine.h>
#include <Eigen/Core>
#include <Eigen/Sparse>
#include "point_cloud_registration_benchmark/metric.hpp"
#include "prob_point_cloud_registration/prob_point_cloud_registration.h"
#include "prob_point_cloud_registration/utilities.hpp"
#include "prob_point_cloud_registration/rapidcsv.h"

typedef pcl::PointXYZ PointType;

using prob_point_cloud_registration::ProbPointCloudRegistration;
using prob_point_cloud_registration::ProbPointCloudRegistrationParams;

int main(int argc, char **argv) {
  bool use_gaussian = false, ground_truth = false;
  std::string problem_file;
  std::string cloud_folder;
  Eigen::Matrix4d initial_transformation = Eigen::Matrix4d::Identity();
  ProbPointCloudRegistrationParams params;
  try {
    TCLAP::CmdLine cmd("Probabilistic point cloud registration", ' ', "1.0");
    TCLAP::UnlabeledValueArg<std::string> problem_file_arg("problem_file", " ",true, ".", "string", cmd);
    TCLAP::UnlabeledValueArg<std::string> cloud_folder_arg("cloud_folder"," ",true, ".", "string", cmd);

    TCLAP::ValueArg<float> source_filter_arg(
        "s", "source_filter_size", "The leaf size of the voxel filter of the source cloud", false, 0, "float", cmd);
    TCLAP::ValueArg<float> target_filter_arg(
        "t", "target_filter_size", "The leaf size of the voxel filter of the target cloud", false, 0, "float", cmd);
    TCLAP::ValueArg<int> max_neighbours_arg("m", "max_neighbours", "The max cardinality of the neighbours' set", false,
                                            20, "int", cmd);
    TCLAP::ValueArg<int> num_iter_arg("i", "num_iter", "The maximum number of iterations to perform", false, 1000,
                                      "int", cmd);
    TCLAP::ValueArg<float> dof_arg("d", "dof", "The Degree of freedom of t-distribution", false, 5, "float", cmd);
    TCLAP::ValueArg<float> radius_arg("r", "radius", "The radius of the neighborhood search", false, 3, "float", cmd);
    TCLAP::ValueArg<float> cost_drop_tresh_arg(
        "c", "cost_drop_treshold",
        "If the cost_drop drops below this threshold for too many iterations, the algorithm terminate", false, 0.01,
        "float", cmd);
    TCLAP::ValueArg<int> num_drop_iter_arg(
        "n", "num_drop_iter",
        "The maximum number of iterations during which the cost drop is allowed to be under cost_drop_thresh", false, 5,
        "int", cmd);
    TCLAP::SwitchArg use_gaussian_arg("u", "use_gaussian", "Whether to use a gaussian instead the a t-distribution",
                                      cmd, false);
    cmd.parse(argc, argv);

    params.max_neighbours = max_neighbours_arg.getValue();
    use_gaussian = use_gaussian_arg.getValue();
    params.dof = dof_arg.getValue();
    params.radius = radius_arg.getValue();
    params.n_iter = num_iter_arg.getValue();
    params.cost_drop_thresh = cost_drop_tresh_arg.getValue();
    params.n_cost_drop_it = num_drop_iter_arg.getValue();
    problem_file = problem_file_arg.getValue();
    cloud_folder = cloud_folder_arg.getValue();
    params.source_filter_size = source_filter_arg.getValue();
    params.target_filter_size = target_filter_arg.getValue();

  } catch (TCLAP::ArgException &e) {
    std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
    exit(EXIT_FAILURE);
  }

  if (use_gaussian) {

    
    params.dof = std::numeric_limits<double>::infinity();
  } 
  
    rapidcsv::Document problems(problem_file, rapidcsv::LabelParams(),
                             rapidcsv::SeparatorParams(' '));

      std::vector<std::string> sources = problems.GetColumn<std::string>("source");
      std::vector<std::string> targets = problems.GetColumn<std::string>("target");
      std::vector<double> t1 = problems.GetColumn<double>("t1");
       std::vector<double> t2 = problems.GetColumn<double>("t2");
      std::vector<double> t3 = problems.GetColumn<double>("t3");
      std::vector<double> t4 = problems.GetColumn<double>("t4");
      std::vector<double> t5 = problems.GetColumn<double>("t5");
      std::vector<double> t6 = problems.GetColumn<double>("t6");
      std::vector<double> t7 = problems.GetColumn<double>("t7");
    std::vector<double> t8 = problems.GetColumn<double>("t8");
      std::vector<double> t9 = problems.GetColumn<double>("t9");
      std::vector<double> t10 = problems.GetColumn<double>("t10");
        std::vector<double> t11 = problems.GetColumn<double>("t11");
      std::vector<double> t12 = problems.GetColumn<double>("t12");

pcl::PointCloud<PointType>::Ptr source_ground_truth = boost::make_shared<pcl::PointCloud<PointType>>();
pcl::PointCloud<PointType>::Ptr source_cloud(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr target_cloud = boost::make_shared<pcl::PointCloud<PointType>>();
std::unique_ptr<ProbPointCloudRegistration> registration;
pcl::PointCloud<PointType>::Ptr aligned_source = boost::make_shared<pcl::PointCloud<PointType>>();

for(std::size_t i = 0; i< sources.size();i++){
    std::cout<<sources[i]<<" "<<targets[i]<<std::endl;
    initial_transformation << t1[i], t2[i], t3[i], t4[i],t5[i],t6[i],t7[i],t8[i],t9[i],t10[i],t11[i],t12[i],0,0,0,1;
    if(i==0 || sources[i] != sources[i-1]){
        if (pcl::io::loadPCDFile<PointType>(sources[i], *source_ground_truth) == -1) {
            std::cout << "Could not load source cloud, closing" << std::endl;
            exit(EXIT_FAILURE);
        }
    }
    pcl::transformPointCloud(*source_ground_truth, *source_cloud, initial_transformation);
    double initial_error = point_cloud_registration_benchmark::calculate_error(source_ground_truth, source_cloud);
    
    if(i==0 || targets[i]!=targets[i-1]){
        if (pcl::io::loadPCDFile<PointType>(targets[i], *target_cloud) == -1) {
            std::cout << "Could not load target cloud, closing" << std::endl;
            exit(EXIT_FAILURE);
        }
    }
    registration = std::make_unique<ProbPointCloudRegistration>(source_cloud, target_cloud, params);
    registration->align();
    auto estimated_transform = registration -> transformation();
    pcl::transformPointCloud(*source_cloud, *aligned_source, estimated_transform);
    double final_error = point_cloud_registration_benchmark::calculate_error(source_ground_truth, aligned_source);

}


return 0;
}
