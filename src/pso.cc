#include <limits>
#include <memory>
#include <vector>
#include <fstream>
#include <string>
#include <stdlib.h>

#include <boost/make_shared.hpp>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <tclap/CmdLine.h>

#include "point_cloud_registration/pso_registration.hpp"
#include "point_cloud_registration/point_cloud_registration_iteration.hpp"
#include "point_cloud_registration/point_cloud_registration_params.hpp"
#include "point_cloud_registration/utilities.hpp"
#include "point_cloud_registration/particle.hpp"


typedef pcl::PointXYZ PointType;

using point_cloud_registration::PSORegistration;
using point_cloud_registration::Particle;
using point_cloud_registration::PointCloudRegistrationParams;

Particle findBest(std::vector<Particle> &particles)
{
    std::vector<double> scores(particles.size());
    for (int j = 0; j < particles.size(); j++) {
        scores[j] = particles[j].getScore();
    }
    int best_index = std::distance(scores.begin(), std::min_element(scores.begin(), scores.end()));
    Particle best = (particles[best_index]);
    return best;
}

int main(int argc, char **argv)
{
    bool use_gaussian = false;
    std::string source_file_name;
    std::string target_file_name;
    int num_part = 0;
    PointCloudRegistrationParams params;
    try {
        TCLAP::CmdLine cmd("PSO probabilistic point cloud registration", ' ', "1.0");
        TCLAP::UnlabeledValueArg<std::string> source_file_name_arg("source_file_name",
                                                                   "The path of the source point cloud", true, "source_cloud.pcd", "string", cmd);
        TCLAP::UnlabeledValueArg<std::string> target_file_name_arg("target_file_name",
                                                                   "The path of the target point cloud", true, "target_cloud.pcd", "string", cmd);
        TCLAP::ValueArg<float> source_filter_arg("s", "source_filter_size",
                                                 "The leaf size of the voxel filter of the source cloud", false, 0, "float", cmd);
        TCLAP::ValueArg<float> target_filter_arg("t", "target_filter_size",
                                                 "The leaf size of the voxel filter of the target cloud", false, 0, "float", cmd);
        TCLAP::ValueArg<int> max_neighbours_arg("m", "max_neighbours",
                                                "The max cardinality of the neighbours' set", false, 10, "int", cmd);
        TCLAP::ValueArg<int> num_iter_arg("i", "num_iter",
                                          "The maximum number of iterations to perform", false, 10, "int", cmd);
        TCLAP::ValueArg<float> dof_arg("d", "dof", "The Degree of freedom of t-distribution", false, 5,
                                       "float", cmd);
        TCLAP::ValueArg<float> radius_arg("r", "radius", "The radius of the neighborhood search", false, 3,
                                          "float", cmd);
        TCLAP::ValueArg<float> cost_drop_tresh_arg("c", "cost_drop_treshold",
                                                   "If the cost_drop drops below this threshold for too many iterations, the algorithm terminate",
                                                   false, 0.01, "float", cmd);
        TCLAP::ValueArg<int> num_drop_iter_arg("n", "num_drop_iter",
                                               "The maximum number of iterations during which the cost drop is allowed to be under cost_drop_thresh",
                                               false, 5, "int", cmd);
        TCLAP::SwitchArg use_gaussian_arg("u", "use_gaussian",
                                          "Whether to use a gaussian instead the a t-distribution", cmd, false);
        TCLAP::ValueArg<int> num_part_arg("p", "num_part",
                                          "The number of particles of the swarm", false, 50, "int", cmd);
        cmd.parse(argc, argv);

        params.max_neighbours = max_neighbours_arg.getValue();
        use_gaussian = use_gaussian_arg.getValue();
        params.dof = dof_arg.getValue();
        params.radius = radius_arg.getValue();
        params.n_iter = num_iter_arg.getValue();
        params.cost_drop_thresh = cost_drop_tresh_arg.getValue();
        params.n_cost_drop_it = num_drop_iter_arg.getValue();
        source_file_name = source_file_name_arg.getValue();
        target_file_name = target_file_name_arg.getValue();
        params.source_filter_size = source_filter_arg.getValue();
        params.target_filter_size = target_filter_arg.getValue();
        num_part = num_part_arg.getValue();
    } catch (TCLAP::ArgException &e) {
        std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
        exit(EXIT_FAILURE);
    }


    if (use_gaussian) {
        std::cout << "Using gaussian model" << std::endl;
        params.dof = std::numeric_limits<double>::infinity();
    } else {
        std::cout << "Using a t-distribution with " << params.dof << " dof" << std::endl;
    }
    std::cout << "Radius of the neighborhood search: " << params.radius << std::endl;
    std::cout << "Max number of neighbours: " << params.max_neighbours << std::endl;
    std::cout << "Max number of iterations: " << params.n_iter << std::endl;
    std::cout << "Cost drop threshold: " << params.cost_drop_thresh << std::endl;
    std::cout << "Num cost drop iter: " << params.n_cost_drop_it << std::endl;
    std::cout << "Loading source point cloud from " << source_file_name << std::endl;
    pcl::PointCloud<PointType>::Ptr source_cloud = boost::make_shared<pcl::PointCloud<PointType>>();
    if (pcl::io::loadPCDFile<PointType>(source_file_name, *source_cloud) == -1) {
        std::cout << "Could not load source cloud, closing" << std::endl;
        exit(EXIT_FAILURE);
    }
    std::cout << "Loading target point cloud from " << target_file_name << std::endl;
    pcl::PointCloud<PointType>::Ptr target_cloud =
        boost::make_shared<pcl::PointCloud<PointType>>();
    if (pcl::io::loadPCDFile<PointType>(target_file_name, *target_cloud) == -1) {
        std::cout << "Could not load target cloud, closing" << std::endl;
        exit(EXIT_FAILURE);
    }

    std::vector<Particle> particles;
    std::vector<double> scores(num_part);
    Particle gbest;
    for (int i = 0; i < num_part; i++) {
        particles.push_back(Particle(source_cloud, target_cloud, params, i));
    }
    gbest = findBest(particles);
    for (auto &part : particles) {
        part.setGlobalBest(gbest);
        std::cout << part << std::endl;
    }
    std::cout << "................." << std::endl;
    std::cout << "Best: " << gbest << std::endl;
    std::cout << "---------------------" << std::endl;


    for (int i = 0; i < 1000; i++) {
        for (int j = 0; j < particles.size(); j++) {
            particles[j].evolve();
            scores[j] = particles[j].getScore();
            std::cout << particles[j] << std::endl;
        }
        std::cout << "................." << std::endl;
        Particle current_best = findBest(particles);
        if (current_best.getScore() < gbest.getScore()) {
            gbest = current_best;
            for (auto &part : particles) {
                part.setGlobalBest(gbest);
            }
        }
        std::cout << "Gen " << i << " best: " << gbest << std::endl;
        std::cout << "---------------------" << std::endl;

    }

//    registration->align();
//    auto estimated_transform = registration->transformation();
//    pcl::PointCloud<PointType>::Ptr aligned_source = boost::make_shared<pcl::PointCloud<PointType>>();
//    pcl::transformPointCloud (*source_cloud, *aligned_source, estimated_transform);

//        std::string aligned_source_name = "aligned_" + source_file_name;
//        std::cout << "Saving aligned source cloud to: " << aligned_source_name.c_str() << std::endl;
//        pcl::io::savePCDFile(aligned_source_name, *aligned_source);

    return 0;
}
