#ifndef PROB_POINT_CLOUD_REGISTRATION_POINT_CLOUD_REGISTRATION_PARAMS_HPP
#define PROB_POINT_CLOUD_REGISTRATION_POINT_CLOUD_REGISTRATION_PARAMS_HPP

namespace prob_point_cloud_registration {
struct ProbPointCloudRegistrationParams {
    int max_neighbours = 20;
    double dof = 5;
    double radius = 1;
    int n_iter = 1000;
    double cost_drop_thresh = 0.01;
    double n_cost_drop_it = 5;
    bool verbose = false;
    bool summary = false;
    double initial_rotation[4] = {1, 0, 0, 0};
    double initial_translation[3] = {0, 0, 0};
    double source_filter_size = 0;
    double target_filter_size = 0;
    double source_points_fraction = 1.0;
};
}

#endif
