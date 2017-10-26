#ifndef POINT_CLOUD_REGISTRATION_POINT_CLOUD_REGISTRATION_PARAMS_H
#define POINT_CLOUD_REGISTRATION_POINT_CLOUD_REGISTRATION_PARAMS_H

namespace point_cloud_registration {
struct PointCloudRegistrationParams {
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
};
}

#endif
