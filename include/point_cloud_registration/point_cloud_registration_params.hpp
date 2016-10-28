#ifndef POINT_CLOUD_REGISTRATION_POINT_CLOUD_REGISTRATION_PARAMS_H
#define POINT_CLOUD_REGISTRATION_POINT_CLOUD_REGISTRATION_PARAMS_H

namespace point_cloud_registration {
struct PointCloudRegistrationParams {
    int max_neighbours;
    double dof = 5;
    double radius = 1;
    int dimension = 3;
    int n_iter = 10;
    double dist_treshold = 0.01;
    bool verbose = false;
    bool debug = false;
    double initial_rotation[4] = {1, 0, 0, 0};
    double initial_translation[3] = {0, 0, 0};
};
}

#endif
