#ifndef POINT_CLOUD_REGISTRATION_PPARTICLE_HPP
#define POINT_CLOUD_REGISTRATION_PPARTICLE_HPP

#include <eigen3/Geometry>

#include "point_cloud_registration/point_cloud_registration_iteration.hpp"
#include "point_cloud_registration/point_cloud_registration_params.hpp"

namespace point_cloud_registration {

class Particle
{
private:
    PointCloudRegistrationIteration registration;
    Position position;
    Position velocity;
    Position best_position;
};

private class Position{
private:
    Eigen::Vector3d translation;
    double angle;
    Eigen::Vector3d axis;
    double radius;
public:
    bool operator<(const Position& l, const Position& r){
        bool result = true;
        if(l.angle < r.angle){
            result &= true;
        }
result &= l.translation.x < r.translation.x && l.translation.x < r.translation.x && l.translation.x < r.translation.x;
    }
}

}  // namespace point_cloud_registration

#endif
