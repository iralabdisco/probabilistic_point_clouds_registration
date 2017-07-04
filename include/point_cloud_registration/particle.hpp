#ifndef POINT_CLOUD_REGISTRATION_PPARTICLE_HPP
#define POINT_CLOUD_REGISTRATION_PPARTICLE_HPP
#include <random>

#include <Eigen/Dense>
#include <pcl/common/common.h>

#include "point_cloud_registration/utilities.hpp"

namespace point_cloud_registration {

#define PARTICLE_STATE_SIZE_ 6

class Particle
{
public:
    Particle(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
             pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, int id): source_cloud_(source_cloud),
        target_cloud_(target_cloud), max_position_(PARTICLE_STATE_SIZE_),
        min_position_(PARTICLE_STATE_SIZE_), position_(PARTICLE_STATE_SIZE_),
        max_velocity_(PARTICLE_STATE_SIZE_),
        velocity_(PARTICLE_STATE_SIZE_), id_(id), generator_(rd_()), uniform_random_(0, 1)
    {
        initial_guess_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        moved_source_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

        pcl::PointXYZ min_t, max_t;
//        pcl::PointXYZ  min_s, max_s;

        pcl::getMinMax3D(*target_cloud_, min_t, max_t);
//        pcl::getMinMax3D(*source_cloud_, min_s, max_s);

        max_position_ << max_t.x, max_t.y, max_t.z, 360, 360, 360;
        min_position_ << min_t.x, min_t.y, min_t.z, 0, 0, 0;
        for (int i = 0; i < velocity_.size(); i++) {
            max_velocity_[i] = (max_position_[i] - min_position_[i]) / 4.0;
        }
        for (int i = 0; i < position_.size(); i++) {
            std::uniform_real_distribution<double> random_position(min_position_[i], max_position_[i]);
            position_[i] = random_position(generator_);
        }
        velocity_  << 0, 0, 0, 0, 0, 0;
        score_ =  score();
        best_score_ = score_;
        best_position_ = position_;
        global_best_ = position_;
        gbest_score_ = score_;
    }

    Particle()
    {

    }

    Particle(const Particle &other)
    {
        *this = other;
    }

    Particle &operator=( const Particle &other )
    {
        source_cloud_ = other.source_cloud_;
        initial_guess_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        moved_source_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        target_cloud_ = other.target_cloud_;
        position_ = other.position_;
        velocity_ = other.velocity_;
        max_velocity_ = other.max_velocity_;
        best_position_ = other.best_position_;
        score_ = other.score_;
        best_score_ = other.best_score_;
        gbest_score_ = other.gbest_score_;
        global_best_ = other.global_best_;
        max_position_ = other.max_position_;
        min_position_ = other.min_position_;
        id_ = other.id_;
        return *this;
    }

    Eigen::Affine3d getTransformation()
    {
        Eigen::Affine3d trans(Eigen::Affine3d::Identity());
        trans.translate(Eigen::Vector3d(position_[0], position_[1], position_[2]));

        double axis_x = sin(position_[4] * 0.0174533) * cos(position_[5] * 0.0174533);
        double axis_y = sin(position_[4] * 0.0174533) * sin(position_[5] * 0.0174533);
        double axis_z = cos(position_[4] * 0.0174533);
        Eigen::Vector3d axis(axis_x, axis_y, axis_z);
        axis.normalize();
        Eigen::AngleAxis<double> rotation(position_[3] * 0.0174533, axis);
        trans.rotate(rotation);
        return trans;
    }

    double getScore() const
    {
        return score_;
    }

    Eigen::VectorXd getPosition() const
    {
        return position_;
    }

    int getId() const
    {
        return id_;
    }

    static bool cmp(const Particle &p1, const Particle &p2)
    {
        return p1.getScore() < p2.getScore();
    }

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr initial_guess_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr moved_source_cloud_;
    Eigen::VectorXd position_;
    Eigen::VectorXd velocity_;
    Eigen::VectorXd max_velocity_;
    Eigen::VectorXd best_position_;
    double score_;
    double best_score_;
    double gbest_score_;
    Eigen::VectorXd global_best_;
    std::random_device rd_;
    std::mt19937 generator_;
    std::uniform_real_distribution<double> uniform_random_;
    Eigen::VectorXd max_position_;
    Eigen::VectorXd min_position_;
    const double ALPHA_ = 0.7298;
    //const double ALPHA = 0.9;
//    const double BETA = 1.4961;
    const double BETA_ = 1.9;
    const Eigen::Index STATE_SIZE_ = 6;
    int id_;

    double score()
    {
        pcl::transformPointCloud (*source_cloud_, *moved_source_cloud_, getTransformation());
        return -robustMedianClosestDistance(moved_source_cloud_, target_cloud_);
    }

    void evolve()
    {
//        std::uniform_real_distribution<double> random_beta(0, 2.05);
//        velocity_ = ALPHA * (velocity_ + random_beta(generator) * (best_position_ - position_) +
//                             random_beta(generator) * (global_best_ - position_));

        velocity_ = velocity_ * ALPHA_ +  2.1 * uniform_random_(generator_) * (best_position_ - position_) +
                    uniform_random_(generator_) * BETA_ * (global_best_ - position_);

        for (int i = 0; i < velocity_.size(); i++) {
            if (velocity_[i] > max_velocity_[i]) {
                velocity_[i] = max_velocity_[i];
            } else if (velocity_[i] < -max_velocity_[i]) {
                velocity_[i] = -max_velocity_[i];
            }
        }

        position_ = position_ + velocity_;
        bool valid = true;
        for (int i = 0; i < position_.size(); i++) {
            if (position_[i] > max_position_[i]) {
                position_[i] = max_position_[i];
                velocity_[i] = - uniform_random_(generator_) * velocity_[i];
            } else if (position_[i] < min_position_[i]) {
                position_[i] = min_position_[i];
                velocity_[i] = - uniform_random_(generator_) * velocity_[i];
            }
        }

        score_ =  score();
        if (score_ > best_score_) {
            best_score_ = score_;
            best_position_ = position_;
        }

    }


    void setGlobalBest(Particle gbest)
    {
        if (gbest_score_ < gbest.score_) {
            global_best_ = gbest.position_;
            gbest_score_ = gbest.score_;
        }
    }

    friend class Swarm;

};

std::ostream &operator<<(std::ostream &os, Particle const &p)
{
    os << p.getId() << ": ";
    for (int i = 0; i < p.getPosition().size(); i++) {
        os << p.getPosition()[i];
        os << " ";
    }
    os << " --> " << p.getScore();
    return os;
}

#undef PARTICLE_STATE_SIZE_

}  // namespace point_cloud_registration

#endif
