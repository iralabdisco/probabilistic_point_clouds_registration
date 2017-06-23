#ifndef POINT_CLOUD_REGISTRATION_PARTICLE_PARAMETERS_HPP
#define POINT_CLOUD_REGISTRATION_PARTICLE_PARAMETERS_HPP
#include <random>

#include <Eigen/Dense>

#include "point_cloud_registration/point_cloud_registration.h"
#include "point_cloud_registration/point_cloud_registration_params.hpp"
#include "point_cloud_registration/utilities.hpp"

namespace point_cloud_registration {

using point_cloud_registration::PointCloudRegistrationParams;
using point_cloud_registration::PointCloudRegistration;

#define PARTICLE_STATE_SIZE_ 2

class ParticleParameters
{
public:
    ParticleParameters(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud,
                       PointCloudRegistrationParams parameters, int id): source_cloud_(source_cloud),
        target_cloud_(target_cloud), params_(parameters), max_position_(PARTICLE_STATE_SIZE_),
        min_position_(PARTICLE_STATE_SIZE_), position_(PARTICLE_STATE_SIZE_),
        max_velocity_(PARTICLE_STATE_SIZE_),
        velocity_(PARTICLE_STATE_SIZE_), id_(id), generator_(rd_())
    {
        aligned_source_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        max_position_ << 20, 50;
        min_position_ <<  0.1, 1;
//        max_velocity_ <<  0, 0;
        for (int i = 0; i < position_.size(); i++) {
            std::uniform_real_distribution<double> random_number(min_position_[i], max_position_[i]);
            position_[i] = random_number(generator_);
        }
        velocity_  << 0, 0;
        score_ = score();

        best_score_ = score_;
        best_position_ = position_;
        global_best_ = position_;
        gbest_score_ = score_;
    }

    double score()
    {
        params_.radius = position_[0];
        params_.max_neighbours = position_[1];
        registration_ = std::make_unique<PointCloudRegistration>(source_cloud_, target_cloud_, params_);
        registration_->align();
        transformation_ = registration_->transformation();
        pcl::transformPointCloud (*source_cloud_, *aligned_source_, transformation_);
        return -sumSquaredError(aligned_source_, target_cloud_);
    }

    ParticleParameters()
    {

    }

    ParticleParameters(const ParticleParameters &other)
    {
        *this = other;
    }

    ParticleParameters &operator=( const ParticleParameters &other )
    {
        source_cloud_ = other.source_cloud_;
        aligned_source_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
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
        params_ = other.params_;
        transformation_ = other.transformation_;
        id_ = other.id_;

        return *this;
    }

    void evolve()
    {
        std::uniform_real_distribution<double> random_beta(0, BETA);
        velocity_ = velocity_ * ALPHA + random_beta(generator_) * (best_position_ - position_) +
                    random_beta(
                        generator_) * (global_best_ - position_);

        position_ = position_ + velocity_;
        bool valid = true;
        for (int i = 0; i < position_.size(); i++) {
            if (position_[i] > max_position_[i] || position_[i] < min_position_[i]) {
                valid = false;
            }
        }
        if (valid) {
            score_ = score();
            if (score_ > best_score_) {
                best_score_ = score_;
                best_position_ = position_;
            }
        }
    }

    Eigen::Affine3d getTransformation()
    {
        return transformation_;
    }

    void setGlobalBest(ParticleParameters gbest)
    {
        if (gbest_score_ < gbest.score_) {
            global_best_ = gbest.position_;
            gbest_score_ = gbest.score_;
        }
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

    static bool cmp(const ParticleParameters &p1, const ParticleParameters &p2)
    {
        return p1.getScore() < p2.getScore();
    }

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_source_;
    std::unique_ptr<PointCloudRegistration> registration_;
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
    Eigen::VectorXd max_position_;
    Eigen::VectorXd min_position_;
    Eigen::Affine3d transformation_;
    PointCloudRegistrationParams params_;
    const double ALPHA = 0.7298;
    //const double ALPHA = 0.9;
//    const double BETA = 1.4961;
    const double BETA = 1.9;
    int id_;
};

std::ostream &operator<<(std::ostream &os, ParticleParameters const &p)
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
