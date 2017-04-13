#ifndef POINT_CLOUD_REGISTRATION_PPARTICLE_HPP
#define POINT_CLOUD_REGISTRATION_PPARTICLE_HPP
#include <random>

#include <Eigen/Dense>

#include "point_cloud_registration/pso_registration.hpp"
#include "point_cloud_registration/point_cloud_registration_params.hpp"
#include "point_cloud_registration/utilities.hpp"

namespace point_cloud_registration {

using point_cloud_registration::PointCloudRegistrationParams;
using point_cloud_registration::PSORegistration;

class Particle
{
public:
    Particle(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
             pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud,
             PointCloudRegistrationParams parameters): source_cloud_(source_cloud),
        target_cloud_(target_cloud), params_(parameters), max_position_(7), min_position_(7), position_(7),
        velocity_(7)
    {
        initial_guess_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        max_position_ << 0, 0, 0, 360, 360, 360, 100;
        min_position_ << 0, 0, 0, 0, 0, 0, 100;
        for (int i = 0; i < position_.size(); i++) {
            std::uniform_real_distribution<double> random_number(min_position_[i], max_position_[i]);
            position_[i] = random_number(generator);
            velocity_ = max_position_ / 3.0;
            velocity_[6] = 0;
        }

        setParamsFromPosition();
        score_ = registration_->align();
        best_score_ = score_;
        best_position_ = position_;
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
        target_cloud_ = other.target_cloud_;
        position_ = other.position_;
        velocity_ = other.velocity_;
        best_position_ = other.best_position_;
        score_ = other.score_;
        best_score_ = other.best_score_;
        gbest_score_ = other.gbest_score_;
        global_best_ = other.global_best_;
        max_position_ = other.max_position_;
        min_position_ = other.min_position_;
        params_ = other.params_;
        setParamsFromPosition();
    }

    void evolve()
    {
        std::uniform_real_distribution<double> random_beta(0, BETA);
        velocity_ = velocity_ * ALPHA + random_beta(generator) * (best_position_ - position_) + random_beta(
                        generator) * (global_best_ - position_);
        position_ = position_ + velocity_;
        for (int i = 0; i < position_.size(); i++) {
            if (position_[i] > max_position_[i]) {
                position_[i] = position_[i] - 2 * velocity_[i];
            } else if (position_[i] < min_position_[i]) {
                position_[i] = position_[i] + 2 * velocity_[i];
            }
        }
        setParamsFromPosition();
        score_ = registration_->align();
        if (score_ < best_score_) {
            best_score_ = score_;
            best_position_ = position_;
        }
    }

    void setParamsFromPosition()
    {
        Eigen::Affine3d initial_guess_trans(Eigen::Affine3d::Identity());
        initial_guess_trans.translate(Eigen::Vector3d(position_[0], position_[1], position_[2]));
        initial_guess_trans.rotate(point_cloud_registration::euler2Quaternion(position_[3] * 0.0174533,
                                                                              position_[4] * 0.0174533, position_[5] * 0.0174533));
        pcl::transformPointCloud (*source_cloud_, *initial_guess_, initial_guess_trans);
        params_.radius = position_[6];
        registration_ = std::make_unique<PSORegistration>(initial_guess_, target_cloud_, params_);
    }

    void setGlobalBest(Particle &gbest)
    {
        global_best_ = gbest.position_;
        gbest_score_ = gbest.score_;
    }

    double getScore()
    {
        return score_;
    }

    Eigen::VectorXd getPosition()
    {
        return position_;
    }

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr initial_guess_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_;
    std::unique_ptr<PSORegistration> registration_;
    Eigen::VectorXd position_;
    Eigen::VectorXd velocity_;
    Eigen::VectorXd best_position_;
    double score_;
    double best_score_;
    double gbest_score_;
    Eigen::VectorXd global_best_;
    std::default_random_engine generator;
    Eigen::VectorXd max_position_;
    Eigen::VectorXd min_position_;
    PointCloudRegistrationParams params_;
    const double ALPHA = 0.7298;
    const double BETA = 1.4961;
};
}  // namespace point_cloud_registration

#endif
