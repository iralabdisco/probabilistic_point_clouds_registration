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
             PointCloudRegistrationParams parameters, int id): source_cloud_(source_cloud),
        target_cloud_(target_cloud), params_(parameters), max_position_(8),
        min_position_(8), position_(8),
        velocity_(8), id_(id), generator(rd())
    {
        initial_guess_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        max_position_ << 0.5, 0.5, 0.5, 360, 360, 360, 0.5, 50;
        min_position_ << -0.5, -0.5, -0.5, 0, 0, 0, 0, 1;
        for (int i = 0; i < position_.size(); i++) {
            std::uniform_real_distribution<double> random_number(min_position_[i], max_position_[i]);
            position_[i] = random_number(generator);
            velocity_ = max_position_ / 3.0;
        }

        setParamsFromPosition();
        score_ = - registration_->align();
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
        id_ = other.id_;
        return *this;
    }

    void evolve()
    {
        std::uniform_real_distribution<double> random_beta(0, BETA);
        velocity_ = velocity_ * ALPHA + random_beta(generator) * (best_position_ - position_) + random_beta(
                        generator) * (global_best_ - position_);
        position_ = position_ + velocity_;
        setParamsFromPosition();
        bool valid = true;
        for (int i = 0; i < position_.size(); i++) {
            if (position_[i] > max_position_[i] || position_[i] < min_position_[i]) {
                valid = false;
            }
        }
        if (valid) {
            score_ =  - registration_->align();
            if (score_ > best_score_) {
                best_score_ = score_;
                best_position_ = position_;
            }
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
        params_.max_neighbours = position_[7];
        registration_ = std::make_unique<PSORegistration>(initial_guess_, target_cloud_, params_);
    }

    void setGlobalBest(Particle gbest)
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

    static bool cmp(const Particle &p1, const Particle &p2)
    {
        return p1.getScore() < p2.getScore();
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
    std::random_device rd;
    std::mt19937 generator;
    Eigen::VectorXd max_position_;
    Eigen::VectorXd min_position_;
    PointCloudRegistrationParams params_;
    const double ALPHA = 0.7298;
    const double BETA = 1.4961;
    std::size_t STATE_SIZE_ = 8;
    int id_;
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

}  // namespace point_cloud_registration

#endif
