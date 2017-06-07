#ifndef POINT_CLOUD_REGISTRATION_SWARM_PARAMETERS_HPP
#define POINT_CLOUD_REGISTRATION_SWARM_PARAMETERS_HPP

#include <fstream>
#include <thread>

#include <omp.h>

#include "point_cloud_registration/particle_parameters.hpp"

using point_cloud_registration::ParticleParameters;

namespace point_cloud_registration {
class SwarmParameters
{
public:
    SwarmParameters(): particles_(), generator_(rd_()) {}

    void add_particle(ParticleParameters part)
    {
        particles_.push_back(part);
    }

    void init()
    {
        #pragma omp parallel for num_threads(std::thread::hardware_concurrency())
        for (int i = 0; i < particles_.size(); i++) {
            particles_[i].setGlobalBest(findLBest(particles_[i].getId()));
        }

        best_ = findGBest();
    }

    ParticleParameters getBest()
    {
        return best_;
    }

    void evolve()
    {
        #pragma omp parallel for num_threads(std::thread::hardware_concurrency())
        for (int j = 0; j < particles_.size(); j++) {
            particles_[j].evolve();
        }

        for (int ind = 0; ind < particles_.size(); ind++) {
            particles_[ind].setGlobalBest(findLBest(ind));
        }

        ParticleParameters gen_best = findGBest();
        if (best_.getScore() < gen_best.getScore()) {
            best_ = gen_best;
        }

    }

private:
    std::vector<ParticleParameters> particles_;
    ParticleParameters best_;
    std::random_device rd_;
    std::mt19937 generator_;

    ParticleParameters findGBest()
    {
        return *(std::max_element(particles_.begin(), particles_.end(), ParticleParameters::cmp));
    }

    ParticleParameters findLBest(int index)
    {
        std::vector<ParticleParameters> neighbours;
        if (index == 0) {
            neighbours.push_back(particles_.back());
        } else {
            neighbours.push_back(particles_[index - 1]);
        }

        if (index == particles_.size() - 1) {
            neighbours.push_back(particles_.front());
        } else {
            neighbours.push_back(particles_[index + 1]);
        }

        ParticleParameters lbest = *(std::max_element(neighbours.begin(), neighbours.begin(),
                                                      ParticleParameters::cmp));

        return lbest;
    }
    friend std::ostream &operator<<(std::ostream &os, SwarmParameters const &s);
};

std::ostream &operator<<(std::ostream &os, SwarmParameters const &s)
{
    for (auto &p : s.particles_) {
        os << p << std::endl;
    }
    os << "................." << std::endl;
    os << "Best: " << s.best_ << std::endl;
    os << "---------------------";
    return os;
}

}

#endif // SWARM_HPP
