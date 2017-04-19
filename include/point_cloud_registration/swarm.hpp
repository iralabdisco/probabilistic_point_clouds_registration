#ifndef POINT_CLOUD_REGISTRATION_SWARM_HPP
#define POINT_CLOUD_REGISTRATION_SWARM_HPP

#include <omp.h>

#include "point_cloud_registration/particle.hpp"

using point_cloud_registration::Particle;

namespace point_cloud_registration {
class Swarm
{
public:
    Swarm(): particles_() {}

    void add_particle(Particle part)
    {
        particles_.push_back(part);
    }

    void evolve(int n_gen)
    {

        for (auto &part : particles_) {
            part.setGlobalBest(findLBest(part.getId()));
            std::cout << part << std::endl;
        }
        best_ = findGBest();
        std::cout << "................." << std::endl;
        std::cout << "Best: " << best_ << std::endl;
        std::cout << "---------------------" << std::endl;

        for (int i = 0; i < n_gen; i++) {
            for (int j = 0; j < particles_.size(); j++) {
                particles_[j].evolve();
                std::cout << particles_[j] << std::endl;
            }
            std::cout << "................." << std::endl;

            for (int ind = 0; ind < particles_.size(); ind++) {
                particles_[ind].setGlobalBest(findLBest(ind));
            }

            Particle gen_best = findGBest();
            if (best_.getScore() < gen_best.getScore()) {
                best_ = gen_best;
            }
            std::cout << "Gen " << i << " best: " << best_ << std::endl;
            std::cout << "---------------------" << std::endl;

        }
    }

private:
    std::vector<Particle> particles_;
    Particle best_;

    Particle findGBest()
    {
        return *(std::max_element(particles_.begin(), particles_.end(), Particle::cmp));
    }

    Particle findLBest(int index)
    {
        std::vector<Particle> neighbours;
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

        Particle lbest = *(std::max_element(neighbours.begin(), neighbours.begin(),
                                            Particle::cmp));

        return lbest;
    }
};
}

#endif // SWARM_HPP
