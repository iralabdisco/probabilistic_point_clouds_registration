#ifndef POINT_CLOUD_REGISTRATION_SWARM_HPP
#define POINT_CLOUD_REGISTRATION_SWARM_HPP

#include "point_cloud_registration/particle.hpp"

using point_cloud_registration::Particle;

namespace point_cloud_registration {
class Swarm
{
public:
    Swarm(int n_gen): particles_() {}

    add_particle(Particle &part)
    {
        particles_.push_back(part);
    }

    void evolve(int n_gen)
    {
        for (auto &part : particles) {
            part.setGlobalBest(findBest(part.getId()));
            std::cout << part << std::endl;
        }
        best_ = findGBest();
        std::cout << "................." << std::endl;
        std::cout << "Best: " << best_ << std::endl;
        std::cout << "---------------------" << std::endl;

        for (int i = 0; i < n_gen; i++) {
            for (int j = 0; j < particles_.size(); j++) {
                particles_[j].evolve();
                std::cout << particles[j] << std::endl;
            }
            std::cout << "................." << std::endl;

            for (int ind = 0; ind < particles.size(); ind++) {
                particles[ind].setGlobalBest(findLBest(ind));
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

    void setScores()
    {
        for (int i = 0; i < particles_.size(); i++) {
            scores_[i] = particles_[i].getScore();
        }
    }

    Particle findGBest()
    {
        int best_index = std::max_element(particles_.begin(), particles_.end(), Particle.cmp);
        return particles_[best_index];
    }

    Particle findLBest(int index)
    {
        std::vector<Particle *> neighbours;
        if (index == 0) {
            neighbours.push_back(&(particles_.back()));
        } else {
            neighbours.push_back(&(particles_[index - 1]));
        }

        if (index == particles_.size() - 1) {
            neighbours.push_back(&(particles_.front()));
        } else {
            neighbours.push_back(&(particles_[index + 1]));
        }

        int best_index = std::max_element(neighbours_scores.begin(), neighbours_scores.begin(),
                                          Particle.cmp);
        Particle lbest = *(neighbours[best_index]);

        return lbest;
    }
};
}

#endif // SWARM_HPP
