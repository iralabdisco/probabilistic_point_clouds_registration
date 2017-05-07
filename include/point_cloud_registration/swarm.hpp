#ifndef POINT_CLOUD_REGISTRATION_SWARM_HPP
#define POINT_CLOUD_REGISTRATION_SWARM_HPP

#include <fstream>
#include <thread>

//#include <omp.h>

#include "point_cloud_registration/particle.hpp"

using point_cloud_registration::Particle;

namespace point_cloud_registration {
class Swarm
{
public:
    Swarm(): particles_(), generator_(rd_()), diversity_count_(0) {}

    void add_particle(Particle part)
    {
        particles_.push_back(part);
    }

    void evolve(int n_gen)
    {
        std::ofstream report_file;
        report_file.open("risultatissimi.txt");
        std::uniform_real_distribution<double> random_gen(0, 1);
        double avoidance_rate;

        #pragma omp parallel for num_threads(std::thread::hardware_concurrency())
        for (int i = 0; i < particles_.size(); i++) {
            particles_[i].setGlobalBest(findLBest(particles_[i].getId()));
        }

        best_ = findGBest();
        std::cout << "................." << std::endl;
        std::cout << "Best: " << best_ << std::endl;
        std::cout << "---------------------" << std::endl;
        for (int i = 0; i < n_gen; i++) {

            if (i < 0.75 * n_gen) {
                avoidance_rate = random_gen(generator_);
            } else {
                avoidance_rate = 1;
            }
            double avoidance_coeff = -2 * (1 - i / n_gen);
            bool restart = false;
            if (diversity_count_ > 20) {
                std::cout << "Random restart" << std::endl;
                std::cout << "Random restart" << std::endl;
                std::cout << "Random restart" << std::endl;
                std::cout << "Random restart" << std::endl;
                std::cout << "Random restart" << std::endl;

                diversity_count_ = 0;
                restart  = true;
            }
            #pragma omp parallel for num_threads(std::thread::hardware_concurrency())
            for (int j = 0; j < particles_.size(); j++) {
                if (restart) {
                    //particles_[j].randomRestart();
                }
                if (avoidance_rate > 0) {
                    particles_[j].evolveTest(1);
                    avoidance_rate--;
                } else {
                    particles_[j].evolveTest(1);
                }
            }
            std::cout << *this << "................." << std::endl;

            for (int ind = 0; ind < particles_.size(); ind++) {
                particles_[ind].setGlobalBest(findLBest(ind));
            }

            Particle gen_best = findGBest();
            if (best_.getScore() < gen_best.getScore()) {
                best_ = gen_best;
                diversity_count_ = 0;
            } else {
                diversity_count_++;
            }
            std::cout << "Gen " << i << " best: " << best_ << std::endl;
            std::cout << "---------------------" << std::endl;
            report_file << best_ << std::endl;

        }
    }

private:
    std::vector<Particle> particles_;
    Particle best_;
    std::random_device rd_;
    std::mt19937 generator_;
    int diversity_count_;
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
    friend std::ostream &operator<<(std::ostream &os, Swarm const &s);
};

std::ostream &operator<<(std::ostream &os, Swarm const &s)
{
    for (auto &p : s.particles_) {
        os << p << std::endl;
    }
    return os;
}

}

#endif // SWARM_HPP
