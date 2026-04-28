#include "instance.h"
#include "observation.h"
#include "observability_models/discrete_boundary.h"
#include <iostream>
#include <random>
#include <vector>

int main() {
    auto obs_model = std::make_shared<VBEE::DiscreteBoundary>(5, 0.5f);
    VBEE::Instance vbee(obs_model);

    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<float> dist(-10.0f, 10.0f);

    std::vector<VBEE::Observation> observations;
    observations.reserve(100);
    for (int i = 0; i < 100; ++i) {
        float x = dist(rng), y = dist(rng), z = dist(rng);
        bool seen = (x > 0 || y > 0 || z > 0) && (x*x + y*y + z*z) < 81.0f;
        observations.push_back({0, Eigen::Vector3f(x, y, z), seen});
    }
    std::cout << vbee.step(observations) << "\n";

    for (int i = 0; i < 10; ++i) {
        observations.clear();
        for (int j = 0; j < 10; ++j) {
            float x = dist(rng), y = dist(rng), z = dist(rng);
            observations.push_back({0, Eigen::Vector3f(x, y, z), false});
        }
        std::cout << vbee.step(observations) << "\n";
    }

    return 0;
}
