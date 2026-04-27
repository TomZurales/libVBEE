#pragma once

#include <Eigen/Core>

class ObservabilityModel
{
    public:
        virtual float query(const Eigen::Vector3f& position) const = 0;
        virtual float update(const std::tuple<Eigen::Vector3f, bool>& observation) = 0;
        virtual float getObservability() const = 0;
};