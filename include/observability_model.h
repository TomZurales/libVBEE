#pragma once

#include "observation.h"
#include <Eigen/Core>
#include <memory>

namespace VBEE {

class ObservabilityModel
{
    public:
        virtual float query(const Eigen::Vector3f& position) const = 0;
        virtual float update(const Observation& observation) = 0;
        virtual float getObservability() const = 0;
        virtual void merge(std::shared_ptr<ObservabilityModel> other) = 0;
};

} // namespace VBEE
