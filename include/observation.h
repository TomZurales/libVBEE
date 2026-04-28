#pragma once

#include <Eigen/Core>

namespace VBEE {

struct Observation
{
    unsigned long mpId;
    Eigen::Vector3f viewpoint;
    bool seen;
};

} // namespace VBEE
