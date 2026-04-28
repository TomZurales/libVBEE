#pragma once

#include "observability_model.h"
#include "observation.h"
#include <memory>
#include <vector>

namespace VBEE {

class Instance {
private:
    std::shared_ptr<ObservabilityModel> obs_model;
    float _pe;
    float false_negative_rate;
    float false_positive_rate;
    float n_eff;

public:
    Instance() = default;
    Instance(std::shared_ptr<ObservabilityModel> obs_model,
             float false_negative_rate = 0.000001f,
             float false_positive_rate = 0.01f,
             float n_eff = 1.0f);

    float step(const std::vector<Observation>& observations);
    float query(const Eigen::Vector3f& viewpoint) const;
    float getPe() const { return _pe; }
    void merge(Instance& other);
};

} // namespace VBEE
