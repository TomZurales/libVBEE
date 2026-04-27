#pragma once

#include "observability_model.h"
#include <memory>
#include <tuple>
#include <vector>

class VBEE {
private:
    std::shared_ptr<ObservabilityModel> obs_model;
    Eigen::Vector3f position;
    float _pe;
    float false_negative_rate;
    float false_positive_rate;

public:
    VBEE(const Eigen::Vector3f& position,
         std::shared_ptr<ObservabilityModel> obs_model,
         float false_negative_rate = 0.01f,
         float false_positive_rate = 0.01f);

    float step(const std::vector<std::tuple<Eigen::Vector3f, bool>>& observations);
    float query(const Eigen::Vector3f& position) const;
    float getPe() const { return _pe; }
    void setPosition(const Eigen::Vector3f& new_position) { position = new_position; }
};
