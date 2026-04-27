#pragma once

#include "observability_model.h"

class DiscreteBoundary : public ObservabilityModel {
private:
    int k;
    float ambiguous_range;
    std::vector<float> max_seen;
    std::vector<float> min_unseen;
    std::vector<float> bin_observabilities;
    std::vector<int> bin_ambiguous;

    int _select_bin(const Eigen::Vector3f& position) const;
public:
    DiscreteBoundary(int k, float ambiguous_range);
    float query(const Eigen::Vector3f& position) const override;
    float update(const std::tuple<Eigen::Vector3f, bool>& observation) override;
    float getObservability() const override;
};