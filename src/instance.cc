#include "instance.h"
#include <cmath>
#include <algorithm>

namespace VBEE {

Instance::Instance(std::shared_ptr<ObservabilityModel> obs_model,
                   float false_negative_rate, float false_positive_rate, float n_eff)
    : obs_model(std::move(obs_model)), _pe(0.9f),
      false_negative_rate(false_negative_rate), false_positive_rate(false_positive_rate),
      n_eff(n_eff) {}

float Instance::step(const std::vector<Observation>& observations) {
    float prior_observability = obs_model->getObservability();
    std::vector<std::pair<float, float>> neg_obs_data;
    int positive_count = 0;

    for (const auto& obs : observations) {
        if (obs.seen) {
            ++positive_count;
            obs_model->update(obs);
        } else {
            float query_val = obs_model->query(obs.viewpoint);
            float posterior_observability = obs_model->update(obs);
            float impact = (prior_observability == 0.0f)
                ? 1.0f
                : std::abs((posterior_observability - prior_observability) / prior_observability);
            impact += 1.0f / static_cast<float>(observations.size());
            neg_obs_data.emplace_back(query_val, impact);
            prior_observability = posterior_observability;
        }
    }

    float log_L = static_cast<float>(positive_count) * std::log(1.0f / false_positive_rate);

    if (!neg_obs_data.empty()) {
        float total_neg_impact = 0.0f;
        for (const auto& [q, impact] : neg_obs_data)
            total_neg_impact += impact;

        float effective_obs = std::min(static_cast<float>(neg_obs_data.size()), n_eff);
        for (const auto& [q, impact] : neg_obs_data)
            log_L += effective_obs * (impact / total_neg_impact) * std::log(1.0f - (1.0f - false_negative_rate) * q);
    }

    float L = std::exp(log_L);
    _pe = std::max(0.01f, std::min(0.99f, (_pe * L) / (_pe * L + (1.0f - _pe))));
    return _pe;
}

void Instance::merge(Instance& other) {
    _pe = std::max(_pe, other._pe);
    obs_model->merge(other.obs_model);
}

float Instance::query(const Eigen::Vector3f& position) const {
    return obs_model->query(position);
}

} // namespace VBEE
