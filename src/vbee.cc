#include "vbee.h"
#include <cmath>
#include <algorithm>

VBEE::VBEE(const Eigen::Vector3f& position,
           std::shared_ptr<ObservabilityModel> obs_model,
           float false_negative_rate, float false_positive_rate)
    : obs_model(std::move(obs_model)), position(position), _pe(0.9f),
      false_negative_rate(false_negative_rate), false_positive_rate(false_positive_rate) {}

float VBEE::step(const std::vector<std::tuple<Eigen::Vector3f, bool>>& observations) {
    float prior_observability = obs_model->getObservability();
    std::vector<std::pair<float, float>> neg_obs_data;
    int positive_count = 0;

    for (const auto& obs : observations) {
        const Eigen::Vector3f& pos = std::get<0>(obs);
        bool seen = std::get<1>(obs);

        if (seen) {
            ++positive_count;
            obs_model->update(obs);
        } else {
            float query_val = obs_model->query(pos);
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

        for (const auto& [q, impact] : neg_obs_data)
            log_L += (impact / total_neg_impact) * std::log(1.0f - (1.0f - false_negative_rate) * q);
    }

    float L = std::exp(log_L);
    _pe = std::max(0.01f, std::min(0.99f, (_pe * L) / (_pe * L + (1.0f - _pe))));
    return _pe;
}

float VBEE::query(const Eigen::Vector3f& position) const {
    return obs_model->query(position);
}
