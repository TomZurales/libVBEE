#include "observability_models/discrete_boundary.h"
#include <cmath>
#include <numeric>

static constexpr float BIN_VOL_COEFF = M_PI / 15.0f;  // (4π/20)/3

static const std::vector<Eigen::Vector3f> ICOS_CENTERS = {
    {-0.1875924741, 0.7946544723, -0.5773502692},
    {0.4911234732, 0.7946544723, 0.3568220898},
    {-0.6070619982, 0.7946544723, -0.0000000000},
    {0.4911234732, 0.7946544723, -0.3568220898},
    {-0.1875924741, 0.7946544723, 0.5773502692},
    {-0.9822469464, 0.1875924741, -0.0000000000},
    {-0.3035309991, 0.1875924741, 0.9341723590},
    {0.7946544723, 0.1875924741, -0.5773502692},
    {-0.3035309991, 0.1875924741, -0.9341723590},
    {0.7946544723, 0.1875924741, 0.5773502692},
    {-0.7946544723, -0.1875924741, -0.5773502692},
    {0.3035309991, -0.1875924741, 0.9341723590},
    {-0.7946544723, -0.1875924741, 0.5773502692},
    {0.3035309991, -0.1875924741, -0.9341723590},
    {0.9822469464, -0.1875924741, 0.0000000000},
    {0.1875924741, -0.7946544723, -0.5773502692},
    {0.6070619982, -0.7946544723, 0.0000000000},
    {-0.4911234732, -0.7946544723, 0.3568220898},
    {-0.4911234732, -0.7946544723, -0.3568220898},
    {0.1875924741, -0.7946544723, 0.5773502692}
};

static float map_value(float value, float from_min, float from_max, float to_min, float to_max) {
    if (from_max == from_min) return to_min;
    return to_min + (to_max - to_min) * ((value - from_min) / (from_max - from_min));
}

namespace VBEE {

DiscreteBoundary::DiscreteBoundary(int k, float ambiguous_range)
    : k(k), ambiguous_range(ambiguous_range),
      max_seen(20, 0.0f), min_unseen(20, INFINITY),
      bin_observabilities(20, 0.0f), bin_ambiguous(20, k) {}

float DiscreteBoundary::query(const Eigen::Vector3f& position) const {
    float r = position.norm();
    int b = _select_bin(position);
    if (r <= max_seen[b]) {
        if (r < min_unseen[b])
            return 1.0f;
        return map_value(bin_ambiguous[b], 0, 2 * k,
                         0.5f - ambiguous_range / 2.0f,
                         0.5f + ambiguous_range / 2.0f);
    }
    return 0.0f;
}

float DiscreteBoundary::update(const Observation& observation) {
    const Eigen::Vector3f& pos = observation.viewpoint;
    bool obs = observation.seen;
    float r = pos.norm();
    int b = _select_bin(pos);

    bool updated = false;

    if (obs) {
        if (r > max_seen[b]) { max_seen[b] = r; updated = true; }
    } else {
        if (r < min_unseen[b]) { min_unseen[b] = r; updated = true; }
    }

    if (r <= max_seen[b] && r >= min_unseen[b]) {
        bin_ambiguous[b] = obs ? std::min(2 * k, bin_ambiguous[b] + 1)
                                : std::max(0,     bin_ambiguous[b] - 1);
        updated = true;
    }

    if (updated) {
        if (min_unseen[b] < max_seen[b]) {
            float s1_vol = BIN_VOL_COEFF * std::pow(min_unseen[b], 3);
            float s2_vol = BIN_VOL_COEFF * std::pow(max_seen[b], 3) - s1_vol;
            float amb = map_value(bin_ambiguous[b], 0, 2 * k,
                                  0.5f - ambiguous_range / 2.0f,
                                  0.5f + ambiguous_range / 2.0f);
            bin_observabilities[b] = s1_vol + amb * s2_vol;
        } else {
            bin_observabilities[b] = BIN_VOL_COEFF * std::pow(max_seen[b], 3);
        }
    }

    return getObservability();
}

void DiscreteBoundary::merge(std::shared_ptr<ObservabilityModel> other) {
    auto* o = static_cast<DiscreteBoundary*>(other.get());
    for (int b = 0; b < 20; ++b) {
        min_unseen[b] = std::min(min_unseen[b], o->min_unseen[b]);
        max_seen[b]   = std::max(max_seen[b],   o->max_seen[b]);
        bin_ambiguous[b] = (bin_ambiguous[b] + o->bin_ambiguous[b]) / 2;

        if (min_unseen[b] < max_seen[b]) {
            float s1_vol = BIN_VOL_COEFF * std::pow(min_unseen[b], 3);
            float s2_vol = BIN_VOL_COEFF * std::pow(max_seen[b], 3) - s1_vol;
            float amb = map_value(bin_ambiguous[b], 0, 2 * k,
                                  0.5f - ambiguous_range / 2.0f,
                                  0.5f + ambiguous_range / 2.0f);
            bin_observabilities[b] = s1_vol + amb * s2_vol;
        } else {
            bin_observabilities[b] = BIN_VOL_COEFF * std::pow(max_seen[b], 3);
        }
    }
}

float DiscreteBoundary::getObservability() const {
    return std::accumulate(bin_observabilities.begin(), bin_observabilities.end(), 0.0f);
}

int DiscreteBoundary::_select_bin(const Eigen::Vector3f& position) const {
    float r = position.norm();
    if (r == 0.0f) return 0;

    Eigen::Vector3f n = position / r;
    int best = 0;
    float best_dot = -INFINITY;
    for (int i = 0; i < static_cast<int>(ICOS_CENTERS.size()); ++i) {
        float dot = n.dot(ICOS_CENTERS[i]);
        if (dot > best_dot) { best_dot = dot; best = i; }
    }
    return best;
}

} // namespace VBEE
