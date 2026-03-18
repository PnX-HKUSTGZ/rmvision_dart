#ifndef LIGHT_DETECTOR__STABILITY_GATE_HPP_
#define LIGHT_DETECTOR__STABILITY_GATE_HPP_

#include <cmath>

namespace rm_auto_aim_dart
{
struct StabilityGateConfig
{
    double enter_angle_threshold_deg = 3.44;
    double exit_angle_threshold_deg = 4.0;
    int confirm_frames = 3;
};

class StabilityGate
{
public:
    StabilityGate() = default;

    explicit StabilityGate(const StabilityGateConfig &config)
        : config_(config)
    {
    }

    void setConfig(const StabilityGateConfig &config)
    {
        config_ = config;
    }

    void reset()
    {
        stable_ = false;
        candidate_frames_ = 0;
    }

    bool update(double angle_deg)
    {
        const double abs_angle_deg = std::abs(angle_deg);

        if (stable_)
        {
            if (abs_angle_deg > config_.exit_angle_threshold_deg)
            {
                stable_ = false;
                candidate_frames_ = 0;
            }
            return stable_;
        }

        if (abs_angle_deg <= config_.enter_angle_threshold_deg)
        {
            ++candidate_frames_;
            if (candidate_frames_ >= config_.confirm_frames)
            {
                stable_ = true;
            }
        }
        else
        {
            candidate_frames_ = 0;
        }

        return stable_;
    }

private:
    StabilityGateConfig config_;
    bool stable_ = false;
    int candidate_frames_ = 0;
};
} // namespace rm_auto_aim_dart

#endif
