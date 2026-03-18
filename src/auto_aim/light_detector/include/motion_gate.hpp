#ifndef LIGHT_DETECTOR__MOTION_GATE_HPP_
#define LIGHT_DETECTOR__MOTION_GATE_HPP_

#include <cmath>

namespace rm_auto_aim_dart
{
struct MotionGateConfig
{
    double angle_delta_threshold_deg = 0.03;
    double pixel_delta_threshold_px = 8.0;
    int stop_confirm_frames = 3;
};

enum class MotionGateState
{
    MOVING,
    STOPPED
};

struct MotionSample
{
    double center_x = 0.0;
    double center_y = 0.0;
    double angle_deg = 0.0;
};

struct MotionGateResult
{
    bool should_send_real = false;
    MotionGateState state = MotionGateState::MOVING;
    int stable_count = 0;
    double angle_delta_deg = 0.0;
    double pixel_delta_px = 0.0;
};

class MotionGate
{
public:
    MotionGate() = default;

    explicit MotionGate(const MotionGateConfig &config)
        : config_(config)
    {
    }

    void setConfig(const MotionGateConfig &config)
    {
        config_ = config;
    }

    void reset()
    {
        has_prev_sample_ = false;
        stable_count_ = 0;
        state_ = MotionGateState::MOVING;
    }

    MotionGateResult update(const MotionSample &sample)
    {
        MotionGateResult result;
        if (!has_prev_sample_)
        {
            prev_sample_ = sample;
            has_prev_sample_ = true;
            stable_count_ = 0;
            state_ = MotionGateState::MOVING;
            result.state = state_;
            return result;
        }

        const double dx = sample.center_x - prev_sample_.center_x;
        const double dy = sample.center_y - prev_sample_.center_y;
        const double pixel_delta_px = std::sqrt(dx * dx + dy * dy);
        const double angle_delta_deg = std::abs(sample.angle_deg - prev_sample_.angle_deg);
        const bool is_moving =
            angle_delta_deg > config_.angle_delta_threshold_deg ||
            pixel_delta_px > config_.pixel_delta_threshold_px;

        if (is_moving)
        {
            stable_count_ = 0;
            state_ = MotionGateState::MOVING;
        }
        else
        {
            ++stable_count_;
            if (stable_count_ >= config_.stop_confirm_frames)
            {
                state_ = MotionGateState::STOPPED;
            }
        }

        prev_sample_ = sample;

        result.should_send_real = state_ == MotionGateState::STOPPED;
        result.state = state_;
        result.stable_count = stable_count_;
        result.angle_delta_deg = angle_delta_deg;
        result.pixel_delta_px = pixel_delta_px;
        return result;
    }

private:
    MotionGateConfig config_;
    MotionSample prev_sample_;
    bool has_prev_sample_ = false;
    int stable_count_ = 0;
    MotionGateState state_ = MotionGateState::MOVING;
};
} // namespace rm_auto_aim_dart

#endif
