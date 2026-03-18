#include <gtest/gtest.h>

#include "motion_gate.hpp"
#include "stability_gate.hpp"

namespace rm_auto_aim_dart
{
namespace
{
MotionSample sample(double x, double y, double angle_deg)
{
    MotionSample s;
    s.center_x = x;
    s.center_y = y;
    s.angle_deg = angle_deg;
    return s;
}
} // namespace

TEST(MotionGateTest, FirstDetectionKeepsDefaultPacket)
{
    MotionGate gate(MotionGateConfig{0.03, 8.0, 3});

    const auto result = gate.update(sample(100.0, 50.0, 1.0));

    EXPECT_FALSE(result.should_send_real);
    EXPECT_EQ(result.state, MotionGateState::MOVING);
    EXPECT_EQ(result.stable_count, 0);
}

TEST(MotionGateTest, AngleDeltaAboveThresholdMeansMoving)
{
    MotionGate gate(MotionGateConfig{0.03, 8.0, 3});
    gate.update(sample(100.0, 50.0, 1.0));

    const auto result = gate.update(sample(100.0, 50.0, 1.05));

    EXPECT_FALSE(result.should_send_real);
    EXPECT_EQ(result.state, MotionGateState::MOVING);
    EXPECT_EQ(result.stable_count, 0);
}

TEST(MotionGateTest, PixelDeltaAboveThresholdMeansMoving)
{
    MotionGate gate(MotionGateConfig{0.03, 8.0, 3});
    gate.update(sample(100.0, 50.0, 1.0));

    const auto result = gate.update(sample(109.0, 50.0, 1.0));

    EXPECT_FALSE(result.should_send_real);
    EXPECT_EQ(result.state, MotionGateState::MOVING);
    EXPECT_EQ(result.stable_count, 0);
}

TEST(MotionGateTest, SendsRealPacketAfterEnoughStableFrames)
{
    MotionGate gate(MotionGateConfig{0.03, 8.0, 3});
    gate.update(sample(100.0, 50.0, 1.0));

    EXPECT_FALSE(gate.update(sample(100.0, 50.0, 1.0)).should_send_real);
    EXPECT_FALSE(gate.update(sample(100.0, 50.0, 1.0)).should_send_real);

    const auto result = gate.update(sample(100.0, 50.0, 1.0));

    EXPECT_TRUE(result.should_send_real);
    EXPECT_EQ(result.state, MotionGateState::STOPPED);
    EXPECT_EQ(result.stable_count, 3);
}

TEST(MotionGateTest, MovementAfterStopReturnsToDefaultPacket)
{
    MotionGate gate(MotionGateConfig{0.03, 8.0, 3});
    gate.update(sample(100.0, 50.0, 1.0));
    gate.update(sample(100.0, 50.0, 1.0));
    gate.update(sample(100.0, 50.0, 1.0));
    ASSERT_TRUE(gate.update(sample(100.0, 50.0, 1.0)).should_send_real);

    const auto result = gate.update(sample(112.0, 50.0, 1.0));

    EXPECT_FALSE(result.should_send_real);
    EXPECT_EQ(result.state, MotionGateState::MOVING);
    EXPECT_EQ(result.stable_count, 0);
}

TEST(MotionGateTest, ResetMakesNextDetectionStartFromDefaultPacket)
{
    MotionGate gate(MotionGateConfig{0.03, 8.0, 3});
    gate.update(sample(100.0, 50.0, 1.0));
    gate.update(sample(100.0, 50.0, 1.0));
    gate.update(sample(100.0, 50.0, 1.0));
    ASSERT_TRUE(gate.update(sample(100.0, 50.0, 1.0)).should_send_real);

    gate.reset();
    const auto result = gate.update(sample(100.0, 50.0, 1.0));

    EXPECT_FALSE(result.should_send_real);
    EXPECT_EQ(result.state, MotionGateState::MOVING);
    EXPECT_EQ(result.stable_count, 0);
}

TEST(StabilityGateTest, RequiresConsecutiveFramesBeforeStable)
{
    StabilityGate gate(StabilityGateConfig{3.44, 4.0, 3});

    EXPECT_FALSE(gate.update(3.0));
    EXPECT_FALSE(gate.update(3.2));
    EXPECT_TRUE(gate.update(3.1));
}

TEST(StabilityGateTest, KeepsStableWithinReleaseThreshold)
{
    StabilityGate gate(StabilityGateConfig{3.44, 4.0, 3});

    gate.update(3.0);
    gate.update(3.2);
    ASSERT_TRUE(gate.update(3.1));

    EXPECT_TRUE(gate.update(3.8));
    EXPECT_TRUE(gate.update(3.9));
}

TEST(StabilityGateTest, DropsStableOnlyAfterCrossingReleaseThreshold)
{
    StabilityGate gate(StabilityGateConfig{3.44, 4.0, 3});

    gate.update(3.0);
    gate.update(3.2);
    ASSERT_TRUE(gate.update(3.1));

    EXPECT_FALSE(gate.update(4.1));
    EXPECT_FALSE(gate.update(3.5));
    EXPECT_FALSE(gate.update(3.3));
    EXPECT_FALSE(gate.update(3.2));
    EXPECT_TRUE(gate.update(3.1));
}
} // namespace rm_auto_aim_dart
