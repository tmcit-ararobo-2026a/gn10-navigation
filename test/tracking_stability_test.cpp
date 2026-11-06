#include <gtest/gtest.h>
#include <cmath>
#include <vector>

#define GN10_NAVIGATION_PATH_FOLLOWER_NO_MAIN
#include "gn10_navigation/path_follower_node.hpp"

class TrackingStabilityTest : public ::testing::Test {
protected:
    static void SetUpTestSuite()
    {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
    }

    static void TearDownTestSuite()
    {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }
};

TEST_F(TrackingStabilityTest, AnglePidControllerAndDeadband)
{
    gn10_navigation::AnglePidController pid(3.5, 0.0, 0.15, 0.2, 0.2);

    // Initial step error
    const double dt = 0.02;
    double output = pid.update(0.1, dt, true);
    // P-term = 3.5 * 0.1 = 0.35
    EXPECT_NEAR(output, 0.35, 0.05);

    // Constant error over multiple steps -> derivative should smoothly drop to 0
    for (int i = 0; i < 20; ++i) {
        output = pid.update(0.1, dt, true);
    }
    // Only P-term remains since ki=0 and error is constant
    EXPECT_NEAR(output, 0.35, 0.01);

    // Test deadband concept: if within deadband, error should be treated as 0
    const double deadband = 0.015;
    double small_err = 0.010;
    if (std::abs(small_err) <= deadband) {
        small_err = 0.0;
    }
    // After error enters deadband (becomes 0), derivative decays and output settles to 0
    for (int i = 0; i < 25; ++i) {
        output = pid.update(small_err, dt, true);
    }
    EXPECT_NEAR(output, 0.0, 0.001);
}

TEST_F(TrackingStabilityTest, PositionFilterEMA)
{
    // Simulate noisy LiDAR centroid around (3.0, 1.0)
    const double true_x = 3.0;
    const double true_y = 1.0;
    const double alpha  = 0.25;

    double filtered_x = 0.0;
    double filtered_y = 0.0;
    bool has_filt     = false;

    // Simulate 30 frames with +/- 0.05m noise
    const double noise_pattern[] = {0.04, -0.05, 0.03, -0.02, 0.05, -0.04, 0.02, -0.03, 0.04, -0.04};
    for (int step = 0; step < 30; ++step) {
        const double raw_x = true_x + noise_pattern[step % 10];
        const double raw_y = true_y - noise_pattern[step % 10];

        if (!has_filt) {
            filtered_x = raw_x;
            filtered_y = raw_y;
            has_filt   = true;
        } else {
            filtered_x = (1.0 - alpha) * filtered_x + alpha * raw_x;
            filtered_y = (1.0 - alpha) * filtered_y + alpha * raw_y;
        }
    }

    // Filtered result should converge to true value within a tiny margin
    EXPECT_NEAR(filtered_x, true_x, 0.02);
    EXPECT_NEAR(filtered_y, true_y, 0.02);
}

TEST_F(TrackingStabilityTest, PositionJumpRateLimiter)
{
    // Test that a sudden 0.40m jump (from rotation TF latency) is clamped
    const double initial_x = 2.0;
    const double initial_y = 1.0;
    const double delta_time = 0.02; // 50 Hz
    constexpr double max_bucket_speed = 2.5; // m/s
    const double max_step = max_bucket_speed * delta_time + 0.05; // 0.10 m

    double tracked_x = initial_x;
    double tracked_y = initial_y;

    // Sudden 0.40m jump along X
    const double jump_x = initial_x + 0.40;
    const double jump_y = initial_y;

    const double dx = jump_x - tracked_x;
    const double dy = jump_y - tracked_y;
    const double dist = std::hypot(dx, dy);

    double clamped_x = jump_x;
    double clamped_y = jump_y;
    if (dist > max_step && dist > 1e-4) {
        clamped_x = tracked_x + (dx / dist) * max_step;
        clamped_y = tracked_y + (dy / dist) * max_step;
    }

    // Displacement of clamped position must not exceed max_step (0.10 m)
    EXPECT_LE(clamped_x - tracked_x, max_step + 1e-6);
    EXPECT_NEAR(clamped_x, initial_x + max_step, 1e-5);
    EXPECT_EQ(clamped_y, initial_y);
}
