#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "gn10_navigation/moving_bucket_launcher_node.hpp"
#include "gn10_navigation/navigation_types.hpp"

class MovingBucketLauncherTest : public ::testing::Test {
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

TEST_F(MovingBucketLauncherTest, DynamicSpeedCalculationFromTF)
{
    rclcpp::NodeOptions options;
    options.append_parameter_override("base_frame", "base_link");
    options.append_parameter_override("bucket_frame", "bucket_bottom");
    options.append_parameter_override("use_tf", true);
    options.append_parameter_override("is_bottom_frame", true);
    options.append_parameter_override("bucket_height", 0.255);
    options.append_parameter_override("launcher_offset_x", 0.306);
    options.append_parameter_override("launcher_offset_z", 0.905);
    options.append_parameter_override("launch_angle_deg", 46.20);
    options.append_parameter_override("velocity_quad", 0.0);
    options.append_parameter_override("velocity_scale", 0.6806);
    options.append_parameter_override("velocity_offset", -0.0726);
    options.append_parameter_override("min_velocity", 2.50);
    options.append_parameter_override("max_velocity", 7.75);
    options.append_parameter_override("max_tracking_age_sec", 2.0);
    options.append_parameter_override("calculation_rate", 50.0);
    options.append_parameter_override("speed_filter_alpha", 1.0);  // unfiltered for exact check

    auto launcher_node = std::make_shared<gn10_navigation::MovingBucketLauncherNode>(options);
    auto helper_node   = std::make_shared<rclcpp::Node>("launcher_test_helper");

    // Dynamic TF broadcaster matching bucket_tracker_node
    tf2_ros::TransformBroadcaster tf_broadcaster(helper_node);
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.frame_id = "base_link";
    tf_msg.child_frame_id = "bucket_bottom";
    tf_msg.transform.translation.x = 2.0;  // 2m ahead
    tf_msg.transform.translation.y = 0.0;
    tf_msg.transform.translation.z = 0.5;  // bottom at z=0.5m
    tf_msg.transform.rotation.w = 1.0;

    // Publishers to control target_id and navigation_command
    auto target_id_pub = helper_node->create_publisher<std_msgs::msg::UInt8>("/robot/target_id", 10);
    auto nav_cmd_pub   = helper_node->create_publisher<std_msgs::msg::UInt8>("/robot/navigation_command", 10);

    // Subscriber to check belt_launcher_speed
    std::vector<float> received_speeds;
    auto speed_sub = helper_node->create_subscription<std_msgs::msg::Float32>(
        "/robot/operation/belt_launcher_speed", 10,
        [&received_speeds](std_msgs::msg::Float32::ConstSharedPtr msg) {
            received_speeds.push_back(msg->data);
        }
    );

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(launcher_node);
    exec.add_node(helper_node);

    // 1. Initially, no command sent -> Should NOT publish speed
    for (int i = 0; i < 5; ++i) {
        exec.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_TRUE(received_speeds.empty());

    // 2. Publish target_id != MovingBucket (e.g. 1) with Move (2) -> Should NOT publish speed
    std_msgs::msg::UInt8 tid_msg;
    tid_msg.data = 1;
    target_id_pub->publish(tid_msg);
    std_msgs::msg::UInt8 cmd_msg;
    cmd_msg.data = static_cast<uint8_t>(gn10_navigation::NavigationCommand::Move);
    nav_cmd_pub->publish(cmd_msg);

    for (int i = 0; i < 5; ++i) {
        exec.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_TRUE(received_speeds.empty());

    // 3. Publish target_id == MovingBucket (7) and Move (2) -> Should publish speed!
    tid_msg.data = static_cast<uint8_t>(gn10_navigation::TargetId::MovingBucket);
    target_id_pub->publish(tid_msg);

    for (int i = 0; i < 30 && received_speeds.empty(); ++i) {
        tf_msg.header.stamp = helper_node->now();
        tf_broadcaster.sendTransform(tf_msg);
        exec.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    ASSERT_FALSE(received_speeds.empty());
    const float speed_at_2m = received_speeds.back();
    EXPECT_GE(speed_at_2m, 2.50f);
    EXPECT_LE(speed_at_2m, 7.75f);

    // 4. Move target further to 3.5m -> speed should increase dynamically!
    tf_msg.transform.translation.x = 3.5;
    received_speeds.clear();

    for (int i = 0; i < 30; ++i) {
        tf_msg.header.stamp = helper_node->now();
        tf_broadcaster.sendTransform(tf_msg);
        exec.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    ASSERT_FALSE(received_speeds.empty());
    const float speed_at_3_5m = received_speeds.back();
    EXPECT_GT(speed_at_3_5m, speed_at_2m);
}
