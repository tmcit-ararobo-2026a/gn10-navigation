#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

class PathPlannerNode : public rclcpp::Node
{
public:
    PathPlannerNode();

private:
    // コストマップを受信するコールバック
    void getCostmapMsg(
        const nav_msgs::msg::OccupancyGrid::SharedPtr grid_msg
    );

    // map上のゴール座標系を受信(/goal_poseより)
    void getGoalPose(
        const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg
    );

    // map上のロボット座標系を受け取る関数(tfより)
    void getRobotPose();

    // コストマップのSubscriber
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
        costmap_sub_;

    // goalの座標系を受信するsubscriber
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
        goal_pose_sub_;

    // 2次元コストマップ grid_[y][x]
    std::vector<std::vector<int>> grid_;

    //TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // timer
    rclcpp::TimerBase::SharedPtr timer_;

    // マップの情報
    unsigned int width_ = 0;
    unsigned int height_ = 0;
    double resolution_ = 0.0;
    double origin_x_ = 0.0;
    double origin_y_ = 0.0;

    // ゴールの情報
    double goal_x_ = 0.0;
    double goal_y_ = 0.0;
};